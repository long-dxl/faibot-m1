# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Đăng ký chi tiết AlohaMini2 (STL bố trí in) vào mesh URDF base_link / vertical_link (mm, vertical_move 0) bằng ICP.
Cần bản clone đủ upstream ở ~/m1-g2a-ref (hardware/mobile_base2/stl, urdf/alohamini2/alohamini2_meshes): không lưu trong repo vì nặng (~100 MB).
Giai đoạn 1 (mặc định): chi tiết cùng hướng bố trí (Rz -90) chạy ICP nhiều điểm xuất phát; chi tiết khác hướng quét 24 phép quay.
Giai đoạn 2 (--stage2): T-connector trái/phải đăng ký lại trên vùng cắt |x_U| > 60 của vertical_link (giai đoạn 1 đặt chúng trượt vào giữa).
Kết quả ~/m1-g2a-ref/reg.json, reg5.json -> gộp vào cad/export/AlohaMini2_registration.json, rồi chạy am2_place.py."""
import sys
STAGE2 = 'import trimesh, numpy as np, os, json, itertools\nfrom scipy.spatial import cKDTree\nH = os.path.expanduser("~/m1-g2a-ref/")\nU = H + "urdf/alohamini2/alohamini2_meshes/"; S = H + "hardware/mobile_base2/stl/"\nvl = trimesh.load(U + "vertical_link.STL"); vl.apply_scale(1000); vl.apply_translation([0, 0, 96.196])\nTVall = vl.sample(300000)\n\n\ndef rots():\n    out = []\n    for perm in itertools.permutations(range(3)):\n        for sg in itertools.product((1, -1), repeat=3):\n            R = np.zeros((3, 3))\n            for i, (p, s) in enumerate(zip(perm, sg)): R[i, p] = s\n            if np.linalg.det(R) > 0: out.append(R)\n    return out\n\n\ndef fit(name, sel):\n    tv = TVall[sel(TVall)]; tree = cKDTree(tv)\n    pts = trimesh.load(S + name + ".stl").sample(1500); best = None\n    for R in rots():\n        p = (R @ pts.T).T; mid = (p.min(0) + p.max(0)) / 2; tmid = (tv.min(0) + tv.max(0)) / 2\n        for dy in (0,):\n            for dx in (-25, 25):\n                for dz in (0,):\n                    t = tmid - mid + np.array([dx, dy, dz])\n                    for _ in range(15):\n                        q = p + t; d, i = tree.query(q); k = d < np.percentile(d, 85); t = t + np.mean(tv[i[k]] - q[k], axis=0)\n                    # điểm chi tiết -> mặt đích và ngược lại (phủ hai chiều)\n                    d1, _ = tree.query(p + t); d2, _ = cKDTree(p + t).query(tv[::40])\n                    s = d1.mean() + d2.mean()\n                    if best is None or s < best[0]: best = (s, R, t, d1.mean(), d2.mean())\n    s, R, t, a, b = best\n    return dict(R=R.tolist(), t=t.tolist(), mean=float(a), mean_nguoc=float(b))\n\n\nres = {}\nfor n, sel in (("OB_T_Connector_Left", lambda v: v[:, 0] > 60), ("OB_T_Connector_Right", lambda v: v[:, 0] < -60)):\n    r = fit(n, sel); res[n] = r; print(n, r, flush=True)\njson.dump(res, open(H + "reg5.json", "w"), indent=1)\n'
if "--stage2" in sys.argv:
    exec(compile(STAGE2, "stage2", "exec")); raise SystemExit
import trimesh, numpy as np, os, json, itertools
from scipy.spatial import cKDTree
H = os.path.expanduser("~/m1-g2a-ref/")
U = H + "urdf/alohamini2/alohamini2_meshes/"; S = H + "hardware/mobile_base2/stl/"
bl = trimesh.load(U + "base_link.STL"); vl = trimesh.load(U + "vertical_link.STL")
bl.apply_scale(1000); vl.apply_scale(1000); vl.apply_translation([0, 0, 96.196])
TV = {"base": bl.sample(300000), "vert": vl.sample(150000)}
TREE = {k: cKDTree(v) for k, v in TV.items()}
res = json.load(open(H + "reg.json"))
RM = np.array(res["OB_T_Connector_Middle"]["R"]); tM = np.array(res["OB_T_Connector_Middle"]["t"])


def icp(pts, R, t, tg, n=40):
    tree, tv = TREE[tg], TV[tg]; p = (R @ pts.T).T
    for _ in range(n):
        q = p + t; d, i = tree.query(q); k = d < np.percentile(d, 85); t = t + np.mean(tv[i[k]] - q[k], axis=0)
    d, _ = tree.query(p + t)
    return t, float(d.mean()), float(np.percentile(d, 90))


def rots():
    out = []
    for perm in itertools.permutations(range(3)):
        for sg in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, (p, s) in enumerate(zip(perm, sg)): R[i, p] = s
            if np.linalg.det(R) > 0: out.append(R)
    return out


for n, tg in (("OB_Main_Assembly_Post1", "base"), ("OB_Main_Assembly_Post2", "base"), ("OB_Main_Assembly_Post3", "base"), ("OB_Z_Axis_Servo_Gear", "vert"),
              ("O_T_Connector_Dowel_Pin_12_25", "vert"), ("OB_T_Camera_Mount", "vert"), ("O_POST4_Connector_Base", "base"), ("OB_POST4_Mount_Adapter", "base"),
              ("OB_Top_Camera_Mount", "base")):
    pts = trimesh.load(S + n + ".stl").sample(3000)
    best = None
    for dz in (0, -200, 200, -400, 400, -100, 100):          # trụ lệch tầng so với khung T
        for dx in (0, -100, 100):
            t0 = tM + np.array([dx, 0, dz])
            t, mean, p90 = icp(pts, RM, t0, tg, 25)
            if best is None or mean < best[1]: best = (t, mean, p90)
    t, mean, p90 = icp(pts, RM, best[0], tg, 40)
    res[n] = dict(R=RM.tolist(), t=t.tolist(), mean=mean, p90=p90, target=tg)
    print(n, tg, round(mean, 2), round(p90, 2), np.round(t, 1), flush=True)
    json.dump(res, open(H + "reg.json", "w"), indent=1)
# chi tiết bố trí khác hướng: tìm hướng trong 24, tịnh tiến khởi đầu theo tâm vùng đích (lưới thô 20 mm trong vùng)
for n, tg, box in (("OB_T_Connector_Left", "vert", ((60, -90, 420), (240, 0, 580))), ("OB_T_Connector_Right", "vert", ((-240, -90, 420), (-60, 0, 580))),
                   ("O_T_Connector_Cross_Bar", "vert", ((-130, -90, 420), (130, 0, 580))), ("O_Main_Assembly_Post4", "base", ((-120, -40, 100), (120, 240, 360)))):
    pts = trimesh.load(S + n + ".stl").sample(2500); best = None
    lo, hi = np.array(box[0], float), np.array(box[1], float)
    for R in rots():
        p = (R @ pts.T).T; mid = (p.min(0) + p.max(0)) / 2
        G = np.array(np.meshgrid(*[np.arange(lo[i], hi[i] + 1, 20.0) for i in range(3)], indexing="ij")).reshape(3, -1).T
        sub = p[::25] - mid
        for c in G:
            d, _ = TREE[tg].query(sub + c, distance_upper_bound=15); s = np.mean(np.minimum(d, 15))
            if best is None or s < best[0]: best = (s, R, c - mid)
    t, mean, p90 = icp(pts, best[1], best[2], tg, 40)
    res[n] = dict(R=best[1].tolist(), t=t.tolist(), mean=mean, p90=p90, target=tg)
    print(n, tg, round(mean, 2), round(p90, 2), np.round(t, 1), best[1].astype(int).tolist(), flush=True)
    json.dump(res, open(H + "reg.json", "w"), indent=1)
