# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Tách mesh link URDF upstream AM-ARM200 follower (liyiteng/AM-ARM commit 5b03af1, Apache-2.0, am-arm200/urdf/am-arm200-meshes/*.STL;
mesh SolidWorks có sẵn thân servo lắp đúng chỗ, tai bắt, ổ bi, bu lông) thành: thân link, chi tiết kim loại (bu lông 42,5 x Ø4,2, ổ bi Ø30),
và từng servo (STS3095 65 x 30, STS3215 45,2 x 24,7: các tấm thân + trục/còi + giắc nằm trong hộp bao servo). Bỏ chi tiết < MIN_MM (ren, đầu vít).
Rút gọn từng nhóm, ghi STL trong KHUNG LINK URDF (m) + derived.json. Bản quyền Faibotic (mã).
Chạy (trimesh, fast-simplification): python extract_urdf_servos.py DIR_MESH_URDF OUT_DIR
Dùng ở web-assets/src/10_build_m1_glb.py (đặt theo FK URDF như export_sim_meshes.pl_urdf_to_fore)."""
import sys, os, json, numpy as np, trimesh, fast_simplification
SRC, OUT = sys.argv[1], sys.argv[2]
MIN_MM = 6.0
# link -> (tam giác thân, [(khoá servo SDK tay trái, loại, tam giác)]) ; servo nằm ở link cha của khớp nó quay
LINKS = {"left_Base": (10**6, [("left_pan", "STS3095", 1200)]), "left_Rotation_Pitch": (10**6, [("left_shoulder", "STS3095", 1200)]),
         "left_Upper_Arm": (4000, [("left_elbow", "STS3095", 1200)]), "left_Lower_Arm": (3500, [("left_wrist_flex", "STS3215", 1300)])}   # v0.5e: rút thêm để cổ tay/kẹp vừa brotli 1,2 MB   # v0.5d: thân link, kim loại giữ nguyên độ phân giải; servo rút còn ~2.600/2.000
# v0.5e: cổ tay + kẹp. wrist_yaw có 3 chồng STS3215 (tâm chồng, mm, khung link): y < 20 = servo yaw, 20..65 = cuộn, > 65 = kẹp
# (6+1, 24/11/2026): servo yaw GIỮ (trước M1 khoá yaw nên bỏ); vẫn ghi hộp bao "bo_servo_yaw" cho sim/tools/am_arm200_pl.py
# (servo kẹp nằm trong mesh wrist_yaw nhưng quay theo khớp cuộn: 10_build gắn nó vào node wrist_roll). Chọn chồng bằng hàm trên tâm chồng.
LINKS.update({"left_Wrist_Pitch_Roll": (1500, []), "left_Fixed_Jaw": (2000, []), "left_Moving_Jaw": (10**6, []), "left_camera": (700, []),
              "left_wrist_yaw": (2500, [("left_wrist_roll", "STS3215", 1300, lambda c: 20 <= c[1] <= 65), ("left_gripper", "STS3215", 1300, lambda c: c[1] > 65),
                                       ("left_wrist_yaw", "STS3215", 1300, lambda c: c[1] < 20)])})
SLAB = {"STS3095": (65.0, 30.0), "STS3215": (45.2, 24.7)}
def dec(m, n):
    if len(m.faces) <= n: return m
    v, f = fast_simplification.simplify(np.asarray(m.vertices), np.asarray(m.faces), target_reduction=1 - n / len(m.faces), agg=7)
    t = trimesh.Trimesh(v, f, process=True)
    for cell in (0.5e-3, 1e-3, 1.5e-3, 2e-3, 3e-3):      # mesh có ren/lỗ nhỏ, quadric dừng sớm: gộp đỉnh theo lưới rồi rút tiếp
        if len(t.faces) <= 1.3 * n or (n >= 1000 and cell > 1e-3): break      # v0.5d: servo chi tiết không gộp lưới (gộp lưới ra khối thô)
        g = trimesh.Trimesh(np.round(np.asarray(m.vertices) / cell) * cell, m.faces, process=True); g.update_faces(g.nondegenerate_faces()); g.remove_unreferenced_vertices()
        v, f = fast_simplification.simplify(np.asarray(g.vertices), np.asarray(g.faces), target_reduction=max(0.0, 1 - n / max(len(g.faces), 1)), agg=7)
        t = trimesh.Trimesh(v, f, process=True)
    return t
os.makedirs(OUT, exist_ok=True); rep = {}
for link, (ntri, servos) in LINKS.items():
    m = trimesh.load(os.path.join(SRC, link + ".STL"), process=True); cs = m.split(only_watertight=False)
    used = set(); r = dict(servo={}, bo=0)
    for sv in servos:
        key, model, st = sv[:3]; sel = sv[3] if len(sv) > 3 else None
        idx = [i for i, c in enumerate(cs) if np.allclose(np.sort(c.extents)[::-1][:2] * 1000, SLAB[model], atol=0.6)]
        if sel is not None:      # nhiều chồng servo: gom tấm theo tâm (cách tâm tấm đầu chồng < 30 mm), chọn chồng
            st_ = []
            for i in idx:
                c = cs[i].bounds.mean(0) * 1000
                for g in st_:
                    if np.linalg.norm(c - g[0]) < 30: g[1].append(i); break
                else: st_.append([c, [i]])
            pick = [g for g in st_ if sel(np.mean([cs[i].bounds.mean(0) * 1000 for i in g[1]], 0))]
            assert len(pick) == 1, (link, key, len(pick), [g[0].round(1) for g in st_]); idx = pick[0][1]
        assert len(idx) >= 3, (link, model, len(idx))
        mg = 0.03 if sel is None else 0.008
        lo = np.min([cs[i].bounds[0] for i in idx], 0); hi = np.max([cs[i].bounds[1] for i in idx], 0)
        horn = [i for i, c in enumerate(cs) if i not in idx and i not in used and (c.bounds[0] >= lo - mg).all() and (c.bounds[1] <= hi + mg).all() and np.prod(c.extents) > 2e-6]
        lo2 = np.min([cs[i].bounds[0] for i in idx + horn], 0) - 1e-3; hi2 = np.max([cs[i].bounds[1] for i in idx + horn], 0) + 1e-3
        keep = [i for i, c in enumerate(cs) if i not in used and (c.bounds[0] >= lo2).all() and (c.bounds[1] <= hi2).all() and (np.max(c.extents) * 1000 >= MIN_MM or key is None)]
        used |= set(keep)
        if key is None: r["bo_servo_yaw"] = dict(thanh_phan=len(keep), hop_bao_m=[lo2.round(5).tolist(), hi2.round(5).tolist()]); continue
        if key == "left_wrist_yaw": r["bo_servo_yaw"] = dict(thanh_phan=len(keep), hop_bao_m=[lo2.round(5).tolist(), hi2.round(5).tolist()])
        s = dec(trimesh.util.concatenate([cs[i] for i in keep]), st); s.export(os.path.join(OUT, f"{link}__servo__{key}.stl"))
        r["servo"][key] = dict(model=model, thanh_phan=len(keep), tam_giac=int(len(s.faces)), hop_bao_m=[lo2.round(5).tolist(), hi2.round(5).tolist()])
    body, metal = [], []
    for i, c in enumerate(cs):
        if i in used: continue
        e = np.sort(c.extents)[::-1] * 1000
        if e[0] < MIN_MM: r["bo"] += 1; continue
        (metal if (np.allclose(e, (42.5, 4.2, 4.2), atol=0.6) or (abs(e[0] - e[1]) < 0.3 and 29 < e[0] < 31 and e[2] < 10)) else body).append(c)
    bb_ = trimesh.util.concatenate(body); b = dec(bb_, min(ntri, int(0.7 * len(bb_.faces)))); b.export(os.path.join(OUT, f"{link}__body.stl"))      # v0.5d: giữ 70% tam giác thân link
    r["than"] = dict(thanh_phan=len(body), tam_giac=int(len(b.faces)))
    if metal:
        k = dec(trimesh.util.concatenate(metal), 150 * len(metal)); k.export(os.path.join(OUT, f"{link}__metal.stl")); r["kim_loai"] = dict(thanh_phan=len(metal), tam_giac=int(len(k.faces)))
    rep[link] = r; print(link, r)
json.dump(dict(nguon="liyiteng/AM-ARM 5b03af1 am-arm200/urdf/am-arm200-meshes (Apache-2.0); đã tách nhóm, bỏ chi tiết < 6 mm, rút gọn", link=rep),
          open(os.path.join(OUT, "derived.json"), "w"), ensure_ascii=False, indent=1)
