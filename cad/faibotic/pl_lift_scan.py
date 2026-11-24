# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""P-L: hành trình thật của xe nâng trên trụ (voxel, nhanh), và (--tay) quét va chạm hai tay bằng mesh.
Trụ và đế trụ (tĩnh) voxel hoá 2 mm; xe nâng S2 (khung T giữa, chốt định vị, đế ngàm, trạm hai tay chưa tay) lấy 40.000 điểm mặt mỗi chi tiết,
dời theo z, đếm điểm rơi vào trụ.
Bánh răng tính riêng: ăn răng là chủ ý; chỉ lấy giới hạn theo dải răng (tâm bánh cách đầu dải răng >= bán kính chia 19,5) và theo chốt hãm.
Lift 0 = vertical_move 0 của URDF AlohaMini2. Ghi ../export/M1-PL-lift-scan.json và lift_range_mm vào AlohaMini2_placement.json."""
import json, os, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
import trimesh
from build123d import export_stl
import assembly_pl as P

PITCH = 2.0


def mesh(s):
    f = tempfile.mktemp(suffix=".stl"); export_stl(s, f, tolerance=0.1, angular_tolerance=0.2); m = trimesh.load(f); os.remove(f); return m


def main():
    stat = [mesh(s) for n, _, s in P.column()]
    lo3 = np.min([m.bounds[0] for m in stat], 0) - 5; hi3 = np.max([m.bounds[1] for m in stat], 0) + 5
    shape = np.ceil((hi3 - lo3) / PITCH).astype(int) + 1; occ = np.zeros(shape, bool)
    for m in stat:       # mỗi chi tiết voxel riêng (tiết kiệm bộ nhớ), gộp vào lưới chung
        v = m.voxelized(PITCH, method="ray") if False else m.voxelized(PITCH)
        v = v.fill(); p = v.points; ijk = np.round((p - lo3) / PITCH).astype(int); occ[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = True
        print("vox", len(p), flush=True)
    T0 = np.eye(4); T0[:3, :3] *= PITCH; T0[:3, 3] = lo3
    inv = np.linalg.inv(T0)
    mov = {n: mesh(s) for n, _, s in P.T_frame(0.0) + P.dock(0.0) + P.station(0.0) if "gear" not in n.lower()}
    pts = {n: m.sample(40000) for n, m in mov.items()}      # điểm mặt: đủ phát hiện xuyên > 1 ô lưới
    res = {}
    for lift in range(-300, 401, 5):
        hits = {}
        for n, p in pts.items():
            q = p + (0, 0, lift); ijk = np.round((inv[:3, :3] @ q.T).T + inv[:3, 3]).astype(int)
            ok = np.all((ijk >= 0) & (ijk < np.array(occ.shape)), axis=1)
            k = ijk[ok]; c = int(occ[k[:, 0], k[:, 1], k[:, 2]].sum())
            if c > 20: hits[n] = c      # số điểm mặt nằm trong trụ (ngưỡng 20 điểm để bỏ chạm mặt / sai số lưới)
        res[lift] = hits
    free = [l for l, h in res.items() if not h]
    lo = hi = 0
    if 0 in free:
        while lo - 5 in free: lo -= 5
        while hi + 5 in free: hi += 5
    gz = P.PLACE["gear_center_U"][2]; rz0, rz1 = P.PLACE["rack_z_W"]; pr = P.GEAR_R      # bánh 20 răng m1,5, vòng chia r 15 (trước 19,5)
    lo_rack, hi_rack = rz0 + pr - gz, rz1 - pr - gz
    pawl_top = P.PLACE["Tmid_top_zW"] + 14.0 + 3.0
    hi_latch = rz1 - 3.0 - pawl_top
    out = dict(phuong_phap=__doc__.strip().splitlines()[1], diem_cham_o_lift_0=res[0], khong_giao_lien_tuc_mm=[lo, hi], gioi_han_rang_mm=[round(lo_rack, 1), round(hi_rack, 1)],
               gioi_han_chot_mm=round(hi_latch, 1), lift_range_mm=[max(lo, lo_rack), min(hi, hi_rack, hi_latch)], quet={str(k): v for k, v in res.items() if k % 25 == 0})
    json.dump(out, open(os.path.join(P.EXP, "M1-PL-lift-scan.json"), "w"), indent=1, ensure_ascii=False)
    # S2: không tự ghi đè lift_range_mm đã chọn trong AlohaMini2_placement.json (giữ -160..290 = LIFT_RANGE SDK); số quét chỉ ở M1-PL-lift-scan.json
    print(json.dumps({k: v for k, v in out.items() if k != "quet"}, ensure_ascii=False))
    for l in range(-300, 401, 25): print(l, res[l])


# ---------------- S2: quét va chạm hai tay (mesh) với cột nâng, cột camera, đầu camera, bệ trạm, xe nâng, đế ----------------
# Vật cản -> trường khoảng cách có dấu (EDT trên lưới voxel, mm). Tay: điểm mặt mesh từng chi tiết (bước ~5 mm), đặt theo FK URDF AM-ARM200
# (amarm.fk) với góc SDK đổi sang URDF (spec.URDF_TO_SDK), kẹp (kẹp gốc AM-ARM200, theo link left_Fixed_Jaw và left_Moving_Jaw) cuộn theo link con của khớp cuộn. Lưới góc SDK trong giới hạn mềm
# (pan ±70, vai -80..48,5, khuỷu tới elbow_max(vai), cổ tay tới wrist_max(vai, khuỷu)). Khe = khoảng cách nhỏ nhất điểm tay -> mặt vật cản
# (sai số lưới + bước điểm ~±3 mm); 20 cấu hình xấu nhất mỗi nhóm đo lại chính xác bằng trimesh (điểm mịn 1,5 mm, mặt thật).
GRID_GROUPS = {   # nhóm -> (tiền tố tên chi tiết, khung: "tram" theo tay, "the_gioi" theo đế)
    "cot_camera": (("M1-TR-006", "M1-TR-005"), "tram"),
    "dau_camera": (("M1-TR-007", "M1-CM-001_head"), "tram"),
    "be_tram": (("M1-TR-001", "M1-TR-002", "M1-TR-003", "M1-TR-004", "M1-TR-008", "M1-TR-009", "M1-TR-010", "M1-TR-011", "M1-TR-012", "M1-TR-013", "M1-TR-014"), "tram"),
    "xe_nang": (("M1-LF-057", "M1-LF-004", "M1-LF-002", "M1-LF-050", "M1-LF-041", "M1-LF-06", "M1-LF-lift", "M1-TR-02"), "tram"),
    "cot_nang": (("M1-LF-051", "M1-LF-052", "M1-LF-053", "M1-LF-054", "M1-LF-055"), "the_gioi"),
    "de": (("M1-BS-", "M1-ST-", "M1-SV-001_wheel"), "the_gioi"),
}


def sdf(meshes, pitch, lo, hi):
    from scipy import ndimage
    shape = np.ceil((hi - lo) / pitch).astype(int) + 1; occ = np.zeros(shape, bool)
    for m in meshes:
        try:
            v = m.voxelized(pitch).fill()
        except Exception:
            v = m.voxelized(pitch)
        ijk = np.round((v.points - lo) / pitch).astype(int)
        ok = np.all((ijk >= 0) & (ijk < shape), axis=1); ijk = ijk[ok]; occ[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = True
    d_out = ndimage.distance_transform_edt(~occ) * pitch; d_in = ndimage.distance_transform_edt(occ) * pitch
    return (d_out - d_in - pitch / 2).astype(np.float32)


def lookup(F, lo, pitch, P):
    ijk = np.round((P - lo) / pitch).astype(int); sh = np.array(F.shape)
    ok = np.all((ijk >= 0) & (ijk < sh), axis=1); out = np.full(len(P), 1e3, np.float32)
    out[ok] = F[ijk[ok, 0], ijk[ok, 1], ijk[ok, 2]]; return out


def arm_point_sets(step=5.0, fine=False):
    """{link: điểm mặt (mm, khung left_Base hệ STEP ở tư thế 0)} của tay trái; J1 (cố định trên gá) bỏ. Kẹp gắn link con của khớp cuộn."""
    import amarm as AM
    roll_child = [j["child"] for j in AM.urdf()["J"] if j["name"] == "left_wrist_roll"][0]
    pts = {}
    for f, (link, s) in P.arm_parts_local().items():
        if f == "O_F_J1_Base": continue
        m = mesh(s); n = max(200, int(m.area / (step * step))); pts.setdefault(link, []).append(m.sample(n))
    for n_, _, s in P.grip_step():       # khoá yaw theo link wrist_yaw; kẹp cũ (M1_GRIPPER khác "am") theo link con khớp cuộn
        m = mesh(s); n = max(100, int(m.area / (step * step)))
        pts.setdefault("left_wrist_yaw" if n_.startswith("M1-AR-025") else roll_child, []).append(m.sample(n) - P._t_step())
    for n_, _, link, s in P.grip_am_local():   # kẹp AM-ARM200 (khung left_Base sẵn), hàm theo link left_Moving_Jaw (góc kẹp URDF của q)
        m = mesh(s); n = max(100, int(m.area / (step * step))); pts.setdefault(link, []).append(m.sample(n))
    return {k: np.concatenate(v) for k, v in pts.items()}, roll_child


def sdk_to_urdf(q):
    """q SDK (pan, vai, khuỷu, gập, cuộn, rad) -> dict URDF amarm."""
    from faibotic import spec
    out = {}
    for k, j in zip(("pan", "lift", "elbow", "wflex", "roll"), ("pan", "shoulder", "elbow", "wrist_flex", "wrist_roll")):
        sg, off = spec.URDF_TO_SDK[j]; out[k] = (q[j] - off) / sg
    return out


def sdk_grid(pan_step=10, sh_vals=None, el_step=9, wr_step=20, rolls=(0.0, 90.0)):
    from faibotic import spec
    import math
    sh_vals = sh_vals or (-80, -64, -48, -32, -16, 0, 8, 16, 24, 32, 40, 48.5)
    out = []
    for sh in sh_vals:
        emax = math.degrees(spec.elbow_max(math.radians(sh)))
        for el in list(np.arange(-10, emax, el_step)) + [emax]:
            wmax = math.degrees(spec.wrist_max(math.radians(sh), math.radians(el)))
            for wr in list(np.arange(-77.5, wmax, wr_step)) + [wmax]:
                for pan in np.arange(-70, 70.1, pan_step):
                    for ro in rolls:
                        out.append((pan, sh, el, wr, ro))
    return np.radians(np.array(out))


def arm_world(pts, roll_child, q_sdk, side, lift):
    """Điểm tay trong W (mm) ở cấu hình SDK q (mảng 5) cho tay side, xe nâng ở lift CAD (mm)."""
    import amarm as AM
    qd = sdk_to_urdf(dict(zip(("pan", "shoulder", "elbow", "wrist_flex", "wrist_roll"), q_sdk)))
    T0 = AM.fk({}); Tq = AM.fk(qd)
    B = P.M4(P.RW) @ P.M4(np.eye(3), np.array(P.ARM_BASE_U["L" if side == "left" else "R"]) + (0, 0, lift))
    out = []
    for link, p in pts.items():
        D = Tq[link] @ np.linalg.inv(T0[link]); D[:3, 3] *= 1000; M = B @ D
        out.append(p @ M[:3, :3].T + M[:3, 3])
    return np.concatenate(out)


def scan_tay(out_json, lifts_sdk=(0.0, 0.45), keepout=True):
    import time, math
    from faibotic import spec
    t0 = time.time()
    L0 = spec.LIFT_CAD_OFFSET * 1000
    pts, rc = arm_point_sets()
    print("diem tay", sum(len(v) for v in pts.values()), flush=True)
    # vật cản ở lift CAD 0 (khung trạm theo xe nâng); nhóm "the_gioi" đứng yên, đổi lift bằng dời điểm tay
    allp = P.build(0.0, "rest")
    O = P.station_origin(0.0)
    fields = {}
    for g, (pre, fr) in GRID_GROUPS.items():
        ms = [mesh(s) for n, _, s in allp if n.startswith(pre)]
        if fr == "the_gioi":
            lo = np.array([-320.0, -420.0, 0.0]); hi = np.array([700.0, 420.0, 1000.0]); pitch = 4.0
        else:
            lo = O + np.array([-260.0, -420.0, -200.0]); hi = O + np.array([760.0, 420.0, 760.0]); pitch = 3.0
        fields[g] = (sdf(ms, pitch, lo, hi), lo, pitch, ms)
        print("sdf", g, len(ms), round(time.time() - t0), "s", flush=True)
    Q = sdk_grid(); print("cau hinh moi tay", len(Q), flush=True)
    cam_c = O + np.array(__import__("tram_hai_tay").cam_axis_point())       # tâm cầu cấm đầu camera (lift 0)
    res = {}
    for side in ("left", "right"):
        for lift_s in lifts_sdk:
            lc = (lift_s + spec.LIFT_CAD_OFFSET) * 1000
            mins = {g: np.full(len(Q), 1e3, np.float32) for g in fields}
            ko = np.zeros(len(Q), bool)
            for i, q in enumerate(Q):
                W = arm_world(pts, rc, q, side, lc)
                for g, (F, lo, pitch, _) in fields.items():
                    Pq = W - (0, 0, lc) if GRID_GROUPS[g][1] == "tram" else W
                    mins[g][i] = lookup(F, lo, pitch, Pq).min()
                if keepout:     # luật mềm SDK (spec.KEEPOUT_*): khuỷu, cổ tay, TCP theo FK SDK
                    ko[i] = bool(spec_keepout_hit(q, side, lift_s))
                if i % 5000 == 0: print(side, lift_s, i, round(time.time() - t0), "s", flush=True)
            key = f"{side}_lift{lift_s:g}"
            res[key] = {}
            for g in fields:
                m = mins[g]; allowed = ~ko
                res[key][g] = dict(khe_min_mm=round(float(m.min()), 1), so_cau_hinh_cham=int((m <= 0).sum()),
                                   khe_min_trong_luat_mm=round(float(m[allowed].min()), 1) if allowed.any() else None,
                                   cham_trong_luat=int(((m <= 0) & allowed).sum()),
                                   q_xau_nhat_deg=[round(float(v), 1) for v in np.degrees(Q[int(np.argmin(np.where(allowed, m, 1e3)))])])
            res[key]["bi_luat_cam_loai"] = int(ko.sum())
            print(key, json.dumps(res[key], ensure_ascii=False), flush=True)
            # đo lại chính xác cấu hình xấu nhất từng nhóm (trong luật)
            for g, (F, lo, pitch, ms) in fields.items():
                m = np.where(~ko, mins[g], 1e3); worst = np.argsort(m)[:20]
                import trimesh
                comb = trimesh.util.concatenate(ms)
                best = 1e3
                for i in worst:
                    W = arm_world(pts, rc, Q[i], side, lc); Pq = W - (0, 0, lc) if GRID_GROUPS[g][1] == "tram" else W
                    near = Pq[np.argsort(lookup(F, lo, pitch, Pq))[:400]]
                    _, d, _ = trimesh.proximity.closest_point(comb, near)
                    inside = comb.contains(near) if comb.is_watertight else np.zeros(len(near), bool)
                    dd = np.where(inside, -d, d).min(); best = min(best, float(dd))
                res[key][g]["khe_chinh_xac_mm"] = round(best, 1)
            print(key, "chinh xac", {g: res[key][g]["khe_chinh_xac_mm"] for g in fields}, round(time.time() - t0), "s", flush=True)
    out = dict(phuong_phap="pl_lift_scan.scan_tay: SDF voxel (trạm 3 mm, thân 4 mm) + điểm mặt tay 5 mm; 20 cấu hình xấu nhất mỗi nhóm đo lại bằng trimesh",
               luoi=dict(so_cau_hinh_moi_tay=len(Q), pan_buoc=10, vai=[-80, -64, -48, -32, -16, 0, 8, 16, 24, 32, 40, 48.5], khuyu_buoc=9, co_tay_buoc=20, cuon=[0, 90]),
               shoulder_y_m=P.ARM_BASE_U["L"][0] / 1000, ket_qua=res, thoi_gian_s=round(time.time() - t0))
    json.dump(out, open(out_json, "w"), indent=1, ensure_ascii=False)


def spec_keepout_hit(q, side, lift_s=0.0):
    from faibotic import limits
    return limits.keepout_violations(side, q, "di_dong", lift_s)


if __name__ == "__main__":
    import sys
    if "--tay" in sys.argv:
        sys.path.insert(0, os.path.join(HERE, "../../sdk"))
        scan_tay(os.path.join(P.EXP, "M1-PL-tram-scan-tay.json"))
    else:
        main()
