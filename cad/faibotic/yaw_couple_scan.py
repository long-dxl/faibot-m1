# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""DK-N66: giới hạn ghép yaw cổ tay theo gập cổ tay. Quét lưới gripper_am.py --yaw-scan cho thấy đáy/vỏ J56 chạm cẳng tay J4 khi gập > 0
và yaw URDF âm (SDK dương), không phụ thuộc cuộn và độ mở kẹp; vỏ J56 chạm giá J5 ở yaw URDF +84,6 / -83,7 độ ở mọi gập.
Script này chia đôi (0,25 độ) góc yaw chạm đầu tiên ở từng nút gập (cuộn 0, kẹp khép), cả hai chiều.
Chạy (venv build123d): python yaw_couple_scan.py -> ../export/AM_wrist_yaw_couple.json"""
import json, os, sys, time
import multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import gripper_am as G

WF = (-77.5, -60.0, -40.0, -20.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 65.0, 70.0, 72.5)


def extra(wf, yw):
    b = G.yaw_pose_hits((wf, 0.0, 0.0, 0.0))[1]; h = G.yaw_pose_hits((wf, yw, 0.0, 0.0))[1]
    return {k: round(v - b.get(k, 0.0), 1) for k, v in h.items() if v != v or v - b.get(k, 0.0) > 0.5}


def first(args):
    wf, sgn = args
    lo, hi = 0.0, None
    for y in range(5, 91, 5):
        if extra(wf, sgn * y): hi = float(y); break
        lo = float(y)
    if hi is None: return wf, sgn, None, {}
    while hi - lo > 0.25:
        m = (lo + hi) / 2; lo, hi = (lo, m) if extra(wf, sgn * m) else (m, hi)
    return wf, sgn, round(sgn * hi, 2), extra(wf, sgn * hi)


if __name__ == "__main__":
    t0 = time.time()
    with mp.get_context("spawn").Pool(int(sys.argv[1]) if len(sys.argv) > 1 else 3, initializer=G._ys_init) as pool:
        R = pool.map(first, [(wf, s) for wf in WF for s in (1, -1)], chunksize=1)
    out = {}
    for wf, sgn, y, cap in R:
        out.setdefault(f"{wf:g}", {})["duong_urdf" if sgn > 0 else "am_urdf"] = dict(cham_deg=y, cap=cap)
    r = dict(ghi_chu="DK-N66: yaw URDF chạm đầu tiên (chia đôi 0,25 độ, vượt nền yaw 0 > 0,5 mm3) theo gập cổ tay (độ URDF = SDK), cuộn 0, kẹp khép. "
                     "Yaw SDK = -URDF. Nút gập >= 73,4 đã chạm cẳng tay J4 ở yaw 0 (DK-N68).", gap_deg=list(WF), cham=out, thoi_gian_s=round(time.time() - t0, 1))
    json.dump(r, open(os.path.join(HERE, "..", "export", "AM_wrist_yaw_couple.json"), "w"), ensure_ascii=False, indent=1)
    print(json.dumps(r, ensure_ascii=False))
