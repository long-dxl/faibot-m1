# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Trần gập cổ tay ở yaw 0 trên cẳng tay J4 upstream: gập -77,5 / 65 / 70 / 71 / 72 / 73 / 73,4 / 73,5 x cuộn -160..160 (9 nút) x kẹp 0/100 %.
Ghi giao mm3 vượt nền lắp sẵn (chốt J5 x đáy J56) -> ../export/AM_wrist_flex_scan.json. Kết quả 24/11/2026: 0 giao tới 73,4; ở 73,5
đáy J56 x cẳng tay J4 0,1 mm3 ở mọi cuộn và độ mở kẹp; phía âm -77,5 sạch. Chạy (venv build123d): python flex_scan.py (~4 phút)."""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripper_am as G

if __name__ == "__main__":
    t0 = time.time(); G._ys_init()
    out = {}
    RL = (-160, -120, -80, -40, 0, 40, 80, 120, 160)
    for g in (0.0, 100.0):
        for f in (-77.5, 65.0, 70.0, 71.0, 72.0, 73.0, 73.4, 73.5):
            for r in RL:
                h = G.yaw_pose_hits((f, 0.0, float(r), g))[1]
                out[f"g{g:g}_f{f:g}_r{r}"] = {k: round(v, 1) for k, v in h.items() if k != "OB_F_J56_BottomxOB_F_J5_Pin_Bearing"}
            print(g, f, {k: v for k, v in out.items() if k.startswith(f"g{g:g}_f{f:g}_") and v}, flush=True)
    json.dump(dict(ket_qua=out, t=round(time.time() - t0, 1)), open(os.path.join(HERE, "..", "export", "AM_wrist_flex_scan.json"), "w"), indent=1)
    print("xong", time.time() - t0)
