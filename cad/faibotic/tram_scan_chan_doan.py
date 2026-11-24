# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Chẩn đoán quét tay trạm: với các cấu hình xấu nhất (cad/export/M1-PL-tram-scan-tay.json), đo khoảng cách từng khâu tay tới từng chi tiết
vật cản (trimesh, điểm mặt 2 mm) để biết chi tiết nào chạm.
Chạy (venv build123d, PYTHONPATH có sdk): python tram_scan_chan_doan.py"""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
import numpy as np
import trimesh
import pl_lift_scan as S
import assembly_pl as P


def main():
    R = json.load(open(os.path.join(P.EXP, "M1-PL-tram-scan-tay.json")))
    pts, rc = S.arm_point_sets(step=2.0)
    allp = P.build(0.0, "rest")
    out = {}
    for key, res in R["ket_qua"].items():
        side = key.split("_")[0]; lift_s = float(key.split("lift")[1]); lc = (lift_s - 0.160) * 1000
        for g, r in res.items():
            if not isinstance(r, dict) or r["khe_min_trong_luat_mm"] is None or r["khe_min_trong_luat_mm"] > 3: continue
            q = np.radians(r["q_xau_nhat_deg"]); pre, fr = S.GRID_GROUPS[g]
            rows = []
            for n, _, s in allp:
                if not n.startswith(pre): continue
                m = S.mesh(s)
                for link, p in pts.items():
                    W = S.arm_world({link: p}, rc, q, side, lc); Pq = W - (0, 0, lc) if fr == "tram" else W
                    lo, hi = m.bounds
                    near = Pq[np.all((Pq > lo - 30) & (Pq < hi + 30), axis=1)]
                    if not len(near): continue
                    _, d, _ = trimesh.proximity.closest_point(m, near[:: max(1, len(near) // 3000)])
                    rows.append((round(float(d.min()), 1), n, link))
            rows.sort()
            out[f"{key}/{g}"] = dict(q_deg=r["q_xau_nhat_deg"], gan_nhat=rows[:5])
            print(key, g, r["q_xau_nhat_deg"], rows[:3], flush=True)
    json.dump(out, open(os.path.join(P.EXP, "M1-PL-tram-scan-chan-doan.json"), "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
