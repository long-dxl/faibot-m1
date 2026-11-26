# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Quét giới hạn khớp: vai -90..-70 x khuỷu 100..130 (bước 5 độ) và kiểm chạm đệm chặn. Gá khuỷu M1-AR-020 với gá vai v2, đệm chặn, link, chốt.
Chạy: python limit_scan.py -> ../export/limit_scan.json. Khe yêu cầu >= 0,3 mm (cùng khe thiết kế)."""
import json, os, sys
sys.path.insert(0, os.path.dirname(__file__))
import balance as b
from params import LINK_LEN
from build123d import Pos, Rot
cr, pad = b.cradle_v2(), b.stop_pad(); ea = b.elbow_assembly(); rows = []
for qs in range(-90, -69, 5):
    link, pin = b.link_v2(qs), b.pin(qs)
    for qe in range(100, 131, 5):
        loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
        mn, vol = 1e9, 0.0
        for en, eo in ea.items():
            e = loc * eo
            for n, o in (("cradle", cr), ("pad", pad), ("pin", pin)) + ((("link", link),) if en == "e_cradle" else ()):
                v, d = b.inter(e, o)
                vol += v or 0
                if isinstance(d, (int, float)): mn = min(mn, d)
        rows.append(dict(q_vai=qs, q_khuyu=qe, giao_mm3=round(vol, 1), khe_min_mm=round(mn, 2) if mn < 1e8 else None)); print(rows[-1], flush=True)
json.dump(rows, open(os.path.join(b.EXP, "limit_scan.json"), "w"), ensure_ascii=False, indent=1)
