# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Khối lượng in ước tính (PETG 1,27 g/cm³, 4 thành 0,4 mm, infill 15%: kịch bản "thấp" của arm_mass.py) của chi tiết tay AM-ARM200 follower,
so với khối lượng URDF upstream (khối đặc 1,0 g/cm³). Cùng công thức cad/export/arm_mass.json: vỏ = min(diện tích x số thành x 0,4 mm, V),
nội thất = infill x phần còn lại. Chạy (venv build123d): python cad/faibotic/print_mass_pl.py -> in JSON (dùng cho spec.PRINT_FACTOR_PL)."""
import json, os
from build123d import import_step
HERE = os.path.dirname(os.path.abspath(__file__)); STP = os.path.join(HERE, "..", "upstream", "AM-ARM", "am-arm200", "stp", "follower")
PARTS = {"OB_F_J3_Shoulder": "upper", "OB_F_J4_Elbow": "fore", "OB_F_J5_Elbow_Wrist_Bracket": "wrist", "OB_F_J5_Pin_Bearing": "wrist",
         "OB_F_J56_Wrist_Housing": "wrist", "OB_F_J56_Bottom": "wrist", "OB_F_J56_Top": "wrist", "O_F_J2_Rotation": "shoulder", "O_F_J1_Base": "mount"}
RHO, WALLS, LINE, INFILL = 1.27, 4, 0.04, 0.15
out = {}
for n, b in PARTS.items():
    s = import_step(os.path.join(STP, n + ".stp"))
    V = s.volume / 1000.0; A = s.area / 100.0
    shell = min(A * WALLS * LINE, V)
    m = RHO * (shell + INFILL * (V - shell))
    out[n] = dict(body=b, V_cm3=round(V, 2), A_cm2=round(A, 1), mass_print_g=round(m, 1), mass_solid_1_g=round(V, 1))
agg = {}
for n, r in out.items():
    a = agg.setdefault(r["body"], [0.0, 0.0]); a[0] += r["mass_print_g"]; a[1] += r["mass_solid_1_g"]
print(json.dumps(dict(parts=out, body={b: dict(print_g=round(p, 1), solid_g=round(s, 1), factor=round(p / s, 3)) for b, (p, s) in agg.items()}), indent=1))
