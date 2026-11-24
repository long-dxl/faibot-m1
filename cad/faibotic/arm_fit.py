# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Kiểm giao cắt khuỷu AM-ARM200 (OB_F_J4_Elbow) với gá vai Faibotic.
Chạy: python (venv có build123d) cad/faibotic/arm_fit.py -> cad/export/arm_fit.json
Đặt J4: tâm cung đầu khuỷu (suy từ bbox) trùng đầu link x = LINK_LEN; hướng tay -y_stp -> +x; mặt đối xứng -> y = 0 [giả định, STEP không có ràng buộc lắp].
Chưa mô hình servo khuỷu STS3095 và tai vít. Chưa kiểm đế xoay (xem bao-cao-cad.md mục v0.2)."""
import json, os, sys, time
sys.path.insert(0, os.path.dirname(__file__))
from build123d import import_step, Pos, Rot
import shoulder
from params import LINK_LEN

HERE = os.path.dirname(os.path.abspath(__file__))
j4 = import_step(os.path.join(HERE, "../upstream/AM-ARM/am-arm200/stp/follower/OB_F_J4_Elbow.stp"))
bb = j4.bounding_box()
xc, ye, ze = (bb.min.X + bb.max.X) / 2, bb.max.Y - (bb.max.Z - bb.min.Z) / 2, (bb.min.Z + bb.max.Z) / 2
servo_l, servo_r, cradle = shoulder.servo(-1), shoulder.servo(1), shoulder.cradle()
STATIC = {"cradle": cradle, "servo_L": servo_l, "servo_R": servo_r}
res = []
for qs in (-100, 0, 90):
    link = shoulder.link(qs)
    for qe in (-20, 60, 90, 110, 130, 150):
        loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe) * Rot(Z=90) * Pos(-xc, -ye, -ze)
        f = loc * j4
        row = {"q_vai": qs, "q_khuyu": qe}
        fb = f.bounding_box()
        for n, o in list(STATIC.items()) + [("link", link)]:
            ob = o.bounding_box()
            if (fb.min.X > ob.max.X or fb.max.X < ob.min.X or fb.min.Y > ob.max.Y or fb.max.Y < ob.min.Y or fb.min.Z > ob.max.Z or fb.max.Z < ob.min.Z):
                row[n] = {"common_mm3": 0.0, "min_dist_mm": ">bbox"}
                continue
            try:
                r = f.intersect(o)
                if r is None: c = 0.0   # None = giao rỗng
                else: c = sum(x.volume for x in r) if isinstance(r, (list, tuple)) or hasattr(r, '__iter__') and not hasattr(r, 'volume') else r.volume  # build123d trả ShapeList khi giao nhiều mảnh
            except Exception as e:
                c = None; row["loi_" + n] = repr(e)[:60]
            d = None
            if not c:
                try: d = round(f.distance_to(o), 2)
                except Exception: d = None
            row[n] = {"common_mm3": None if c is None else round(c, 1), "min_dist_mm": d}
        res.append(row)
        print(row, flush=True)
json.dump({"gia_dinh": "J4 đặt theo tâm cung đầu khuỷu trùng đầu link; link 140 mm; không servo khuỷu", "ket_qua": res},
          open(os.path.join(HERE, "../export/arm_fit.json"), "w"), ensure_ascii=False, indent=1)
