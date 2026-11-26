# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Thân đế omni Faibot M1: M1-BS-001 (thân), M1-BS-002 (nắp), M1-BS-003 (cửa D0), M1-BS-010 (bánh omni Ø127). THIẾT KẾ MỚI, NHÁP (22/11/2026).
build123d. Hệ toạ độ như sdk: x trước, y trái, z lên, gốc sàn, tâm đế x=y=0 (mm).
Là thân ĐƠN GIẢN dựng cho bộ ảnh sản phẩm và kiểm giao cắt lắp ráp; CHƯA phải bản vẽ chế tạo. Kích thước lấy từ sdk/faibotic/spec.py (R 200, z 35..170,
bánh Ø127 rộng 40 tại r 170, góc 60/180/300) và battery_box.py (thành 5 mm [giả định]); cửa D0 theo battery_box.fit_base (mặt E-stop/D0 ở phi ~348 độ, bên trước).
Không dựng: khung trong, servo bánh, trục bánh/gối đỡ, vỏ pin (nằm kín trong thân), cáp, ốc.
Chạy: python base_shell.py  -> ../export/M1-BS-001_base_shell.step (thân + nắp + cửa + 3 bánh)."""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); EXP = os.path.join(HERE, "../export")
sys.path.insert(0, os.path.join(HERE, "../../sdk"))
from build123d import Box, Cylinder, Pos, Rot, Align, Compound, export_step, fillet, Sphere
from faibotic import spec
C = (Align.CENTER,) * 3; MIN = (Align.MIN,) * 3
R = spec.BASE_RADIUS * 1000; Z0, Z1 = spec.BASE_Z0 * 1000, spec.BASE_Z1 * 1000      # 200; 35; 170
WALL = 5.0                                                                          # [giả định, battery_box.BASE_WALL_MM]
WR, WW, WRING, WZ = spec.WHEEL_RADIUS * 1000, spec.WHEEL_WIDTH * 1000, spec.WHEEL_MOUNT_RADIUS * 1000, spec.WHEEL_RADIUS * 1000
WANG = [math.degrees(a) for a in spec.WHEEL_ANGLES]
CAP_Z0 = 150.0                         # nắp: z 150..170 (vành nắp ngoài), thân z 35..150
POCKET_W, POCKET_TOP = 137.0, 135.0    # hốc bánh: rộng tiếp tuyến (bánh 127 + 5 mỗi bên), đỉnh vòm z 135
DOOR_PHI, DOOR_W, DOOR_H, DOOR_Z = -12.0, 100.0, 80.0, 100.0    # cửa D0: tâm phi -12 độ (battery_box pcb_phi 348), z tâm 100
GAP = 0.8
D0_SHAFT_XY, D0_SHAFT_HOLE = (33.33, -114.48), 12.0   # v0.4.5: trục đứng D0 (battery_box.d0_loto đặt theo fit vỏ pin), lỗ Ø12 xuyên nắp; assembly_full kiểm trùng < 0,5 mm

def body():
    t = Pos(0, 0, (Z0 + CAP_Z0) / 2) * Cylinder(R, CAP_Z0 - Z0)
    t -= Pos(0, 0, (Z0 + CAP_Z0) / 2 + WALL / 2) * Cylinder(R - WALL, CAP_Z0 - Z0 - WALL)      # đáy 5 mm
    for a in WANG:    # hốc bánh xuyên thành + đáy (bánh nhô xuống đất, z 0..127)
        # hộp từ r 140 ra ngoài; đỉnh vòm bán kính POCKET_W/2 tâm z = POCKET_TOP - POCKET_W/2
        arch = Pos(140, 0, 0) * Box(80, POCKET_W, POCKET_TOP - POCKET_W / 2, align=(Align.MIN, Align.CENTER, Align.MIN)) \
            + Pos(140, 0, POCKET_TOP - POCKET_W / 2) * Rot(Y=90) * Cylinder(POCKET_W / 2, 80, align=(Align.CENTER, Align.CENTER, Align.MIN))
        t -= Rot(Z=a) * arch
    t -= Rot(Z=DOOR_PHI) * Pos(R - WALL / 2, 0, DOOR_Z) * Box(WALL + 2, DOOR_W, DOOR_H)          # cửa sổ cửa D0
    return t

def cap():
    c = Pos(0, 0, (CAP_Z0 + Z1) / 2) * Cylinder(R, Z1 - CAP_Z0)
    # lỗ M6 chân cột (PCD 100 quanh x = -100): nhìn thấy dưới tấm đế cột; cột đặt từ z 170
    for sx in (-50, 50):
        for sy in (-50, 50):
            c -= Pos(spec.COLUMN_X * 1000 + sx, sy, (CAP_Z0 + Z1) / 2) * Cylinder(3.3, Z1 - CAP_Z0 + 2)
    c -= Pos(D0_SHAFT_XY[0], D0_SHAFT_XY[1], (CAP_Z0 + Z1) / 2) * Cylinder(D0_SHAFT_HOLE / 2, Z1 - CAP_Z0 + 2)
    return fillet(c.edges().filter_by_position(__import__("build123d").Axis.Z, Z1 - 0.1, Z1 + 0.1), 3.0) if False else c

def door():
    return Rot(Z=DOOR_PHI) * Pos(R - WALL / 2, 0, DOOR_Z) * Box(WALL - 0.4, DOOR_W - 2 * GAP, DOOR_H - 2 * GAP)

def key():
    return Rot(Z=DOOR_PHI) * Pos(R - 0.2 + 2.5, DOOR_W / 2 - 18, DOOR_Z) * Rot(Y=90) * Cylinder(7, 5)

def omni_wheel():
    """M1-BS-010 nháp: bánh omni Ø127 x 40, trục dọc +x cục bộ (đặt radial). Mua sẵn: hub nhôm + 2 hàng x 10 con lăn (cao su/PU đen mờ) so le."""
    parts = {}
    parts["hub"] = Rot(Y=90) * Cylinder(40.0, 2 * 17.0)
    parts["plateA"] = Pos(18.5, 0, 0) * Rot(Y=90) * Cylinder(33.0, 3.0); parts["plateB"] = Pos(-18.5, 0, 0) * Rot(Y=90) * Cylinder(33.0, 3.0)
    rr, rl, rc, n = 10.0, 19.0, WR - 10.0, 10
    base = Cylinder(rr, rl, rotation=(0, 0, 0))
    base = fillet(base.edges(), 5.0)
    k = 0
    for row, ax in enumerate((-10.0, 10.0)):
        for i in range(n):
            ang = 360.0 / n * i + (18.0 if row else 0.0)
            # con lăn: trục tiếp tuyến (y cục bộ khi ang = 0), tâm trên vòng r = rc trong mặt phẳng y-z
            parts[f"roller{k}"] = Pos(ax, 0, 0) * Rot(X=ang) * Pos(0, 0, rc) * Rot(X=90) * base; k += 1
    return parts

def wheel_world(i):
    a = WANG[i]
    return {n: Pos(0, 0, WZ) * Rot(Z=a) * Pos(WRING, 0, 0) * s for n, s in omni_wheel().items()}

def parts():
    out = [("M1-BS-001_base_shell", "ivory", body()), ("M1-BS-002_base_cap", "navy", cap()), ("M1-BS-003_door_D0", "ivory_hi", door()), ("M1-BS-003_door_key", "steel", key())]
    for i in range(3):
        for n, s in wheel_world(i).items():
            out.append((f"M1-BS-010_wheel{i}_{n}", "hub" if n.startswith(("hub", "plate")) else "rubber", s))
    return out

if __name__ == "__main__":
    ps = parts()
    export_step(Compound([s for _, _, s in ps]), os.path.join(EXP, "M1-BS-001_base_shell.step"))
    for n, _, s in ps[:4]:
        print(n, round(s.volume), [round(v) for v in (s.bounding_box().min.X, s.bounding_box().max.X, s.bounding_box().min.Z, s.bounding_box().max.Z)])
