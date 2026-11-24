# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Vỏ pin 3S5P (15 x 21700) + BMS 3S 200 A + khoang F1/D0, v2 (vòng 2 theo QC2-vo-pin.md). build123d + numpy.
 Chạy: nice -n 15 python battery_box.py [thư mục xuất]  (mặc định ../export)
Mọi số có cờ nguồn trong ../../archive/v0.4/cad/bao-cao-vo-pin.md. Kích thước linh kiện chưa có bản vẽ hãng: [giả định], đo hàng thật rồi sửa."""
import json, math, os, sys, itertools
import numpy as np
from build123d import Box, Cylinder, Pos, Rot, Align, Compound, export_step

MIN = (Align.MIN, Align.MIN, Align.MIN)

# ---------------- tham số (mm) ----------------
CELL_D, CELL_L = 21.22, 70.30      # R0 23/11: Samsung INR21700-40T (BOM M1-PW-001, D7) tối đa Ø21,22 x 70,30 đầu phẳng [số hãng tự báo: 18650batterystore, Conrad, Keeppower; chưa có bản vẽ Samsung]; cũ 21,55 x 70,15 (P42A)
PACK_AXIS = 76.0                   # cell + nickel [QC2]
CELL_G = 70.0
PITCH = 22.2                       # tâm cell; khe 0,65 mm cho thành giá đỡ in [giả định]
NX, NZ = 5, 3
CLR_SIDE, CLR_END = 3.0, 4.0
W, FLOOR, LID_T, PART = 3.0, 3.0, 3.0, 3.0

BMS = (184.0, 33.0, 65.0)          # x dài, y dày, z cao: bản 100 A 184x65x33 [tinh-toan.md, snippet]; QC2 yêu cầu dùng kích thước lớn hơn; bản 200 A chưa có số liệu
BMS_G = 200.0                      # [giả định]
F1 = (19.05, 68.58, 10.67)         # holder MEGA: 68,58 x 19,05 x 10,67 [snippet RS 8783946, chưa kiểm]; nằm dọc y
F1_G = 60.0
D0 = (56.0, 45.0, 56.0)            # Blue Sea 6005: rộng 55,2-56,2 mm [spec sheet BS]; sâu, cao [giả định]
D0_STUD_SP, D0_STUD_LEN, D0_STUD_D = 40.0, 22.0, 9.5   # cọc 3/8 in (9,5 mm) [BOM M1-PW-008 ghi cọc 3/8 in, snippet]; khoảng cách, dài [giả định]
D0_G = 150.0
LUG_PALM = (22.0, 3.0, 24.0)       # cos 2 AWG lỗ 3/8 in: bản (rộng, dày, cao) [giả định, thông lệ]
LUG_BARREL_D, LUG_BARREL_L = 15.0, 32.0
NUT38 = (16.5, 8.5)                # đai ốc 3/8 in: đường kính ngoài, cao [giả định]
WASHER = (22.0, 2.0)
K1 = (80.5, 65.6, 60.0)            # EV200: 80,48 x 65,6 [snippet RS 6172757, chưa chắc đúng mã]; cao [giả định]
PCB = (100.0, 80.0, 1.6); PCB_COMP_H = 18.0; PCB_POST_H = 8.0
WB_SIZES = ((150.0, 75.0), (140.0, 70.0))   # R0 23/11: bảng đi dây EVT (electrical/bang-di-day-evt.md) thay PCB, tấm nhôm 3 mm + 2 ray DIN 150 mm; quét 200x80, 180x80, 160x80 không lọt cùng vỏ pin + K1 với biên 5 mm
WB_H = 75.0                        # cao linh kiện trên tấm (cầu chì MEGA, MCB, rơ-le trên đế) [giả định]
WALL_MARGIN = 5.0                  # R0 23/11: mọi khối cách thành thân >= 5 mm (08: PCB+E-stop cũ chỉ 1,4 mm); nút E-stop/Reset lắp trên thành thân, nối dây, không nằm trên bảng
ESTOP_HOLE, ESTOP_HEAD = 22.0, 40.0
BAR_W_D0 = 21.0                    # R0 v2 (QC2 M2): đoạn đứng tại cọc 3/8 in D0 rộng 21: (21-10,2)x5 = 54 mm² (biên 61%; 17 chỉ còn 34 mm², biên 1,2%)
BAR_T, BAR_W = 5.0, 17.0           # R0 23/11: thanh đồng dẹt 5 x 17 = 85 mm²; tại lỗ Ø8,5 (M8) còn (17-8,5)x5 = 42,5 mm², tại lỗ Ø10,2 (cọc 3/8 in) còn 34 mm² >= 33,6 mm² của 2 AWG (cũ 3 x 15: 15-19,5 mm²); 4 x 20 thử trước: khe N1-N2 còn 2,9 mm
WIRE2_OD = 11.5
BEND_MIN = 5 * WIRE2_OD            # 57,5 mm
FT_Y, FT_GAP = 62.0, 11.0          # v0.4.5: cọc xuyên nắp tại (X_STUD_B; 62) như lỗ cáp cũ; thanh ngang cách mặt dưới nắp khoang 11 mm (ống đỡ PA)
LIP_H, LIP_W = 2.0, 3.0
EAR_BUSH_OD, EAR_BUSH_ID = 10.0, 5.5   # ống lót thép có gờ (QC2): lỗ tai Ø10
EAR_D = EAR_BUSH_OD
EAR_T, EAR_Y = 4.0, 14.0
LUG = 9.0
LUG_Z = 12.0
INSERT_D, INSERT_DEPTH = 5.6, 8.0

# thân robot (spec.py, đọc để kiểm; không sửa spec.py)
BASE_RADIUS_MM = 200.0
BASE_WALL_MM = 5.0                 # thành thân ước [giả định]
R_IN = BASE_RADIUS_MM - BASE_WALL_MM
BASE_Z0, BASE_Z1 = 35.0, 170.0
WHEEL_R, WHEEL_W, WHEEL_MR = 63.5, 40.0, 170.0
WHEEL_ANG = (60.0, 180.0, 300.0)
SERVO_RAD = (105.0, 150.0); SERVO_TAN = 46.0; SERVO_H = 35.0   # STS3215 (servo bánh theo D9) ~45,2 x 35 x 24,7 [chưa kiểm], trục hướng tâm, 46 mm tiếp tuyến [giả định bố trí]
T_DECK = 4.0

# ---------------- hình học vỏ ----------------
PACK_X = NX * PITCH
PACK_Z = NZ * PITCH
IN_PACK = (PACK_X + 2 * CLR_SIDE + 15.0, PACK_AXIS + 2 * CLR_END, PACK_Z + 2 * CLR_SIDE)   # +15 mm dư cuối +x cho cực P- của BMS
IN_BAY_X = 100.0
IN_BMS_Y = BMS[1] + 5.0
IN_X = IN_BAY_X + PART + IN_PACK[0]
IN_Y = IN_PACK[1] + PART + IN_BMS_Y
IN_Z = IN_PACK[2]
OUT = (IN_X + 2 * W, IN_Y + 2 * W, FLOOR + IN_Z + LID_T)
BODY_H = FLOOR + IN_Z
# khoang F1/D0 ở -x, pack ở +x
X_BAY0 = W
X_P1 = W + IN_BAY_X                # vách P1 [X_P1, X_P1+PART]
X_PACK0 = X_P1 + PART
Y0, Y_P2, Y_BMS0 = W, W + IN_PACK[1], W + IN_PACK[1] + PART
ZC = FLOOR + IN_Z / 2
EAR_X = (X_PACK0 + 15.0, OUT[0] - 25.0)
EAR_X = (30.0, OUT[0] - 30.0)

# vị trí linh kiện khoang (hệ hộp)
BMS_X0 = X_PACK0 - 68.0           # đầu -x (cọc P+) hướng về khoang F1/D0; đầu +x còn ~16 mm dư cho cọc P-
BMS_Y0 = Y_BMS0 + 3.5
X_PB = BMS_X0 - 6.0                # bản cos/thanh tại cọc P+ BMS
X_F1 = X_PB - 11.0                 # tâm F1 (thanh 1 nằm trên cọc sau F1)
F1_Y0 = 14.9
F1_STUD_F = F1_Y0 + 7.6
F1_STUD_R = F1_Y0 + F1[1] - 7.6
X_D0 = 70.0                        # tâm D0
Z_STUD = ZC - 10.0                 # cọc D0 thấp 10 mm để cos ra phía trên (cáp ra qua nắp)
X_STUD_A, X_STUD_B = X_D0 - D0_STUD_SP / 2, X_D0 + D0_STUD_SP / 2   # A nối thanh 2 -> F1, B nối cáp ra
Y_D0_BACK = Y0 + D0[1]
Y_PALM = Y_D0_BACK + 6.0           # tâm bản cos trên cọc

FRONT_LUGS = [20.0, 85.0]          # bắt nắp khoang
MAIN_FRONT_LUGS = [130.0, 200.0]
REAR_LUGS = [20.0, 85.0, 150.0, 200.0]
BAY_END_LUG_Y = 45.0
P2_BOSS = (60.0, Y_P2 - 9.0)       # (x, y) bướu hạt chèn nắp khoang trên P2
INSERT_POS = ([(x, -LUG / 2) for x in FRONT_LUGS] + [(x, -LUG / 2) for x in MAIN_FRONT_LUGS] + [(x, OUT[1] + LUG / 2) for x in REAR_LUGS]
              + [(-LUG / 2, BAY_END_LUG_Y), (P2_BOSS[0], P2_BOSS[1] + 4.5)])


def cyl_y(x, z, y0, y1, d):
    return Pos(x, y0, z) * Rot(X=-90) * Cylinder(d / 2, y1 - y0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def cyl_x(y, z, x0, x1, d):
    return Pos(x0, y, z) * Rot(Y=90) * Cylinder(d / 2, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def cyl_z(x, y, z0, z1, d):
    return Pos(x, y, z0) * Cylinder(d / 2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def box(x0, y0, z0, dx, dy, dz):
    return Pos(x0, y0, z0) * Box(dx, dy, dz, align=MIN)


EARS = 'y'


def ear_rects():
    if EARS == 'y':
        return [(x - 11, (-EAR_Y if sg < 0 else OUT[1]), 22, EAR_Y) for x in EAR_X for sg in (-1, 1)]
    return [((-EAR_Y if sg < 0 else OUT[0]), y - 11, EAR_Y, 22) for y in (30.0, OUT[1] - 30.0) for sg in (-1, 1)]


def ear_footprint():
    return (OUT[0], OUT[1] + 2 * EAR_Y) if EARS == 'y' else (OUT[0] + 2 * EAR_Y, OUT[1])


def lips(b, holes):
    for ax, p, q, c0, dep in holes:
        if ax == 'y':
            for yy in (c0 - LIP_H, c0 + dep):
                b += cyl_y(p, q, yy, yy + LIP_H, 14.0 + 2 * LIP_W) - cyl_y(p, q, yy - 1, yy + LIP_H + 1, 14.0)
        else:
            for xx in (c0 - LIP_H, c0 + dep):
                b += cyl_x(p, q, xx, xx + LIP_H, 14.0 + 2 * LIP_W) - cyl_x(p, q, xx - 1, xx + LIP_H + 1, 14.0)
    return b


LV_BOSS, LV_W, LV_SHIFT = 3.0, 3.0, 4.0
def louver_chevron(xs, z0, h):
    """kênh khe vào chữ chi qua thành trước dày LV_BOSS + W (y -LV_BOSS..W): rộng LV_W theo x, lệch LV_SHIFT ở giữa thành, cao h (z)"""
    from build123d import Polygon, extrude
    y0, y1 = -LV_BOSS - 0.5, W + 0.5; ym = (-LV_BOSS + W) / 2
    pts = [(xs, y0), (xs, -LV_BOSS), (xs + LV_SHIFT, ym), (xs, W), (xs, y1), (xs + LV_W, y1), (xs + LV_W, W), (xs + LV_SHIFT + LV_W, ym), (xs + LV_W, -LV_BOSS), (xs + LV_W, y0)]
    return Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), amount=h)


def louver_chevron_check(n=400):
    """2D (mặt cắt x-y, kênh là lăng trụ theo z nên đủ): có đoạn thẳng nào từ miệng ngoài (y -LV_BOSS) tới miệng trong (y W) nằm trọn trong kênh không."""
    ym = (-LV_BOSS + W) / 2
    def inside(x, y):
        t = 1 - abs(y - ym) / ((W + LV_BOSS) / 2); lo = LV_SHIFT * t
        return lo - 1e-9 <= x <= lo + LV_W + 1e-9
    xs = np.linspace(0, LV_W, 61); found = 0
    for xa in xs:
        for xb in xs:
            ys = np.linspace(-LV_BOSS, W, n); xl = xa + (xb - xa) * (ys + LV_BOSS) / (W + LV_BOSS)
            if all(inside(x, y) for x, y in zip(xl, ys)):
                found += 1
    return dict(duong_thang_xuyen=found, ghi_chu="0 = không có đường thẳng (dây/phoi thẳng không lọt); độ rộng kênh vuông góc %.2f mm" % (LV_W * math.cos(math.atan(LV_SHIFT / ((W + LV_BOSS) / 2)))))


def louver_cut(shape, x, y, z, w, l, h, tilt):
    """khe xiên (louver): hộp w x l x h nghiêng quanh trục x một góc tilt độ, chặn tầm nhìn thẳng"""
    return shape - Pos(x, y, z) * Rot(X=tilt) * Box(w, l, h, align=(Align.MIN, Align.CENTER, Align.CENTER))


def shell():
    b = box(0, 0, 0, OUT[0], OUT[1], BODY_H)
    b -= box(W, W, FLOOR, IN_X, IN_Y, IN_Z + 1)
    b += box(X_P1, Y0, FLOOR, PART, IN_PACK[1], IN_Z)
    b += box(W, Y_P2, FLOOR, IN_X, PART, IN_Z)
    # tai bắt đế (EARS = 'y': 2 mỗi cạnh dọc; 'x': 2 mỗi đầu), lỗ Ø10 cho ống lót thép
    for (ex, ey, dx, dy) in ear_rects():
        b += box(ex, ey, 0, dx, dy, EAR_T)
        b -= cyl_z(ex + dx / 2, ey + dy / 2, -1, EAR_T + 1, EAR_D)
    for (x, y) in INSERT_POS:
        if x < 0:
            b += box(-LUG, y - 7, BODY_H - LUG_Z, LUG + 0.5, 14, LUG_Z)
        elif y == P2_BOSS[1] + 4.5:
            b += box(x - 5, y - 9, BODY_H - LUG_Z, 10, 9.0 + 0.0, LUG_Z)
        else:
            y0 = -LUG if y < 0 else OUT[1] - 0.5
            b += box(x - 7, y0, BODY_H - LUG_Z, 14, LUG + 0.5, LUG_Z)
        b -= cyl_z(x, y, BODY_H - INSERT_DEPTH, BODY_H + 1, INSERT_D)
    return b


def build_box():
    b = shell()
    cuts = []
    # B+/B- cell -> BMS: 2 lỗ Ø14 trong vách P2 phía pack
    for x in (X_PACK0 + 12.0, X_PACK0 + PACK_X - 8.0):
        b -= cyl_y(x, ZC, Y_P2 - 1, Y_P2 + PART + 1, 14.0)
        cuts.append(('y', x, ZC, Y_P2, PART))
    # khe dây cân bằng + NTC 14 x 5 có gờ lăn
    xb = X_PACK0 + 60
    b -= box(xb - 7, Y_P2 - 1, ZC - 2.5, 14, PART + 2, 5)
    for yy in (Y_P2 - LIP_H, Y_P2 + PART):
        b += box(xb - 11, yy, ZC - 6.5, 22, LIP_H, 2) + box(xb - 11, yy, ZC + 4.5, 22, LIP_H, 2)
    # thanh 1 (BMS P+ -> F1): khe 30 x 12 xuyên P2 (thanh đồng dẹt, có gờ cách điện: bọc co nhiệt)
    b -= box(X_F1 - 15, Y_P2 - 1, ZC - 12.5 - 6, 30, PART + 2, 14)   # R0: cao 12 -> 14 cho thanh dày 5 (khe tới nhựa >= 2 mm)
    # P- BMS ra thành +x khoang BMS
    b -= cyl_x(Y_BMS0 + IN_BMS_Y / 2, ZC, OUT[0] - W - 1, OUT[0] + 1, 14.0)
    cuts.append(('x', Y_BMS0 + IN_BMS_Y / 2, ZC, OUT[0] - W, W))
    # lỗ D0 trên thành trước khoang F1/D0: lỗ trục 7/8 in = 22,2 mm [snippet, chưa kiểm]
    b -= cyl_y(X_D0, ZC, -1, W + 1, 22.2)
    # v0.4.5: khe vào chữ chi (thay louver xiên 40 độ: còn đường thẳng 3 x 8,9 mm, QC2 V2.4). Gờ ngoài 3 mm trên vùng khe
    # (thành trước dày 6 tại đó, y -3..3); mỗi khe là kênh rộng 3 (x) đổi hướng ở giữa thành, lệch LV_SHIFT 4 > 3: không có đường thẳng nào xuyên qua (louver_chevron_check)
    b += box(X_PACK0 + 4, -LV_BOSS, FLOOR + 3, 6 * 18.0, LV_BOSS, 36.0)
    for i in range(6):
        b -= louver_chevron(X_PACK0 + 8 + i * 18.0, FLOOR + 6, 30.0)
    b = lips(b, cuts)
    return b


BAY_WIN = (0.0, 0.0, X_P1 + 0.0, Y_P2)   # x0, y0, dx, dy (cửa sổ nắp khoang, gồm thành -x)


def build_lid():
    l = box(0, 0, 0, OUT[0], OUT[1], LID_T)
    l -= box(BAY_WIN[0] - 1, BAY_WIN[1] - 1, -1, BAY_WIN[2] + 0.5 + 1, BAY_WIN[3] + 1.5, LID_T + 2)
    l -= box(P2_BOSS[0] - 6, P2_BOSS[1] - 0.5, -1, 12, 10, LID_T + 2)
    for x in MAIN_FRONT_LUGS:
        l += box(x - 7, -LUG, 0, 14, LUG + 1, LID_T); l -= cyl_z(x, -LUG / 2, -1, LID_T + 1, 4.5)
    for x in REAR_LUGS:
        l += box(x - 7, OUT[1] - 1, 0, 14, LUG + 1, LID_T); l -= cyl_z(x, OUT[1] + LUG / 2, -1, LID_T + 1, 4.5)
    rim = box(W + 0.4, W + 0.4, -2.0, IN_X - 0.8, IN_Y - 0.8, 2.0) - box(W + 1.6, W + 1.6, -3, IN_X - 3.2, IN_Y - 3.2, 4)
    rim -= box(-1, -1, -3, BAY_WIN[2] + 6, BAY_WIN[3] + 2, 4)
    rim -= box(X_P1 - 0.2, 0, -3, PART + 0.4, OUT[1], 4) + box(0, Y_P2 - 0.2, -3, OUT[0], PART + 0.4, 4)
    l += rim
    # louver xiên 40 độ trên nắp phía pack (8 khe), che dây/phoi nhìn thẳng
    for i in range(8):
        l = louver_cut(l, X_PACK0 + 8 + i * 13.0, W + 42, LID_T / 2, 3.0, 8.0, 40.0, 0) if False else l
        cx = X_PACK0 + 8 + i * 13.0
        l -= Pos(cx, W + 42, LID_T / 2) * Rot(Y=40) * Box(8.0, 40.0, 3.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return l


def build_bay_lid():
    l = box(BAY_WIN[0], BAY_WIN[1], 0, BAY_WIN[2] + 0.0, BAY_WIN[3], LID_T)
    for x in FRONT_LUGS:
        l += box(x - 7, -LUG, 0, 14, LUG + 1, LID_T); l -= cyl_z(x, -LUG / 2, -1, LID_T + 1, 4.5)
    l += box(-LUG, BAY_END_LUG_Y - 7, 0, LUG + 1, 14, LID_T); l -= cyl_z(-LUG / 2, BAY_END_LUG_Y, -1, LID_T + 1, 4.5)
    l += box(P2_BOSS[0] - 5, P2_BOSS[1], 0, 10, 10, LID_T); l -= cyl_z(P2_BOSS[0], P2_BOSS[1] + 4.5, -1, LID_T + 1, 4.5)
    # v0.4.5: lỗ Ø20 cho ống lót cách điện của cọc xuyên nắp (cọc ra D0)
    l -= cyl_z(X_STUD_B, FT_Y, -1, LID_T + 1, 20.0)
    return l


# ---------------- linh kiện giả ----------------
def lug_stack_y(x, z, y_palm):
    """chồng cọc 3/8 in trục +y (D0): cọc, bản cos, long-đền, đai ốc. Trả về dict các khối."""
    d = {}
    d['stud'] = cyl_y(x, z, Y_D0_BACK, Y_D0_BACK + D0_STUD_LEN, D0_STUD_D)
    d['palm'] = box(x - LUG_PALM[0] / 2, y_palm - LUG_PALM[1] / 2, z - LUG_PALM[2] / 2, LUG_PALM[0], LUG_PALM[1], LUG_PALM[2])
    d['washer'] = cyl_y(x, z, y_palm + LUG_PALM[1] / 2, y_palm + LUG_PALM[1] / 2 + WASHER[1], WASHER[0])
    y_n0 = y_palm + LUG_PALM[1] / 2 + WASHER[1]
    d['nut'] = cyl_y(x, z, y_n0, y_n0 + NUT38[1], NUT38[0])
    return d


def components():
    c = {}
    zc0 = FLOOR + CLR_SIDE
    cells = None
    for iz in range(NZ):
        for ix in range(NX):
            y0 = Y0 + CLR_END + (PACK_AXIS - CELL_L) / 2
            cyl = cyl_y(X_PACK0 + CLR_SIDE + (ix + 0.5) * PITCH, zc0 + (iz + 0.5) * PITCH, y0, y0 + CELL_L, CELL_D)
            cells = cyl if cells is None else cells + cyl
    c['cells'] = cells
    c['busbar_ends'] = (box(X_PACK0 + CLR_SIDE, Y0 + CLR_END, zc0, PACK_X, 0.6, PACK_Z)
                        + box(X_PACK0 + CLR_SIDE, Y0 + CLR_END + PACK_AXIS - 0.6, zc0, PACK_X, 0.6, PACK_Z))
    # R0 23/11: lưới thép không gỉ mắt 1 mm (dây 0,3) dán sau 6 louver vào, che đường thẳng 3 x 8,9 mm; tấm 0,6 mm sát mặt trong thành trước, cao 40
    c['mesh_inlet'] = box(X_PACK0 + 4, W, FLOOR + 1, 6 * 18.0, 0.6, 40.0)
    c['bms'] = box(BMS_X0, BMS_Y0, FLOOR + (IN_Z - BMS[2]) / 2, BMS[0], BMS[1], BMS[2])
    # F1 nằm dọc y, đặt trên cột đỡ sao cho mặt bản cos ở z = ZC - 12,5
    z_palm_f1 = ZC - 12.5
    f1_base = z_palm_f1 - F1[2] - 6.5
    c['f1'] = box(X_F1 - F1[0] / 2, F1_Y0, f1_base, F1[0], F1[1], F1[2])
    for nm, ys in (('r', F1_STUD_R), ('f', F1_STUD_F)):
        c[f'f1_stud_{nm}'] = cyl_z(X_F1, ys, f1_base + F1[2], z_palm_f1 + BAR_T + 1.6 + 6.5, 8.0)
        c[f'f1_spacer_{nm}'] = cyl_z(X_F1, ys, f1_base + F1[2], z_palm_f1, 14.0)
        c[f'f1_nut_{nm}'] = cyl_z(X_F1, ys, z_palm_f1 + BAR_T + 1.6, z_palm_f1 + BAR_T + 1.6 + 6.5, 14.5)
    # D0
    c['d0_body'] = box(X_D0 - D0[0] / 2, Y0, ZC - D0[2] / 2, D0[0], D0[1], D0[2])
    for nm, xs in (('A', X_STUD_A), ('B', X_STUD_B)):
        for k, v in lug_stack_y(xs, Z_STUD, Y_PALM).items():
            if k == 'palm':      # A: thanh 2 tựa long-đền; B (v0.4.5): thanh 3 (bar3_v) thay bản cos
                continue
            c[f'd0_{nm}_{k}'] = v
    # v0.4.5: ra D0 qua cọc xuyên nắp khoang thay cáp đứng. Thanh đồng 5 x 21/17 từ cọc B lên dưới nắp khoang (bar3_v, bar3_h), cọc M8 đồng thau
    # đứng (đầu dưới thanh) qua ống cách điện PA trong lỗ Ø20 nắp khoang; trên nắp: cos 2 AWG M8 nằm ngang, thùng cos và cáp đi ngang về +y (phía sau vỏ, tới bảng đi dây).
    # Cáp không còn phải uốn đứng -> ngang trong 32 mm dưới nắp thân (QC2 V2.6: đỉnh cáp 57-69 mm > chỗ có). Chiều dài vỏ không đổi (241, lọt bàn in 256).
    yb = Y_PALM + LUG_PALM[1] / 2          # mặt sau thanh tựa long-đền cọc B (như bản cos cũ)
    zb_top = BODY_H - FT_GAP - BAR_T       # đáy thanh ngang
    c['bar3_v'] = box(X_STUD_B - BAR_W_D0 / 2, yb - BAR_T, Z_STUD - 12.0, BAR_W_D0, BAR_T, zb_top + BAR_T - (Z_STUD - 12.0))
    c['bar3_h'] = box(X_STUD_B - BAR_W / 2, yb - BAR_T, zb_top, BAR_W, FT_Y + BAR_W / 2 - (yb - BAR_T), BAR_T)
    c['ft_head'] = cyl_z(X_STUD_B, FT_Y, zb_top - 5.5, zb_top, 13.0)
    c['ft_stud'] = cyl_z(X_STUD_B, FT_Y, zb_top, BODY_H + LID_T + 2.0 + 3.0 + 2.0 + 6.5 + 2.0, 8.0)
    zt = BODY_H + LID_T + 2.0                # mặt trên gờ ống cách điện
    c['outlug_palm'] = box(X_STUD_B - LUG_PALM[0] / 2, FT_Y - 11.0, zt, LUG_PALM[0], 22.0, LUG_PALM[1])
    c['outlug_barrel'] = cyl_y(X_STUD_B, zt + LUG_BARREL_D / 2, FT_Y + 11.0, FT_Y + 11.0 + LUG_BARREL_L, LUG_BARREL_D)
    c['out_cable'] = cyl_y(X_STUD_B, zt + LUG_BARREL_D / 2, FT_Y + 11.0 + LUG_BARREL_L, FT_Y + 11.0 + LUG_BARREL_L + 60.0, WIRE2_OD)
    c['ft_washer'] = cyl_z(X_STUD_B, FT_Y, zt + LUG_PALM[1], zt + LUG_PALM[1] + 2.0, 16.0)
    c['ft_nut'] = cyl_z(X_STUD_B, FT_Y, zt + LUG_PALM[1] + 2.0, zt + LUG_PALM[1] + 2.0 + 6.5, 14.5)
    # cách điện (không mang điện): ống đỡ PA giữa thanh ngang và nắp khoang, ống lót có gờ trong lỗ Ø20 nắp khoang
    c['ft_standoff'] = cyl_z(X_STUD_B, FT_Y, zb_top + BAR_T, BODY_H, 16.0) - cyl_z(X_STUD_B, FT_Y, zb_top, BODY_H + 1, 8.5)
    c['ft_bush'] = (cyl_z(X_STUD_B, FT_Y, BODY_H, BODY_H + LID_T, 20.0) + cyl_z(X_STUD_B, FT_Y, BODY_H + LID_T, zt, 26.0)) - cyl_z(X_STUD_B, FT_Y, BODY_H - 1, zt + 1, 8.5)
    # v0.4.5: NTC 1 giữa pack (mặt trên cell giữa hàng trên), NTC 2 dán trên vùng FET BMS (đầu -x, cọc P+), dây về BMS qua khe cân bằng [giả định cỡ 4 x 3 x 12]
    c['ntc1'] = box(X_PACK0 + CLR_SIDE + 2.5 * PITCH - 2, Y0 + CLR_END + PACK_AXIS / 2 - 6, zc0 + NZ * PITCH, 4.0, 12.0, 3.0)
    c['ntc2'] = box(BMS_X0 + 10.0, BMS_Y0 + BMS[1] / 2 - 6, FLOOR + (IN_Z - BMS[2]) / 2 + BMS[2], 4.0, 12.0, 3.0)
    # thanh 1: BMS P+ (cọc trục -x tại đầu -x của BMS) -> cọc sau F1
    bms_ys = BMS_Y0 + BMS[1] / 2
    c['bms_stud_pos'] = cyl_x(bms_ys, ZC, BMS_X0 - D0_STUD_LEN, BMS_X0, 8.0)
    zp = ZC - 12.5
    hw = BAR_W / 2
    c['bar1_v'] = box(X_PB - BAR_T / 2, bms_ys - hw, zp, BAR_T, BAR_W, 25.0)
    c['bar1_h'] = box(X_F1 - hw, F1_STUD_R - hw, zp, X_PB + BAR_T / 2 - (X_F1 - hw), bms_ys + hw - (F1_STUD_R - hw), BAR_T)
    # thanh 2: cọc trước F1 -> cọc A của D0, đi vòng sau lưng D0 (y >= lưng D0 + 1,5)
    yb0 = Y_D0_BACK + 3.0
    c['bar2_a'] = box(X_F1 - hw, F1_STUD_F - hw, zp, BAR_W, yb0 + 12.0 - (F1_STUD_F - hw), BAR_T)
    c['bar2_b'] = box(X_F1 - hw, yb0, zp, X_STUD_A + hw - (X_F1 - hw), 12.0, BAR_T)
    c['bar2_v'] = box(X_STUD_A - BAR_W_D0 / 2, Y_PALM - BAR_T / 2, zp, BAR_W_D0, BAR_T, Z_STUD + 12.0 - zp)   # R0 v2: đầu cọc D0 rộng hơn
    return c


# ---------------- v0.4.5: truy cập D0 từ ngoài thân + khoá LOTO ----------------
# Cần nối dài ngang ~82 mm đi theo trục D0 (-y hộp) ra thành thân ở phi ~ -55 độ: lọt vào hốc bánh 300 độ (kiểm d0_ext_check trong assembly_full --d0).
# Thay bằng bộ đổi hướng 90 độ (bánh răng côn 1:1) trên trục D0 + trục đứng Ø10 lên qua nắp thân (lỗ Ø12 trên M1-BS-002) + núm và tấm khoá LOTO trên mặt nắp thân.
# Hệ hộp; z thế giới = z hộp + PW_Z (assembly_full). Kích thước bộ đổi hướng, trục, núm [giả định], chưa có mã.
D0_DRIVE = (32.0, 30.0, 32.0)        # x, y, z hộp bao bộ đổi hướng
D0_SHAFT_D, D0_TOP_Z = 10.0, 131.0   # trục đứng; đỉnh trục (hệ hộp) = 171 thế giới (nắp thân z 150..170)
def d0_loto():
    yc = -D0_DRIVE[1] / 2 - 1.0
    d = {}
    d['d0_drive'] = box(X_D0 - D0_DRIVE[0] / 2, yc - D0_DRIVE[1] / 2, ZC - D0_DRIVE[2] / 2, *D0_DRIVE)
    d['d0_adapter'] = cyl_y(X_D0, ZC, yc + D0_DRIVE[1] / 2, 0.0, 20.0)
    d['d0_shaft'] = cyl_z(X_D0, yc, ZC + D0_DRIVE[2] / 2, D0_TOP_Z, D0_SHAFT_D)
    return d, (X_D0, yc)


# ---------------- khớp thân Ø400 ----------------
def _zone_mask(P):
    bad = np.zeros(len(P), bool)
    for a in WHEEL_ANG:
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        rad = P[:, 0] * c + P[:, 1] * s; per = -P[:, 0] * s + P[:, 1] * c
        bad |= (rad >= WHEEL_MR - WHEEL_W / 2 - 3) & (rad <= BASE_RADIUS_MM) & (abs(per) <= WHEEL_R + 3)
        bad |= (rad >= SERVO_RAD[0] - 3) & (rad <= SERVO_RAD[1] + 3) & (abs(per) <= SERVO_TAN / 2 + 3)
    return bad


def _pts(cx, cy, w, h, th, step=4.0):
    xs = np.arange(-w / 2, w / 2 + 1e-6, step); ys = np.arange(-h / 2, h / 2 + 1e-6, step)
    X, Y = np.meshgrid(xs, ys); X = X.ravel(); Y = Y.ravel()
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    return np.stack([cx + X * c - Y * s, cy + X * s + Y * c], 1)


def _ok(r, margin=WALL_MARGIN):
    P = _pts(*r)
    return (np.hypot(P[:, 0], P[:, 1]) <= R_IN - margin).all() and not _zone_mask(P).any()


def _overlap(a, b, gap=4.0):
    def corners(r):
        cx, cy, w, h, th = r; c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        return np.array([[cx + x * c - y * s, cy + x * s + y * c] for x, y in ((-w / 2 - gap / 2, -h / 2 - gap / 2), (w / 2 + gap / 2, -h / 2 - gap / 2), (w / 2 + gap / 2, h / 2 + gap / 2), (-w / 2 - gap / 2, h / 2 + gap / 2))])
    A, B = corners(a), corners(b)
    for poly in (A, B):
        for i in range(4):
            e = poly[(i + 1) % 4] - poly[i]; n = np.array([-e[1], e[0]])
            pa = A @ n; pb = B @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


WB_PREF = (-80.0, 104.0, 150.0, 75.0, 40.0)   # = assembly_full.WB_FIT
K1_PREF = (106.0, -38.0, K1[0], K1[1], 60.0)  # vị trí K1 đã dùng ở v0.4.4 (giữ nếu còn hợp lệ)


def pack_rects(pack):
    """Vật cản 2D có dải cao (hệ hộp, z từ mặt đế hộp) của vỏ pin đặt theo pack: [(chữ nhật hệ thân, z0, z1)]. Hộp chính (không tai) cao tới nắp,
    tai bắt đế (chỉ cao EAR_T), tai hạt chèn M4 (lồi LUG ở BODY_H - LUG_Z..BODY_H + LID_T), gờ khe vào. Khối thấp hơn tai hạt chèn (K1 cao 60) không bị tai chặn."""
    cx, cy, w, h, th = pack; t = math.radians(th); c, s_ = math.cos(t), math.sin(t)
    def tr(lx, ly, dw, dh):
        x, y = lx - OUT[0] / 2, ly - OUT[1] / 2
        return (cx + x * c - y * s_, cy + x * s_ + y * c, dw, dh, th)
    out = [(tr(OUT[0] / 2, OUT[1] / 2, OUT[0], OUT[1]), 0.0, BODY_H + LID_T)]
    for (ex, ey, dx, dy) in ear_rects():
        out.append((tr(ex + dx / 2, ey + dy / 2, dx, dy), 0.0, EAR_T))
    for (x, y) in INSERT_POS:
        if x < 0:
            out.append((tr(-LUG / 2, y, LUG, 14.0), BODY_H - LUG_Z, BODY_H + LID_T))
        elif y == P2_BOSS[1] + 4.5:
            continue
        else:
            out.append((tr(x, y, 14.0, LUG), BODY_H - LUG_Z, BODY_H + LID_T))
    out.append((tr(X_PACK0 + 4 + 54.0, -LV_BOSS / 2, 108.0, LV_BOSS), FLOOR + 3, FLOOR + 39))
    return out


def fit_base():
    """Đặt hộp pin (kèm tai), cụm PCB+E-stop (tấm E-stop sát thành thân, hướng ra ngoài), K1 trong thân Ø400, tránh vùng bánh + servo bánh.
    Chữ nhật theo lưới 8 mm, 10 độ; thành thân ước 5 mm [giả định]. Cột nâng ở mặt trên thân (z >= 170): bu-lông chân cột không nằm dưới mặt đó."""
    global EARS
    best = None
    opts = []
    for ears in ('y', 'x'):
        EARS = ears
        opts.append((ears, ear_footprint()))
    for ears, box_r in opts:
      for th in range(0, 360, 5):
        for cx in np.arange(-168, 169, 6.0):
            for cy in np.arange(-168, 169, 6.0):
                r = (cx, cy, box_r[0], box_r[1], th)
                if math.hypot(cx, cy) > 170:
                    continue
                if not _ok(r):
                    continue
                sc = math.hypot(cx, cy)
                if best is None or sc < best[0]:
                    best = (sc, r, ears)
    if best is None:
        return None
    pack = best[1]
    EARS = best[2]
    # v0.4.5: vật cản 2D của vỏ = hộp + tai đế + mọi tai hạt chèn M4 (lồi LUG 9) + gờ khe vào; trước chỉ hộp + tai đế (tai hạt chèn giao khối bảng 696 mm3)
    prects = pack_rects(pack)
    def hit_pack(r, hgt):
        return any(_overlap(r, q) for q, z0, z1 in prects if z0 < hgt)
    # R0 23/11: bảng đi dây EVT (thay PCB + tấm E-stop); E-stop/Reset trên thành thân, nối dây. Thử cỡ lớn trước, cách thành >= WALL_MARGIN
    # v0.4.5: thử trước vị trí đã chốt ở (assembly_full.WB_FIT, khe 3D 6,1 mm tới vỏ pin); hợp lệ theo ràng buộc mới thì giữ, không thì quét lưới
    pcb = None
    if _ok(WB_PREF) and not hit_pack(WB_PREF, WB_H):
        pcb = (math.hypot(WB_PREF[0], WB_PREF[1]), WB_PREF, WB_PREF[4])
    for wsz in (WB_SIZES if pcb is None else ()):
        for th in range(0, 180, 10):
            for cx in np.arange(-180, 181, 6.0):
                for cy in np.arange(-180, 181, 6.0):
                    r = (cx, cy, wsz[0], wsz[1], th)
                    if math.hypot(cx, cy) > R_IN or not _ok(r) or hit_pack(r, WB_H):
                        continue
                    sc = math.hypot(cx, cy)
                    if pcb is None or sc < pcb[0]:
                        pcb = (sc, r, th)
        if pcb:
            break
    k1 = None
    if pcb and not (hit_pack(K1_PREF, K1[2]) or _overlap(K1_PREF, pcb[1]) or not _ok(K1_PREF)):
        k1 = (0.0, K1_PREF)
    if pcb and k1 is None:
        for th in range(0, 180, 10):
            for cx in np.arange(-190, 191, 8.0):
                for cy in np.arange(-190, 191, 8.0):
                    r = (cx, cy, K1[0], K1[1], th)
                    if math.hypot(cx, cy) > R_IN or hit_pack(r, K1[2]) or _overlap(r, pcb[1]) or not _ok(r):
                        continue
                    sc = math.hypot(cx, cy)
                    if k1 is None or sc < k1[0]:
                        k1 = (sc, r)
    return dict(ears=EARS, pack=pack, pcb=pcb[1] if pcb else None, pcb_phi=pcb[2] if pcb else None, k1=k1[1] if k1 else None)


def to_base(r):
    """phép biến đổi hệ cục bộ chữ nhật (tâm tại tâm hộp) -> hệ thân (z = 0 ở mặt đất)"""
    cx, cy, w, h, th = r
    return Pos(cx, cy, BASE_Z0 + T_DECK) * Rot(Z=th)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "../export"
    os.makedirs(out, exist_ok=True)
    fit = fit_base()
    body, lid, bay_lid = build_box(), build_lid(), build_bay_lid()
    comp = components()
    R = {'fit': {k: ([round(float(x), 1) for x in v] if isinstance(v, tuple) else v) for k, v in (fit or {}).items()}}
    R['dims'] = dict(outer=OUT, outer_with_ears=(*ear_footprint(), OUT[2]), ears=EARS, inner_pack=IN_PACK, inner=(IN_X, IN_Y, IN_Z),
                     bay_inner_x=IN_BAY_X, bms_bay_inner_y=IN_BMS_Y)
    # --- kiểm trong hệ hộp (giao cắt, khe)
    parts = dict(body=body, lid=Pos(0, 0, BODY_H) * lid, bay_lid=Pos(0, 0, BODY_H) * bay_lid, **comp)
    pairs = []
    node = {'N1': ('bms_stud_pos', 'bar1_v', 'bar1_h', 'f1_stud_r', 'f1_nut_r', 'f1_spacer_r'),
            'N2': ('f1_stud_f', 'f1_nut_f', 'f1_spacer_f', 'bar2_v', 'bar2_a', 'bar2_b', 'd0_A_stud', 'd0_A_washer', 'd0_A_nut'),
            'N3': ('d0_B_stud', 'd0_B_washer', 'd0_B_nut', 'bar3_v', 'bar3_h', 'ft_head', 'ft_stud', 'outlug_palm', 'outlug_barrel', 'out_cable', 'ft_washer', 'ft_nut')}
    where = {n: k for k, v in node.items() for n in v}
    for (n1, a), (n2, b) in itertools.combinations(parts.items(), 2):
        try:
            v = a & b
            vol = float(v.volume) if v is not None else 0.0
        except Exception:
            vol = float('nan')
        pairs.append(dict(a=n1, b=n2, common_mm3=round(vol, 3), min_dist_mm=round(float(a.distance_to(b)), 3), na=where.get(n1), nb=where.get(n2)))
    intended = {('d0_body', 'body'), ('d0_body', 'd0_A_stud'), ('d0_body', 'd0_B_stud'), ('body', 'lid'), ('body', 'bay_lid'), ('lid', 'bay_lid'), ('f1', 'f1_stud_r'), ('f1', 'f1_stud_f'), ('f1', 'f1_spacer_r'), ('f1', 'f1_spacer_f'), ('bms', 'bms_stud_pos'),
                ('ft_bush', 'bay_lid'), ('ft_standoff', 'bay_lid'), ('ft_standoff', 'ft_bush'), ('ft_standoff', 'bar3_h'), ('ft_bush', 'outlug_palm'), ('ft_stud', 'ft_bush'), ('ft_stud', 'ft_standoff'),
                ('ntc1', 'cells'), ('ntc2', 'bms')}   # v0.4.5: ống cách điện tựa thanh/nắp/cos (chạm mặt, 0 mm3 trừ sai số lưới), NTC dán
    def isint(p):
        return (p['a'], p['b']) in intended or (p['b'], p['a']) in intended or (p['na'] is not None and p['na'] == p['nb'])
    R['interference_unexpected'] = [p for p in pairs if not (p['common_mm3'] <= 1e-6) and not isint(p)]
    R['interference_intended'] = [(p['a'], p['b'], p['common_mm3']) for p in pairs if not (p['common_mm3'] <= 1e-6) and isint(p)]
    R['cross_node_gap_mm'] = sorted([dict(a=p['a'], b=p['b'], d=p['min_dist_mm'], nodes=f"{p['na']}-{p['nb']}") for p in pairs if p['na'] and p['nb'] and p['na'] != p['nb']], key=lambda r: r['d'])[:8]
    live = [n for n in where]
    R['live_vs_other_lt3mm'] = [dict(a=p['a'], b=p['b'], d=p['min_dist_mm']) for p in pairs if p['min_dist_mm'] < 3.0 and not isint(p) and (p['na'] or p['nb']) and not (p['na'] and p['nb'] and p['na'] == p['nb'])
                                and p['a'] not in ('body',) and p['b'] not in ('body',)]
    R['live_to_body_min'] = sorted([(p['min_dist_mm'], p['a'], p['b']) for p in pairs if (p['a'] == 'body' and p['nb']) or (p['b'] == 'body' and p['na'])])[:4]
    R['cell_to_shell_mm'] = round(float(comp['cells'].distance_to(body)), 3)
    # --- cáp: bán kính uốn và khả thi
    zb0 = Z_STUD + LUG_PALM[2] / 2
    R['cable'] = dict(OD=WIRE2_OD, bend_min_mm=BEND_MIN, W01_BMS_to_F1='thanh đồng dẹt %gx%g Cu-ETP (%g mm²; tại lỗ Ø8,5 còn %g mm² >= 33,6 mm² 2 AWG; 120,5 A), bọc co nhiệt, không uốn cáp; thanh uốn gấp 2 chỗ r >= %g mm' % (BAR_T, BAR_W, BAR_T * BAR_W, (BAR_W - 8.5) * BAR_T, BAR_T),
                      W19_F1_to_D0='thanh đồng dẹt %gx%g (dải sau D0 rộng 12: %g mm²; đầu cọc D0 rộng %g, tại lỗ Ø10,2 còn %g mm²), như trên' % (BAR_T, BAR_W, 12 * BAR_T, BAR_W_D0, (BAR_W_D0 - 10.2) * BAR_T), W_out_D0_to_PCB='v0.4.5: thanh đồng %gx%g từ cọc B D0 lên cọc M8 xuyên nắp khoang (ống cách điện PA), cos 2 AWG M8 nằm ngang trên nắp, thùng cos + cáp ngang về +y, tâm cáp z %.1f (hệ hộp), đỉnh cáp %.1f; khoảng trống tới tấm trên thân (z=170, hệ thân) còn %.1f mm; không uốn đứng. Trên nắp phải có nắp chụp cách điện (boot) cho cọc + cos [giả định, chưa vẽ]' % (BAR_T, BAR_W_D0, BODY_H + LID_T + 2.0 + LUG_BARREL_D / 2, BODY_H + LID_T + 2.0 + LUG_BARREL_D / 2 + WIRE2_OD / 2, BASE_Z1 - (BASE_Z0 + T_DECK + BODY_H + LID_T + 2.0 + LUG_BARREL_D / 2 + WIRE2_OD / 2)),
                      bar3_tiet_dien='thanh 3: tại lỗ cọc B Ø10,2 còn (%g-10,2)x%g = %.1f mm²; tại lỗ cọc xuyên Ø8,5 còn (%g-8,5)x%g = %.1f mm² (>= 33,6 mm² 2 AWG)' % (BAR_W_D0, BAR_T, (BAR_W_D0 - 10.2) * BAR_T, BAR_W, BAR_T, (BAR_W - 8.5) * BAR_T),
                      F1_lug='cos M8 trên cọc F1: thay bằng đầu thanh đồng (khoan Ø8,5); nếu bắt buộc cos 2 AWG tại F1 cần thêm ~44 mm (cos + thùng) và bán kính 57,5 mm: không đủ chỗ trong khoang sâu 84 mm [tự tính]')
    # --- khối lượng + trọng tâm
    shell_g = {n: round(float(s.volume) / 1000.0 * 1.2 * 0.85, 1) for n, s in (('body', body), ('lid', lid), ('bay_lid', bay_lid))}
    bars_g = 8.96 * (BAR_T * BAR_W * 200) / 1000.0
    mp = {'cells': (15 * CELL_G, comp['cells']), 'busbar_nickel': (120.0, comp['busbar_ends']), 'bms': (BMS_G, comp['bms']), 'f1': (F1_G, comp['f1']),
          'd0': (D0_G, comp['d0_body']), 'foam_insulation': (40.0, comp['cells']), 'thanh_dong_hai_cai+cap_ra': (round(bars_g + 60, 1), comp['bar1_h']),
          'ear_bushings_4': (4 * 6.0, comp['cells']), 'body': (shell_g['body'], body), 'lid': (shell_g['lid'], lid), 'bay_lid': (shell_g['bay_lid'], bay_lid)}
    tot = mx = my = mz = 0.0
    rows = {}
    for k, (g, s) in mp.items():
        c = s.center(); cz = c.Z + (BODY_H if k in ('lid', 'bay_lid') else 0.0)
        rows[k] = dict(g=round(g, 1), cog=(round(c.X, 1), round(c.Y, 1), round(cz, 1)))
        tot += g; mx += g * c.X; my += g * c.Y; mz += g * cz
    R['mass_g'] = rows; R['mass_total_g'] = round(tot, 1)
    cog = (mx / tot, my / tot, mz / tot)
    R['cog_box_frame_mm'] = dict(x=round(cog[0], 1), y=round(cog[1], 1), z_above_deck=round(cog[2], 1))
    # --- đặt vào thân
    if fit and fit['pack']:
        cx, cy, w, h, th = fit['pack']
        th_r = math.radians(th)
        lx, ly = cog[0] - OUT[0] / 2, cog[1] - OUT[1] / 2     # trọng tâm so tâm hộp (ears đối xứng)
        gx = cx + lx * math.cos(th_r) - ly * math.sin(th_r); gy = cy + lx * math.sin(th_r) + ly * math.cos(th_r)
        R['cog_base_frame_mm'] = dict(x=round(gx, 1), y=round(gy, 1), r=round(math.hypot(gx, gy), 1), z_above_ground=round(BASE_Z0 + T_DECK + cog[2], 1), mass_g=round(tot, 1),
                                      moment_g_mm=(round(tot * gx), round(tot * gy)))
        # D0: toạ độ cần gạt, hướng, khoảng tới thành thân
        d0x, d0y = X_D0 - OUT[0] / 2, 0.0 - OUT[1] / 2 - 25.0     # mặt trước + cần gạt nhô ~25 mm
        pg = (cx + d0x * math.cos(th_r) - d0y * math.sin(th_r), cy + d0x * math.sin(th_r) + d0y * math.cos(th_r))
        dirv = (math.sin(th_r), -math.cos(th_r))          # hướng trục -y cục bộ
        # giao tia với vòng R_IN
        b_ = pg[0] * dirv[0] + pg[1] * dirv[1]; c_ = pg[0] ** 2 + pg[1] ** 2 - R_IN ** 2
        t = -b_ + math.sqrt(max(b_ * b_ - c_, 0))
        R['d0_access'] = dict(knob_xy=[round(pg[0], 1), round(pg[1], 1)], facing_deg=round(math.degrees(math.atan2(dirv[1], dirv[0])), 1), distance_to_body_wall_mm=round(t, 1),
                              note='mặt D0 hướng ra thành thân; cần cửa nắp ở thành thân hoặc cần nối dài ~%d mm (đề xuất)' % round(t))
    # --- kiểm lọt thân bằng lưới (độc lập với tìm kiếm): mọi điểm mép
    chk = {}
    if fit:
        for nm, r in (('pack_with_ears', fit['pack']), ('bang_di_day_evt', fit['pcb']), ('k1', fit['k1'])):
            if r is None:
                chk[nm] = None; continue
            P = _pts(*r, step=2.0)
            chk[nm] = dict(max_r=round(float(np.hypot(P[:, 0], P[:, 1]).max()), 1), bien_toi_thanh_mm=round(R_IN - float(np.hypot(P[:, 0], P[:, 1]).max()), 1), in_zone_points=int(_zone_mask(P).sum()), r_in=R_IN)
        R['fit_check'] = chk
        R['fit_params'] = dict(R_IN=R_IN, wall=BASE_WALL_MM, wheel_r=WHEEL_R, wheel_w=WHEEL_W, wheel_mount_r=WHEEL_MR, servo_zone=(SERVO_RAD, SERVO_TAN), assumption='thành thân 5 mm, servo bánh STS3215 trục hướng tâm 105-150 mm x 46 mm tiếp tuyến [giả định]; cột nâng ở mặt trên thân (z >= 170), bu-lông chân cột không xuống dưới tấm trên')
    R['louver_vao'] = louver_chevron_check()
    R['ntc'] = dict(ntc1='giữa pack, mặt trên cell giữa hàng trên', ntc2='dán vùng FET BMS đầu -x (cọc P+), cách cọc P+ ~10 mm', ghi_chu='BMS cần >= 2 kênh NTC (QC2 V2.5); NTC dán có lớp cách điện, không bắt lên cọc mang điện')
    R['thermal'] = thermal()
    R['loads'] = loads(tot)
    # --- xuất
    export_step(body, f"{out}/M1-PW-013_battery_box.step")
    export_step(lid, f"{out}/M1-PW-014_battery_box_lid.step")
    export_step(bay_lid, f"{out}/M1-PW-015_battery_box_bay_lid.step")
    asm = [body, Pos(0, 0, BODY_H) * lid, Pos(0, 0, BODY_H) * bay_lid] + list(comp.values())
    if fit and fit['pack']:
        t1 = to_base(fit['pack']) * Pos(-OUT[0] / 2, -OUT[1] / 2, 0)
        parts_base = [t1 * s for s in asm]
        # đĩa đế Ø390 dày 4, bánh + servo bánh giả (kiểm thị giác), PCB+E-stop, K1
        disk = Pos(0, 0, BASE_Z0) * Cylinder(R_IN, T_DECK, align=(Align.CENTER, Align.CENTER, Align.MIN))
        parts_base.append(disk)
        for a in WHEEL_ANG:
            ar = math.radians(a)
            parts_base.append(Pos(WHEEL_MR * math.cos(ar), WHEEL_MR * math.sin(ar), WHEEL_R) * Rot(Z=a) * Rot(Y=90) * Cylinder(WHEEL_R, WHEEL_W, align=(Align.CENTER, Align.CENTER, Align.CENTER)))
            rc = (SERVO_RAD[0] + SERVO_RAD[1]) / 2
            parts_base.append(Pos(rc * math.cos(ar), rc * math.sin(ar), WHEEL_R) * Rot(Z=a) * Box(SERVO_RAD[1] - SERVO_RAD[0], SERVO_TAN, SERVO_H))
        if fit['pcb']:
            cx2, cy2, w2, h2, th2 = fit['pcb']
            parts_base.append(Pos(cx2, cy2, BASE_Z0 + T_DECK) * Rot(Z=th2) * Box(w2, h2, WB_H, align=(Align.CENTER, Align.CENTER, Align.MIN)))   # bảng đi dây EVT (hộp bao)
        if fit['k1']:
            cx3, cy3, w3, h3, th3 = fit['k1']
            parts_base.append(Pos(cx3, cy3, BASE_Z0 + T_DECK) * Rot(Z=th3) * Box(w3, h3, K1[2], align=(Align.CENTER, Align.CENTER, Align.MIN)))
        export_step(Compound(parts_base), f"{out}/M1-PW-016_battery_box_assembly.step")
    json.dump(R, open(f"{out}/M1-PW-016_battery_box_check.json", "w"), indent=1, default=str)
    print(json.dumps({k: R[k] for k in R if k not in ('mass_g',)}, indent=1, default=str))


def thermal():
    """Hai nút: pack (C 1050 J/K) và khối BMS+F1+D0 (C ~ 250 J/K), nối nhau qua vách (R_pb 20 K/W [giả định]); mỗi nút có Rth ra môi trường.
    Mọi thông số [giả định], chưa đo."""
    I = 49.8; n_par = 5; i_cell = I / n_par
    C_pack, C_els = 15 * 70.0, 250.0
    Ah = 4.0 * n_par * 0.85; t_run = Ah / I * 3600
    A_out = 2 * (123 * 90 + 123 * 78.6 + 90 * 78.6) / 1e6 - 123 * 90 / 1e6
    A_in = 0.030
    R_closed = 0.002 / (0.04 * A_in) + 0.001 / (0.026 * A_in) + 0.003 / (0.20 * A_out) + 1 / (9.0 * A_out)
    R_els = 4.2                                  # khoang BMS: QC2 0,0276 m2, h 10 [QC2, tính tay]
    R_pb = 20.0
    # nhiệt BMS/F1/D0 (QC2: 12-18 W): BMS FET 2-4 mOhm; F1 2,0 mOhm (tinh-toan.md); D0 0,5 mOhm; thanh/cos ~1,5 W
    els = {'low': 49.8 ** 2 * (0.002 + 0.002 + 0.0005) + 1.0, 'high': 49.8 ** 2 * (0.004 + 0.002 + 0.0005) + 1.5}
    res = dict(I_A=I, t_full_discharge_min=round(t_run / 60, 1), R_pack_ambient_K_per_W=round(R_closed, 2), R_els_ambient_K_per_W=R_els, R_pack_els_K_per_W=R_pb,
               Q_els_W={k: round(v, 1) for k, v in els.items()})
    cases = {}
    for name, r in (('12 mΩ AC (datasheet tentative)', 12.0), ('18 mΩ DC (1,5 x AC, giả định)', 18.0), ('20 mΩ (tinh_toan.py R_cell)', 20.0)):
        Qp = 15 * i_cell ** 2 * r * 1e-3 * 1.15
        for qe_name, Qe in els.items():
            dt = 1.0; Tp = Te = 0.0; t = 0.0; t55 = None
            while t < t_run:
                flow = (Te - Tp) / R_pb
                Tp += (Qp + flow - Tp / R_closed) / C_pack * dt
                Te += (Qe - flow - Te / R_els) / C_els * dt
                t += dt
                if t55 is None and Tp + 25.0 >= 55.0:
                    t55 = t
            cases[f'{name} | BMS+F1+D0 {qe_name}'] = dict(Q_pack_W=round(Qp, 1), dT_pack_end_K=round(Tp, 1), dT_els_end_K=round(Te, 1),
                                                          T_pack_end_at_25C=round(Tp + 25, 1), min_to_55C_from_25C=None if t55 is None else round(t55 / 60, 1))
    res['cases'] = cases
    # ngưỡng môi trường tối đa để cuối xả 49,8 A <= 55 C (cắt trước 60 C)
    mx = {}
    for name, r in (('12 mΩ', 12.0), ('18 mΩ', 18.0), ('20 mΩ', 20.0)):
        row = cases[f'{[k for k in cases if k.startswith(name[:2])][0].split(" |")[0]} | BMS+F1+D0 high']
        mx[name] = round(55.0 - row['dT_pack_end_K'], 1)
    res['T_ambient_max_for_55C_end_of_discharge'] = mx
    return res


def loads(m_g):
    m = m_g / 1000.0; g = 9.81
    out = {}
    for lab, a in (('rung/vận hành 50 g', 50), ('sốc UN38.3 T4 pack nhỏ 150 g [chưa kiểm]', 150)):
        F = m * a * g
        out[lab] = dict(F_N=round(F), per_bolt_4xM5_N=round(F / 4))
    out['rơi 1 m'] = dict(v_m_s=round(math.sqrt(2 * g), 2), E_J=round(0.5 * m * 2 * g, 1))
    # ống lót thép OD10 x dày 4 mm (ép lỗ lên PC-ABS tai): ứng suất ép = F_bolt / (OD x t)
    for lab, a in (('50 g', 50), ('150 g', 150)):
        Fb = m * a * g / 4
        out[f'ép lỗ tai, ống lót OD10 x 4 mm @ {lab} (MPa)'] = round(Fb / (EAR_BUSH_OD * EAR_T), 1)
        out[f'ép lỗ tai, không ống lót d5,5 x 4 mm @ {lab} (MPa)'] = round(Fb / (5.5 * EAR_T), 1)
    return out


if __name__ == '__main__':
    main()
