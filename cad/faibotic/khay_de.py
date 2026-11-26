# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Khay đế M1: sàn thiết bị cho laptop HOẶC Jetson + phụ kiện. Yêu cầu: docs/M1-TRAM/R5-khay-de.md.

Thay tấm khay M1-ST-010 (giữ cột đỡ in M1-ST-011, cắt ngắn để đỡ thanh 2040). Mọi số tham chiếu chung với đế:
mặt sàn đế = assembly_pl.base_heights()["deck_top"] (64 mm), chân trụ spec.COLUMN_FOOT_XR, mặt khay spec.TRAY_POS (giữ 210 mm như khay cũ).

Cấu tạo (mm, hệ robot W: x trước, y trái, z lên):
  - Tấm sàn PETG da 2,4 mm, mặt trên z 210; x 64..320, y ±190 (spec.KHAY_DE_X, KHAY_DE_Y). Gờ 4 cạnh cao 10 mm, dày 3 mm, bo góc R8.
  - Gân dưới: 5 gân dọc x (y 0, ±85, ±170) và 3 gân ngang y (x 100, 230, 300), cao 14 mm, dày 3 mm; gân khuyết chỗ thanh 2040.
  - Thanh nhôm 2040 dài 200 mm nằm ngang theo y dưới da, x 130..170 (mặt 40 nằm ngang), bắt 2 ốc M5 vào cột M1-ST-011.
    Khay đặt lên, 2 vấu định vị ôm hai mặt bên thanh, vặn 2 núm M5 + đai ốc T (y ±70): không dụng cụ, <= 30 s.
  - Khe thông gió dưới vùng laptop (x 80..300, y ±165): rãnh 6 x 50 mm, tỷ lệ hở >= 30% vùng đó.
  - Vùng Jetson tâm (150; 0): lỗ VESA 75 và 100 (heat-set M4), mẫu lỗ devkit Orin Nano 86 x 58 [CẦN-XÁC-MINH theo bản vẽ hãng] (heat-set M3),
    ô quạt 80 mm xoay 45° (lỗ Ø76, vít M3 khoảng 71,5) để quạt hút gió từ dưới khay.
  - 2 cặp khe dây đai 30 x 4 mm (x 105 và 265, sát gờ bên), 2 khe đi dây 40 x 12 mm ở mép sau (y ±60), 6 tai lạt cáp.
  - Mặt chống trượt: 4 dải cao su silicone tự dính 2 mm (không in).
  - Chia mảnh vừa bàn in BED (mặc định 220 x 220, chừa 10 mm): mộng đuôi én dọc mối ghép + vít M3 qua gân.

Chạy:
  python3 khay_de.py              -> in bảng tính (khối lượng, võng, tỷ lệ hở, mảnh in) [chỉ cần numpy]
  python3 khay_de.py --bed 180    -> chia mảnh cho bàn 180 x 180 (Bambu A1 mini)
  python3 khay_de.py --xuat       -> STEP + STL từng mảnh vào cad/export/khay-de/ (cần build123d, chạy trên máy CAD)
"""
from __future__ import annotations

import argparse
import functools
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.join(HERE, "..", "export", "khay-de")
sys.path.insert(0, os.path.join(HERE, "..", "..", "sdk"))
from faibotic import spec  # noqa: E402

# ---------------- tham số (mm) ----------------
X0, X1 = spec.KHAY_DE_X[0] * 1000, spec.KHAY_DE_X[1] * 1000
YH = spec.KHAY_DE_Y * 1000
TOP = 210.0                     # mặt da trên (= mặt khay cũ M1-ST-010, assembly_pl.TRAY["top"])
DECK_TOP_DE = 64.0              # mặt sàn đế (assembly_pl.base_heights()["deck_top"]); kiểm lại khi có build123d
SKIN = 2.4
LIP_H, LIP_T, CORNER_R = 10.0, 3.0, 8.0
RIB_H, RIB_T = 14.0, 3.0
RIBS_Y = (0.0, 85.0, -85.0, 170.0, -170.0)      # gân dọc x
RIBS_X = (100.0, 230.0, 300.0)                  # gân ngang y
RAIL_X = (130.0, 170.0)                         # thanh 2040 (mặt 40 theo x), dài RAIL_L theo y
RAIL_L = 190.0                                  # cắt từ phần dư thanh T05a (1,2 m cắt 1,0 m)
RAIL_Z = (TOP - SKIN - 20.0, TOP - SKIN)        # 187,6..207,6
KNOB_Y = (70.0, -70.0)                          # 2 núm M5 + đai ốc T trong rãnh trên của 2040
VENT_ZONE = ((80.0, 300.0), (-165.0, 165.0))    # vùng thông gió dưới laptop
VENT_W, VENT_L, VENT_PITCH_X, VENT_PITCH_Y = 6.0, 54.0, 10.0, 60.0
JET_C = (150.0, 0.0)
VESA = (75.0, 100.0)
ORIN_NANO_HOLES = (86.0, 58.0)                  # [CẦN-XÁC-MINH] khoảng lỗ carrier devkit
FAN_HOLE, FAN_SCREW = 76.0, 71.5                # quạt 80 mm, xoay 45°
STRAP_X, STRAP_SLOT = (105.0, 265.0), (30.0, 4.0)
CABLE_SLOTS = [((72.0, 84.0), (20.0, 60.0)), ((72.0, 84.0), (-60.0, -20.0))]
RHO_PETG = 1.27e-3              # g/mm³
E_PETG_IN = 1500.0              # MPa, PETG in 4 vách [giả định, thận trọng; datasheet khối 2000-2100]
E_AL = 69000.0
G = 9.81
TEST_KG = 1.5 * spec.KHAY_DE_TAI_MAX            # 4,5 kg
FLOW_G_H = 45.0                 # g/h: như M1-ST-010 trong product/danh-sach-in.md (304,9 g / 6,7 h) [ước], chờ slicer
# Mối ghép (bàn 220): cắt theo hai mặt gân để gân nguyên về một mảnh: x = 231,5 (mặt phải gân x 230), y = 1,5 (mặt trên gân y 0)
SEAM_X = RIBS_X[1] + RIB_T / 2
SEAM_Y = RIB_T / 2
BOSS_H = RIB_H                  # ụ ghép cao bằng gân, treo dưới da
BOSS_A, BOSS_B, BOSS_L = 7.0, 11.0, 16.0     # sâu về phía mảnh A, về phía mảnh B (chứa ổ mộng), dài dọc mối
TAIL_D, TAIL_W0, TAIL_W1 = 5.0, 7.0, 11.0    # mộng đuôi én: sâu 5 mm, cổ 7, đầu 11 (góc ~22°)
TAIL_CLR = 0.2                  # khe ổ mộng mỗi bên
M3_CLEAR, M3_PILOT, M3_CBORE = 3.4, 2.7, 6.2
SEAM_X_BOSS_Y = (-150.0, -60.0, 60.0, 150.0)  # vị trí ụ dọc mối x (tránh gân y và khe gió)
SEAM_Y_BOSS_X = (100.0, 200.0, 308.0)         # vị trí ụ dọc mối y (vùng Jetson/ngoài khe gió)
POST_OLD_TOP = 205.0            # đỉnh cột M1-ST-011 hiện tại (assembly_pl.tray_parts: z0 = top - t)


def boss_boxes():
    """Hộp (x0, x1, y0, y1) của các ụ ghép dưới da."""
    out = [(SEAM_X - BOSS_A, SEAM_X + BOSS_B, y - BOSS_L / 2, y + BOSS_L / 2) for y in SEAM_X_BOSS_Y]
    out += [(x - BOSS_L / 2, x + BOSS_L / 2, SEAM_Y - BOSS_A, SEAM_Y + BOSS_B) for x in SEAM_Y_BOSS_X]
    return out


def vent_slots():
    """Danh sách rãnh (x0, x1, y0, y1), tránh vùng Jetson, thanh 2040, gân."""
    (vx0, vx1), (vy0, vy1) = VENT_ZONE
    out = []
    y = vy0
    while y + VENT_L <= vy1 + 1e-9:
        x = vx0
        while x + VENT_W <= vx1 + 1e-9:
            c = (x + VENT_W / 2, y + VENT_L / 2)
            in_rail = RAIL_X[0] - 4 <= c[0] <= RAIL_X[1] + 4
            in_rib = any(abs(c[0] - rx) < VENT_W / 2 + RIB_T for rx in RIBS_X)
            in_jet = abs(c[0] - JET_C[0]) < 60 and abs(c[1] - JET_C[1]) < 60
            in_boss = any(a0 - 0.5 < x + VENT_W and x - 0.5 < a1 and b0 - 0.5 < y + VENT_L and y - 0.5 < b1 for a0, a1, b0, b1 in boss_boxes())
            if not (in_rail or in_rib or in_jet or in_boss):
                out.append((x, x + VENT_W, y, y + VENT_L))
            x += VENT_PITCH_X
        y += VENT_PITCH_Y
    return out


def open_area():
    (vx0, vx1), (vy0, vy1) = VENT_ZONE
    zone = (vx1 - vx0) * (vy1 - vy0)
    slots = sum((a1 - a0) * (b1 - b0) for a0, a1, b0, b1 in vent_slots())
    fan = math.pi * (FAN_HOLE / 2) ** 2
    return (slots + fan) / zone, slots + fan


def mass_g():
    """Khối lượng ước (g): chi tiết mỏng <= 4 vách nên coi đặc; gyroid chỉ ở mộng và vấu."""
    L, W = X1 - X0, 2 * YH
    skin = (L * W - open_area()[1]) * SKIN
    lips = 2 * (L + W) * LIP_H * LIP_T
    ribs = len(RIBS_Y) * (L - (RAIL_X[1] - RAIL_X[0])) * RIB_H * RIB_T + len(RIBS_X) * W * RIB_H * RIB_T
    lugs = 4 * 40 * 12 * 6 + 2 * 30 * 30 * 6        # vấu định vị 2040 + ụ núm
    dovetail = 0.25 * 2 * (L + W) * RIB_H * 6        # mộng ghép (25% gyroid)
    return RHO_PETG * (skin + lips + ribs + lugs + dovetail)


def section_I(width: float, n_ribs: int, n_lips: int, skin_frac: float):
    """Mô-men quán tính (mm⁴) tiết diện chữ T ghép: da (width × skin_frac) + gân dưới + gờ trên. Trục trung hoà tính theo diện tích."""
    parts = [(width * skin_frac * SKIN, SKIN / 2, width * skin_frac * SKIN ** 3 / 12)]
    parts += [(RIB_T * RIB_H, -RIB_H / 2, RIB_T * RIB_H ** 3 / 12)] * n_ribs
    parts += [(LIP_T * LIP_H, SKIN + LIP_H / 2, LIP_T * LIP_H ** 3 / 12)] * n_lips
    A = sum(a for a, _, _ in parts)
    zc = sum(a * z for a, z, _ in parts) / A
    return sum(i + a * (z - zc) ** 2 for a, z, i in parts)


def deflection():
    """Võng (mm) dưới tải thử TEST_KG phân bố đều. Mô hình dầm công-xôn tải đều: δ = Q·L³ / (8·E·I).
    (1) công-xôn trước theo x từ mặt trước thanh 2040 tới mép trước; (2) công-xôn bên theo y từ đầu thanh 2040 tới mép bên;
    góc trước-bên ≈ (1) + (2). Bỏ qua da nhịp hai chiều (thận trọng), bỏ qua võng thanh 2040 và cột (tính riêng: nhỏ)."""
    L, W = X1 - X0, 2 * YH
    q_area = TEST_KG * G / (L * W)                               # N/mm²
    Lx = X1 - RAIL_X[1]
    Qx = q_area * Lx * W
    Ix = section_I(W, len(RIBS_Y), 2, 0.65)                      # da trừ 35% khe (thận trọng)
    dx = Qx * Lx ** 3 / (8 * E_PETG_IN * Ix)
    Ly = YH - RAIL_L / 2
    Qy = q_area * Ly * L
    Iy = section_I(L, len(RIBS_X), 2, 0.65)
    dy = Qy * Ly ** 3 / (8 * E_PETG_IN * Iy)
    # thanh 2040 công-xôn ngoài cột (cột rộng 60 theo y): I theo phương đứng (cạnh 20 đứng) ~ 0,70 cm⁴ [catalog 2040 phổ biến, ước]
    Lr = RAIL_L / 2 - 30
    dr = (TEST_KG * G / 2) * Lr ** 3 / (3 * E_AL * 0.70e4)
    return dict(truoc=dx, ben=dy, goc=dx + dy, thanh_2040=dr, Ix=Ix, Iy=Iy, Lx=Lx, Ly=Ly)


def plate_alt(t: float, E: float, rho: float, lips_bent: bool):
    """Phương án tấm cắt laser cùng khuôn: võng công-xôn trước + bên, khối lượng (g)."""
    L, W = X1 - X0, 2 * YH
    q_area = TEST_KG * G / (L * W)
    frac = 1 - open_area()[1] / (L * W)
    def I(width):
        parts = [(width * 0.65 * t, t / 2, width * 0.65 * t ** 3 / 12)]
        if lips_bent:                                            # gờ gập 10 mm ở hai mép song song phương uốn
            parts += [(t * LIP_H, t + LIP_H / 2, t * LIP_H ** 3 / 12)] * 2
        A = sum(a for a, _, _ in parts); zc = sum(a * z for a, z, _ in parts) / A
        return sum(i + a * (z - zc) ** 2 for a, z, i in parts)
    Lx, Ly = X1 - RAIL_X[1], YH - RAIL_L / 2
    d = q_area * Lx * W * Lx ** 3 / (8 * E * I(W)) + q_area * Ly * L * Ly ** 3 / (8 * E * I(L))
    m = rho * (L * W * frac * t + (2 * (L + W) * LIP_H * t if lips_bent else 0))
    return d, m


def pieces(bed: float = 220.0, margin: float = 10.0):
    """Số mảnh theo x, y để mỗi mảnh (gồm mộng 8 mm) nằm trong bàn in bed x bed."""
    lim = bed - margin
    if abs(bed - 220.0) < 1e-6 and margin == 10.0:      # mối ghép thật (build_parts): cạnh lớn nhất kể cả mộng
        return 2, 2, max(SEAM_X + TAIL_D - X0, X1 - SEAM_X + BOSS_B), max(SEAM_Y + YH + 0.0, YH - SEAM_Y) + 8
    nx = math.ceil((X1 - X0 + 8) / lim)
    ny = math.ceil((2 * YH + 8) / lim)
    return nx, ny, (X1 - X0) / nx + 8, 2 * YH / ny + 8


def lift_clear():
    """Khe đứng (mm) từ đỉnh vật tới đáy thấp nhất xe nâng ở lift 0 (spec.TMID_Z)."""
    z_low = (spec.ARM_BASE_Z0 + spec.TMID_Z[0]) * 1000
    return z_low, {k: z_low - (TOP + h) for k, h in (("laptop gập 25", 25.0), ("Jetson AGX 80", 80.0), ("laptop mở nắp 260", 260.0))}


def report(bed: float = 220.0) -> int:
    oa, _ = open_area()
    m = mass_g()
    d = deflection()
    nx, ny, px, py = pieces(bed)
    print(f"Khay đế: x {X0:.0f}..{X1:.0f}, y ±{YH:.0f}, mặt {TOP:.0f} mm; gờ {LIP_H:.0f} mm; thanh 2040 {RAIL_L:.0f} mm tại x {RAIL_X[0]:.0f}..{RAIL_X[1]:.0f}")
    print(f"Khe sau tới chân trụ (x {spec.COLUMN_FOOT_XR[1] * 1000:.1f}): {X0 - spec.COLUMN_FOOT_XR[1] * 1000:.1f} mm")
    print(f"Tỷ lệ hở vùng laptop: {100 * oa:.1f}% ({len(vent_slots())} rãnh + ô quạt) [tính]")
    print(f"Khối lượng khay in: {m:.0f} g [ước]; giờ in ~{m / FLOW_G_H:.0f} h ở {FLOW_G_H:.0f} g/h [ước]")
    print(f"Võng ở {TEST_KG:.1f} kg phân bố, E {E_PETG_IN:.0f} MPa [tính]: công-xôn trước {d['truoc']:.2f} mm (L {d['Lx']:.0f}), "
          f"bên {d['ben']:.2f} mm (L {d['Ly']:.0f}), góc ~{d['goc']:.2f} mm; thanh 2040 {d['thanh_2040']:.3f} mm")
    print(f"Ma sát cần để laptop không trượt khi phanh 0,5 m/s²: μ >= {0.5 / G:.3f}; cao su silicone ~0,5 [ước]")
    for ten, t, E, rho, lips in (("Nhôm 2 mm gập gờ", 2.0, E_AL, 2.68e-3, True), ("Nhôm 3 mm gập gờ", 3.0, E_AL, 2.68e-3, True),
                                 ("Mica 3 mm phẳng", 3.0, 3200.0, 1.19e-3, False), ("Mica 5 mm phẳng", 5.0, 3200.0, 1.19e-3, False)):
        dd, mm_ = plate_alt(t, E, rho, lips)
        print(f"  {ten}: võng góc ~{dd:.2f} mm, {mm_:.0f} g (+ gờ dán nếu phẳng) [tính]")
    print(f"Chia mảnh bàn {bed:.0f}: {nx} x {ny} = {nx * ny} mảnh, mỗi mảnh ~{px:.0f} x {py:.0f} mm")
    z_low, cl = lift_clear()
    print(f"Đáy xe nâng ở lift 0: z {z_low:.1f} mm; khe tới đỉnh vật: " + ", ".join(f"{k}: {v:.0f} mm" for k, v in cl.items()))
    print(f"Cột M1-ST-011: cắt ngắn từ đỉnh {POST_OLD_TOP:.0f} xuống {RAIL_Z[0]:.1f} mm (đỡ thanh 2040)")
    return 0


# ---------------- hình học (build123d, máy CAD) ----------------
def _post_rail():
    from build123d import Align, Box, Pos
    MIN = (Align.MIN,) * 3
    return [("M1-ST-011_cot_do_cat", Pos(122.0, -30.0, DECK_TOP_DE) * Box(55.0, 60.0, RAIL_Z[0] - DECK_TOP_DE, align=MIN)),
            ("2040_200_tham_chieu", Pos(RAIL_X[0], -RAIL_L / 2, RAIL_Z[0]) * Box(RAIL_X[1] - RAIL_X[0], RAIL_L, 20.0, align=MIN))]


@functools.lru_cache(maxsize=None)
def build_parts(display: bool = False):
    """Trả [(tên, shape)]: các mảnh khay đã chia theo pieces(), cột đỡ cắt ngắn, thanh 2040 (tham chiếu). Chưa chạy trên máy không có build123d.
    display=True: hình hiển thị cho sim/GLB (chỉ da + gờ, 4 mảnh cách nhau 0,3 mm, không gân, khe, lỗ, mộng) để mỗi mảnh <= 160 tam giác
    và không bị rút tam giác làm vỡ tấm mỏng; khối lượng, STL in và PDF dùng bản đủ chi tiết (display=False)."""
    from build123d import Align, Box, Cylinder, Pos, Rot
    MIN = (Align.MIN,) * 3
    L, W = X1 - X0, 2 * YH
    deck = Pos(X0, -YH, TOP - SKIN) * Box(L, W, SKIN, align=MIN)
    lip = Pos(X0, -YH, TOP) * Box(L, W, LIP_H, align=MIN) - Pos(X0 + LIP_T, -YH + LIP_T, TOP) * Box(L - 2 * LIP_T, W - 2 * LIP_T, LIP_H, align=MIN)
    body = deck + lip
    if display:
        g = 0.15
        xs, ys = (X0 - 1, SEAM_X, X1 + 1), (-YH - 1, SEAM_Y, YH + 1)
        out = []
        for i in range(2):
            for j in range(2):
                xa, xb = xs[i] + (g if i else 0), xs[i + 1] - (0 if i else g)
                ya, yb = ys[j] + (g if j else 0), ys[j + 1] - (0 if j else g)
                out.append((f"M1-ST-012_khay_de_{i}{j}", body & (Pos(xa, ya, TOP - SKIN - 1) * Box(xb - xa, yb - ya, SKIN + LIP_H + 2, align=MIN))))
        return out + _post_rail()
    for y in RIBS_Y:
        for xa, xb in ((X0 + LIP_T, RAIL_X[0]), (RAIL_X[1], X1 - LIP_T)):
            body += Pos(xa, y - RIB_T / 2, TOP - SKIN - RIB_H) * Box(xb - xa, RIB_T, RIB_H, align=MIN)
    for x in RIBS_X:
        body += Pos(x - RIB_T / 2, -YH + LIP_T, TOP - SKIN - RIB_H) * Box(RIB_T, W - 2 * LIP_T, RIB_H, align=MIN)
    for xa in (RAIL_X[0] - 6, RAIL_X[1]):          # vấu định vị ôm thanh 2040
        for y in KNOB_Y:
            body += Pos(xa, y - 20, RAIL_Z[0] + 4) * Box(6, 40, RAIL_Z[1] - RAIL_Z[0] - 4, align=MIN)
    for x0, x1, y0, y1 in vent_slots():
        body -= Pos(x0, y0, TOP - SKIN - 1) * Box(x1 - x0, y1 - y0, SKIN + 2, align=MIN)
    jx, jy = JET_C
    body -= Pos(jx, jy, TOP - SKIN - RIB_H - 1) * Cylinder(FAN_HOLE / 2, SKIN + RIB_H + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    a = FAN_SCREW / 2 * math.sqrt(2)
    holes = [(jx + a, jy, 1.6), (jx - a, jy, 1.6), (jx, jy + a, 1.6), (jx, jy - a, 1.6)]
    for v in VESA:
        holes += [(jx + sx * v / 2, jy + sy * v / 2, 2.8) for sx in (-1, 1) for sy in (-1, 1)]       # lỗ heat-set M4 Ø5,6
    holes += [(jx + sx * ORIN_NANO_HOLES[0] / 2, jy + sy * ORIN_NANO_HOLES[1] / 2, 2.0) for sx in (-1, 1) for sy in (-1, 1)]  # heat-set M3 Ø4
    holes += [(x, y, 2.75) for x in ((RAIL_X[0] + RAIL_X[1]) / 2,) for y in KNOB_Y]                 # lỗ núm M5
    for x, y, r in holes:
        body -= Pos(x, y, TOP - SKIN - 1) * Cylinder(r, SKIN + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for sx in STRAP_X:
        for sy in (1, -1):
            yc = sy * (YH - LIP_T - 6)
            body -= Pos(sx - STRAP_SLOT[0] / 2, yc - STRAP_SLOT[1] / 2, TOP - SKIN - 1) * Box(*STRAP_SLOT, SKIN + 2, align=MIN)
    for (x0, x1), (y0, y1) in CABLE_SLOTS:
        body -= Pos(x0, y0, TOP - SKIN - 1) * Box(x1 - x0, y1 - y0, SKIN + 2, align=MIN)
    # ụ ghép + mộng đuôi én + vít M3 (vẽ trước khi chia)
    zb = TOP - SKIN - BOSS_H
    seams = [("x", SEAM_X, y) for y in SEAM_X_BOSS_Y] + [("y", SEAM_Y, x) for x in SEAM_Y_BOSS_X]

    def frame(kind, c, v):
        """(u, v) -> (x, y): u vuông góc mối từ mảnh A sang B (u = 0 ở mối), v dọc mối."""
        return (lambda u, w: (c + u, v + w)) if kind == "x" else (lambda u, w: (v + w, c + u))

    def prism(pts, z0, h):
        from build123d import Polygon, extrude
        sol = extrude(Polygon(*pts, align=None), amount=h)      # hướng đùn phụ thuộc chiều quay đa giác: chuẩn hoá về z0..z0+h
        return Pos(0, 0, z0 - sol.bounding_box().min.Z) * sol

    def tail_pts(kind, c, v, grow=0.0):
        f = frame(kind, c, v)
        pts = [(-2.0, -TAIL_W0 / 2), (0.0, -TAIL_W0 / 2), (TAIL_D + grow, -TAIL_W1 / 2 - grow), (TAIL_D + grow, TAIL_W1 / 2 + grow), (0.0, TAIL_W0 / 2), (-2.0, TAIL_W0 / 2)]
        if grow:
            pts = [(-2.0, -TAIL_W0 / 2 - grow), (0.0, -TAIL_W0 / 2 - grow), (TAIL_D + grow, -TAIL_W1 / 2 - grow), (TAIL_D + grow, TAIL_W1 / 2 + grow), (0.0, TAIL_W0 / 2 + grow), (-2.0, TAIL_W0 / 2 + grow)]
        return [f(u, w) for u, w in pts]

    def box_uv(kind, c, v, u0, u1, w0, w1, z0, h):
        f = frame(kind, c, v)
        (xa, ya), (xb, yb) = f(u0, w0), f(u1, w1)
        return Pos(min(xa, xb), min(ya, yb), z0) * Box(abs(xb - xa), abs(yb - ya), h, align=MIN)

    def cyl_u(kind, c, v, u0, u1, r):
        f = frame(kind, c, v)
        (xa, ya), (xb, yb) = f(u0, 0.0), f(u1, 0.0)
        zc = zb + BOSS_H / 2
        if kind == "x":
            return Pos((xa + xb) / 2, ya, zc) * (Rot(0, 90, 0) * Cylinder(r, abs(xb - xa)))
        return Pos(xa, (ya + yb) / 2, zc) * (Rot(90, 0, 0) * Cylinder(r, abs(yb - ya)))

    for kind, c, v in seams:
        body += box_uv(kind, c, v, -BOSS_A, BOSS_B, -BOSS_L / 2, BOSS_L / 2, zb, BOSS_H)
    nx, ny, _, _ = pieces()
    xs, ys = (X0 - 1, SEAM_X, X1 + 1), (-YH - 1, SEAM_Y, YH + 1)

    def region(i, j):
        return Pos(xs[i], ys[j], TOP - SKIN - RIB_H - 2) * Box(xs[i + 1] - xs[i], ys[j + 1] - ys[j], SKIN + RIB_H + LIP_H + 4, align=MIN)

    def piece_at(x, y):
        return (0 if x < SEAM_X else 1), (0 if y < SEAM_Y else 1)

    pcs = {(i, j): body & region(i, j) for i in range(2) for j in range(2)}
    for kind, c, v in seams:
        f = frame(kind, c, v)
        pa, pb = piece_at(*f(-1.0, 0.0)), piece_at(*f(1.0, 0.0))
        tail = prism(tail_pts(kind, c, v), zb, BOSS_H)
        sock = prism(tail_pts(kind, c, v, TAIL_CLR), zb - 0.2, BOSS_H + 0.2) & box_uv(kind, c, v, 0.0, TAIL_D + 1.0, -TAIL_W1, TAIL_W1, zb - 1, BOSS_H + 2)
        pcs[pa] = pcs[pa] + tail - cyl_u(kind, c, v, -BOSS_A - 1, TAIL_D + 1, M3_CLEAR / 2) - cyl_u(kind, c, v, -BOSS_A - 1, -BOSS_A + 2.0, M3_CBORE / 2)
        pcs[pb] = pcs[pb] - sock - cyl_u(kind, c, v, TAIL_D + TAIL_CLR - 0.5, BOSS_B - 1.0, M3_PILOT / 2)
    out = [(f"M1-ST-012_khay_de_{i}{j}", pcs[(i, j)]) for i in range(2) for j in range(2)]
    # chốt ghép (mộng + vít) nằm trên mối x = 231,5 (4 ụ) và y = 1,5 (3 ụ)
    return out + _post_rail()


def export():
    from build123d import export_step, export_stl
    os.makedirs(EXP, exist_ok=True)
    for n, s in build_parts():
        export_step(s, os.path.join(EXP, n + ".step"))
        export_stl(s, os.path.join(EXP, n + ".stl"))
        print("ghi", n)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--bed", type=float, default=220.0)
    ap.add_argument("--xuat", action="store_true")
    a = ap.parse_args()
    if a.xuat:
        export()
    raise SystemExit(report(a.bed))
