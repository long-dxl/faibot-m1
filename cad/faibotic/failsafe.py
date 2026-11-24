# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""v0.4: chặn mất điện vai (CHẶN 2 của QC2 v0.3). build123d.
Đổi so với balance.py: (1) thanh chặn M1-SH-016 bằng nhôm 6061 8 mm thay thanh PETG 6 mm, dời lên bán kính ~90 mm
(z 80..100, trước đây z 40..56) để lực chặn trên cùng mô-men nhỏ hơn ~2 lần; (2) đệm PU M1-SH-015 dời để link chạm đệm
từ -85,5 độ (chân đệm), ép hết cỡ ở -88 độ (khe cradle-gá khuỷu còn sạch tới khuỷu 130, xem failsafe_scan.json);
(3) quét giao cắt LƯỚI CƠ KHÍ (không dựa giới hạn mềm): vai -96..+120, khuỷu -60..150.
Chạy: python failsafe.py [--scan]  -> ../export/failsafe_scan.json, M1-SH-016_stop_bar_alu.step, M1-SH-015_stop_pad_v04.step"""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from build123d import Box, Cylinder, Align, Polygon, extrude, Pos, Rot, Part, export_step
import balance as b
import shoulder as sh
from params import *

MIN = b.MIN
Q_CONTACT, Q_BOTTOM = -85.5, -88.0                   # độ: link chạm chân đệm / ép hết cỡ
BAR_Z0, BAR_Z1, BAR_X0, BAR_T = 80.0, 100.0, -32.0, 8.0   # thanh nhôm: x -32..-24, z 80..100
PAD_T, PAD_FRONT_X, PAD_W = 16.0, -8.0, 40.0          # đệm PU dày 16, mặt trước x = -8, rộng 40 (y +-20)
PAD_Z0, PAD_Z1 = 82.0, 98.0

# v0.4.3 (DFM): thanh chặn trên M1-SH-016 nằm GIỮA hai trụ neo (không chồng trụ), bắt 1 x M4 xuyên trụ theo trục y vào đầu thanh.
TOP_BAR_XF = -0.8                 # mặt trước đệm vai trên (find_top_xf trong failsafe_v042.py; test kiểm khớp)
TOP_BAR_T = 12.0                  # dày 12 (x): tâm lỗ M4 cách hai mặt 6 = 1,5d
TOP_BAR_X0 = TOP_BAR_XF - 20.0 - TOP_BAR_T
TOP_BAR_XM, TOP_BAR_ZM = TOP_BAR_X0 + TOP_BAR_T / 2, (BAR_Z0 + BAR_Z1) / 2
FLANGE_HOLES_X = (-94.0, -72.0)   # M1-SH-017 v3: 4 x M4 xuyên chân gá vai (y +-8): khớp failsafe_v042_low.FLANGE_HOLES_X
FOOT_Z0 = -CRADLE_H / 2

def stop_bar_alu() -> Part:
    """M1-SH-016: thanh chặn nhôm 6061-T6, nối hai trụ neo (y -(Y_OUT+POST_Y1)+3 .. +), 2 bu-lông M4 mỗi đầu vào trụ [giả định]."""
    w = 2 * (b.Y_OUT + b.POST_Y1) - 6
    return Pos(BAR_X0, -w / 2, BAR_Z0) * Box(BAR_T, w, BAR_Z1 - BAR_Z0, align=MIN)

def stop_pad_v04() -> Part:
    return Pos(PAD_FRONT_X - PAD_T, -PAD_W / 2, PAD_Z0) * Box(PAD_T, PAD_W, PAD_Z1 - PAD_Z0, align=MIN)

def cradle_v4() -> Part:
    """Gá vai v2 bỏ thanh chặn PETG cũ (thanh nhôm lắp rời, M1-SH-016)."""
    c = sh.cradle()
    for s in (-1, 1):
        y0 = b.Y_IN if s > 0 else -(b.Y_OUT + b.POST_Y1)
        c = c + Pos(b.X_A - b.POST_W / 2, y0, CRADLE_H / 2 - 1) * Box(b.POST_W, b.Y_OUT - b.Y_IN + b.POST_Y1, b.POST_TOP - CRADLE_H / 2 + 1, align=MIN)
        c = c - Pos(b.X_A, s * (b.Y_OUT + b.POST_Y1 / 2), b.H_A) * Rot(X=90) * b.Cylinder(b.PIN_HOLE / 2, 40, align=b.C)
        # v0.4.3: trụ nhô 8 mm ngoài má: vát 45 độ phía dưới (thêm nêm tam giác bám mặt ngoài má) để không có góc treo > 45 độ
        yo = s * b.Y_OUT; ye = s * (b.Y_OUT + b.POST_Y1); z1 = CRADLE_H / 2 - 1; d = b.POST_Y1
        wedge = Pos(b.X_A - b.POST_W / 2, 0, 0) * Rot(Y=90) * extrude(Polygon((-(z1 - d), yo), (-z1, ye), (-z1, yo), align=None), amount=b.POST_W)
        c = c + wedge
        # lỗ M4 (Ø4,5 xuyên trụ, trục y) cho đầu thanh M1-SH-016: một lỗ mỗi trụ
        c = c - Pos(TOP_BAR_XM, 0, TOP_BAR_ZM) * Rot(X=90) * Cylinder(2.25, 2 * (b.Y_OUT + b.POST_Y1) + 2, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    # 4 lỗ Ø4,5 xuyên chân (chân nhôm M1-SH-017 v3 bắt từ dưới lên, đai ốc DIN 985 mặt trên chân)
    for hx in FLANGE_HOLES_X:
        for hy in (-8.0, 8.0):
            c = c - Pos(hx, hy, FOOT_Z0) * Cylinder(2.25, FOOT_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return c

def pad_depth(q: float, pad) -> float:
    """Thể tích giao link - đệm (mm3) ở vai q: > 0 = đệm bị ép."""
    return b.inter(b.link_v2(q), pad)[0] or 0.0

def scan(shoulders, elbows, tag):
    cr, bar, pad = cradle_v4(), stop_bar_alu(), stop_pad_v04()
    sL, sR = sh.servo(-1), sh.servo(1); ea = b.elbow_assembly(); rows = []
    for qs in shoulders:
        link, pin = b.link_v2(qs), b.pin(qs)
        springs = [b.spring(qs, -1), b.spring(qs, 1)]
        for qe in elbows:
            loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
            hard, soft, parts = 0.0, 0.0, {}
            for en, eo in ea.items():
                e = loc * eo
                for n, o in (("cradle", cr), ("bar", bar), ("pin", pin), ("sL", sL), ("sR", sR), ("spring", springs[0]), ("spring", springs[1])) + ((("link", link),) if en == "e_cradle" else ()):
                    v, _ = b.inter(e, o); v = v or 0.0
                    if v: parts[f"{en}|{n}"] = round(parts.get(f"{en}|{n}", 0) + v, 1); hard += v
                v, _ = b.inter(e, pad); soft += v or 0.0
            # chi tiết cố định của chính cơ cấu vai (link v2, chốt, lò xo, đệm) với gá vai: không phụ thuộc khuỷu, ghi ở hàng khuỷu đầu tiên
            rows.append(dict(q_vai=qs, q_khuyu=qe, giao_cung_mm3=round(hard, 1), giao_dem_mm3=round(soft, 1), chi_tiet=parts))
        row_s = dict(kind="vai", q_vai=qs, link_dem_mm3=round(pad_depth(qs, pad), 1), link_bar=b.inter(link, bar)[0], link_cradle=b.inter(link, cr)[0], pin_cradle=b.inter(pin, cr)[0],
                     pin_bar=b.inter(pin, bar)[0], pin_servo=max(b.inter(pin, sL)[0] or 0, b.inter(pin, sR)[0] or 0))
        rows.append(row_s); print(row_s, flush=True)
    return rows

def main():
    full = "--scan" in sys.argv
    out = os.path.join(b.EXP, "failsafe_scan.json")
    pad = stop_pad_v04()
    # vị trí chạm đệm: quét mịn quanh -85,5..-88
    depth = {q: round(pad_depth(q, pad), 1) for q in (-84.0, -85.0, -85.5, -86.0, -87.0, -88.0, -89.0, -90.0)}
    print("link-đệm", depth, flush=True)
    export_step(stop_bar_alu(), os.path.join(b.EXP, "M1-SH-016_stop_bar_alu.step"))
    export_step(pad, os.path.join(b.EXP, "M1-SH-015_stop_pad_v04.step"))
    export_step(cradle_v4(), os.path.join(b.EXP, "M1-SH-003_cradle_v4.step"))
    if not full:
        return
    rows = scan(list(range(-92, -83, 1)), list(range(-60, 151, 10)), "min")
    rows += scan([-80, -60, -30, 0, 30, 60, 90, 100, 110, 120], list(range(-60, 151, 10)), "wide")
    json.dump(dict(pad_depth=depth, q_contact=Q_CONTACT, q_bottom=Q_BOTTOM, rows=rows), open(out, "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
