# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Cơ cấu cân bằng trọng lực vai (2 lò xo kéo + chốt M6 trên link + trụ neo liền gá) và gá khuỷu ghép đôi. build123d.
 Chạy: python balance.py  -> ../export/balance.json + STEP.
Mã: M1-SH-003 gá vai v2 (trụ neo liền), M1-SH-004 link v2, M1-SH-012 chốt M6x140 + đai ốc/chặn dọc trục (mua), M1-SH-013 lò xo (mua),
M1-SH-014 bu-lông neo M6 (mua), M1-AR-020 gá khuỷu ghép đôi (đối xứng gá vai, không trụ). Toạ độ: mặt phẳng x-z, q dương = link hạ xuống.
Lò xo mô hình hoá bằng ống Ø10 (khối bao) từ chốt neo A tới chốt B trên link, ở mặt phẳng y = +-BAL_Y."""
import json, math, os, sys, itertools
sys.path.insert(0, os.path.dirname(__file__))
from build123d import Box, Cylinder, Pos, Rot, Align, Part, Compound, export_step, mirror, Plane, fillet
import shoulder as sh
from params import *

HERE = os.path.dirname(os.path.abspath(__file__)); EXP = os.path.join(HERE, "../export")
MIN = (Align.MIN, Align.MIN, Align.MIN); C = (Align.CENTER,) * 3
# --- tham số (khớp sdk/faibotic/spec.py BAL_*) ---
A_B, H_A, X_A, Y_S = 35.0, 105.0, -25.0, 62.0      # chốt B cách trục vai; neo A cao, lùi; mặt phẳng lò xo
POST_W, POST_Y1, POST_TOP = 24.0, 8.0, 114.0         # bề rộng trụ (x), phần dày thêm ngoài má (y), đỉnh (z)
PIN_D, PIN_HOLE, PIN_L = 6.0, 6.2, 140.0            # M6x140, thân trơn tại link, đầu mũ một phía, ren phía kia [cấp 8.8: giả định]
SPR_D = 16.8                                         # Ø ngoài lò xo đặt theo thông số (d 1,4, D tb 15,4)
SPR_K, SPR_F0, SPR_L0 = 0.50, 5.1, 58.5             # N/mm, N, mm (tự tính, bao-cao-cad.md v0.3 mục 1b)
STOP_X0, STOP_BAR_T, STOP_PAD_T, STOP_Z0, STOP_Z1 = -42.0, 6.0, 16.0, 40.0, 56.0   # thanh chặn: x -42..-36, đệm x -36..-20
Y_IN = LINK_T / 2 + HORN_T + SERVO_W + CLEAR         # mặt trong má (37,9)
Y_OUT = Y_IN + CHEEK_T                               # mặt ngoài má (43,9)

def cradle_v2() -> Part:
    c = sh.cradle()
    for s in (-1, 1):
        y0 = Y_IN if s > 0 else -(Y_OUT + POST_Y1)
        post = Pos(X_A - POST_W / 2, y0, CRADLE_H / 2 - 1) * Box(POST_W, Y_OUT - Y_IN + POST_Y1, POST_TOP - CRADLE_H / 2 + 1, align=MIN)
        c = c + post
        c = c - Pos(X_A, s * (Y_OUT + POST_Y1 / 2), H_A) * Rot(X=90) * Cylinder(PIN_HOLE / 2, 40, align=C)
    # M1-SH-003: thanh chặn ngang nối hai trụ (chặn cứng hành trình lên của link vai), đệm M1-SH-015 gắn mặt trước
    bar = Pos(STOP_X0, -(Y_OUT + POST_Y1) + 3, STOP_Z0) * Box(STOP_BAR_T, 2 * (Y_OUT + POST_Y1) - 6, STOP_Z1 - STOP_Z0, align=MIN)
    return c + bar

def stop_pad() -> Part:
    """M1-SH-015: đệm chặn PU/cao su Shore 70A [giả định] dày 16 mm, dán lên thanh chặn."""
    return Pos(STOP_X0 + STOP_BAR_T, -10, STOP_Z0 + 2) * Box(STOP_PAD_T, 20, STOP_Z1 - STOP_Z0 - 4, align=MIN)

def link_v2(q: float = 0.0) -> Part:
    """Link v2: hub R18 hai đầu (đầu khuỷu có 4 x M3 PCD20 như hub vai) + lỗ Ø5,2 cho chốt cân bằng ở x = A_B."""
    plate = sh.link(0.0)
    end = Pos(LINK_LEN, -LINK_T / 2, 0) * Rot(X=-90) * Cylinder(HUB_R, LINK_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = plate + end
    for k in range(4):
        a = math.radians(45 + 90 * k)
        p = p - Pos(LINK_LEN + HORN_PCD / 2 * math.cos(a), -LINK_T / 2, HORN_PCD / 2 * math.sin(a)) * Rot(X=-90) * Cylinder(HORN_HOLE / 2, LINK_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = p - Pos(A_B, 0, 0) * Rot(X=90) * Cylinder(PIN_HOLE / 2, 20, align=C)
    # v0.4.3 (DFM): bo R1,5 bốn góc trong chỗ hub R18 gặp cổ 30 (dao Ø3); 2 lỗ M3 xuyên cho tab M1-AR-023
    xs = math.sqrt(HUB_R ** 2 - (LINK_NECK_H / 2) ** 2)
    sel = [e for e in p.edges() if abs(e.start_point().Y - e.end_point().Y) > LINK_T - 0.1 and any(abs(e.center().X - cx) < 0.5 and abs(abs(e.center().Z) - LINK_NECK_H / 2) < 0.5 for cx in (xs, LINK_LEN - xs))]
    assert len(sel) == 4, len(sel)
    p = fillet(sel, 1.5)
    # vòng 2: tab M1-AR-023 chữ U kẹp hai mặt, 2 x M3 xuyên link (Ø3,4 trục y, x 105,5, z +8 và -8: cách mép trên/dưới 7)
    # v0.4.5: lỗ dưới dời về x 91,5 (TAB_HOLES, params.py)
    for hx, hz in TAB_HOLES:
        p = p - Pos(hx, 0, hz) * Rot(X=90) * Cylinder(1.7, LINK_T + 2, align=C)
    return Rot(Y=q) * p

def pin(q: float) -> Part:
    return Rot(Y=q) * (Pos(A_B, 0, 0) * Rot(X=90) * Cylinder(PIN_D / 2, PIN_L, align=C))

def anchor_bolts() -> list:
    return [Pos(X_A, s * (Y_OUT + POST_Y1 + (PIN_L / 2 - (Y_OUT + POST_Y1 - 2 * 0)) / 2), H_A) * Rot(X=90) * Cylinder(PIN_D / 2, 2 * 0 + 0.1, align=C) for s in (-1, 1)] if False else \
        [Pos(X_A, s * (Y_OUT + POST_Y1 + 9.0), H_A) * Rot(X=90) * Cylinder(PIN_D / 2, 18.0, align=C) for s in (-1, 1)]

def spring(q: float, s: int) -> Part:
    B = (A_B * math.cos(math.radians(q)), -A_B * math.sin(math.radians(q)))
    dx, dz = B[0] - X_A, B[1] - H_A; L = math.hypot(dx, dz)
    th = math.degrees(math.atan2(dx, dz))
    return Pos((B[0] + X_A) / 2, s * Y_S, (B[1] + H_A) / 2) * Rot(Y=th) * Cylinder(SPR_D / 2, L, align=C)

def spring_len(q: float) -> float:
    return math.hypot(A_B * math.cos(math.radians(q)) - X_A, -A_B * math.sin(math.radians(q)) - H_A)

# v0.4.5: cầu chặn ngược trên M1-AR-020: tấm nối hai má, nằm trên mặt trên hai servo khuỷu (z 24..30, x -10..6 hệ khuỷu).
# Mép dưới-sau (-10; 24) trùng góc trên-sau servo nên chạm đệm chặn ngược M1-AR-023 ở cùng góc (-23,5) như trước; đệm dời vào giữa (|y| <= 6,5),
# ngoài dải y của servo (7,5..37,5), nên góc dưới-sau servo không còn quét qua đệm khi khuỷu 104..133 (vòng 2 QC2 v0.4.3: 18 mm3).
# Mặt sau vát từ (-10; 24) tới (-5; 30): điểm cao hơn của mặt sau (r tới 30,4) không chạm đệm trước góc (-10; 24) (bản mặt sau thẳng: chạm từ -17,5 độ).
BRIDGE_X0, BRIDGE_X1, BRIDGE_Z0, BRIDGE_Z1, BRIDGE_XT = -10.0, 6.0, SERVO_H / 2, 30.0, -5.0
def elbow_bridge() -> Part:
    p = Pos(BRIDGE_X0, -Y_OUT, BRIDGE_Z0) * Box(BRIDGE_X1 - BRIDGE_X0, 2 * Y_OUT, BRIDGE_Z1 - BRIDGE_Z0, align=MIN)
    th = math.degrees(math.atan2(BRIDGE_XT - BRIDGE_X0, BRIDGE_Z1 - BRIDGE_Z0))
    return p - Pos(BRIDGE_X0, 0, BRIDGE_Z0) * Rot(Y=th) * Box(50, 200, 100, align=(Align.MAX, Align.CENTER, Align.CENTER))

def elbow_assembly(bridge: bool = True):
    """Gá khuỷu ghép đôi M1-AR-020: đối xứng gá vai qua mặt x = 0, đặt trục ở gốc (đầu link = gốc cục bộ). v0.4.5: thêm cầu chặn ngược (bridge=False: bản v0.4.4)."""
    cr = mirror(sh.cradle(), about=Plane.YZ)
    if bridge: cr = cr + elbow_bridge()
    sl = mirror(sh.servo(-1), about=Plane.YZ); sr = mirror(sh.servo(1), about=Plane.YZ)
    return dict(e_cradle=cr, e_servoL=sl, e_servoR=sr)

def inter(a, b):
    ba, bb = a.bounding_box(), b.bounding_box()
    if (ba.min.X > bb.max.X or ba.max.X < bb.min.X or ba.min.Y > bb.max.Y or ba.max.Y < bb.min.Y or ba.min.Z > bb.max.Z or ba.max.Z < bb.min.Z):
        return 0.0, ">bbox"
    try:
        r = a & b; v = 0.0 if r is None else float(r.volume)
    except Exception:
        v = float("nan")
    d = None
    if not v:
        try: d = round(float(a.distance_to(b)), 2)
        except Exception: d = None
    return round(v, 1), d

def main():
    cr = cradle_v2(); sL, sR = sh.servo(-1), sh.servo(1); bolts = anchor_bolts()
    static = dict(cradle=cr, servo_L=sL, servo_R=sR, bolt_A_L=bolts[0], bolt_A_R=bolts[1], stop_pad=stop_pad())
    ea = elbow_assembly()
    rows = []
    for qs in (-90, -50, 0, 45, 90):
        mov = dict(link=link_v2(qs), pin=pin(qs), spring_L=spring(qs, -1), spring_R=spring(qs, 1))
        rows.append(dict(kind="vai", q_vai=qs, L_spring=round(spring_len(qs), 1), **{f"{m}|{n}": inter(a, b) for (m, a) in mov.items() for (n, b) in static.items() if not (m.startswith("spring") and n.startswith("bolt_A")) and not (m == "link" and n.startswith("bolt"))}))
        for qe in (-20, 60, 110, 120, 130, 140, 150):
            loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
            e = {k: loc * v for k, v in ea.items()}
            row = dict(kind="khuyu", q_vai=qs, q_khuyu=qe)
            for en, eo in e.items():
                for n, o in list(static.items()) + list(mov.items()):
                    if en != "e_cradle" and n in ("servo_L", "servo_R") and False: continue
                    row[f"{en}|{n}"] = inter(eo, o)
            rows.append(row); print(qs, qe, flush=True)
    json.dump(rows, open(os.path.join(EXP, "balance_fit.json"), "w"), ensure_ascii=False, indent=1)
    # xuất STEP
    export_step(cr, os.path.join(EXP, "M1-SH-003_cradle_v2.step"))
    export_step(link_v2(0), os.path.join(EXP, "M1-SH-004_link_v2.step"))
    export_step(stop_pad(), os.path.join(EXP, "M1-SH-015_stop_pad.step"))
    export_step(Compound([pin(0), *bolts]), os.path.join(EXP, "M1-SH-012-014_pin_bolts.step"))
    export_step(Compound(list(ea.values())), os.path.join(EXP, "M1-AR-020_elbow_cradle_pair.step"))
    asm = [cr, sL, sR, link_v2(0), pin(0), spring(0, -1), spring(0, 1), *bolts]
    export_step(Compound(asm), os.path.join(EXP, "shoulder_balance_assembly.step"))
    print("vol cradle_v2 cm3", cr.volume / 1000)

if __name__ == "__main__":
    main()
