# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""v0.4.2 (phương án 2): chặn dưới vai để LINK chạm đệm TRƯỚC khi gá khuỷu tới thanh chặn. build123d.
Đệm PU 50A dán mặt dưới link (hệ link: x = r +-6, z -35..-15, y +-7; diện tích tiếp xúc 8 x 6 = 48 mm2), lưng đệm là tấm nhôm 8 mm trên
giá chữ Z M1-SH-017 (bắt dưới chân gá vai). So hai hình:
  A "tay dài": đệm ở r = 90 mm trên link (tay đòn lớn, lực nhỏ), dầm nhôm dài qua dưới má gá vai
  B "gọn": đệm ở r = 45 mm, dầm ngắn nằm dưới đáy má (z < -47), lực lớn gấp đôi
Chạy: python failsafe_v042_low.py [--scan] -> ../export/failsafe_v042_low_scan.json (+ STEP khi --step)."""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from build123d import Box, Cylinder, Align, Pos, Rot, Part, Compound, export_step
import balance as b
import failsafe as f
import failsafe_v042 as g2
import shoulder as sh
from params import *

MIN = b.MIN
PAD_H, PAD_AX, PAD_Y = 20.0, 12.0, 7.0     # đệm: dày 20 (theo pháp tuyến link), dài 12 dọc link, rộng y +-7; Shore 50A
PLATE_T = 8.0
BEAM_H = 16.0
FLANGE_HOLES_X = (-94.0, -72.0)           # 4 x M4 xuyên chân gá vai (M1-SH-003 cần 4 lỗ Ø4,5 tương ứng), bu-lông từ dưới lên, đai ốc DIN 985 mặt trên chân
XB = -103.0                                # mép sau giá chữ Z = mép sau chân gá vai

def shapes(r: float, qc: float):
    """Trả (pad, bracket, (xc, zc)) trong hệ gá vai; link chạm đệm tại vai qc."""
    pad = Rot(Y=qc) * (Pos(r - PAD_AX / 2, -PAD_Y, -15.0 - PAD_H) * Box(PAD_AX, 2 * PAD_Y, PAD_H, align=MIN))
    plate = Rot(Y=qc) * (Pos(r - 10.0, -PAD_Y, -15.0 - PAD_H - PLATE_T) * Box(20.0, 2 * PAD_Y, PLATE_T, align=MIN))
    c = Rot(Y=qc) * Pos(r, 0, -15.0 - PAD_H - PLATE_T / 2) * Pos(0, 0, 0)
    xc, zc = c.position.X, c.position.Z
    zt, zb = zc + BEAM_H / 2, zc - BEAM_H / 2
    beam = Pos(XB, -PAD_Y, zb) * Box(xc + 6.0 - XB, 2 * PAD_Y, BEAM_H, align=MIN)
    web = Pos(XB, -14.0, zb) * Box(8.0, 28.0, -30.0 - zb, align=MIN)           # web rộng y +-14 (ứng suất uốn 35 MPa ở 106 N, SF ~ 7, xem báo cáo)
    flange = Pos(XB, -14.0, -36.0) * Box(40.0, 28.0, 6.0, align=MIN)           # bắt 2 x M4 xuyên chân gá vai, đai ốc dưới (xem báo cáo)
    br = plate + beam + web + flange
    for hx in FLANGE_HOLES_X:
        for hy in (-8.0, 8.0):
            br = br - Pos(hx, hy, -36.0) * Cylinder(2.25, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN))   # M4 xuyên, lỗ Ø4,5, cách mép >= 6
    return pad, br, (xc, zc)

def vol(a, c):
    return b.inter(a, c)[0] or 0.0

def scan(r, qc, hard):
    pad, br, _ = shapes(r, qc)
    ea = b.elbow_assembly(); cr = f.cradle_v4(); sL, sR = sh.servo(-1), sh.servo(1)
    tp, tb = g2.top_pad(g2.find_top_xf()), g2.top_bar(g2.find_top_xf())
    out = {"r": r, "qc": qc, "hard": hard}
    # (a) link / chốt / lò xo / gá vai / servo / thanh trên so với giá chặn (qs đủ dải tới giới hạn cứng)
    rows = []
    for qs in (-80, -40, 0, 30, qc - 3, qc, qc + 2, qc + 4, hard - 1, hard):
        lk, pn, sp = b.link_v2(qs), b.pin(qs), [b.spring(qs, -1), b.spring(qs, 1)]
        rows.append(dict(q_vai=qs, link_dem=round(vol(lk, pad), 1), link_gia=round(vol(lk, br), 1), pin=round(vol(pn, br) + vol(pn, pad), 1),
                         spring=round(max(vol(s, br) + vol(s, pad) for s in sp), 1), gia_cradle=round(vol(cr, br), 1), gia_servo=round(vol(sL, br) + vol(sR, br), 1),
                         gia_thanh_tren=round(vol(tb, br) + vol(tp, br), 1)))
    out["link_gia"] = rows
    # (b) gá khuỷu (cả cơ cấu, mọi góc khuỷu cơ khí -32..136) so với đệm + giá, vai -80..hard
    worst = {}; tot = 0.0; bad = []
    qs_list = [-80, -60, -40, -20, 0, 20, 30, 40, 50, qc - 3, qc, qc + 2, qc + 4, hard - 1, hard]
    for qs in qs_list:
        for qe in range(-32, 137, 8):
            loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
            v = sum(vol(loc * o, br) + vol(loc * o, pad) for o in ea.values())
            if v > 0: bad.append((qs, qe, round(v, 1)))
            tot += v
    fine = {}
    for qs in (30, 40, 46, 50, 52, 54, 56, hard):
        for qe in (110, 120, 124, 128, 130, 131, 133, 136):
            loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
            fine[f"{qs}|{qe}"] = round(sum(vol(loc * o, br) + vol(loc * o, pad) for o in ea.values()), 1)
    out["gakhuyu_min"] = fine
    out["gakhuyu_vs_gia"] = dict(tong_mm3=round(tot, 1), cac_o_giao=bad[:40], so_o_giao=len(bad), luoi=f"{len(qs_list)} vai x {len(range(-32,137,8))} khuyu")
    print(json.dumps({k: v for k, v in out.items() if k not in ('link_gia', 'gakhuyu_min')}, ensure_ascii=False), flush=True)
    return out

if __name__ == "__main__":
    res = {}
    if "--try" in sys.argv:
        import itertools
        for r, qc, hd, t in ((45.0, 46.0, 62.0, 24.0), (50.0, 46.0, 62.0, 24.0), (45.0, 50.0, 62.0, 20.0), (50.0, 46.0, 60.0, 20.0)):
            PAD_H = t
            print("PAD_H", t, flush=True); scan(r, qc, hd)
        sys.exit(0)
    cand = (("A_tay_dai_r90", 90.0, 55.0, 62.0), ("B_gon_r45", 45.0, 55.0, 62.0), ("B2_r45_qc50", 45.0, 50.0, 58.0), ("C_r60_qc52", 60.0, 52.0, 60.0), ("D_r60_qc50", 60.0, 50.0, 58.0)) if "--all" in sys.argv else (("B3_r45_qc50_h62", 45.0, 50.0, 62.0),)
    for name, r, qc, hard in cand:
        res[name] = scan(r, qc, hard)
    if "--scan" in sys.argv:
        json.dump(res, open(os.path.join(b.EXP, "failsafe_v042_low_scan.json"), "w"), ensure_ascii=False, indent=1)
    if "--step" in sys.argv:
        pad, br, _ = shapes(45.0, 50.0)   # B3: r 45, link chạm ở 50, cứng 62
        export_step(br, os.path.join(b.EXP, "M1-SH-017_low_stop_v3.step")); export_step(pad, os.path.join(b.EXP, "M1-SH-018_low_pad_v3.step"))
