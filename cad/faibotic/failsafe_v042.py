# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""v0.4.1 vòng 2: giới hạn mềm vai -80 / khuỷu ngược -10. Dời chặn vai trên (đệm chạm -80,5, dày 20), chặn khuỷu ngược (đệm chạm -17,5),
và chặn dưới vai dạng HẸP giữa (qua khe giữa hai servo khuỷu) để gá khuỷu không đập thanh. build123d.
Chạy: python failsafe_v042.py [--scan] -> ../export/failsafe_v042_scan.json + STEP."""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from build123d import Box, Cylinder, Align, Pos, Rot, Part, Compound, export_step
import balance as b
import failsafe as f
import failsafe_v041 as g
import shoulder as sh
from params import *

MIN = b.MIN
TOP_PAD_T, TOP_PAD_H, TOP_PAD_W, TOP_PAD_Z = 20.0, 6.0, 40.0, 89.0     # đệm vai trên M1-SH-015: 40 x 6 x 20 (bắt 8 x 6 = 48 mm2, Shore 50A)
Q_TOP_CONTACT = -80.5
TAB_T = 12.0                                                          # v0.4.3 vòng 2: tab M1-AR-023 chữ U dài 12 (trước 8; 24 chạm đệm vai trên 612 mm3 ở vai -80)
TAB_LEG_Z0, TAB_HOLE_Z, TAB_HOLE_DX = -14.0, (8.0, -8.0), 6.0
ELB_T = 10.0                                                          # đệm khuỷu ngược dày 10 (đổi từ 8)
ELB_PAD_FRONT_XE = -18.5                                              # mặt trước đệm (hệ khuỷu): gờ servo chạm ở ~-23,5 (-17 làm đệm giao gá khuỷu khi gập 110-136: 37-51 mm3)
NARROW_Y = 7.0                                                        # chặn dưới hẹp: y +-7 (khe giữa hai servo khuỷu: +-7,5)

def top_pad(xf: float) -> Part:
    return Pos(xf - TOP_PAD_T, -TOP_PAD_W / 2, TOP_PAD_Z - TOP_PAD_H / 2) * Box(TOP_PAD_T, TOP_PAD_W, TOP_PAD_H, align=MIN)

def top_bar(xf: float) -> Part:
    """M1-SH-016 dời theo đệm: nhôm 8 x 20 x 98, mặt trước x = xf - 20."""
    # v0.4.3: nằm giữa hai trụ neo (mặt trong y = +-Y_IN, khe 0,2 mỗi bên), dày 12, 2 lỗ M4 tarô (Ø3,3 sâu 10) ở hai đầu, trục y
    w = 2 * b.Y_IN - 0.4
    bar = Pos(xf - TOP_PAD_T - f.TOP_BAR_T, -w / 2, f.BAR_Z0) * Box(f.TOP_BAR_T, w, f.BAR_Z1 - f.BAR_Z0, align=MIN)
    for s in (-1, 1):
        bar = bar - Pos(xf - TOP_PAD_T - f.TOP_BAR_T / 2, s * w / 2, (f.BAR_Z0 + f.BAR_Z1) / 2) * Rot(X=90) * Cylinder(1.65, 20.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return bar

def find_top_xf() -> float:
    lk = b.link_v2(Q_TOP_CONTACT)
    lo, hi = -40.0, 40.0           # lo: xa link (không chạm); hi: lấn vào link
    for _ in range(16):
        mid = (lo + hi) / 2
        v = b.inter(lk, top_pad(mid))[0] or 0.0
        lo, hi = (mid, hi) if v == 0 else (lo, mid)
    return round(lo, 1)

PAD_HALF_Y = 6.5                                                      # v0.4.5: một đệm giữa |y| <= 6,5 (10 x 13 x 12, 156 mm2 như hai đệm 6,5 x 12 cũ)

def elbow_rev(v045: bool = True):
    """M1-AR-023 (dời): tab nhôm xe -34,8..-26,8 + 2 đệm dày 10 (xe -26,8..-16,8), z 15..27."""
    # v0.4.3 vòng 2: tab chữ U kẹp hai mặt link (không khoan tab vào cạnh link 8 mm): bản trên 12 x 28 x 12 (z 15..27) + hai chân dày 3 (y +-4,1..7,1) xuống z -14;
    # 2 x M3 xuyên chân-link-chân theo y (x x0+6; z +8 và -8; cách mép tab/chân/link >= 6 mm), đai ốc DIN 985 M3 phía +y. Mặt trước đệm giữ nguyên (xe -18,5).
    # v0.4.5: lỗ dưới dời về x 91,5 (params.TAB_HOLES), chân kéo dài về sau x 85,5..x0 ở z -14..-2 (lỗ cách mép 6);
    # đệm PU 70A thành một đệm giữa |y| <= 6,5 trên mặt link (chạm cầu chặn ngược M1-AR-020, không còn nằm trong dải y servo).
    x0 = 140.0 + ELB_PAD_FRONT_XE - ELB_T - TAB_T
    tab = Pos(x0, -14.0, 15.0) * Box(TAB_T, 28.0, 12.0, align=MIN)
    holes = TAB_HOLES if v045 else tuple((x0 + TAB_HOLE_DX, z) for z in TAB_HOLE_Z)
    xe0 = min(hx for hx, _ in holes) - 6.0
    for sg in (-1, 1):
        y0 = 4.1 if sg > 0 else -7.1
        tab = tab + Pos(x0, y0, TAB_LEG_Z0) * Box(TAB_T, 3.0, 15.0 - TAB_LEG_Z0, align=MIN)
        if v045 and xe0 < x0:
            ext = Pos(xe0, y0, TAB_LEG_Z0) * Box(x0 - xe0 + 0.01, 3.0, 12.0, align=MIN)
            ext = ext - Pos(xe0, 0, TAB_LEG_Z0 + 5.0) * Rot(Y=45) * Box(20, 20, 20, align=(Align.CENTER, Align.CENTER, Align.MAX))   # vát 5 x 5 góc sau-dưới (tay đỡ LF-023 ở vai 62)
            tab = tab + ext
    for hx, hz in holes:
        tab = tab - Pos(hx, 0, hz) * Rot(X=90) * Cylinder(1.7, 20.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    if v045:
        pads = [Pos(x0 + TAB_T, -PAD_HALF_Y, 15.0) * Box(ELB_T, 2 * PAD_HALF_Y, 12.0, align=MIN)]
    else:
        pads = [Pos(x0 + TAB_T, y0, 15.0) * Box(ELB_T, 6.5, 12.0, align=MIN) for y0 in (7.5, -14.0)]
    return tab, pads

def low_narrow(xf: float):
    """Chặn dưới hẹp M1-SH-017 (v2): tay đòn L nhôm y +-7: thân đứng x -95..-55 (bắt vào mặt dưới đế gá vai, z -30) + tay ngang tới đệm 14 x 6 x 16."""
    web = Pos(-95.0, -NARROW_Y, -100.0) * Box(40.0, 2 * NARROW_Y, 70.0, align=MIN)
    arm = Pos(-55.0, -NARROW_Y, -100.0) * Box((xf - g.LOW_PAD_T) - (-55.0), 2 * NARROW_Y, 22.0, align=MIN)
    pad = Pos(xf - g.LOW_PAD_T, -NARROW_Y, -89.0 - g.LOW_PAD_H / 2) * Box(g.LOW_PAD_T, 2 * NARROW_Y, g.LOW_PAD_H, align=MIN)
    return web + arm, pad

def vol(a, c):
    return b.inter(a, c)[0] or 0.0

def scan(xt, xl):
    ea = b.elbow_assembly(); out = {}
    tp, tb = top_pad(xt), top_bar(xt)
    tab, rpads = elbow_rev(); rp = Compound(rpads)
    cr = f.cradle_v4()
    # (1) đệm/bar vai trên: link chạm đệm, giao cứng với bar/gá khuỷu (đệm ép được)
    rows = []
    for qs in (-90, -89, -88, -85, -82, -80.5, -80, -79, -76):
        lk = b.link_v2(qs)
        r = dict(q_vai=qs, link_dem=vol(lk, tp), link_thanh=vol(lk, tb), thanh_vs_cradle=vol(cr, tb) if qs == -90 else None, khuyu_vs_thanh={}, khuyu_vs_dem={})
        for qe in (-10, 30, 60, 90, 110, 130):
            loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
            r["khuyu_vs_thanh"][qe] = round(sum(vol(loc * o, tb) for o in ea.values()), 1)
            r["khuyu_vs_dem"][qe] = round(sum(vol(loc * o, tp) for o in ea.values()), 1)
        rows.append(r); print(r, flush=True)
    out["vai_tren"] = rows
    # (2) chặn khuỷu ngược dời
    kr = []
    for qe in (-40, -32, -28, -24, -20, -18, -17, -10, 0, 60, 120, 130, 131, 136):
        loc = Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
        kr.append(dict(q_khuyu=qe, tab_cung=round(sum(vol(loc * o, tab) for o in ea.values()), 1), dem_ar023=round(sum(vol(loc * o, rp) for o in ea.values()), 1)))
        print(kr[-1], flush=True)
    out["khuyu_nguoc"] = kr
    # (3) chặn dưới hẹp vs gá khuỷu: so với thanh rộng cũ
    arm, lpad = low_narrow(xl)
    old = g.low_stop(xl)   # thanh rộng thấp (z -94..-80) cho cả hai: so sánh
    lk = {}
    for qs in (0, 30, 50, 60, 75, 85, 90, 97.5, 105):
        for qe in (80, 95, 100, 110, 120, 130):
            loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
            lk[f"{qs}|{qe}"] = dict(moi=round(sum(vol(loc * o, arm) + vol(loc * o, lpad) for o in ea.values()), 1), cu=round(sum(vol(loc * o, old) for o in ea.values()), 1))
    out["chan_duoi_hep"] = lk; print(lk, flush=True)
    link_rows = []
    for qs in (90, 96, 97, 97.5, 98, 100, 105):
        l = b.link_v2(qs); sp = [b.spring(qs, -1), b.spring(qs, 1)]; pn = b.pin(qs)
        link_rows.append(dict(q_vai=qs, link_dem=round(vol(l, lpad), 1), link_arm=vol(l, arm), pin=vol(pn, arm) + vol(pn, lpad), spring=max(vol(sp[0], arm), vol(sp[1], arm)), arm_cradle=vol(cr, arm),
                              arm_servo=max(vol(sh.servo(1), arm), vol(sh.servo(-1), arm))))
        print(link_rows[-1], flush=True)
    out["chan_duoi_hep_link"] = link_rows
    return out

def main():
    xt = find_top_xf(); xl = g.find_xf(g.Q_UP_CONTACT)
    print("xf đệm vai trên", xt, "xf chặn dưới", xl, flush=True)
    tab, rpads = elbow_rev(); arm, lpad = low_narrow(xl)
    for name, o in (("M1-SH-015_top_pad_v042", top_pad(xt)), ("M1-SH-016_top_bar_v042", top_bar(xt)), ("M1-AR-023_rev_stop_tab_v042", tab), ("M1-AR-023_rev_pads_v042", Compound(rpads)), ("M1-SH-017_low_stop_narrow", arm), ("M1-SH-018_low_pad_narrow", lpad)):
        export_step(o, os.path.join(b.EXP, name + ".step"))
    if "--scan" in sys.argv:
        res = scan(xt, xl); res["xf_top"] = xt; res["xf_low"] = xl
        json.dump(res, open(os.path.join(b.EXP, "failsafe_v042_scan.json"), "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
