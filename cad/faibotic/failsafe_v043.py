# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""v0.4.3: (1) đệm khuỷu M1-AR-022 dời để chạm ở 128 độ (chặn cứng khuỷu 136 -> 133); (2) kiểm DFM trong CAD: thanh M1-SH-016 giữa hai trụ,
vát 45 độ trụ M1-SH-003, lỗ giọt nước, R1,5 link, tab M1-AR-023 chữ U 24 (2 x M3 xuyên link); (3) xuất STEP v043. build123d.
Chạy: python failsafe_v043.py [--check] [--step] [--elbow]  -> ../export/failsafe_v043_scan.json, STEP *_v043.step"""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from build123d import Box, Pos, Rot, Compound, export_step
import balance as b, failsafe as f, failsafe_v042 as g2, failsafe_v042_low as L, shoulder as sh
from params import LINK_LEN

MIN = b.MIN
EP_X0, EP_X1 = 77.0, 81.0
EP_SPACER, EP_PU = 5.0, 7.0          # đế lót 5 (PETG/nhôm, dán cùng đệm) + PU 7: cao tổng 12 dưới link (v0.4.1: 7)
Q_ELBOW_CONTACT, Q_ELBOW_HARD = 128.0, 133.0

def elbow_pad() -> "Part":
    """M1-AR-022 v043: PU 50A 8 x 4 x 7 dán lên đế lót 8 x 4 x 5 bắt dưới link (hệ link, x 77..81, z -15 -> -27)."""
    return Pos(EP_X0, -4.0, -15.0 - EP_SPACER - EP_PU) * Box(EP_X1 - EP_X0, 8.0, EP_SPACER + EP_PU, align=MIN)

def pad_pu():
    """M1-AR-022: PU 50A 8 x 4 x 7 (z -27..-20)."""
    return Pos(EP_X0, -4.0, -15.0 - EP_SPACER - EP_PU) * Box(EP_X1 - EP_X0, 8.0, EP_PU, align=MIN)

def pad_base():
    """M1-AR-024: đế lót nhôm 6061-T6 8 x 4 x 5 (z -20..-15), dán epoxy lên mặt dưới link và PU lên đế [giả định, chưa tính bền mối dán]."""
    return Pos(EP_X0, -4.0, -15.0 - EP_SPACER) * Box(EP_X1 - EP_X0, 8.0, EP_SPACER, align=MIN)

def vol(a, c):
    return b.inter(a, c)[0] or 0.0

def main():
    out = {}
    ea = b.elbow_assembly(); pad = elbow_pad()
    xt = g2.find_top_xf(); assert abs(xt - f.TOP_BAR_XF) < 1e-9, (xt, f.TOP_BAR_XF)
    # (1) đệm khuỷu v043: chạm 128, sạch với cơ cấu khuỷu ở <= 127,5
    rows = []
    for qe in (120, 126, 127, 127.5, 128, 128.1, 129, 130, 131, 133):
        loc = Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
        rows.append(dict(q_khuyu=qe, dem_ar022=round(sum(vol(loc * o, pad) for o in ea.values()), 1)))
    out["khuyu_dem"] = rows; print(rows, flush=True)
    # (2) đệm khuỷu v043 gắn link so với giá, đệm, thanh trên, gá vai (vai -80..62)
    cr = f.cradle_v4(); pd_, br, _ = L.shapes(45.0, 50.0); tb, tp = g2.top_bar(xt), g2.top_pad(xt); tab, rp = g2.elbow_rev(); rpp = Compound(rp)
    obs = dict(cradle=cr, chan_duoi=br, dem_duoi=pd_, thanh_tren=tb, dem_tren=tp)
    rr = []
    for qs in (-80, -60, -30, 0, 30, 45, 50, 55, 62):
        lt = Rot(Y=qs) * (pad + tab + rpp)
        rr.append(dict(q_vai=qs, **{k: round(vol(lt, o), 1) for k, o in obs.items()})); 
    out["dem_tab_gan_link_vs_gia"] = rr; print(rr, flush=True)
    # (3) thanh trên giữa hai trụ: giao với gá vai, link, chốt, lò xo ở các góc vai tới chặn (link chạm đệm ở -80,5)
    sv = []
    for qs in (-88, -85, -80.5, -80, -60):
        lk, pn, sp = b.link_v2(qs), b.pin(qs), [b.spring(qs, -1), b.spring(qs, 1)]
        sv.append(dict(q_vai=qs, thanh_cradle=round(vol(tb, cr), 1), thanh_link=round(vol(tb, lk), 1), thanh_pin=round(vol(tb, pn), 1), thanh_spring=round(max(vol(tb, s) for s in sp), 1),
                       dem_link=round(vol(tp, lk), 1)))
    out["thanh_tren_v043"] = sv; print(sv, flush=True)
    # (4) cradle v5 vs servo (nêm, lỗ giọt nước không đụng servo/link): giao cứng servo, link ở vai 0
    out["cradle_vs_servo"] = round(vol(cr, sh.servo(-1)) + vol(cr, sh.servo(1)), 1); out["cradle_vs_link0"] = round(vol(cr, b.link_v2(0)), 1)
    out["cradle_volume_mm3"] = round(float(cr.volume), 1)
    out["thanh_vs_trụ_gia_vai"] = round(vol(tb, cr), 1)
    print(out["cradle_vs_servo"], out["cradle_vs_link0"], out["cradle_volume_mm3"], flush=True)
    json.dump(out, open(os.path.join(b.EXP, "failsafe_v043_scan.json"), "w"), ensure_ascii=False, indent=1)
    for name, o in (("M1-SH-003_cradle_v043", cr), ("M1-SH-016_top_bar_v043", tb), ("M1-SH-004_link_v043", b.link_v2(0)), ("M1-AR-022_elbow_pad_v043", pad_pu()), ("M1-AR-024_pad_base_v043", pad_base()), ("M1-AR-023_rev_stop_tab_v043", tab), ("M1-AR-020_elbow_cradle_pair_v043", None)):
        if o is not None:
            export_step(o, os.path.join(b.EXP, name + ".step"))

if __name__ == "__main__":
    main()
