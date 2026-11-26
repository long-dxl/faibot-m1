# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""v0.4.1: chặn dưới vai (tay thõng, M1-SH-017 + đệm M1-SH-018), đệm khuỷu trước M1-AR-022, chặn khuỷu ngược M1-AR-023. build123d.
 Kích thước đệm khớp spec.FAILSAFE_PADS (diện tích tiếp xúc = bề dày link 8 mm x chiều cao đệm).
Chạy: python failsafe_v041.py [--scan]  -> ../export/failsafe_v041_scan.json + STEP của chi tiết mới."""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from build123d import Box, Pos, Rot, Part, Compound, export_step
import balance as b
import failsafe as f
import shoulder as sh
from params import *

MIN = b.MIN
Y_UP_IN, Y_UP_T = b.Y_IN, 6.0                          # trụ chặn dưới nằm dưới má gá vai (y 37,9..43,9), bắt 2 x M4 vào đáy má [giả định]
LOW_PAD_H, LOW_PAD_T, LOW_PAD_W = 6.0, 16.0, 40.0      # đệm chặn dưới M1-SH-018: 40 x 6 x 16 (diện tích tiếp xúc 8 x 6 = 48 mm2)
LOW_BAR_T, LOW_BAR_Z0, LOW_BAR_Z1 = 8.0, -94.0, -80.0   # v0.4.1 vòng 2: thanh thấp lại (trước -100..-78): gá khuỷu chạm cạnh dưới thanh
ELB_PAD = dict(x0=77.0, x1=81.0, t=7.0)                # đệm khuỷu M1-AR-022: dán dưới link (z -15 -> -22), x link 77..81, rộng 8 (y +-4): 8 x 4 = 32 mm2
TAB_X0 = 140.0 - 35.5                                  # tab nhôm M1-AR-023 (hệ link): xe -35,5..-27,5 quanh trục khuỷu
Q_UP_CONTACT = 97.5

def low_pad(xf: float) -> Part:
    """Đệm chặn dưới: mặt trước (hướng link) tại x = xf, cao 6 (z -92..-86), rộng 40."""
    return Pos(xf - LOW_PAD_T, -LOW_PAD_W / 2, -89.0 - LOW_PAD_H / 2) * Box(LOW_PAD_T, LOW_PAD_W, LOW_PAD_H, align=MIN)

def low_stop(xf: float) -> Part:
    """M1-SH-017 chặn dưới nhôm 6061-T6: thanh ngang x (xf-24 .. xf-16) nối hai trụ đứng ngay dưới má gá vai (z -30 .. -100)."""
    x0 = xf - LOW_PAD_T - LOW_BAR_T
    bar = Pos(x0, -(Y_UP_IN + Y_UP_T), LOW_BAR_Z0) * Box(LOW_BAR_T, 2 * (Y_UP_IN + Y_UP_T), LOW_BAR_Z1 - LOW_BAR_Z0, align=MIN)
    ups = [Pos(x0, y0, LOW_BAR_Z0) * Box(LOW_BAR_T, Y_UP_T, -30.0 - LOW_BAR_Z0, align=MIN) for y0 in (Y_UP_IN, -(Y_UP_IN + Y_UP_T))]
    return bar + ups[0] + ups[1]

def find_xf(q_contact: float) -> float:
    """Tìm mặt trước đệm sao cho link bắt đầu chạm đệm ở q_contact (bisection theo x, link_v2)."""
    lk = b.link_v2(q_contact)
    lo, hi = -60.0, 0.0                                  # lo: đệm xa link (không chạm), hi: lấn vào link
    for _ in range(14):
        mid = (lo + hi) / 2
        v = (b.inter(lk, low_pad(mid))[0] or 0.0)
        lo, hi = (mid, hi) if v == 0 else (lo, mid)
    return round(lo, 1)

def elbow_pad() -> Part:
    """M1-AR-022: đệm PU 50A 8 x 4 x 7 dán mặt dưới link (hệ link, trục vai ở gốc) tại x 77..81."""
    return Pos(ELB_PAD["x0"], -4.0, -15.0 - ELB_PAD["t"]) * Box(ELB_PAD["x1"] - ELB_PAD["x0"], 8.0, ELB_PAD["t"], align=MIN)

def rev_stop():
    """M1-AR-023: tab nhôm 8 x 28 x 12 bắt M4 vào mặt trên link (z 15..27, y +-14) + 2 đệm PU 50A 8 x 6,5 x 12 (y 7,5..14 hai bên, 2 x 78 = 156 mm2): chặn khuỷu ngược.
    Hạn chế (quét CAD): kéo tab/đệm xuống z < 15 để bắt gờ servo ở khuỷu < -33 thì đệm giao gá khuỷu từ khuỷu ~100 (472 mm3 ở 120), chặn tầm khuỷu; nên giữ z >= 15, hành trình chỉ ~5 độ (-27,5 -> -33).
    Gờ sau của gá servo khuỷu (xe -10, z 24) chạm mặt đệm ở ~ -24 độ."""
    tab = Pos(TAB_X0, -14.0, 15.0) * Box(8.0, 28.0, 12.0, align=MIN)
    pads = [Pos(TAB_X0 + 8.0, y0, 15.0) * Box(8.0, 6.5, 12.0, align=MIN) for y0 in (7.5, -14.0)]
    return tab, pads

def vol(a, c):
    return b.inter(a, c)[0] or 0.0

def scan(xf):
    out = {}
    ea = b.elbow_assembly()
    pad = elbow_pad(); tab, rpads = rev_stop(); rp = rpads[0] + rpads[1]
    # (1) đệm khuỷu và chặn ngược so với cơ cấu khuỷu, qs bất kỳ (tương đối): khuỷu -60..140
    rows = []
    for qe in (-60, -45, -40, -35, -30, -26, -24, -22, -20, 0, 60, 120, 128, 130, 131, 132, 134, 136, 138):
        loc = Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
        r = dict(q_khuyu=qe, dem_khuyu_ar022=sum(vol(loc * o, pad) for o in ea.values()), chan_nguoc_cung=sum(vol(loc * o, tab) for o in ea.values()),
                 chan_nguoc_dem_ar023=sum(vol(loc * o, rp) for o in ea.values()))
        rows.append({k: (round(v, 1) if isinstance(v, float) else v) for k, v in r.items()}); print(rows[-1], flush=True)
    out["khuyu"] = rows
    # (2) chặn dưới vai (M1-SH-017/018): link, chốt, gá khuỷu, lò xo, gá vai, servo
    cr, sL, sR = f.cradle_v4(), sh.servo(-1), sh.servo(1)
    stop, lpad = low_stop(xf), low_pad(xf)
    srows = []
    for qs in (90, 95, 96, 97, 97.5, 98, 99, 100, 102, 104, 105):
        link, pin_, sp = b.link_v2(qs), b.pin(qs), [b.spring(qs, -1), b.spring(qs, 1)]
        r = dict(q_vai=qs, link_dem=vol(link, lpad), link_thanh=vol(link, stop), pin_stop=vol(pin_, stop) + vol(pin_, lpad), spring_stop=max(vol(sp[0], stop), vol(sp[1], stop)),
                 stop_cradle=vol(cr, stop), tab_cradle=0.0)
        # tab + đệm khuỷu gắn link so với gá vai và thanh chặn (hệ gá vai): đủ qs
        lt = Rot(Y=qs) * (tab + rp + pad)
        r["tab_stop_cradle"] = round(vol(lt, stop) + vol(lt, cr) + vol(lt, f.stop_bar_alu()), 1)
        # gá khuỷu (qe 0, 60, 90, 120, 130) so với chặn dưới
        r["khuyu_vs_stop"] = {}
        for qe in (-20, 0, 60, 90, 120, 130):
            loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
            r["khuyu_vs_stop"][qe] = round(sum(vol(loc * o, stop) for o in ea.values()), 1)
        srows.append({k: (round(v, 1) if isinstance(v, float) else v) for k, v in r.items()}); print(srows[-1], flush=True)
    out["vai_chan_duoi"] = srows
    # ngưỡng giao gá khuỷu - chặn dưới: vai x khuỷu (độ), thể tích mm3 cứng (thanh+trụ+đệm)
    th = {}
    for qs in (0, 30, 45, 50, 55, 60, 75, 85, 90, 97.5, 105):
        for qe in (80, 90, 95, 100, 105, 110, 115, 120, 130):
            loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
            th[f"{qs}|{qe}"] = round(sum(vol(loc * o, stop) + vol(loc * o, lpad) for o in ea.values()), 1)
    out["gakhuyu_vs_chanduoi"] = th; print(th, flush=True)
    # (3) tab/đệm khuỷu vs gá vai và thanh chặn trên ở dải vai -88..-80, 0 (không giao cứng)
    trow = []
    for qs in (-89, -88, -85, -60, 0, 60, 90):
        lt = Rot(Y=qs) * (tab + rp + pad)
        trow.append(dict(q_vai=qs, tab_cradle=round(vol(lt, cr), 1), tab_bar=round(vol(lt, f.stop_bar_alu()), 1)))
    out["tab_vs_gavai"] = trow; print(trow, flush=True)
    return out

def main():
    xf = find_xf(Q_UP_CONTACT)
    print("mặt trước đệm chặn dưới xf =", xf, flush=True)
    stop, lpad = low_stop(xf), low_pad(xf)
    tab, rpads = rev_stop()
    for name, o in (("M1-SH-017_low_stop_alu", stop), ("M1-SH-018_low_pad", lpad), ("M1-AR-022_elbow_pad", elbow_pad()), ("M1-AR-023_rev_stop_tab", tab), ("M1-AR-023_rev_stop_pads", Compound(rpads))):
        export_step(o, os.path.join(b.EXP, name + ".step"))
    if "--scan" not in sys.argv:
        return
    res = scan(xf)
    res["xf"] = xf; res["q_up_contact"] = Q_UP_CONTACT
    json.dump(res, open(os.path.join(b.EXP, "failsafe_v041_scan.json"), "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
