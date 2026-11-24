# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""v0.4.5 (22/11/2026): tab U M1-AR-023 + ốc M3 thật + cầu chặn ngược M1-AR-020. build123d.
Sửa: (1) lỗ M3 dưới dời (105,5; -8) -> (91,5; -8) (params.TAB_HOLES), chân tab kéo dài; (2) đệm PU 70A chặn ngược dời vào giữa |y| <= 6,5,
ngoài dải y servo khuỷu; (3) cầu chặn ngược trên M1-AR-020 (balance.elbow_bridge) chạm đệm thay góc trên-sau servo, cùng góc chạm -23,5.
Ốc mô hình theo kích thước chuẩn: vòng đệm DIN 125 M3 Ø7 x 0,5; đai ốc DIN 985 M3 bao Ø6,4 (e 6,01) x 4; đầu vít ISO 4762 M3 Ø5,5 x 3; thân Ø3 dài 20.
Quét: (A) khuỷu -32..133 (bước 1, thêm biên) cơ cấu khuỷu (gá + cầu + 2 servo) với mọi chi tiết gắn link, bản v0.4.4 và v0.4.5;
(B) chi tiết gắn link mới (tab, đệm, ốc) với gá vai, chặn trên/dưới, đế xoay, xe nâng, tay đỡ ở vai -88..62;
(C) cầu chặn ngược với mọi vật cản tĩnh + chi tiết gắn link theo ELBOW_COUPLE hiện hành (spec.elbow_max) và giới hạn cứng.
Chạy: python failsafe_v045.py [--scan]  -> ../export/failsafe_v045_scan.json + STEP *_v045.step"""
import json, math, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
from build123d import Box, Cylinder, Pos, Rot, Align, Compound, export_step
import balance as b, failsafe as f, failsafe_v042 as g2, failsafe_v042_low as L, failsafe_v043 as g3, shoulder as sh, lift
from params import LINK_LEN, TAB_HOLES, SERVO_H
from faibotic import spec

C = (Align.CENTER, Align.CENTER, Align.MIN)
WASH_D, WASH_T, NUT_D, NUT_H, HEAD_D, HEAD_K, SHANK_D, SHANK_L = 7.0, 0.5, 6.4, 4.0, 5.5, 3.0, 3.0, 20.0
LEG_OUT = 7.1    # mặt ngoài chân tab

def _cy(x, z, y0, y1, d):
    return Pos(x, y0, z) * Rot(X=-90) * Cylinder(d / 2, y1 - y0, align=C)

def hardware(holes=TAB_HOLES):
    """M1-HW-008: mỗi lỗ 1 vít ISO 4762 M3x20 + 2 vòng đệm + đai ốc DIN 985 (+y). Hệ link."""
    out = {}
    for i, (hx, hz) in enumerate(holes):
        out[f"wash_p{i}"] = _cy(hx, hz, LEG_OUT, LEG_OUT + WASH_T, WASH_D)
        out[f"nut{i}"] = _cy(hx, hz, LEG_OUT + WASH_T, LEG_OUT + WASH_T + NUT_H, NUT_D)
        out[f"wash_n{i}"] = _cy(hx, hz, -LEG_OUT - WASH_T, -LEG_OUT, WASH_D)
        out[f"head{i}"] = _cy(hx, hz, -LEG_OUT - WASH_T - HEAD_K, -LEG_OUT - WASH_T, HEAD_D)
        out[f"shank{i}"] = _cy(hx, hz, -LEG_OUT - WASH_T, -LEG_OUT - WASH_T + SHANK_L, SHANK_D)
        hole = _cy(hx, hz, -LEG_OUT - 10, LEG_OUT + 10, 3.2)      # lỗ vòng đệm/đai ốc (thân vít đi qua; đầu vít liền thân)
        for k in (f"wash_p{i}", f"nut{i}", f"wash_n{i}"): out[k] = out[k] - hole
    return out

def link_parts(v045=True):
    tab, rp = g2.elbow_rev(v045)
    old_holes = ((140.0 + g2.ELB_PAD_FRONT_XE - g2.ELB_T - g2.TAB_T + g2.TAB_HOLE_DX, z) for z in g2.TAB_HOLE_Z)
    P = {"tab_023": tab, "dem_022": g3.pad_pu(), "de_024": g3.pad_base(), "pin": b.pin(0)}
    for i, r in enumerate(rp): P[f"dem_023_{i}"] = r
    P.update(hardware(TAB_HOLES if v045 else tuple(old_holes)))
    return P

def vol(a, c):
    return b.inter(a, c)[0] or 0.0

def scan_elbow(v045, angles):
    ea = b.elbow_assembly(bridge=v045); lp = link_parts(v045); lp["link"] = b.link_v2(0)
    if v045:
        import assembly_full as A
        ea = dict(ea, cang_tay_021=A.forearm_021())       # v0.4.5: ống cẳng tay kéo về gá khuỷu, quét cùng
    rows = []
    for qe in angles:
        loc = Pos(LINK_LEN, 0, 0) * Rot(Y=qe); r = {}
        for en, eo in ea.items():
            e = loc * eo
            for n, o in lp.items():
                v = vol(e, o)
                if v > 0: r[f"{en}|{n}"] = round(v, 1)
        rows.append(dict(q_khuyu=qe, giao=r))
    return rows

def shoulder_static(za=200.0):
    """Vật cản tĩnh trong hệ vai trái: gá vai, 2 servo vai, chặn trên/dưới + đệm, đế xoay, xe nâng, tay đỡ (lift.carriage), cột."""
    import assembly_full as A
    xt = g2.find_top_xf(); pad, br, _ = L.shapes(45.0, 50.0)
    S = {"gia_vai": f.cradle_v4(), "servo_vai_L": sh.servo(-1), "servo_vai_R": sh.servo(1), "thanh_tren": g2.top_bar(xt), "dem_tren": g2.top_pad(xt),
         "chan_duoi": br, "dem_duoi": pad, "de_xoay_024": A.pan_hub_024()}
    for i, s in enumerate(b.anchor_bolts()): S[f"neo{i}"] = s
    T = Pos(-lift.SX, -lift.SY, -(lift.Z_AX0 + za))
    for n, s in lift.carriage(lift.Z_AX0 + za).items(): S[f"xe_{n}"] = T * s
    S["cot"] = T * lift.column()
    return S

def scan_link_vs_static(S, v045=True):
    lp = link_parts(v045); new = {k: v for k, v in lp.items() if k.startswith(("tab", "dem_023", "wash", "nut", "head", "shank"))}
    rows = []
    for qs in [-88, -85, -82, -80.5, -80] + list(range(-75, 50, 5)) + [48.5, 50, 52, 55, 58, 62]:
        r = {}
        for n, o in new.items():
            s = Rot(Y=qs) * o
            for m, so in S.items():
                v = vol(s, so)
                if v > 0: r[f"{n}|{m}"] = round(v, 1)
        rows.append(dict(q_vai=qs, giao=r))
    return rows

def scan_bridge(S):
    """Cầu chặn ngược (gắn khuỷu) với vật cản tĩnh + link + chi tiết gắn link, theo khuỷu <= elbow_max(vai) và các biên cứng."""
    br = b.elbow_bridge(); lp = link_parts(True); lp["link"] = b.link_v2(0)
    rows = []
    vs = list(spec.ELBOW_COUPLE_SHOULDER_DEG) + [-88.0, 62.0]
    for qs in vs:
        em = math.degrees(spec.elbow_max(math.radians(min(max(qs, -80.0), spec.SHOULDER_SOFT_MAX_DEG))))
        qes = sorted(set([-32.0, -23.0, -10.0] + [float(x) for x in range(0, int(em) + 1, 5)] + [round(em, 1), 133.0]))
        for qe in qes:
            loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe) * br; r = {}
            for m, so in S.items():
                v = vol(loc, so)
                if v > 0: r[m] = round(v, 1)
            for m, so in lp.items():
                v = vol(loc, Rot(Y=qs) * so)
                if v > 0: r[m] = round(v, 1)
            rows.append(dict(q_vai=qs, q_khuyu=qe, elbow_max=round(em, 1), giao=r))
    return rows

def main():
    tab, rp = g2.elbow_rev(True); hw = hardware()
    ea = b.elbow_assembly(True)
    for name, o in (("M1-AR-023_rev_stop_tab_v045", tab), ("M1-AR-023_rev_pad_v045", Compound(rp)), ("M1-SH-004_link_v045", b.link_v2(0)),
                    ("M1-AR-020_elbow_cradle_v045", ea["e_cradle"]), ("M1-HW-008_tab_screws_v045", Compound(list(hw.values())))):
        export_step(o, os.path.join(b.EXP, name + ".step"))
    import assembly_full as A
    fa = A.forearm_021(); export_step(fa, os.path.join(b.EXP, "M1-AR-021_forearm_v045.step"))
    rr = fa & ea["e_cradle"]
    noi = dict(khe_mm=round(float(fa.distance_to(ea["e_cradle"])), 3), giao_mm3=0.0 if rr is None else round(float(rr.volume), 3),
               x0_ong=A.FORE_X0, the_tich_cm3=round(fa.volume / 1000, 2), khe_servo_khuyu_mm=round(min(float(fa.distance_to(ea["e_servoL"])), float(fa.distance_to(ea["e_servoR"]))), 2))
    out = dict(noi_cang_tay_ga_khuyu=noi, lo=dict(TAB_HOLES=TAB_HOLES), cau=dict(x=(b.BRIDGE_X0, b.BRIDGE_X1), z=(b.BRIDGE_Z0, b.BRIDGE_Z1), y=(-b.Y_OUT, b.Y_OUT)),
               the_tich_cm3=dict(tab=round(tab.volume / 1000, 3), dem=round(sum(r.volume for r in rp) / 1000, 3), cau=round(b.elbow_bridge().volume / 1000, 3),
                                 link=round(b.link_v2(0).volume / 1000, 3)))
    if "--scan" in sys.argv:
        t = time.time()
        fine = sorted(set([float(x) for x in range(-32, 134)] + [-23.5, 117.5, 118.5, 127.5, 128.5, 132.5]))
        out["A_khuyu_v045"] = scan_elbow(True, fine); print("A done", time.time() - t, flush=True)
        out["A_khuyu_v044"] = scan_elbow(False, [-30.0, -23.0, 104.0, 110.0, 115.0, 118.0, 120.0, 122.0, 125.0, 127.0, 130.0, 133.0]); print("A0 done", flush=True)
        S = shoulder_static()
        out["B_link_vs_tinh"] = scan_link_vs_static(S); print("B done", time.time() - t, flush=True)
        out["B_link_vs_tinh_v044"] = [r for r in scan_link_vs_static(S, False) if r["q_vai"] >= 55]
        out["C_cau"] = scan_bridge(S); print("C done", time.time() - t, flush=True)
        # tóm tắt
        def bad_A(rows, allow):
            return [(r["q_khuyu"], k, v) for r in rows for k, v in r["giao"].items() if not allow(r["q_khuyu"], k)]
        allowA = lambda q, k: ("dem_023" in k and "e_cradle" in k and q <= -23.0) or ("dem_022" in k and q >= 128.0) or (q <= -32.0 and "tab_023" in k and "e_cradle" in k)
        out["tom_tat"] = dict(
            A_v045_ngoai_chu_y=bad_A(out["A_khuyu_v045"], allowA),
            A_v045_dem023_cham_dau=min([r["q_khuyu"] for r in out["A_khuyu_v045"] if any("dem_023" in k for k in r["giao"])], default=None),
            A_v045_dem023_max_mm3=max([v for r in out["A_khuyu_v045"] for k, v in r["giao"].items() if "dem_023" in k], default=0),
            A_v045_servo_vs_dem023=[(r["q_khuyu"], k, v) for r in out["A_khuyu_v045"] for k, v in r["giao"].items() if "dem_023" in k and "servo" in k],
            A_v044_ngoai_chu_y=bad_A(out["A_khuyu_v044"], allowA),
            B_giao=[(r["q_vai"], r["giao"]) for r in out["B_link_vs_tinh"] if r["giao"]],
            B_giao_v044=[(r["q_vai"], r["giao"]) for r in out["B_link_vs_tinh_v044"] if r["giao"]],
            C_giao=[(r["q_vai"], r["q_khuyu"], r["giao"]) for r in out["C_cau"] if r["giao"]])
        print(json.dumps(out["tom_tat"], ensure_ascii=False), flush=True)
    json.dump(out, open(os.path.join(b.EXP, "failsafe_v045_scan.json" if "--scan" in sys.argv else "failsafe_v045_parts.json"), "w"), ensure_ascii=False, indent=1, default=float)

if __name__ == "__main__":
    main()
