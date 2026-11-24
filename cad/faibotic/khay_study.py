# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""R4: khay đặt vật của M1 6+1. In bảng (1) khay hiện tại và (2) so 3 phương án A/B/C. Chỉ nghiên cứu: không sửa spec, sim, GLB, BOM.

    .venv/bin/python cad/faibotic/khay_study.py          (cần mujoco cho tải tay; chạy ~1 phút)

Quy ước:
- Đặt vật = tâm kẹp (TCP) ở mặt khay + KEEPOUT_TRAY_MARGIN (3 cm, đúng luật keepout SDK) + 1 mm; IK giải tích kinematics.arm_ik
  (yaw khoá 0, tự kẹp giới hạn khớp và ghép cổ tay), rồi limits.keepout_violations (luật đang nối vào lệnh) với khay của phương án.
- Lưới 2 cm trên mặt khay (bỏ gờ 8 mm). Ô đạt nếu ít nhất một tay đạt. Hai tư thế: kẹp chúi xuống (pitch -90°), kẹp ngang (pitch 0).
- Tải tay: demos/static_torque.max_payload trên tư thế IK (cận trên khối lượng), đỉnh = 60% chặn, liên tục = định mức.
- Lật: mô hình tests/tipover_dong.py (khối lượng M1-PL-masses.json, tay duỗi 1 kg/tay, pan ±70°), thay khay của phương án;
  tải khay 1 kg + khay 0,3 kg dồn ở góc xa như quy ước R1b. Tĩnh: a = 0. Động: biên (A = BASE_MAX_ACC + BASE_TURN_ACC_MAX,
  α = BASE_MAX_ALPHA, ω = BASE_MAX_W_TIP).
- Lift: lực răng = m (g + 0,5) trên bánh r = LIFT_PINION_R, % định mức STS3095, trần 70%.
"""
from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "sdk"), str(ROOT / "demos"), str(ROOT / "tests"), str(ROOT / "cad" / "faibotic")]
from faibotic import spec, limits, kinematics as K  # noqa: E402

G = spec.G
LIFTS = (0.0, 0.20, 0.45)
LIP = 0.008             # gờ khay
STEP = 0.02
LOAD_KG = 1.0           # tải khay quy ước (R1b)
TRAY_SHELL_KG = 0.3     # khay rỗng dồn cùng tải ở góc xa (quy ước tipover_dong)
J1 = spec.ARM_BASE_Z0   # độ cao đáy J1 ở lift 0

# Phương án: tâm khay (x, y), nửa kích thước (hx, hy), mặt khay z(lift); 'theo_lift' = khay đi cùng xe nâng.
# A: khay đế hiện tại (spec.TRAY_POS/TRAY_HALF). B: khay trên xe nâng, mặt khay thấp hơn đáy J1 0,17 m, x 0,20..0,32, y ±0,13
# (0,17 = H_J1_ABOVE_TABLE 0,12 + 0,05: khi tay làm việc trên bàn, mặt khay B thấp hơn mặt bàn 5 cm nên khay chui dưới bàn ;
# 0,08 thì khay cao hơn mặt bàn 4 cm, đập mép bàn khi tiến vào)
# (đã thử x 0,21..0,35: lật động 1,482 < 1,5 với 1 kg; 0,20..0,32 cho 1,502 và vẫn đạt 100% kẹp xuống).
# C: khay đế nâng lên cột phụ, mặt khay 0,40 m (thêm 0,18 m), cùng x, y như A.
B_TOP_BELOW_J1 = 0.17
C_TOP = 0.40
A_TOP = spec.TRAY_POS[2] + spec.TRAY_HALF[2]
OPTS = {
    "A": dict(ten="A: khay trên đế (hiện tại)", cx=spec.TRAY_POS[0], cy=0.0, hx=spec.TRAY_HALF[0], hy=spec.TRAY_HALF[1],
              top=lambda L: A_TOP, theo_lift=False, kg_kc=0.501, bom=0, cad_ngay=0.0),
    "B": dict(ten="B: khay gắn xe nâng, dưới-trước hai tay", cx=0.26, cy=0.0, hx=0.06, hy=0.13,
              top=lambda L: J1 + L - B_TOP_BELOW_J1, theo_lift=True, kg_kc=0.36, bom=100_000, cad_ngay=1.5),
    "C": dict(ten="C: khay đế nâng trên cột phụ", cx=spec.TRAY_POS[0], cy=0.0, hx=spec.TRAY_HALF[0], hy=spec.TRAY_HALF[1],
              top=lambda L: C_TOP, theo_lift=False, kg_kc=0.501 + 0.25, bom=150_000, cad_ngay=0.5),
}


def keepout(side, q, lift, o):
    """limits.keepout_violations với khay của phương án (thay tạm spec.TRAY_POS/TRAY_HALF trong lúc gọi, khôi phục ngay)."""
    old = spec.TRAY_POS, spec.TRAY_HALF
    hz = 0.0065
    spec.TRAY_POS, spec.TRAY_HALF = (o["cx"], o["cy"], o["top"](lift) - hz), (o["hx"], o["hy"], hz)
    try:
        return limits.keepout_violations(side, q, "di_dong", lift)
    finally:
        spec.TRAY_POS, spec.TRAY_HALF = old


def grid(o):
    xs = np.arange(o["cx"] - o["hx"] + LIP, o["cx"] + o["hx"] - LIP + 1e-9, STEP)
    ys = np.arange(-o["hy"] + LIP, o["hy"] - LIP + 1e-9, STEP)
    return [(float(x), float(y)) for x in xs for y in ys]


def place_q(x, y, lift, o, pitch, side):
    z = o["top"](lift) + spec.KEEPOUT_TRAY_MARGIN + 0.001
    p = np.array([x, y, z]) - K.arm_mount(side, lift)
    q = K.arm_ik(p, pitch=pitch, side=side)
    if q is None:
        return None
    if keepout(side, q, lift, o):
        return None
    return q


def scan(o, lift, pitch):
    """(tỉ lệ ô đạt, danh sách (x, y, side, q, tầm ngang từ trục pan))."""
    pts = grid(o)
    hit = []
    for x, y in pts:
        best = None
        for side in spec.SIDES:
            q = place_q(x, y, lift, o, pitch, side)
            if q is None:
                continue
            m = K.arm_mount(side, lift)
            r = math.hypot(x - m[0], y - m[1])
            if best is None or r < best[4]:
                best = (x, y, side, q, r)
        if best:
            hit.append(best)
    return len(hit) / len(pts), hit


def payload(hit):
    """(đỉnh min, đỉnh max, liên tục min, liên tục max) kg trên các ô đạt; None nếu không có mujoco hoặc không ô nào."""
    if not hit:
        return None
    try:
        import static_torque as st
    except Exception:
        return None
    pk = [st.max_payload(list(h[3]), "peak")[0] for h in hit]
    ct = [st.max_payload(list(h[3]), "continuous")[0] for h in hit]
    return min(pk), max(pk), min(ct), max(ct)


def tip(o, lift_top: bool, dyn: bool, load: float = LOAD_KG, extra=(), keep_base_tray: bool = False):
    import tipover as TP
    import tipover_dong as TD
    M, cfg, z_sh = TD._cfg(lift_top)
    M = copy.deepcopy(M)
    L = spec.LIFT_RANGE[1] if lift_top else 0.0
    cd = M["co_dinh"]
    kc = [cd.pop("M1-ST-010_tray"), cd.pop("M1-ST-011_tray_bracket")]
    fixed = [tuple(v) for v in cd.values()]
    top = o["top"](L)
    if o is OPTS["A"]:
        fixed += [tuple(v) for v in kc]
    elif o is OPTS["C"]:
        fixed += [(v[0], v[1], v[2], v[3] + (C_TOP - A_TOP)) for v in kc] + [(0.25, o["cx"] + o["hx"], 0.0, (A_TOP + C_TOP) / 2)]
    else:
        fixed += [(o["kg_kc"], o["cx"], 0.0, top - 0.01)] + ([tuple(v) for v in kc] if keep_base_tray else [])
    fixed += [tuple(v) for v in extra]
    E = TP.edges(cfg["R"], cfg["angles"])
    arm_m = max(cfg["arm_m"], spec.ARM_MASS_MOVING) + spec.BASE_TIP_ARM_EXTRA_KG
    A, al, w = ((spec.BASE_MAX_ACC + spec.BASE_TURN_ACC_MAX, spec.BASE_MAX_ALPHA, spec.BASE_MAX_W_TIP) if dyn else (0.0, 0.0, 0.0))
    pans = np.arange(-70.0, 70.0 + 1e-9, 10.0)
    res = math.inf
    for sy in (1, -1):
        f = fixed + [(TRAY_SHELL_KG + load, o["cx"] + o["hx"] - 0.02, sy * o["hy"], top - 0.0085)]   # A: đúng khay_goc_xa (0,30; ±0,15; 0,21)
        for pl in pans:
            Lp = TP.arm_points(cfg["pan_xy"], 1, pl, arm_m, cfg["arm_r"], cfg["load_m"], cfg["load_r"], z_sh)
            for pr in pans:
                Rp = TP.arm_points(cfg["pan_xy"], -1, pr, arm_m, cfg["arm_r"], cfg["load_m"], cfg["load_r"], z_sh)
                ms = np.array([tuple(v) + ((z_sh,) if len(v) == 3 else ()) for v in f + Lp + Rp], float)
                res = min(res, TD._factor(ms, E, A, al, w, 24))
    return res


def tip_max_load(o, target: float = spec.BASE_TIP_MIN) -> float:
    """Tải khay lớn nhất (kg, ở góc xa) để hệ số lật động ở biên, lift đỉnh, còn >= target (chia đôi 0..3 kg)."""
    f = lambda m: tip(o, True, True, m)
    if f(0.0) < target:
        return 0.0
    if f(3.0) >= target:
        return 3.0
    lo, hi = 0.0, 3.0
    for _ in range(12):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if f(mid) >= target else (lo, mid)
    return lo


ETA = 0.9   # hiệu suất ăn khớp bánh/thanh răng PETG [giả định, R3d], chờ đo DK-N9


def lift_pct(extra_kg: float, eta: float = 1.0) -> float:
    F = (spec.LIFT_DESIGN_LOAD_KG + extra_kg) * (G + 0.5)
    return 100 * F * spec.LIFT_PINION_R / (eta * spec.STS3095.rated_kgcm * spec.KGCM)


def lift_max_extra(eta: float = 1.0) -> float:
    cap = spec.LIFT_SERVO_LIMIT_PCT / 100 * spec.STS3095.rated_kgcm * spec.KGCM * eta
    return cap / spec.LIFT_PINION_R / (G + 0.5) - spec.LIFT_DESIGN_LOAD_KG


def vi(x, nd=2):
    return f"{x:.{nd}f}".replace(".", ",")


def main() -> int:
    out = {}
    print("## (1)+(2) Tầm đặt vật trên khay (lưới 2 cm, ô đạt nếu một tay đạt; TCP = mặt khay + 31 mm)\n")
    print("| Phương án | Lift (m) | Mặt khay (m) | Kẹp xuống: % diện tích | Kẹp ngang: % | Tầm từ trục pan (m), xuống | "
          "Tải đỉnh / liên tục (kg), xuống, min–max |")
    print("|---|---:|---:|---:|---:|---|---|")
    for k, o in OPTS.items():
        for L in LIFTS:
            fd, hd = scan(o, L, K.TOOL_DOWN)
            fh, hh = scan(o, L, 0.0)
            r = [h[4] for h in hd]
            pl = payload(hd)
            out[(k, L)] = dict(down=fd, hor=fh, r=(min(r), max(r)) if r else None, pl=pl)
            rs = f"{vi(min(r))}–{vi(max(r))}" if r else "–"
            ps = f"{vi(pl[0], 1)}–{vi(pl[1], 1)} / {vi(pl[2], 2)}–{vi(pl[3], 2)}" if pl else "–"
            print(f"| {k} | {vi(L)} | {vi(o['top'](L))} | {100 * fd:.0f}% | {100 * fh:.0f}% | {rs} | {ps} |")
    print()
    print(f"Tốc độ lift spec {spec.LIFT_SPEED_MAX * 1000:.1f} mm/s: hạ 0,20 m mất {0.20 / spec.LIFT_SPEED_MAX:.1f} s, "
          f"0,45 m mất {0.45 / spec.LIFT_SPEED_MAX:.1f} s (một chiều).\n")
    t_back = limits.base_move_time(0.12, 0.0, 0.0)
    print(f"B ở bàn: khay chui dưới mặt bàn 5 cm; nếu cần lộ cả khay thì lùi đế 0,12 m mất {t_back:.1f} s (limits.base_move_time, dốc).\n")
    print("## (2) Lật và lift\n")
    print(f"Lift: tải thiết kế {vi(spec.LIFT_DESIGN_LOAD_KG)} kg -> {lift_pct(0):.1f}% định mức; trần 70% còn dư "
          f"{vi(lift_max_extra())} kg.\n")
    print(f"Với η {vi(ETA, 1)} [giả định]: {lift_pct(0, ETA):.1f}%, còn dư {vi(lift_max_extra(ETA))} kg. Tải thiết kế đã gồm 0,5 kg/tay "
          f"(1,0 kg chung). B 'ngân sách chung': vật trên khay B + vật trên hai tay <= 1,0 kg, lift chỉ gánh thêm khung khay "
          f"{vi(OPTS['B']['kg_kc'])} kg: {lift_pct(OPTS['B']['kg_kc']):.1f}% (η 1) / {lift_pct(OPTS['B']['kg_kc'], ETA):.1f}% (η {vi(ETA, 1)}).\n")
    print("| Phương án | Lật tĩnh lift thấp / đỉnh | Lật động lift thấp / đỉnh | Lift %ĐM (khay + 1 kg cộng thêm), η 1 / η 0,9 | Tải khay max để lift ≤ 70% (kg), η 1 / η 0,9 | Tải khay max để lật động ≥ 1,5 (kg) |")
    print("|---|---:|---:|---:|---:|---:|")
    for k, o in OPTS.items():
        s0, s1 = tip(o, False, False), tip(o, True, False)
        d0, d1 = tip(o, False, True), tip(o, True, True)
        if o["theo_lift"]:
            pct = (lift_pct(o["kg_kc"] + LOAD_KG), lift_pct(o["kg_kc"] + LOAD_KG, ETA))
            mx = (lift_max_extra() - o["kg_kc"], lift_max_extra(ETA) - o["kg_kc"])
        else:
            pct, mx = (lift_pct(0), lift_pct(0, ETA)), None
        tm = tip_max_load(o)
        out[k] = dict(s=(s0, s1), d=(d0, d1), pct=pct, mx=mx, tm=tm)
        print(f"| {k} | {vi(s0, 3)} / {vi(s1, 3)} | {vi(d0, 3)} / {vi(d1, 3)} | {pct[0]:.1f}% / {pct[1]:.1f}% | "
              f"{'không gánh (khay trên đế)' if mx is None else vi(mx[0]) + ' / ' + vi(mx[1])} | {vi(tm)} |")
    print("\n| Phương án | BOM ± (VND) [ước] | Sửa CAD (ngày) [ước] |\n|---|---:|---:|")
    for k, o in OPTS.items():
        print(f"| {k} | {format(o['bom'], '+,').replace(',', '.')} | {vi(o['cad_ngay'], 1)} |")
    hai_tang()
    return 0


# ---------------- Bổ sung 24/11 tối: hai tầng (khay đế thấp cho máy tính/Jetson/pin dự phòng + khay B trên xe nâng) ----------------
# Khay đế giữ nguyên M1-ST-010 (mặt 0,218 m). Vật trên khay đế: khối điểm ở tâm hình học, z = mặt khay + nửa bề dày.
# Laptop đặt ngang, cạnh dài theo y; xét lệch về trước/sang bên trong giới hạn khay. [ước] cho khối lượng, kích thước.
BASE_ITEMS = {
    "laptop 2,5 kg trên khay đế hiện tại, tựa mép sau x 0,12 (tâm x 0,25)": [(2.5, 0.25, 0.0, A_TOP + 0.0125)],
    "laptop 2,5 kg, khay đế kéo dài ra sau tới x 0,05 (tâm x 0,18)": [(2.5, 0.18, 0.0, A_TOP + 0.0125)],
    "laptop 2,5 kg, tâm x 0,18 lệch bên 4 cm": [(2.5, 0.18, 0.04, A_TOP + 0.0125)],
    "laptop 2,5 kg, lệch trước + bên (x 0,26, y 0,04): cảnh báo": [(2.5, 0.26, 0.04, A_TOP + 0.0125)],
    "laptop 1,5 kg, tâm x 0,18": [(1.5, 0.18, 0.0, A_TOP + 0.0125)],
    "Jetson AGX Orin 1,1 kg (x 0,18) + pin dự phòng 0,6 kg (x 0,12)": [(1.1, 0.18, 0.0, A_TOP + 0.04), (0.6, 0.12, 0.0, A_TOP + 0.03)],
    "Jetson Orin NX/Nano 0,6 kg + DC-DC 0,15 kg (x 0,18)": [(0.6, 0.18, 0.0, A_TOP + 0.03), (0.15, 0.14, 0.0, A_TOP + 0.02)],
}
PACK_WH, USABLE, RUN_H = 172.8, 0.85, 2.0        # electrical/tinh-toan.md L3: dùng được 0,85 [giả định], 2 giờ <-> TB 6,62 A
DCDC_ETA = 0.90                                   # [giả định] buck/boost 12 V -> 19 V hoặc 12 V
JETSON_W = {"Orin Nano (7–15 W)": (7, 15), "Orin NX (10–25 W)": (10, 25), "AGX Orin (15–60 W)": (15, 60)}


def hai_tang():
    print("\n## (4) Hai tầng: khay đế (máy tính) + khay B trên xe nâng\n")
    print("Khay B mang 1 kg vật (+0,3 vỏ, góc xa), tay duỗi 1 kg/tay, khay đế giữ khung cũ 0,501 kg. Lật động: biên.\n")
    print("| Vật trên khay đế | Lật tĩnh lift thấp / đỉnh | Lật động lift thấp / đỉnh | ≥ 1,5? |")
    print("|---|---:|---:|---|")
    o = OPTS["B"]
    for k, items in BASE_ITEMS.items():
        r = [tip(o, lt, dy, extra=items, keep_base_tray=True) for dy in (False, True) for lt in (False, True)]
        print(f"| {k} | {vi(r[0], 3)} / {vi(r[1], 3)} | {vi(r[2], 3)} / {vi(r[3], 3)} | {'đạt' if min(r) >= spec.BASE_TIP_MIN else 'KHÔNG'} |")
    b_bot0 = o["top"](0.0) - 0.0145
    print(f"\nKhe đứng: đáy khay B ở lift 0 = {vi(b_bot0, 3)} m; mặt khay đế {vi(A_TOP, 3)} m. Laptop dày 25 mm -> đỉnh "
          f"{vi(A_TOP + 0.025, 3)} m: cần lift ≥ {vi(max(0.0, A_TOP + 0.025 + 0.02 - b_bot0), 3)} m (khe 20 mm) khi có laptop.\n")
    p_now = PACK_WH * USABLE / RUN_H
    print(f"## (5) Ngân sách điện Jetson (pack {vi(PACK_WH, 1)} Wh, dùng được {vi(USABLE)}, nền 2 giờ = {vi(p_now, 1)} W TB)\n")
    print("| Module | Công suất (W) thấp–cao | Lấy từ pack qua DC-DC η 0,9 (W) | Dòng ở 11,1 V (A) | Thời gian chạy (giờ) | Giảm |")
    print("|---|---|---|---|---|---|")
    for k, (lo, hi) in JETSON_W.items():
        a, b = lo / DCDC_ETA, hi / DCDC_ETA
        t = [PACK_WH * USABLE / (p_now + x) for x in (a, b)]
        print(f"| {k} | {lo}–{hi} | {vi(a, 1)}–{vi(b, 1)} | {vi(a / 11.1)}–{vi(b / 11.1)} | {vi(t[0])}–{vi(t[1])} | "
              f"{100 * (1 - t[0] / RUN_H):.0f}–{100 * (1 - t[1] / RUN_H):.0f}% |")


if __name__ == "__main__":
    raise SystemExit(main())
