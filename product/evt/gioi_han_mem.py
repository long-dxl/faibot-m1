"""Nạp và đọc lại giới hạn mềm MIN/MAX_ANGLE vào EEPROM servo tay (EVT-01, EVT-09; 06-lap-rap bước 9.2; phiếu IPQC).

Theo spec.ARM_LIMITS (không đổi thông số): vai −80…+48,5°, khuỷu −10…127°, pan ±70°, cổ tay −100…120°, xoay ±160°.
Giới hạn ghép vai–khuỷu–cổ tay (spec.elbow_max, wrist_max) servo không tự biết: SDK áp ở write_joints, kịch bản ghi lại để đối chiếu.
Chạy được trên bus thiếu servo (trước 3B: servo B rút): chỉ ghi servo trả lời ping. Mô-men PHẢI tắt.
TORQUE_LIMIT (vai 316‰, lift 166‰) là RAM: SDK ghi và đọc lại ở mỗi enable(), không nạp ở đây.
  --chi-doc   chỉ đọc lại và so (không ghi EEPROM)
"""
from __future__ import annotations

import math
import types

import evtlib as E
from evtlib import P, spec


def add_args(ap):
    ap.add_argument("--chi-doc", action="store_true")
    ap.add_argument("--dung-sai", type=float, default=0.2, help="độ, so đọc lại với spec")


def main(args) -> int:
    from faibotic import limits
    from faibotic.backends.feetech import FeetechBackend
    PAIRS = E.EVT_PAIRS
    ctx = E.Ctx(args, "gioi_han_mem", ["EVT-01", "EVT-09", "Q-FQC-C1"])
    cal = ctx.calibration()
    conv = types.SimpleNamespace(cal=cal)
    to_raw = lambda n, v: FeetechBackend.to_raw(conv, n, v)      # noqa: E731
    from_raw = lambda n, r: FeetechBackend.from_raw(conv, n, r)  # noqa: E731
    bus = ctx.bus()
    bus.retries = 1
    servos = []
    for s in spec.SIDES:
        for j in spec.ARM_JOINTS:
            n = spec.joint_name(s, j)
            servos.append((j, n))
            if n in PAIRS:
                servos.append((j, PAIRS[n]))
    present = [(j, n) for j, n in servos if bus.ping(E.EVT_IDS[n])]
    ctx.check("EVT-01", "servo tay trả lời", f"{len(present)}/{len(servos)}", "đủ 14 (trước 3B: 10)", None,
              note="thiếu: " + ", ".join(n for _, n in servos if (_, n) not in present) if len(present) < len(servos) else "")
    on = [n for _, n in present if bus.read(E.EVT_IDS[n], P.TORQUE_ENABLE)]
    if on:
        raise E.Abort(f"mô-men đang bật ở {on}: tắt (disable) trước khi ghi EEPROM")
    if not args.chi_doc:
        ctx.confirm("Ghi EEPROM MIN/MAX_ANGLE cho servo tay theo spec.py (ghi có giới hạn số lần: chỉ chạy khi cần)")
    rows, bad = {}, []
    for j, n in present:
        sid = E.EVT_IDS[n]
        lo, hi = spec.ARM_LIMITS[j]
        a, b = sorted((to_raw(n, lo), to_raw(n, hi)))
        want = (max(0, a), min(4095, b))
        if not args.chi_doc:
            bus.unlock_eeprom(sid)
            try:
                bus.write(sid, P.MIN_ANGLE, want[0])
                bus.write(sid, P.MAX_ANGLE, want[1])
            finally:
                bus.lock_eeprom(sid)
        got = (bus.read(sid, P.MIN_ANGLE), bus.read(sid, P.MAX_ANGLE))
        deg = tuple(sorted(round(math.degrees(from_raw(n, r)), 2) for r in got))
        spec_deg = (round(math.degrees(lo), 2), round(math.degrees(hi), 2))
        clipped = want != (a, b)
        ok = all(abs(g - w) <= args.dung_sai for g, w in zip(deg, spec_deg))
        rows[n] = dict(id=sid, raw_doc=got, raw_muon=want, do_doc=deg, do_spec=spec_deg, bi_cat_bien_encoder=clipped, lock=bus.read(sid, P.LOCK))
        if not ok:
            bad.append(n)
        ctx.say(f"{n:18} ID {sid:2} raw {got} -> {deg}° (spec {spec_deg}°){' CẮT BIÊN 0/4095: chỉnh zero' if clipped else ''}")
    ctx.data.update(servo=rows, sdk_ghep=limits.loaded())
    ctx.check("EVT-01", "MIN/MAX_ANGLE đọc lại khớp spec.ARM_LIMITS", f"{len(present) - len(bad)}/{len(present)}", f"tất cả, ±{args.dung_sai}°",
              not bad and bool(present), note=("lệch: " + ", ".join(bad)) if bad else "")
    unlocked = [n for n, r in rows.items() if r["lock"] != 1]
    ctx.check("EVT-01", "EEPROM khoá lại (LOCK = 1)", len(unlocked), "0 servo mở khoá", not unlocked)
    e = ctx.ask("khuyu_127_khong_cham", "EVT-09: chạy chậm khuỷu tới 127° (vai ≤ 0°), soi khe: không giao cắt? (c/k)", True, bool)
    ctx.check("EVT-09", "khuỷu ≤ 127° không giao cắt (mẫu in thật)", e, "có", e)
    c1 = ctx.ask("c1_du_dai", "Q-FQC-C1: từng khớp chạy chậm tới giới hạn, đủ dải, không kẹt, không lỗi bus? (c/k)", True, bool)
    ctx.check("Q-FQC-C1", "dải khớp, chạy trơn", c1, "có", c1)
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
