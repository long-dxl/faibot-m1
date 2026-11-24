"""Tải 0,5 kg mỗi tay giữ tĩnh + mô-men vai/khuỷu + nhiệt servo (EVT-07; Q-FQC-C4, Q-FQC-C5, Q-FQC-05).

Điều kiện vào (06-lap-rap bước 10): quay_nguoc.py đạt, cân bằng trọng lực đã lắp, giới hạn mềm đã nạp, 3B xong.
Tư thế duỗi ngang (vai 0, khuỷu 0, cổ tay 0). Quả cân 0,5 kg treo tại điểm kẹp. Giữ 60 s đọc tải cặp vai (C4), tiếp tới
--phut phút đọc nhiệt qua bus (C5); người đo nhập nhiệt kế hồng ngoại và độ võng đồng hồ so (Q-FQC-05).
Mô-men quy từ PRESENT_LOAD (‰ mô-men chặn 95 kg·cm) [giả định tuyến tính]: chỉ để so tương đối, đối chiếu cân lực khi có.
"""
from __future__ import annotations

import evtlib as E
from evtlib import P, spec


def add_args(ap):
    ap.add_argument("--tay", choices=spec.SIDES, default="left")
    ap.add_argument("--phut", type=float, default=10.0)


def main(args) -> int:
    from faibotic.backends.feetech import PairFault
    PAIRS = E.EVT_PAIRS
    ctx = E.Ctx(args, "tai_nhiet", ["EVT-07", "Q-FQC-C4", "Q-FQC-C5", "Q-FQC-05"])
    s = args.tay
    sh, el = spec.joint_name(s, "shoulder"), spec.joint_name(s, "elbow")
    ctx.confirm("quay_nguoc đạt, lò xo cân bằng đã lắp, giới hạn mềm đã nạp; tay buộc dây chặn; không ai dưới tay")
    r = ctx.robot()
    be = r.backend
    ids = [E.EVT_IDS[n] for n in (sh, PAIRS[sh], el, PAIRS[el])]
    loads, temps, err = [], {i: 0 for i in ids}, None
    try:
        r.move_joints({sh: 0.0, el: 0.0, spec.joint_name(s, "wrist_flex"): 0.0}, duration=4.0)
        def hang():
            for n, v in ((sh, 120), (PAIRS[sh], 115), (el, 90), (PAIRS[el], 88)):
                ctx.port.set_load(E.EVT_IDS[n], v * be.cal[n]["sign"])
        ctx.action("Treo quả cân 0,5 kg tại điểm kẹp, buông tay từ từ", hang if ctx.dry else None)
        t0 = ctx.now()
        while ctx.now() - t0 < args.phut * 60:
            r.step()
            if ctx.now() - t0 <= 60:
                pl = be.read_pair_loads()
                loads.append((pl[sh], pl[el]))
            if int(ctx.now() - t0) % 10 == 0:
                for i, v in be.bus.sync_read(P.PRESENT_TEMPERATURE, ids).items():
                    temps[i] = max(temps[i], v)
            ctx.sleep(0.2)
    except PairFault as e:
        err = str(e)
    finally:
        r.close()
    kg = lambda nm: abs(nm) / spec.KGCM   # noqa: E731
    if loads:
        tot_sh = max(kg(a) + kg(b) for (a, b), _ in loads)
        tot_el = max(kg(a) + kg(b) for _, (a, b) in loads)
        diff = max(abs(abs(a) - abs(b)) / max(0.5 * (abs(a) + abs(b)), 1e-9) for (a, b), _ in loads)
    else:
        tot_sh = tot_el = diff = None
    ctx.data.update(loads_nm=loads[-1] if loads else None, temps_max=temps, pair_fault=err)
    ctx.check("Q-FQC-C4", f"mô-men vai {s} tổng cặp, 0,5 kg, duỗi ngang (lớn nhất 60 s)", tot_sh, "≤ 36 kg·cm", None if tot_sh is None else tot_sh <= 36 and not err, "kg·cm", err or "")
    ctx.check("Q-FQC-C4", "chênh tải hai servo vai", None if diff is None else 100 * diff, "≤ 20% [giả định]", None if diff is None else diff <= 0.20, "%")
    ctx.check("EVT-07", f"mô-men khuỷu {s} tổng cặp", tot_el, f"ghi nhận (định mức cặp {2 * spec.STS3095.rated_kgcm:.0f} kg·cm)", None, "kg·cm")
    tmax = max(temps[E.EVT_IDS[sh]], temps[E.EVT_IDS[PAIRS[sh]]])
    ctx.check("Q-FQC-C5", f"nhiệt servo vai qua bus sau {args.phut:.0f} phút", tmax, "≤ 60 °C [giả định]", tmax <= 60 if args.phut >= 10 else None, "°C",
              "" if args.phut >= 10 else "chưa đủ 10 phút")
    ir = ctx.ask("nhiet_ir", "Nhiệt kế hồng ngoại vỏ servo vai nóng nhất (°C)", tmax + 2)
    ctx.check("Q-FQC-C5", "nhiệt vỏ servo vai (hồng ngoại)", ir, "≤ 60 °C [giả định]", None if ir is None else ir <= 60, "°C")
    vong = ctx.ask("vong_mm", "Độ võng đầu kẹp khi treo 0,5 kg, 60 s (đồng hồ so, mm)", 2.1)
    ctx.check("Q-FQC-05", f"tay {s} giữ 0,5 kg: võng", vong, "≤ 5 mm [giả định], không trượt", None if vong is None else vong <= 5 and not err, "mm")
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
