"""Đo dòng tĩnh và teleop (EVT-05, EVT-11, EVT-15 ước tính; Q-FQC-13). Nguồn số: INA226 trên PCB (U1 0x40 bus servo sau K1,
U2 0x41 đầu vào buck) qua I2C Pi, hoặc đồng hồ/kẹp dòng (--nguon dong-ho, người đo nhập). Bộ #0 (--mach bang-di-day) không có
INA226: mặc định dong-ho (kẹp dòng DC, electrical/bang-di-day-evt.md mục 5).

Pha: (1) tĩnh, mô-men tắt (K1 đóng, 20 servo cấp điện); (2) tĩnh, mô-men bật giữ tư thế zero; (3) teleop: kịch bản nhả bus,
người đo chạy teleop ở cửa sổ khác trong --teleop-s giây, kịch bản chỉ lấy mẫu INA226 (I2C không chung cổng bus servo).
"""
from __future__ import annotations

import statistics

import evtlib as E

I_AVG_MAX_2H = 8.27   # A, dòng trung bình tối đa để chạy ≥ 120 phút (Q-FQC-09, pack 3S5P 216 Wh) [tự tính]


class SimINA:
    def __init__(self, ctx, base_a):
        self.ctx, self.base = ctx, base_a

    def read(self):
        import math
        t = self.ctx.now()
        return 11.9 - 0.02 * self.base, self.base * (1 + 0.3 * math.sin(7 * t))


def add_args(ap):
    ap.add_argument("--nguon", choices=("ina226", "dong-ho"), default=None, help="mặc định: dong-ho với bộ #0, ina226 với PCB")
    ap.add_argument("--i2c", type=int, default=1)
    ap.add_argument("--mau-s", type=float, default=20.0, help="giây lấy mẫu mỗi pha tĩnh")
    ap.add_argument("--teleop-s", type=float, default=300.0, help="giây lấy mẫu pha teleop")


def sample(ctx, ina, secs, hz=50):
    v, i, t0 = [], [], ctx.now()
    while ctx.now() - t0 < secs:
        a, b = ina.read()
        v.append(a)
        i.append(b)
        ctx.sleep(1 / hz)
    return dict(i_tb=statistics.fmean(i), i_dinh=max(i), v_min=min(v), n=len(i))


def main(args) -> int:
    ctx = E.Ctx(args, "dong_dien", ["EVT-05", "EVT-11", "EVT-15", "Q-FQC-13"])
    res = {}
    if args.nguon is None:
        args.nguon = "dong-ho" if ctx.hw == "bang_di_day" else "ina226"
    if args.nguon == "ina226" and ctx.hw == "bang_di_day" and not ctx.dry:
        raise SystemExit("bộ #0 (bảng đi dây) không có INA226: dùng --nguon dong-ho (kẹp dòng DC)")
    if args.nguon == "ina226" and not ctx.dry:
        u1 = E.INA226(0x40, 0.001, 0.0025, args.i2c)
        u2 = E.INA226(0x41, 0.020, 0.0002, args.i2c)
    for phase, sim_a, enable in (("tinh_tat", 0.9, False), ("tinh_giu", 2.4, True)):
        r = ctx.robot(enable=enable) if enable else None
        try:
            if args.nguon == "dong-ho":
                res[phase] = dict(i_tb=ctx.ask(f"{phase}_a", f"{phase}: dòng bus servo trung bình (A)", sim_a), i_dinh=None, v_min=None)
            else:
                res[phase] = sample(ctx, SimINA(ctx, sim_a) if ctx.dry else u1, args.mau_s)
        finally:
            if r:
                r.close()
        ctx.say(f"{phase}: {res[phase]}")
    ctx.action(f"Chạy teleop điển hình (hai tay + đế + nâng nếu đã đạt) ở cửa sổ khác trong {args.teleop_s:.0f} s, rồi Enter")
    if args.nguon == "dong-ho":
        res["teleop"] = dict(i_tb=ctx.ask("teleop_a", "teleop: dòng trung bình (A)", 6.5), i_dinh=ctx.ask("teleop_dinh", "teleop: dòng đỉnh (A)", 28.0),
                             v_min=ctx.ask("teleop_vmin", "teleop: điện áp hub tay thấp nhất (V)", 11.3))
        buck = ctx.ask("buck_a", "dòng đầu vào buck (Pi, camera) trung bình (A)", 0.9)
    else:
        res["teleop"] = sample(ctx, SimINA(ctx, 6.5) if ctx.dry else u1, args.teleop_s)
        buck = sample(ctx, SimINA(ctx, 0.9) if ctx.dry else u2, min(args.mau_s, 10))["i_tb"]
    ctx.data.update(res, buck_a=buck)
    it = res["tinh_tat"]["i_tb"]
    ctx.check("EVT-11", "I tĩnh 20 servo, mô-men tắt (tổng; /20 = mỗi servo)", it, "ghi nhận (N2 mở)", None, "A",
              "" if it is None else f"{1000 * it / 20:.0f} mA/servo")
    ctx.check("EVT-05", "dòng giữ tư thế, mô-men bật", res["tinh_giu"]["i_tb"], "ghi nhận", None, "A")
    tl = res["teleop"]
    ctx.check("EVT-05", "dòng teleop trung bình / đỉnh", None if tl["i_tb"] is None else f"{tl['i_tb']:.2f} / {tl['i_dinh']:.2f}" if tl['i_dinh'] is not None else f"{tl['i_tb']:.2f}", "ghi nhận; thay hệ số 0,4 giả định", None, "A")
    tot = None if tl["i_tb"] is None or buck is None else tl["i_tb"] + buck
    ctx.check("EVT-15", "dòng trung bình toàn robot khi teleop (servo + buck)", tot, f"≤ {I_AVG_MAX_2H} A để ≥ 120 phút [tự tính]",
              None if tot is None else tot <= I_AVG_MAX_2H, "A", "ước tính; ký EVT-15 bằng chạy pin thật (ghi_tay.py)")
    ctx.check("Q-FQC-13", "điện áp bus servo thấp nhất khi teleop", tl["v_min"], "≥ 10,8 V [giả định]", None if tl["v_min"] is None else tl["v_min"] >= 10.8, "V")
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
