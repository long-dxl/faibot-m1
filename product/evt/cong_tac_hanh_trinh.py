"""Thử công tắc hành trình trục nâng (QC2-cot-nang #4: chưa có GPIO thật; Q-FQC-C8).

Không chạy servo lift (chờ lift_da_vong.py có kết luận). Bước:
 1. Đọc hai công tắc (GPIO Pi: --pin LIFT_BOT=.. --pin LIFT_TOP=..) ở trạng thái nhả. Mặc định theo bảng đi dây: công tắc NC
    nối GND, PiGPIO kéo lên LIFT_* (cùng cấu hình SDK FAIBOTIC_GPIO_PULL/ACTIVE_LOW) -> nhả = thấp, chạm hoặc đứt dây = cao
    (--muc-cham cao). Công tắc NO: --muc-cham thap.
 2. Người đo nhấn tay từng công tắc (hoặc đẩy xe nâng bằng tay tới công tắc, servo tắt mô-men): kịch bản thấy đổi mức, đếm dội.
 3. Kiểm logic chặn của SDK (limits.lift_gate) với số đọc thật: chạm dưới thì không cho đi xuống, chạm trên không cho đi lên.
 4. Người đo nhập khoảng cách công tắc tới đệm PU chặn cứng (dưới, trên) so spec.LIFT_HARD_STOP_MARGIN.
"""
from __future__ import annotations

import evtlib as E
from evtlib import spec


class SimSwitch(E.FakeGPIO):
    """Công tắc giả theo cách đi dây: touch_level là mức đọc khi chạm (NC + kéo lên: True)."""
    def __init__(self, touch_level: bool = True):
        self.touch_level = touch_level
        super().__init__(LIFT_BOT=not touch_level, LIFT_TOP=not touch_level)

    def touch(self, pin: str, on: bool) -> None:
        self.levels[pin] = self.touch_level if on else not self.touch_level


class GpioSwitches:
    """(dưới, trên), True = chạm. touch_high: mức cao = chạm (NC + kéo lên, mặc định); False: NO, mức thấp = chạm."""
    def __init__(self, g, touch_high: bool = True):
        self.g, self.hi = g, touch_high

    def read(self):
        b, t = self.g.read("LIFT_BOT"), self.g.read("LIFT_TOP")
        return (b, t) if self.hi else (not b, not t)


def add_args(ap):
    ap.add_argument("--muc-cham", choices=("cao", "thap"), default="cao",
                    help="mức GPIO khi công tắc chạm: cao = công tắc NC nối GND + kéo lên (bảng đi dây, mặc định); thap = NO")
    ap.add_argument("--timeout", type=float, default=30.0)


def main(args) -> int:
    from faibotic import limits
    ctx = E.Ctx(args, "cong_tac_hanh_trinh", ["QC2-cot-nang #4", "Q-FQC-C8"], dict(ids={}))
    hi = args.muc_cham == "cao"
    sim = SimSwitch(touch_level=hi) if ctx.dry else None
    g = ctx.gpio(lambda: sim, required=("LIFT_BOT", "LIFT_TOP"))
    sw = GpioSwitches(g, touch_high=hi)
    idle = sw.read()
    ctx.check("QC2-cot-nang #4", "trạng thái nhả (dưới, trên)", idle, "(False, False)", idle == (False, False))
    for name, idx, pin in (("dưới", 0, "LIFT_BOT"), ("trên", 1, "LIFT_TOP")):
        ctx.action(f"Nhấn và GIỮ công tắc {name} (servo lift tắt mô-men)", (lambda p=pin: sim.touch(p, True)) if sim else None)
        t0, seen, flips, prev = ctx.now(), False, 0, sw.read()[idx]
        while ctx.now() - t0 < (0.3 if ctx.dry else args.timeout):
            v = sw.read()[idx]
            flips += v != prev
            prev = v
            if v:
                seen = True
                if ctx.dry:
                    break
            ctx.sleep(0.001)
        ctx.check("QC2-cot-nang #4", f"công tắc {name} đọc được khi chạm", seen, "có", seen, note=f"đổi mức {flips} lần (dội)")
        cur = 0.25
        gate_dn = limits.lift_gate(cur, cur - 0.01, sw)
        gate_up = limits.lift_gate(cur, cur + 0.01, sw)
        ok = (gate_dn == cur and gate_up > cur) if idx == 0 else (gate_up == cur and gate_dn < cur)
        ctx.check("QC2-cot-nang #4", f"SDK lift_gate với công tắc {name} chạm thật", ok, "chặn đúng chiều, cho chiều ngược", ok)
        ctx.action(f"Nhả công tắc {name}", (lambda p=pin: sim.touch(p, False)) if sim else None)
    for k, name in enumerate(("dưới", "trên")):
        m = ctx.ask(f"khe_{k}", f"Khoảng cách công tắc {name} tác động tới đệm PU chặn cứng (mm)", spec.LIFT_HARD_STOP_MARGIN[k] * 1000)
        want = spec.LIFT_HARD_STOP_MARGIN[k] * 1000
        ctx.check("Q-FQC-C8", f"khe công tắc–đệm {name}", m, f"ghi nhận, so thiết kế {want:.0f} mm", None, "mm",
                  "" if m is None else f"lệch {m - want:+.1f} mm so CAD; nhỏ hơn nhiều thì chỉnh vị trí công tắc trước khi chạy lift")
    ctx.data["ghi_chu"] = ("SDK: FeetechBackend lấy công tắc từ FAIBOTIC_GPIO_PINS (gpio_pi.PiLiftSwitches, NC + kéo lên); "
                           f"kịch bản chạy --muc-cham {args.muc_cham}; lift_gate so vị trí LỆNH (_cmd), chưa so vị trí đọc")
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
