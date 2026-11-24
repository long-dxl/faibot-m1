"""SafetyLatch cấu hình "bang_di_day" (bộ #0) trên mạch rơ-le giả lập, theo từng dòng bảng trạng thái
electrical/bang-di-day-evt.md mục 2. Đồng hồ ảo; người bấm nút là lịch sự kiện theo thời gian."""
import threading

import pytest

from faibotic.safety import DockSwitches, FakeGPIO as _DockGPIO  # S2: khoá trạm đóng (cấu hình di_dong bắt buộc DOCK)
from faibotic.feetech.mockbus import FakeServoPort

from faibotic import spec
from faibotic.safety import (BangDiDayLatch, FakeGPIO, PcbLatch, ResetRefused, SafetyLatch, SimBangDiDay,
                             resolve_hw)


class Bench:
    """Đồng hồ ảo + mạch giả + lịch thao tác. Mỗi bước sleep kiểm bất biến: đang 'run' thì K1 phải đóng thật."""

    def __init__(self, latch_kw=None, **sim_kw):
        self.t = 0.0
        self.sim = SimBangDiDay(clock=lambda: self.t, **sim_kw)
        self.events: list[tuple[float, callable]] = []
        self.stops: list[float] = []
        self.violations: list[str] = []
        self.latch_kw = dict(servo_discharge_s=self.sim.cap_hold_s) | (latch_kw or {})   # "đã đo" = đúng thời gian tụ giả
        self.lt = self.new_latch()

    def new_latch(self, **kw):
        self.lt = SafetyLatch(self.sim, on_stop=lambda: self.stops.append(self.t), clock=lambda: self.t,
                              sleep=self.sleep, reset_timeout_s=5.0, **(self.latch_kw | kw))
        return self.lt

    def reset(self, **kw):
        """reset() rồi kiểm bất biến NGAY khi trả về (không chỉ trong sleep)."""
        try:
            self.lt.reset(**kw)
        finally:
            if self.lt.ok and not self.sim.k1:
                self.violations.append(f"t={self.t:.3f}: reset() trả run khi K1 hở")

    def at(self, dt: float, fn) -> None:
        self.events.append((self.t + dt, fn))
        self.events.sort(key=lambda e: e[0])

    def sleep(self, dt: float) -> None:
        end = self.t + dt
        while self.events and self.events[0][0] <= end:
            te, fn = self.events.pop(0)
            self.t = max(self.t, te)
            fn()
        self.t = end
        if self.lt.ok and not self.sim.k1:
            self.violations.append(f"t={self.t:.3f}: run khi K1 hở")

    def run_for(self, secs: float, step: float = 0.01) -> None:
        n = int(round(secs / step))
        for _ in range(n):
            self.sleep(step)
            self.lt.poll()

    def operator_click(self, hold_s: float = 0.2, after_s: float = 0.3):
        """on_wait cho reset(): sau after_s người nhấn Reset, giữ hold_s rồi nhả."""
        def go():
            self.at(after_s, lambda: self.sim.press("reset", True))
            self.at(after_s + hold_s, lambda: self.sim.press("reset", False))
        return go

    def start(self) -> None:
        self.reset(on_wait=self.operator_click())
        assert self.lt.ok and self.sim.k1 and not self.violations


def test_config_selection():
    assert spec.SAFETY_HW == "bang_di_day"
    assert resolve_hw(None) == "bang_di_day" and resolve_hw(FakeGPIO()) == "pcb"
    assert resolve_hw(FakeGPIO(), "bang-di-day") == "bang_di_day"
    assert isinstance(SafetyLatch(FakeGPIO()), PcbLatch)
    b = Bench()
    assert isinstance(b.lt, BangDiDayLatch)
    with pytest.raises(ValueError):
        resolve_hw(None, "khac")


# ---- dòng 1: D0 bật, chưa Reset ----
def test_row01_startup_k5_low_no_auto_run():
    b = Bench()
    b.sim.k5 = True                          # K5 đang cao từ trước (Pi khởi động lại): latch phải tự hạ
    b.new_latch()
    assert b.sim.k5 is False and b.lt.state == "stopped"
    b.sim.click_reset()                      # người bấm Reset khi phần mềm chưa cho phép: K2 không chốt
    b.run_for(1.0)
    assert not b.sim.run and not b.sim.k1 and not b.lt.ok and b.lt.fault is None


# ---- dòng 2–4: nhấn Reset < 0,5 s, > 0,5 s, nhả -> chạy ----
def test_rows02_04_reset_sequence_torque_only_after_arm_sense():
    b = Bench()
    seen = {}

    def probe(key):
        return lambda: seen.setdefault(key, (b.lt.state, b.sim.k2, b.sim.k4, b.sim.k1, b.sim.arm))
    b.at(0.0, lambda: None)
    b.reset(on_wait=lambda: (b.operator_click(hold_s=0.8, after_s=0.1)(),
                                b.at(0.3, probe("giu_0_2s")), b.at(0.75, probe("giu_0_65s")), b.at(0.95, probe("vua_nha"))))
    assert seen["giu_0_2s"] == ("arming", True, True, False, True)      # nạp trước qua R_pc, chưa chạy
    assert seen["giu_0_65s"][:4] == ("arming", True, False, False)      # KT xong, K4 nhả, K1 chưa (Reset đang giữ)
    assert seen["vua_nha"][0] == "arming" and seen["vua_nha"][3]        # K1 đóng nhưng chưa đủ arm_confirm_s
    assert b.lt.ok and b.sim.k1 and not b.violations
    t_run = b.lt.log[-1][0]
    assert t_run - 0.9 >= spec.EVT_ARM_CONFIRM_S - 1e-9                 # chạy sau khi ARM_SENSE cao đủ lâu kể từ lúc nhả


def test_precharge_alone_never_confirms():
    """Tụ giữ điện lâu (cap_hold 0,25 s) + Reset kẹt: ARM_SENSE cao qua KT nhưng chưa đủ arm_confirm_s -> không chạy."""
    b = Bench(cap_hold_s=0.25)
    with pytest.raises(ResetRefused):
        b.reset(on_wait=lambda: b.at(0.1, lambda: b.sim.press("reset", True)), timeout_s=3.0)
    assert not b.violations and not b.sim.k1


def test_k5_raised_before_and_kept_during_reset_press():
    """DK-F9: K5 phải cao TRƯỚC và SUỐT lúc người bấm Reset; reset() không hạ K5 trước khi đọc."""
    b = Bench()
    k5_at_press = []
    b.reset(on_wait=lambda: (b.at(0.2, lambda: (k5_at_press.append(b.sim.k5), b.sim.press("reset", True))),
                                b.at(0.4, lambda: (k5_at_press.append(b.sim.k5), b.sim.press("reset", False)))))
    assert k5_at_press == [True, True] and b.sim.history == [("K5_EN", False), ("K5_EN", True)]


def test_software_never_closes_without_human():
    b = Bench()
    with pytest.raises(ResetRefused, match="không thấy K1"):
        b.reset(timeout_s=3.0)            # không ai bấm Reset
    assert not b.sim.k1 and not b.sim.run and b.sim.k5 is False and not b.lt.ok


# ---- dòng 5–6: E-stop khi chạy, nhả E-stop ----
def test_rows05_06_estop_while_running_then_release_stays_off():
    b = Bench()
    b.start()
    b.sim.press("estop", True)
    assert not b.lt.poll()
    assert b.sim.k5 is False and len(b.stops) == 1 and b.lt.state == "estop" and not b.sim.k1
    b.sim.press("estop", False)
    b.run_for(2.0)
    assert not b.lt.ok and not b.sim.k1 and not b.sim.arm and b.lt.fault is None
    b.sim.click_reset()                      # Reset tay mà phần mềm chưa reset(): K5 hạ nên không chốt
    b.run_for(1.0)
    assert not b.sim.k1
    b.start()                                # đúng quy trình: reset() rồi người bấm Reset
    assert not b.violations


def test_reset_refused_while_estop_pressed_times_out():
    b = Bench()
    b.sim.press("estop", True)
    with pytest.raises(ResetRefused):
        b.reset(on_wait=b.operator_click(), timeout_s=2.0)
    assert b.sim.k5 is False and not b.sim.k1


# ---- dòng 7: Reset kẹt ----
def test_row07_reset_stuck_bench_without_reset_sense():
    """Thử bàn (bench=True, không servo, không RESET_SENSE): Reset kẹt vẫn không chạy (hết giờ)."""
    with pytest.warns(RuntimeWarning, match="THỬ BÀN"):
        b = Bench(latch_kw=dict(bench=True, reset_pin=None), reset_sense=False)
    b.sim.press("reset", True)               # kẹt từ trước
    with pytest.raises(ResetRefused, match="Reset"):
        b.reset(timeout_s=3.0)
    assert not b.sim.k1 and b.sim.k5 is False and not b.sim.arm and not b.lt.ok and not b.violations


def test_missing_reset_sense_refused_before_k5():
    b = Bench(latch_kw=dict(reset_pin=None), reset_sense=False)
    with pytest.raises(ResetRefused, match="RESET_SENSE"):
        b.reset(on_wait=b.operator_click())
    assert b.sim.history == [("K5_EN", False)]


def test_unmeasured_discharge_refused_before_k5():
    b = Bench(latch_kw=dict(servo_discharge_s=None))
    with pytest.raises(ResetRefused, match="chưa đo"):
        b.reset(on_wait=b.operator_click())
    assert b.sim.history == [("K5_EN", False)]


def test_row07_reset_stuck_with_reset_sense_refused_immediately():
    b = Bench()
    b.sim.press("reset", True)
    with pytest.raises(ResetRefused, match="kẹt"):
        b.reset()
    assert b.sim.k5 is False and b.sim.history[-1] == ("K5_EN", False)


def test_row07_reset_stuck_after_press_never_runs():
    b = Bench()
    with pytest.raises(ResetRefused):
        b.reset(on_wait=lambda: b.at(0.2, lambda: b.sim.press("reset", True)), timeout_s=3.0)
    assert not b.sim.k1 and not b.violations


# ---- dòng 8: Pi tắt K5 / Pi treo ----
def test_row08_pi_drops_k5_cuts_k1():
    b = Bench()
    b.start()
    b.lt.trip("thử rút K5")
    assert not b.sim.k2 and not b.sim.k1 and len(b.stops) == 1
    b.run_for(1.0)
    assert not b.lt.ok and b.lt.fault is None


def test_row08_monitor_thread_death_trips():
    b = Bench()
    b.start()
    calls = {"n": 0}
    orig = b.sim.read

    def bad(pin):
        calls["n"] += 1
        if calls["n"] > 3:
            raise OSError("gpiod lỗi")
        return orig(pin)
    b.sim.read = bad
    b.lt.start_monitor(0.001)
    b.lt._thread.join(2.0)
    assert not b.lt.monitor_alive and b.sim.k5 is False and not b.lt.ok and "gpiod" in b.lt.monitor_error


# ---- dòng 9: Q6 nhảy / mất 12 V ----
def test_row09_loss_of_12v():
    b = Bench()
    b.start()
    b.sim.press("v12", True)
    assert not b.lt.poll() and b.sim.k5 is False and not b.sim.k1
    b.sim.press("v12", False)                # có lại 12 V: không tự đóng
    b.run_for(2.0)
    assert not b.sim.k1 and not b.lt.ok and b.lt.fault is None


def test_full_power_loss_pi_restart_requires_reset():
    b = Bench()
    b.start()
    b.sim.press("v12", True)
    b.sim.k5 = False                         # mất cả Pi: chân về thấp
    b.sim.press("v12", False)
    b.new_latch()
    b.run_for(1.0)
    assert not b.lt.ok and not b.sim.k1
    b.start()


# ---- dòng 10: K1 dính, nhấn E-stop ----
def test_row10_k1_welded_detected_after_estop():
    b = Bench()
    b.start()
    b.sim.weld.add("K1")
    b.sim.press("estop", True)
    b.run_for(0.1)
    assert b.lt.state == "estop" and b.lt.fault is None   # chưa đủ stuck_s
    b.run_for(0.2)
    assert b.lt.state == "fault" and "K1" in b.lt.fault and b.sim.arm and b.sim.k5 is False
    b.sim.press("estop", False)
    with pytest.raises(ResetRefused, match="K1"):
        b.reset(on_wait=b.operator_click())
    with pytest.raises(ResetRefused):
        b.lt.clear_fault()
    assert b.sim.history.count(("K5_EN", True)) == 1         # không bật lại K5 khi đang lỗi


def test_row10_k1_welded_at_startup_refuses_reset():
    b = Bench()
    b.sim.weld.add("K1")
    with pytest.raises(ResetRefused, match="K1"):
        b.reset(on_wait=b.operator_click())
    b.run_for(0.3)
    assert b.lt.state == "fault" and not b.lt.ok


def test_fault_clear_after_repair():
    b = Bench()
    b.start()
    b.sim.weld.add("K1")
    b.sim.press("estop", True)
    b.run_for(0.5)
    assert b.lt.state == "fault"
    b.sim.weld.clear()
    b.sim.press("estop", False)
    b.run_for(0.1)
    b.lt.clear_fault()
    b.start()


# ---- dòng 11: K4 dính, dừng ----
def test_row11_k4_welded_detected():
    b = Bench()
    b.start()
    b.sim.weld.add("K4")
    b.sim.press("estop", True)
    b.run_for(0.5)
    assert b.lt.state == "fault" and "K4" in b.lt.fault and b.sim.k4 and not b.sim.k1


# ---- dòng 12: K2 dính ----
def test_row12_k2_welded_detected():
    b = Bench()
    b.start()
    b.sim.weld.add("K2")
    b.sim.press("estop", True)
    assert not b.sim.k1 and b.sim.run                        # NC2 vẫn cắt K1; RUN còn cao (K2.2 dính)
    b.run_for(0.05)
    assert b.lt.state == "estop"                             # chỉ ARM_SENSE rớt mới báo dừng
    b.run_for(0.5)
    assert b.lt.state == "fault" and "K2" in b.lt.fault
    b.sim.press("estop", False)                              # K1 tự đóng lại không cần Reset: phần mềm vẫn chặn
    b.run_for(1.0)
    assert b.sim.k1 and not b.lt.ok
    with pytest.raises(ResetRefused):
        b.reset()


def test_k5_welded_human_reset_detected():
    b = Bench()
    b.sim.weld.add("K5")
    b.sim.click_reset()                                      # K5 dính + người bấm Reset: chốt dù Pi không cho phép
    b.run_for(1.0)
    assert b.lt.state == "fault" and not b.lt.ok and len(b.stops) >= 1


# ---- dòng 13: KT dính ----
def test_row13_kt_welded_runs_without_precharge():
    """Không mất dừng; phần mềm không phân biệt được (chỉ dòng xung K1 lớn). Vẫn chỉ chạy sau ARM_SENSE."""
    b = Bench()
    b.sim.weld.add("KT")
    b.start()
    assert b.lt.fault is None and not b.violations
    b.sim.press("estop", True)
    assert not b.lt.poll() and not b.sim.k1


# ---- dòng 14: chập hai kênh E-stop ----
def test_row14_estop_channels_shorted_not_detectable():
    """Giới hạn đã ghi: chập chéo W14 làm E-stop không cắt, phần mềm KHÔNG thấy (bang-di-day-evt.md mục 5)."""
    b = Bench()
    b.start()
    b.sim.estop_bypass = True
    b.sim.press("estop", True)
    assert b.lt.poll() and b.sim.k1


# ---- race cho phép / E-stop ----
def test_race_estop_during_confirm_window():
    b = Bench()
    with pytest.raises(ResetRefused, match="E-stop"):
        b.reset(on_wait=lambda: (b.operator_click(hold_s=0.1, after_s=0.1)(),
                                    b.at(0.8, lambda: b.sim.press("estop", True))))   # K1 vừa đóng, chưa đủ xác nhận
    assert b.sim.k5 is False and not b.lt.ok and not b.sim.k1 and not b.violations


def test_race_short_estop_tap_between_polls():
    b = Bench()
    with pytest.raises(ResetRefused):
        b.reset(on_wait=lambda: (b.operator_click(hold_s=0.1, after_s=0.1)(),
                                    b.at(0.705, lambda: b.sim.press("estop", True)),
                                    b.at(0.706, lambda: b.sim.press("estop", False))))  # nhấn-nhả trong 1 ms
    assert b.sim.k5 is False and not b.sim.k1


def test_race_monitor_thread_and_reset_with_estop():
    """Luồng giám sát thật chạy song song reset(); E-stop đến từ luồng khác: không bao giờ 'run' khi K1 hở."""
    outcomes = set()
    for k in range(20):
        b = Bench()
        b.lt.start_monitor(0.0005)
        go = threading.Event()

        def presser():
            go.wait(1.0)
            b.sim.press("estop", True)
        th = threading.Thread(target=presser)
        th.start()
        try:
            b.reset(on_wait=lambda: (b.operator_click(hold_s=0.1, after_s=0.1)(),
                                        b.at(0.7 + 0.07 * k, go.set)))
        except ResetRefused:
            pass
        ran = b.lt.ok
        go.set()
        th.join()
        outcomes.add(ran)
        for _ in range(50):
            b.lt.poll()
        assert not b.lt.ok and b.sim.k5 is False and not b.sim.k1 and not b.violations
        b.lt.stop_monitor()
    assert outcomes                           # ca trước/sau xác nhận tuỳ lịch luồng; hai phía có test tất định ở trên


def test_backend_with_bang_di_day_latch():
    from faibotic.backends.feetech import EStopActive, FeetechBackend
    from faibotic.feetech import protocol as P
    port = FakeServoPort(list(spec.SERVO_IDS.values()))
    b = Bench()
    be = FeetechBackend(port=port, sleep=lambda s: None, latch=b.lt, dock=DockSwitches(_DockGPIO(DOCK_L=True, DOCK_R=True)))
    with pytest.raises(EStopActive):
        be.enable()
    b.start()
    be.enable()
    assert all(port.mem[i][P.TORQUE_ENABLE[0]] == 1 for i in port.mem)
    b.sim.press("estop", True)
    be.step(0.02)
    assert not be.enabled and all(port.mem[i][P.TORQUE_ENABLE[0]] == 0 for i in port.mem)
    with pytest.raises(EStopActive):
        be.enable()
    be.close()


# ---- M1 vòng 2: không tin mức ARM_SENSE do tụ giữ ----
SWEEP = [0.05, 0.1, 0.2, 0.3, 0.35, 0.5, 0.6, 0.8, 1.0, 1.5, 2.0]


@pytest.mark.parametrize("cap", SWEEP)
@pytest.mark.parametrize("ca", ["reset_ket", "k1_khong_dong"])
def test_cap_hold_sweep_never_runs_with_k1_open(cap, ca):
    """Tụ giữ 0,05–2 s, đã đo đúng: Reset kẹt hoặc K1 không đóng thì reset() không bao giờ trả 'run'.
    Tụ giữ > arm_confirm_s / 2 thì từ chối ngay từ cấu hình (trước khi bật K5)."""
    b = Bench(cap_hold_s=cap)
    if ca == "k1_khong_dong":
        b.sim.k1_coil_open = True
        wait = b.operator_click()
    else:
        wait = lambda: b.at(0.1, lambda: b.sim.press("reset", True))      # noqa: E731
    with pytest.raises(ResetRefused) as e:
        b.reset(on_wait=wait, timeout_s=4.0)
    assert not b.lt.ok and not b.violations and b.sim.k5 is False
    if 2 * cap > spec.EVT_ARM_CONFIRM_S:
        assert "xả" in str(e.value) and b.sim.history == [("K5_EN", False)]


@pytest.mark.parametrize("cap", SWEEP)
def test_cap_hold_sweep_unmeasured_still_safe_below_confirm(cap):
    """Số đo sai (khai 0,05 s) mà tụ thật giữ cap: vẫn an toàn khi cap < arm_confirm_s (1,0 s mặc định)."""
    b = Bench(cap_hold_s=cap, latch_kw=dict(servo_discharge_s=0.05))
    b.sim.k1_coil_open = True
    try:
        b.reset(on_wait=b.operator_click(), timeout_s=4.0)
    except ResetRefused:
        pass
    if cap < spec.EVT_ARM_CONFIRM_S:
        assert not b.lt.ok and not b.violations


@pytest.mark.parametrize("cap", [0.05, 0.2, 0.5])
def test_cap_hold_sweep_normal_reset_runs(cap):
    b = Bench(cap_hold_s=cap)
    b.start()
    b.sim.press("estop", True)
    b.run_for(2 * cap + 0.3)
    assert b.lt.state == "estop" and b.lt.fault is None          # tụ xả chậm không báo giả "K1 dính"


def test_k1_open_after_release_with_long_reset_hold():
    b = Bench(cap_hold_s=0.4)
    b.sim.k1_coil_open = True
    with pytest.raises(ResetRefused):
        b.reset(on_wait=b.operator_click(hold_s=1.5, after_s=0.1), timeout_s=5.0)
    assert not b.violations


def test_k1_aux_confirms_without_discharge_measurement():
    b = Bench(cap_hold_s=2.0, k1_aux=True, latch_kw=dict(servo_discharge_s=None, k1_aux_pin="K1_AUX"))
    b.sim.k1_coil_open = True
    with pytest.raises(ResetRefused):
        b.reset(on_wait=b.operator_click(), timeout_s=4.0)      # tụ giữ 2 s nhưng K1_AUX mở: không chạy
    assert not b.violations
    b.sim.k1_coil_open = False
    b.run_for(2.5)
    b.start()
    b.sim.weld.add("K1")
    b.sim.press("estop", True)
    b.run_for(0.3)
    assert b.lt.state == "fault" and "K1_AUX" in b.lt.fault


def test_nested_reset_refused():
    b = Bench()
    seen = []

    def inner():
        try:
            b.lt.reset()
        except ResetRefused as e:
            seen.append(str(e))
        b.operator_click()()
    b.reset(on_wait=inner)
    assert seen and "luồng khác" in seen[0] and b.lt.ok


# ---- M2 vòng 2: clear_fault ----
def test_clear_fault_refused_while_arming_keeps_state():
    b = Bench()
    errs = []

    def wait():
        try:
            b.lt.clear_fault()
        except ResetRefused as e:
            errs.append(str(e))
        assert b.lt.state == "arming" and b.sim.k5
        b.operator_click()()
    b.reset(on_wait=wait)
    assert errs and "không có lỗi" in errs[0] and b.lt.ok


def test_clear_fault_refused_when_not_faulted():
    b = Bench()
    with pytest.raises(ResetRefused, match="không có lỗi"):
        b.lt.clear_fault()
    b.start()
    with pytest.raises(ResetRefused, match="không có lỗi"):
        b.lt.clear_fault()
    assert b.lt.ok


def test_clear_fault_needs_clean_signal_for_clear_clean_s():
    b = Bench()
    b.start()
    b.sim.weld.add("K1")
    b.sim.press("estop", True)
    b.run_for(0.5)
    assert b.lt.state == "fault"
    b.sim.weld.clear()
    b.run_for(0.1)
    b.at(0.5, lambda: b.sim.weld.add("K1"))        # sạch 0,6 s rồi dính lại: không được xoá
    with pytest.raises(ResetRefused, match="ARM_SENSE"):
        b.lt.clear_fault()
    assert b.lt.state == "fault" and b.lt.fault and b.sim.k5 is False
    b.sim.weld.clear()
    b.run_for(0.1)
    t0 = b.t
    b.lt.clear_fault()
    assert b.t - t0 >= 1.0 - 1e-9 and b.lt.state == "stopped" and b.sim.k5 is False


def test_clear_fault_lowers_k5_first():
    b = Bench()
    b.sim.weld.add("K5")
    b.sim.click_reset()
    b.run_for(1.0)
    assert b.lt.state == "fault"
    b.sim.k5 = True                                 # giả chân ra bị kéo cao
    b.sim.weld.clear()
    b.sim.press("estop", True)
    b.sim.press("estop", False)
    b.run_for(0.1)
    b.lt.clear_fault()
    assert b.sim.history[-1] == ("K5_EN", False) and b.lt.state == "stopped"
