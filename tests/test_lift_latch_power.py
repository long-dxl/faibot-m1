"""v0.5 (G2 nhóm N, B3 + SDK trục nâng): chốt hãm thanh răng theo trình tự R2 7.2, E-stop cắt cuộn chốt, kẹt/quá mô-men,
chân GPIO chốt, TORQUE_LIMIT mọi servo + ngân sách dòng (R2 7.4), gói hỏng ở servo ghép đôi (CH-8)."""
import math
import sys

import pytest

from faibotic.safety import DockSwitches, FakeGPIO as _DockGPIO  # S2: khoá trạm đóng (cấu hình di_dong bắt buộc DOCK)
from faibotic import gpio_pi, limits, spec
from faibotic.backends.feetech import CurrentBudgetExceeded, FeetechBackend, LiftFault, PairFault
from faibotic.feetech import protocol as P
from faibotic.feetech.mockbus import FakeServoPort
from faibotic.gpio_pi import GPIOConfigError
from faibotic.safety import FakeGPIO, LiftLatch, LiftLatchError, SafetyLatch, SimLiftLatchHW

NOSLEEP = dict(sleep=lambda s: None, allow_no_latch=True)
LID = spec.SERVO_IDS["lift"]
# P-A (ghép đôi, lịch sử) và P-B: pan STS3215 như trước P-L; P-L mặc định xem test_current_budget_pl_config
_H16 = {k: v for k, v in spec.SERVO_IDS.items() if not k.endswith("wrist_yaw")}   # cấu hình lịch sử 5+1 (trước)
PA_IDS = {**spec.SERVO_IDS, **spec.PA_PAIR_IDS}
PAIRED = list(_H16) + list(spec.PA_PAIR_IDS)
UNPAIRED = list(_H16)
PAN_3215 = {f"{s}_pan": spec.STS3215 for s in spec.SIDES}


def torque(port, sid):
    return port.mem[sid][P.TORQUE_ENABLE[0]]


@pytest.fixture
def port():
    return FakeServoPort(list(PA_IDS.values()))


class Rec:
    """Gộp nhật ký: lệnh bus tới servo lift (mô-men) và chân cuộn chốt, để kiểm THỨ TỰ."""

    def __init__(self, be, hw):
        self.ev = []
        bw = be.bus._bus.write

        def write(sid, reg, val):
            if sid == LID and reg == P.TORQUE_ENABLE:
                self.ev.append(("lift_torque", bool(val)))
            return bw(sid, reg, val)
        be.bus._bus.write = write
        sw = be.bus._bus.sync_write

        def sync_write(reg, vals):
            if reg == P.TORQUE_ENABLE and LID in vals:
                self.ev.append(("lift_torque", bool(vals[LID])))
            return sw(reg, vals)
        be.bus._bus.sync_write = sync_write
        hw_write = hw.write

        def gw(pin, level):
            self.ev.append(("coil", bool(level)))
            return hw_write(pin, level)
        hw.write = gw


def make(port, hw=None, **kw):
    hw = hw or SimLiftLatchHW()
    lt = LiftLatch(hw, sleep=lambda s: None)
    kw.setdefault("allow_lift_underpowered", True)   # bus giả không tải (luật từ chối có test riêng)
    be = FeetechBackend(port=port, enable=True, lift_latch=lt, **NOSLEEP, **kw)
    return be, hw, lt


# ---------------- 1. trình tự chốt (đơn vị) ----------------
def test_release_order_hold_then_coil_then_sensor():
    hw = SimLiftLatchHW(delay_reads=3)
    lt = LiftLatch(hw, sleep=lambda s: None)
    ev = []
    assert lt.state == "engaged" and not lt.motion_allowed() and hw.en is False
    lt.release(lambda: ev.append(("hold", hw.en)))
    assert ev == [("hold", False)]                  # servo giữ tải TRƯỚC khi cuộn có điện
    assert lt.state == "retracted" and lt.motion_allowed() and hw.en is True


def test_engage_order_coil_off_then_sensor_then_torque_off():
    hw = SimLiftLatchHW()
    lt = LiftLatch(hw, sleep=lambda s: None)
    lt.release(lambda: None)
    ev = []
    lt.engage(lambda: ev.append("hold"), lambda: ev.append(("torque_off", hw.en, lt.retracted())))
    assert ev == ["hold", ("torque_off", False, False)]    # tắt mô-men chỉ sau khi cuộn tắt VÀ cảm biến báo cài
    assert lt.state == "engaged" and not lt.motion_allowed()


def test_engage_sensor_stuck_retracted_keeps_torque_and_faults():
    hw = SimLiftLatchHW()
    lt = LiftLatch(hw, sleep=lambda s: None)
    lt.release(lambda: None)
    hw.stuck = "retracted"
    off = []
    with pytest.raises(LiftLatchError, match="không cài"):
        lt.engage(lambda: None, lambda: off.append(1))
    assert off == [] and lt.fault and hw.en is False


def test_release_refused_when_sensor_says_retracted_with_coil_off():
    hw = SimLiftLatchHW()
    hw.stuck = "retracted"
    for _ in range(5):
        hw.read(hw.sensor_pin)                 # cảm biến đã ổn định ở "rút" dù cuộn tắt
    lt = LiftLatch(hw, sleep=lambda s: None)
    held = []
    with pytest.raises(LiftLatchError, match="đảo mức|kẹt rút"):
        lt.release(lambda: held.append(1))
    assert held == [] and hw.en is False


@pytest.mark.parametrize("fault", ["stuck", "dead", "no_servo_power"])
def test_release_timeout_coil_dropped(fault):
    hw = SimLiftLatchHW(servo_power=(lambda: fault != "no_servo_power"))
    if fault == "stuck":
        hw.stuck = "engaged"
    if fault == "dead":
        hw.sensor_dead = True
    lt = LiftLatch(hw, sleep=lambda s: None)
    with pytest.raises(LiftLatchError, match="không rút"):
        lt.release(lambda: None)
    assert hw.en is False and lt.state == "fault" and not lt.motion_allowed()


def test_coil_fed_from_servo_plus_failsafe():
    """Cuộn lấy 12 V từ SERVO+ sau K1: K1 nhả (E-stop) thì cuộn mất điện dù Pi còn giữ chân EN cao."""
    power = {"on": True}
    hw = SimLiftLatchHW(servo_power=lambda: power["on"])
    lt = LiftLatch(hw, sleep=lambda s: None)
    lt.release(lambda: None)
    power["on"] = False                     # E-stop phần cứng: SERVO+ = 0
    assert hw.en is True and hw.coil is False
    for _ in range(5):
        lt.retracted()
    assert not lt.motion_allowed() and lt.fault       # chốt rơi khi đang cho chạy: lỗi, dừng


def test_lift_gate_blocks_without_retracted_latch():
    hw = SimLiftLatchHW()
    lt = LiftLatch(hw, sleep=lambda s: None)
    assert limits.lift_gate(0.2, 0.3, None, lt) == 0.2
    assert limits.lift_gate(0.2, 0.2, None, lt) == 0.2
    lt.release(lambda: None)
    assert limits.lift_gate(0.2, 0.3, None, lt) == 0.3
    assert limits.lift_gate(0.2, 0.3, limits.FakeLiftSwitches(top=True), lt) == 0.2   # công tắc vẫn áp


# ---------------- 2. backend ----------------
def test_enable_leaves_lift_on_latch_then_unlock_in_order(port):
    be, hw, lt = make(port)
    assert torque(port, LID) == 0 and torque(port, spec.SERVO_IDS["left_elbow"]) == 1
    rec = Rec(be, hw)
    cur = be._cmd["lift"]
    sent = be.write_joints({"lift": cur + 0.01})
    assert sent["lift"] == pytest.approx(cur + 0.01)
    on = rec.ev.index(("lift_torque", True))
    coil = rec.ev.index(("coil", True))
    assert on < coil and lt.state == "retracted" and torque(port, LID) == 1
    be.close()


def test_disable_engages_latch_before_torque_off(port):
    be, hw, lt = make(port)
    be.write_joints({"lift": be._cmd["lift"] + 0.01})
    rec = Rec(be, hw)
    be.disable()
    assert rec.ev.index(("coil", False)) < rec.ev.index(("lift_torque", False))
    assert lt.state == "engaged" and torque(port, LID) == 0


def test_disable_with_latch_not_engaging_keeps_lift_holding(port):
    be, hw, lt = make(port)
    be.write_joints({"lift": be._cmd["lift"] + 0.01})
    hw.stuck = "retracted"
    with pytest.raises(LiftFault, match="không cài"):
        be.disable()
    assert torque(port, LID) == 1 and torque(port, spec.SERVO_IDS["left_pan"]) == 0
    with pytest.raises(LiftFault):
        be.write_joints({"left_pan": 0.1})
    be.emergency_stop()


def test_estop_cuts_latch_coil_with_servos(port):
    """SafetyLatch trip -> on_stop -> emergency_stop: tắt mô-men mọi servo và hạ chân cuộn chốt."""
    hw = SimLiftLatchHW()
    lt = LiftLatch(hw, sleep=lambda s: None)
    g = FakeGPIO()
    sl = SafetyLatch(g, sleep=lambda s: None, settle_s=0, hw="pcb")
    sl.reset()
    be = FeetechBackend(port=port, enable=True, sleep=lambda s: None, latch=sl, lift_latch=lt, allow_lift_underpowered=True,
                        dock=DockSwitches(_DockGPIO(DOCK_L=True, DOCK_R=True)))
    be.write_joints({"lift": be._cmd["lift"] + 0.01})
    assert hw.en is True
    g.levels["ESTOP_SENSE"] = False
    sl.poll()
    assert hw.en is False and lt.state == "engaged"
    assert all(torque(port, i) == 0 for i in spec.SERVO_IDS.values())
    be.close()


def test_jam_into_extended_latch_stops_and_errors(port):
    be, hw, lt = make(port)
    be.write_joints({"lift": be._cmd["lift"] + 0.01})
    hw.stuck = "engaged"                       # chốt bật ra (lò xo/cuộn hỏng) khi đang chạy
    for _ in range(5):
        hw.read(hw.sensor_pin)
    with pytest.raises(LiftFault, match="chốt rơi"):
        be.step(0.02)
    with pytest.raises(LiftFault):
        be.write_joints({"lift": 0.2})
    # cảm biến báo "cài" (chốt đã bật ra): theo trình tự dừng, cuộn tắt rồi mới tắt mô-men lift; tải nằm trên chốt
    assert hw.en is False and lt.state == "engaged" and torque(port, LID) == 0


def test_overtorque_while_latch_not_retracted_is_jam(port):
    be, hw, lt = make(port)
    be._lift_hold()                            # mô-men lift bật mà chốt còn cài (vd lỗi trình tự): servo đẩy vào chốt
    port.set_load(LID, spec.LIFT_TORQUE_LIMIT_PERMILLE)
    with pytest.raises(LiftFault, match="đẩy vào chốt"):
        be.step(0.02)


def test_overtorque_while_running_trips_after_cycles(port):
    be, hw, lt = make(port)
    be.write_joints({"lift": be._cmd["lift"] + 0.01})
    port.set_load(LID, -spec.LIFT_TORQUE_LIMIT_PERMILLE)
    for _ in range(spec.LIFT_JAM_CYCLES - 1):
        be.step(0.02)
    with pytest.raises(LiftFault, match="quá mô-men"):
        be.step(0.02)
    port.set_load(LID, 0)
    be.clear_fault()
    assert be.lift_fault is None and lt.fault is None


def test_idle_engages_latch_and_turns_lift_torque_off(port):
    be, hw, lt = make(port)
    be.write_joints({"lift": be._cmd["lift"] + 0.01})
    n = math.ceil(spec.LIFT_LATCH_IDLE_S / 0.5)
    for _ in range(n):
        be.step(0.5)
    assert lt.state == "engaged" and torque(port, LID) == 0 and hw.en is False
    be.write_joints({"lift": be._cmd["lift"] + 0.01})     # lệnh mới: rút lại theo trình tự
    assert lt.state == "retracted" and torque(port, LID) == 1
    be.close()


def test_unlock_failure_blocks_lift(port):
    hw = SimLiftLatchHW()
    hw.stuck = "engaged"
    be, hw, lt = make(port, hw)
    cur = be._cmd["lift"]
    with pytest.raises(LiftFault, match="không rút"):
        be.write_joints({"lift": cur + 0.01})
    assert be.read_joints()["lift"] == pytest.approx(cur, abs=1e-4) and hw.en is False


# ---------------- 3. GPIO chân chốt ----------------
@pytest.fixture
def lg(monkeypatch):
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
    from test_gpio_pi import FakeLgpio
    m = FakeLgpio()
    monkeypatch.setitem(sys.modules, "lgpio", m)
    return m


BASE = {"K5_EN": 22, "ESTOP_OK": 17, "ARM_SENSE": 27, "RESET_SENSE": 23}


def test_latch_pins_default_output_pull_and_active_low(lg):
    pins = dict(BASE, **gpio_pi.LATCH_DEFAULT_BCM)
    g = gpio_pi.PiGPIO(pins, backend="lgpio")
    assert gpio_pi.LATCH_DEFAULT_BCM == {"LIFT_LATCH_EN": 24, "LIFT_LATCH_RETRACTED": 25}
    assert "LIFT_LATCH_EN" in g.outputs and lg.level[24] == 0          # khởi tạo thấp: cuộn mất điện, chốt cài
    assert g.pull["LIFT_LATCH_RETRACTED"] == "up" and "LIFT_LATCH_RETRACTED" in g.active_low
    lg.level[25] = 1                                                    # đứt dây / công tắc hở: kéo lên
    assert g.read("LIFT_LATCH_RETRACTED") is False                      # -> "không rút": không cho chạy
    lg.level[25] = 0
    assert g.read("LIFT_LATCH_RETRACTED") is True
    lt = gpio_pi.lift_latch_for(g, sleep=lambda s: None)
    assert isinstance(lt, LiftLatch)
    g.close()
    assert lg.level[24] == 0


@pytest.mark.parametrize("pin,bcm", [("LIFT_LATCH_EN", 8), ("LIFT_LATCH_RETRACTED", 4), ("LIFT_LATCH_EN", 22)])
def test_latch_pins_forbidden(lg, pin, bcm):
    pins = dict(BASE, **gpio_pi.LATCH_DEFAULT_BCM)
    pins[pin] = bcm
    with pytest.raises(GPIOConfigError, match="BCM 0–8|trùng"):
        gpio_pi.PiGPIO(pins, backend="lgpio")


def test_no_latch_pins_no_latch(lg):
    g = gpio_pi.PiGPIO(BASE, backend="lgpio")
    assert gpio_pi.lift_latch_for(g) is None
    g.close()


# ---------------- 4. TORQUE_LIMIT mọi servo + ngân sách dòng ----------------
def test_torque_limit_written_for_every_servo(port):
    be = FeetechBackend(port=port, enable=True, **NOSLEEP)       # P-L: 16 servo, vai đơn
    got = be.read_soft_limits()["torque_limit_permille"]
    assert set(got) == set(spec.SERVO_IDS)
    assert got["left_shoulder"] == got["right_shoulder"] == 600       # min(60%, 60 kg·cm / 95 = 632‰)
    assert got["left_elbow"] == 600 and got["left_pan"] == 500 and got["left_wrist_flex"] == 600
    assert got["wheel_1"] == 500 and got["lift"] == spec.LIFT_TORQUE_LIMIT_PERMILLE
    be.close()


def test_torque_limit_written_for_every_servo_pa(port):
    be = FeetechBackend(port=port, enable=True, paired_joints=spec.PA_PAIRED_JOINTS, **NOSLEEP)   # P-A lịch sử: 20 servo
    got = be.read_soft_limits()["torque_limit_permille"]
    assert set(got) == set(PA_IDS)
    assert got["left_shoulder"] == got["right_shoulder_b"] == 316        # 60 kg·cm cả cặp vẫn chặt hơn 60%
    assert got["left_elbow_b"] == 600
    be.close()


def test_current_budget_both_configs():
    """Ghép đôi 5+1 (9 STS3095 + 11 STS3215) và P-B (5 + 11). Dòng chặn 9,8 / 2,7 A [trang bán]; phụ tải Pi 2,6 A,
    mạch điều khiển 1,2 A [giả định], cuộn chốt lift 1,0 A [giả định] = 4,8 A."""
    pa = limits.current_budget(PAIRED, models=PAN_3215)
    pb = limits.current_budget(UNPAIRED, models=PAN_3215)
    assert pa["pi_a"] == pytest.approx(2.6 + 1.2 + 1.0) == pb["pi_a"]
    assert pa["stall_a"] == pytest.approx(9 * 9.8 + 11 * 2.7 + 4.8) == pytest.approx(122.7)
    assert pb["stall_a"] == pytest.approx(5 * 9.8 + 11 * 2.7 + 4.8) == pytest.approx(83.5)
    # tay mỗi bên: vai 2 x 9,8 x 0,316 + khuỷu 2 x 9,8 x 0,6 + pan/cuộn/kẹp 3 x 2,7 x 0,5 + gập 2,7 x 0,6
    arm_a = 2 * 9.8 * 0.316 + 2 * 9.8 * 0.6 + 3 * 2.7 * 0.5 + 2.7 * 0.6
    arm_b = 9.8 * 0.6 + 9.8 * 0.6 + 3 * 2.7 * 0.5 + 2.7 * 0.6          # vai đơn: min(60%, 60 kg·cm / 95) = 600‰
    rest = 3 * 2.7 * 0.5 + 9.8 * spec.LIFT_TORQUE_LIMIT_PERMILLE / 1000 + 4.8
    assert pa["allowed_a"] == pytest.approx(2 * arm_a + rest)
    assert pb["allowed_a"] == pytest.approx(2 * arm_b + rest)
    assert pa["ok"] and pb["ok"] and pa["allowed_a"] <= 80 and pb["allowed_a"] <= 80
    # (bánh lift 20 răng, r 15 mm): TORQUE_LIMIT lift 138 -> 97‰ theo răng giữ chỗ 120 N, dòng lift 1,35 -> 0,95 A (trước: 57,45 / 45,06)
    assert pa["allowed_a"] == pytest.approx(57.05, abs=0.01) and pb["allowed_a"] == pytest.approx(44.66, abs=0.01)


def test_enable_refuses_over_budget(port, monkeypatch):
    monkeypatch.setattr(spec, "TORQUE_LIMIT_PCT", {k: 100 for k in spec.TORQUE_LIMIT_PCT})
    monkeypatch.setattr(spec, "SHOULDER_TORQUE_LIMIT_KGCM", 190.0)
    be = FeetechBackend(port=port, **NOSLEEP)
    with pytest.raises(CurrentBudgetExceeded, match="80 A"):
        be.enable()
    assert all(torque(port, i) == 0 for i in spec.SERVO_IDS.values())


# ---------------- 5. / CH-8: gói hỏng ở servo ghép đôi ----------------
def test_corrupt_packet_in_pair_read_trips_pairs(port):
    be = FeetechBackend(port=port, enable=True, paired_joints=spec.PA_PAIRED_JOINTS, **NOSLEEP)
    port.corrupt_next_reply = True             # trả lời đầu tiên của lần đọc cặp sai checksum
    with pytest.raises(PairFault, match="gói hỏng"):
        be.check_pairs()
    for j in ("left_shoulder", "left_shoulder_b", "right_elbow", "right_elbow_b"):
        assert torque(port, PA_IDS[j]) == 0
    assert torque(port, PA_IDS["left_pan"]) == 1        # servo không ghép đôi giữ nguyên
    assert be.fault and "gói hỏng" in be.fault
    with pytest.raises(PairFault):
        be.write_joints({"left_elbow": 0.2})


# ---------------- QC2 vòng 1 ----------------
def test_overtorque_while_retracted_engages_latch(port):
    """MAJOR: quá mô-men khi chốt đã rút -> LiftFault VÀ cài chốt (cảm biến xác nhận) rồi tắt mô-men lift, không để xe trượt."""
    be, hw, lt = make(port)
    be.write_joints({"lift": be._cmd["lift"] + 0.01})
    port.set_load(LID, spec.LIFT_TORQUE_LIMIT_PERMILLE)
    rec = Rec(be, hw)
    with pytest.raises(LiftFault, match="quá mô-men"):
        for _ in range(spec.LIFT_JAM_CYCLES):
            be.step(0.02)
    assert lt.state == "engaged" and hw.en is False and not lt.retracted() and torque(port, LID) == 0
    assert rec.ev.index(("coil", False)) < rec.ev.index(("lift_torque", False))


def test_overtorque_while_retracted_latch_cannot_engage_keeps_holding(port):
    be, hw, lt = make(port)
    be.write_joints({"lift": be._cmd["lift"] + 0.01})
    hw.stuck = "retracted"
    port.set_load(LID, spec.LIFT_TORQUE_LIMIT_PERMILLE)
    with pytest.raises(LiftFault, match="KHÔNG cài"):
        for _ in range(spec.LIFT_JAM_CYCLES):
            be.step(0.02)
    assert torque(port, LID) == 1 and "KHÔNG cài" in be.lift_fault
    be.emergency_stop()


def test_estop_during_release_leaves_coil_off_and_torque_off(port):
    """MINOR: E-stop đến đúng lúc release() đang giữ khoá (sau hold()): không bật cuộn, mô-men lift tắt, EStopActive."""
    from faibotic.backends.feetech import EStopActive
    hw = SimLiftLatchHW()
    lt = LiftLatch(hw, sleep=lambda s: None)
    g = FakeGPIO()
    sl = SafetyLatch(g, sleep=lambda s: None, settle_s=0, hw="pcb")
    sl.reset()
    be = FeetechBackend(port=port, enable=True, sleep=lambda s: None, latch=sl, lift_latch=lt, allow_lift_underpowered=True,
                        dock=DockSwitches(_DockGPIO(DOCK_L=True, DOCK_R=True)))
    hold = be._lift_hold

    def hold_then_estop():
        hold()
        g.levels["ESTOP_SENSE"] = False
        sl.poll()                                # luồng giám sát: emergency_stop() + cut() khi release() đang chạy
    be._lift_hold = hold_then_estop
    with pytest.raises(EStopActive, match="rút chốt"):
        be.write_joints({"lift": be._cmd["lift"] + 0.01})
    assert hw.en is False and ("LIFT_LATCH_EN", True) not in hw.history
    assert all(torque(port, i) == 0 for i in spec.SERVO_IDS.values()) and lt.state == "engaged"
    be.close()


def test_emergency_stop_cuts_coil_before_torque_off(port):
    be, hw, lt = make(port)
    be.write_joints({"lift": be._cmd["lift"] + 0.01})
    rec = Rec(be, hw)
    be.emergency_stop()
    assert rec.ev[0] == ("coil", False) and rec.ev.index(("coil", False)) < rec.ev.index(("lift_torque", False))


@pytest.mark.parametrize("pin", ["LIFT_LATCH_EN", "LIFT_LATCH_RETRACTED"])
def test_lift_latch_for_one_pin_missing_is_config_error(lg, pin):
    pins = dict(BASE, **{pin: gpio_pi.LATCH_DEFAULT_BCM[pin]})
    g = gpio_pi.PiGPIO(pins, backend="lgpio")
    with pytest.raises(GPIOConfigError, match="cần đủ"):
        gpio_pi.lift_latch_for(g)
    g.close()


def test_current_budget_pl_config():
    """P-L 6+1 (24/11): AM-ARM200 nguyên, 7 STS3095 (pan, vai, khuỷu x2 + lift) + 11 STS3215. Chặn: 7 x 9,8 + 11 x 2,7
    + Pi 2,6 = 100,9 A; cộng điều khiển 1,2 + cuộn chốt 1,0 [giả định] = 103,1 A (5+1: 97,7). Hàm nhận danh sách servo P-L qua models=."""
    m = spec.PL_SERVO_MODELS
    assert sum(1 for v in m.values() if v is spec.STS3095) == 7 and sum(1 for v in m.values() if v is spec.STS3215) == 11
    b = limits.current_budget(m, models=m)
    assert b["stall_a"] - spec.CTRL_CURRENT_A - spec.LIFT_LATCH_COIL_A == pytest.approx(7 * 9.8 + 11 * 2.7 + 2.6) == pytest.approx(100.9)
    want = (2 * 9.8 * 0.5 + 2 * 9.8 * 0.6 + 2 * 9.8 * 0.6 + 2 * 2.7 * 0.6 + 2 * 2.7 * 0.8 + 2 * 2.7 * 0.5 + 2 * 2.7 * 0.5 + 3 * 2.7 * 0.5
            + 9.8 * spec.LIFT_TORQUE_LIMIT_PERMILLE / 1000 + 4.8)
    assert b["allowed_a"] == pytest.approx(want) == pytest.approx(56.08, abs=0.01) and b["ok"]   # +2 yaw x 2,7 x 0,8 (5+1: 51,76): lift 97‰
    assert b["rows"]["pan"]["stall_a"] == pytest.approx(19.6)             # pan là STS3095 ở AM-ARM200
    assert b["rows"]["shoulder"]["permille"] == {600}                     # vai đơn: min(60%, 60 kg·cm / 95) = 600‰
