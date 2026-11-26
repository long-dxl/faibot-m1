"""v0.3/v0.4.1 cho EVT (S1C: cặp ghép đôi chạy ở cấu hình P-A tường minh, paired_joints=spec.PA_PAIRED_JOINTS; P-L mặc định 16 ID): bản đồ ID, cặp servo ghép đôi, giới hạn mềm (cả đoạn nội suy), chốt E-stop phần mềm, lò xo đứt."""
import math

import numpy as np
import pytest

from faibotic.safety import DockSwitches, FakeGPIO as _DockGPIO  # S2: khoá trạm đóng (cấu hình di_dong bắt buộc DOCK)
from faibotic import Robot, limits, spec
from faibotic.backends.feetech import FeetechBackend, PairFault, default_calibration, pairs_for
from faibotic.feetech import protocol as P
from faibotic.feetech.mockbus import FakeServoPort
from faibotic.safety import FakeGPIO, ResetRefused, SafetyLatch, SpringMonitor, shoulder_model_torque

D = math.radians
PA_IDS = {**spec.SERVO_IDS, **spec.PA_PAIR_IDS}   # P-A (ghép đôi, lịch sử): 20 ID
ALL_IDS = list(PA_IDS.values())
# bench test; test chốt E-stop dùng LATCHED. Backend trong file này chạy cấu hình ghép đôi P-A để giữ test đường mã cặp.
NOSLEEP = dict(sleep=lambda s: None, allow_no_latch=True, paired_joints=spec.PA_PAIRED_JOINTS)


def raw_goal(port, sid):
    return P.from_u16(port.mem[sid][P.GOAL_POSITION[0]:P.GOAL_POSITION[0] + 2])


def torque(port, sid):
    return port.mem[sid][P.TORQUE_ENABLE[0]]


@pytest.fixture
def port():
    return FakeServoPort(ALL_IDS)


@pytest.fixture
def be(port):
    b = FeetechBackend(port=port, enable=True, **NOSLEEP)
    yield b


# ---------------- 1. bản đồ ID ----------------
def test_servo_id_map_20_unique():
    ids = spec.SERVO_IDS     # P-L 6+1: 18 servo, mỗi khớp một servo; yaw 21, 22 (tránh 17-20 của P-A)
    assert len(ids) == 18 and sorted(ids.values()) == list(range(1, 17)) + [21, 22]
    assert not set(ids.values()) & set(spec.PA_PAIR_IDS.values())
    assert spec.PAIRED_JOINTS == () and all(n == 1 for n in spec.ARM_SERVO_COUNT.values())
    assert pairs_for() == {}
    # P-A (ghép đôi): ID 17-20 cho servo B, giữ cho đường mã có tham số
    assert spec.PA_PAIR_IDS == {"left_shoulder_b": 17, "right_shoulder_b": 18, "left_elbow_b": 19, "right_elbow_b": 20}
    assert len(PA_IDS) == 22 and sorted(PA_IDS.values()) == list(range(1, 21)) + [21, 22]   # + yaw 21, 22
    assert sorted(pairs_for(spec.PA_PAIRED_JOINTS)) == sorted(spec.joint_name(s, j) for s in spec.SIDES for j in spec.PA_PAIRED_JOINTS)


def test_pl_backend_has_no_pairs(port):
    be = FeetechBackend(port=port, sleep=lambda s: None, allow_no_latch=True)
    assert be.pairs == {} and set(be.ids) == set(spec.SERVO_IDS)
    be.check_pairs()


# ---------------- 2. cặp servo ----------------
def test_pair_one_command_two_servos_mirrored(port, be):
    be.write_joints({"left_shoulder": -0.5, "right_elbow": 1.0})
    d = round(0.5 * 4096 / (2 * math.pi))
    assert raw_goal(port, 2) == 2048 - d and raw_goal(port, 17) == 2048 + d   # ngược chiều
    e = round(1.0 * 4096 / (2 * math.pi))
    assert raw_goal(port, 9) == 2048 + e and raw_goal(port, 20) == 2048 - e
    j = be.read_joints()
    assert j["left_shoulder"] == pytest.approx(-0.5, abs=2e-3) and j["right_elbow"] == pytest.approx(1.0, abs=2e-3)
    be.check_pairs()
    assert be.pair_metrics["left_shoulder"]["pos_diff_deg"] < 0.2


def test_pair_separate_offsets_and_no_mirror(port):
    cal = default_calibration(mirror=False, paired_joints=spec.PA_PAIRED_JOINTS)
    cal["left_shoulder_b"]["zero"] = 2100
    be = FeetechBackend(port=port, calibration=cal, enable=True, **NOSLEEP)
    be.write_joints({"left_shoulder": 0.3})
    d = round(0.3 * 4096 / (2 * math.pi))
    assert raw_goal(port, 2) == 2048 + d and raw_goal(port, 17) == 2100 + d
    assert be.read_joints()["left_shoulder"] == pytest.approx(0.3, abs=2e-3)
    be.check_pairs()


def test_pair_position_mismatch_trips_pair_only(port, be):
    port.stuck.add(17)                      # vai-B trái kẹt
    be.write_joints({"left_shoulder": D(20)})
    with pytest.raises(PairFault, match="left_shoulder"):
        for _ in range(spec.PAIR_FAULT_CYCLES):
            be.step(0.02)
    assert torque(port, 2) == 0 and torque(port, 17) == 0
    assert torque(port, 3) == 1 and torque(port, 8) == 1      # cặp khác vẫn giữ
    with pytest.raises(PairFault):
        be.write_joints({"left_pan": 0.1})
    port.stuck.clear()
    be.clear_fault()
    be.enable()
    assert torque(port, 2) == 1


def test_pair_transient_below_cycles_no_trip(port, be):
    port.set_load(2, 400)
    port.set_load(17, -100)                 # khung khớp: +100 -> chênh 120%
    for _ in range(spec.PAIR_FAULT_CYCLES - 1):
        be.step(0.02)
    port.set_load(17, -400)
    be.step(0.02)
    assert be.fault is None


def test_pair_load_mismatch_trips(port, be):
    port.set_load(8, 300)
    port.set_load(18, -280)                 # cân: chênh 7%
    be.step(0.02)
    assert be.pair_metrics["right_shoulder"]["load_diff"] < spec.PAIR_MAX_LOAD_DIFF
    port.set_load(18, -200)                 # chênh 40% > 20% (Q-FQC-C4)
    with pytest.raises(PairFault, match="chênh tải"):
        for _ in range(spec.PAIR_FAULT_CYCLES):
            be.step(0.02)
    assert torque(port, 8) == 0 and torque(port, 18) == 0


def test_pair_low_load_uses_floor(port, be):
    port.set_load(3, 30)
    # 3% và 1,5% mô-men chặn: chênh tương đối 67% nhưng tính theo sàn 10% (100 ‰) chỉ 15% < 20%
    port.set_load(19, -15)
    for _ in range(5):
        be.step(0.02)
    assert be.fault is None


def test_pair_servo_not_responding_trips_immediately(port, be):
    port.dead.add(19)                       # khuỷu-B trái mất cáp
    with pytest.raises(PairFault, match="không phản hồi"):
        be.step(0.02)
    assert torque(port, 3) == 0 and be.fault and "left_elbow" in be.fault
    assert torque(port, 9) == 1


def test_pair_threshold_parameters(port):
    be = FeetechBackend(port=port, enable=True, pair_max_load_diff=0.5, pair_fault_cycles=1, **NOSLEEP)
    port.set_load(2, 300)
    port.set_load(17, -200)                 # 40% < 50%
    be.step(0.02)
    port.set_load(17, -100)                 # 100% > 50%, 1 chu kỳ là dừng
    with pytest.raises(PairFault):
        be.step(0.02)


# ---------------- 3. giới hạn mềm ----------------

def P3(s, e, w):
    return {"left_shoulder": D(s), "left_elbow": D(e), "left_wrist_flex": D(w)}


def test_wrist_couple_table_direct():
    """v0.4.3: giới hạn ghép cổ tay-khuỷu-vai (quét CAD J7 + J56). Tra nhỏ nhất bốn nút; vai thấp không bị ghép; vai/khuỷu cao thì cổ tay bị hạ."""
    S, E, T = spec.WRIST_COUPLE_SHOULDER_DEG, spec.WRIST_COUPLE_ELBOW_DEG, spec.WRIST_COUPLE_MAX_DEG
    assert len(T) == len(S) and all(len(r) == len(E) for r in T)
    whi = spec.ARM_LIMITS["wrist_flex"][1]
    for i, sd in enumerate(S):
        for j, ed in enumerate(E):
            ai = (i, i) if i in (0, len(S) - 1) else (i - 1, i)       # tại nút: hai đầu = chính nó; nút giữa = ô phía dưới (nhỏ nhất bốn nút, bảo thủ)
            bj = (j, j) if j in (0, len(E) - 1) else (j - 1, j)
            exp = min(T[a][b] for a in ai for b in bj)
            assert spec.wrist_max(D(sd), D(ed)) == pytest.approx(min(D(exp), whi))
            assert spec.wrist_max(D(sd), D(ed)) <= min(D(T[i][j]), whi) + 1e-12
    assert spec.wrist_max(D(-80), D(100)) == pytest.approx(D(70))               # vai -80, khuỷu <= 120: ô bảng 87, trần gập 70 (đáy J56 chạm cẳng tay J4 ở 73,4)
    assert spec.wrist_max(D(-80), D(127)) == pytest.approx(D(70))                # v0.4.5 (DK-W4): camera ở cuộn -160 chạm đệm chặn dưới ở 80 (84,5)
    assert spec.wrist_max(D(48.5), D(127)) < D(60)                              # vai tối đa, khuỷu tối đa: cổ tay thấp
    assert spec.HOME_ARM["wrist_flex"] <= spec.wrist_max(spec.HOME_ARM["shoulder"], spec.HOME_ARM["elbow"]) + 1e-9
    assert not hasattr(spec, "LOWSTOP_COUPLE_DEG")      # mã ghép vai-khuỷu ngủ đã bỏ; elbow_max (cột nâng, QC2 vòng 2) là giới hạn khác: khuỷu theo vai
    r = Robot("sim")
    r.keepout = False   # G1/E3: test này kiểm giới hạn khớp/bảng ghép, tư thế cực (cổ tay thấp sau trục pan) nằm trong vùng cấm bệ; vùng cấm kiểm ở test_tram
    em = math.degrees(spec.elbow_max(D(40)))      # 79 (cột nâng, quét mesh)
    r.set_joint_positions(P3(40, em, 40))
    out = r.set_joint_positions({"left_wrist_flex": D(100)})                    # chỉ lệnh cổ tay: kẹp
    assert out["left_wrist_flex"] == pytest.approx(spec.wrist_max(D(40), D(em))) and out["left_wrist_flex"] < D(88)
    out = r.set_joint_positions({"left_elbow": D(127)})                          # chỉ lệnh khuỷu: cổ tay hạ theo
    assert out["left_elbow"] == pytest.approx(D(em)) and r._targets["left_wrist_flex"] <= spec.wrist_max(D(40), D(em)) + 1e-9   # khuỷu 127 bị hạ về elbow_max
    r.close()


def test_intermediate_path_checked():
    q0, q1 = P3(9, 98, -21), P3(41, 70, 68)    # v0.5: khuỷu <= elbow_max(vai) (cột nâng); cặp này vẫn cắt vùng cấm ghép cổ tay (24/11: gập 85 -> 68 theo trần 70, vai 31 -> 41)
    assert not limits.violations(q0) and not limits.violations(q1)
    assert not limits.segment_ok(q0, q1)                              # giữa đường cổ tay còn cao ở vai/khuỷu cao
    legs = limits.plan(q0, q1)
    assert len(legs) == 3
    prev = q0
    for leg in legs:
        assert limits.segment_ok(prev, leg) and not limits.violations(leg)
        prev = {**prev, **leg}
    assert prev == pytest.approx(q1)


def test_plan_random_pairs_never_cross():
    rng = np.random.default_rng(0)
    (slo, shi), (elo, ehi), (wlo, _) = spec.ARM_LIMITS["shoulder"], spec.ARM_LIMITS["elbow"], spec.ARM_LIMITS["wrist_flex"]
    n_split = 0
    for _ in range(2000):
        def pt():
            s, e = rng.uniform(slo, shi), rng.uniform(elo, ehi)
            if rng.random() < 0.5:
                s, e = rng.uniform(0, shi), rng.uniform(D(70), ehi)
            e = min(e, spec.elbow_max(s))      # v0.5: khuỷu theo vai (cột nâng)
            wm = spec.wrist_max(s, e)
            lo = wlo if rng.random() < 0.5 else max(wlo, wm - D(25))     # bảng mới phẳng hơn (trần 87): nửa số điểm sát giới hạn để còn đủ cặp phải tách
            return {"left_shoulder": s, "left_elbow": e, "left_wrist_flex": rng.uniform(lo, wm)}
        q0, q1 = pt(), pt()
        legs = limits.plan(q0, q1)
        n_split += len(legs) > 1
        prev = q0
        for leg in legs:
            assert limits.segment_ok(prev, leg) and not limits.violations(leg)
            prev = {**prev, **leg}
        assert prev == pytest.approx(q1)
    assert n_split > 3


@pytest.mark.parametrize("backend", ["sim", "feetech"])
def test_backend_guard_blocks_crossing_jump(backend, port):
    kw = dict(port=port, enable=True, **NOSLEEP) if backend == "feetech" else {}
    r = Robot(backend, **kw)
    r.keepout = False   # G1/E3: test này kiểm giới hạn khớp/bảng ghép, tư thế cực (cổ tay thấp sau trục pan) nằm trong vùng cấm bệ; vùng cấm kiểm ở test_tram
    r.move_joints(P3(9, 98, -21), duration=0.2)
    sent = r.backend.write_joints(P3(41, 70, 68))   # gọi thẳng backend, bỏ qua Robot
    assert sent["left_shoulder"] == pytest.approx(D(9)) and sent["left_elbow"] == pytest.approx(D(98))
    sent = r.backend.write_joints({"left_elbow": D(150), "left_pan": D(200)})      # ngoài ARM_LIMITS
    assert sent["left_elbow"] <= D(spec.ELBOW_SOFT_MAX_DEG) + 1e-9 and sent["left_pan"] == pytest.approx(D(spec.PAN_SOFT_MAX_DEG))
    r.close()


def test_move_joints_every_written_target_valid():
    r = Robot("sim")
    r.keepout = False   # G1/E3: test này kiểm giới hạn khớp/bảng ghép, tư thế cực (cổ tay thấp sau trục pan) nằm trong vùng cấm bệ; vùng cấm kiểm ở test_tram
    r.move_joints(P3(0, 70, 100), duration=0.3)
    seen = []
    orig = r.backend.write_joints

    def spy(t):
        out = orig(t)
        seen.append(dict(r.backend._targets))
        return out
    r.backend.write_joints = spy
    r.move_joints(P3(48.5, 127, 40), duration=1.0)
    keys = ("left_shoulder", "left_elbow", "left_wrist_flex")
    for a, b in zip(seen, seen[1:]):
        assert not limits.violations({k: b[k] for k in keys})
        assert limits.segment_ok({k: a[k] for k in keys}, {k: b[k] for k in keys})
    assert r._targets["left_shoulder"] == pytest.approx(D(48.5)) and r._targets["left_wrist_flex"] == pytest.approx(D(40))
    r.close()


def test_soft_limits_load_and_read_back(port):
    be = FeetechBackend(port=port, **NOSLEEP)
    be.load_soft_limits()
    assert be.verify_soft_limits() == []
    got = be.read_soft_limits()
    assert got["servo_deg"]["left_shoulder_b"] == pytest.approx((-80, spec.SHOULDER_SOFT_MAX_DEG), abs=0.1)
    assert got["servo_deg"]["right_elbow"] == pytest.approx((-10, spec.ELBOW_SOFT_MAX_DEG), abs=0.1)
    assert got["couple"]["wrist_table_nodes"] == [len(spec.WRIST_COUPLE_SHOULDER_DEG), len(spec.WRIST_COUPLE_ELBOW_DEG)]
    be.enable()
    lim = be.read_soft_limits()["shoulder_torque_limit_permille"]
    assert set(lim.values()) == {316}   # 2 x 316 ‰ x 95 kg·cm = 60 kg·cm tổng cặp
    with pytest.raises(RuntimeError):
        be.load_soft_limits()            # đang bật mô-men: không ghi EEPROM
    port.mem[17][P.MAX_ANGLE[0]:P.MAX_ANGLE[0] + 2] = P.u16(4000)
    assert any("left_shoulder_b" in x for x in be.verify_soft_limits())


def test_sim_soft_limits_read_back():
    r = Robot("sim")
    got = r.soft_limits()
    assert got["model_deg"]["left_shoulder"] == pytest.approx(got["arm_deg"]["shoulder"], abs=1e-3)
    r.close()


# ---------------- 4. chốt E-stop phần mềm ----------------
def make_latch(**lv):
    g = FakeGPIO(**lv)
    stops = []
    return g, SafetyLatch(g, on_stop=lambda: stops.append(1), settle_s=0), stops


def test_latch_startup_no_auto_enable():
    g, lt, _ = make_latch()                 # E-stop nhả, RELAY_EN đang cao từ trước
    assert g.levels["RELAY_EN"] is False and not lt.ok and not lt.poll()


def test_latch_reset_rising_edge_then_press_release():
    g, lt, stops = make_latch()
    lt.reset()
    assert lt.ok and g.levels["RELAY_EN"] and g.rising_edges() == 1
    g.levels["ESTOP_SENSE"] = False         # nhấn
    assert not lt.poll() and g.levels["RELAY_EN"] is False and stops == [1]
    lt.poll()
    assert stops == [1]                     # không gọi lặp
    g.levels["ESTOP_SENSE"] = True          # nhả: không tự chạy lại
    for _ in range(5):
        assert not lt.poll()
    assert g.levels["RELAY_EN"] is False
    lt.reset()
    assert lt.ok and g.rising_edges() == 2


def test_latch_reset_refused_while_pressed():
    g, lt, _ = make_latch(ESTOP_SENSE=False)
    with pytest.raises(ResetRefused):
        lt.reset()
    assert g.levels["RELAY_EN"] is False and g.rising_edges() == 0


def test_latch_restart_after_run_stays_off():
    g, lt, _ = make_latch()
    lt.reset()
    lt2 = SafetyLatch(g)                    # Pi khởi động lại
    assert g.levels["RELAY_EN"] is False and not lt2.ok


def test_latch_sense_drop_and_reset_requirement():
    g, lt, stops = make_latch(LATCH_SENSE=False)
    with pytest.raises(ResetRefused):
        lt.reset()
    g.levels["LATCH_SENSE"] = True
    lt.reset()
    g.levels["LATCH_SENSE"] = False         # chốt phần cứng nhả khi đang chạy
    assert not lt.poll() and stops == [1] and g.levels["RELAY_EN"] is False


def test_latch_with_feetech_backend(port):
    g = FakeGPIO()
    be = FeetechBackend(port=port, **NOSLEEP)
    be.latch = SafetyLatch(g, on_stop=be.emergency_stop)
    with pytest.raises(RuntimeError, match="reset"):
        be.enable()
    be.latch.reset()
    be.enable()
    assert all(torque(port, i) == 1 for i in ALL_IDS)
    g.levels["ESTOP_SENSE"] = False
    be.step(0.02)
    assert all(torque(port, i) == 0 for i in ALL_IDS) and not be.enabled


# ---------------- 5. lò xo đứt ----------------
# P-L bỏ lò xo cân bằng: SpringMonitor không áp dụng (mô hình không có tendon). Bỏ qua, không nới ngưỡng.
NO_SPRING = pytest.mark.skipif(spec.BAL_N_PER_SIDE == 0, reason="P-L không có lò xo cân bằng vai; SpringMonitor chỉ cho P-A")
def run_monitor(qs, qe, scale, seconds=3.0, dt=0.02):
    mon = SpringMonitor()
    meas = shoulder_model_torque(D(qs), D(qe), 0.0, scale)
    t_alarm = None
    for i in range(int(seconds / dt)):
        if mon.update(D(qs), D(qe), 0.0, meas, dt) and t_alarm is None:
            t_alarm = (i + 1) * dt
    return mon, t_alarm


@NO_SPRING
@pytest.mark.parametrize("qs,qe", [(0, 0), (30, 90), (-45, 45)])
def test_spring_ok_no_alarm(qs, qe):
    assert run_monitor(qs, qe, 1.0)[1] is None


@NO_SPRING
@pytest.mark.parametrize("scale", [0.5, 0.0])
def test_spring_broken_alarm_after_hold(scale):
    mon, t = run_monitor(0, 0, scale)
    assert t is not None and mon.hold - 0.03 <= t <= mon.hold + 0.05 and "lò xo" in mon.message


@NO_SPRING
def test_spring_short_glitch_and_unobservable_pose():
    mon = SpringMonitor()
    bad = shoulder_model_torque(0.0, 0.0, 0.0, 0.0)
    good = shoulder_model_torque(0.0, 0.0, 0.0, 1.0)
    for _ in range(20):                     # 0,4 s lệch rồi về đúng
        mon.update(0.0, 0.0, 0.0, bad, 0.02)
    for _ in range(100):
        mon.update(0.0, 0.0, 0.0, good, 0.02)
    assert not mon.alarm
    mon2, t = run_monitor(-80, 0, 0.5)      # gần thẳng đứng: một lò xo đứt chỉ đổi 0,28 N·m, không phán
    assert t is None and not mon2.observable


# ---------------- 6. QC2: chốt E-stop bắt buộc, luồng nền, lỗi servo, bật lại ngoài dải ----------------
import time as _time  # noqa: E402
import warnings as _warnings  # noqa: E402

from faibotic.backends.feetech import EStopActive, OutOfSoftLimits  # noqa: E402

LATCHED = dict(sleep=lambda s: None, latch_period=0.002, paired_joints=spec.PA_PAIRED_JOINTS, dock=DockSwitches(_DockGPIO(DOCK_L=True, DOCK_R=True)))


def wait_until(cond, timeout=1.0):
    t0 = _time.monotonic()
    while _time.monotonic() - t0 < timeout:
        if cond():
            return True
        _time.sleep(0.002)
    return cond()


def test_no_latch_refuses_enable_by_default(port):
    be = FeetechBackend(port=port, sleep=lambda s: None)      # GPIO thật chưa có: không có latch
    assert be.latch is None
    with pytest.raises(EStopActive, match="allow_no_latch"):
        be.enable()
    assert all(torque(port, i) == 0 for i in ALL_IDS)
    with pytest.raises(EStopActive):
        be.write_joints({"left_shoulder": 0.1})


def test_allow_no_latch_warns(port):
    with pytest.warns(RuntimeWarning, match="KHÔNG có chốt"):
        be = FeetechBackend(port=port, sleep=lambda s: None, allow_no_latch=True)
    be.enable()
    assert be.enabled


def test_gpio_creates_latch_wired_and_monitored(port):
    g = FakeGPIO()
    be = FeetechBackend(port=port, gpio=g, **LATCHED)
    assert isinstance(be.latch, SafetyLatch) and be.latch.monitor_alive
    with pytest.raises(EStopActive, match="reset"):
        be.enable()
    be.latch.reset()
    be.enable()
    assert all(torque(port, i) == 1 for i in ALL_IDS)
    # không nối on_stop thủ công, KHÔNG gọi step(): luồng nền vẫn tắt mô-men
    g.levels["ESTOP_SENSE"] = False
    assert wait_until(lambda: not be.enabled)
    assert all(torque(port, i) == 0 for i in ALL_IDS) and g.levels["RELAY_EN"] is False
    with pytest.raises(EStopActive):
        be.write_joints({"left_shoulder": 0.1})
    be.close()
    assert not be.latch.monitor_alive


def test_enable_race_estop_during_enable_ends_off(port):
    """QC2 vòng 2: E-stop nhấn giữa enable() (sau khi kiểm chốt, trước khi bật mô-men) không được để servo bật."""
    g = FakeGPIO()
    be = FeetechBackend(port=port, gpio=g, **LATCHED)
    be.latch.reset()
    orig = be.bus.sync_write

    def sw(reg, data):
        if reg == P.TORQUE_LIMIT:                    # ngay trước lệnh bật mô-men
            g.levels["ESTOP_SENSE"] = False
            assert wait_until(lambda: be.latch.state == "estop")
        return orig(reg, data)

    be.bus.sync_write = sw
    with pytest.raises(EStopActive):
        be.enable()
    be.bus.sync_write = orig
    assert not be.enabled
    assert all(torque(port, i) == 0 for i in ALL_IDS)
    be.close()


def test_latch_assigned_later_keeps_user_callback(port):
    be = FeetechBackend(port=port, **NOSLEEP)
    calls = []
    g = FakeGPIO()
    be.latch = SafetyLatch(g, on_stop=lambda: calls.append("user"), settle_s=0)
    be.latch.reset()
    be.enable()
    g.levels["LATCH_SENSE"] = False                  # chốt phần cứng nhả khi đang chạy
    assert wait_until(lambda: calls == ["user"])
    assert all(torque(port, i) == 0 for i in ALL_IDS) and not be.enabled
    be.close()


class BrokenGPIO(FakeGPIO):
    def __init__(self):
        super().__init__()
        self.broken = False

    def read(self, pin):
        if self.broken:
            raise OSError("gpiod: mất thiết bị")
        return super().read(pin)


def test_monitor_thread_death_stops_robot(port):
    g = BrokenGPIO()
    be = FeetechBackend(port=port, gpio=g, **LATCHED)
    be.latch.reset()
    be.enable()
    g.broken = True
    assert wait_until(lambda: not be.latch.monitor_alive and not be.enabled)
    assert all(torque(port, i) == 0 for i in ALL_IDS)
    assert "OSError" in be.latch.monitor_error
    with pytest.raises(EStopActive, match="luồng"):
        be.write_joints({"left_shoulder": 0.1})


def test_reset_rereads_latch_sense_after_edge():
    g = FakeGPIO()
    slept = []

    def sleep(s):                        # K1 không hút: LATCH_SENSE rớt trong lúc chờ RC
        slept.append(s)
        g.levels["LATCH_SENSE"] = False
    lt = SafetyLatch(g, settle_s=0.3, sleep=sleep)
    with pytest.raises(ResetRefused, match="không giữ"):
        lt.reset()
    assert slept == [0.3] and lt.state == "estop" and g.levels["RELAY_EN"] is False
    g.levels["LATCH_SENSE"] = True
    lt._sleep = lambda s: None
    lt.reset()
    assert lt.ok


@pytest.mark.parametrize("bit", [5, 2])          # quá tải, quá nhiệt
def test_servo_error_bit_trips_pair(be, port, bit):
    sid = PA_IDS["right_elbow_b"]
    port.errors[sid] = 1 << bit
    with pytest.raises(PairFault, match="right_elbow"):
        be.check_pairs()
    a = PA_IDS["right_elbow"]
    assert torque(port, a) == 0 and torque(port, sid) == 0
    assert torque(port, PA_IDS["left_elbow"]) == 1
    assert be.fault and "right_elbow" in be.fault
    with pytest.raises(PairFault):
        be.write_joints({"right_elbow": 0.2})


def test_enable_refused_outside_soft_limits_after_power_loss(port):
    cal = default_calibration(paired_joints=spec.PA_PAIRED_JOINTS)
    be = FeetechBackend(port=port, calibration=cal, **NOSLEEP)
    for n in ("left_shoulder", "left_shoulder_b"):    # tay nằm ở chặn cơ khí vai -88°
        port.set_position(PA_IDS[n], be.to_raw(n, D(-88)))
    with pytest.raises(OutOfSoftLimits, match="bằng tay"):
        be.enable()
    assert all(torque(port, i) == 0 for i in ALL_IDS) and not be.enabled
    for n in ("left_shoulder", "left_shoulder_b"):    # người dùng đưa tay về trong dải
        port.set_position(PA_IDS[n], be.to_raw(n, D(-75)))
    be.enable()
    assert be.enabled


def test_load_soft_limits_relocks_on_error(port):
    be = FeetechBackend(port=port, **NOSLEEP)
    sid = PA_IDS["left_shoulder"]
    orig = be.bus._bus.write

    def bad_write(s, reg, v):
        if s == sid and reg == P.MAX_ANGLE:
            raise P.ProtocolError("lỗi giữa chừng")
        return orig(s, reg, v)
    be.bus._bus.write = bad_write
    with pytest.raises(P.ProtocolError):
        be.load_soft_limits()
    assert port.mem[sid][P.LOCK[0]] == 1


def test_v043_soft_max_margins_exceed_pad_tolerance():
    """v0.4.3: biên mềm-đệm >= dung sai dày đệm PU (+-0,5 mm): vai (r 45) và khuỷu (r 61); chặn cứng khuỷu 133; lệnh vai 90 bị kẹp."""
    for name, soft in (("shoulder_up", spec.SHOULDER_SOFT_MAX_DEG), ("elbow_high", spec.ELBOW_SOFT_MAX_DEG)):
        p = spec.FAILSAFE_PADS[name]
        assert (p["contact_deg"] - soft) * math.pi / 180 * p["r_m"] * 1e3 >= 0.5 + 0.3, name
    assert spec.FAILSAFE_ELBOW_STOP_DEG == spec.FAILSAFE_PADS["elbow_high"]["hard_deg"] == 133.0
    r = Robot("sim")
    r.keepout = False   # G1/E3: test này kiểm giới hạn khớp/bảng ghép, tư thế cực (cổ tay thấp sau trục pan) nằm trong vùng cấm bệ; vùng cấm kiểm ở test_tram
    out = r.set_joint_positions({"left_shoulder": D(90), "left_elbow": D(125), "left_wrist_flex": D(0)})
    assert out["left_shoulder"] == pytest.approx(D(spec.SHOULDER_SOFT_MAX_DEG)) and out["left_elbow"] == pytest.approx(spec.elbow_max(D(spec.SHOULDER_SOFT_MAX_DEG)))   # v0.5: khuỷu hạ theo vai (cột nâng)
    r.close()


def test_wrist_couple_table_matches_cad_scan():
    """(v0.4.4,a): bảng ghép gắn với quét CAD theo cuộn (v0.4.5 DK-W4: cad/export/failsafe_v045_wrist_roll_scan.json, camera + giá M1-AR-009; scan_wrist_v044.py --sweep).
    Mọi tư thế quét có giao (vật cản + tự va chạm cẳng tay, mọi cuộn) phải bị bảng/giới hạn gập cấm; mọi tư thế quét trong giới hạn là sạch;
    tại mỗi nút, mức giao đầu tiên (nhỏ nhất qua mọi cuộn) cách ô bảng >= 3 độ (biên); dải cuộn phủ -160..160 gồm các góca."""
    import json
    from pathlib import Path
    scan = json.loads((Path(__file__).resolve().parents[1] / "cad" / "export" / "failsafe_v045_wrist_roll_scan.json").read_text())
    rows = scan["rows"]
    assert len(rows) > 5000 and {-160, -90, -45, 0, 45, 90, 160} <= set(scan["cuon"]) and scan["bien_deg"] == 3.0
    wlo = math.degrees(spec.ARM_LIMITS["wrist_flex"][0])
    first = {}
    for qs, qe, qw, roll, hit in rows:
        wm = math.degrees(spec.wrist_max(D(qs), D(qe)))
        if hit:
            assert qw > wm + 1e-6 or qw < wlo - 1e-6, (qs, qe, qw, roll, hit, wm)     # tư thế giao nhưng giới hạn cho phép
            if qw >= 0:
                first[(qs, qe)] = min(first.get((qs, qe), 999.0), qw)
        if wlo - 1e-6 <= qw <= wm + 1e-6:
            assert not hit, (qs, qe, qw, roll)                                     # hợp lệ mà có giao
    S, E, T = spec.WRIST_COUPLE_SHOULDER_DEG, spec.WRIST_COUPLE_ELBOW_DEG, spec.WRIST_COUPLE_MAX_DEG
    assert [list(r) for r in T] == scan["WRIST_COUPLE_MAX_DEG"]
    for i, sd in enumerate(S):
        for j, ed in enumerate(E):
            assert first[(sd, ed)] - T[i][j] >= 3.0 - 1e-9, (sd, ed, first[(sd, ed)], T[i][j])

