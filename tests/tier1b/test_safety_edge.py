"""Tầng 1b nhóm (b): an toàn khi chạy (docs/S4/KE-HOACH-TANG-1B.md, T1b.R-15..R-25).

E-stop, DOCK, keepout, dốc đế, chốt lift, watchdog. Ca mà code hiện tại trượt đánh xfail(strict=True) kèm bản sửa kỳ vọng.
"""
import math
import random
import time

import numpy as np
import pytest

from faibotic import Robot, kinematics, limits, spec
from faibotic.backends.feetech import DockOpen, EStopActive, LiftFault
from faibotic.feetech import protocol as P
from faibotic.teleop import K, Teleop

DT = 0.02
LIMIT_S = 0.2
TORQUE = P.TORQUE_ENABLE[0]


def press_estop(rig):
    rig.t_inject = time.monotonic()
    rig.estop.levels["ESTOP_SENSE"] = False


def wait_for(cond, timeout=1.0):
    t = time.monotonic()
    while time.monotonic() - t < timeout:
        if cond():
            return True
        time.sleep(0.001)
    return cond()


def first_torque_off_after(rig, t0):
    ts = [e[0] for sid in rig.port.mem for e in rig.port.writes_to(TORQUE, sid) if e[0] >= t0 and e[3] == b"\x00"]
    return min(ts) if ts else None


def run(rig, fn):
    try:
        fn()
    except BaseException as e:      # noqa: BLE001
        return e
    return None


# ---------------------------------------------------------------- R-15
def test_R15_estop_giua_move_joints(u):
    """T1b.R-15: hạ ESTOP ở 50% hành trình: cuộn chốt cắt trước, rồi mọi servo TORQUE 0; lệnh kế ném EStopActive; reset chốt không tự chạy."""
    rig = u.build()
    coil_t = []
    w = rig.lift_hw.write
    rig.lift_hw.write = lambda pin, lv: (coil_t.append((time.monotonic(), lv)), w(pin, lv))[1]
    rig.inject_at(25, lambda: press_estop(rig))
    exc = run(rig, lambda: rig.robot.move_joints({"left_pan": 0.5, "right_pan": -0.5}, duration=1.0))
    assert isinstance(exc, EStopActive), repr(exc)
    assert wait_for(rig.ttat, LIMIT_S), u.ttat_report(rig)
    t_off = first_torque_off_after(rig, rig.t_inject)
    assert t_off is not None and t_off - rig.t_inject <= LIMIT_S
    cut = [t for t, lv in coil_t if lv is False and t >= rig.t_inject]
    assert cut and min(cut) <= t_off, "cuộn chốt phải cắt trước (hoặc cùng lúc) khi mô-men servo tắt"
    # reset chốt xong vẫn không tự chạy
    rig.estop.levels["ESTOP_SENSE"] = True
    rig.latch.reset()
    for _ in range(10):
        rig.robot.step()
    assert not rig.backend.enabled and rig.ttat()


# ---------------------------------------------------------------- R-16
def _teleop_loop(rig, keys):
    tele = Teleop(rig.robot)
    dt, n, exc = 1 / 30, 0, None
    for i in range(200):
        if i == 40:
            press_estop(rig)
        try:
            tele.update(keys, dt)
            rig.robot.step(dt)
            n += 1
        except BaseException as e:      # noqa: BLE001
            exc = e
            break
    return exc, n


def test_R16_estop_khi_dang_teleop(u):
    """T1b.R-16 (a): vòng teleop (update + step) 30 Hz có lệnh lift mỗi chu kỳ: ESTOP thì vòng thoát bằng EStopActive, không ngoại lệ khác."""
    rig = u.build()
    exc, n = _teleop_loop(rig, {K["W"], K["R"]})
    assert isinstance(exc, EStopActive), repr(exc)
    assert wait_for(rig.ttat, LIMIT_S), u.ttat_report(rig)
    assert time.monotonic() - rig.t_inject <= LIMIT_S and n < 200


def test_R16_estop_vong_chi_co_lenh_de(u):
    """T1b.R-16 (b): vòng teleop chỉ giữ phím đế (không có lệnh ghi khớp): ESTOP vẫn phải làm vòng thoát bằng EStopActive."""
    rig = u.build()
    exc, n = _teleop_loop(rig, {K["W"]})
    assert isinstance(exc, EStopActive), f"vòng chạy hết {n} chu kỳ, exc={exc!r}"
    assert rig.ttat()


# ---------------------------------------------------------------- R-17
def test_R17_estop_khi_de_dang_chay(u):
    """T1b.R-17: E-stop ở đỉnh vận tốc đế; reset rồi enable(): dốc về 0, GOAL_SPEED đầu tiên = 0, không nhảy về đích cũ."""
    rig = u.build()
    be = rig.backend
    rig.inject_at(60, lambda: press_estop(rig))
    exc = run(rig, lambda: rig.robot.move_base(0.0, 0.5, 0.0, duration=2.0))
    assert exc is None or isinstance(exc, EStopActive), repr(exc)
    assert wait_for(rig.ttat, LIMIT_S), u.ttat_report(rig)
    assert all(u.torque(rig.port, i) == 0 for i in u.WHEEL_IDS)
    rig.estop.levels["ESTOP_SENSE"] = True
    rig.latch.reset()
    n0 = len(rig.port.timeline)
    be.enable()
    assert be.base_velocity().tolist() == [0.0, 0.0, 0.0]
    for _ in range(5):
        rig.robot.step()
    gs = [(sid, P.decode_signed(P.from_u16(e[3]))) for e in list(rig.port.timeline)[n0:] if e[2] == P.GOAL_SPEED[0] for sid in [e[1]]
          if sid in u.WHEEL_IDS]
    assert all(v == 0 for _, v in gs), gs
    assert be.base_velocity().tolist() == [0.0, 0.0, 0.0]


# ---------------------------------------------------------------- R-18
@pytest.mark.parametrize("what", ["move_joints", "move_base"])
def test_R18_dock_mo_khi_dang_chay(u, what):
    """T1b.R-18: DOCK_L hạ giữa chuyển động: DockOpen trong 1 chu kỳ, TTAT, dock_fault giữ tới clear_fault(), enable() bị từ chối khi khoá còn mở."""
    rig = u.build()
    rig.inject_at(8, lambda: rig.dock.levels.__setitem__("DOCK_L", False))
    if what == "move_joints":
        exc = run(rig, lambda: rig.robot.move_joints({"left_pan": 0.4}, duration=1.0))
    else:
        exc = run(rig, lambda: rig.robot.move_base(0.0, 0.3, 0.0, duration=1.0))
    assert isinstance(exc, DockOpen) and "DOCK_L" in str(exc), repr(exc)
    assert rig.steps - rig.step_inject <= 1
    assert rig.ttat() and not rig.backend.enabled
    assert rig.backend.dock_fault
    with pytest.raises(DockOpen, match="DOCK_L"):
        rig.backend.enable()
    rig.dock.levels["DOCK_L"] = True
    with pytest.raises(DockOpen):                        # khoá đóng lại nhưng chưa clear_fault(): vẫn chặn
        rig.backend.enable()
    rig.backend.clear_fault()
    rig.backend.enable()
    assert rig.backend.enabled


# ---------------------------------------------------------------- R-19
def test_R19_dock_chap_chon_mot_chu_ky(u):
    """T1b.R-19: DOCK rớt đúng một chu kỳ rồi đóng lại: vẫn dừng (không lọc nhiễu, an toàn trước)."""
    rig = u.build()
    orig, cnt = rig.backend.step, [0]

    def step(dt):
        cnt[0] += 1
        if cnt[0] == 8:
            rig.dock.levels["DOCK_R"] = False
        try:
            return orig(dt)
        finally:
            if cnt[0] == 8:
                rig.dock.levels["DOCK_R"] = True
    rig.backend.step = step
    exc = run(rig, lambda: rig.robot.move_joints({"left_pan": 0.4}, duration=1.0))
    assert isinstance(exc, DockOpen), repr(exc)
    assert rig.ttat() and rig.backend.dock_fault


def test_R19_loi_doc_gpio_coi_la_khoa_mo(u):
    """T1b.R-19: gpio.read ném lỗi (đứt dây/opto hỏng): coi là khoá mở, dừng."""
    rig = u.build()

    def boom(pin):
        raise OSError("GPIO chết")
    rig.inject_at(8, lambda: setattr(rig.dock, "read", boom))
    exc = run(rig, lambda: rig.robot.move_joints({"left_pan": 0.4}, duration=1.0))
    assert isinstance(exc, DockOpen) and "khoá trạm" in str(exc), repr(exc)
    assert rig.ttat()


# ---------------------------------------------------------------- R-20
def _rand_arm_targets(rng):
    t = {}
    for s in spec.SIDES:
        for j in spec.ARM_JOINTS:
            lo, hi = spec.ARM_LIMITS[j]
            t[spec.joint_name(s, j)] = rng.uniform(lo, hi)
    t["lift"] = rng.uniform(*spec.LIFT_RANGE)
    return t


def _storm(u, backend, check_cmd):
    if backend == "feetech":
        rig = u.build(latch=False, dock=False, lift_latch=False, record=False, enable=False)
        robot, cmd = rig.robot, lambda: rig.backend._cmd
    else:
        robot = Robot("sim")
        cmd = lambda: robot.backend._targets      # noqa: E731
    rng = random.Random(20)
    n_ok = n_bad = 0
    for _ in range(5000):
        tgt = _rand_arm_targets(rng)
        before = dict(robot._targets)
        try:
            robot.set_joint_positions(tgt)
            n_ok += 1
        except limits.KeepoutError:
            n_bad += 1
            assert robot._targets == before, "đích bị đổi sau khi từ chối"
            continue
        if check_cmd:
            c = cmd()
            for s in spec.SIDES:
                q = [c[spec.joint_name(s, j)] for j in spec.ARM_JOINTS]
                v = limits.keepout_violations(s, q, robot.config, c["lift"])
                assert not v, f"đích đã gửi xuống backend vi phạm vùng cấm: {v}"
    assert n_ok > 0 and n_bad > 0, (n_ok, n_bad)         # phép thử có cả đích hợp lệ lẫn vi phạm
    if backend == "sim":
        robot.close()


@pytest.mark.parametrize("backend", ["feetech", "sim"])
def test_R20_keepout_tu_choi_khong_doi_trang_thai(u, backend):
    """T1b.R-20 (a): 5000 set_joint_positions ngẫu nhiên: chỉ có chấp nhận hoặc KeepoutError (không ngoại lệ khác); từ chối thì _targets giữ nguyên."""
    _storm(u, backend, check_cmd=False)


@pytest.mark.parametrize("backend", ["feetech", "sim"])
def test_R20_keepout_dich_da_gui_xuong_backend(u, backend):
    """T1b.R-20 (b): sau mỗi lệnh được chấp nhận, tư thế backend đã nhận (feetech _cmd, sim _targets) ngoài vùng cấm."""
    _storm(u, backend, check_cmd=True)


# ---------------------------------------------------------------- R-21
def test_R21_keepout_tren_duong_move_tcp():
    """T1b.R-21: điểm đầu (tư thế gốc) và đích hợp lệ, nhưng đường thẳng cắt vùng cấm (khay): từ chối trước khi gửi lệnh nào."""
    r = Robot("sim")
    goal = [0.356, 0.1315, 0.1924]
    q = r.solve_ik("left", goal)
    assert q is not None and not limits.keepout_violations("left", q, "di_dong", 0.0)       # đích hợp lệ
    calls = []
    orig = r.backend.write_joints
    r.backend.write_joints = lambda t: (calls.append(dict(t)), orig(t))[1]
    exc = run(r, lambda: r.move_tcp("left", goal, duration=2.0))
    assert isinstance(exc, limits.KeepoutError), repr(exc)
    assert calls == [], f"đã gửi {len(calls)} lệnh xuống backend trước khi từ chối"
    r.close()


# ---------------------------------------------------------------- R-22
def _ramp_invariants(vel, label):
    prev = np.zeros(3)
    for k, v in enumerate(vel):
        dv = float(np.linalg.norm(v[:2] - prev[:2])) / DT
        dw = abs(v[2] - prev[2]) / DT
        assert dv <= spec.BASE_MAX_ACC * 1.001 + 1e-9, f"{label} chu kỳ {k}: |Δv|/dt = {dv}"
        assert dw <= spec.BASE_MAX_ALPHA * 1.001 + 1e-9, f"{label} chu kỳ {k}: |Δω|/dt = {dw}"
        assert float(np.linalg.norm(v[:2])) * abs(v[2]) <= spec.BASE_TURN_ACC_MAX * 1.001 + 1e-9, f"{label} chu kỳ {k}: |v||ω|"
        assert float(np.linalg.norm(v[:2])) <= spec.BASE_MAX_V + 1e-9
        prev = v


def _step_pattern(rng, k):
    """Lệnh bậc: 0 -> max -> -max -> 0 xoay vòng, kèm hướng và ω ngẫu nhiên."""
    a = rng.uniform(0, 2 * math.pi)
    mag = (0.0, 1.0, -1.0, 0.0)[k % 4]
    return mag * 10 * math.cos(a), mag * 10 * math.sin(a), mag * 10 * rng.uniform(-1, 1)


def test_R22_doc_de_lenh_bac_tren_bus_gia(u):
    """T1b.R-22 (bus giả): lệnh bậc, 100 lần set_base_velocity mỗi chu kỳ, 10.000 chu kỳ: bất biến dốc; GOAL_SPEED khớp dốc."""
    rig = u.build()
    be, rng, vel, worst = rig.backend, random.Random(22), [], 0.0
    for k in range(10000):
        cmd = _step_pattern(rng, k // 20)
        for _ in range(100):
            rig.robot.set_base_velocity(*cmd)
        be.step(DT)
        v = be.base_velocity().copy()
        vel.append(v)
        if k % 50 == 0:
            worst = max(worst, float(np.max(np.abs(kinematics.base_twist(be.read_wheel_speeds()) - v))))
    _ramp_invariants(vel, "feetech")
    assert worst < 2e-3, f"GOAL_SPEED lệch dốc {worst}"


def test_R22_doc_de_lenh_bac_tren_sim():
    """T1b.R-22 (sim): như trên, vận tốc qua dốc của backend sim."""
    r = Robot("sim")
    rng, vel = random.Random(22), []
    for k in range(10000):
        cmd = _step_pattern(rng, k // 20)
        for _ in range(100):
            r.set_base_velocity(*cmd)
        r.step()
        vel.append(r.backend.base_velocity())
    _ramp_invariants(vel, "sim")
    r.close()


# ---------------------------------------------------------------- R-23
def _lift_fault_run(u):
    rig = u.build()
    lid = u.LIFT_ID
    rig.inject_at(30, lambda: setattr(rig.lift_hw, "stuck", "engaged"))      # chốt bật ra giữa chuyển động (cảm biến "rút" rớt)
    exc = run(rig, lambda: rig.robot.move_joints({"lift": 0.03}, duration=3.0))
    return rig, exc, lid


def test_R23_loi_chot_lift_giua_chuyen_dong(u):
    """T1b.R-23 (a): LiftFault, lift giữ rồi cài chốt, không lệnh GOAL_POSITION lift nào sau lỗi."""
    rig, exc, lid = _lift_fault_run(u)
    assert isinstance(exc, LiftFault), repr(exc)
    assert rig.lift_hw.en is False and rig.backend.lift_fault
    t_fault = time.monotonic()
    n0 = len(rig.port.writes_to(P.GOAL_POSITION[0], lid))
    for _ in range(5):
        run(rig, rig.robot.step)
    assert len(rig.port.writes_to(P.GOAL_POSITION[0], lid)) == n0, "còn đẩy GOAL_POSITION lift sau lỗi"
    assert time.monotonic() - t_fault <= LIMIT_S


def test_R23_khong_bao_ngoai_le_sau_loi_lift(u):
    """T1b.R-23 (b): sau LiftFault, 20 lần step() liên tiếp không ném thêm ngoại lệ."""
    rig, exc, _ = _lift_fault_run(u)
    assert isinstance(exc, LiftFault)
    errs = [run(rig, rig.robot.step) for _ in range(20)]
    assert all(e is None for e in errs), f"{sum(e is not None for e in errs)}/20 chu kỳ ném lại {type(errs[0]).__name__}"


# ---------------------------------------------------------------- R-24
@pytest.mark.parametrize("backend", ["feetech", "sim"])
@pytest.mark.parametrize("end", ["top", "bottom"])
def test_R24_lift_cham_cu(u, backend, end):
    """T1b.R-24: công tắc hành trình đóng: dừng chiều đó và giữ vị trí; chiều ngược vẫn chạy; không ngoại lệ lặp."""
    sw = limits.FakeLiftSwitches()
    if backend == "feetech":
        rig = u.build(latch=False, dock=False, lift_latch=False, enable=True, lift_switches=sw)
        robot = rig.robot
    else:
        robot = Robot("sim")
        robot.backend.lift_switches = sw
    robot.move_joints({"lift": 0.02}, duration=2.0) if backend == "feetech" else robot.move_joints({"lift": 0.2}, duration=1.0)
    for _ in range(60):
        robot.step()                                        # chờ lift lắng
    start = robot.get_state()["lift"]
    assert start > 0.01
    setattr(sw, end, True)
    up = end == "top"
    robot.set_joint_positions({"lift": start + (0.1 if up else -0.1)})
    errs = [run(robot, robot.step) for _ in range(40)]
    assert all(e is None for e in errs), errs[:1]
    # sim: vị trí đo còn võng do trọng lực (3-4 mm), nên đo theo đích backend giữ lại; feetech đo vị trí đọc về
    now = (lambda: robot.backend._targets["lift"]) if backend == "sim" else (lambda: robot.get_state()["lift"])
    assert abs(now() - start) < 0.003, "lift đi tiếp chiều bị công tắc chặn"
    robot.set_joint_positions({"lift": start + (-0.02 if up else 0.02)})        # chiều ngược lại được
    for _ in range(120):
        robot.step()
    assert (now() < start - 0.01) if up else (now() > start + 0.01)
    if backend == "sim":
        robot.close()


# ---------------------------------------------------------------- R-25
def test_R25_watchdog_may_chu_ngung_goi_step(u):
    """T1b.R-25: đế đang chạy 0,3 m/s, luồng chính ngủ (giả treo): bánh về 0 trong T_WD + 1 chu kỳ giám sát."""
    t_wd = getattr(spec, "WATCHDOG_S", 0.2)
    rig = u.build()
    rig.robot.set_base_velocity(0.0, 0.3, 0.0)
    for _ in range(40):
        rig.robot.step()
    assert any(u.goal_speed(rig.port, i) != 0 for i in u.WHEEL_IDS), "đế chưa chạy"
    t0 = time.monotonic()
    ok = wait_for(lambda: all(u.goal_speed(rig.port, i) == 0 for i in u.WHEEL_IDS), t_wd + 0.1)
    assert ok, f"sau {time.monotonic() - t0:.2f} s bánh vẫn giữ tốc độ cũ"
    with pytest.raises(Exception) as ei:
        rig.robot.step()
    assert type(ei.value).__name__ == "WatchdogHet"
