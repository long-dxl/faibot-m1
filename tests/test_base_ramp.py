"""(E2): dốc gia tốc đế và trần độ lớn vận tốc, chống lật động >= 1,5 ở ca xấu nhất."""
import math

import numpy as np
import pytest

import tipover_dong as TD
from faibotic import limits, spec
from faibotic.backends import feetech as FB


# ---------------- chống lật ----------------

def test_chong_lat_dong_o_bien_gioi_han():
    """Biên bốn trần cùng lúc, lift đỉnh, khay 1,3 kg góc xa, tay duỗi tải 1 kg/tay, khối tay cận trên + 1 STS3215 dự phòng."""
    f = TD.envelope()
    assert f >= spec.BASE_TIP_MIN, f


def test_chong_lat_con_bien_cho_tay_6dof():
    """(tay 6+1 đã vào spec): servo yaw nằm trong ARM_MASS_MOVING; thêm dự phòng BASE_TIP_ARM_EXTRA_KG (70,5 g/tay) vẫn >= 1,5.
    Trước test này cộng hai lần dự phòng để chờ lượt 6 DOF; một lần trong hai đã thành khối thật của servo yaw."""
    yaw = [m for b, n, m, _ in spec.ARM_POINT_MASSES if "yaw" in n and "servo" in n]
    assert yaw == [spec.STS3215.mass_kg]
    assert TD.envelope(extra_arm_kg=0.0) >= spec.BASE_TIP_MIN
    assert TD.envelope(extra_arm_kg=spec.BASE_TIP_ARM_EXTRA_KG) >= spec.BASE_TIP_MIN


def test_gia_toc_cu_0_5_khong_dat():
    """Bằng chứng vì sao cần dốc: 0,5 m/s² (giả định cũ) ở ca xấu nhất < 1,5; tĩnh >= 1,5."""
    assert TD.worst(0.5, 0, 0, spec.BASE_TIP_ARM_EXTRA_KG) < spec.BASE_TIP_MIN
    assert TD.worst(0.0, 0, 0, spec.BASE_TIP_ARM_EXTRA_KG) >= spec.BASE_TIP_MIN


def test_mo_hinh_khop_tipover_cad():
    """tipover_dong không gia tốc, tay URDF, lift thấp: trùng tipover.pl_min (kiểm bản vector hoá)."""
    import json
    import tipover as TP
    M = json.load(open(TD.ROOT / "cad" / "export" / "M1-PL-masses.json", encoding="utf-8"))
    ref = TP.pl_min(M, acc=0.3, z_sh=spec.SHOULDER_Z0)[0]
    _, cfg, _ = TD._cfg(False)
    old = spec.ARM_MASS_MOVING
    try:
        spec.ARM_MASS_MOVING = 0.0            # dùng khối tay URDF như tipover.pl_min
        got = TD.worst(0.3, 0, 0, 0.0, lift_top=False, pan_step=5.0, ndir=72)
    finally:
        spec.ARM_MASS_MOVING = old
    assert got == pytest.approx(ref, abs=0.003)


# ---------------- kẹp đích ----------------

def test_do_lon_van_toc_cheo():
    vx, vy, wz = limits.base_limit_goal(0.4, 0.4, 0.0)
    assert math.hypot(vx, vy) == pytest.approx(spec.BASE_MAX_V)
    assert vx == pytest.approx(vy)                         # giữ hướng
    vx, vy, wz = limits.base_limit_goal(1.0, -2.0, 5.0)
    assert math.hypot(vx, vy) <= spec.BASE_MAX_V + 1e-12
    assert abs(wz) * math.hypot(vx, vy) <= spec.BASE_TURN_ACC_MAX + 1e-12
    assert abs(limits.base_limit_goal(0, 0, -9)[2]) == pytest.approx(min(spec.BASE_MAX_W, spec.BASE_MAX_W_TIP))


def _check_trace(tr, dt):
    tr = np.asarray(tr)
    v = tr[:, :2]
    assert np.all(np.linalg.norm(v, axis=1) <= spec.BASE_MAX_V + 1e-9)
    assert np.all(np.abs(tr[:, 2]) <= spec.BASE_MAX_W_TIP + 1e-9)
    assert np.all(np.linalg.norm(v, axis=1) * np.abs(tr[:, 2]) <= spec.BASE_TURN_ACC_MAX + 1e-9)
    d = np.diff(tr, axis=0) / dt
    assert np.all(np.linalg.norm(d[:, :2], axis=1) <= spec.BASE_MAX_ACC + 1e-9)
    assert np.all(np.abs(d[:, 2]) <= spec.BASE_MAX_ALPHA + 1e-9)


def test_doc_ngau_nhien_khong_vuot_tran():
    rng = np.random.default_rng(1)
    r = limits.BaseRamp()
    dt, tr = 0.02, [np.zeros(3)]
    for _ in range(40):
        r.set_goal(*rng.uniform([-1, -1, -3], [1, 1, 3]))
        for _ in range(int(rng.integers(1, 120))):
            tr.append(r.step(dt))
    _check_trace(tr, dt)


def test_doc_toi_dich_va_dung():
    r = limits.BaseRamp()
    r.set_goal(0.4, 0, 0)
    n = 0
    while not r.settled:
        r.step(0.02); n += 1
    assert n * 0.02 == pytest.approx(spec.BASE_MAX_V / spec.BASE_MAX_ACC, abs=0.03)
    r.set_goal(0, 0, 0)
    while not r.settled:
        r.step(0.02)
    assert not np.any(r.cur)


# ---------------- sim ----------------

@pytest.fixture(scope="module")
def robot():
    from faibotic import Robot
    r = Robot("sim")
    yield r
    r.close()


def _measure(robot, cmd, seconds):
    be = robot.backend
    dt = robot.dt
    vel, pos = [], []
    robot.set_base_velocity(*cmd)
    for _ in range(int(round(seconds / dt))):
        robot.step()
        vel.append(be.base_velocity())
        pos.append(be._base_target.copy())
    return np.array(vel), np.array(pos), dt


def test_sim_cheo_va_gia_toc(robot):
    """Lệnh chéo (0,4; 0,4) nhảy bậc: vận tốc đo từ tích phân tư thế đế <= 0,40 m/s, gia tốc <= BASE_MAX_ACC."""
    robot.backend.reset()
    robot.backend.base_ramp.reset()
    vel, pos, dt = _measure(robot, (0.4, 0.4, 0.0), spec.BASE_MAX_V / spec.BASE_MAX_ACC + 1.0)
    _check_trace(np.vstack([np.zeros(3), vel]), dt)
    v_meas = np.linalg.norm(np.diff(pos[:, :2], axis=0) / dt, axis=1)          # khung thế giới, yaw 0
    a_meas = np.abs(np.diff(v_meas)) / dt
    assert v_meas.max() <= spec.BASE_MAX_V + 1e-6
    assert v_meas.max() >= spec.BASE_MAX_V - 1e-3                            # tới trần
    assert a_meas.max() <= spec.BASE_MAX_ACC + 1e-6
    robot.stop_base()
    assert not np.any(robot.backend.base_velocity())


def test_sim_move_base_di_du_quang(robot):
    """move_base tự kéo dài thời gian theo trần quãng đi đủ (vòng hở) và dốc không phải cắt."""
    robot.backend.reset()
    robot.backend.base_ramp.reset()
    robot.move_base(dx=0.3, dy=0.1, duration=1.0)
    x, y, _ = robot.backend.base_pose()
    assert x == pytest.approx(0.3, abs=0.01) and y == pytest.approx(0.1, abs=0.01)


# ---------------- feetech (bus giả) ----------------

def test_feetech_ghi_acceleration_va_doc_banh():
    from faibotic.feetech.mockbus import FakeServoPort
    be = FB.FeetechBackend(port=FakeServoPort(list(spec.SERVO_IDS.values())), enable=True, sleep=lambda s: None, allow_no_latch=True)
    acc = FB.wheel_acc_raw()
    assert 1 <= acc <= 254
    for n in FB.WHEELS:
        assert be.bus.read(be.ids[n], FB.P.ACCELERATION) == acc
    tr = [np.zeros(3)]
    for _ in range(int((spec.BASE_MAX_V / spec.BASE_MAX_ACC + 1) / be.control_dt)):
        be.set_base_velocity(0.4, 0.4, 0.0)            # đặt đích mỗi chu kỳ (như teleop): dốc không được nhanh gấp đôi
        be.set_base_velocity(0.4, 0.4, 0.0)
        be.step(be.control_dt)
        tr.append(be.base_velocity())
        raw = [FB.P.decode_signed(be.bus.read(be.ids[n], FB.P.GOAL_SPEED)) for n in FB.WHEELS]
        assert np.allclose(np.abs(raw), np.abs(np.round(be._wheel_cmd * FB.P.STEPS_PER_REV / (2 * math.pi))), atol=1)
    _check_trace(tr, be.control_dt)
    assert np.linalg.norm(tr[-1][:2]) == pytest.approx(spec.BASE_MAX_V)


def test_acc_thanh_ghi_du_cho_doc_phan_mem():
    """Chặn phụ phần cứng không làm chậm dốc phần mềm: gia tốc góc bánh cho phép >= gia tốc bánh cần ở biên."""
    from faibotic import kinematics
    need = 0.0
    for th in np.linspace(0, 2 * math.pi, 73):
        a = spec.BASE_MAX_ACC + spec.BASE_TURN_ACC_MAX
        for sa in (1, -1):
            need = max(need, float(np.max(np.abs(kinematics.wheel_speeds(a * math.cos(th), a * math.sin(th), sa * spec.BASE_MAX_ALPHA)))))
    allowed = FB.wheel_acc_raw() * FB.STS_ACC_UNIT_STEPS * 2 * math.pi / FB.P.STEPS_PER_REV
    assert allowed >= need
