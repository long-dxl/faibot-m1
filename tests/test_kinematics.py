import math

import mujoco
import numpy as np
import pytest

from faibotic import kinematics as K
from faibotic import model as M
from faibotic import spec


@pytest.fixture(scope="module")
def sim():
    m = mujoco.MjModel.from_xml_string(M.robot_mjcf(scene=False))
    return m, mujoco.MjData(m)


def _set(m, d, q, lift):
    for s in spec.SIDES:
        for j, v in zip(spec.ARM_JOINTS, q):
            d.qpos[m.joint(f"{s}_{j}").qposadr[0]] = v
    d.qpos[m.joint("lift").qposadr[0]] = lift
    mujoco.mj_kinematics(m, d)


def test_fk_matches_mujoco(sim):
    m, d = sim
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = [rng.uniform(*spec.ARM_LIMITS[j]) for j in spec.ARM_JOINTS]
        lift = rng.uniform(*spec.LIFT_RANGE)
        _set(m, d, q, lift)
        for s in spec.SIDES:
            p = K.arm_mount(s, lift) + K.arm_fk(q, s)[0]
            assert np.allclose(p, d.site(f"{s}_tcp").xpos, atol=1e-9)


def test_shoulder_axis_height_matches_cad(sim):
    """Trục vai ở SHOULDER_Z0 + lift như CAD P-L (khớp URDF AlohaMini2 + D0: 0,5619 + lift, S1G; trước 0,55 + lift theo lift.py v0.4)."""
    import xml.etree.ElementTree as ET
    m, d = sim
    q0 = [0.0] * len(spec.ARM_JOINTS)
    urdf = {j.get("name"): j for j in ET.fromstring(M.robot_urdf()).iter("joint")}
    oz = lambda n: float(urdf[n].find("origin").get("xyz").split()[2])
    for lift in (0.0, 0.25, spec.LIFT_RANGE[1]):
        _set(m, d, q0, lift)
        for s in spec.SIDES:
            z_axis = K.arm_mount(s, lift)[2] + spec.D0
            assert z_axis == pytest.approx(0.561926 + lift, abs=1e-9)
            assert d.body(f"{s}_upper").xpos[2] == pytest.approx(0.561926 + lift, abs=1e-9)   # MJCF
            assert d.body(f"{s}_fore").xpos[2] == pytest.approx(0.561926 + lift, abs=1e-9)   # tay duỗi ngang: trục khuỷu cùng độ cao trục vai
            # tâm kẹp thấp hơn trục vai = trục gập (WF_DZ) + trục cuộn (ROLL_DZ) + tâm kẹp (TCP_DZ), theo URDF upstream + CAD kẹp
            assert (K.arm_mount(s, lift) + K.arm_fk(q0, s)[0])[2] == pytest.approx(0.561926 + lift + spec.WF_DZ + spec.ROLL_DZ + spec.TCP_DZ, abs=1e-9)
            assert oz("lift") + lift + oz(f"{s}_pan") + oz(f"{s}_shoulder") == pytest.approx(0.561926 + lift, abs=1e-9)   # URDF


@pytest.mark.parametrize("side", spec.SIDES)
def test_ik_roundtrip_tool_down(side):
    rng = np.random.default_rng(2)
    n_ok = 0
    for _ in range(300):
        p = np.array([rng.uniform(0.1, 0.35), rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.15)])
        q = K.arm_ik(p, grip_yaw=rng.uniform(-1, 1), side=side)
        if q is None:
            continue
        n_ok += 1
        pf, pitch = K.arm_fk(q, side)
        assert np.allclose(pf, p, atol=1e-9)
        assert pitch == pytest.approx(K.TOOL_DOWN)
        assert K.within_limits(q)
    # cổ tay theo URDF upstream: khuỷu -> trục gập ngắn đi 30 mm, tâm kẹp lệch trục cuộn ~30 mm nên trục gập ra xa 3 cm khi kẹp chĩa xuống: 136 -> 123 mẫu (-10%), không do giới hạn khớp
    # giới hạn gập -100..120 -> -77,5..87,5 và bảng ghép theo cuộn (ô 87 ở vai -80): 123 -> 87 (trái) / 83 (phải) mẫu (-30%), mất ở các điểm thấp gần thân
    # (cần cổ tay > 87 để chĩa xuống). Không nới giới hạn; ngưỡng biên ~10%.
    assert n_ok > 75   # > 115; trước > 130. QC2 v0.4.1: ngưỡng 130 (biên ~6% như trước); v0.4.1 vòng 2: vai -85 -> -80 làm số mẫu IK khả thi 160 -> 138 (-14%); khuỷu -10 không đổi


def test_grip_yaw_sets_finger_axis(sim):
    """hàm khép theo trục y khung tay (khớp kẹp quanh z khung tay); grip_yaw giữ nghĩa cũ."""
    m, d = sim
    for yaw in (0.0, 0.4, -0.6):
        q = K.arm_ik([0.25, -0.05, -0.05], grip_yaw=yaw)
        _set(m, d, q, 0.0)
        R = d.body("left_hand").xmat.reshape(3, 3)
        assert np.allclose(R[:, 0], [0, 0, -1], atol=1e-9)  # dụng cụ chĩa xuống
        assert abs(abs(m.jnt_axis[m.joint("left_gripper").id][2]) - 1) < 1e-9   # khớp kẹp quanh z khung tay
        axis = R[:, 1]
        assert abs(abs(axis[0] * math.cos(yaw) + axis[1] * math.sin(yaw)) - 1) < 1e-9


def test_ik_with_explicit_roll_keeps_tcp():
    """Tâm kẹp lệch trục cuộn: IK với roll đặt thẳng phải tính lại vị trí, không ghi đè q5 sau IK."""
    for roll in (0.0, math.pi / 2, -1.0):
        p = [0.45, 0.02, 0.05]   # S1C (P-L): gốc tay ở đáy J1 (vai cao D0 0,148) nên điểm cũ (0,40; 0,02; 0) ngoài tầm
        q = K.arm_ik(p, pitch=0.0, roll=roll)
        assert q is not None and q[5] == pytest.approx(roll) and q[4] == 0.0   # IK khoá yaw 0
        assert np.allclose(K.arm_fk(q)[0], p, atol=1e-9)


def test_ik_out_of_reach():
    assert K.arm_ik([1.0, 0, 0]) is None
    K.arm_ik([0.0, 0.0, spec.D0 - spec.LT])  # sát trục: không được ném lỗi


def test_omni_inverse():
    for v in ([0.3, 0, 0], [0, 0.2, 0], [0, 0, 1.0], [0.1, -0.2, 0.5]):
        w = K.wheel_speeds(*v)
        assert np.allclose(K.base_twist(w), v)
    # quay tại chỗ: ba bánh cùng tốc độ
    w = K.wheel_speeds(0, 0, 1.0)
    assert np.allclose(w, w[0])


def test_tool_down_ik_needs_no_shoulder_above_10_deg():
    """-2: vai mềm tối đa 48,5 (v0.4.3; chặn dưới ở 50): mẫu IK tool-down không dùng vai > 25 độ (trước 10), nên tầm bàn 0,60-0,90 m không mất; số mẫu khả thi y như vai +90."""
    rng = np.random.default_rng(2)
    top = -90.0
    for _ in range(300):
        p = np.array([rng.uniform(0.1, 0.35), rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.15)])
        q = K.arm_ik(p, grip_yaw=rng.uniform(-1, 1))
        if q is not None:
            top = max(top, math.degrees(q[1]))
    assert top < 25.0, top     # kẹp AM-ARM200 ngắn hơn kẹp cũ 31,7 mm nên điểm gần đế cần vai cao hơn (16,6 độ); vẫn xa giới hạn mềm 48,5


def test_leader_map_passes_wrist_yaw_d60():
    """leader AM-ARM200 6+1 -> follower 6+1, yaw 1:1 (đổi dấu theo URDF_TO_SDK, kẹp ±90)."""
    from faibotic.teleop import LeaderMap
    out = LeaderMap("left")({"wrist_yaw": 30.0, "wrist_roll": 10.0})
    assert out["left_wrist_yaw"] == pytest.approx(-math.radians(30.0)) and out["left_wrist_roll"] == pytest.approx(math.radians(10.0))
    assert LeaderMap("left")({"wrist_yaw": 170.0})["left_wrist_yaw"] == pytest.approx(spec.ARM_LIMITS["wrist_yaw"][0])
