import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest

from faibotic import Robot, spec
from faibotic import model as M

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def mdl():
    return mujoco.MjModel.from_xml_string(M.robot_mjcf(scene=True))


def test_sim_files_up_to_date():
    assert (ROOT / "sim/m1_scene.xml").read_text() == M.robot_mjcf(True, meshdir="assets")
    assert (ROOT / "sim/m1.xml").read_text() == M.robot_mjcf(False, meshdir="assets")
    assert (ROOT / "sim/m1.urdf").read_text() == M.robot_urdf()


def test_joint_ranges_match_spec(mdl):
    for s in spec.SIDES:
        for j in spec.ARM_JOINTS:
            lo, hi = mdl.joint(f"{s}_{j}").range
            assert (lo, hi) == pytest.approx(spec.ARM_LIMITS[j], rel=1e-5)
    assert tuple(mdl.joint("lift").range) == pytest.approx(spec.LIFT_RANGE)


def test_actuator_torque_limited_by_datasheet(mdl):
    """S1C: forcerange = TORQUE_LIMIT nhóm (spec.TORQUE_LIMIT_PCT x mô-men chặn), chặt hơn mô-men chặn (trước: = chặn x số servo)."""
    for s in spec.SIDES:
        for j in spec.ARM_JOINTS:
            lo, hi = mdl.actuator(f"{s}_{j}").forcerange
            want = spec.ARM_SERVO[j].stall_nm * spec.TORQUE_LIMIT_PCT[j] / 100
            assert hi == pytest.approx(want, rel=1e-5) and lo == pytest.approx(-want, rel=1e-5)
            assert hi <= spec.ARM_SERVO[j].stall_nm


def test_dimensions_v0(mdl):
    d = mujoco.MjData(mdl)
    mujoco.mj_forward(mdl, d)
    # S1G: kích thước theo CAD P-L (khớp URDF AlohaMini2, hành trình thanh răng -160..+290 mm): trục vai 0,562..1,012 m (trước: 0,55..1,05)
    assert spec.SHOULDER_Z0 == pytest.approx(0.5619, abs=1e-4)
    assert spec.SHOULDER_Z0 + spec.LIFT_RANGE[1] == pytest.approx(1.0119, abs=1e-4)
    assert spec.WHEEL_RADIUS * 2 == pytest.approx(0.1016)      # omni 4 in (CAD P-L)
    # khối lượng robot (trừ bàn, hộp). P-L: đối trọng thép ~8,2 kg chủ ý (chống lật >= 1,5, DK-N40), nên mức v0 <= 15 kg không còn áp;
    # kiểm tổng = các phần trong spec (đế + đối trọng + khay + nan + cột + 3 bánh + xe nâng + trạm + 2 tay) và <= 22 kg
    robot_mass = sum(mdl.body_mass[i] for i in range(mdl.nbody)
                     if mdl.body(i).name not in ("world", "table", "box"))
    arms = 2 * (spec.ARM_MASS_MOVING + spec.LINK_MASS["mount"])
    want = (spec.BASE_MASS + spec.COUNTERWEIGHT_MASS + spec.TRAY_MASS + 3 * spec.SPOKE_MASS + spec.COLUMN_MASS + 3 * 0.20 + spec.CARRIAGE_MASS
            + spec.STATION_MASS + arms)   # S2: + trạm hai tay chưa tay
    assert robot_mass == pytest.approx(want, abs=0.03) and robot_mass <= 22.0   # 0,02 kg: logo + bánh răng kẹp


def test_urdf_matches_spec():
    root = ET.fromstring((ROOT / "sim/m1.urdf").read_text())
    joints = {j.get("name"): j for j in root.findall("joint")}
    for s in spec.SIDES:
        for j in spec.ARM_JOINTS:
            lim = joints[f"{s}_{j}"].find("limit")
            assert float(lim.get("lower")) == pytest.approx(spec.ARM_LIMITS[j][0], abs=1e-5)
            assert float(lim.get("upper")) == pytest.approx(spec.ARM_LIMITS[j][1], abs=1e-5)
    assert joints["lift"].get("type") == "prismatic"
    m = mujoco.MjModel.from_xml_path(str(ROOT / "sim/m1.urdf"))
    assert m.njnt == 18   # 2 x (6 khớp + khớp hàm kẹp) + lift + 3 bánh (kẹp AM-ARM200, không còn 2 ngón trượt)


def test_robot_clamps_to_limits():
    r = Robot("sim")
    out = r.set_joint_positions({"left_shoulder": math.radians(-40), "left_elbow": 10.0, "left_wrist_flex": 0.0, "lift": -1.0, "right_gripper": 500})  # vai -40 để khỏi dính giới hạn ghép (vai 0 nay khuỷu <= 111)
    assert out["left_elbow"] == pytest.approx(spec.ARM_LIMITS["elbow"][1])
    assert out["lift"] == 0.0 and out["right_gripper"] == 100.0
    r.close()


def test_sim_stable_no_nan_at_rest():
    r = Robot("sim")
    r.wait(2.0)
    d = r.backend.data
    assert np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))
    assert np.abs(d.qvel).max() < 0.05
    r.close()


def test_wrist_yaw_limits_from_mesh_scan_d61():
    """(DK-N66): tầm yaw SDK lấy từ quét mesh (cad/export/AM_wrist_yaw_scan.json): vỏ J56 chạm giá J5 ở yaw URDF +84,6 / -83,7 độ,
    biên 3 độ; giới hạn ghép theo gập (AM_wrist_yaw_couple.json) hẹp dần khi gập tăng, không rộng hơn ARM_LIMITS, và chứa yaw 0."""
    import json
    from faibotic import limits
    scan = json.loads((ROOT / "cad" / "export" / "AM_wrist_yaw_scan.json").read_text())
    lo, hi = spec.ARM_LIMITS["wrist_yaw"]
    assert [round(math.degrees(lo), 3), round(math.degrees(hi), 3)] == scan["gioi_han_sdk_deg"]
    assert math.degrees(hi) <= -scan["cham_yaw"]["am"]["urdf_deg"] - 3 + 1e-6
    assert math.degrees(lo) >= -scan["cham_yaw"]["duong"]["urdf_deg"] + 3 - 1e-6
    couple = json.loads((ROOT / "cad" / "export" / "AM_wrist_yaw_couple.json").read_text())["cham"]
    for w, a, b in zip(spec.WRIST_YAW_COUPLE_FLEX_DEG, spec.WRIST_YAW_COUPLE_MIN_DEG, spec.WRIST_YAW_COUPLE_MAX_DEG):
        assert a <= 0 <= b
        c = couple.get(f"{w:g}")
        if c and c["am_urdf"]["cham_deg"] is not None:
            assert b <= max(0.0, -c["am_urdf"]["cham_deg"] - 3) + 1e-6      # nút sát chạm gập (DK-N68): khoá 0
        if c and c["duong_urdf"]["cham_deg"] is not None:
            assert a >= -c["duong_urdf"]["cham_deg"] + 3 - 1e-6
    assert spec.wrist_yaw_range(0.0) == pytest.approx((lo, hi))
    y60 = spec.wrist_yaw_range(math.radians(60))
    assert y60[1] < hi
    # clamp: lệnh yaw vượt tầm ghép bị kẹp; lệnh gập làm yaw đang có vượt thì yaw lùi về
    out = limits.clamp({"left_wrist_yaw": math.radians(80)}, {"left_wrist_flex": math.radians(60), "left_wrist_yaw": 0.0})
    assert out["left_wrist_yaw"] == pytest.approx(y60[1])
    out = limits.clamp({"left_wrist_flex": math.radians(60)}, {"left_wrist_flex": 0.0, "left_wrist_yaw": math.radians(80)})
    assert out["left_wrist_yaw"] == pytest.approx(y60[1])
    assert limits.violations({"left_wrist_flex": math.radians(60), "left_wrist_yaw": math.radians(80)})
    assert not limits.violations({"left_wrist_flex": 0.0, "left_wrist_yaw": math.radians(80)})
