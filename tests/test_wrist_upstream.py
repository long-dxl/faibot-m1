"""P-L (S1C) + (6+1): chuỗi khớp SDK (spec.py + kinematics.py) so với URDF upstream AM-ARM200, tay trái, gồm khớp yaw.

Mốc độc lập với spec.py/kinematics.py: FK URDF viết lại ở đây (cad/upstream/AM-ARM/am-arm200/urdf/am-arm200.urdf). Góc SDK đổi sang góc URDF bằng
spec.URDF_TO_SDK (q_sdk = dấu x q_urdf + lệch). kẹp gốc AM-ARM200 (thân J7 cố định theo link left_Fixed_Jaw); tâm kẹp lệch trục cuộn
(LT - LW, TCP_DY, TCP_DZ) trong khung tay, số đo CAD ở cad/export/AM_gripper_PL.json (gripper_am.py --scan). S1C: kẹp song song cũ (lưu trữ nội bộ)."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from faibotic import kinematics as K
from faibotic import spec

ROOT = Path(__file__).resolve().parents[1]
UP = ROOT / "cad" / "upstream" / "AM-ARM" / "am-arm200"
pytestmark = pytest.mark.skipif(not (UP / "urdf" / "am-arm200.urdf").exists(), reason="thiếu URDF upstream")
R_SDK = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])     # x_sdk = -y_urdf, y_sdk = x_urdf
URDF_NAME = {"pan": "left_shoulder_pan", "shoulder": "left_shoulder_lift", "elbow": "left_elbow_flex", "wrist_flex": "left_wrist_flex",
             "wrist_yaw": "left_wrist_yaw_joint", "wrist_roll": "left_wrist_roll"}


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _axang(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * k + (1 - np.cos(t)) * k @ k


def urdf_fk(q: dict):
    """Khung từng link (R, p mét) của tay trái URDF upstream; khớp revolute quay q[tên] quanh trục của nó."""
    root = ET.parse(UP / "urdf" / "am-arm200.urdf").getroot()
    T = {"left_Base": (np.eye(3), np.zeros(3))}
    axis = {}
    for j in root.findall("joint"):
        o, a = j.find("origin"), j.find("axis")
        R, p = T[j.find("parent").get("link")]
        Rj = R @ _rpy(*map(float, o.get("rpy").split()))
        pj = p + R @ np.array(list(map(float, o.get("xyz").split())))
        if j.get("type") == "revolute":
            ax = np.array(list(map(float, a.get("xyz").split())))
            axis[j.get("name")] = Rj @ ax
            Rj = Rj @ _axang(ax, q.get(j.get("name"), 0.0))
        T[j.find("child").get("link")] = (Rj, pj)
    return T, axis


def sdk_to_urdf(q):
    return {URDF_NAME[j]: (v - spec.URDF_TO_SDK[j][1]) / spec.URDF_TO_SDK[j][0] for j, v in zip(spec.ARM_JOINTS, q)}


def urdf_tcp(q):
    """Tâm kẹp = điểm gắn cứng link left_Fixed_Jaw; lệch (LT - LW, TCP_DY, TCP_DZ) khung tay SDK ở q_sdk = 0 đổi sang khung link đó."""
    T, axis = urdf_fk(sdk_to_urdf(q))
    R, p = T["left_Fixed_Jaw"]
    R0 = urdf_fk(sdk_to_urdf([0.0] * len(spec.ARM_JOINTS)))[0]["left_Fixed_Jaw"][0]
    off = R0.T @ R_SDK.T @ np.array([spec.LT - spec.LW, spec.TCP_DY, spec.TCP_DZ])
    return R_SDK @ (p + R @ off)


# q = [pan, vai, khuỷu, gập, yaw, cuộn]
POSES = [[0, 0, 0, 0, 0, 0], [0.3, -0.5, 1.2, 0.6, 0, 0.4], [-0.6, -1.2, 1.6, 1.4, 0, -1.0], [0.1, 0.4, 0.3, -0.9, 0, 2.0], [0, -1.39, 1.62, 1.52, 0, 0],
         [0, 0, 0, 0, 0.7, 0], [0.3, -0.5, 1.2, 0.6, -1.2, 0.4], [-0.6, -1.2, 1.6, 1.4, 1.5, -1.0], [0.1, 0.4, 0.3, -0.9, -0.5, 2.0]]


@pytest.mark.parametrize("q", POSES)
def test_tcp_matches_urdf_chain(q):
    """TCP SDK (giải tích) trùng tâm kẹp gắn link left_Fixed_Jaw theo URDF upstream, <= 0,5 mm, ở nhiều tư thế (kiểm cả dấu/lệch đổi góc, cuộn)."""
    p_sdk, _ = K.arm_fk(np.array(q, float), "left")
    p_ref = urdf_tcp(q)
    assert np.linalg.norm(p_sdk - p_ref) <= 5e-4, (q, p_sdk, p_ref)


def test_urdf_zero_maps_to_folded_rest():
    """Tư thế 0 URDF (tay gập ra sau, tư thế nghỉ của AM-ARM200) = vai -162,25, khuỷu +162,25 độ trong quy ước SDK."""
    assert spec.URDF_TO_SDK["shoulder"][1] == pytest.approx(-np.radians(162.2524), abs=1e-6)
    assert spec.URDF_TO_SDK["elbow"][1] == pytest.approx(np.radians(162.2524), abs=1e-6)


def test_flex_axis_and_roll_axis_from_urdf():
    T, _ = urdf_fk({})
    el = T["left_Lower_Arm"][1]
    fl = T["left_Wrist_Pitch_Roll"][1]
    roll = T["left_Fixed_Jaw"][1]
    assert np.hypot(fl[1] - el[1], fl[2] - el[2]) == pytest.approx(np.hypot(spec.L2, spec.WF_DZ), abs=5e-4)
    assert roll[2] - fl[2] == pytest.approx(spec.ROLL_DZ, abs=5e-4)
    assert roll[0] == pytest.approx(spec.ROLL_DY, abs=5e-4)
    sh = T["left_Upper_Arm"][1]
    assert sh[2] == pytest.approx(spec.D0, abs=5e-4) and -sh[1] == pytest.approx(spec.SHOULDER_DR, abs=5e-4)
    assert np.hypot(el[1] - sh[1], el[2] - sh[2]) == pytest.approx(spec.L1, abs=5e-4)


def test_spec_matches_derivation_tool():
    """Số trong spec.py trùng số suy từ URDF bằng sim/tools/am_arm200_pl.py (kích thước và khối lượng/trọng tâm phần in)."""
    import sys
    sys.path.insert(0, str(ROOT / "sim" / "tools"))
    import am_arm200_pl as A
    d = A.derive()
    g = d["geometry"]
    for k in ("D0", "SHOULDER_DR", "L1", "L2", "WF_DZ", "LW", "ROLL_DY", "ROLL_DZ", "UPPER_URDF_DEG", "YAW_DX", "YAW_DZ"):
        assert getattr(spec, k) == pytest.approx(g[k], abs=1e-4), k
    assert g["YAW_DY"] == pytest.approx(spec.ROLL_DY, abs=1e-4)     # khớp yaw cùng lệch ngang với trục cuộn
    assert d["check_err_m"] < 1e-6
    for b, v in d["bodies"].items():
        m, c, _ = spec.LINK_PRINT[b]
        assert m == pytest.approx(v["mass"], abs=1e-4) and np.allclose(c, v["com"], atol=1e-4), b


def test_gripper_matches_cad_scan():
    """số kẹp trong spec trùng quét CAD kẹp AM-ARM200 (cad/export/AM_gripper_PL.json)."""
    g = json.loads((ROOT / "cad" / "export" / "AM_gripper_PL.json").read_text())
    c = np.array(g["tam_kep_lui_tip"]) / 1000
    assert spec.LT - spec.LW == pytest.approx(c[0], abs=2e-4)
    assert spec.TCP_DY == pytest.approx(c[1], abs=2e-4) and spec.TCP_DZ == pytest.approx(c[2], abs=2e-4)
    assert spec.TIP_LEN * 1000 == pytest.approx(g["tip_len_mm"])
    assert spec.GRIPPER_CLOSE_URDF == pytest.approx(np.radians(g["goc_khep_urdf_deg"]), abs=1e-4)
    assert spec.GRIPPER_OPEN_RAD == pytest.approx(np.radians(g["mo_mem_deg"]), abs=1e-6)
    assert spec.GRIPPER_MAX_GAP == pytest.approx(g["khe_dau_ngon_mo_mem_mm"] / 1000, abs=1e-3)
    assert np.allclose(spec.JAW_PIVOT, np.array(g["truc_kep_goc"]) / 1000, atol=2e-4)
    assert g["chieu_khep_urdf"] == 1.0 and abs(g["truc_kep_huong"][2]) > 0.999     # khép = quay +z khung tay; model.py: mở quanh -z (tay trái)


def test_right_arm_mirrors_left():
    for deg in (0.0, 45.0, 90.0):
        q = [0.0, 0.0, 0.0, np.radians(deg), 0.2, 0.3]
        a, b = K.fingertip_fk(q, "left"), K.fingertip_fk([0.0, 0.0, 0.0, np.radians(deg), -0.2, -0.3], "right")
        assert np.allclose(a * [1, -1, 1], b, atol=1e-9)


def test_yaw_axis_from_urdf():
    """trục yaw URDF (left_wrist_yaw_joint) ở tư thế SDK 0 là -z khung SDK (URDF_TO_SDK dấu -1 -> SDK quay quanh +z), gốc khớp tại
    trục gập + (YAW_DX, ROLL_DY, YAW_DZ)."""
    q0 = sdk_to_urdf([0.0] * len(spec.ARM_JOINTS))
    T, axis = urdf_fk(q0)
    assert np.allclose(R_SDK @ axis["left_wrist_yaw_joint"], [0, 0, -1], atol=1e-4)
    d = R_SDK @ (T["left_wrist_yaw"][1] - T["left_Wrist_Pitch_Roll"][1])
    assert np.allclose(d, [spec.YAW_DX, spec.ROLL_DY, spec.YAW_DZ], atol=5e-4)
