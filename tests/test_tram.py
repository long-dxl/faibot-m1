"""S2: trạm hai tay tháo lắp (G1b): khoảng vai, hai cấu hình robot, bus, metadata dataset, vùng cấm trạm, khoá trạm DOCK."""
import math

import numpy as np
import pytest

from faibotic import limits, spec
from faibotic.backends.feetech import DockOpen, FeetechBackend
from faibotic.feetech.mockbus import FakeServoPort
from faibotic.safety import DockSwitches, FakeGPIO, dock_state

NOSLEEP = dict(sleep=lambda s: None, allow_no_latch=True, allow_lift_underpowered=True)


def test_shoulder_spacing_d44():
    assert 2 * spec.SHOULDER_Y == pytest.approx(0.475)


def test_lift_on_bus_b_and_bus_map():
    assert spec.SERVO_BUS["lift"] == "BUS_B"                                     # lift rời BUS_L (BUS_L nay trên trạm)
    assert set(spec.SERVO_BUS) == set(spec.SERVO_IDS)
    assert {spec.SERVO_BUS[f"left_{j}"] for j in ("pan", "gripper")} == {"BUS_L"}
    assert {spec.SERVO_BUS[f"right_{j}"] for j in ("pan", "gripper")} == {"BUS_R"}
    assert sorted(spec.BUS_B_IDS) == [13, 14, 15, 16]


def test_robot_configs():
    tb, dd = spec.ROBOT_CONFIGS["tram_ban"], spec.ROBOT_CONFIGS["di_dong"]
    assert not tb["base"] and not tb["lift"] and "BUS_B" not in tb["buses"] and "lift" not in tb["servos"] and len(tb["servos"]) == 14
    assert dd["base"] and dd["lift"] and len(dd["servos"]) == 18   # 6+1, 18 servo and dd["dock_required"]
    assert tb["current_budget_a"] <= 15.0 * 0.8 + 1e-9                           # adapter 12 V 15 A
    assert dd["current_budget_a"] == spec.CURRENT_BUDGET_A


def test_detect_config():
    all_ids = set(spec.SERVO_IDS.values())
    assert spec.detect_config(all_ids) == "di_dong"
    assert spec.detect_config(all_ids - set(spec.BUS_B_IDS)) == "tram_ban"
    with pytest.raises(ValueError, match="BUS_B"):
        spec.detect_config(all_ids - {16})                                       # thiếu lift: lỗi, không đoán


def test_dataset_meta():
    m = spec.dataset_meta("tram_ban")
    assert m["che_do"] == "tram_ban" and m["robot_type"] == "faibot_m1_lite"
    assert m["khoang_vai_m"] == pytest.approx(0.475)
    assert m["camera_dau"]["chui_deg"] == 50.0 and m["camera_dau"]["cao_tren_ban_m"] == pytest.approx(0.65, abs=1e-3)
    assert spec.dataset_meta("di_dong", head_tilt_deg=35)["camera_dau"]["chui_deg"] == 35
    with pytest.raises(ValueError):
        spec.dataset_meta("ban")


def test_head_camera_d48():
    assert spec.HEAD_CAM_TILT_DEG == 50.0
    # trục nhìn camera sim (-z camera) chúi 50° xuống, nhìn ra trước
    x, y = np.array(spec.HEAD_CAM_XYAXES[:3]), np.array(spec.HEAD_CAM_XYAXES[3:])
    look = -np.cross(x, y)
    assert math.degrees(math.asin(-look[2])) == pytest.approx(50.0, abs=0.1) and look[0] > 0
    assert spec.H_J1_ABOVE_TABLE + spec.HEAD_CAM_AXIS[2] == pytest.approx(0.65, abs=1e-3)


D = math.radians


def _tcp(q):
    return limits.arm_points_carriage("left", [D(v) for v in q])["tâm kẹp"]


def test_keepout_head_and_deck():
    home = [spec.HOME_ARM[j] for j in spec.ARM_JOINTS]
    for s in spec.SIDES:
        assert limits.keepout_violations(s, home) == []                         # tư thế gốc hợp lệ
    pts = limits.arm_points_carriage("left", [0, 0, 0, 0, 0, 0])
    assert pts["tâm kẹp"][0] > pts["khuỷu"][0] > spec.SHOULDER_X
    # tay giơ cao gập vào giữa (pan -70 vào trong): cấm (cầu đầu camera / trụ cột camera)
    hit = [limits.keepout_violations("left", [D(-70), D(-80), D(e), D(w), 0, 0]) for e in range(-10, 60, 5) for w in range(-75, 80, 10)]
    assert any(any("camera" in v for v in h) for h in hit)
    # tư thế cố định (số FK độc lập: chuỗi URDF upstream AM-ARM200 + tâm kẹp, tests/test_wrist_upstream.urdf_tcp, tính lại 24/11 cho 6+1 
    # tâm kẹp x -25 mm sau trục pan, z -87 mm dưới đáy J1; số cũ -54/-98 là của kẹp cũ dài hơn 31,7 mm): vi phạm luật bệ
    q = [0, 0, 100, 60, 0, 0]
    t = _tcp(q)
    assert t[0] - spec.SHOULDER_X == pytest.approx(-0.025, abs=0.002) and t[2] == pytest.approx(-0.087, abs=0.002)
    assert any("bệ" in v for v in limits.keepout_violations("left", [D(v) for v in q]))


def test_keepout_mast_and_tray():
    """Luật thêm sau quét mesh 24/11: trụ quanh cột camera; khay ở chế độ di động (theo lift)."""
    # pan -70, vai -80, khuỷu -10, gập 87 (cấu hình chạm cột camera -10 mm trong quét mesh): bị cấm
    assert any("cột camera" in v or "đầu camera" in v for v in limits.keepout_violations("left", [D(-70), D(-80), D(-10), D(87), 0, 0]))
    # tâm kẹp trên khay: q cố định, số FK độc lập (URDF upstream, 6+1 yaw 0; x 0,293; y 0,110; z -0,208 dưới đáy J1 -> z robot 0,206 ở lift 0, 0,656 ở lift 0,45)
    q = [-30, 0, 60, 30, 0, 0]
    t = _tcp(q)
    assert (t[0], t[1], t[2]) == pytest.approx((0.293, 0.110, -0.208), abs=0.002)
    qq = [D(v) for v in q]
    assert any("khay" in v for v in limits.keepout_violations("left", qq, "di_dong", 0.0))        # 0,206 < mặt khay 0,218 + 0,03
    assert not any("khay" in v for v in limits.keepout_violations("left", qq, "di_dong", 0.45))
    assert not any("khay" in v for v in limits.keepout_violations("left", qq, "tram_ban"))


def test_keepout_table_only_in_bench():
    # mặt bàn ở chế độ bàn: z = -0,12 m dưới đáy J1 (đế bàn, CAD), biên 1 cm -> tâm kẹp phải >= -0,11 m
    assert spec.H_J1_ABOVE_TABLE == pytest.approx(0.12) and spec.KEEPOUT_TABLE_MARGIN == pytest.approx(0.01)
    low, high = [0, 0, 30, 60, 0, 0], [0, -20, 40, 70, 0, 0]   # q 6 khớp (yaw 0)
    assert _tcp(low)[2] == pytest.approx(-0.137, abs=0.002) and _tcp(high)[2] == pytest.approx(-0.059, abs=0.002)   # FK độc lập URDF (6+1, 24/11)
    assert any("bàn" in v for v in limits.keepout_violations("left", [D(v) for v in low], "tram_ban"))
    assert not any("bàn" in v for v in limits.keepout_violations("left", [D(v) for v in high], "tram_ban"))
    assert not any("bàn" in v for v in limits.keepout_violations("left", [D(v) for v in low], "di_dong"))


# ---------------- khoá trạm (DOCK_L/DOCK_R) trên backend feetech ----------------
@pytest.fixture
def port():
    return FakeServoPort(list(spec.SERVO_IDS.values()))


def make(port, g, **kw):
    with pytest.warns(RuntimeWarning):
        return FeetechBackend(port=port, dock=DockSwitches(g), **NOSLEEP, **kw)


def test_dock_state_and_none():
    g = FakeGPIO(DOCK_L=True, DOCK_R=False)
    assert dock_state(DockSwitches(g)) == {"DOCK_L": True, "DOCK_R": False, "ok": False}
    assert dock_state(None) == {"ok": None}


def test_enable_refused_until_both_latches_closed(port):
    g = FakeGPIO(DOCK_L=True, DOCK_R=False)
    be = make(port, g)
    assert be.config == "di_dong"
    with pytest.raises(DockOpen, match="DOCK_R"):
        be.enable()
    assert not be.enabled
    g.levels["DOCK_R"] = True
    with pytest.warns(RuntimeWarning):
        be.enable()
    assert be.enabled and be.dock_state()["ok"]
    be.close()


def test_dock_drop_while_running_stops_and_needs_clear(port):
    g = FakeGPIO(DOCK_L=True, DOCK_R=True)
    be = make(port, g)
    with pytest.warns(RuntimeWarning):
        be.enable()
    cur = be._cmd["lift"]
    be.write_joints({"lift": cur + 0.01})                                        # đủ khoá: lift chạy
    g.levels["DOCK_L"] = False
    with pytest.raises(DockOpen, match="lift"):
        be.write_joints({"lift": cur + 0.02})                                    # lệnh lift bị từ chối
    with pytest.raises(DockOpen, match="đế"):
        be.set_base_velocity(0.1, 0, 0)
    be.write_joints({"left_pan": 0.05})                                          # tay vẫn nhận lệnh (vòng E-stop phần cứng đã cắt K1 trên robot thật)
    with pytest.raises(DockOpen, match="DOCK_L"):
        be.step(0.02)
    assert not be.enabled and be.dock_fault
    g.levels["DOCK_L"] = True
    with pytest.raises(DockOpen, match="clear_fault"):
        be.enable()
    be.clear_fault()
    with pytest.warns(RuntimeWarning):
        be.enable()
    assert be.enabled
    be.close()


def test_auto_config_bench_without_bus_b_is_refused():
    p = FakeServoPort([i for i in spec.SERVO_IDS.values() if i not in spec.BUS_B_IDS])
    with pytest.raises(ConnectionError, match="tram_ban"):
        FeetechBackend(port=p, **NOSLEEP)
    p = FakeServoPort([i for i in spec.SERVO_IDS.values() if i != 16])
    with pytest.raises(ConnectionError, match="BUS_B"):
        FeetechBackend(port=p, **NOSLEEP)


# ---------------- QC2 S2: fail-closed khi không khai khoá, lỗi đọc GPIO, lift_unlock ----------------
class _RunLatch:
    """Chốt E-stop giả đang ở "run" (để kiểm riêng khoá trạm, không cần allow_no_latch)."""
    on_stop = None
    monitor_alive, ok, monitor_error = True, True, None

    def start_monitor(self, period):
        pass

    def stop_monitor(self, timeout=1.0):
        pass

    def poll(self):
        return True


def test_no_dock_pins_refuses_di_dong(port):
    """Không khai DOCK_L/DOCK_R, không cờ: cấu hình di_dong (dock_required) từ chối enable, lift, đế."""
    assert spec.ROBOT_CONFIGS["di_dong"]["dock_required"]
    be = FeetechBackend(port=port, sleep=lambda s: None, latch=_RunLatch(), allow_lift_underpowered=True)
    assert be.dock is None and be.dock_state() == {"ok": None}
    with pytest.raises(DockOpen, match="chưa khai chân DOCK"):
        be.enable()
    assert not be.enabled
    with pytest.raises(DockOpen):
        be.set_base_velocity(0.1, 0, 0)
    with pytest.raises(DockOpen):
        be.write_joints({"lift": be._cmd["lift"] + 0.01})
    be.close()


def test_no_dock_with_explicit_flag_warns_and_runs(port):
    with pytest.warns(RuntimeWarning, match="allow_no_dock"):
        be = FeetechBackend(port=port, sleep=lambda s: None, latch=_RunLatch(), allow_lift_underpowered=True, allow_no_dock=True)
    with pytest.warns(RuntimeWarning):
        be.enable()
    assert be.enabled
    be.close()


class _BrokenGPIO(FakeGPIO):
    def read(self, pin):
        if self.levels.get("_hong"):
            raise OSError("đọc GPIO lỗi")
        return super().read(pin)


def test_dock_read_error_while_running_cuts_torque(port):
    g = _BrokenGPIO(DOCK_L=True, DOCK_R=True)
    be = make(port, g)
    with pytest.warns(RuntimeWarning):
        be.enable()
    g.levels["_hong"] = True
    with pytest.raises(DockOpen, match="không đọc được"):
        be.step(0.02)
    assert not be.enabled and be.dock_fault
    with pytest.raises(DockOpen):
        be.enable()
    be.close()


def test_lift_unlock_checks_dock(port):
    from faibotic.safety import LiftLatch, SimLiftLatchHW
    g = FakeGPIO(DOCK_L=True, DOCK_R=False)
    hw = SimLiftLatchHW()
    lt = LiftLatch(hw, sleep=lambda s: None)
    with pytest.warns(RuntimeWarning):
        be = FeetechBackend(port=port, dock=DockSwitches(g), lift_latch=lt, **NOSLEEP)
    with pytest.raises(DockOpen, match="lift_unlock"):
        be.lift_unlock()
    assert not hw.coil and lt.state == "engaged"                                  # chốt không rút
    be.close()
