"""Trục nâng M1 (QC2 cột nâng vòng 2): giới hạn mô-men, tốc độ, công tắc hành trình, khuỷu theo vai, hình học ray."""
import json, math
from pathlib import Path

import pytest

from faibotic import Robot, limits, spec
from faibotic.backends.feetech import FeetechBackend
from faibotic.feetech import protocol as P
from faibotic.feetech.bus import FeetechBus
from faibotic.feetech.mockbus import FakeServoPort

ROOT = Path(__file__).resolve().parents[1]
D = math.radians
NOSLEEP = dict(sleep=lambda s: None, allow_no_latch=True)
LIFT_OK = dict(allow_lift_underpowered=True)   # bus giả không tải: cho lift chạy dù giới hạn răng < tải thiết kế


@pytest.fixture
def port():
    return FakeServoPort(list(spec.SERVO_IDS.values()))


def make(port, **kw):
    return FeetechBackend(port=port, enable=True, **NOSLEEP, **kw)


def test_lift_torque_limit_is_min_of_70pct_rated_and_half_tooth_strength():
    """R2 7.2 (ii): TORQUE_LIMIT = min(70% định mức, 0,5 x F_đo x r), ‰ của mô-men CHẶN. F_đo là chỗ giữ [giả định, chờ đo DK-N18]:
    test kiểm CÔNG THỨC, không khẳng định răng PETG đạt."""
    stall_nm = 95.0 * spec.KGCM
    r = 0.030 / 2                                                       # bánh 20 răng m1,5 (trước 0,0428 / 2)
    assert spec.LIFT_PINION_R == pytest.approx(r)
    rated70 = round(1000 * 0.70 * 22.5 / 95.0)
    assert rated70 == 166                                               # trần 70% định mức (v0.4 dùng đúng số này)
    for f in (60.0, 120.0, 150.0, 1e6):
        want = round(1000 * min(0.70 * 22.5 * spec.KGCM, 0.5 * f * r) / stall_nm)
        assert spec.lift_torque_limit_permille(f) == want
    assert spec.lift_torque_limit_permille(1e6) == 166                 # răng rất bền: về trần 70%
    assert spec.LIFT_TORQUE_LIMIT_PERMILLE == spec.lift_torque_limit_permille(spec.LIFT_TOOTH_F_MEASURED_N)
    assert spec.LIFT_TORQUE_LIMIT_PERMILLE <= 166 < 200                 # 700 nghĩa là 66,5 kg·cm: phá răng in
    # lực răng tại giới hạn không vượt 50% F_đo (sai số làm tròn ‰ <= 0,5‰ x mô-men chặn / r)
    assert spec.LIFT_FORCE_LIMIT_N <= 0.5 * spec.LIFT_TOOTH_F_MEASURED_N + 0.5e-3 * stall_nm / r


def test_underpowered_lift_refused_by_default_and_override_is_explicit(port, monkeypatch):
    """Hành vi (thay test tự khẳng định cũ): khi giới hạn răng < lực giữ tải thiết kế, backend TỪ CHỐI lệnh lift (lệnh khớp
    khác vẫn chạy); chỉ allow_lift_underpowered=True mới cho chạy (thử không tải có giám sát); số đo răng đủ thì tự cho chạy."""
    from faibotic.backends.feetech import LiftUnderpowered
    assert spec.LIFT_LIMIT_COVERS_DESIGN_LOAD is False          # F_đo giữ chỗ 120 N: 60 N < 85,4 N (tải có trạm 8,28 kg)
    with pytest.warns(RuntimeWarning, match="tải thiết kế"):
        be = make(port)
    cur = be._cmd["lift"]
    with pytest.raises(LiftUnderpowered, match="allow_lift_underpowered"):
        be.write_joints({"lift": cur + 0.01})
    assert be.read_joints()["lift"] == pytest.approx(cur) and be.lift_fault is None
    assert be.write_joints({"left_pan": 0.1, "lift": cur})["left_pan"] == pytest.approx(0.1)   # lift đứng yên: không chặn
    be.close()
    be = make(port, **LIFT_OK)
    assert be.write_joints({"lift": cur + 0.01})["lift"] == pytest.approx(cur + 0.01)
    be.close()
    monkeypatch.setattr(spec, "LIFT_LIMIT_COVERS_DESIGN_LOAD", True)   # như khi đo F_đo >= ~126 N
    be = make(port)
    assert be.write_joints({"lift": cur + 0.02})["lift"] == pytest.approx(cur + 0.02)
    be.close()


def test_lift_rack_steps_per_metre_and_raw_zero():
    """Thanh răng, bánh 20 răng 1 vòng servo = pi x 30 mm; raw 0 ở đáy (không còn dời gốc giữa hành trình như vitme v0.4)."""
    assert spec.LIFT_STEPS_PER_M == round(4096 / (math.pi * 0.030)) == 43460
    assert spec.LIFT_RAW_ZERO_M == 0.0
    assert spec.LIFT_STEPS_PER_M * spec.LIFT_RANGE[1] < 32767                         # cả hành trình vừa 15 bit, từ 0
    assert spec.LIFT_NOTCH_PITCH == pytest.approx(math.pi * 1.5e-3)                     # mô-đun 1,5 [giả định]


def test_feetech_writes_and_reads_back_lift_torque_and_speed(port):
    be = make(port)
    lid = spec.SERVO_IDS["lift"]
    assert be.bus.read(lid, P.TORQUE_LIMIT) == spec.LIFT_TORQUE_LIMIT_PERMILLE
    assert be.read_soft_limits()["lift_torque_limit_permille"] == spec.LIFT_TORQUE_LIMIT_PERMILLE
    assert be.bus.read(lid, P.GOAL_SPEED) == round(spec.LIFT_SPEED_MAX * spec.LIFT_STEPS_PER_M) <= 2000
    be.close()


def test_feetech_enable_refuses_when_torque_limit_not_stored(port):
    be = make(port)
    orig = be.bus._bus.sync_write

    def drop(reg, vals):     # servo bỏ qua ghi TORQUE_LIMIT
        return None if reg == P.TORQUE_LIMIT else orig(reg, vals)
    be.bus._bus.sync_write = drop
    port.mem[spec.SERVO_IDS["lift"]][P.TORQUE_LIMIT[0]:P.TORQUE_LIMIT[0] + 2] = bytes([0xE8, 0x03])     # 1000
    be.enabled = False
    with pytest.raises(RuntimeError, match="TORQUE_LIMIT"):
        be.enable()


def test_lift_calibration_fits_15bit_and_roundtrips(port):
    be = make(port, enable=False) if False else make(port)
    for h in (0.0, 0.25, 0.5):
        raw = be.to_raw("lift", h)
        # v0.5: 1 bước = 1/30.463 m (~33 µm), không chia hết như 128.000 bước/m của vitme: sai số quy tròn <= nửa bước
        assert 0 <= raw <= 32767 and be.from_raw("lift", raw) == pytest.approx(h, abs=0.51 / spec.LIFT_STEPS_PER_M)
    be.close()


def test_slew_speed_never_exceeds_spec():
    cur, vmax = 0.0, spec.LIFT_SPEED_MAX
    for _ in range(100):
        nxt = limits.lift_slew(cur, 0.5, 0.02)
        assert nxt - cur <= vmax * 0.02 + 1e-12
        cur = nxt
    assert cur == pytest.approx(vmax * 2.0)


def test_robot_feetech_lift_speed_clamped(port):
    r = Robot("feetech", port=port, enable=True, **NOSLEEP, **LIFT_OK)
    assert r.lift_vmax == spec.LIFT_SPEED_MAX
    h0 = r._targets["lift"]
    r.set_lift(0.45)
    for _ in range(50):
        r.step()
    assert 0 < r._targets["lift"] - h0 <= spec.LIFT_SPEED_MAX * (50 + 1) * r.dt + 1e-9
    r.close()


def test_sim_realistic_lift_speed_and_default_fast():
    fast = Robot("sim")
    assert fast.lift_vmax is None
    fast.set_lift(0.3)
    fast.wait(0.5)
    assert fast.get_state()["lift"] > 0.2          # sim nhanh mặc định cho demo/test, không phải phần cứng
    slow = Robot("sim", lift_speed_max=spec.LIFT_SPEED_MAX, realistic_lift=True)
    slow.set_lift(0.3)
    slow.wait(2.0)
    assert slow.get_state()["lift"] <= spec.LIFT_SPEED_MAX * 2.0 + 0.002
    fast.close(); slow.close()


@pytest.mark.parametrize("top", [True, False])
def test_feetech_limit_switch_stops_motion(port, top):
    sw = limits.FakeLiftSwitches()
    be = make(port, lift_switches=sw, **LIFT_OK)
    be.write_joints({"lift": 0.25})
    if top:
        sw.top = True
        sent = be.write_joints({"lift": 0.4})
        assert sent["lift"] == pytest.approx(0.25)         # chạm công tắc trên: không đi lên
        assert be.write_joints({"lift": 0.2})["lift"] == pytest.approx(0.2)   # chiều ngược lại được
    else:
        sw.bottom = True
        assert be.write_joints({"lift": 0.1})["lift"] == pytest.approx(0.25)
        assert be.write_joints({"lift": 0.3})["lift"] == pytest.approx(0.3)
    be.close()


def test_sim_limit_switch_gate():
    r = Robot("sim")
    r.backend.lift_switches = limits.FakeLiftSwitches(top=True)
    r.set_lift(0.3)
    r.wait(0.5)
    assert r.get_state()["lift"] < 0.01
    r.close()


def test_elbow_max_and_pan_limits():
    assert spec.PAN_SOFT_MAX_DEG == 70.0
    assert spec.ARM_LIMITS["pan"][1] == pytest.approx(D(70))
    assert spec.elbow_max(D(-80)) == pytest.approx(D(spec.ELBOW_SOFT_MAX_DEG)) == spec.elbow_max(D(-32))
    assert spec.elbow_max(D(0)) == pytest.approx(D(111)) and spec.elbow_max(D(48.5)) == pytest.approx(D(62))     # quét mesh, làm lõm
    slopes = [(b - a) / (y - x) for (x, a), (y, b) in zip(zip(spec.ELBOW_COUPLE_SHOULDER_DEG, spec.ELBOW_COUPLE_MAX_DEG),
                                                        list(zip(spec.ELBOW_COUPLE_SHOULDER_DEG, spec.ELBOW_COUPLE_MAX_DEG))[1:])]
    assert all(s2 <= s1 + 1e-9 for s1, s2 in zip(slopes, slopes[1:]))     # lõm: vùng cho phép lồi, nội suy tuyến tính an toàn
    cur = {"left_shoulder": 0.0, "left_elbow": D(100), "left_wrist_flex": 0.0}
    out = limits.clamp({"left_shoulder": D(48.5), "left_elbow": D(127), "left_pan": D(100)}, cur)
    assert out["left_elbow"] == pytest.approx(D(62)) and out["left_pan"] == pytest.approx(D(70))
    assert limits.clamp({"left_elbow": D(120)}, cur)["left_elbow"] == pytest.approx(D(111))      # vai 0 khuỷu tới 111
    assert limits.clamp({"left_elbow": D(120)}, {**cur, "left_shoulder": D(-40)})["left_elbow"] == pytest.approx(D(120))   # vai thấp: khuỷu cao được
    assert limits.clamp({"left_shoulder": D(40)}, cur)["left_elbow"] == pytest.approx(D(79))     # lệnh vai: hạ khuỷu theo
    assert limits.violations({"left_shoulder": D(40), "left_elbow": D(100), "left_wrist_flex": 0.0})


def _json(name):
    p = ROOT / "cad/export" / name
    if not p.exists():
        pytest.skip(f"{name} chưa tạo (chạy cad/faibotic/lift.py hoặc lift_scan.py)")
    return json.loads(p.read_text())


def test_rail_geometry_blocks_inside_rail_and_spec_matches():
    ray = _json("M1-LF-build.json")["ray"]
    assert ray["ray_dai_mm"] == pytest.approx(spec.LIFT_RAIL_LENGTH * 1000) and ray["dat"] is True
    for tag, m in ray["bien_khoi"].items():
        if "lift500" in tag:
            assert m["bien_tren_mm"] >= 10, tag
        else:
            assert m["bien_duoi_mm"] >= 10, tag
    assert ray["qua_dem_duoi_mm"] / 1000 == pytest.approx(spec.LIFT_HARD_STOP_MARGIN[0], abs=5e-4)
    assert ray["qua_dem_tren_mm"] / 1000 == pytest.approx(spec.LIFT_HARD_STOP_MARGIN[1], abs=5e-4)


def test_arm_chain_scan_with_spec_limits_has_zero_hits():
    s = _json("M1-LF-scan-verify.json")
    assert s["so_cham"] == 0 and s["so_tu_the"] > 10000
    assert s["pan"] == [-70, 70]
