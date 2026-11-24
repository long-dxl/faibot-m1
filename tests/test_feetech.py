import math

import numpy as np
import pytest

from faibotic import Robot, spec
from faibotic.backends.feetech import FeetechBackend
from faibotic.feetech import protocol as P
from faibotic.feetech.bus import FeetechBus
from faibotic.feetech.mockbus import FakeServoPort

ALL_IDS = list(spec.SERVO_IDS.values())


# ---- test vector theo ví dụ trong tài liệu giao thức Feetech ----
def test_vector_read_position():
    # đọc 2 byte từ địa chỉ 0x38 (vị trí hiện tại) của servo ID 1
    assert P.read_packet(1, P.PRESENT_POSITION) == bytes.fromhex("FF FF 01 04 02 38 02 BE")


def test_vector_write_goal_position_time_speed():
    # ghi vị trí 2048, thời gian 0, tốc độ 1000 vào địa chỉ 0x2A của ID 1
    pkt = P.write_packet(1, 0x2A, P.u16(2048) + P.u16(0) + P.u16(1000))
    assert pkt == bytes.fromhex("FF FF 01 09 03 2A 00 08 00 00 E8 03 D5")


def test_vector_ping():
    assert P.make_packet(1, P.PING) == bytes.fromhex("FF FF 01 02 01 FB")


def test_checksum_and_parse():
    pkt = P.make_packet(5, 0, bytes([0x10, 0x20]))
    sid, err, params, used = P.parse_packet(b"\x00" + pkt + b"\x99")
    assert (sid, err, params, used) == (5, 0, bytes([0x10, 0x20]), len(pkt) + 1)
    bad = bytearray(pkt)
    bad[-1] ^= 1
    with pytest.raises(P.ProtocolError, match="checksum"):
        P.parse_packet(bytes(bad))


def test_sign_bit():
    for v in (0, 1, 2047, -1, -300, 30000, -30000):
        assert P.decode_signed(P.encode_signed(v)) == v
    assert P.encode_signed(-1) == 0x8001


def test_sync_write_packet():
    pkt = P.sync_write_packet(0x2A, 2, {1: P.u16(0x0800), 2: P.u16(0x0400)})
    assert pkt[:5] == bytes([0xFF, 0xFF, 0xFE, 2 + 2 + 2 * 3, 0x83])
    assert P.parse_packet(pkt)[0] == 0xFE


# ---- bus với servo giả ----
@pytest.fixture
def port():
    return FakeServoPort(ALL_IDS)


def test_bus_ping_read_write(port):
    bus = FeetechBus(port)
    assert bus.ping(1) and not bus.ping(99)
    assert bus.read(3, P.PRESENT_POSITION) == 2048
    bus.write(3, P.TORQUE_ENABLE, 1)
    bus.write(3, P.GOAL_POSITION, 3000)
    assert bus.read(3, P.PRESENT_POSITION) == 3000
    assert port.bad_checksums == 0
    assert port.log[-1] == P.read_packet(3, P.PRESENT_POSITION)


def test_bus_sync_read_write(port):
    bus = FeetechBus(port)
    bus.sync_write(P.TORQUE_ENABLE, {1: 1, 2: 1, 7: 1})
    bus.sync_write(P.GOAL_POSITION, {1: 1000, 2: 2000, 7: 3000})
    assert bus.sync_read(P.PRESENT_POSITION, [1, 2, 7]) == {1: 1000, 2: 2000, 7: 3000}


def test_bus_detects_bad_reply_checksum_and_retries(port):
    bus = FeetechBus(port, retries=1)
    port.corrupt_next_reply = True
    assert bus.read(1, P.PRESENT_VOLTAGE) == 120  # lần 1 sai checksum, lần 2 đúng


def test_eeprom_locked_until_unlock(port):
    bus = FeetechBus(port)
    bus.write(13, P.MODE, 1)
    assert bus.read(13, P.MODE) == 0
    bus.unlock_eeprom(13)
    bus.write(13, P.MODE, 1)
    bus.lock_eeprom(13)
    assert bus.read(13, P.MODE) == 1


# ---- backend ----
def test_backend_missing_servo():
    with pytest.raises(ConnectionError):
        FeetechBackend(port=FakeServoPort(ALL_IDS[:-1]), sleep=lambda s: None, allow_no_latch=True)


def test_backend_positions_roundtrip(port):
    # allow_lift_underpowered: bus giả không tải; phần cứng thật từ chối lift khi spec.LIFT_LIMIT_COVERS_DESIGN_LOAD False
    be = FeetechBackend(port=port, enable=True, sleep=lambda s: None, allow_no_latch=True, allow_lift_underpowered=True)
    assert port.mem[13][P.MODE[0]] == P.MODE_WHEEL and port.mem[1][P.MODE[0]] == P.MODE_POSITION
    targets = {"left_shoulder": -0.5, "right_elbow": 1.2, "left_gripper": 50.0, "lift": 0.3}
    be.write_joints(targets)
    # raw đúng quy đổi 4096 bước/vòng, tâm 2048
    assert P.from_u16(port.mem[2][42:44]) == round(2048 - 0.5 * 4096 / (2 * math.pi))
    j = be.read_joints()
    for k, v in targets.items():
        assert j[k] == pytest.approx(v, abs=2 * math.pi / 4096 + 1e-3)
    be.close()
    assert port.mem[1][P.TORQUE_ENABLE[0]] == 0


def test_backend_base_velocity_and_odometry(port):
    be = FeetechBackend(port=port, enable=True, sleep=lambda s: None, allow_no_latch=True)
    be.set_base_velocity(0.2, 0.0, 0.0)
    want = 0.0
    for _ in range(int(round((0.2 / spec.BASE_MAX_ACC + 1.0) / 0.02))):   # tăng tốc theo dốc rồi chạy đều 1 s
        want += be.base_velocity()[0] * 0.02                                  # odometry đọc vận tốc bánh đã ghi trước bước
        be.step(0.02)
    x, y, yaw = be.base_pose()
    assert be.base_velocity()[0] == pytest.approx(0.2)
    assert x == pytest.approx(want, abs=0.01) and abs(y) < 0.01 and abs(yaw) < 0.01
    # P-L: bánh trước (0°, wheel_0) không quay khi đi thẳng (v0.4: bánh sau 180°, wheel_1)
    assert P.decode_signed(P.from_u16(port.mem[spec.SERVO_IDS["wheel_0"]][46:48])) == 0


def test_same_demo_script_runs_on_feetech_mock(port):
    """Cùng kịch bản demo chạy được qua API Robot với backend feetech (bus giả, không vật lý)."""
    from demo_pick_place import run_pick_place
    r = Robot("feetech", port=port, enable=True, sleep=lambda s: None, allow_no_latch=True, allow_lift_underpowered=True)
    run_pick_place(r, log=lambda *_: None)
    assert port.bad_checksums == 0
    sync_writes = [p for p in port.log if p[4] == P.SYNC_WRITE]
    assert len(sync_writes) > 1000
    r.close()
