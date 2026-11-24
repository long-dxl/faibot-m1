"""Tầng 1b nhóm (a): lỗi bus/servo chèn trên bus giả (docs/S4/KE-HOACH-TANG-1B.md, T1b.R-01..R-14).

Chỉ viết test. Ca mà code hiện tại trượt đánh xfail(strict=True) kèm bản sửa kỳ vọng; khi bản sửa vào, test tự đỏ (XPASS strict)
cho tới khi bỏ đánh dấu. Mỗi ca đạt khi: về TTAT <= 200 ms (hoặc theo chu kỳ), lỗi có kiểu + mã (.ma), không treo, không tự chạy lại.
"""
import logging
import time

import pytest

from faibotic.feetech import protocol as P
from faibotic.feetech.bus import FeetechBus

DT = 0.02
LIMIT_S = 0.2             # TTAT <= 200 ms kể từ lúc chèn lỗi
FAST = 0.003              # timeout bus rút ngắn cho ca chạy nhiều chu kỳ (ca đo thời gian dùng timeout thật 0,02 s)
SHOULDER = 2              # left_shoulder
PAN = 1                   # left_pan
WHEEL0 = 13


def run_move(rig, inject, at=8, tgt=None, dur=1.0):
    """move_joints (tay) rồi chèn lỗi ở chu kỳ `at`. Trả (exc, giây từ lúc chèn tới khi hàm trả về)."""
    rig.inject_at(at, lambda: inject(rig))
    exc = None
    try:
        rig.robot.move_joints(tgt or {"left_pan": 0.4, "left_shoulder": -0.2, "left_elbow": 0.4}, duration=dur)
    except BaseException as e:      # noqa: BLE001
        exc = e
    return exc, time.monotonic() - (rig.t_inject or time.monotonic())


def assert_trip(u, rig, exc, dt, *, cls=None, ma=None, word=None, cycles=None):
    """Khuôn chung: có lỗi có kiểu + mã, về TTAT trong hạn, robot không còn enabled."""
    assert exc is not None, "lỗi chèn vào mà không có ngoại lệ nào (robot chạy tiếp)"
    assert not u.is_raw(exc), f"ngoại lệ thô lọt qua API: {type(exc).__name__}: {exc}"
    assert u.typed(exc, ma), f"thiếu mã lỗi .ma ({ma}): {type(exc).__name__}: {exc}"
    if cls:
        assert type(exc).__name__ == cls, type(exc).__name__
    if word:
        assert word in str(exc), str(exc)
    assert rig.ttat(), u.ttat_report(rig)
    assert not rig.backend.enabled
    assert dt <= LIMIT_S, f"TTAT sau {dt * 1000:.0f} ms > 200 ms"
    if cycles is not None:
        assert rig.steps - rig.step_inject <= cycles, f"{rig.steps - rig.step_inject} chu kỳ > {cycles}"


# ====================== lớp chèn lỗi của chính bus giả (A1) ======================
def test_fault_layer_defaults_keep_old_behaviour(u):
    port = u.FakeServoPort([1, 2])
    bus = FeetechBus(port)
    assert bus.ping(1) and bus.sync_read(P.PRESENT_POSITION, [1, 2]) == {1: 2048, 2: 2048}
    assert port.timeline is None and port.log == port.log[:] and port.rx_dropped == 0


def test_fault_layer_drop_rate_is_seeded_and_reproducible(u):
    def run():
        port = u.FakeServoPort([1], seed=7)
        port.drop_rate = 0.3
        bus, out = FeetechBus(port, timeout=0.001, retries=0), []
        for _ in range(60):
            out.append(bus.ping(1))
        return out
    a, b = run(), run()
    assert a == b and 5 < sum(not x for x in a) < 40


def test_fault_layer_corrupt_ids_and_dup_ids_break_checksum(u):
    port = u.FakeServoPort([1, 2])
    port.corrupt_ids = {2}
    bus = FeetechBus(port, timeout=0.001, retries=0)
    assert bus.ping(1) and not bus.ping(2)
    port.corrupt_ids = set()
    port.dup_ids = {2: 2}
    assert bus.ping(1) and not bus.ping(2)


def test_fault_layer_drop_rx_unplug_replug_and_timeline(u):
    port = u.FakeServoPort([1, 2])
    port.record_timeline()
    port.drop_rx_ids = {2}
    bus = FeetechBus(port, timeout=0.001, retries=0)
    bus.sync_write(P.GOAL_POSITION, {1: 1000, 2: 1000})
    assert port.rx_dropped == 1 and [e[1] for e in port.writes_to(P.GOAL_POSITION[0])] == [1]
    port.unplug("exc")
    with pytest.raises(OSError):
        bus.ping(1)
    port.replug()
    port.drop_rx_ids = set()
    assert bus.ping(1)
    port.unplug("silent")
    assert not bus.ping(1)
    port.write(P.sync_write_packet(P.TORQUE_ENABLE[0], 1, {1: b"\x01"}))
    assert port.mem[1][P.TORQUE_ENABLE[0]] == 1          # lệnh vẫn tới servo, gói trả không về


def test_fault_layer_voltage_temperature_flags_log_max_on_write(u):
    port = u.FakeServoPort([1], log_max=5)
    port.set_voltage(1, 85, auto_flag=True)
    port.set_temperature(1, 75, auto_flag=True)
    assert port.errors[1] == 1 | (1 << 2)
    port.set_voltage(1, 120, auto_flag=True)
    port.set_temperature(1, 30, auto_flag=True)
    assert port.errors[1] == 0
    bus = FeetechBus(port)
    for _ in range(20):
        bus.ping(1)
    assert len(port.log) == 5
    seen = []
    port.on_write = lambda t, sid, addr, data: seen.append((sid, addr))
    bus.write(1, P.GOAL_POSITION, 1500)
    assert seen == [(1, P.GOAL_POSITION[0])]


def test_fault_layer_latency_and_chunk(u):
    port = u.FakeServoPort([1])
    port.latency_s, port.chunk = 0.05, 2
    port.write(P.make_packet(1, P.PING))
    assert port.read(64) == b""
    time.sleep(0.06)
    assert len(port.read(64)) == 2


# ====================== R-01 .. R-14 ======================
def test_R01_mat_goi_le_khong_dung_nham(u):
    """T1b.R-01 (a): mất 1% gói trả, 300 chu kỳ (seed cố định): không dừng nhầm."""
    rig = u.build(timeout=FAST)
    rig.port.drop_rate = 0.01
    rig.robot.move_joints({"left_pan": 0.4, "right_pan": -0.4}, duration=6.0)      # 300 chu kỳ
    assert rig.backend.enabled and rig.port.rx_dropped == 0


def test_R01_mat_ba_chu_ky_lien_thi_ttat(u):
    """T1b.R-01 (b): servo vai trái mất gói liên tục từ chu kỳ 8: TTAT <= 3 chu kỳ."""
    rig = u.build(timeout=FAST)
    exc, dt = run_move(rig, lambda r: setattr(r.port, "drop_reply", lambda sid, instr: sid == SHOULDER))
    assert_trip(u, rig, exc, dt, cls="ServoMat", word="left_shoulder", cycles=3)
    assert rig.estop.levels["RELAY_EN"] is False, "mất liên lạc phải cắt chốt E-stop (QC2 m2)"


def test_R02_checksum_hong_le_bo_qua(u):
    """T1b.R-02 (a): một gói trả hỏng checksum đơn lẻ giữa chuyển động: không dừng."""
    rig = u.build(timeout=FAST)
    rig.inject_at(10, lambda: setattr(rig.port, "corrupt_next_reply", True))
    rig.robot.move_joints({"left_pan": 0.4}, duration=1.0)       # không được ném
    assert rig.backend.enabled


def test_R02_checksum_hong_lien_tiep_thi_ttat(u):
    """T1b.R-02 (b): corrupt_ids={vai trái} từ chu kỳ 8."""
    rig = u.build(timeout=FAST)
    exc, dt = run_move(rig, lambda r: r.port.corrupt_ids.add(SHOULDER))
    assert_trip(u, rig, exc, dt, ma="BUS-02", cycles=3)
    assert rig.estop.levels["RELAY_EN"] is False, "mất liên lạc phải cắt chốt E-stop (QC2 m2)"


@pytest.mark.xfail(strict=True, reason="T1b.R-03 (ngoài phạm vi lô sửa 24/11): giám sát sai số bám (đích - vị trí > ngưỡng N chu kỳ) cho P-L, ném ServoKhongTheo; "
                                       "ngưỡng và N do opus chốt theo tốc độ khớp tối đa")
@pytest.mark.parametrize("kind", ["drop_rx", "stuck"])
def test_R03_servo_khong_theo_lenh(u, kind):
    """T1b.R-03: servo pan nhận mà không theo (bỏ lệnh ghi, hoặc kẹt): phát hiện trước khi chuyển động xong, TTAT."""
    rig = u.build(timeout=FAST)
    exc, dt = run_move(rig, lambda r: (r.port.drop_rx_ids if kind == "drop_rx" else r.port.stuck).add(PAN),
                       at=5, tgt={"left_pan": 0.6}, dur=1.5)
    assert_trip(u, rig, exc, dt, cls="ServoKhongTheo", word="left_pan")


def test_R04_servo_khong_tra_loi_giua_chuyen_dong(u):
    """T1b.R-04 (timeout thật 0,02 s): dead.add(vai trái) giữa move_joints.
    Servo `dead` bỏ mọi lệnh ghi, nên TORQUE_ENABLE = 0 chỉ tới được nó khi chốt E-stop cắt nguồn servo (K1): bus giả mô hình hoá
    nguồn servo theo RELAY_EN (power_from) để ttat() thấy đúng tác dụng của việc cắt chốt."""
    rig = u.build()
    rig.port.power_from(rig.estop, "RELAY_EN")
    exc, dt = run_move(rig, lambda r: r.port.dead.add(SHOULDER))
    assert_trip(u, rig, exc, dt, cls="ServoMat", word="left_shoulder")
    assert rig.estop.levels["RELAY_EN"] is False, "mất liên lạc phải cắt chốt E-stop (QC2 m2)"


def test_R05_servo_thieu_luc_ket_noi(u):
    port = u.FakeServoPort(list(u.spec.SERVO_IDS.values()))
    port.dead.add(SHOULDER)
    with pytest.raises(ConnectionError) as ei:
        u.build(port=port, enable=False)
    e = ei.value
    assert "left_shoulder" in str(e) and "2" in str(e)
    assert u.typed(e), f"thiếu .ma: {e}"
    assert all(u.torque(port, i) == 0 for i in u.spec.SERVO_IDS.values())         # không bật mô-men


def test_R06_trung_id(u):
    port = u.FakeServoPort(list(u.spec.SERVO_IDS.values()))
    port.dup_ids = {SHOULDER: 2}
    with pytest.raises(ConnectionError) as ei:
        u.build(port=port, enable=False)
    e = ei.value
    assert "trùng" in str(e) and "không thấy" not in str(e), str(e)
    assert type(e).__name__ == "BusCauHinhSai" and u.typed(e)
    assert all(u.torque(port, i) == 0 for i in u.spec.SERVO_IDS.values())


def test_R07_dien_ap_tut(u, caplog):
    """T1b.R-07: áp mọi servo giảm 0,1 V mỗi chu kỳ từ 12,0 V; dưới V_CAT thì dừng, cảnh báo đúng một lần, enable() bị từ chối."""
    v_cat = getattr(u.spec, "V_CAT", 9.0)
    rig = u.build()
    ids = list(u.spec.SERVO_IDS.values())
    state = {"dv": 120}
    orig = rig.backend.step

    def step(dt):
        state["dv"] -= 1
        for i in ids:
            rig.port.set_voltage(i, state["dv"])
        if state["dv"] == int(v_cat * 10) - 1:
            rig.t_inject = time.monotonic()
        return orig(dt)
    rig.backend.step = step
    caplog.set_level(logging.WARNING, logger="faibotic")
    exc = None
    try:
        rig.robot.move_joints({"left_pan": 0.4}, duration=2.5)            # 125 chu kỳ > 40 chu kỳ để xuống 8,0 V
    except BaseException as e:      # noqa: BLE001
        exc = e
    assert exc is not None and type(exc).__name__ == "DienApThap" and u.typed(exc)
    assert rig.ttat() and not rig.backend.enabled
    assert rig.t_inject is None or time.monotonic() - rig.t_inject <= 1.0
    assert sum("điện áp" in r.getMessage() for r in caplog.records if r.levelno == logging.WARNING) == 1
    rig.backend.clear_fault()
    with pytest.raises(Exception) as ei:
        rig.backend.enable()
    assert type(ei.value).__name__ == "DienApThap"


def test_R07_bit_dien_ap_cua_servo(u):
    rig = u.build()
    exc, dt = run_move(rig, lambda r: r.port.set_voltage(SHOULDER, 85, auto_flag=True))
    assert_trip(u, rig, exc, dt, cls="DienApThap", cycles=1)


def test_R08_qua_nhiet_servo_tay(u):
    rig = u.build()
    exc, dt = run_move(rig, lambda r: r.port.set_temperature(SHOULDER, 80, auto_flag=True))
    assert_trip(u, rig, exc, dt, cls="ServoBaoLoi", word="quá nhiệt", cycles=2)
    assert "left_shoulder" in str(exc)


def test_R09_qua_tai_servo_tay(u):
    rig = u.build()
    exc, dt = run_move(rig, lambda r: r.port.errors.__setitem__(SHOULDER, 1 << 5))
    assert_trip(u, rig, exc, dt, cls="ServoBaoLoi", word="quá tải", cycles=2)
    assert "left_shoulder" in str(exc)


def test_R10_co_loi_servo_banh(u):
    """T1b.R-10: wheel_0 báo quá tải khi đế đang chạy: bánh nhận tốc độ 0 trong 1 chu kỳ, TTAT."""
    rig = u.build()
    rig.inject_at(15, lambda: rig.port.errors.__setitem__(WHEEL0, 1 << 5))
    exc = None
    try:
        rig.robot.move_base(0.0, 0.5, 0.0, duration=2.0)
    except BaseException as e:      # noqa: BLE001
        exc = e
    assert exc is not None and type(exc).__name__ == "ServoBaoLoi" and u.typed(exc)
    assert all(u.goal_speed(rig.port, i) == 0 for i in u.WHEEL_IDS), u.ttat_report(rig)
    assert rig.backend.base_velocity().tolist() == [0.0, 0.0, 0.0]
    assert rig.ttat() and not rig.backend.enabled


@pytest.mark.parametrize("what", ["move_joints", "move_base"])
def test_R11_rut_usb_giua_phien(u, what):
    rig = u.build()
    rig.inject_at(8, lambda: rig.port.unplug("exc"))
    exc = None
    try:
        if what == "move_joints":
            rig.robot.move_joints({"left_pan": 0.4}, duration=1.0)
        else:
            rig.robot.move_base(0.0, 0.3, 0.0, duration=1.0)
    except BaseException as e:      # noqa: BLE001
        exc = e
    assert isinstance(exc, ConnectionError) and type(exc).__name__ == "BusMat" and u.typed(exc, "BUS-01"), repr(exc)
    assert not u.is_raw(exc) and not rig.backend.enabled
    with pytest.raises(ConnectionError) as ei:                      # mọi lệnh sau ném lại BusMat
        rig.robot.set_joint_positions({"left_pan": 0.1})
    assert type(ei.value).__name__ == "BusMat"
    t = time.monotonic()
    rig.robot.close()                                              # không ném, không treo
    assert time.monotonic() - t <= 1.0


def test_R12_cap_servo_dut_usb_con(u):
    rig = u.build()
    exc, dt = run_move(rig, lambda r: r.port.unplug("silent"))
    assert_trip(u, rig, exc, dt, cls="BusMat", word="im lặng")
    assert rig.estop.levels["RELAY_EN"] is False, "mất liên lạc phải cắt chốt E-stop (QC2 m2)"


def test_R13_tre_duoi_timeout_van_doc_dung(u):
    """T1b.R-13 (a, hồi quy): servo trả chậm 0,9 x timeout: read_joints vẫn đúng và không quá 0,1 s."""
    rig = u.build(enable=False)
    rig.port.latency_s = 0.9 * 0.02
    worst = 0.0
    for _ in range(20):
        t = time.monotonic()
        rig.backend.read_joints()
        worst = max(worst, time.monotonic() - t)
    assert worst <= 0.1, f"{worst * 1000:.0f} ms"


@pytest.mark.parametrize("lat_x", [0.0, 1.1])
def test_R13_tran_thoi_gian_khi_co_servo_im(u, lat_x):
    """T1b.R-13 (b): một servo im lặng (có/không kèm trễ 1,1 x timeout của servo khác): read_joints phải trả/ném trong <= 100 ms."""
    rig = u.build(enable=False)
    rig.port.latency_s = lat_x * 0.02
    rig.port.dead.add(SHOULDER)
    t = time.monotonic()
    with pytest.raises(Exception):
        rig.backend.read_joints()
    dt = time.monotonic() - t
    assert dt <= 0.1, f"{dt * 1000:.0f} ms"


def test_R14_goi_vun_va_byte_rac(u):
    """T1b.R-14: chunk=3, byte rác 0x00/0xFF chèn trước gói: 1000 lần đọc đúng 100%."""
    rig = u.build(enable=False)
    rig.port.chunk = 3
    rig.port.garbage_rate = 0.5
    ids = [u.spec.SERVO_IDS[n] for n in ("left_pan", "left_shoulder", "lift")]
    raws = {1: 1500, 2: 2600, 16: 100}
    for sid, raw in raws.items():
        rig.port.set_position(sid, raw)
    bus = rig.backend.bus
    bad = 0
    for _ in range(1000):
        got = bus.sync_read(P.PRESENT_POSITION, ids)
        bad += got != raws
    assert bad == 0
