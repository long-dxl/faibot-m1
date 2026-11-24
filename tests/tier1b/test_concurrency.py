"""Tầng 1b nhóm (e): đồng thời (docs/S4/KE-HOACH-TANG-1B.md, T1b.R-37..R-42).

Hai client, hai tiến trình mở cùng cổng, close() từ luồng khác, đóng nhiều lần, kết nối lại, đua E-stop. Ca trượt đánh xfail(strict=True).
"""
import threading
import time

import pytest

from faibotic import Robot
from faibotic.backends.feetech import EStopActive
from faibotic.feetech import protocol as P

LIMIT_S = 0.2


def run(fn):
    try:
        fn()
    except BaseException as e:      # noqa: BLE001
        return e
    return None


def wait_for(cond, timeout=1.0):
    t = time.monotonic()
    while time.monotonic() - t < timeout:
        if cond():
            return True
        time.sleep(0.001)
    return cond()


# ---------------------------------------------------------------- R-37
def test_R37_hai_client_cung_mot_robot(u):
    """T1b.R-37: hai luồng cùng gọi set_joint_positions/read_joints/step trên một Robot, 1000 lần: không gói hỏng, không treo, luồng hai nhận RobotBan."""
    rig = u.build(timeout=0.01)
    robot = rig.robot
    gate = threading.Barrier(2)
    errs = {"A": [], "B": []}

    def worker(name, joint, sign):
        gate.wait()
        for i in range(1000):
            try:
                robot.set_joint_positions({joint: sign * 0.2 * ((i % 10) / 10)})
                robot.get_state()
                robot.step()
            except BaseException as e:      # noqa: BLE001
                errs[name].append(e)
                if type(e).__name__ == "RobotBan":
                    return
            time.sleep(0)
    ts = [threading.Thread(target=worker, args=("A", "left_pan", 1), daemon=True),
          threading.Thread(target=worker, args=("B", "right_pan", -1), daemon=True)]
    t0 = time.monotonic()
    for t in ts:
        t.start()
    for t in ts:
        t.join(15)
    assert not any(t.is_alive() for t in ts), "treo"
    assert time.monotonic() - t0 < 15
    assert rig.port.bad_checksums == 0
    bans = [e for v in errs.values() for e in v if type(e).__name__ == "RobotBan"]
    assert bans and all(u.typed(e) for e in bans), {k: [type(e).__name__ for e in v[:3]] for k, v in errs.items()}
    others = [e for v in errs.values() for e in v if type(e).__name__ != "RobotBan"]
    assert not others, others[:3]


# ---------------------------------------------------------------- R-38
def test_R38_cong_ban_bi_bao_dung_loai_loi(monkeypatch):
    import serial

    def busy(*a, **k):
        raise serial.SerialException("[Errno 16] Device or resource busy: '/dev/ttyUSB9'")
    monkeypatch.setattr(serial, "Serial", busy)
    with pytest.raises(Exception) as ei:
        Robot("feetech", port="/dev/ttyUSB9", allow_no_latch=True)
    e = ei.value
    assert isinstance(e, ConnectionError) and type(e).__name__ == "BusMat" and hasattr(e, "ma"), repr(e)
    assert "chương trình khác" in str(e)


def test_R38_cong_mo_voi_exclusive(monkeypatch):
    import serial
    seen = {}

    class Stop(Exception):
        pass

    def spy(*a, **k):
        seen.update(k)
        raise Stop
    monkeypatch.setattr(serial, "Serial", spy)
    with pytest.raises(Exception):
        Robot("feetech", port="/dev/ttyUSB9", allow_no_latch=True)
    assert seen.get("exclusive") is True, seen



# ---------------------------------------------------------------- R-39
def test_R39_close_tu_luong_khac_giua_move_joints(u):
    rig = u.build(sleep=time.sleep)                         # chu kỳ thật 20 ms để luồng B chen vào giữa chuyển động
    res = {}

    def a():
        res["exc"] = run(lambda: rig.robot.move_joints({"left_pan": 0.5, "right_pan": -0.5}, duration=1.5))
        res["t_end"] = time.monotonic()
    ta = threading.Thread(target=a, daemon=True)
    ta.start()
    time.sleep(0.5)
    t_close = time.monotonic()
    rig.robot.close()
    ta.join(3)
    assert not ta.is_alive(), "treo"
    assert rig.ttat() and not rig.backend.enabled, u.ttat_report(rig)
    assert time.monotonic() - t_close <= LIMIT_S + 0.1
    exc = res.get("exc")
    assert exc is not None and type(exc).__name__ == "RobotDaDong" and u.typed(exc), repr(exc)
    assert res["t_end"] - t_close <= 0.1, "A không thoát trong ~1 chu kỳ sau close()"


# ---------------------------------------------------------------- R-40
def test_R40_close_hai_lan_khong_loi(u):
    """T1b.R-40 (a, hồi quy): close() gọi lại không lỗi, không treo."""
    rig = u.build()
    rig.robot.close()
    t = time.monotonic()
    rig.robot.close()
    assert time.monotonic() - t <= 1.0 and rig.ttat()


def test_R40_lenh_sau_khi_dong(u):
    rig = u.build()
    rig.robot.close()
    for call in (lambda: rig.robot.set_joint_positions({"left_pan": 0.1}), lambda: rig.robot.set_base_velocity(0, 0.1, 0),
                 lambda: rig.robot.step()):
        e = run(call)
        assert e is not None and type(e).__name__ == "RobotDaDong" and u.typed(e), repr(e)


def test_R40_ctrl_c_giua_move_base(u):
    """T1b.R-40 (c): KeyboardInterrupt giữa move_base trong `with Robot(...)`: __exit__ dừng đế và tắt mô-men, ngoại lệ được truyền ra nguyên vẹn."""
    rig = u.build()

    def boom():
        raise KeyboardInterrupt
    rig.inject_at(30, boom)
    with pytest.raises(KeyboardInterrupt):
        with rig.robot:
            rig.robot.move_base(0.0, 0.5, 0.0, duration=2.0)
    assert rig.ttat() and not rig.backend.enabled, u.ttat_report(rig)
    assert all(u.goal_speed(rig.port, i) == 0 for i in u.WHEEL_IDS), u.ttat_report(rig)


# ---------------------------------------------------------------- R-41
def test_R41_ket_noi_lai_sau_mat_bus(u):
    base = threading.active_count()
    old = u.build()
    old.inject_at(8, lambda: old.port.unplug("exc"))
    exc = run(lambda: old.robot.move_joints({"left_pan": 0.4}, duration=1.0))
    assert type(exc).__name__ == "BusMat", repr(exc)
    assert run(old.robot.close) is None
    assert old.estop.levels["RELAY_EN"] is False
    old.port.replug()
    new = u.build(port=old.port)
    new.backend.enable()
    assert new.backend.enabled
    new.robot.close()
    assert wait_for(lambda: threading.active_count() <= base, 1.0), f"{threading.active_count()} luồng > gốc {base}"


# ---------------------------------------------------------------- R-42
def _race_once(u, op):
    rig = u.build(enable=(op != "enable"))
    main = threading.current_thread()
    fired = []

    def match(sid, addr, data):
        if op == "enable":
            return addr == P.TORQUE_ENABLE[0] and data == b"\x01"
        if op == "step":
            return addr == P.GOAL_SPEED[0]
        return addr == P.GOAL_POSITION[0]

    def hook(t, sid, addr, data):
        if fired or threading.current_thread() is not main or not match(sid, addr, data):
            return
        fired.append(t)
        rig.estop.levels["ESTOP_SENSE"] = False                       # E-stop đến đúng lúc luồng chính đang giữ khoá bus
        wait_for(lambda: rig.latch.state == "estop", 0.5)
    rig.port.on_write = hook
    if op == "enable":
        exc = run(rig.backend.enable)
    elif op == "step":
        rig.robot.set_base_velocity(0.0, 0.2, 0.0)
        exc = run(lambda: [rig.robot.step() for _ in range(3)])
    else:
        exc = run(lambda: rig.robot.set_joint_positions({"left_pan": 0.1}))
    ok = wait_for(rig.ttat, 0.5) and not rig.backend.enabled
    fired_ok = bool(fired)
    rep = u.ttat_report(rig)
    u.close(rig)
    return ok, fired_ok, exc, rep


@pytest.mark.parametrize("op,n", [("write_joints", 334), ("step", 333), ("enable", 333)])
def test_R42_dua_estop_voi_luong_chinh(u, op, n):
    """T1b.R-42: E-stop bắn đúng lúc luồng chính giữ khoá bus (qua on_write) trong write_joints / step / enable: trạng thái cuối luôn là TTAT
    (tổng 1000 lần lặp). enable() không bao giờ để servo bật; các lệnh khác không ném ngoại lệ thô."""
    bad = []
    for i in range(n):
        ok, fired, exc, rep = _race_once(u, op)
        assert fired, "hook không chạy (ca vô nghĩa)"
        if not ok or (exc is not None and not isinstance(exc, EStopActive)):
            bad.append((i, rep, repr(exc)))
    assert not bad, f"{len(bad)}/{n} lần không về TTAT hoặc ném lỗi lạ: {bad[:2]}"
