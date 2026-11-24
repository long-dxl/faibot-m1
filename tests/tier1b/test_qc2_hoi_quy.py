"""Hồi quy QC2 SDK Tầng 1b: M1 watchdog dừng nhầm, M2 khoá chết close() với luồng chốt E-stop.

Các ca chạy với chờ thật (time.sleep) và bus giả có trễ trả lời, vì lỗi M1 không lộ khi sleep rỗng và bus không trễ.
"""
import threading
import time

import pytest

from faibotic import spec

pytestmark = pytest.mark.t1b
LAT = 0.012          # trễ trả lời của bus giả (gần bus thật 1 Mbps, 18 servo)


def _rig(u, **kw):
    rig = u.build(sleep=time.sleep, latch_period=0.01, **kw)
    rig.port.latency_s = LAT
    return rig


def _wheels_zero(u, rig):
    return all(u.goal_speed(rig.port, i) == 0 for i in u.WHEEL_IDS)


# ---------------------------------------------------------------- M1
def test_m1_nghi_lau_roi_chay_de_khong_dung_nham(u):
    """Nghỉ 1 s (không step), rồi đặt vận tốc đế khác 0 và step(): không được WatchdogHet."""
    rig = _rig(u)
    time.sleep(1.0)
    rig.robot.set_base_velocity(0.0, 0.3, 0.0)
    for _ in range(15):
        rig.robot.step()
    assert rig.backend.enabled and rig.backend.watchdog_fault is None
    assert not _wheels_zero(u, rig), "đế chưa chạy"


def test_m1_step_dt_toi_da_khi_dang_chay_khong_dung_nham(u):
    """step(dt lớn nhất) lặp lại khi đế đang chạy: dt_max + một lần đọc bus nhỏ hơn hẳn WATCHDOG_S, không dừng nhầm."""
    dt_max = spec.MAX_STEP_DT_CYCLES * u.CHU_KY
    assert dt_max + 2 * spec.BUS_READ_DEADLINE_S < spec.WATCHDOG_S
    rig = _rig(u)
    rig.robot.set_base_velocity(0.0, 0.3, 0.0)
    for _ in range(12):
        rig.robot.step(dt_max)
        rig.robot.step(10.0)                     # bị chặn về dt_max ở Robot.step
    assert rig.backend.enabled and rig.backend.watchdog_fault is None


def test_m1_may_chu_treo_that_thi_watchdog_dung(u):
    """Máy chủ ngừng step() thật lâu hơn WATCHDOG_S khi đế chạy: bánh về 0, lệnh kế ném WatchdogHet."""
    rig = _rig(u)
    rig.robot.set_base_velocity(0.0, 0.3, 0.0)
    for _ in range(10):
        rig.robot.step()
    assert not _wheels_zero(u, rig)
    t0 = time.monotonic()
    while not _wheels_zero(u, rig) and time.monotonic() - t0 < spec.WATCHDOG_S + 0.3:
        time.sleep(0.005)
    assert _wheels_zero(u, rig), "bánh vẫn giữ tốc độ cũ"
    assert time.monotonic() - t0 <= spec.WATCHDOG_S + 0.15
    with pytest.raises(Exception) as ei:
        rig.robot.step()
    assert type(ei.value).__name__ == "WatchdogHet" and ei.value.ma == "SYS-03"


# ---------------------------------------------------------------- M2
@pytest.mark.parametrize("lan", range(5))
def test_m2_rut_usb_estop_va_close_dong_thoi_khong_khoa_chet(u, lan):
    """Rút USB + nhấn E-stop + close() cùng lúc, GPIO ghi chậm 50 ms (trong khoá chốt): close() trả về có hạn, RELAY_EN thấp."""
    rig = _rig(u)
    g = rig.estop
    w = g.write

    def slow_write(pin, level):
        time.sleep(0.05)
        w(pin, level)
    g.write = slow_write
    rig.port.unplug("exc")
    g.levels["ESTOP_SENSE"] = False                 # luồng chốt bắt đầu trip (giữ khoá chốt, ghi GPIO chậm)
    time.sleep(0.003 * lan)
    res = {}

    def closer():
        try:
            rig.robot.close()
        except BaseException as e:      # noqa: BLE001
            res["exc"] = e
    th = threading.Thread(target=closer, daemon=True)
    th.start()
    th.join(3.0)
    assert not th.is_alive(), "close() treo (khoá chết với luồng chốt E-stop)"
    assert "exc" not in res, repr(res.get("exc"))
    assert g.levels["RELAY_EN"] is False
    assert not rig.backend.enabled
