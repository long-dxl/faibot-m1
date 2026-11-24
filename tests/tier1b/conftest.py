"""Tầng 1b, lane A: dựng robot trên bus giả có lớp chèn lỗi, và các hàm đo TTAT (trạng thái an toàn).

TTAT (docs/S4/KE-HOACH-TANG-1B.md mục 1): mô-men mọi servo tay và kẹp tắt; bánh GOAL_SPEED = 0 (hoặc mô-men tắt);
lift được giữ bằng chốt (cuộn chốt mất điện). Dùng qua fixture `u` (không import trực tiếp, tránh trùng tên conftest).
"""
from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest

from faibotic import Robot, spec
from faibotic.backends.feetech import ARM_NAMES, GRIP_NAMES, WHEELS
from faibotic.feetech import protocol as P
from faibotic.feetech.mockbus import FakeServoPort
from faibotic.safety import DockSwitches, FakeGPIO, LiftLatch, SafetyLatch, SimLiftLatchHW

RAW_TYPES = (OSError, ValueError, ZeroDivisionError, KeyError, OverflowError, AttributeError, TypeError)
CHU_KY = 0.02                     # backend.control_dt (50 Hz)
ARM_IDS = [spec.SERVO_IDS[n] for n in ARM_NAMES + GRIP_NAMES]
WHEEL_IDS = [spec.SERVO_IDS[n] for n in WHEELS]
LIFT_ID = spec.SERVO_IDS["lift"]


class Rig(SimpleNamespace):
    """port, robot, backend, estop (FakeGPIO của chốt), dock (FakeGPIO), lift_hw (SimLiftLatchHW), latch, t_inject."""

    def ttat(self) -> bool:
        return ttat(self)

    def inject_at(self, n: int, fn) -> None:
        """Gọi fn() ngay trước lần backend.step thứ n (1-based) và ghi t_inject (time.monotonic)."""
        be, orig, cnt = self.backend, self.backend.step, [0]

        def step(dt):
            cnt[0] += 1
            self.steps = cnt[0]
            if cnt[0] == n:
                self.t_inject = time.monotonic()
                self.step_inject = n
                fn()
            return orig(dt)
        be.step = step


def goal_speed(port, sid: int) -> int:
    a = P.GOAL_SPEED[0]
    return P.decode_signed(P.from_u16(bytes(port.mem[sid][a:a + 2])))


def torque(port, sid: int) -> int:
    return port.mem[sid][P.TORQUE_ENABLE[0]]


def ttat(rig) -> bool:
    p = rig.port
    if any(torque(p, i) for i in ARM_IDS):
        return False
    if any(torque(p, i) and goal_speed(p, i) for i in WHEEL_IDS):
        return False
    if rig.lift_hw is not None and rig.lift_hw.en:
        return False
    return True


def ttat_report(rig) -> str:
    p = rig.port
    return (f"arm_torque={[torque(p, i) for i in ARM_IDS]} wheel=({[torque(p, i) for i in WHEEL_IDS]},"
            f"{[goal_speed(p, i) for i in WHEEL_IDS]}) coil={getattr(rig.lift_hw, 'en', None)}")


def is_raw(e: BaseException) -> bool:
    """Ngoại lệ thô lọt qua API (không có mã lỗi .ma của M1Loi)."""
    return not hasattr(e, "ma") and isinstance(e, RAW_TYPES + (TimeoutError,))


def typed(e: BaseException, ma: str | None = None) -> bool:
    """Lớp lỗi có kiểu: có .ma (mã lỗi, mục 2.5) và, nếu cho, mã khớp tiền tố."""
    m = getattr(e, "ma", None)
    return isinstance(m, str) and (ma is None or m.startswith(ma))


def build(*, enable: bool = True, latch: bool = True, dock: bool = True, lift_latch: bool = True, timeout: float | None = None,
          ids=None, record: bool = True, port=None, **kw) -> Rig:
    """Robot("feetech") trên FakeServoPort. Mặc định: chốt E-stop PCB (FakeGPIO, luồng giám sát 2 ms), khoá trạm đóng, chốt lift giả,
    allow_lift_underpowered=True, sleep rỗng (không chờ thật). timeout: đè FeetechBus.timeout (mặc định 0,02 s như thật)."""
    port = port or FakeServoPort(list((ids or spec.SERVO_IDS).values()), seed=1234)
    if record:
        port.record_timeline()
    estop = FakeGPIO()
    dock_g = FakeGPIO(DOCK_L=True, DOCK_R=True)
    lift_hw = SimLiftLatchHW() if lift_latch else None
    bk = dict(port=port, sleep=lambda s: None, allow_lift_underpowered=True, latch_period=0.002)
    lt = None
    if latch:
        lt = SafetyLatch(estop, sleep=lambda s: None, settle_s=0, hw="pcb")
        lt.reset()
        bk["latch"] = lt
    else:
        bk["allow_no_latch"] = True
    if dock:
        bk["dock"] = DockSwitches(dock_g)
    if lift_latch:
        bk["lift_latch"] = LiftLatch(lift_hw, sleep=lambda s: None)
    if ids is not None:
        bk["ids"] = ids
    bk.update(kw)
    robot = Robot("feetech", **{**bk, "enable": False})
    if timeout is not None:
        robot.backend.bus._bus.timeout = timeout
    rig = Rig(port=port, robot=robot, backend=robot.backend, estop=estop, dock=dock_g, lift_hw=lift_hw, latch=lt,
              t_inject=None, steps=0, step_inject=None)
    if enable:
        robot.backend.enable()
    return rig


@pytest.fixture
def u():
    """Hàm tiện ích + dựng robot; tự dừng luồng chốt của mọi robot khi test xong."""
    made = []

    def _build(**kw):
        r = build(**kw)
        made.append(r)
        return r

    def _close(r):
        """Dừng luồng chốt của một rig sớm (ca lặp nhiều lần)."""
        try:
            if r.backend.latch is not None:
                r.backend.latch.stop_monitor()
        finally:
            if r in made:
                made.remove(r)

    ns = SimpleNamespace(build=_build, close=_close, ttat=ttat, ttat_report=ttat_report, is_raw=is_raw, typed=typed, torque=torque,
                         goal_speed=goal_speed, ARM_IDS=ARM_IDS, WHEEL_IDS=WHEEL_IDS, LIFT_ID=LIFT_ID, CHU_KY=CHU_KY,
                         FakeServoPort=FakeServoPort, spec=spec, P=P, threads_base=threading.active_count())
    yield ns
    for r in made:
        try:
            if r.backend.latch is not None:
                r.backend.latch.stop_monitor()
        except Exception:      # noqa: BLE001
            pass
