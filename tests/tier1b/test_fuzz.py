"""Tầng 1b, nhóm (c) fuzz, R-26..R-33 (docs/S4/KE-HOACH-TANG-1B.md mục 3).

Bộ sinh ca: `random.Random(seed)` của thư viện chuẩn, seed cố định, N ca mỗi đầu vào (hypothesis chưa duyệt giấy phép MPL-2.0).
Có thể đổi sang hypothesis sau: mỗi hàm `gen_*` trả một ca, mỗi `kiem_*` là mệnh đề cần đúng với mọi ca.
Biến môi trường: FUZZ_N (mặc định 200; đêm 10000), FUZZ_SEED (mặc định 20261124).

Chỉ viết test, không sửa SDK. Ca nào code hiện tại trượt: xfail(strict=True, reason="T1b.R-xx: <sửa cần có>").
Mỗi test gom toàn bộ ca hỏng rồi assert một lần, in tối đa 5 ca đầu kèm seed và chỉ số để tái hiện.
"""
from __future__ import annotations

import contextlib
import importlib.util
import math
import os
import random
import re
import signal
import time

import numpy as np
import pytest

from faibotic import Robot, limits, spec
from faibotic.backends.feetech import (DockOpen, EStopActive, FeetechBackend, LiftFault, OutOfSoftLimits, PairFault,
                                        default_calibration)
from faibotic.cli import main as cli_main
from faibotic.feetech import protocol as P
from faibotic.feetech.bus import FeetechBus
from faibotic.feetech.mockbus import FakeServoPort
from faibotic.gpio_pi import GPIOConfigError, GPIOUnavailable, PiGPIO, config_from_env
from faibotic.safety import DockSwitches, FakeGPIO

pytestmark = [pytest.mark.t1b, pytest.mark.fuzz, pytest.mark.filterwarnings("ignore::RuntimeWarning")]

N = int(os.environ.get("FUZZ_N", "200"))
SEED = int(os.environ.get("FUZZ_SEED", "20261124"))
ALL_IDS = list(spec.SERVO_IDS.values())
NAMES = spec.all_joint_names()
NOSLEEP = dict(sleep=lambda s: None, allow_lift_underpowered=True)

# Ngoại lệ do SDK chủ động ném khi từ chối lệnh (được phép). Mọi loại khác (OverflowError, ZeroDivisionError, IndexError, AttributeError...) là lọt.
ALLOWED = (ValueError, TypeError, KeyError, limits.KeepoutError, PairFault, LiftFault, DockOpen, EStopActive, OutOfSoftLimits)
# Câu báo do thư viện chuẩn tự sinh (không phải SDK viết): coi như lộ lỗi thô dù kiểu là ValueError/TypeError.
CAU_THO = re.compile(r"max\(\)|min\(\)|cannot convert|not supported between|has no attribute|unsupported operand|can't multiply|"
                     r"could not convert|division|^''$|^$|must be real number|out of range for|index out of range|object is not|"
                     r"not iterable|invalid literal")
SPECIAL = [math.nan, math.inf, -math.inf, 1e308, -1e308, -0.0, 5e-324, 1e-310, 1e300, -1e300, 0.0, 1.0, -1.0, 1e6, -1e6]


def ro(e: BaseException, ten: str | None = None) -> bool:
    """Câu báo do SDK viết: đủ dài, không phải câu thô của Python, (nếu cho) nhắc tên khớp/đối số."""
    msg = e.args[0] if isinstance(e, KeyError) and e.args else str(e)
    msg = str(msg)
    return len(msg) >= 8 and not CAU_THO.search(msg) and (ten is None or ten in msg)


class Het(Exception):
    """Hết giờ (lệnh treo hoặc chạy vô hạn)."""


@contextlib.contextmanager
def han(giay: float):
    """Trần thời gian cho một lệnh (SIGALRM, chỉ luồng chính, Linux/macOS)."""
    def _h(*a):
        raise Het(f"quá {giay} s")
    old = signal.signal(signal.SIGALRM, _h)
    signal.setitimer(signal.ITIMER_REAL, giay)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)


class Bao:
    """Gom ca hỏng, assert một lần ở cuối."""

    def __init__(self, ten: str):
        self.ten, self.loi = ten, []

    def them(self, i, ca, vi_sao):
        if len(self.loi) < 5:
            self.loi.append(f"[{self.ten} seed={SEED} ca#{i}] {ca!r}: {vi_sao}")
        self.dem = getattr(self, "dem", 0) + 1

    def kiem(self):
        assert not self.loi, f"{getattr(self, 'dem', 0)}/{N} ca hỏng; 5 ca đầu:\n" + "\n".join(self.loi)


def xf(ma: str, sua: str):
    return pytest.mark.xfail(strict=True, reason=f"T1b.R-{ma}: {sua}")


# ------------------------------------------------------------------ dựng robot
def dock_ok():
    return DockSwitches(FakeGPIO(DOCK_L=True, DOCK_R=True))


def make_robot(kind: str) -> Robot:
    if kind == "sim":
        return Robot("sim")
    return Robot("feetech", port=FakeServoPort(ALL_IDS), dock=dock_ok(), allow_no_latch=True, enable=True, **NOSLEEP)


@pytest.fixture(params=["sim", "fsp"])
def robot(request):
    r = make_robot(request.param)
    r.kind = request.param
    yield r
    with contextlib.suppress(BaseException):
        r.close()


def snap(r: Robot) -> dict:
    """Ảnh chụp trạng thái lệnh: so trước/sau để biết lệnh bị từ chối đã không gửi gì."""
    s = {"targets": dict(r._targets), "lift_goal": r._lift_goal}
    if r.kind == "sim":
        s["ctrl"] = r.backend.data.ctrl.copy()
        s["btargets"] = dict(r.backend._targets)
        s["goal"] = r.backend.base_ramp.goal.copy()
    else:
        s["cmd"] = dict(r.backend._cmd)
        s["mem"] = {sid: bytes(m[P.GOAL_POSITION[0]:P.GOAL_POSITION[0] + 2]) for sid, m in r.backend.bus._bus.port.mem.items()}
        s["goal"] = r.backend.base_ramp.goal.copy()
    return s


def giong(a: dict, b: dict) -> bool:
    for k in a:
        x, y = a[k], b[k]
        if isinstance(x, np.ndarray):
            if not np.array_equal(x, y):
                return False
        elif x != y:
            return False
    return True


def huu_han(r: Robot) -> str | None:
    """None nếu mọi giá trị lệnh/trạng thái còn hữu hạn; ngược lại mô tả chỗ hỏng."""
    vals = dict(r._targets)
    if r.kind == "sim":
        vals.update({f"btarget.{k}": v for k, v in r.backend._targets.items()})
        arrs = {"data.ctrl": r.backend.data.ctrl, "data.qpos": r.backend.data.qpos, "base_vel": r.backend.base_velocity()}
    else:
        vals.update({f"cmd.{k}": v for k, v in r.backend._cmd.items()})
        arrs = {"base_vel": r.backend.base_velocity(), "wheel_cmd": r.backend._wheel_cmd}
    for k, v in vals.items():
        if not math.isfinite(v):
            return f"{k}={v}"
    for k, a in arrs.items():
        if not np.all(np.isfinite(a)):
            return f"{k} có giá trị không hữu hạn"
    return None


def rnd_float(rng: random.Random, name: str | None = None) -> float:
    u = rng.random()
    if u < 0.35:
        return rng.choice(SPECIAL)
    if u < 0.6 or name is None:
        return rng.uniform(-10, 10)
    lo, hi = spec.limits(name)
    return rng.uniform(lo - 0.1 * (hi - lo), hi + 0.1 * (hi - lo))


# ------------------------------------------------------------------ R-26 giá trị set_joint_positions
def test_r26_gia_tri_set_joint_positions(robot):
    rng = random.Random(SEED + 26)
    bao = Bao("R-26")
    for i in range(N):
        ks = rng.sample(NAMES, rng.randint(1, 4))
        ca = {k: rnd_float(rng, k) for k in ks}
        xau = [k for k, v in ca.items() if not math.isfinite(v)]
        truoc = snap(robot)
        try:
            robot.set_joint_positions(dict(ca))
            loi = None
        except BaseException as e:   # noqa: BLE001
            loi = e
        if loi is not None and not isinstance(loi, ALLOWED):
            bao.them(i, ca, f"ngoại lệ thô {type(loi).__name__}: {loi}")
        elif xau and loi is None:
            bao.them(i, ca, f"giá trị không hữu hạn {xau} được nhận, không báo lỗi")
        elif xau and not (isinstance(loi, (ValueError, TypeError)) and ro(loi, xau[0])):
            bao.them(i, ca, f"cần ValueError nêu tên khớp {xau[0]}, nhận {type(loi).__name__}: {loi}")
        elif loi is not None and not giong(truoc, snap(robot)):
            bao.them(i, ca, "lệnh bị từ chối nhưng trạng thái lệnh đã đổi")
        chuoi = huu_han(robot)
        if chuoi:
            bao.them(i, ca, f"trạng thái không hữu hạn: {chuoi}")
            if robot.kind == "sim":
                break      # sim đã hỏng (NaN): các ca sau vô nghĩa
    bao.kiem()


# ------------------------------------------------------------------ R-27 kiểu và khoá
OK_KIEU = [lambda: np.float32(0.1), lambda: np.float64(0.05), lambda: np.array(0.1), lambda: np.int32(0), lambda: 0, lambda: 0.2,
           lambda: np.array([0.1])[0]]
XAU_KIEU = [lambda: "x", lambda: None, lambda: [0.1], lambda: (0.1,), lambda: {}, lambda: b"1", lambda: 1 + 2j, lambda: object()]
TEN_SAI = ["zzz", "Lift", "left_shoulder ", "left-shoulder", "LEFT_PAN", "left_", "lift2", "gripper"]


def test_r27_kieu_va_khoa(robot):
    rng = random.Random(SEED + 27)
    bao = Bao("R-27")
    for i in range(N):
        loai = rng.choice(["ok", "xau", "ten", "khong_dict"])
        k = rng.choice([n for n in NAMES if n != "lift" and "gripper" not in n])
        if loai == "ok":
            ca, dung = {k: rng.choice(OK_KIEU)()}, "ok"
        elif loai == "xau":
            ca, dung = {k: rng.choice(XAU_KIEU)()}, "type"
        elif loai == "ten":
            ca, dung = {rng.choice(TEN_SAI): 0.1}, "key"
        else:
            ca, dung = rng.choice(["x", None, 5, 3.5, [("lift", 0.1)], [], (1, 2)]), "dict"
        truoc = snap(robot)
        try:
            robot.set_joint_positions(ca)
            loi = None
        except BaseException as e:   # noqa: BLE001
            loi = e
        if dung == "ok":
            if loi is not None and not isinstance(loi, limits.KeepoutError):
                bao.them(i, ca, f"kiểu số hợp lệ bị từ chối: {type(loi).__name__}: {loi}")
        elif dung == "type":
            if not (isinstance(loi, TypeError) and ro(loi, k)):
                bao.them(i, ca, f"cần TypeError nêu tên khớp {k}, nhận {type(loi).__name__ if loi else 'không lỗi'}: {loi}")
        elif dung == "dict":
            if not (isinstance(loi, TypeError) and ro(loi)):
                bao.them(i, ca, f"cần TypeError (đối số không phải dict), nhận {type(loi).__name__ if loi else 'không lỗi'}: {loi}")
        else:
            ten = next(iter(ca))
            if not (isinstance(loi, KeyError) and ro(loi, ten) and NAMES[0] in str(loi.args[0])):
                bao.them(i, ca, f"cần KeyError nêu '{ten}' và danh sách khớp hợp lệ, nhận {type(loi).__name__ if loi else 'không lỗi'}: {loi}")
        if loi is not None and not giong(truoc, snap(robot)):
            bao.them(i, ca, "lệnh bị từ chối nhưng trạng thái lệnh đã đổi")
        if huu_han(robot):
            bao.them(i, ca, f"trạng thái không hữu hạn: {huu_han(robot)}")
    bao.kiem()


def test_r27_dict_rong_khong_lam_gi(robot):
    truoc = snap(robot)
    assert robot.set_joint_positions({}) == {}
    assert giong(truoc, snap(robot))


# ------------------------------------------------------------------ R-28 move_tcp
P_TOT = [0.33, 0.24, 0.47]     # tay trái với tới được từ tư thế gốc (đã thử trên sim)
P_XA = [0.40, 0.22, 0.45]


def gen_tcp(rng: random.Random):
    """Một ca move_tcp: đúng MỘT tham số hỏng, còn lại hợp lệ."""
    side, p, dur, gy = "left", list(P_TOT), 1.0, 0.0
    which = rng.choice(["p", "p", "side", "dur", "gy"])
    if which == "p":
        bad = rng.choice(["nan", "inf", "short", "long", "str", "none", "scalar", "nested"])
        p = {"nan": [math.nan, 0.1, 0.8], "inf": [0.3, rng.choice([math.inf, -math.inf]), 0.8], "short": [0.3, 0.1],
             "long": [0.3, 0.1, 0.8, 0.0], "str": "abc", "none": None, "scalar": 5, "nested": [[0.3], [0.1], [0.8]]}[bad]
    elif which == "side":
        side = rng.choice(["middle", "", None, 5, "LEFT", "left "])
    elif which == "dur":
        dur = rng.choice([math.nan, math.inf, -math.inf, -1.0, -1e-9, "1"])
    else:
        gy = rng.choice([math.nan, math.inf, -math.inf])
    return side, p, dur, gy


class DemBuoc:
    """Bọc robot.step đếm số chu kỳ; vượt trần thì ném Het (chạy vô hạn)."""

    def __init__(self, r: Robot, tran: int = 20000):
        self.r, self.tran, self.n = r, tran, 0
        self._step = r.step

    def __enter__(self):
        def step(dt=None):
            self.n += 1
            if self.n > self.tran:
                raise Het(f"quá {self.tran} chu kỳ step")
            return self._step(dt)
        self.r.step = step
        return self

    def __exit__(self, *a):
        self.r.step = self._step


def test_r28_move_tcp_dau_vao_hong():
    r = make_robot("sim")
    r.kind = "sim"
    rng = random.Random(SEED + 28)
    bao = Bao("R-28")
    for i in range(N):
        ca = gen_tcp(rng)
        truoc = snap(r)
        try:
            with DemBuoc(r) as d, han(5):
                r.move_tcp(ca[0], ca[1], duration=ca[2], grip_yaw=ca[3])
            loi, buoc = None, d.n
        except BaseException as e:   # noqa: BLE001
            loi, buoc = e, d.n
        if loi is None:
            bao.them(i, ca, "đầu vào hỏng được nhận, không báo lỗi")
        elif not (isinstance(loi, (ValueError, TypeError)) and ro(loi)):
            bao.them(i, ca, f"cần ValueError/TypeError có câu rõ, nhận {type(loi).__name__}: {loi}")
        elif buoc or not giong(truoc, snap(r)):
            bao.them(i, ca, f"lỗi ném sau khi đã chạy {buoc} chu kỳ hoặc đã đổi lệnh")
        if huu_han(r):
            bao.them(i, ca, f"trạng thái không hữu hạn: {huu_han(r)}")
            break
    bao.kiem()


def test_r28_duration_rat_lon_bi_chan_tran():
    r = make_robot("sim")
    r.kind = "sim"
    for dur in (1e6, 1e9):
        try:
            with DemBuoc(r, tran=6000) as d, han(10):
                r.move_tcp("left", P_XA, duration=dur)
        except ValueError:
            continue          # từ chối cũng được
        except Het as e:
            pytest.fail(f"duration={dur}: không có trần ({e})")
        assert d.n <= 6000


# ------------------------------------------------------------------ R-29 đế
def gen_vel(rng: random.Random):
    def one():
        u = rng.random()
        if u < 0.4:
            return rng.choice(SPECIAL)
        if u < 0.5:
            return rng.choice(["x", None, [0.1], b"1"])
        return rng.uniform(-3, 3)
    return one(), one(), one()


def _vel_xau(v) -> bool:
    return not isinstance(v, (int, float)) or not math.isfinite(v)


def test_r29_set_base_velocity(robot):
    rng = random.Random(SEED + 29)
    bao = Bao("R-29")
    for i in range(N):
        ca = gen_vel(rng)
        xau = any(_vel_xau(v) for v in ca)
        truoc = snap(robot)
        try:
            robot.set_base_velocity(*ca)
            loi = None
        except BaseException as e:   # noqa: BLE001
            loi = e
        if loi is not None and not isinstance(loi, ALLOWED):
            bao.them(i, ca, f"ngoại lệ thô {type(loi).__name__}: {loi}")
        elif xau and not (isinstance(loi, (ValueError, TypeError)) and ro(loi)):
            bao.them(i, ca, f"cần ValueError/TypeError có câu rõ, nhận {type(loi).__name__ if loi else 'không lỗi'}: {loi}")
        elif loi is not None and not giong(truoc, snap(robot)):
            bao.them(i, ca, "lệnh bị từ chối nhưng đích đã đổi")
        if not np.all(np.isfinite(robot.backend.base_ramp.goal)):
            bao.them(i, ca, f"đích dốc không hữu hạn {robot.backend.base_ramp.goal}")
            robot.backend.base_ramp.reset()
        try:
            robot.step()
        except BaseException as e:   # noqa: BLE001
            bao.them(i, ca, f"step() sau lệnh ném {type(e).__name__}: {e}")
            robot.backend.base_ramp.reset()
        v = robot.backend.base_velocity()
        if not np.all(np.isfinite(v)) or math.hypot(v[0], v[1]) > spec.BASE_MAX_V + 1e-9:
            bao.them(i, ca, f"vận tốc đế sau step không hữu hạn hoặc vượt trần: {v}")
            robot.backend.base_ramp.reset()
            if robot.kind == "sim":
                break
    bao.kiem()


def test_r29_base_ramp_tu_chon_lop_hai():
    for v in ((math.nan, 0, 0), (0, math.inf, 0), (0, 0, -math.inf), (1e308, 1e308, 0)):
        ramp = limits.BaseRamp()
        try:
            ramp.set_goal(*v)
        except ValueError:
            pass
        ramp.step(0.02)
        assert np.all(np.isfinite(ramp.cur)) and np.all(np.isfinite(ramp.goal)), (v, ramp.cur, ramp.goal)
        if not all(math.isfinite(x) for x in v):
            with pytest.raises(ValueError):
                limits.BaseRamp().set_goal(*v)


def test_r29_move_base_dau_vao_hong():
    r = make_robot("sim")
    r.kind = "sim"
    ca = [(math.inf, 0, 0, 2.0), (math.nan, 0, 0, 2.0), (0, -math.inf, 0, 2.0), (0, 0, math.nan, 2.0), (0.1, 0, 0, math.nan),
          (0.1, 0, 0, math.inf), (0.1, 0, 0, -1.0), ("x", 0, 0, 1.0)]
    bao = Bao("R-29 move_base")
    for i, (dx, dy, dyaw, dur) in enumerate(ca):
        try:
            with DemBuoc(r, tran=5000) as d, han(5):
                r.move_base(dx, dy, dyaw, duration=dur)
            loi = None
        except BaseException as e:   # noqa: BLE001
            loi = e
        if not (isinstance(loi, (ValueError, TypeError)) and ro(loi)):
            bao.them(i, ca[i], f"cần ValueError/TypeError có câu rõ, nhận {type(loi).__name__ if loi else 'không lỗi'}: {loi}")
        if huu_han(r):
            bao.them(i, ca[i], f"trạng thái không hữu hạn: {huu_han(r)}")
            break
    assert not bao.loi, "\n".join(bao.loi)


# ------------------------------------------------------------------ R-30 tần số rất cao
def test_r30_10000_lenh_khop_khong_step_nhanh_va_huu_han(robot):
    """10.000 lệnh khớp không step: mỗi lệnh p99 <= 2 ms (sim), trạng thái hữu hạn."""
    rng = random.Random(SEED + 30)
    t = []
    for _ in range(10000):
        v = rng.uniform(-0.3, 0.3)
        t0 = time.perf_counter()
        robot.set_joint_positions({"left_pan": v})
        t.append(time.perf_counter() - t0)
    p99 = float(np.percentile(t, 99))
    assert p99 <= 0.002, f"p99 {p99 * 1e3:.2f} ms"
    assert huu_han(robot) is None


def test_r30_1000_set_base_velocity_moi_chu_ky_khong_lam_doc_nhanh_hon(robot):
    t = []
    for _ in range(1000):
        t0 = time.perf_counter()
        robot.set_base_velocity(spec.BASE_MAX_V, 0.0, 0.0)
        t.append(time.perf_counter() - t0)
    assert float(np.percentile(t, 99)) <= 0.002
    robot.step()
    v = robot.backend.base_velocity()
    assert v[0] <= spec.BASE_MAX_ACC * robot.dt * 1.01 + 1e-9, v        # một chu kỳ chỉ tăng ACC*dt dù đặt đích 1000 lần
    robot.set_base_velocity(0, 0, 0)


@pytest.mark.parametrize("dt", [0, 0.0, -1.0, -1e-9, math.nan, math.inf, -math.inf])
def test_r30_step_dt_khong_hop_le(robot, dt):
    truoc = snap(robot)
    with pytest.raises(ValueError) as e:
        robot.step(dt)
    assert ro(e.value) and giong(truoc, snap(robot))


def test_r30_step_dt_lon_bi_chan_tran(robot):
    if robot.kind == "sim":
        t0 = robot.backend.data.time
        try:
            robot.step(10.0)
        except ValueError:
            return
        assert robot.backend.data.time - t0 <= 10 * robot.dt * 1.01
    else:
        robot.set_base_velocity(spec.BASE_MAX_V, 0, 0)
        try:
            robot.step(10.0)
        except ValueError:
            return
        assert robot.backend.base_velocity()[0] <= spec.BASE_MAX_ACC * 10 * robot.dt * 1.01 + 1e-9


@pytest.mark.parametrize("dt", [1e-9, 1e-6])
def test_r30_step_dt_rat_nho_van_huu_han(robot, dt):
    robot.step(dt)
    assert huu_han(robot) is None


# ------------------------------------------------------------------ R-31 giao thức servo
def gen_bytes(rng: random.Random) -> bytes:
    u = rng.random()
    if u < 0.3:                                   # ngẫu nhiên thuần, thiên về 0xFF
        return bytes(rng.choice([0xFF, 0x00, rng.randrange(256)]) for _ in range(rng.randint(0, 300)))
    sid, ins = rng.randrange(256), rng.choice([P.PING, P.READ, P.WRITE, P.SYNC_WRITE, P.SYNC_READ, 0, 0xFF, rng.randrange(256)])
    pr = bytes(rng.randrange(256) for _ in range(rng.randint(0, 12)))
    try:
        pkt = bytearray(P.make_packet(sid, ins, pr))
    except ValueError:
        pkt = bytearray(b"\xff\xff\x01\x02\x01\xfb")
    for _ in range(rng.randint(0, 3)):            # biến đổi gói hợp lệ
        m = rng.choice(["flip", "cut", "junk", "len", "dup"])
        if m == "flip" and pkt:
            pkt[rng.randrange(len(pkt))] ^= 1 << rng.randrange(8)
        elif m == "cut" and pkt:
            del pkt[rng.randrange(len(pkt)):]
        elif m == "junk":
            pkt[:0] = bytes(rng.choice([0xFF, 0, rng.randrange(256)]) for _ in range(rng.randint(1, 5)))
        elif m == "len" and len(pkt) > 3:
            pkt[3] = rng.randrange(256)
        elif m == "dup":
            pkt = pkt + bytearray(pkt)
    return bytes(pkt)


def test_r31_parse_packet_chi_nem_protocol_error():
    rng = random.Random(SEED + 31)
    bao = Bao("R-31 parse_packet")
    for i in range(N):
        b = gen_bytes(rng)
        t0 = time.perf_counter()
        try:
            sid, ins, params, used = P.parse_packet(b)
            if not (0 < used <= len(b)) or len(params) > len(b):
                bao.them(i, b.hex(), f"used={used} ngoài khoảng")
        except P.ProtocolError:
            pass
        except BaseException as e:   # noqa: BLE001
            bao.them(i, b.hex(), f"{type(e).__name__}: {e}")
        if time.perf_counter() - t0 > 0.05:
            bao.them(i, b.hex(), "quá 50 ms")
    bao.kiem()


class PortKichBan:
    """Cổng giả trả byte theo kịch bản (hỗn độn), để fuzz FeetechBus._recv / sync_read."""

    def __init__(self, chunks):
        self.chunks, self.sent = list(chunks), []

    def write(self, data):
        self.sent.append(bytes(data))
        return len(data)

    def read(self, n=1):
        return self.chunks.pop(0) if self.chunks else b""

    def reset_input_buffer(self):
        pass

    def close(self):
        pass


def gen_reply(rng: random.Random, sid: int) -> list[bytes]:
    """Vài mẩu trả lời: gói hợp lệ checksum đúng nhưng ID/ERR/độ dài params tuỳ ý, hoặc byte hỗn độn."""
    out = []
    for _ in range(rng.randint(0, 4)):
        if rng.random() < 0.5:
            out.append(P.make_packet(rng.choice([sid, sid, rng.randrange(255)]), rng.choice([0, 0, 0, 1 << 5, rng.randrange(256)]),
                                     bytes(rng.randrange(256) for _ in range(rng.randint(0, 6)))))
        else:
            out.append(gen_bytes(rng))
    return out


def test_r31_bus_recv_va_sync_read_chi_nem_loi_giao_thuc():
    rng = random.Random(SEED + 311)
    bao = Bao("R-31 bus")
    cho_phep = (P.ProtocolError, TimeoutError)
    for i in range(N):
        sid = rng.choice(ALL_IDS)
        kind = rng.choice(["read1", "read2", "sync1", "sync2", "ping"])
        chunks = gen_reply(rng, sid)
        bus = FeetechBus(PortKichBan(chunks), timeout=0.001, retries=0)
        t0 = time.perf_counter()
        try:
            if kind == "read1":
                bus.read(sid, P.PRESENT_TEMPERATURE)
            elif kind == "read2":
                bus.read(sid, P.PRESENT_POSITION)
            elif kind == "sync1":
                bus.sync_read(P.PRESENT_TEMPERATURE, [sid, ALL_IDS[0]])
            elif kind == "sync2":
                bus.sync_read(P.PRESENT_POSITION, [sid, ALL_IDS[1]])
            else:
                bus.ping(sid)
        except cho_phep:
            pass
        except BaseException as e:   # noqa: BLE001
            bao.them(i, (kind, sid, [c.hex() for c in chunks]), f"{type(e).__name__}: {e}")
        if time.perf_counter() - t0 > 0.05:
            bao.them(i, (kind, sid), "quá 50 ms")
    bao.kiem()


def test_r31_fakeservoport_write_khong_ne_va_khong_treo():
    rng = random.Random(SEED + 312)
    bao = Bao("R-31 FakeServoPort")
    port = FakeServoPort(ALL_IDS)
    for i in range(N):
        b = gen_bytes(rng)
        t0 = time.perf_counter()
        try:
            port.write(b)
            port.read(4096)
            port.reset_input_buffer()
        except BaseException as e:   # noqa: BLE001
            bao.them(i, b.hex(), f"{type(e).__name__}: {e}")
            port.rx = b""
        if time.perf_counter() - t0 > 0.05:
            bao.them(i, b.hex(), "quá 50 ms")
    bao.kiem()
    assert all(len(m) == 256 for m in port.mem.values())


# ------------------------------------------------------------------ R-32 hiệu chuẩn, ID, biến môi trường
DEL = object()


def _tt_so(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def cal_hop_le(c) -> str | None:
    """None nếu hiệu chuẩn hợp lệ cho P-L; ngược lại lý do (đây là mệnh đề đúng/sai, không phải mã của SDK)."""
    ref = default_calibration()
    if not isinstance(c, dict):
        return "không phải dict"
    if set(c) != set(ref):
        return f"khớp thiếu/lạ: {sorted(set(c) ^ set(ref))}"
    for n, f in c.items():
        if not isinstance(f, dict) or set(f) != set(ref[n]):
            return f"{n}: trường thiếu/lạ"
        if not all(_tt_so(v) for v in f.values()):
            return f"{n}: giá trị không phải số hữu hạn"
        if n == "lift":
            if f["counts_per_m"] == 0:
                return "lift.counts_per_m = 0"
        elif n.endswith("gripper"):
            if f["open"] == f["closed"] or not (0 <= f["open"] <= 4095 and 0 <= f["closed"] <= 4095):
                return f"{n}: open == closed hoặc ngoài 0..4095"
        else:
            if f["sign"] not in (1, -1):
                return f"{n}.sign ∉ ±1"
            if "zero" in f and not 0 <= f["zero"] <= 4095:
                return f"{n}.zero ngoài 0..4095"
    return None


def mutate_cal(rng: random.Random) -> dict:
    import copy
    c = copy.deepcopy(default_calibration())
    for _ in range(rng.choice([0, 1, 1, 1, 2, 3])):
        n = rng.choice(list(c))
        op = rng.choice(["set", "set", "del_field", "del_joint", "add_joint", "type"])
        if op == "del_joint":
            c.pop(n, None)
        elif op == "add_joint":
            c[f"khop_la_{rng.randrange(9)}"] = {"zero": 2048, "sign": 1}
        elif n in c and c[n]:
            f = rng.choice(list(c[n]))
            if op == "del_field":
                c[n].pop(f, None)
            elif op == "type":
                c[n][f] = rng.choice(["x", None, [1], {"a": 1}, True])
            else:
                c[n][f] = rng.choice([0, -1, 1, 2, 4095, 4096, 5000, math.nan, math.inf, -math.inf, 1e308, 0.0, 2048, 1.5])
    return c


def test_r32_hieu_chuan_ngau_nhien():
    rng = random.Random(SEED + 321)
    bao = Bao("R-32 hiệu chuẩn")
    for i in range(N):
        c = mutate_cal(rng)
        ly_do = cal_hop_le(c)
        try:
            be = FeetechBackend(port=FakeServoPort(ALL_IDS), calibration=c, allow_no_latch=True, **NOSLEEP)
            loi = None
        except BaseException as e:   # noqa: BLE001
            be, loi = None, e
        if ly_do is None:
            if loi is not None:
                bao.them(i, "hợp lệ", f"cấu hình đúng bị từ chối: {type(loi).__name__}: {loi}")
            else:
                be.close()
        else:
            if loi is None:
                bao.them(i, ly_do, "cấu hình sai mà backend vẫn tạo được")
            elif not (isinstance(loi, ValueError) and ro(loi)):
                bao.them(i, ly_do, f"cần ValueError có câu rõ, nhận {type(loi).__name__}: {loi}")
    bao.kiem()


def mutate_ids(rng: random.Random) -> tuple[dict, str]:
    ids = dict(spec.SERVO_IDS)
    a, b = rng.sample(list(ids), 2)
    op = rng.choice(["dup", "range", "type", "missing", "extra"])
    if op == "dup":
        ids[a] = ids[b]
    elif op == "range":
        ids[a] = rng.choice([-1, 254, 255, 300, 2 ** 40, 0])
    elif op == "type":
        ids[a] = rng.choice([1.5, "3", None, math.nan, True])
    elif op == "missing":
        ids.pop(a)
    else:
        ids["khop_la"] = rng.randrange(40, 99)
    return ids, f"{op}:{a}"


def test_r32_bang_id_ngau_nhien():
    rng = random.Random(SEED + 322)
    bao = Bao("R-32 ids")
    for i in range(N):
        ids, mo_ta = mutate_ids(rng)
        try:
            be = FeetechBackend(port=FakeServoPort(ALL_IDS), ids=ids, allow_no_latch=True, **NOSLEEP)
            loi = None
        except BaseException as e:   # noqa: BLE001
            be, loi = None, e
        if loi is None:
            bao.them(i, mo_ta, "bảng ID sai mà backend vẫn tạo được")
            be.close()
        elif not (isinstance(loi, (ValueError, ConnectionError)) and ro(loi)):
            bao.them(i, mo_ta, f"cần ValueError/ConnectionError có câu rõ, nhận {type(loi).__name__}: {loi}")
    bao.kiem()


GPIO_ALPHABET = list("K5_EN=,;: 0123456789abcXYZ\n\t-.") + ["\x00", "ESTOP_OK", "ARM_SENSE", "RESET_SENSE", "DOCK_L=", "LIFT_LATCH_EN=", "=="]


def test_r32_gpio_pins_chuoi_hong_chi_nem_loi_cau_hinh():
    rng = random.Random(SEED + 323)
    bao = Bao("R-32 FAIBOTIC_GPIO_PINS")
    for i in range(N):
        s = "".join(rng.choice(GPIO_ALPHABET) for _ in range(rng.randint(0, 40)))
        env = {"FAIBOTIC_GPIO_PINS": s}
        if rng.random() < 0.3:
            env["FAIBOTIC_GPIO_ACTIVE_LOW"] = "".join(rng.choice(GPIO_ALPHABET) for _ in range(rng.randint(0, 12)))
        if rng.random() < 0.3:
            env["FAIBOTIC_GPIO_PULL"] = "".join(rng.choice(GPIO_ALPHABET) for _ in range(rng.randint(0, 12)))
        try:
            cfg = config_from_env(env)
            assert cfg is None or isinstance(cfg["pins"], dict)
            if not os.environ.get("FAIBOTIC_FUZZ_GPIO_REAL") and cfg is not None and not (
                    importlib.util.find_spec("lgpio") or importlib.util.find_spec("gpiod")):
                with pytest.raises((GPIOConfigError, GPIOUnavailable)):
                    PiGPIO(**cfg)
        except (GPIOConfigError, GPIOUnavailable):
            pass
        except BaseException as e:   # noqa: BLE001
            bao.them(i, env, f"{type(e).__name__}: {e}")
    bao.kiem()


# ------------------------------------------------------------------ R-33 CLI
CLI_LENH = ["ports", "model", "scan", "status", "doctor", "--version", "--help", "-h", "", "xyz", "--json", "sim-", "lab", "labx"]
CLI_PORT = ["mock", "/dev/ttyNOPE_faibotic", "COM99", "x", "-1", "mock ", "MOCK", "\x00", "/dev/ttyUSB999"]
CLI_NHIEU = ["--json", "--port", "--bogus", "-x", "--", "=", "--port=", "--port=mock", "--full"]


def gen_argv(rng: random.Random) -> list[str]:
    a = [rng.choice(CLI_LENH)]
    if rng.random() < 0.8:
        a += ["--port", rng.choice(CLI_PORT)]
    for _ in range(rng.randint(0, 2)):
        a.insert(rng.randint(0, len(a)), rng.choice(CLI_NHIEU))
    if rng.random() < 0.15:
        rng.shuffle(a)
    return a


def test_r33_cli_argv_ngau_nhien(capsys):
    rng = random.Random(SEED + 33)
    bao = Bao("R-33")
    for i in range(N):
        argv = gen_argv(rng)
        if argv and argv[0] in ("sim", "selftest", "demo", "lab"):
            continue           # sinh tiến trình con, ngoài phạm vi fuzz
        try:
            with han(10):
                ma = cli_main(list(argv))
        except SystemExit as e:
            ma = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
        except BaseException as e:   # noqa: BLE001
            bao.them(i, argv, f"ngoại lệ thô {type(e).__name__}: {e}")
            capsys.readouterr()
            continue
        out, err = capsys.readouterr()
        if ma not in (0, 1, 2):
            bao.them(i, argv, f"mã thoát {ma!r}")
        if "Traceback" in err or "Traceback" in out:
            bao.them(i, argv, "có traceback")
        if ma in (1, 2) and "scan" in argv and not (out.strip() or err.strip()):
            bao.them(i, argv, "lỗi mà không có câu báo")
    bao.kiem()
