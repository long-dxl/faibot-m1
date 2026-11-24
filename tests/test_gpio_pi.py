"""Lớp GPIO thật Pi 5 (faibotic.gpio_pi) với thư viện lgpio/gpiod giả: chạy được trên máy không có GPIO."""
import sys
import threading
import types

import pytest

from faibotic import gpio_pi, limits
from faibotic.gpio_pi import GPIOConfigError, GPIOUnavailable, PiGPIO, PiLiftSwitches
from faibotic.safety import BangDiDayLatch, ResetRefused, SafetyLatch

PINS = {"K5_EN": 22, "ESTOP_OK": 17, "ARM_SENSE": 27, "RESET_SENSE": 23, "LIFT_BOT": 5, "LIFT_TOP": 6}


class FakeLgpio(types.ModuleType):
    """lgpio giả: chip theo nhãn, mức chân đặt từ test, ghi lại mọi lệnh."""
    SET_PULL_NONE, SET_PULL_UP, SET_PULL_DOWN = 0, 32, 64

    def __init__(self, labels=None):
        super().__init__("lgpio")
        self.labels = {0: "pinctrl-rp1"} if labels is None else labels
        self.level: dict[int, int] = {}
        self.claims: dict[int, tuple] = {}
        self.writes: list[tuple[int, int]] = []
        self.open_handles: set[int] = set()
        self.fail_read = False

    def gpiochip_open(self, n):
        if n not in self.labels:
            raise RuntimeError("no chip")
        self.open_handles.add(100 + n)
        return 100 + n

    def gpiochip_close(self, h):
        self.open_handles.discard(h)

    def gpio_get_chip_info(self, h):
        return (0, 54, f"gpiochip{h - 100}", self.labels[h - 100])

    def gpio_claim_output(self, h, off, level=0, flags=0):
        self.claims[off] = ("out", h, flags)
        self.level[off] = level
        self.writes.append((off, level))

    def gpio_claim_input(self, h, off, flags=0):
        self.claims[off] = ("in", h, flags)
        self.level.setdefault(off, 0)

    def gpio_read(self, h, off):
        if self.fail_read:
            raise RuntimeError("bad read")
        return self.level[off]

    def gpio_write(self, h, off, v):
        assert self.claims[off][0] == "out", "ghi vào ngõ vào"
        self.level[off] = v
        self.writes.append((off, v))

    def gpio_free(self, h, off):
        self.claims.pop(off, None)


@pytest.fixture
def lg(monkeypatch):
    m = FakeLgpio()
    monkeypatch.setitem(sys.modules, "lgpio", m)
    return m


def make(**kw):
    kw.setdefault("pins", PINS)
    return PiGPIO(backend="lgpio", **kw)


def test_import_without_hardware(monkeypatch):
    """Import module không cần thư viện GPIO; tạo PiGPIO khi thiếu thư viện thì báo GPIOUnavailable."""
    monkeypatch.setitem(sys.modules, "lgpio", None)
    monkeypatch.setitem(sys.modules, "gpiod", None)
    with pytest.raises(GPIOUnavailable, match="lgpio.*gpiod"):
        PiGPIO(PINS)


def test_output_starts_low_and_inputs_pull(lg):
    g = make(pull={"LIFT_BOT": "up", "LIFT_TOP": "up"})
    assert g.backend == "lgpio" and g.chip == 0 and g.chip_label == "pinctrl-rp1"
    assert lg.claims[22][0] == "out" and lg.writes[0] == (22, 0)          # K5 hở ngay khi giữ chân
    assert lg.claims[5] == ("in", 100, FakeLgpio.SET_PULL_UP)
    assert lg.claims[17] == ("in", 100, FakeLgpio.SET_PULL_NONE)
    g.write("K5_EN", True)
    assert lg.level[22] == 1
    with pytest.raises(KeyError):
        g.write("ESTOP_OK", True)                                          # ngõ vào không ghi được


def test_active_low_inputs_inverted(lg):
    g = make(active_low=("ESTOP_OK", "ARM_SENSE"))
    lg.level[17] = 0
    lg.level[27] = 1
    assert g.read("ESTOP_OK") is True and g.read("ARM_SENSE") is False
    lg.level[23] = 1
    assert g.read("RESET_SENSE") is True                                   # không khai active_low: mức thô


@pytest.mark.parametrize("kw, msg", [
    (dict(pins={"K5_EN": 22, "ESTOP_OK": 17}), "thiếu chân"),
    (dict(pins={**PINS, "ARM_SENSE": 22}), "trùng"),
    (dict(pins={**PINS, "K1_AUX": 40}), "ngoài header"),
    (dict(active_low=("K5_EN",)), "tích cực mức thấp"),
    (dict(pull={"K5_EN": "up"}), "pull sai"),
    (dict(pull={"LIFT_BOT": "sideways"}), "pull sai"),
    (dict(active_low=("K1_AUX",)), "không khai"),
    (dict(hw=None), "hw"),
])
def test_config_errors(lg, kw, msg):
    with pytest.raises(GPIOConfigError, match=msg):
        make(**kw)
    assert not lg.claims, "khai sai thì không được giữ chân nào"


def test_chip_autodetect_prefers_rp1(monkeypatch):
    m = FakeLgpio(labels={0: "pinctrl-bcm2712", 4: "pinctrl-rp1"})          # kernel cũ Pi 5: RP1 ở gpiochip4
    monkeypatch.setitem(sys.modules, "lgpio", m)
    g = make()
    assert g.chip == 4 and m.open_handles == {104}


def test_chip_not_found(monkeypatch):
    monkeypatch.setitem(sys.modules, "lgpio", FakeLgpio(labels={0: "some-other-chip"}))
    monkeypatch.setitem(sys.modules, "gpiod", None)
    with pytest.raises(GPIOUnavailable, match="không thấy chip"):
        PiGPIO(PINS)


def test_close_drives_outputs_low_and_releases(lg):
    g = make()
    g.write("K5_EN", True)
    g.close()
    assert lg.level[22] == 0 and not lg.claims and not lg.open_handles
    g.close()                                                               # gọi lại không lỗi
    with pytest.raises(OSError):
        g.read("ESTOP_OK")


def test_lift_switches_nc_wiring(lg):
    """NC nối GND + pull up: nhả = 0, chạm (hoặc đứt dây) = 1 -> lift_gate chặn đúng chiều."""
    g = make(pull={"LIFT_BOT": "up", "LIFT_TOP": "up"})
    sw = gpio_pi.lift_switches_for(g)
    assert isinstance(sw, PiLiftSwitches) and sw.read() == (False, False)
    lg.level[5] = 1
    assert sw.read() == (True, False)
    assert limits.lift_gate(0.25, 0.24, sw) == 0.25 and limits.lift_gate(0.25, 0.26, sw) == 0.26
    lg.level[6] = 1                                                         # đứt cả hai dây: chặn cả hai chiều
    assert limits.lift_gate(0.25, 0.24, sw) == 0.25 and limits.lift_gate(0.25, 0.26, sw) == 0.25


def test_lift_switches_missing_pins(lg):
    g = make(pins={k: v for k, v in PINS.items() if not k.startswith("LIFT")})
    assert gpio_pi.lift_switches_for(g) is None
    with pytest.raises(GPIOConfigError):
        PiLiftSwitches(g)


def test_from_env(lg):
    assert gpio_pi.from_env({}) is None                                     # không khai: không có GPIO -> backend từ chối enable
    env = {"FAIBOTIC_GPIO_PINS": "K5_EN=22, ESTOP_OK=17,ARM_SENSE=27,RESET_SENSE=23,K1_AUX=24",
           "FAIBOTIC_GPIO_ACTIVE_LOW": "ESTOP_OK,ARM_SENSE", "FAIBOTIC_GPIO_PULL": "K1_AUX=down",
           "FAIBOTIC_GPIO_BACKEND": "lgpio", "FAIBOTIC_SERVO_DISCHARGE_MS": "300"}
    g = gpio_pi.from_env(env)
    assert g.hw == "bang_di_day" and g.pins["K1_AUX"] == 24 and lg.claims[24][2] == FakeLgpio.SET_PULL_DOWN
    assert gpio_pi.latch_kwargs(g, env) == {"k1_aux_pin": "K1_AUX", "servo_discharge_s": 0.3}
    with pytest.raises(GPIOConfigError):
        gpio_pi.from_env({"FAIBOTIC_GPIO_PINS": "K5_EN=abc"})
    with pytest.raises(GPIOConfigError, match="thiếu chân"):
        gpio_pi.from_env({"FAIBOTIC_GPIO_PINS": "K5_EN=22", "FAIBOTIC_GPIO_BACKEND": "lgpio"})


def test_latch_refuses_without_discharge_or_aux(lg):
    """Mặc định an toàn giữ nguyên: GPIO thật, không K1_AUX, chưa đo xả tụ -> SafetyLatch từ chối reset, K5 vẫn hạ."""
    g = make()
    lt = SafetyLatch(g, **gpio_pi.latch_kwargs(g, {}))
    assert isinstance(lt, BangDiDayLatch) and lg.level[22] == 0
    with pytest.raises(ResetRefused, match="xả tụ"):
        lt.reset(timeout_s=0.01)
    assert lg.level[22] == 0


def test_read_error_trips_monitor(lg):
    """Đọc GPIO lỗi trong luồng giám sát -> trip: K5 hạ."""
    g = make()
    stopped = threading.Event()
    with pytest.warns(RuntimeWarning):
        lt = SafetyLatch(g, on_stop=stopped.set, bench=True)
    lt._en = True
    g.write("K5_EN", True)
    lg.fail_read = True
    lt.start_monitor(0.001)
    assert stopped.wait(2.0)
    assert lg.level[22] == 0 and "lỗi" in (lt.monitor_error or "")


class FakeGpiodModule(types.ModuleType):
    pass


def _fake_gpiod(labels):
    gd = FakeGpiodModule("gpiod")
    line = types.ModuleType("gpiod.line")

    class _E:
        def __init__(self, *names):
            for n in names:
                setattr(self, n, n)
    line.Direction = _E("INPUT", "OUTPUT")
    line.Value = _E("ACTIVE", "INACTIVE")
    line.Bias = _E("PULL_UP", "PULL_DOWN")
    gd.line = line
    state = {"vals": {}, "cfg": None, "released": False}

    class LineSettings(dict):
        def __init__(self, **kw):
            super().__init__(kw)
    gd.LineSettings = LineSettings

    class Chip:
        def __init__(self, path):
            n = int(path.replace("/dev/gpiochip", ""))
            if n not in labels:
                raise OSError("no chip")
            self.label = labels[n]

        def __enter__(self):
            return self

        def __exit__(self, *a):
            pass

        def get_info(self):
            return types.SimpleNamespace(label=self.label)
    gd.Chip = Chip

    class Req:
        def get_value(self, off):
            return state["vals"].get(off, "INACTIVE")

        def set_value(self, off, v):
            state["vals"][off] = v

        def release(self):
            state["released"] = True

    def request_lines(path, consumer, config):
        state["path"], state["cfg"] = path, config
        for off, st in config.items():
            if st.get("direction") == "OUTPUT":
                state["vals"][off] = st["output_value"]
        return Req()
    gd.request_lines = request_lines
    return gd, line, state


def test_gpiod_backend(monkeypatch):
    gd, line, st = _fake_gpiod({0: "pinctrl-bcm2712", 4: "pinctrl-rp1"})
    monkeypatch.setitem(sys.modules, "gpiod", gd)
    monkeypatch.setitem(sys.modules, "gpiod.line", line)
    g = PiGPIO(PINS, backend="gpiod", pull={"LIFT_TOP": "up"}, active_low=("ARM_SENSE",))
    assert g.backend == "gpiod" and st["path"] == "/dev/gpiochip4"
    assert st["cfg"][22]["output_value"] == "INACTIVE" and st["cfg"][6]["bias"] == "PULL_UP"
    st["vals"][27] = "INACTIVE"
    assert g.read("ARM_SENSE") is True
    g.write("K5_EN", True)
    assert st["vals"][22] == "ACTIVE"
    g.close()
    assert st["vals"][22] == "INACTIVE" and st["released"]


def test_auto_falls_back_to_gpiod(monkeypatch):
    gd, line, st = _fake_gpiod({0: "pinctrl-rp1"})
    monkeypatch.setitem(sys.modules, "lgpio", None)
    monkeypatch.setitem(sys.modules, "gpiod", gd)
    monkeypatch.setitem(sys.modules, "gpiod.line", line)
    assert PiGPIO(PINS).backend == "gpiod"


def test_evtlib_pigpio_wraps_sdk(lg):
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1] / "product" / "evt"))
    import evtlib
    g = evtlib.PiGPIO({"LIFT_BOT": 5, "LIFT_TOP": 6}, outputs=("K5_EN",), hw="bang_di_day", backend="lgpio")
    assert g.outputs == () and g.read("LIFT_BOT") is False                 # kịch bản đo một phần: không bắt đủ chân


def test_feetech_default_gpio_none_refuses_enable(monkeypatch):
    """Không khai FAIBOTIC_GPIO_PINS: default_gpio() None -> FeetechBackend không có chốt -> enable() bị từ chối."""
    from faibotic.backends import feetech
    monkeypatch.delenv("FAIBOTIC_GPIO_PINS", raising=False)
    assert feetech.default_gpio() is None


def test_feetech_uses_env_gpio_for_latch_and_lift(monkeypatch, lg):
    """Khai FAIBOTIC_GPIO_PINS: backend mở PiGPIO, có SafetyLatch bộ #0 (K5 hạ, chưa reset -> enable bị từ chối),
    lấy công tắc hành trình từ LIFT_BOT/LIFT_TOP; close() hạ K5 và nhả chân."""
    from faibotic import spec
    from faibotic.backends.feetech import EStopActive, FeetechBackend
    from faibotic.feetech.mockbus import FakeServoPort
    monkeypatch.setenv("FAIBOTIC_GPIO_PINS", ",".join(f"{k}={v}" for k, v in PINS.items()))
    monkeypatch.setenv("FAIBOTIC_GPIO_BACKEND", "lgpio")
    monkeypatch.setenv("FAIBOTIC_GPIO_PULL", "LIFT_BOT=up,LIFT_TOP=up")
    be = FeetechBackend(port=FakeServoPort(list(spec.SERVO_IDS.values())), sleep=lambda s: None)
    try:
        assert isinstance(be.latch, BangDiDayLatch) and lg.level[22] == 0
        assert isinstance(be.lift_switches, PiLiftSwitches)
        with pytest.raises(EStopActive):
            be.enable()
    finally:
        be.close()
    assert lg.level[22] == 0 and not lg.claims


# ---------- vòng sửa QC2-dk-dien-sdk (M1, M2, M3, M5, m1, m11) ----------
@pytest.mark.parametrize("raw", ["0", "nan", "-5", "0.3", "0,3", "abc", "inf", "6000", "49"])
def test_discharge_ms_invalid_refuses_arm(lg, raw):
    """M2: số xả tụ sai (0, nan, âm, nhầm giây, chữ, ngoài 50–5000 ms) -> config_problem từ chối, K5 giữ hạ."""
    g = make()
    kw = gpio_pi.latch_kwargs(g, {"FAIBOTIC_SERVO_DISCHARGE_MS": raw})
    assert "servo_discharge_s" not in kw and "FAIBOTIC_SERVO_DISCHARGE_MS" in kw["config_error"]
    lt = SafetyLatch(g, **kw)
    assert lt.config_problem() == kw["config_error"]
    with pytest.raises(ResetRefused, match="FAIBOTIC_SERVO_DISCHARGE_MS"):
        lt.reset(timeout_s=0.01)
    assert lg.level[22] == 0


@pytest.mark.parametrize("raw, s", [("300", 0.3), (" 50 ", 0.05), ("5000", 5.0)])
def test_discharge_ms_valid(lg, raw, s):
    kw = gpio_pi.latch_kwargs(make(), {"FAIBOTIC_SERVO_DISCHARGE_MS": raw})
    assert kw == {"servo_discharge_s": pytest.approx(s)}


@pytest.mark.parametrize("sd", [0.0, -0.1, float("nan"), float("inf"), 6.0])
def test_config_problem_rejects_bad_discharge_direct(lg, sd):
    """M2: truyền thẳng servo_discharge_s sai vào SafetyLatch cũng bị từ chối (không chỉ qua biến môi trường)."""
    lt = SafetyLatch(make(), servo_discharge_s=sd)
    assert "không hợp lệ" in lt.config_problem()


def test_lift_default_pull_up_without_config(lg):
    """M3: quên FAIBOTIC_GPIO_PULL -> LIFT_* vẫn kéo lên (NC đứt dây đọc 'chạm'); khai 'none' rõ ràng thì tôn trọng."""
    make()
    assert lg.claims[5][2] == FakeLgpio.SET_PULL_UP and lg.claims[6][2] == FakeLgpio.SET_PULL_UP
    assert lg.claims[17][2] == FakeLgpio.SET_PULL_NONE                     # opto không tự đổi
    g = gpio_pi.from_env({"FAIBOTIC_GPIO_PINS": ",".join(f"{k}={v}" for k, v in PINS.items()),
                          "FAIBOTIC_GPIO_BACKEND": "lgpio"})
    assert g.pull["LIFT_BOT"] == "up"
    g.close()
    g = make(pull={"LIFT_BOT": "none"})
    assert lg.claims[5][2] == FakeLgpio.SET_PULL_NONE and lg.claims[6][2] == FakeLgpio.SET_PULL_UP


@pytest.mark.parametrize("bcm", [0, 1, 2, 4, 8])
def test_output_on_default_pullup_bcm_rejected(lg, bcm):
    """m1: K5_EN trên BCM 0–8 (kéo lên mặc định) bị từ chối."""
    with pytest.raises(GPIOConfigError, match="BCM 0–8"):
        make(pins={**{k: v for k, v in PINS.items() if not k.startswith("LIFT")}, "K5_EN": bcm})


def test_trip_write_error_closes_gpio(lg):
    """m11: ghi K5 lỗi hai lần trong trip() -> đóng GPIO (nhả chân) rồi báo lỗi."""
    g = make()
    lt = SafetyLatch(g, servo_discharge_s=0.3)
    g.write("K5_EN", True)
    orig = lg.gpio_write

    def bad(h, off, v):
        if v == 0 and off == 22 and not g._closed:
            raise RuntimeError("ghi lỗi")
        return orig(h, off, v)
    lg.gpio_write = bad
    with pytest.raises(RuntimeError):
        lt.trip("thử")
    assert g._closed and not lg.claims


def test_backend_close_lowers_k5_with_external_latch(lg):
    """M5: Robot/backend nhận latch từ ngoài (kịch bản EVT) -> close() vẫn hạ K5."""
    from faibotic import spec
    from faibotic.backends.feetech import FeetechBackend
    from faibotic.feetech.mockbus import FakeServoPort
    g = make()
    lt = SafetyLatch(g, servo_discharge_s=0.3)
    be = FeetechBackend(port=FakeServoPort(list(spec.SERVO_IDS.values())), sleep=lambda s: None, latch=lt)
    with lt._lock:                                                          # giả đã reset xong
        g.write("K5_EN", True)
        lt._en, lt.state = True, "run"
    be.close()
    assert lg.level[22] == 0 and lt.state == "stopped" and not lt.ok


def test_evtlib_pigpio_uses_sdk_env_config(lg, monkeypatch):
    """M1: kịch bản EVT đọc cùng FAIBOTIC_GPIO_ACTIVE_LOW / PULL như SDK (chỉ cho chân đã khai)."""
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1] / "product" / "evt"))
    import evtlib
    env = {"FAIBOTIC_GPIO_ACTIVE_LOW": "ESTOP_OK,ARM_SENSE,RESET_SENSE", "FAIBOTIC_GPIO_PULL": "ESTOP_OK=up,K1_AUX=down"}
    g = evtlib.PiGPIO({k: v for k, v in PINS.items()}, outputs=("K5_EN",), hw="bang_di_day", backend="lgpio", env=env)
    assert g.active_low == {"ESTOP_OK", "ARM_SENSE", "RESET_SENSE"} and lg.claims[17][2] == FakeLgpio.SET_PULL_UP
    assert lg.claims[5][2] == FakeLgpio.SET_PULL_UP                         # LIFT_* mặc định kéo lên
    lg.level[17] = 0
    assert g.read("ESTOP_OK") is True
    evtlib.release_all()


def test_evtlib_run_releases_gpio_on_error(lg):
    """M5: kịch bản lỗi giữa chừng (K5 đang cao) -> run() hạ K5 và nhả chân trong finally."""
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1] / "product" / "evt"))
    import evtlib

    def main(args):
        g = evtlib.PiGPIO({k: v for k, v in PINS.items() if not k.startswith("LIFT")}, outputs=("K5_EN",),
                          hw="bang_di_day", backend="lgpio", env={})
        g.write("K5_EN", True)
        raise RuntimeError("lỗi giữa chừng")
    with pytest.raises(RuntimeError):
        evtlib.run(main, argv=[])
    assert lg.level[22] == 0 and not lg.claims and not lg.open_handles
