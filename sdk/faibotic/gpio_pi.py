# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""GPIO thật trên Raspberry Pi 5 cho bộ #0 (bảng đi dây EVT, electrical/bang-di-day-evt.md mục 2): ngõ ra K5_EN, ngõ vào opto
ESTOP_OK, ARM_SENSE, RESET_SENSE (K1_AUX nếu có), hai công tắc hành trình trục nâng LIFT_BOT, LIFT_TOP, và chốt hãm trục nâng
thanh răng v0.5 (R2 7.2): ngõ ra LIFT_LATCH_EN (kích cuộn solenoid qua MOSFET/rơ-le; cuộn lấy 12 V từ SERVO+ sau K1) và ngõ vào
LIFT_LATCH_RETRACTED (cảm biến "chốt đã rút").

Thư viện nạp LƯỜI (chỉ khi tạo PiGPIO), nên import module này trên máy không có GPIO vẫn chạy; test dùng thư viện giả.
Hai backend: "lgpio" (mặc định trên Raspberry Pi OS Bookworm, cũng là backend gpiozero dùng trên Pi 5) và "gpiod" (libgpiod v2,
như product/evt/evtlib.py). "auto" thử lgpio rồi gpiod. CHƯA chạy trên Pi thật: chân BCM chưa chốt (WE07), nên KHÔNG có chân
mặc định; mức tích cực của module opto/rơ-le chưa chọn mã cũng phải khai.

Mặc định an toàn không đổi:
- Không đặt FAIBOTIC_GPIO_PINS thì from_env() trả None -> FeetechBackend không có chốt E-stop -> từ chối enable().
- Thiếu chân bắt buộc của mạch, thiếu thư viện, không tìm thấy chip GPIO của Pi -> lỗi, không âm thầm chạy.
- Ngõ ra (K5_EN) khởi tạo mức thấp = K5 hở; ngõ ra không được khai tích cực mức thấp (module rơ-le kích mức thấp có thể
  hút K5 qua đường hồi của opto khi Pi tắt hoặc chân thả nổi). close() hạ mọi ngõ ra trước khi nhả chân.
- read()/write() lỗi thì ném ngoại lệ: luồng giám sát SafetyLatch coi là lỗi và trip() (hạ K5, tắt mô-men).

read(pin) trả mức LOGIC (True = tích cực): ESTOP_OK True = K2 đã chốt, ARM_SENSE True = SERVO+ có điện, RESET_SENSE True = nút
Reset đang nhấn, LIFT_* True = công tắc đang bị chạm, LIFT_LATCH_RETRACTED True = chốt đã rút. Chân khai trong active_low được
đảo trong phần mềm.

Chân chốt hãm (đề xuất, electrical/bang-di-day-evt.md WE10/WE11 = bang-day.csv G13/G15): LIFT_LATCH_EN = BCM 24, LIFT_LATCH_RETRACTED = BCM 25. Cả hai
phải là BCM >= 9 (BCM 0–8 kéo lên lúc khởi động: cuộn có thể có điện, chốt rút trước khi SDK giữ chân) và khác mọi chân khác
(kể cả K5_EN). Cảm biến: công tắc NO đóng xuống GND khi chốt rút, Pi kéo lên trong, mặc định tích cực mức thấp: đứt dây đọc
"không rút" (không cho chạy). Mặc định này tự thêm, không cần khai trong FAIBOTIC_GPIO_ACTIVE_LOW/PULL.

Biến môi trường cho from_env() (ví dụ, chân CHƯA chốt):
  FAIBOTIC_GPIO_PINS="K5_EN=22,ESTOP_OK=17,ARM_SENSE=27,RESET_SENSE=23,LIFT_BOT=5,LIFT_TOP=6,LIFT_LATCH_EN=24,LIFT_LATCH_RETRACTED=25,DOCK_L=16,DOCK_R=26"
  (DOCK_L/DOCK_R: công tắc khoá trạm hai tay qua opto, S2; đề xuất DOCK_DEFAULT_BCM, mặc định kéo lên + tích cực mức thấp như cảm biến chốt.
   Cấu hình di_dong không khai hai chân này thì FeetechBackend từ chối enable/lift/đế, trừ allow_no_dock=True)
  FAIBOTIC_GPIO_ACTIVE_LOW="ESTOP_OK,ARM_SENSE,RESET_SENSE"   (opto PC817 kéo xuống khi LED sáng)
  FAIBOTIC_GPIO_PULL="LIFT_BOT=up,LIFT_TOP=up"                 (mặc định cho LIFT_*: công tắc NC nối GND, chạm -> hở -> cao)
  FAIBOTIC_GPIO_BACKEND=auto|lgpio|gpiod   FAIBOTIC_GPIO_CHIP=<số hoặc /dev/gpiochipN> (mặc định tự tìm chip "rp1")
  FAIBOTIC_SAFETY_HW=bang_di_day|pcb   FAIBOTIC_SERVO_DISCHARGE_MS=<số đo EVT-11, estop_jig.py --che-do xa-tu>
"""
from __future__ import annotations

import importlib
import math
import os
import threading

from .limits import LiftSwitches
from .safety import GPIO, HW_BANG, HW_PCB, resolve_hw

# chân bắt buộc theo mạch dừng khẩn cấp (khớp product/evt/evtlib.py GPIO_PINS)
REQUIRED = {HW_BANG: ("K5_EN", "ESTOP_OK", "ARM_SENSE", "RESET_SENSE"), HW_PCB: ("ESTOP_SENSE", "LATCH_SENSE", "RELAY_EN")}
OUTPUTS = {HW_BANG: ("K5_EN",), HW_PCB: ("RELAY_EN",)}
LIFT_PINS = ("LIFT_BOT", "LIFT_TOP")
LATCH_COIL, LATCH_SENSOR = "LIFT_LATCH_EN", "LIFT_LATCH_RETRACTED"
LATCH_PINS = (LATCH_COIL, LATCH_SENSOR)
LATCH_DEFAULT_BCM = {LATCH_COIL: 24, LATCH_SENSOR: 25}    # đề xuất, chưa đi dây thật
DOCK_PINS = ("DOCK_L", "DOCK_R")
DOCK_DEFAULT_BCM = {"DOCK_L": 16, "DOCK_R": 26}            # S2 đề xuất: ngõ vào BCM >= 9, khác K5/ESTOP/ARM/RESET/LIFT/chốt; chưa đi dây thật
PULLS = ("up", "down", "none")
# BCM 0–8 có kéo lên mặc định khi Pi khởi động [theo tài liệu BCM2711; Pi 5 chưa kiểm]: ngõ ra K5_EN đặt ở đó có thể lên cao
# trước khi PiGPIO giữ chân hoặc sau khi tiến trình chết. Cấm cho mọi ngõ ra.
OUTPUT_FORBIDDEN_BCM = frozenset(range(0, 9))
# số đo xả tụ EVT-11 chấp nhận (ms); ngoài khoảng coi là nhập sai (vd nhập giây thay ms)
DISCHARGE_MS_RANGE = (50.0, 5000.0)
# nhãn chip GPIO header 40 chân: Pi 5 (RP1) trước, Pi 4/3 sau
CHIP_LABELS = ("pinctrl-rp1", "rp1", "pinctrl-bcm2712", "pinctrl-bcm2711", "pinctrl-bcm2835")


class GPIOUnavailable(RuntimeError):
    """Không có thư viện GPIO hoặc không mở được chip của Pi."""


class GPIOConfigError(ValueError):
    """Khai chân sai (thiếu chân bắt buộc, trùng số, ngõ ra tích cực thấp...)."""


def _import(name: str):
    try:
        return importlib.import_module(name)
    except ImportError as e:
        raise GPIOUnavailable(f"thiếu thư viện {name} (Pi 5: sudo apt install python3-lgpio hoặc pip install gpiod)") from e


class PiGPIO(GPIO):
    """GPIO Pi 5. pins: {"K5_EN": 22, ...} số BCM (offset trên chip). outputs: tên là ngõ ra (mặc định theo hw).

    active_low: tên ngõ VÀO đảo mức. pull: {"LIFT_BOT": "up", ...} điện trở kéo trong. chip: None = tự tìm theo nhãn.
    backend: "auto" | "lgpio" | "gpiod". require_pins: False chỉ dùng cho kịch bản đo một phần (công tắc hành trình).
    """

    def __init__(self, pins: dict[str, int], hw: str | None = HW_BANG, outputs=None, active_low=(), pull=None,
                 chip=None, backend: str = "auto", consumer: str = "faibotic-m1", require_pins: bool = True):
        if hw is None:
            raise GPIOConfigError("PiGPIO cần hw='bang_di_day' hoặc 'pcb' (mạch dừng khẩn cấp)")
        self.hw = resolve_hw(hw=hw)
        self.pins = {str(k): int(v) for k, v in dict(pins).items()}
        self.outputs = tuple(OUTPUTS[self.hw] if outputs is None else outputs)
        if outputs is None and LATCH_COIL in self.pins:
            self.outputs += (LATCH_COIL,)     # cuộn chốt: ngõ ra, mức thấp lúc khởi tạo = cuộn mất điện = chốt cài
        self.active_low = frozenset(active_low)
        self.pull = dict(pull or {})
        for p in LIFT_PINS:                  # công tắc NC nối GND: mặc định kéo lên -> chạm hoặc đứt dây đọc "chạm"
            if p in self.pins and p not in self.outputs:
                self.pull.setdefault(p, "up")
        if LATCH_SENSOR in self.pins and LATCH_SENSOR not in self.outputs:   # NO xuống GND khi rút: đứt dây = "không rút"
            self.pull.setdefault(LATCH_SENSOR, "up")
            self.active_low = self.active_low | {LATCH_SENSOR}
        for p in DOCK_PINS:                  # opto kéo xuống khi khoá đóng (như ESTOP_OK): đứt dây/opto hỏng = "mở" (an toàn)
            if p in self.pins and p not in self.outputs:
                self.pull.setdefault(p, "up")
                self.active_low = self.active_low | {p}
        self._validate(require_pins)
        self._lock = threading.RLock()
        self._closed = False
        self.backend = None
        self.chip = None
        self.chip_label = None
        order = ("lgpio", "gpiod") if backend == "auto" else (backend,)
        errs = []
        for b in order:
            if b not in ("lgpio", "gpiod"):
                raise GPIOConfigError(f"backend không biết: {b!r} (auto | lgpio | gpiod)")
            try:
                getattr(self, f"_open_{b}")(chip, consumer)
                self.backend = b
                break
            except GPIOUnavailable as e:
                errs.append(f"{b}: {e}")
        if self.backend is None:
            raise GPIOUnavailable("không mở được GPIO: " + "; ".join(errs))

    # ---------- kiểm khai báo ----------
    def _validate(self, require_pins: bool) -> None:
        if require_pins:
            miss = [p for p in REQUIRED[self.hw] if p not in self.pins]
            if miss:
                raise GPIOConfigError(f"thiếu chân {miss} cho mạch {self.hw} (khai số BCM theo dây WE07)")
        nums = list(self.pins.values())
        dup = sorted({n for n in nums if nums.count(n) > 1})
        if dup:
            raise GPIOConfigError(f"trùng số BCM {dup}")
        bad = [n for n in nums if not 0 <= n <= 27]
        if bad:
            raise GPIOConfigError(f"số BCM ngoài header 40 chân (0–27): {bad}")
        if LATCH_COIL in self.pins and LATCH_COIL not in self.outputs:
            raise GPIOConfigError(f"{LATCH_COIL} phải là ngõ ra")
        latch_early = [f"{p}=BCM{self.pins[p]}" for p in LATCH_PINS if self.pins.get(p) in OUTPUT_FORBIDDEN_BCM]
        if latch_early:
            raise GPIOConfigError(f"chân chốt trục nâng {latch_early} nằm trên BCM 0–8: chọn BCM ≥ 9 (đề xuất "
                                  f"{LATCH_COIL}=24, {LATCH_SENSOR}=25)")
        early = [f"{p}=BCM{self.pins[p]}" for p in self.outputs if self.pins.get(p) in OUTPUT_FORBIDDEN_BCM]
        if early:
            raise GPIOConfigError(f"ngõ ra {early} nằm trên BCM 0–8 (kéo lên mặc định khi khởi động): chọn BCM ≥ 9 có kéo "
                                  "xuống, thêm điện trở kéo xuống 10 kΩ ở đầu vào module K5")
        lo = [p for p in self.outputs if p in self.active_low]
        if lo:
            raise GPIOConfigError(f"ngõ ra {lo} không được tích cực mức thấp: dùng module rơ-le kích mức cao (K5 hở khi Pi "
                                  "tắt hoặc chân thả nổi)")
        unknown = [p for p in list(self.active_low) + list(self.pull) if p not in self.pins]
        if unknown:
            raise GPIOConfigError(f"active_low/pull nhắc chân không khai: {unknown}")
        badp = {p: v for p, v in self.pull.items() if v not in PULLS or p in self.outputs}
        if badp:
            raise GPIOConfigError(f"pull sai (chỉ ngõ vào, up|down|none): {badp}")

    # ---------- lgpio ----------
    def _open_lgpio(self, chip, consumer) -> None:
        lg = _import("lgpio")
        self._lg = lg
        h = self._lg_open_chip(lg, chip)
        try:
            for name, off in self.pins.items():
                if name in self.outputs:
                    lg.gpio_claim_output(h, off, 0)
                else:
                    flag = {"up": lg.SET_PULL_UP, "down": lg.SET_PULL_DOWN}.get(self.pull.get(name), lg.SET_PULL_NONE)
                    lg.gpio_claim_input(h, off, flag)
        except Exception as e:
            lg.gpiochip_close(h)
            raise GPIOUnavailable(f"không giữ được chân trên gpiochip{self.chip}: {e!r}") from e
        self._h = h

    def _lg_open_chip(self, lg, chip):
        cands = [int(str(chip).replace("/dev/gpiochip", ""))] if chip is not None else list(range(0, 6))
        found = []
        for n in cands:
            try:
                h = lg.gpiochip_open(n)
            except Exception:                  # noqa: BLE001 - chip không có
                continue
            try:
                label = str(lg.gpio_get_chip_info(h)[3])
            except Exception:                  # noqa: BLE001
                label = ""
            if chip is not None or label in CHIP_LABELS:
                found.append((CHIP_LABELS.index(label) if label in CHIP_LABELS else 0, n, h, label))
            else:
                lg.gpiochip_close(h)
        if not found:
            raise GPIOUnavailable(f"không thấy chip GPIO của Pi (nhãn {CHIP_LABELS}) trong gpiochip{cands}")
        found.sort()
        for _, _, h, _ in found[1:]:
            lg.gpiochip_close(h)
        _, self.chip, h, self.chip_label = found[0]
        return h

    # ---------- gpiod v2 ----------
    def _open_gpiod(self, chip, consumer) -> None:
        gd = _import("gpiod")
        line = _import("gpiod.line")
        path = self._gd_find_chip(gd, chip)
        bias = {"up": line.Bias.PULL_UP, "down": line.Bias.PULL_DOWN}
        cfg = {}
        for name, off in self.pins.items():
            if name in self.outputs:
                cfg[off] = gd.LineSettings(direction=line.Direction.OUTPUT, output_value=line.Value.INACTIVE)
            elif name in self.pull and self.pull[name] != "none":
                cfg[off] = gd.LineSettings(direction=line.Direction.INPUT, bias=bias[self.pull[name]])
            else:
                cfg[off] = gd.LineSettings(direction=line.Direction.INPUT)
        try:
            self._req = gd.request_lines(path, consumer=consumer, config=cfg)
        except Exception as e:
            raise GPIOUnavailable(f"không giữ được chân trên {path}: {e!r}") from e
        self._Value = line.Value
        self.chip = path

    def _gd_find_chip(self, gd, chip) -> str:
        if chip is not None:
            return chip if str(chip).startswith("/dev/") else f"/dev/gpiochip{int(chip)}"
        found = []
        for n in range(0, 6):
            path = f"/dev/gpiochip{n}"
            try:
                with gd.Chip(path) as c:
                    label = str(c.get_info().label)
            except Exception:                  # noqa: BLE001
                continue
            if label in CHIP_LABELS:
                found.append((CHIP_LABELS.index(label), path, label))
        if not found:
            raise GPIOUnavailable(f"không thấy chip GPIO của Pi (nhãn {CHIP_LABELS}) trong /dev/gpiochip0..5")
        found.sort()
        self.chip_label = found[0][2]
        return found[0][1]

    # ---------- đọc/ghi ----------
    def _raw_read(self, off: int) -> bool:
        if self.backend == "lgpio":
            v = self._lg.gpio_read(self._h, off)
            if v not in (0, 1):
                raise OSError(f"lgpio.gpio_read({off}) trả {v!r}")
            return bool(v)
        return self._req.get_value(off) == self._Value.ACTIVE

    def _raw_write(self, off: int, level: bool) -> None:
        if self.backend == "lgpio":
            self._lg.gpio_write(self._h, off, int(bool(level)))
        else:
            self._req.set_value(off, self._Value.ACTIVE if level else self._Value.INACTIVE)

    def read(self, pin: str) -> bool:
        with self._lock:
            if self._closed:
                raise OSError("PiGPIO đã đóng")
            if pin not in self.pins:
                raise KeyError(pin)
            v = self._raw_read(self.pins[pin])
            return (not v) if pin in self.active_low else v

    def write(self, pin: str, level: bool) -> None:
        with self._lock:
            if self._closed:
                raise OSError("PiGPIO đã đóng")
            if pin not in self.outputs or pin not in self.pins:
                raise KeyError(f"{pin} không phải ngõ ra đã khai")
            self._raw_write(self.pins[pin], bool(level))

    def has(self, pin: str) -> bool:
        return pin in self.pins

    def close(self) -> None:
        """Hạ mọi ngõ ra (K5 hở) rồi nhả chân. Gọi nhiều lần không sao."""
        with self._lock:
            if self._closed:
                return
            try:
                for p in self.outputs:
                    if p in self.pins:
                        self._raw_write(self.pins[p], False)
            finally:
                self._closed = True
                if self.backend == "lgpio":
                    for off in self.pins.values():
                        try:
                            self._lg.gpio_free(self._h, off)
                        except Exception:      # noqa: BLE001
                            pass
                    self._lg.gpiochip_close(self._h)
                else:
                    self._req.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class PiLiftSwitches(LiftSwitches):
    """Công tắc hành trình trục nâng qua GPIO: read() -> (dưới, trên), True = đang chạm (đã đảo theo active_low của gpio).

    Đi dây (bang-di-day-evt.md: công tắc NC): NC nối GND, PiGPIO mặc định kéo lên LIFT_BOT/LIFT_TOP -> chạm hoặc đứt dây đều
    đọc "chạm", lift_gate chặn chiều đó (an toàn khi hỏng). Đọc lỗi thì ném ngoại lệ (write_joints dừng, không bỏ qua công tắc).
    """

    def __init__(self, gpio: GPIO, bottom: str = "LIFT_BOT", top: str = "LIFT_TOP"):
        if hasattr(gpio, "has") and not (gpio.has(bottom) and gpio.has(top)):
            raise GPIOConfigError(f"GPIO chưa khai {bottom}/{top}")
        self.gpio, self.bottom, self.top = gpio, bottom, top

    def read(self) -> tuple[bool, bool]:
        return bool(self.gpio.read(self.bottom)), bool(self.gpio.read(self.top))


# ---------- cấu hình từ biến môi trường ----------
def _kv(s: str) -> dict[str, str]:
    out = {}
    for part in s.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        if "=" not in part:
            raise GPIOConfigError(f"thiếu '=' trong {part!r} (dạng TÊN=giá_trị)")
        k, v = part.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def config_from_env(env=None) -> dict | None:
    """Đọc cấu hình PiGPIO từ biến môi trường; None nếu FAIBOTIC_GPIO_PINS chưa đặt."""
    env = os.environ if env is None else env
    raw = env.get("FAIBOTIC_GPIO_PINS", "").strip()
    if not raw:
        return None
    try:
        pins = {k: int(v) for k, v in _kv(raw).items()}
    except ValueError as e:
        raise GPIOConfigError(f"FAIBOTIC_GPIO_PINS sai: {e}") from e
    al = [p.strip() for p in env.get("FAIBOTIC_GPIO_ACTIVE_LOW", "").split(",") if p.strip()]
    chip = env.get("FAIBOTIC_GPIO_CHIP") or None
    return dict(pins=pins, hw=env.get("FAIBOTIC_SAFETY_HW") or HW_BANG, active_low=al,
                pull=_kv(env.get("FAIBOTIC_GPIO_PULL", "")), chip=chip, backend=env.get("FAIBOTIC_GPIO_BACKEND", "auto"))


def from_env(env=None) -> PiGPIO | None:
    """PiGPIO theo biến môi trường, hoặc None nếu chưa khai (backend sẽ từ chối enable). Khai mà mở lỗi thì ném lỗi."""
    cfg = config_from_env(env)
    return None if cfg is None else PiGPIO(**cfg)


def parse_discharge_ms(value) -> tuple[float | None, str | None]:
    """Số đo xả tụ EVT-11 (ms) -> (giây, None) nếu hợp lệ; (None, lý do) nếu sai. None/"" -> (None, None) = chưa đo.

    Hợp lệ: số hữu hạn trong DISCHARGE_MS_RANGE (50–5000 ms). "0,3"/"0.3" (nhầm giây), 0, âm, nan, chữ đều sai."""
    if value is None or (isinstance(value, str) and not value.strip()):
        return None, None
    try:
        ms = float(str(value).strip())
    except ValueError:
        return None, f"số xả tụ {value!r} không phải số (đơn vị ms)"
    lo, hi = DISCHARGE_MS_RANGE
    if not math.isfinite(ms) or not lo <= ms <= hi:
        return None, (f"số xả tụ {value!r} ms ngoài khoảng {lo:.0f}–{hi:.0f} ms (nhập ms, không phải giây; số đo EVT-11 "
                      "từ estop_jig.py --che-do xa-tu)")
    return ms / 1000.0, None


def latch_kwargs(gpio, env=None) -> dict:
    """Tham số thêm cho SafetyLatch bộ #0 theo phần cứng đã khai: K1_AUX nếu có chân, số đo xả tụ EVT-11 nếu có.
    Số xả tụ sai -> config_error: SafetyLatch từ chối reset (không bật K5) với lý do."""
    env = os.environ if env is None else env
    if resolve_hw(gpio) != HW_BANG:
        return {}
    kw = {}
    if getattr(gpio, "has", lambda p: False)("K1_AUX"):
        kw["k1_aux_pin"] = "K1_AUX"
    sd, err = parse_discharge_ms(env.get("FAIBOTIC_SERVO_DISCHARGE_MS"))
    if err:
        kw["config_error"] = "FAIBOTIC_SERVO_DISCHARGE_MS: " + err
    elif sd is not None:
        kw["servo_discharge_s"] = sd
    return kw


def lift_latch_for(gpio, **kw):
    """safety.LiftLatch nếu gpio có đủ LIFT_LATCH_EN và LIFT_LATCH_RETRACTED; không khai chân nào: None; khai thiếu: GPIOConfigError."""
    has = getattr(gpio, "has", None)
    if has is None:
        return None
    got = [p for p in LATCH_PINS if has(p)]
    if len(got) == len(LATCH_PINS):
        from .safety import LiftLatch
        return LiftLatch(gpio, coil_pin=LATCH_COIL, sensor_pin=LATCH_SENSOR, **kw)
    if got:     # khai một chân: lift sẽ chạy KHÔNG có chốt mà người dùng tưởng có -> lỗi cấu hình
        raise GPIOConfigError(f"chốt trục nâng cần đủ {LATCH_PINS}, mới khai {got}")
    return None


def dock_for(gpio):
    """safety.DockSwitches nếu gpio có đủ DOCK_L và DOCK_R (công tắc khoá trạm, S2); không khai chân nào: None; khai thiếu: GPIOConfigError."""
    has = getattr(gpio, "has", None)
    if has is None:
        return None
    got = [p for p in DOCK_PINS if has(p)]
    if len(got) == len(DOCK_PINS):
        from .safety import DockSwitches
        return DockSwitches(gpio, DOCK_PINS)
    if got:
        raise GPIOConfigError(f"khoá trạm cần đủ {DOCK_PINS}, mới khai {got}")
    return None


def lift_switches_for(gpio) -> PiLiftSwitches | None:
    """PiLiftSwitches nếu gpio có đủ LIFT_BOT và LIFT_TOP, không thì None."""
    has = getattr(gpio, "has", None)
    if has is not None and all(has(p) for p in LIFT_PINS):
        return PiLiftSwitches(gpio)
    return None
