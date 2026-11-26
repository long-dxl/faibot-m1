# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""An toàn phần mềm M1: chốt E-stop tạm bằng phần mềm và phát hiện lò xo cân bằng vai đứt.

SafetyLatch là lớp TẠM để chẩn đoán. Pi chạy Linux, không phải thiết bị an toàn: dừng thật là K1 cắt bằng phần cứng.
Hai cấu hình phần cứng (chọn bằng hw=, thuộc tính hw của GPIO, hoặc spec.SAFETY_HW):

- "bang_di_day" (bộ #0 EVT, mặc định; electrical/bang-di-day-evt.md mục 2): Pi ra K5_EN (K5 nằm trong nhánh cho phép của K2,
  theo mức), vào ESTOP_OK (opto trên RUN: cao = K2 đã chốt), ARM_SENSE (opto trên SERVO+: cao = servo có điện),
  RESET_SENSE (nút Reset, bắt buộc trừ thử bàn), K1_AUX (tiếp điểm phụ K1, tuỳ chọn). Không có RELAY_EN, LATCH_SENSE, U4, INA226.
- "pcb" (PCB nguồn sau EVT; electrical/quyet-dinh-bms-estop.md mục 2.6): ESTOP_SENSE (cao = E-stop nhả), LATCH_SENSE
  (K1_FEED), RELAY_EN (U4 chỉ chạy khi có CẠNH LÊN sau mỗi lần reset). Hành vi giữ như trước.

Cả hai: phần mềm không bao giờ tự đóng mạch thay người. Reset luôn là thao tác tay trên nút (IEC 60204-1 / ISO 13850).
"""
from __future__ import annotations

import math
import threading
import time

from . import spec as _spec

HW_PCB, HW_BANG = "pcb", "bang_di_day"


def resolve_hw(gpio=None, hw: str | None = None) -> str:
    """Thứ tự: hw truyền vào > gpio.hw > spec.SAFETY_HW. Chấp nhận "bang-di-day"."""
    h = hw or getattr(gpio, "hw", None) or _spec.SAFETY_HW
    h = h.replace("-", "_")
    if h not in (HW_PCB, HW_BANG):
        raise ValueError(f"cấu hình mạch an toàn không biết: {h!r} (pcb | bang_di_day)")
    return h


# ---------------- GPIO trừu tượng ----------------
class GPIO:
    """Giao diện tối thiểu. Bản thật trên Pi 5 (gpiod) ở product/evt/evtlib.py PiGPIO; test dùng FakeGPIO/SimBangDiDay."""

    hw: str | None = None

    def read(self, pin: str) -> bool:
        raise NotImplementedError

    def write(self, pin: str, level: bool) -> None:
        raise NotImplementedError


class FakeGPIO(GPIO):
    """GPIO giả theo chân PCB: levels đặt từ test, history ghi mọi lần ghi (để kiểm cạnh lên RELAY_EN)."""

    hw = HW_PCB

    def __init__(self, **levels: bool):
        self.levels = {"ESTOP_SENSE": True, "LATCH_SENSE": True, "RELAY_EN": True}   # RELAY_EN cao lúc đầu: SafetyLatch phải tự hạ
        self.levels.update(levels)
        self.history: list[tuple[str, bool]] = []

    def read(self, pin: str) -> bool:
        return bool(self.levels[pin])

    def write(self, pin: str, level: bool) -> None:
        self.levels[pin] = bool(level)
        self.history.append((pin, bool(level)))

    def rising_edges(self, pin: str = "RELAY_EN") -> int:
        n, prev = 0, None
        for p, v in self.history:
            if p == pin:
                n += int(prev is False and v is True)
                prev = v
        return n


class SimBangDiDay(GPIO):
    """Mạch điều khiển bảng đi dây bộ #0 giả lập theo rơ-le (bang-di-day-evt.md mục 2), đồng hồ do test/kịch bản cấp.

    +12V_CTRL - ES-NC1 - K5 - (RST-NO // K2.1) - cuộn K2;  RUN = K2.2;  K4 = RUN và KT chưa xong;
    K1 = RUN và KT xong và RST-NC đóng (Reset nhả) và ES-NC2 đóng.  ESTOP_OK = RUN; ARM_SENSE = SERVO+ (K1 hoặc K4, cộng
    thời gian tụ xả qua ngưỡng opto cap_hold_s [mô phỏng]).  press(): "estop", "reset", "v12" (Q6 / 12 V);
    weld: tập phần tử dính tiếp điểm ("K1", "K2", "K4", "KT", "K5");  estop_bypass: chập hai kênh E-stop (dòng 14);
    k1_coil_open: K1 không đóng (cuộn hở, NC2 hỏng);  RESET_SENSE đọc được nếu reset_sense, K1_AUX nếu k1_aux.
    """

    hw = HW_BANG

    def __init__(self, clock, kt_s: float = _spec.EVT_KT_S, cap_hold_s: float = 0.03, reset_sense: bool = True,
                 k1_aux: bool = False):
        self.clock, self.kt_s, self.cap_hold_s = clock, kt_s, cap_hold_s
        self.reset_sense, self.k1_aux = reset_sense, k1_aux
        self.k1_coil_open = False            # cuộn K1 hở / NC2 hỏng: K1 không bao giờ đóng
        self.k5 = self.estop = self.reset_held = False
        self.v12, self.estop_bypass = True, False
        self.weld: set[str] = set()
        self.k2 = self.k1 = self.k4 = self.run = self.arm = False
        self._t_run = self._t_src = None
        self.history: list[tuple[str, bool]] = []
        self.script: list[tuple[float, str, bool]] = []
        self._lock = threading.RLock()
        self._eval()

    def press(self, what: str, down: bool = True) -> None:
        with self._lock:
            if what == "estop":
                self.estop = down
            elif what == "reset":
                self.reset_held = down
            elif what == "v12":
                self.v12 = not down          # press("v12") = Q6 nhảy / mất 12 V
            else:
                raise KeyError(what)
            self._eval()

    def click_reset(self) -> None:
        self.press("reset", True)
        self.press("reset", False)

    def click_reset_at(self, t: float, hold_s: float = 0.2) -> None:
        """Người ảo: nhấn Reset ở thời điểm t (đồng hồ của mạch giả), giữ hold_s rồi nhả."""
        with self._lock:
            self.script += [(t, "reset_held", True), (t + hold_s, "reset_held", False)]
            self.script.sort(key=lambda e: e[0])

    def _eval(self) -> None:
        now = self.clock()
        while self.script and self.script[0][0] <= now:
            te, attr, val = self.script.pop(0)
            self._logic(te)
            setattr(self, attr, val)
            self._logic(te)
        self._logic(now)

    def _logic(self, now: float) -> None:
        es = self.estop and not self.estop_bypass
        k5 = self.k5 or "K5" in self.weld
        hold = self.k2 or "K2" in self.weld
        self.k2 = self.v12 and not es and k5 and (self.reset_held or hold)
        self.run = self.v12 and (self.k2 or "K2" in self.weld)
        if self.run and self._t_run is None:
            self._t_run = now
        if not self.run:
            self._t_run = None
        kt = self.run and ("KT" in self.weld or now - self._t_run >= self.kt_s)
        self.k4 = (self.run and not kt) or "K4" in self.weld
        self.k1 = (self.run and kt and not self.reset_held and not es and not self.k1_coil_open) or "K1" in self.weld
        if self.k1 or self.k4:
            self._t_src = now
        self.arm = self.k1 or self.k4 or (self._t_src is not None and now - self._t_src < self.cap_hold_s)

    def read(self, pin: str) -> bool:
        with self._lock:
            self._eval()
            if pin == "ESTOP_OK":
                return self.run
            if pin == "ARM_SENSE":
                return self.arm
            if pin == "RESET_SENSE" and self.reset_sense:
                return self.reset_held
            if pin == "K1_AUX" and self.k1_aux:
                return self.k1
            if pin == "K5_EN":
                return self.k5
            raise KeyError(pin)

    def write(self, pin: str, level: bool) -> None:
        with self._lock:
            if pin != "K5_EN":
                raise KeyError(pin)
            self.k5 = bool(level)
            self.history.append((pin, bool(level)))
            self._eval()


class ResetRefused(RuntimeError):
    pass


class SafetyLatch:
    """Chốt E-stop phần mềm (lớp TẠM; dừng thật là K1 cắt bằng phần cứng).

    SafetyLatch(gpio, ..., hw=None) trả về PcbLatch hoặc BangDiDayLatch theo resolve_hw(). Chung cho cả hai:
    - Khởi tạo: hạ chân ra, trạng thái "stopped". KHÔNG tự bật lại sau khởi động, kể cả khi E-stop đang nhả.
    - poll(): dừng -> hạ chân ra ngay, gọi on_stop() (tắt mô-men), trạng thái "estop". Nhả E-stop KHÔNG tự chạy lại.
    - start_monitor(period): luồng nền gọi poll() mỗi period giây, độc lập với step() của chương trình người dùng.
      Luồng chết vì lỗi (đọc GPIO lỗi, on_stop lỗi...) hoặc dừng ngoài ý muốn -> trip() trước khi thoát.
    - reset(): người dùng chủ động reset sau khi nhả E-stop và kiểm tra vùng làm việc; chi tiết theo từng mạch.
    """

    hw: str | None = None

    def __new__(cls, gpio=None, *args, hw: str | None = None, **kwargs):
        if cls is SafetyLatch:
            cls = PcbLatch if resolve_hw(gpio, hw) == HW_PCB else BangDiDayLatch
        return super().__new__(cls)

    def _init_common(self, gpio, on_stop, clock, sleep) -> None:
        self.gpio, self.on_stop = gpio, on_stop
        self.on_tick = None      # gọi sau mỗi poll() trong luồng giám sát (watchdog của backend, R-25)
        self.clock, self._sleep = clock, sleep
        self.log: list[tuple[float, str]] = []
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()
        self.period: float | None = None
        self.last_poll: float | None = None
        self.monitor_error: str | None = None

    def _event(self, msg: str) -> None:
        self.log.append((self.clock(), msg))

    @property
    def ok(self) -> bool:
        return self.state == "run"

    # ---------- luồng giám sát nền ----------
    @property
    def monitor_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start_monitor(self, period: float = 0.01) -> None:
        if self.monitor_alive:
            return
        self.period = period
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, name="faibotic-estop", daemon=True)
        self._thread.start()

    def stop_monitor(self, timeout: float = 1.0) -> None:
        """Dừng luồng có chủ đích (close()); không trip."""
        self._stop_evt.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout)

    def _run(self) -> None:
        why = "luồng giám sát E-stop dừng ngoài ý muốn"
        try:
            while not self._stop_evt.is_set():
                self.poll()
                tick = self.on_tick
                if tick is not None:
                    tick()
                self._stop_evt.wait(self.period)
        except BaseException as e:       # noqa: BLE001 - mọi lỗi trong luồng an toàn đều phải dừng robot
            why = f"luồng giám sát E-stop lỗi: {e!r}"
            self.monitor_error = why
        finally:
            if not self._stop_evt.is_set():
                try:
                    self.trip(why)
                except BaseException as e:   # noqa: BLE001
                    self.monitor_error = f"{why}; trip lỗi: {e!r}"


class PcbLatch(SafetyLatch):
    """Cấu hình "pcb" (giữ nguyên hành vi trước 23/11).

    - poll(): ESTOP_SENSE thấp (hoặc LATCH_SENSE rớt khi đang chạy) -> hạ RELAY_EN ngay, on_stop(), "estop".
    - reset(): chỉ nhận khi ESTOP_SENSE cao (và LATCH_SENSE cao nếu có dùng); tạo cạnh lên RELAY_EN (thấp rồi cao) cho U4,
      chờ settle_s (RC trễ K1, ~0,3 s) rồi đọc lại LATCH_SENSE/ESTOP_SENSE: chưa cao thì về "estop" và từ chối.
    """

    hw = HW_PCB

    def __init__(self, gpio: GPIO, on_stop=None, estop_pin: str = "ESTOP_SENSE",
                 latch_pin: str | None = "LATCH_SENSE", relay_pin: str = "RELAY_EN", clock=time.monotonic,
                 settle_s: float = 0.3, sleep=time.sleep, hw: str | None = None):
        self._init_common(gpio, on_stop, clock, sleep)
        self.estop_pin, self.latch_pin, self.relay_pin = estop_pin, latch_pin, relay_pin
        self.settle_s = settle_s
        self.gpio.write(relay_pin, False)
        self.state = "stopped"
        self._event("khởi động: RELAY_EN thấp, chờ reset()")

    def estop_released(self) -> bool:
        return self.gpio.read(self.estop_pin)

    def poll(self) -> bool:
        """Đọc cảm biến; trả True nếu được chạy."""
        with self._lock:
            released = self.gpio.read(self.estop_pin)
            latch_ok = True if self.latch_pin is None else self.gpio.read(self.latch_pin)
            self.last_poll = self.clock()
            if not released or (self.state == "run" and not latch_ok):
                if self.state != "estop":
                    self.trip("E-stop nhấn" if not released else "LATCH_SENSE rớt (chốt phần cứng nhả)")
            return self.ok

    def trip(self, why: str) -> None:
        with self._lock:
            try:
                self.gpio.write(self.relay_pin, False)   # hạ RELAY_EN trước, rồi mới tắt mô-men qua bus
            finally:
                self.state = "estop"
                self._event(f"dừng: {why}")
                if self.on_stop is not None:
                    self.on_stop()

    def reset(self, on_wait=None) -> None:
        """Người dùng chủ động reset sau khi nhả E-stop và kiểm tra vùng làm việc (đã bấm Reset phần cứng trước)."""
        if on_wait is not None:
            on_wait()
        with self._lock:
            if not self.gpio.read(self.estop_pin):
                self._event("reset bị từ chối: E-stop đang nhấn")
                raise ResetRefused("E-stop đang nhấn: nhả nút trước khi reset")
            if self.latch_pin is not None and not self.gpio.read(self.latch_pin):
                self._event("reset bị từ chối: LATCH_SENSE thấp")
                raise ResetRefused("LATCH_SENSE thấp: bấm và nhả nút Reset phần cứng trước")
            self.gpio.write(self.relay_pin, False)
            self.gpio.write(self.relay_pin, True)    # cạnh lên cho U4
            self.state = "resetting"                 # luồng nền trip trong lúc chờ -> "estop"
            self._event("reset: RELAY_EN cạnh lên")
        if self.settle_s > 0:
            self._sleep(self.settle_s)               # chờ K1 hút (RC trễ), ngoài khoá để luồng nền vẫn đọc được E-stop
        with self._lock:
            if self.state != "resetting":            # luồng nền đã thấy E-stop trong lúc chờ
                raise ResetRefused("E-stop nhấn trong lúc reset")
            released = self.gpio.read(self.estop_pin)
            latch_ok = True if self.latch_pin is None else self.gpio.read(self.latch_pin)
            if not (released and latch_ok):
                self.trip("reset không giữ được: " + ("E-stop nhấn" if not released else "LATCH_SENSE vẫn thấp sau cạnh lên"))
                raise ResetRefused("chốt phần cứng không giữ sau reset: kiểm K1/U4 rồi reset lại")
            self.state = "run"
            self._event("reset: đã đọc lại cảm biến, chạy")


class BangDiDayLatch(SafetyLatch):
    """Cấu hình "bang_di_day" (bộ #0). Trình tự reset():

    0. Điều kiện cấu hình (kiểm trước khi bật K5): phải đọc được RESET_SENSE (nút Reset), và phải có một cách biết K1 đóng
       không dựa vào thời gian xả tụ: hoặc K1_AUX (tiếp điểm phụ K1, ví dụ EV200HAANA), hoặc servo_discharge_s đã đo
       (ARM_SENSE rớt sau bao lâu khi K4 nhả mà K1 không đóng, có servo, mô-men tắt) với arm_confirm_s >= 2 x số đo.
       bench=True (thử bàn, KHÔNG cắm servo) bỏ hai điều kiện này, có cảnh báo.
    1. Kiểm khi K5 còn hạ: ARM_SENSE thấp (không thì K1/K4 dính), ESTOP_OK thấp (không thì K2/K5 dính), Reset không đang
       nhấn. Sai thì từ chối, K5 giữ hạ.
    2. Pi bật K5 (chỉ là "cho phép": K5 nối tiếp ES-NC1, không song song với Reset), trạng thái "arming", gọi on_wait().
       Phần mềm không tự đóng mạch: không có người bấm Reset thì K2 không chốt, K1 không đóng.
    3. Người nhấn và nhả Reset: K2 tự giữ (ESTOP_OK lên), nạp trước kt_s, K1 đóng khi nhả Reset.
    4. Mốc K1 phải đóng = muộn hơn của (ESTOP_OK lên + kt_s) và (lúc thấy Reset nhả). Từ mốc đó, ESTOP_OK, ARM_SENSE (và
       K1_AUX nếu có) phải cao, Reset nhả, liên tục arm_confirm_s, mới chuyển "run". Không có K1_AUX thì tụ giữ ARM_SENSE
       tối đa servo_discharge_s, nên arm_confirm_s >= 2 x servo_discharge_s bảo đảm K1 hở thì ARM_SENSE rớt trước khi xác
       nhận xong. Hết reset_timeout_s chưa đạt -> hạ K5, từ chối.
    poll(): đang chạy mà ESTOP_OK rớt (E-stop, K2 nhả, mất 12 V), ARM_SENSE rớt hoặc K1_AUX mở -> hạ K5, on_stop(), "estop";
    không tự reset. Mọi trạng thái: SERVO+ còn điện khi K5 hạ hoặc ESTOP_OK thấp quá stuck_s (mặc định max(0,2 s,
    2 x servo_discharge_s)) -> lỗi "K1/K4 dính"; K1_AUX đóng khi đó quá 0,2 s -> lỗi "K1 dính"; ESTOP_OK cao khi K5 hạ quá
    0,2 s -> lỗi "K2/K5 dính". Lỗi giữ tới clear_fault().

    Giới hạn: phần mềm lấy mẫu mỗi chu kỳ poll (10 ms). Nếu E-stop được nhấn-nhả VÀ Reset được bấm trong cùng một chu kỳ
    (hoặc luồng giám sát bị treo lâu hơn kt_s), K2 chốt lại, K1 đóng lại sau kt_s mà phần mềm không thấy ESTOP_OK rớt;
    servo mất điện trong khoảng đó nên mô-men về tắt, SDK vẫn tưởng còn bật. Không sửa được bằng logic mức.
    """

    hw = HW_BANG

    def __init__(self, gpio: GPIO, on_stop=None, enable_pin: str = "K5_EN", run_pin: str = "ESTOP_OK",
                 arm_pin: str = "ARM_SENSE", reset_pin: str | None = "RESET_SENSE", k1_aux_pin: str | None = None,
                 clock=time.monotonic, sleep=time.sleep, kt_s: float = _spec.EVT_KT_S,
                 arm_confirm_s: float = _spec.EVT_ARM_CONFIRM_S, servo_discharge_s: float | None = _spec.EVT_SERVO_DISCHARGE_S,
                 stuck_s: float | None = None, clear_clean_s: float = 1.0, bench: bool = False,
                 reset_timeout_s: float = 30.0, wait_step_s: float = 0.01, hw: str | None = None,
                 config_error: str | None = None):
        self._init_common(gpio, on_stop, clock, sleep)
        self.config_error = config_error     # lỗi khai báo phát hiện trước (vd số xả tụ nhập sai): reset() từ chối
        self.enable_pin, self.run_pin, self.arm_pin = enable_pin, run_pin, arm_pin
        self.reset_pin, self.k1_aux_pin = reset_pin, k1_aux_pin
        self.kt_s, self.arm_confirm_s, self.servo_discharge_s = kt_s, arm_confirm_s, servo_discharge_s
        base = _spec.EVT_STUCK_S
        self.relay_stuck_s = base if stuck_s is None else stuck_s
        self.stuck_s = (max(base, 2 * (servo_discharge_s or 0.0)) if stuck_s is None else stuck_s)   # ARM_SENSE: chờ tụ xả
        self.clear_clean_s, self.bench = clear_clean_s, bench
        self.reset_timeout_s, self.wait_step_s = reset_timeout_s, wait_step_s
        self.fault: str | None = None
        self._since: dict[str, float] = {}
        self._en = False
        self.gpio.write(enable_pin, False)
        self.state = "stopped"
        self._event("khởi động: K5 hạ, chờ reset()")
        if bench:
            import warnings
            warnings.warn("SafetyLatch bộ #0 chế độ THỬ BÀN (bench=True): không cắm servo; xác nhận K1 dựa vào thời gian, "
                          "không dùng khi có servo", RuntimeWarning, stacklevel=3)
            self._event("CẢNH BÁO: chế độ thử bàn, không cắm servo")

    # ---------- đọc ----------
    def _read(self):
        run = bool(self.gpio.read(self.run_pin))
        arm = bool(self.gpio.read(self.arm_pin))
        rst = None if self.reset_pin is None else bool(self.gpio.read(self.reset_pin))
        aux = None if self.k1_aux_pin is None else bool(self.gpio.read(self.k1_aux_pin))
        return run, arm, rst, aux

    def _held(self, key: str, cond: bool, now: float) -> float:
        """Số giây cond đúng liên tục (-1 nếu đang sai)."""
        if not cond:
            self._since.pop(key, None)
            return -1.0
        return now - self._since.setdefault(key, now)

    def _check_faults(self, run: bool, arm: bool, aux, now: float) -> None:
        off = not self._en or not run
        k1 = self._held("k1", arm and off, now)
        k1a = self._held("k1aux", bool(aux) and off, now)
        k2 = self._held("k2", run and not self._en, now)
        if self.fault is None:
            if k1a >= self.relay_stuck_s:
                self._set_fault("K1_AUX đóng khi K5 hạ hoặc K2 chưa chốt: K1 dính tiếp điểm")
            elif k1 >= self.stuck_s:
                self._set_fault("SERVO+ còn điện khi K5 hạ hoặc K2 chưa chốt: nghi K1 hoặc K4 dính tiếp điểm")
            elif k2 >= self.relay_stuck_s:
                self._set_fault("ESTOP_OK cao khi K5 hạ: nghi K2 hoặc K5 dính (mất chốt)")

    def _set_fault(self, why: str) -> None:
        self.fault = why
        self.trip(why)
        self.state = "fault"

    # ---------- dừng ----------
    def poll(self) -> bool:
        """Đọc cảm biến; trả True nếu được chạy."""
        with self._lock:
            run, arm, _, aux = self._read()
            now = self.last_poll = self.clock()
            if self.state == "run":
                if not run:
                    self.trip("ESTOP_OK rớt: E-stop nhấn, K2 nhả hoặc mất 12 V")
                elif not arm:
                    self.trip("ARM_SENSE rớt: SERVO+ mất điện (K1 nhả)")
                elif aux is False:
                    self.trip("K1_AUX mở: K1 nhả")
            self._check_faults(run, arm, aux, now)
            return self.ok

    def trip(self, why: str) -> None:
        with self._lock:
            try:
                try:
                    self.gpio.write(self.enable_pin, False)   # hạ K5 trước, rồi mới tắt mô-men qua bus
                except BaseException:
                    try:
                        self.gpio.write(self.enable_pin, False)   # thử lại một lần
                    except BaseException:
                        close = getattr(self.gpio, "close", None)
                        if close is not None:          # nhả chân: K5 về điện trở kéo xuống ngoài
                            try:
                                close()
                            except BaseException:      # noqa: BLE001
                                pass
                        raise
            finally:
                self._en = False
                if self.state != "fault":
                    self.state = "estop"
                self._event(f"dừng: {why}")
                if self.on_stop is not None:
                    self.on_stop()

    # ---------- reset bằng tay ----------
    def config_problem(self) -> str | None:
        """Lý do cấu hình không đủ để xác nhận K1 đóng; None nếu đủ."""
        if self.config_error:
            return self.config_error
        if self.bench:
            return None
        if self.reset_pin is None:
            return "thiếu RESET_SENSE (nút Reset phải đọc được ở bộ #0); thử bàn không servo thì dùng bench=True"
        if self.k1_aux_pin is None:
            sd = self.servo_discharge_s
            if sd is not None and not (isinstance(sd, (int, float)) and math.isfinite(sd) and 0 < sd <= 5.0):
                return (f"thời gian xả tụ servo không hợp lệ ({sd!r} s; cần số hữu hạn, > 0, ≤ 5 s): nhập lại số đo "
                        "EVT-11 tính bằng ms, hoặc đi dây K1_AUX")
            if self.servo_discharge_s is None:
                return ("chưa đo thời gian xả tụ servo (ARM_SENSE rớt sau K4 nhả, Reset kẹt, có servo): đo bằng "
                        "estop_jig.py --che-do xa-tu rồi đặt servo_discharge_s, hoặc đi dây K1_AUX")
            if self.arm_confirm_s < 2 * self.servo_discharge_s:
                return (f"thời gian xả tụ {self.servo_discharge_s:.2f} s quá dài so với xác nhận {self.arm_confirm_s:.2f} s "
                        "(cần xác nhận >= 2 x xả): tăng arm_confirm_s, thêm tải xả SERVO+ hoặc đi dây K1_AUX")
        return None

    def reset(self, on_wait=None, timeout_s: float | None = None) -> None:
        """Bật K5 rồi chờ NGƯỜI nhấn và nhả nút Reset; chỉ trả về khi tín hiệu xác nhận K1 đã đóng."""
        tmo = self.reset_timeout_s if timeout_s is None else timeout_s
        with self._lock:
            if self.state == "run":
                return
            if self.state == "arming":
                raise ResetRefused("đang chờ reset ở luồng khác")
            if self.fault is not None:
                self._event(f"reset bị từ chối: lỗi {self.fault}")
                raise ResetRefused(f"{self.fault}: kiểm phần cứng rồi gọi clear_fault()")
            run, arm, rst, aux = self._read()
            refuse = (self.config_problem() or
                      ("ARM_SENSE cao khi K5 hạ: SERVO+ đang có điện, nghi K1/K4 dính" if arm else
                       "K1_AUX đóng khi K5 hạ: K1 dính" if aux else
                       "ESTOP_OK cao khi K5 hạ: nghi K2/K5 dính" if run else
                       "nút Reset đang nhấn hoặc kẹt: nhả nút rồi reset lại" if rst else None))
            if refuse:
                self._event(f"reset bị từ chối: {refuse}")
                raise ResetRefused(refuse)
            self.gpio.write(self.enable_pin, True)    # chỉ cho phép; K2 chỉ chốt khi người nhấn Reset
            self._en = True
            self.state = "arming"
            self._event("reset: K5 bật, chờ người nhấn và nhả nút Reset")
        try:
            if on_wait is not None:
                on_wait()
            self._wait_k1(tmo)
        except BaseException:
            with self._lock:
                if self.state == "arming":
                    self.trip("reset bị ngắt")
            raise

    def _wait_k1(self, tmo: float) -> None:
        t0 = self.clock()
        t_run = t_rel = t_ok = None
        while True:
            with self._lock:
                if self.state != "arming":           # luồng nền đã dừng (lỗi, trip) trong lúc chờ
                    raise ResetRefused(f"dừng trong lúc chờ reset: {self.log[-1][1]}")
                run, arm, rst, aux = self._read()
                now = self.clock()
                self._check_faults(run, arm, aux, now)
                if self.state != "arming":
                    raise ResetRefused(f"dừng trong lúc chờ reset: {self.fault}")
                if run:
                    t_run = now if t_run is None else t_run
                    if not rst and t_rel is None:
                        t_rel = now                  # lần đầu thấy Reset nhả sau khi K2 chốt (không có RESET_SENSE: lúc chốt)
                elif t_run is not None:              # K2 đã chốt rồi nhả: E-stop nhấn (kể cả nhấn-nhả nhanh) hoặc mất 12 V
                    self.trip("ESTOP_OK rớt trong lúc reset")
                    raise ResetRefused("E-stop nhấn hoặc mất 12 V trong lúc reset: nhả, kiểm vùng làm việc rồi reset lại")
                if rst:
                    t_rel = None                     # nhấn lại: tính lại mốc nhả
                due = None if t_run is None or t_rel is None else max(t_run + self.kt_s, t_rel)
                good = run and arm and not rst and aux is not False
                if due is not None and now >= due and good:
                    t_ok = now if t_ok is None else t_ok
                    if now - t_ok >= self.arm_confirm_s:
                        self.state = "run"
                        self._event("reset: ESTOP_OK, ARM_SENSE" + (", K1_AUX" if aux else "") + " xác nhận K1 đóng, chạy")
                        return
                else:
                    t_ok = None
                if now - t0 > tmo:
                    self.trip("hết giờ chờ Reset: K1 không đóng (Reset kẹt, chưa bấm, hoặc KT/K1 hỏng)")
                    raise ResetRefused("không thấy K1 đóng sau Reset: kiểm nút Reset (kẹt?), KT, K1 rồi reset lại")
            self._sleep(self.wait_step_s)

    def clear_fault(self) -> None:
        """Người kiểm xong phần cứng xoá lỗi dính. Chỉ khi đang "fault"; hạ K5 trước; tín hiệu phải sạch liên tục
        clear_clean_s (ARM_SENSE, ESTOP_OK, K1_AUX đều thấp) mới xoá."""
        with self._lock:
            if self.state != "fault":
                raise ResetRefused(f"không có lỗi để xoá (trạng thái {self.state})")
            self.gpio.write(self.enable_pin, False)
            self._en = False
            t0 = self.clock()
        while True:
            with self._lock:
                if self.state != "fault":
                    raise ResetRefused("trạng thái đổi trong lúc xoá lỗi")
                run, arm, _, aux = self._read()
                if arm or run or aux:
                    sig = "ARM_SENSE" if arm else "ESTOP_OK" if run else "K1_AUX"
                    self._event(f"xoá lỗi bị từ chối: {sig} cao khi K5 hạ")
                    raise ResetRefused(f"chưa xoá được lỗi: {sig} vẫn cao khi K5 hạ")
                if self.clock() - t0 >= self.clear_clean_s:
                    self._event(f"xoá lỗi: {self.fault}")
                    self.fault = None
                    self._since.clear()
                    self.state = "stopped"
                    return
            self._sleep(self.wait_step_s)


# ---------------- chốt hãm trục nâng thanh răng (v0.5, R2 7.2, DK-N17) ----------------
class DockSwitches:
    """Hai công tắc khoá kéo của trạm hai tay (R3b 5.7): DOCK_L, DOCK_R (GPIO qua opto, True = khoá đã đóng). Phần cứng đã nối tiếp hai
    công tắc vào vòng E-stop (thiếu một khoá thì K1 không đóng); SDK đọc riêng từng khoá để báo khoá nào chưa đóng và chặn enable/lift/đế.
    Đứt dây hoặc opto hỏng đọc là "mở" (an toàn)."""

    def __init__(self, gpio: GPIO, pins=_spec.DOCK_PINS):
        self.gpio, self.pins = gpio, tuple(pins)

    def state(self) -> dict:
        st = {p: bool(self.gpio.read(p)) for p in self.pins}
        st["ok"] = all(st[p] for p in self.pins)
        return st


def dock_state(dock) -> dict:
    """Trạng thái hai khoá trạm: {"DOCK_L": bool, "DOCK_R": bool, "ok": bool}; dock None (chưa khai chân) -> {"ok": None}."""
    return {"ok": None} if dock is None else dock.state()


class LiftLatchError(RuntimeError):
    """Chốt hãm trục nâng không theo đúng trình tự (cảm biến không xác nhận, kẹt, rơi chốt khi chạy)."""


class SimLiftLatchHW(GPIO):
    """Phần cứng chốt giả lập cho test: cuộn 12 V lấy từ SERVO+ sau K1 (servo_power()), chân LIFT_LATCH_EN (Pi), cảm biến
    LIFT_LATCH_RETRACTED (True = chốt đã rút, mức LOGIC sau active_low). Cuộn có điện khi EN và SERVO+; cảm biến theo cuộn sau
    delay_reads lần đọc. Lỗi: stuck = "engaged" | "retracted" (chốt hoặc cảm biến kẹt), sensor_dead (đứt dây: luôn "không rút")."""

    def __init__(self, servo_power=lambda: True, delay_reads: int = 2, coil_pin: str = "LIFT_LATCH_EN",
                 sensor_pin: str = "LIFT_LATCH_RETRACTED"):
        self.servo_power, self.delay_reads = servo_power, delay_reads
        self.coil_pin, self.sensor_pin = coil_pin, sensor_pin
        self.en = False
        self.stuck: str | None = None
        self.sensor_dead = False
        self._pos = False            # True = chốt đang rút
        self._pending = 0
        self.history: list[tuple[str, bool]] = []

    @property
    def coil(self) -> bool:
        return self.en and bool(self.servo_power())

    def read(self, pin: str) -> bool:
        if pin != self.sensor_pin:
            raise KeyError(pin)
        want = self.coil
        if self.stuck == "engaged":
            want = False
        elif self.stuck == "retracted":
            want = True
        if want != self._pos:
            if self._pending >= self.delay_reads:
                self._pos, self._pending = want, 0
            else:
                self._pending += 1
        else:
            self._pending = 0
        return False if self.sensor_dead else self._pos

    def write(self, pin: str, level: bool) -> None:
        if pin != self.coil_pin:
            raise KeyError(pin)
        self.en = bool(level)
        self.history.append((pin, bool(level)))

    def has(self, pin: str) -> bool:
        return pin in (self.coil_pin, self.sensor_pin)


class LiftLatch:
    """Chốt solenoid trục nâng: cuộn có điện = chốt rút (lift được chạy); cuộn mất điện (Pi hạ chân, E-stop cắt SERVO+, mất
    điện) = lò xo đẩy chốt cài vào răng (an toàn khi hỏng). Trình tự (R2 7.2), backend cấp hai hàm:
      hold(): bật mô-men servo lift, giữ vị trí hiện tại;  torque_off(): tắt mô-men riêng servo lift.
    - release(hold): kiểm cảm biến đang "cài" -> hold() -> cuộn có điện -> chờ cảm biến "rút" -> state "retracted" (mới cho chạy).
    - engage(hold, torque_off): hold() -> cuộn mất điện -> chờ cảm biến "cài" -> torque_off() -> state "engaged".
      Cảm biến không xác nhận "cài": KHÔNG tắt mô-men lift (servo còn giữ), lỗi.
    - cut(): E-stop / SafetyLatch trip: hạ chân cuộn ngay (phần cứng đã cắt cùng SERVO+), không chờ.
    - motion_allowed(): chỉ True khi "retracted" và cảm biến đang báo rút; cảm biến rớt khi đang chạy -> lỗi (chốt rơi vào răng).
    Lỗi giữ tới clear_fault(). Một cảm biến chỉ biết "rút / không rút": chốt ra nhưng tì đỉnh răng vẫn đọc là "cài", lift có thể
    rơi tối đa một nấc (spec.LIFT_NOTCH_PITCH) khi tắt mô-men: đúng tiêu chí thử rơi DK-N4, không phát hiện được bằng cảm biến này.
    """

    def __init__(self, gpio: GPIO, coil_pin: str = "LIFT_LATCH_EN", sensor_pin: str = "LIFT_LATCH_RETRACTED",
                 sleep=time.sleep, retract_timeout_s: float = _spec.LIFT_LATCH_RETRACT_TIMEOUT_S,
                 engage_timeout_s: float = _spec.LIFT_LATCH_ENGAGE_TIMEOUT_S, confirm_s: float = _spec.LIFT_LATCH_CONFIRM_S,
                 step_s: float = 0.01):
        self.gpio, self.coil_pin, self.sensor_pin = gpio, coil_pin, sensor_pin
        self._sleep = sleep
        self.retract_timeout_s, self.engage_timeout_s = retract_timeout_s, engage_timeout_s
        self.confirm_n = max(1, math.ceil(confirm_s / step_s))
        self.step_s = step_s
        self.fault: str | None = None
        self.log: list[str] = []
        self._lock = threading.RLock()
        self.gpio.write(coil_pin, False)
        self.state = "engaged"           # khởi động: cuộn mất điện, coi như cài (release() kiểm lại cảm biến)

    def retracted(self) -> bool:
        return bool(self.gpio.read(self.sensor_pin))

    def _wait(self, want: bool, timeout_s: float) -> bool:
        n_ok, n_max = 0, max(1, math.ceil(timeout_s / self.step_s))
        for _ in range(n_max + self.confirm_n):
            n_ok = n_ok + 1 if self.retracted() == want else 0
            if n_ok >= self.confirm_n:
                return True
            self._sleep(self.step_s)
        return False

    def _fail(self, why: str) -> None:
        self.fault = why
        self.state = "fault"
        self.log.append(f"lỗi: {why}")
        raise LiftLatchError(why)

    def _abort_if_stopped(self, still_ok) -> None:
        """E-stop (cut() từ luồng giám sát, không lấy khoá) có thể đến giữa chừng: kiểm lại, hạ cuộn, dừng rút chốt."""
        if still_ok is not None and not still_ok():
            self.gpio.write(self.coil_pin, False)
            self.state = "engaged"
            self.log.append("dừng khẩn trong lúc rút chốt: hạ cuộn")
            raise LiftLatchError("dừng khẩn (E-stop) trong lúc rút chốt")

    def release(self, hold, still_ok=None) -> None:
        """still_ok(): False khi chốt E-stop phần mềm không còn ở "run"; kiểm sau hold(), sau khi bật cuộn và sau khi cảm biến
        báo rút, để E-stop giữa chừng không để lại cuộn có điện hay mô-men lift bật lại (backend tắt mô-men lần nữa)."""
        with self._lock:
            if self.fault:
                raise LiftLatchError(f"chốt trục nâng đang lỗi: {self.fault} (kiểm rồi clear_fault())")
            if self.state == "retracted" and self.retracted():
                return
            self.gpio.write(self.coil_pin, False)
            if self.retracted():
                self._fail("cảm biến báo chốt đã rút khi cuộn chưa có điện: cảm biến hỏng/đảo mức hoặc chốt kẹt rút")
            self._abort_if_stopped(still_ok)
            hold()                                   # 1. servo lift giữ vị trí hiện tại, nhận tải trước khi chốt rời răng
            self._abort_if_stopped(still_ok)
            self.gpio.write(self.coil_pin, True)     # 2. rút chốt
            self.state = "retracting"
            self._abort_if_stopped(still_ok)
            if not self._wait(True, self.retract_timeout_s):   # 3. chờ cảm biến "rút"
                self.gpio.write(self.coil_pin, False)
                self.state = "engaged"
                self._abort_if_stopped(still_ok)
                self._fail(f"chốt không rút sau {self.retract_timeout_s:.2f} s (kẹt, cuộn hỏng, SERVO+ mất điện hoặc cảm biến hỏng)")
            self._abort_if_stopped(still_ok)
            self.state = "retracted"                 # 4. cho chạy
            self.log.append("chốt rút, cho chạy")

    def engage(self, hold, torque_off) -> None:
        with self._lock:
            if self.state == "engaged" and not self.retracted():
                torque_off()
                return
            hold()                                   # 1. servo giữ
            self.gpio.write(self.coil_pin, False)    # 2. cài chốt
            self.state = "engaging"
            if not self._wait(False, self.engage_timeout_s):   # 3. chờ cảm biến "cài"
                self._fail(f"chốt không cài sau {self.engage_timeout_s:.2f} s: servo lift VẪN GIỮ mô-men, kiểm chốt/cảm biến")
            torque_off()                             # 4. mới tắt mô-men lift
            self.state = "engaged"
            self.log.append("chốt cài, tắt mô-men lift")

    def cut(self) -> None:
        """E-stop: hạ chân cuộn ngay, không chờ cảm biến (SERVO+ đã mất nên cuộn mất điện cả khi chân hỏng).
        Cố ý KHÔNG lấy khoá: release()/engage() giữ khoá khi chờ cảm biến tới 0,5 s, E-stop không được đợi. release() kiểm lại
        still_ok() sau mỗi bước nên không bật lại cuộn sau cut()."""
        try:
            self.gpio.write(self.coil_pin, False)
        finally:
            if self.state != "fault":
                self.state = "engaged"
            self.log.append("E-stop: cắt cuộn chốt")

    def motion_allowed(self) -> bool:
        with self._lock:
            if self.state != "retracted":
                return False
            if not self.retracted():
                self.gpio.write(self.coil_pin, False)
                self.fault = "chốt rơi khi đang cho chạy (cảm biến hết báo rút): dừng lift"
                self.state = "fault"
                self.log.append(f"lỗi: {self.fault}")
                return False
            return True

    def clear_fault(self) -> None:
        with self._lock:
            self.gpio.write(self.coil_pin, False)
            self.fault = None
            self.state = "engaged"


# ---------------- phát hiện lò xo cân bằng vai đứt ----------------
_MODEL = {}


def shoulder_model_torque(q_shoulder: float, q_elbow: float, q_wrist: float = 0.0, spring_scale: float = 1.0,
                          payload: float = 0.0) -> float:
    """Mô-men servo vai (cả cặp, N·m, dương = giữ chống hạ) để đứng yên, theo mô hình MuJoCo có lò xo (spring_scale 1 = 2 lò xo)."""
    import mujoco
    from . import model as m1model
    key = (spring_scale, payload)
    if key not in _MODEL:
        m = mujoco.MjModel.from_xml_string(m1model.robot_mjcf(scene=False))
        m.tendon_stiffness[:] *= spring_scale
        m.body_mass[m.body("left_hand").id] += payload
        _MODEL[key] = (m, mujoco.MjData(m))
    m, d = _MODEL[key]
    for jn, v in (("shoulder", q_shoulder), ("elbow", q_elbow), ("wrist_flex", q_wrist)):
        d.qpos[m.joint(f"left_{jn}").qposadr[0]] = v
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    a = m.joint("left_shoulder").dofadr[0]
    return float(d.qfrc_bias[a] - d.qfrc_passive[a])


class SpringMonitor:
    """So mô-men vai đo (tổng cặp servo, khung khớp) với mô hình có lò xo; lệch > threshold_nm liên tục hold_s giây -> cảnh báo.

    Chỉ cảnh báo (không dừng): lò xo đứt làm servo gánh thêm, không làm tay rơi khi còn mô-men.
    Ngưỡng mặc định [giả định]: 0,6 N·m (một lò xo đứt ở tư thế ngang lệch ~1 N·m theo mô hình), 1 s.
    Chỉ dùng khi tay gần đứng yên (tải động làm lệch); update() bỏ qua mẫu khi |qd| > max_speed và ở tư thế
    mà một lò xo đứt đổi mô-men ít hơn ngưỡng (vai gần -80 độ hoặc >= 60 độ khi khuỷu duỗi).
    """

    def __init__(self, threshold_nm: float = 0.6, hold_s: float = 1.0, max_speed: float = math.radians(10),
                 payload: float = 0.0, model_fn=shoulder_model_torque):
        self.threshold, self.hold, self.max_speed = threshold_nm, hold_s, max_speed
        self.payload, self.model_fn = payload, model_fn
        self.t_bad = 0.0
        self.alarm = False
        self.message = ""
        self.residual = 0.0
        self.observable = False

    def update(self, q_shoulder: float, q_elbow: float, q_wrist: float, measured_nm: float, dt: float,
               speed: float = 0.0) -> bool:
        if abs(speed) > self.max_speed:
            return self.alarm
        expected = self.model_fn(q_shoulder, q_elbow, q_wrist, 1.0, self.payload)
        # tư thế mà một lò xo đứt đổi mô-men ít hơn ngưỡng (gần thẳng đứng): không phán, giữ bộ đếm
        if abs(expected - self.model_fn(q_shoulder, q_elbow, q_wrist, 0.5, self.payload)) < self.threshold:
            self.observable = False
            return self.alarm
        self.observable = True
        self.residual = measured_nm - expected
        if abs(self.residual) > self.threshold:
            self.t_bad += dt
        else:
            self.t_bad = 0.0
        if self.t_bad >= self.hold and not self.alarm:
            self.alarm = True
            self.message = (f"nghi lò xo cân bằng vai đứt/lỏng: mô-men đo {measured_nm:.2f} N·m, mô hình {expected:.2f} N·m "
                            f"(lệch {self.residual:+.2f}) quá {self.hold:.1f} s ở vai {math.degrees(q_shoulder):.0f}°")
        return self.alarm
