"""Thư viện chung cho kịch bản thử EVT Faibot M1 (product/evt/*.py).

Mỗi kịch bản chạy hai chế độ:
- bus thật: `--port /dev/ttyUSB0` (Pi 5 hoặc laptop + Waveshare Bus Servo Adapter), GPIO Pi qua `--pin TÊN=số_BCM`;
- chế độ khô: `--dry-run` dùng bus giả (EvtPort), GPIO giả, đồng hồ ảo. Số in ra ở chế độ khô là MÔ PHỎNG, không phải số đo.
Mạch dừng khẩn cấp: `--mach bang-di-day` (bộ #0, mặc định; chân K5_EN ra, ESTOP_OK và ARM_SENSE vào) hoặc `--mach pcb`
(PCB nguồn sau EVT; RELAY_EN ra, ESTOP_SENSE và LATCH_SENSE vào). Xem faibotic.safety.

Kết quả: product/evt/ket-qua/<serial>/<kịch bản>_<thời điểm>.json + .csv (một dòng mỗi tiêu chí).
Trạng thái tiêu chí: "dat", "khong_dat", "chua_kiem" (thiếu số, tính là KHÔNG ĐẠT khi ký phiếu), "ghi_nhan" (chỉ ghi số).

Giả định thanh ghi chưa đối chiếu tài liệu hãng (docs/vi/servo-datasheet.md mục 12 chưa có số liệu): firmware ở địa chỉ 0 và 1,
model ở 3–4, chế độ bước MODE = 3, dòng 6,5 mA/LSB. Kịch bản ghi số thô bên cạnh số quy đổi để đối chiếu lại sau.
"""
from __future__ import annotations

import argparse
import atexit
import csv
import datetime as _dt
import json
import math
import os
import re
import signal
import sys
import time
from pathlib import Path

EVT_DIR = Path(__file__).resolve().parent
ROOT = EVT_DIR.parents[1]
if str(ROOT / "sdk") not in sys.path:
    sys.path.insert(0, str(ROOT / "sdk"))

from faibotic import spec  # noqa: E402
from faibotic.feetech import protocol as P  # noqa: E402
from faibotic.feetech.bus import FeetechBus  # noqa: E402
from faibotic.feetech.mockbus import FakeServoPort  # noqa: E402
from faibotic.gpio_pi import PiGPIO as _SdkPiGPIO, _kv as _gpio_kv, parse_discharge_ms  # noqa: E402
from faibotic.safety import GPIO, FakeGPIO, SimBangDiDay  # noqa: E402,F401

# chân GPIO theo mạch dừng khẩn cấp (tên dùng trong --pin); chân Pi chưa chốt, đo theo dây WE07 (bộ #0) / W15 (PCB)
GPIO_PINS = {"bang_di_day": dict(required=("K5_EN", "ESTOP_OK", "ARM_SENSE", "RESET_SENSE"), outputs=("K5_EN",)),
             "pcb": dict(required=("ESTOP_SENSE", "LATCH_SENSE", "RELAY_EN"), outputs=("RELAY_EN",))}

# [giả định, đối chiếu bảng thanh ghi Feetech STS ở EVT]
FW_MAJOR = (0, 1)
FW_MINOR = (1, 1)
MODEL_NUMBER = (3, 2)
MODE_STEP = 3
CURRENT_MA_PER_LSB = 6.5
VOLT_PER_LSB = 0.1

# Bộ #0 EVT lắp theo cấu hình ghép đôi P-A (v0.4.x, 20 servo, pan STS3215): kịch bản EVT dùng bản đồ này, KHÔNG theo P-L mặc định của SDK
# (P-L: 16 servo, pan STS3095). Bộ P-L cần chạy lại kịch bản với EVT_CONFIG = "P-L" [chưa làm: chờ bộ P-L].
EVT_CONFIG = "P-A"
EVT_PAIRED_JOINTS = spec.PA_PAIRED_JOINTS if EVT_CONFIG == "P-A" else ()
EVT_IDS = {**spec.SERVO_IDS, **(spec.PA_PAIR_IDS if EVT_CONFIG == "P-A" else {})}
EVT_PAIRS = {spec.joint_name(s, j): spec.pair_name(spec.joint_name(s, j)) for s in spec.SIDES for j in EVT_PAIRED_JOINTS}
STS3095_NAMES = {n for n in EVT_IDS
                 if n == "lift" or any(n.endswith(j) or n.endswith(j + spec.PAIR_SUFFIX) for j in EVT_PAIRED_JOINTS)}
SERIAL_RE = re.compile(r"^FM1-\d{4}-\d{3}$")
SERIAL_HINT = "FM1-YYMM-NNN (bộ EVT chưa qua FQC: FM1-0000-000)"


class Abort(RuntimeError):
    """Người đo dừng kịch bản hoặc điều kiện an toàn chưa đạt."""


def servo_type(name: str) -> str:
    return "STS3095" if name in STS3095_NAMES else "STS3215"


# ---------------- bus giả cho chế độ khô ----------------
class EvtPort(FakeServoPort):
    """Bus giả mở rộng: model/firmware theo loại servo, chế độ bước (đa vòng) bật/tắt, mất điện, nhiệt, dòng.

    multiturn: True = MODE 3 nhận đích có dấu ngoài 0..4095; False = servo kẹp về 0..4095 (để kịch bản thấy nhánh "không hỗ trợ").
    retain: đa vòng có giữ số đếm sau mất điện không.
    """

    MODEL_SIM = {"STS3215": 0x0309, "STS3095": 0x0A0B}   # số giả, không phải số hãng

    def __init__(self, ids=None, multiturn: bool = True, retain: bool = False):
        ids = dict(EVT_IDS if ids is None else ids)
        super().__init__(list(ids.values()))
        self.multiturn, self.retain = multiturn, retain
        for n, sid in ids.items():
            m = self.mem[sid]
            m[FW_MAJOR[0]], m[FW_MINOR[0]] = 3, 10
            m[MODEL_NUMBER[0]:MODEL_NUMBER[0] + 2] = P.u16(self.MODEL_SIM[servo_type(n)])
            m[P.MAX_ANGLE[0]:P.MAX_ANGLE[0] + 2] = P.u16(4095)
        self.temp = {sid: 30.0 for sid in self.mem}

    def _tick(self, sid) -> None:
        m = self.mem[sid]
        mode = m[P.MODE[0]]
        if mode == P.MODE_WHEEL or not m[P.TORQUE_ENABLE[0]] or sid in self.stuck:
            return super()._tick(sid)
        goal = P.from_u16(m[P.GOAL_POSITION[0]:P.GOAL_POSITION[0] + 2])
        if mode == MODE_STEP and self.multiturn:
            raw = goal                                  # có dấu bit 15, servo đếm nhiều vòng
        else:
            g = P.decode_signed(goal) if mode == MODE_STEP else goal
            lo = P.from_u16(m[P.MIN_ANGLE[0]:P.MIN_ANGLE[0] + 2])
            hi = P.from_u16(m[P.MAX_ANGLE[0]:P.MAX_ANGLE[0] + 2]) or 4095
            raw = max(lo, min(hi, max(0, min(4095, g))))
        m[P.PRESENT_POSITION[0]:P.PRESENT_POSITION[0] + 2] = P.u16(raw)
        m[P.PRESENT_CURRENT[0]:P.PRESENT_CURRENT[0] + 2] = P.u16(25)

    def power_cycle(self) -> None:
        """Mất điện rồi cấp lại: RAM về mặc định, encoder một vòng (nếu không giữ đếm)."""
        for sid, m in self.mem.items():
            m[P.TORQUE_ENABLE[0]] = 0
            m[P.LOCK[0]] = 1
            m[P.TORQUE_LIMIT[0]:P.TORQUE_LIMIT[0] + 2] = P.u16(1000)
            raw = P.decode_signed(P.from_u16(m[P.PRESENT_POSITION[0]:P.PRESENT_POSITION[0] + 2]))
            if not (self.retain and m[P.MODE[0]] == MODE_STEP):
                m[P.PRESENT_POSITION[0]:P.PRESENT_POSITION[0] + 2] = P.u16(raw % 4096)

    def advance(self, dt: float) -> None:
        """Nhiệt: tiến tới 30 °C (tắt) hoặc 44 °C (bật mô-men, có tải), hằng số thời gian 300 s [mô phỏng]."""
        for sid, m in self.mem.items():
            target = 44.0 if m[P.TORQUE_ENABLE[0]] else 30.0
            self.temp[sid] += (target - self.temp[sid]) * (1 - math.exp(-dt / 300.0))
            m[P.PRESENT_TEMPERATURE[0]] = int(round(self.temp[sid]))


# ---------------- GPIO thật trên Pi 5 ----------------
class PiGPIO(_SdkPiGPIO):
    """GPIO Pi 5 cho kịch bản EVT: lớp faibotic.gpio_pi.PiGPIO (lgpio hoặc gpiod v2, nạp lười). Mức tích cực và điện trở kéo
    lấy CÙNG cấu hình với SDK (FAIBOTIC_GPIO_ACTIVE_LOW, FAIBOTIC_GPIO_PULL; LIFT_* mặc định kéo lên), chỉ cho chân đã khai
    bằng --pin. CHƯA chạy trên Pi thật; chân chưa chốt nên không có mặc định. Mọi PiGPIO tạo ra được ghi để release_all()
    hạ ngõ ra và nhả chân khi kịch bản thoát (kể cả lỗi, Ctrl-C, SIGTERM).

    pins: {"ESTOP_OK": 17, ...} số BCM; tên trong outputs là đầu ra (K5_EN cho bộ #0, RELAY_EN cho PCB), còn lại là đầu vào.
    Đầu ra khởi tạo mức thấp (K5 hở; RELAY_EN có R21 100 kΩ kéo xuống trên PCB). hw: mạch mà SafetyLatch sẽ dùng.
    chip None: tự tìm chip nhãn pinctrl-rp1 (Pi 5). Kịch bản tự kiểm đủ chân nên ở đây không bắt đủ bộ chân của mạch.
    """

    def __init__(self, pins: dict[str, int], chip: str | None = None, outputs=("RELAY_EN",), hw: str | None = None,
                 backend: str | None = None, env=None):
        if hw not in ("bang_di_day", "pcb"):
            raise ValueError(f"PiGPIO cần hw='bang_di_day' hoặc 'pcb' (mạch dừng khẩn cấp), nhận {hw!r}")
        env = os.environ if env is None else env
        al = [p.strip() for p in env.get("FAIBOTIC_GPIO_ACTIVE_LOW", "").split(",") if p.strip() in pins]
        pull = {k: v for k, v in _gpio_kv(env.get("FAIBOTIC_GPIO_PULL", "")).items() if k in pins}
        super().__init__(pins, hw=hw, outputs=tuple(o for o in outputs if o in pins), chip=chip or env.get("FAIBOTIC_GPIO_CHIP"),
                         backend=backend or env.get("FAIBOTIC_GPIO_BACKEND", "auto"), active_low=al, pull=pull,
                         consumer="faibot-evt", require_pins=False)
        _OPEN_GPIO.append(self)


_OPEN_GPIO: list = []


def release_all() -> None:
    """Hạ mọi ngõ ra (K5_EN / RELAY_EN) rồi nhả chân của các PiGPIO kịch bản đã mở. Gọi khi thoát; gọi lại không sao.
    Chỉ có tác dụng khi tiến trình còn chạy được: kill -9 hay Pi treo thì phần cứng phải nhả K1 (bang-di-day-evt.md, dòng
    "Pi treo": cần watchdog phần cứng cho K5, DK-S4)."""
    while _OPEN_GPIO:
        g = _OPEN_GPIO.pop()
        try:
            g.close()                     # PiGPIO.close() hạ ngõ ra trước khi nhả chân
        except Exception as e:            # noqa: BLE001
            print(f"  ! không đóng được GPIO: {e!r}")


atexit.register(release_all)


class INA226:
    """INA226 trên I2C Pi (smbus2). U1 0x40: bus servo sau K1, shunt 1 mΩ, LSB 2,5 mA; U2 0x41: đầu vào buck, 20 mΩ, 0,2 mA
    (electrical/tinh_toan.py mục 11). CHƯA chạy trên phần cứng thật."""

    def __init__(self, addr: int = 0x40, r_shunt: float = 0.001, current_lsb: float = 0.0025, bus: int = 1):
        from smbus2 import SMBus
        self.addr, self.lsb = addr, current_lsb
        self.i2c = SMBus(bus)
        self._w(0x05, int(round(0.00512 / (current_lsb * r_shunt))))

    def _w(self, reg, v):
        self.i2c.write_word_data(self.addr, reg, ((v & 0xFF) << 8) | (v >> 8))

    def _r(self, reg):
        v = self.i2c.read_word_data(self.addr, reg)
        return ((v & 0xFF) << 8) | (v >> 8)

    def read(self) -> tuple[float, float]:
        """(điện áp bus V, dòng A)."""
        i = self._r(0x04)
        i = i - 65536 if i & 0x8000 else i
        return self._r(0x02) * 1.25e-3, i * self.lsb


def require_mode0(bus, sid) -> None:
    """Trước khi bật mô-men servo rời: MODE phải là 0 (vị trí). Servo còn MODE 1/3 (vd sau lift_da_vong --giu-mode) quay khác dự kiến."""
    m = bus.read(sid, P.MODE)
    if m != P.MODE_POSITION:
        raise Abort(f"ID {sid} đang MODE {m}, không phải 0 (vị trí): không bật mô-men. Đặt lại MODE 0 bằng lift_da_vong/bộ test rồi chạy lại")


# ---------------- khung chạy ----------------
def parser(desc: str) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--port", default="/dev/ttyUSB0", help="cổng bus servo (mặc định /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--dry-run", action="store_true", help="chạy khô với bus/GPIO giả; số là mô phỏng")
    ap.add_argument("--serial", default=None, help=SERIAL_HINT)
    ap.add_argument("--nguoi-do", default="", help="họ tên người đo")
    ap.add_argument("--out", default=str(EVT_DIR / "ket-qua"), help="thư mục kết quả gốc")
    ap.add_argument("--answers", default=None, help="file JSON điền sẵn các số đo tay (khoá = mã câu hỏi)")
    ap.add_argument("--pin", action="append", default=[], metavar="TEN=BCM",
                    help="chân GPIO Pi, ví dụ --pin K5_EN=22 --pin ESTOP_OK=17 --pin ARM_SENSE=27 (bộ #0; chưa chốt chân)")
    ap.add_argument("--xa-tu-ms", default=None,
                    help="bộ #0: thời gian xả tụ servo đã đo (estop_jig.py --che-do xa-tu), bắt buộc trước khi bật mô-men nếu không có K1_AUX")
    ap.add_argument("--mach", choices=("bang-di-day", "pcb"), default="bang-di-day",
                    help="mạch dừng khẩn cấp: bang-di-day = bộ #0 EVT (mặc định), pcb = PCB nguồn sau EVT")
    ap.add_argument("--gpiochip", default=None, help="mặc định tự tìm chip pinctrl-rp1 (Pi 5)")
    return ap


class Ctx:
    def __init__(self, args, script: str, codes: list[str], mock_kwargs: dict | None = None):
        self.args, self.script, self.codes = args, script, codes
        self.dry = bool(args.dry_run)
        serial = args.serial or ("FM1-0000-000" if self.dry else None)
        if not serial or not SERIAL_RE.match(serial):
            raise SystemExit(f"--serial phải có dạng {SERIAL_HINT}, nhận {serial!r}")
        if not self.dry and not args.nguoi_do:
            raise SystemExit("--nguoi-do bắt buộc khi đo thật")
        self.serial = serial
        self.answers = json.loads(Path(args.answers).read_text(encoding="utf-8")) if args.answers else {}
        self.checks: list[dict] = []
        self.data: dict = {}
        self.t_start = _dt.datetime.now()
        self._vt = 0.0
        self.port = EvtPort(**(mock_kwargs or {})) if self.dry else None
        self.pins = {}
        for kv in args.pin:
            k, _, v = kv.partition("=")
            self.pins[k.strip()] = int(v)
        print(f"== {script} | {', '.join(codes)} | serial {serial} | {'CHẠY KHÔ (mô phỏng)' if self.dry else 'BUS THẬT ' + args.port}")

    # thời gian: chế độ khô dùng đồng hồ ảo để burn-in 30 phút chạy trong vài giây
    def now(self) -> float:
        return self._vt if self.dry else time.monotonic()

    def sleep(self, dt: float) -> None:
        if self.dry:
            self._vt += max(0.0, dt)
            self.port.advance(max(0.0, dt))
        elif dt > 0:
            time.sleep(dt)

    def say(self, msg: str) -> None:
        print("  " + msg)

    def ask(self, key: str, prompt: str, sim, kind=float):
        """Số đo tay. Thứ tự: file --answers, chế độ khô (giá trị mô phỏng), nhập bàn phím. Bỏ trống = None (chưa kiểm)."""
        if key in self.answers:
            v = self.answers[key]
        elif self.dry:
            v = sim
        else:
            s = input(f"  ? {prompt} [{key}] (bỏ trống = chưa đo): ").strip().replace(",", ".")
            if s == "":
                return None
            v = s
        if v is None:
            return None
        if kind is bool:
            return v if isinstance(v, bool) else str(v).lower() in ("1", "c", "co", "có", "y", "yes", "dat", "đạt", "true")
        return kind(v)

    def confirm(self, msg: str) -> None:
        """Điều kiện an toàn: đo thật phải gõ 'co'; chế độ khô chỉ in."""
        print(f"  ! {msg}")
        if self.dry or self.answers.get("_xac_nhan_tat_ca") is True:
            return
        if input("    Gõ 'co' để xác nhận: ").strip().lower() not in ("co", "có"):
            raise Abort(f"chưa xác nhận: {msg}")

    def action(self, msg: str, sim=None) -> None:
        """Yêu cầu người đo thao tác (nhấn nút, treo tải...). Chế độ khô gọi sim() thay người."""
        print(f"  > {msg}")
        if self.dry:
            if sim:
                sim()
        elif not self.answers.get("_xac_nhan_tat_ca"):
            input("    Enter khi xong...")

    def check(self, code: str, item: str, value, threshold: str, ok: bool | None, unit: str = "", note: str = "") -> bool | None:
        status = "chua_kiem" if value is None else ("ghi_nhan" if ok is None else ("dat" if ok else "khong_dat"))
        if isinstance(value, float):
            value = round(value, 4)
        self.checks.append(dict(ma=code, hang_muc=item, gia_tri=value, don_vi=unit, nguong=threshold, trang_thai=status, ghi_chu=note))
        tag = {"dat": "ĐẠT", "khong_dat": "KHÔNG ĐẠT", "chua_kiem": "CHƯA KIỂM", "ghi_nhan": "ghi"}[status]
        print(f"  [{tag:9}] {code} {item}: {value} {unit} (ngưỡng {threshold}) {note}".rstrip())
        return ok

    # ---------- phần cứng ----------
    def bus(self) -> FeetechBus:
        return FeetechBus(self.port if self.dry else self.args.port, baudrate=self.args.baud)

    @property
    def hw(self) -> str:
        return getattr(self.args, "mach", "bang-di-day").replace("-", "_")

    def gpio(self, sim_factory=None, required=None):
        cfg = GPIO_PINS[self.hw]
        if self.dry:
            if sim_factory:
                return sim_factory()
            return SimBangDiDay(clock=self.now) if self.hw == "bang_di_day" else FakeGPIO()
        miss = [p for p in (cfg["required"] if required is None else required) if p not in self.pins]
        if miss:
            raise SystemExit(f"thiếu chân GPIO {miss}: thêm --pin TÊN=BCM (chân Pi chưa chốt trong thiết kế, "
                             f"đo theo dây {'WE07' if self.hw == 'bang_di_day' else 'W15'})")
        return PiGPIO(self.pins, chip=self.args.gpiochip, outputs=cfg["outputs"], hw=self.hw)

    def latch(self, g, on_stop=None, bench: bool = False):
        """SafetyLatch theo --mach, đồng hồ và sleep của kịch bản (đồng hồ ảo khi chạy khô).

        Bộ #0: RESET_SENSE bắt buộc; K1_AUX nếu có --pin K1_AUX; thời gian xả tụ từ --xa-tu-ms (chạy khô: số của mạch giả).
        bench=True chỉ cho thử bàn KHÔNG cắm servo (estop_jig)."""
        from faibotic.safety import SafetyLatch
        kw = dict(on_stop=on_stop, hw=self.hw, clock=self.now, sleep=self.sleep)
        if self.hw == "bang_di_day":
            raw = getattr(self.args, "xa_tu_ms", None)
            xa, err = parse_discharge_ms(raw) if raw is not None else (spec.EVT_SERVO_DISCHARGE_S, None)
            if err:
                kw["config_error"] = "--xa-tu-ms: " + err        # SafetyLatch từ chối reset, K5 giữ hạ
            if self.dry and isinstance(g, SimBangDiDay) and xa is None:
                xa = g.cap_hold_s
            aux = "K1_AUX" in self.pins or (isinstance(g, SimBangDiDay) and g.k1_aux)
            kw.update(reset_pin="RESET_SENSE", k1_aux_pin="K1_AUX" if aux else None, servo_discharge_s=xa, bench=bench)
        return SafetyLatch(g, **kw)

    def reset_latch(self, lt, g) -> None:
        """Reset bằng tay: bộ #0 thì phần mềm bật K5 rồi người nhấn và nhả nút Reset trên thân; PCB thì cạnh RELAY_EN."""
        sim = g if isinstance(g, SimBangDiDay) else None
        msg = ("Nhấn rồi nhả nút Reset trên thân (K5 đã bật; phần mềm chờ ARM_SENSE)" if self.hw == "bang_di_day"
               else "Nhấn rồi nhả nút Reset phần cứng (Pi tạo cạnh RELAY_EN sau đó)")
        lt.reset(on_wait=lambda: self.action(msg, (lambda: sim.click_reset_at(self.now() + 0.1)) if sim else None))

    def calibration(self):
        from faibotic.backends.feetech import default_calibration
        f = self.result_dir() / "calib.json"
        if f.exists():
            cal = json.loads(f.read_text(encoding="utf-8"))
            self.say(f"dùng hiệu chuẩn {f.relative_to(ROOT) if f.is_relative_to(ROOT) else f}")
            return cal
        return default_calibration(paired_joints=EVT_PAIRED_JOINTS)

    def robot(self, enable: bool = True):
        """Robot SDK với backend feetech + chốt E-stop phần mềm (GPIO thật hoặc giả). Reset chốt rồi bật mô-men."""
        from faibotic.robot import Robot
        port = self.port if self.dry else self.args.port
        g = self.gpio()
        kw = dict(port=port, baudrate=self.args.baud, sleep=self.sleep, latch=self.latch(g), calibration=self.calibration(),
                  paired_joints=EVT_PAIRED_JOINTS)
        # S2 (QC2 trạm M1): bộ #0 EVT đo servo/nguồn trên bàn thử, chưa có trạm hai tay. Không khai DOCK_L/DOCK_R thì bỏ kiểm khoá trạm
        # bằng cờ tường minh (SDK cảnh báo); khai đủ hai chân thì SDK kiểm khoá như robot thật.
        pins = getattr(self, "pins", None) or {}
        kw["allow_no_dock"] = not ("DOCK_L" in pins and "DOCK_R" in pins)
        r = Robot("feetech", **kw)
        if enable:
            self.confirm("E-stop nhả, một người giữ E-stop, không ai trong vùng quét tay; reset chốt và bật mô-men?")
            from faibotic.safety import ResetRefused
            try:
                self.reset_latch(r.backend.latch, g)
            except ResetRefused as e:
                r.close()
                raise Abort(f"chốt E-stop không reset được, không bật mô-men: {e}") from e
            r.backend.enable()
        return r

    # ---------- kết quả ----------
    def result_dir(self) -> Path:
        d = Path(self.args.out) / self.serial
        d.mkdir(parents=True, exist_ok=True)
        return d

    def checkpoint(self, extra: dict) -> None:
        """Ghi tạm giữa chừng (burn-in dài): Ctrl-C hoặc mất điện không mất số đã có."""
        f = self.result_dir() / f"{self.script}_{self.t_start.strftime('%Y%m%d-%H%M%S')}{'_dryrun' if self.dry else ''}_dang-chay.json"
        f.write_text(json.dumps(dict(serial=self.serial, nguoi_do=self.args.nguoi_do, du_lieu=extra), ensure_ascii=False, default=str), encoding="utf-8")
        self.partial_file = f

    def finish(self) -> int:
        stamp = self.t_start.strftime("%Y%m%d-%H%M%S")
        d = self.result_dir()
        base = d / f"{self.script}_{stamp}{'_dryrun' if self.dry else ''}"
        k = 2
        while base.with_suffix(".json").exists():
            base = d / f"{self.script}_{stamp}-{k}{'_dryrun' if self.dry else ''}"
            k += 1
        meta = dict(kich_ban=self.script, ma=self.codes, serial=self.serial, nguoi_do=self.args.nguoi_do,
                    ngay=self.t_start.isoformat(timespec="seconds"),
                    che_do="dry-run: MÔ PHỎNG, không phải số đo" if self.dry else f"bus thật {self.args.port}")
        out = dict(meta, tieu_chi=self.checks, du_lieu=self.data)
        base.with_suffix(".json").write_text(json.dumps(out, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
        with base.with_suffix(".csv").open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=["serial", "nguoi_do", "ngay", "ma", "hang_muc", "gia_tri", "don_vi", "nguong", "trang_thai", "ghi_chu"])
            w.writeheader()
            for c in self.checks:
                w.writerow(dict(serial=self.serial, nguoi_do=self.args.nguoi_do, ngay=meta["ngay"], **c))
        bad = [c for c in self.checks if c["trang_thai"] == "khong_dat"]
        pend = [c for c in self.checks if c["trang_thai"] == "chua_kiem"]
        print(f"== {len(self.checks)} mục: {len(bad)} không đạt, {len(pend)} chưa kiểm. Ghi {base.with_suffix('.json')}")
        self.out_file = base.with_suffix(".json")
        return 1 if bad else 0


def run(main_fn, argv=None, desc: str = "", add_args=None) -> int:
    """Bọc main: tạo parser, bắt Abort (dừng có ghi lý do, mã 3)."""
    ap = parser(desc)
    if add_args:
        add_args(ap)
    args = ap.parse_args(argv)
    old = {}

    def _stop(signum, frame):
        raise SystemExit(128 + signum)    # để finally hạ K5; SIGINT/SIGTERM
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            old[sig] = signal.signal(sig, _stop)
        except ValueError:                # không phải luồng chính
            pass
    try:
        return main_fn(args)
    except Abort as e:
        print(f"DỪNG: {e}")
        return 3
    finally:
        release_all()
        for sig, h in old.items():
            signal.signal(sig, h)
