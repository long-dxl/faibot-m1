"""Backend phần cứng thật: 16 servo Feetech STS trên một bus (P-L, xem spec.SERVO_IDS).

CHƯA chạy với phần cứng thật; đã test với bus giả (faibotic.feetech.mockbus).
Không bật mô-men hay ghi EEPROM khi chưa gọi enable() để tránh robot tự chạy lúc kết nối.

P-L (S1C): mỗi khớp một servo, mặc định không ghép đôi (spec.PAIRED_JOINTS rỗng). Đường mã ghép đôi giữ, bật bằng paired_joints=spec.PA_PAIRED_JOINTS.
v0.3: vai và khuỷu ghép đôi (servo A + servo B "<khớp>_b", lắp đối xứng nên mặc định quay ngược chiều).
Một lệnh khớp ghi cả hai servo, mỗi servo có zero/sign riêng; check_pairs() so vị trí và tải hai servo mỗi chu kỳ,
vượt ngưỡng, một servo im lặng hoặc báo lỗi (quá tải, quá nhiệt) thì tắt mô-men cả cặp và ném PairFault.
QC2: chốt E-stop phần mềm (SafetyLatch) bắt buộc, có luồng giám sát nền; xem docs/kien-truc.md.
v0.5: trục nâng thanh răng không tự hãm + chốt solenoid (safety.LiftLatch, R2 7.2): lift chỉ chạy khi chốt đã rút theo trình tự
giữ -> rút -> cảm biến; dừng thì giữ -> cài -> cảm biến -> tắt mô-men lift; E-stop cắt cuộn cùng servo; kẹt/quá mô-men -> LiftFault.
TORQUE_LIMIT cho mọi servo theo nhóm + kiểm tổng dòng khi enable() (limits.torque_limits, limits.current_budget; R2 7.4).
S2 (trạm hai tay): tự nhận cấu hình theo BUS_B (spec.detect_config; chỉ di_dong chạy được ở backend này, tram_ban chờ core S3); khoá trạm
DOCK_L/DOCK_R (safety.DockSwitches): enable(), lệnh lift và đế bị từ chối khi thiếu một khoá; khoá rớt khi chạy -> tắt mô-men, DockOpen, chờ clear_fault().
Bus: spec.SERVO_BUS (lift trên BUS_B); backend này vẫn mở MỘT cổng (bus gộp/bench); ba cổng riêng là việc core S3.
"""
from __future__ import annotations

import math
import numbers
import statistics
import threading
import time
import warnings

import numpy as np

from .. import kinematics, limits, spec
from ..feetech import protocol as P
from ..feetech.bus import FeetechBus, SyncReadTimeout
from ..loi import (BusCauHinhSai, BusMat, CauHinhSai, DienApThap, HieuChuanSai, LenhKhongHopLe, M1Loi, RobotDaDong, ServoBaoLoi,
                   ServoMat, WatchdogHet, log)
from ..safety import LiftLatchError, SafetyLatch, dock_state
from .base import Backend

ARM_NAMES = [spec.joint_name(s, j) for s in spec.SIDES for j in spec.ARM_JOINTS]
GRIP_NAMES = [spec.joint_name(s, "gripper") for s in spec.SIDES]
# Thanh ghi ACCELERATION (41) STS3215: đơn vị 100 bước/s² [CẦN-XÁC-MINH theo bảng nhớ Feetech; 0 = không giới hạn]. lớp chặn phụ
# dưới dốc phần mềm limits.BaseRamp (lớp chính): đặt vừa đủ để bánh theo được dốc phần mềm ở ca xấu nhất, nên không làm chậm lệnh hợp lệ
# nhưng chặn lệnh nhảy bậc (bỏ qua SDK, lỗi phần mềm).
STS_ACC_UNIT_STEPS = 100


def wheel_acc_raw() -> int:
    """Giá trị ACCELERATION cho servo bánh: gia tốc góc bánh lớn nhất mà dốc cần (tịnh tiến + |v|·|ω| theo mọi hướng, cộng
    R·α), đổi ra bước/s², làm tròn lên, kẹp 1..254 (không bao giờ 0 vì 0 = không giới hạn)."""
    a = ((spec.BASE_MAX_ACC + spec.BASE_TURN_ACC_MAX) + spec.WHEEL_MOUNT_RADIUS * spec.BASE_MAX_ALPHA) / spec.WHEEL_RADIUS
    raw = math.ceil(a * P.STEPS_PER_REV / (2 * math.pi) / STS_ACC_UNIT_STEPS)
    return max(1, min(254, raw))


POS_NAMES = list(spec.POS_NAMES)   # ARM_NAMES + GRIP_NAMES + lift
WHEELS = ["wheel_0", "wheel_1", "wheel_2"]


def pairs_for(paired_joints=None) -> dict[str, str]:
    """Khớp A -> servo B của các khớp ghép đôi (P-L: rỗng)."""
    pj = spec.PAIRED_JOINTS if paired_joints is None else paired_joints
    return {spec.joint_name(s, j): spec.pair_name(spec.joint_name(s, j)) for s in spec.SIDES for j in pj}


PAIRS = pairs_for()            # mặc định theo spec (P-L: rỗng)
PAIR_B = list(PAIRS.values())


class PairFault(M1Loi):
    """Cặp servo ghép đôi lệch vị trí/tải, báo lỗi (quá tải, quá nhiệt) hoặc mất phản hồi: đã tắt mô-men cả cặp."""
    ma = "SER-02"


class LiftFault(M1Loi):
    """Trục nâng: chốt hãm không theo trình tự, kẹt hoặc quá mô-men vào chốt; lift đã giữ tại chỗ, lệnh bị chặn tới clear_fault()."""
    ma = "LIFT-01"


class LiftUnderpowered(M1Loi):
    """Giới hạn mô-men lift không đủ giữ tải thiết kế (spec.LIFT_LIMIT_COVERS_DESIGN_LOAD False): lệnh lift bị từ chối."""
    ma = "LIFT-02"


class CurrentBudgetExceeded(M1Loi):
    """Tổng dòng ở giới hạn mô-men vượt spec.CURRENT_BUDGET_A: không bật mô-men."""
    ma = "PWR-03"


class DockOpen(M1Loi):
    """Khoá trạm hai tay chưa đóng / rớt khi đang chạy (S2, DOCK_L/DOCK_R)."""
    ma = "SAF-02"


class EStopActive(M1Loi):
    """Chốt E-stop phần mềm không ở trạng thái chạy (hoặc luồng giám sát đã chết): không nhận lệnh."""
    ma = "SAF-01"


class OutOfSoftLimits(M1Loi):
    """Tay đang nằm ngoài giới hạn mềm (vd ở chặn cơ khí sau mất điện): không bật mô-men để tránh giật."""
    ma = "SAF-03"


def default_gpio():
    """GPIO thật trên Pi 5 theo biến môi trường FAIBOTIC_GPIO_PINS (faibotic.gpio_pi). Chưa đặt -> None -> backend từ chối
    enable(). Đặt mà thiếu thư viện/chip/chân bắt buộc -> ném lỗi (không âm thầm chạy không chốt)."""
    from ..gpio_pi import from_env
    return from_env()


class _LockedBus:
    """Bọc FeetechBus: mỗi giao dịch giữ một khoá, để luồng E-stop và chương trình chính không chen gói trên cùng cổng.

    Tầng 1b: backend đã đóng -> RobotDaDong (không chạm cổng); bus đã mất -> BusMat ngay; giao dịch gặp BusMat -> báo backend
    (_on_bus_lost, gọi NGOÀI khoá bus để không khoá chết với luồng chốt E-stop) rồi ném lại."""

    def __init__(self, bus, owner=None):
        self._bus, self._lock, self._owner = bus, threading.RLock(), owner

    def __getattr__(self, name):
        attr = getattr(self._bus, name)
        if not callable(attr):
            return attr

        def call(*a, **k):
            be = self._owner
            lost = None
            with self._lock:
                if be is not None:
                    if be._closed:
                        raise RobotDaDong(f"bus.{name}: robot đã đóng (close()), không gửi gói nào nữa")
                    if be.bus_lost:
                        raise BusMat(f"bus.{name}: {be.bus_lost}")
                try:
                    return attr(*a, **k)
                except BusMat as e:
                    if be is None:
                        raise
                    lost = e
            be._on_bus_lost(lost)
            raise lost
        return call

    def try_call(self, name: str, *a, timeout: float = 0.1, **k) -> bool:
        """Đường dừng khẩn (QC2 M2): lấy khoá bus có hạn. Hết hạn (luồng khác giữ khoá, vd close() đang chờ chốt) thì bỏ qua, trả False:
        chốt E-stop đã hạ chân cho phép trước đó nên phần cứng vẫn cắt nguồn servo; không bao giờ khoá chết với luồng chốt."""
        if not self._lock.acquire(timeout=timeout):
            return False
        try:
            getattr(self, name)(*a, **k)
            return True
        finally:
            self._lock.release()


def default_calibration(mirror: bool = True, paired_joints=None) -> dict:
    """Hiệu chuẩn mặc định: raw 2048 = 0 rad; kẹp raw 2048 đóng, 2900 mở; trục nâng 0 m tại raw 0.

    Lift (v0.5): bánh răng Ø42,8 trên thanh răng, truyền động trực tiếp, spec.LIFT_STEPS_PER_M bước/m, servo ở chế độ nhiều vòng
; raw 0 tại đáy (homing về LIFT_BOT). Giá trị thật đo ở bước lắp ráp.
    """
    cal = {n: {"zero": 2048, "sign": 1} for n in ARM_NAMES}
    # servo B lắp đối xứng qua link: cùng góc khớp thì quay ngược chiều (mirror=True). zero đo riêng khi calibrate.
    for b in pairs_for(paired_joints).values():
        cal[b] = {"zero": 2048, "sign": -1 if mirror else 1}
    for g in GRIP_NAMES:
        cal[g] = {"closed": 2048, "open": 2900}
    # v0.5 thanh răng: ~30.463 bước/m, raw 0 ở đáy. v0.4 vitme 16:1: 128.000 bước/m, raw 0 giữa hành trình. Cũ hơn: GT2 40 mm/vòng.
    cal["lift"] = {"zero": 0, "counts_per_m": float(spec.LIFT_STEPS_PER_M), "offset_m": spec.LIFT_RAW_ZERO_M}
    for w in WHEELS:
        cal[w] = {"sign": 1}
    return cal


def _so(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _cal_fields(name: str) -> set[str]:
    if name == "lift":
        return {"zero", "counts_per_m", "offset_m"}
    if name.endswith("gripper"):
        return {"closed", "open"}
    if name in WHEELS:
        return {"sign"}
    return {"zero", "sign"}


def validate_calibration(cal, names) -> dict:
    """R-49/R-32: kiểm hiệu chuẩn ngay lúc khởi tạo (không đợi lần đọc đầu ném ZeroDivisionError/KeyError). Sai -> HieuChuanSai
    (ValueError, CFG-01) ghi khớp và trường. Đủ đúng các khớp `names`, mỗi khớp đúng bộ trường, mọi giá trị là số hữu hạn;
    sign thuộc {1, -1}; zero tay trong 0..4095; kẹp open != closed, cả hai trong 0..4095; lift counts_per_m != 0."""
    if not isinstance(cal, dict):
        raise HieuChuanSai(f"hiệu chuẩn phải là dict {{tên servo: {{trường: số}}}}, nhận {type(cal).__name__}")
    names = list(names)
    thieu = [n for n in names if n not in cal]
    if thieu:
        raise HieuChuanSai(f"hiệu chuẩn thiếu khớp: {', '.join(thieu)}", khop=thieu[0])
    la = [str(n) for n in cal if n not in names]
    if la:
        raise HieuChuanSai(f"hiệu chuẩn có khớp lạ: {', '.join(la)} (khớp hợp lệ: {', '.join(names)})", khop=la[0])
    for n in names:
        f, want = cal[n], _cal_fields(n)
        if not isinstance(f, dict) or set(f) != want:
            co = sorted(map(str, f)) if isinstance(f, dict) else type(f).__name__
            raise HieuChuanSai(f"hiệu chuẩn {n}: cần đúng các trường {sorted(want)}, có {co}", khop=n)
        for k, v in f.items():
            if not _so(v):
                raise HieuChuanSai(f"hiệu chuẩn {n}.{k} = {v!r}: cần số thực hữu hạn", khop=n)
        if n == "lift":
            if f["counts_per_m"] == 0:
                raise HieuChuanSai("hiệu chuẩn lift.counts_per_m = 0: không quy đổi được bước sang mét", khop=n)
        elif n.endswith("gripper"):
            for k in ("open", "closed"):
                if not 0 <= f[k] <= 4095:
                    raise HieuChuanSai(f"hiệu chuẩn {n}.{k} = {f[k]} nằm ngoài 0..4095", khop=n)
            if f["open"] == f["closed"]:
                raise HieuChuanSai(f"hiệu chuẩn {n}: open = closed = {f['open']} (kẹp không có hành trình)", khop=n)
        else:
            if f["sign"] not in (1, -1):
                raise HieuChuanSai(f"hiệu chuẩn {n}.sign = {f['sign']}: phải là 1 hoặc -1", khop=n)
            if "zero" in f and not 0 <= f["zero"] <= 4095:
                raise HieuChuanSai(f"hiệu chuẩn {n}.zero = {f['zero']} nằm ngoài 0..4095", khop=n)
    return cal


def validate_ids(ids, names) -> dict:
    """R-50/R-32: bảng ID servo: đủ đúng các khớp `names`, ID nguyên 1..253, không trùng. Sai -> CauHinhSai (ValueError, CFG-02)."""
    if not isinstance(ids, dict):
        raise CauHinhSai(f"ids phải là dict {{tên servo: ID}}, nhận {type(ids).__name__}")
    names = list(names)
    thieu = [n for n in names if n not in ids]
    if thieu:
        raise CauHinhSai(f"bảng ID thiếu khớp: {', '.join(thieu)}", khop=thieu[0])
    la = [str(n) for n in ids if n not in names]
    if la:
        raise CauHinhSai(f"bảng ID có khớp lạ: {', '.join(la)} (khớp hợp lệ: {', '.join(names)})", khop=la[0])
    seen: dict[int, str] = {}
    for n in names:
        i = ids[n]
        if isinstance(i, bool) or not isinstance(i, numbers.Integral) or not 1 <= int(i) <= 253:
            raise CauHinhSai(f"bảng ID {n} = {i!r}: ID servo phải là số nguyên 1..253", khop=n)
        if int(i) in seen:
            raise CauHinhSai(f"bảng ID trùng: {seen[int(i)]} và {n} cùng ID {int(i)}", khop=n)
        seen[int(i)] = n
    return {n: int(ids[n]) for n in names}


class FeetechBackend(Backend):
    name = "feetech"

    def __init__(self, port="/dev/ttyUSB0", baudrate: int = 1_000_000, calibration: dict | None = None,
                 ids: dict | None = None, enable: bool = False, sleep=time.sleep, latch=None, gpio=None,
                 allow_no_latch: bool = False, latch_period: float = 0.01, lift_switches=None, soft_limit_tol_deg: float = 1.0,
                 monitor_pairs: bool = True, pair_max_pos_deg: float = spec.PAIR_MAX_POS_DIFF_DEG,
                 pair_max_load_diff: float = spec.PAIR_MAX_LOAD_DIFF, pair_load_floor: float = spec.PAIR_LOAD_FLOOR,
                 pair_fault_cycles: int = spec.PAIR_FAULT_CYCLES, lift_latch=None, allow_lift_underpowered: bool = False,
                 paired_joints=None, dock=None, config: str = "auto", allow_no_dock: bool = False):
        # Tầng 1b: trạng thái vòng đời và lỗi khi chạy
        self._closed = False
        self.bus_lost: str | None = None          # BUS-01: cổng mất giữa phiên (vĩnh viễn với backend này)
        self.run_fault: tuple | None = None       # (lớp lỗi, mã, câu): ServoMat/ServoBaoLoi/DienApThap/bus im lặng; chờ clear_fault()
        self.watchdog_fault: str | None = None    # SYS-03
        # paired_joints None = spec.PAIRED_JOINTS (P-L: không ghép). P-A: spec.PA_PAIRED_JOINTS, ids mặc định thêm spec.PA_PAIR_IDS.
        self.pairs = pairs_for(paired_joints)
        self.pair_b = list(self.pairs.values())
        need = POS_NAMES + WHEELS + self.pair_b
        if not isinstance(config, str) or config not in ("auto", "di_dong", "tram_ban"):
            raise CauHinhSai(f"config phải là auto/di_dong/tram_ban, nhận {config!r}")
        self.ids = validate_ids({**spec.SERVO_IDS, **{b: spec.PA_PAIR_IDS[b] for b in self.pair_b}} if ids is None else ids, need)
        self.cal = validate_calibration(default_calibration(paired_joints=paired_joints) if calibration is None else calibration, need)
        self._name_of = {i: n for n, i in self.ids.items()}
        self._sleep = sleep
        self._pose = np.zeros(3)
        self._wheel_cmd = np.zeros(3)
        self.base_ramp = limits.BaseRamp()      # dốc vận tốc đế, lớp chính
        self._reset_monitors()
        self.bus = _LockedBus(FeetechBus(port, baudrate=baudrate), self)
        try:
            self._connect(config)
        except BaseException:
            self._close_port()
            raise
        self.dock = dock                         # safety.DockSwitches; None: lấy từ GPIO Pi nếu có DOCK_L/DOCK_R
        self.dock_fault: str | None = None
        # QC2 S2 M1: cấu hình có trạm (spec.ROBOT_CONFIGS[...]["dock_required"]) mà không có khoá DOCK thì enable/lift/đế bị từ chối
        # (fail-closed), trừ khi người gọi chủ động allow_no_dock=True (cảnh báo) hoặc bench allow_no_latch=True (đã cảnh báo).
        self.allow_no_dock = allow_no_dock or allow_no_latch
        self.enabled = False
        self.lift_switches = lift_switches       # limits.LiftSwitches; None: lấy từ GPIO Pi nếu có LIFT_BOT/LIFT_TOP
        self.lift_latch = lift_latch             # safety.LiftLatch; None: lấy từ GPIO Pi nếu có LIFT_LATCH_EN/RETRACTED
        self.lift_fault: str | None = None
        self._lift_torque = False
        self._lift_idle_s = 0.0
        self._lift_jam = 0
        self.latch_period = latch_period
        self.soft_limit_tol = math.radians(soft_limit_tol_deg)
        self.allow_no_latch = allow_no_latch
        # spec.LIFT_LIMIT_COVERS_DESIGN_LOAD False (giới hạn răng < lực giữ tải thiết kế): lift KHÔNG được chạy trừ khi người
        # dùng chủ động cho phép (thử không tải / tải nhẹ ở EVT, có người giám sát).
        self.allow_lift_underpowered = allow_lift_underpowered
        self._latch = None
        g = gpio if gpio is not None else (default_gpio() if latch is None else None)
        self._owned_gpio = g if gpio is None else None   # GPIO do backend tự mở: close() hạ K5 và nhả chân
        try:
            self._init_rest(latch, g, sleep, enable, allow_no_latch, monitor_pairs, pair_max_pos_deg, pair_max_load_diff,
                            pair_load_floor, pair_fault_cycles)
        except BaseException:
            if self._latch is not None:    # lỗi khi khởi tạo: dừng luồng giám sát, không để K5/chân GPIO bị giữ
                self._latch.stop_monitor()
            self._release_gpio()
            self._close_port()
            raise

    def _connect(self, config: str) -> None:
        """Ping mọi servo; trùng ID -> BusCauHinhSai (BUS-07); thiếu -> BusCauHinhSai (BUS-06) ghi tên khớp và ID; cấu hình theo BUS_B;
        đọc MODEL_NUMBER, số lạ -> cảnh báo (R-51)."""
        status = {n: self.bus.ping_detail(i) for n, i in self.ids.items()}
        garbled = [n for n, st in status.items() if st == "garbled"]
        if garbled:
            raise BusCauHinhSai("trùng ID trên bus: " + ", ".join(f"ID {self.ids[n]} ({n})" for n in garbled)
                                + " trả gói chồng nhau ở mọi lần thử (hai servo cùng ID, hoặc nhiễu nặng trên dây). Gỡ bớt servo, "
                                "đặt lại ID rồi kết nối lại", ma="BUS-07", khop=garbled[0])
        missing = [n for n, st in status.items() if st != "ok"]
        txt = ", ".join(f"{n} (ID {self.ids[n]})" for n in missing)
        # cấu hình theo BUS_B (3 bánh + lift). Chế độ bàn (không BUS_B) chưa có trong backend này: core S3.
        seen = {i for n, i in self.ids.items() if n not in missing}
        try:
            found = spec.detect_config(seen)
        except ValueError as e:
            raise BusCauHinhSai(f"không thấy servo: {missing} [{txt}] ({e})", khop=missing[0] if missing else None) from None
        if config != "auto" and config != found:
            raise BusCauHinhSai(f"chọn cấu hình {config} nhưng bus cho {found} (servo thiếu: {missing})")
        self.config = found
        if found == "tram_ban":
            raise BusCauHinhSai("cấu hình tram_ban (không có BUS_B): backend feetech chưa hỗ trợ, dùng core Studio (S3)")
        if missing:
            raise BusCauHinhSai(f"không thấy servo: {txt}", khop=missing[0])
        self._check_models()

    def _check_models(self) -> None:
        """R-51: MODEL_NUMBER lạ (không thuộc spec.SERVO_MODEL_NUMBERS) hoặc khác loại servo của khớp -> RuntimeWarning (không chặn)."""
        known = set(spec.SERVO_MODEL_NUMBERS.values())
        for n, sid in self.ids.items():
            try:
                m = self.bus.read(sid, P.MODEL_NUMBER)
            except (TimeoutError, P.ProtocolError):
                continue
            try:
                want = spec.SERVO_MODEL_NUMBERS.get(limits.servo_model(n).model.split("-")[0])
            except (KeyError, ValueError, AttributeError):
                want = None
            if m not in known or (want is not None and m != want):
                exp = f"STS3215 = {want}" if want is not None else "STS3095/STS3215"
                warnings.warn(f"servo {n} (ID {sid}): model {m} ({m:#06x}) khác loại cần ({exp}); kiểm lại servo lắp ở khớp này",
                              RuntimeWarning, stacklevel=4)

    def _reset_monitors(self) -> None:
        self._miss: dict[int, int] = {}
        self._miss_kind: dict[int, str] = {}
        self._silent = 0
        self._v_low = 0
        self._v_warn_n = 0
        self._v_warned = False
        self._last_step = time.monotonic()

    def _close_port(self) -> None:
        bus = getattr(self, "bus", None)
        if bus is None:
            return
        try:
            bus._bus.close()
        except Exception:                      # noqa: BLE001 - cổng đã mất: đóng không được cũng không sao
            pass

    def _init_rest(self, latch, g, sleep, enable, allow_no_latch, monitor_pairs, pair_max_pos_deg, pair_max_load_diff,
                   pair_load_floor, pair_fault_cycles) -> None:
        if latch is None and g is not None:
            from ..gpio_pi import PiGPIO, latch_kwargs
            latch = SafetyLatch(g, sleep=sleep, **(latch_kwargs(g) if isinstance(g, PiGPIO) else {}))
        if self.lift_switches is None and g is not None:
            from ..gpio_pi import lift_switches_for
            self.lift_switches = lift_switches_for(g)
        if self.lift_latch is None and g is not None:
            from ..gpio_pi import lift_latch_for
            self.lift_latch = lift_latch_for(g, sleep=sleep)
        if self.dock is None and g is not None:
            from ..gpio_pi import dock_for
            self.dock = dock_for(g)
        if self.dock is None and self.dock_required() and self.allow_no_dock and not allow_no_latch:
            warnings.warn("FeetechBackend chạy KHÔNG kiểm khoá trạm DOCK_L/DOCK_R (allow_no_dock=True): chỉ dựa vào hai công tắc khoá "
                          "nối tiếp vòng E-stop phần cứng", RuntimeWarning, stacklevel=3)
        if latch is not None:
            self.latch = latch             # tự nối on_stop -> emergency_stop và chạy luồng giám sát nền
        elif allow_no_latch:
            warnings.warn("FeetechBackend chạy KHÔNG có chốt E-stop phần mềm (allow_no_latch=True): chỉ dùng cho bench test, "
                          "E-stop phải cắt K1 bằng phần cứng; khoá trạm DOCK cũng không bắt buộc", RuntimeWarning, stacklevel=3)
        self.monitor_pairs = monitor_pairs
        self.pair_max_pos = math.radians(pair_max_pos_deg)
        self.pair_max_load_diff = pair_max_load_diff
        self.pair_load_floor = pair_load_floor
        self.pair_fault_cycles = pair_fault_cycles
        self._pair_bad = {j: 0 for j in self.pairs}
        self.fault: str | None = None
        self.pair_metrics: dict = {}
        self._cmd = self.read_joints()     # đích đã ra lệnh gần nhất (gốc để kiểm đoạn nội suy)
        if enable:
            self.enable()

    # ---------- quy đổi ----------
    def to_raw(self, name: str, value: float) -> int:
        c = self.cal[name]
        if name == "lift":
            return int(round(c["zero"] + (value - c.get("offset_m", 0.0)) * c["counts_per_m"]))
        if name.endswith("gripper"):
            return int(round(c["closed"] + (c["open"] - c["closed"]) * value / 100.0))
        return int(round(c["zero"] + c["sign"] * value * P.STEPS_PER_REV / (2 * math.pi)))

    def from_raw(self, name: str, raw: int) -> float:
        c = self.cal[name]
        if name == "lift":
            return (raw - c["zero"]) / c["counts_per_m"] + c.get("offset_m", 0.0)
        if name.endswith("gripper"):
            return 100.0 * (raw - c["closed"]) / (c["open"] - c["closed"])
        return c["sign"] * (raw - c["zero"]) * 2 * math.pi / P.STEPS_PER_REV

    # ---------- chốt E-stop ----------
    @property
    def latch(self):
        return self._latch

    @latch.setter
    def latch(self, lt) -> None:
        """Gắn safety.SafetyLatch: on_stop luôn gồm emergency_stop (giữ callback cũ chạy sau), luồng nền chạy ngay."""
        if self._latch is not None and self._latch is not lt:
            self._latch.stop_monitor()
        self._latch = lt
        if lt is None:
            return
        prev = lt.on_stop
        if prev is None or prev == self.emergency_stop:
            lt.on_stop = self.emergency_stop
        elif not getattr(prev, "_faibotic_estop", False):
            def on_stop():
                try:
                    self.emergency_stop()
                finally:
                    prev()
            on_stop._faibotic_estop = True
            lt.on_stop = on_stop
        lt.on_tick = self._watchdog_tick         # R-25: watchdog chạy trong luồng giám sát chốt
        lt.start_monitor(self.latch_period)

    # ---------- Tầng 1b: canh cổng, watchdog, dừng an toàn có mã ----------
    def _guard(self, action: str, motion: bool = True) -> None:
        """Đã đóng -> RobotDaDong; bus mất -> BusMat; (lệnh chuyển động) watchdog hết / lỗi khi chạy chưa xoá -> lỗi cùng lớp."""
        if self._closed:
            raise RobotDaDong(f"{action}: robot đã đóng (close()); tạo Robot mới để chạy tiếp")
        if self.bus_lost:
            raise BusMat(f"{action}: {self.bus_lost}")
        if motion:
            if self.watchdog_fault:
                raise WatchdogHet(f"{action}: {self.watchdog_fault} (kiểm tra máy chủ, clear_fault() rồi enable())")
            if self.run_fault:
                cls, ma, msg = self.run_fault
                raise cls(f"{action}: đang dừng do lỗi: {msg} (kiểm tra rồi clear_fault(), enable())", ma=ma)

    def _on_bus_lost(self, e: BaseException) -> None:
        """BUS-01: cổng mất giữa phiên. SDK không còn gửi được gì: đánh dấu, dừng dốc đế, cắt cuộn chốt lift và cắt chốt E-stop
        (hạ RELAY_EN/K5 để phần cứng cắt nguồn servo). Mọi lệnh sau ném BusMat; close() không ném."""
        if self.bus_lost:
            return
        self.bus_lost = f"mất bus servo ({e}); đã cắt chốt E-stop, kiểm cáp USB rồi tạo Robot mới"
        self.enabled = False
        self._lift_torque = False
        self.base_ramp.reset()
        self._wheel_cmd = np.zeros(3)
        try:
            if self.lift_latch is not None:
                self.lift_latch.cut()
        except Exception:                      # noqa: BLE001
            pass
        lt = self._latch
        if lt is not None:
            try:
                lt.trip(f"[BUS-01] {self.bus_lost}")
            except Exception:                  # noqa: BLE001 - on_stop không gửi được gói: phần cứng vẫn cắt
                pass

    def _halt(self, cls, ma: str, msg: str, khop: str | None = None, cut_latch: bool = False):
        """Về TTAT rồi ném lỗi có mã: ghi GOAL_SPEED bánh 0 (bus còn sống), tắt mô-men mọi servo + cắt cuộn chốt lift, dốc về 0.
        cut_latch: lỗi mất liên lạc (không chắc servo nhận lệnh tắt) -> cắt luôn chốt E-stop để phần cứng cắt nguồn servo."""
        self.run_fault = (cls, ma, msg)
        lt = self._latch
        if cut_latch and lt is not None:       # QC2 m3: hạ chân chốt TRƯỚC mọi lần ghi bus
            try:
                lt.trip(f"[{ma}] {msg}")
            except Exception:                  # noqa: BLE001
                pass
        try:
            self.bus.sync_write(P.GOAL_SPEED, {self.ids[n]: 0 for n in WHEELS})
        except Exception:                      # noqa: BLE001 - bus hỏng: vẫn đi tiếp tắt mô-men
            pass
        self._wheel_cmd = np.zeros(3)
        try:
            self.emergency_stop()
        except Exception:                      # noqa: BLE001
            pass
        raise cls(msg, ma=ma, khop=khop)

    def _watchdog_tick(self) -> None:
        """R-25, gọi từ luồng giám sát chốt mỗi latch_period: đế đang có lệnh bánh khác 0 mà step() im quá spec.WATCHDOG_S
        -> ghi GOAL_SPEED bánh 0, tắt mô-men mọi servo; lệnh kế ném WatchdogHet."""
        if not self.enabled or self.watchdog_fault or self._closed or self.bus_lost or not np.any(self._wheel_cmd):
            return
        idle = time.monotonic() - self._last_step
        if idle <= spec.WATCHDOG_S:
            return
        self.watchdog_fault = f"watchdog: không có step() trong {idle:.2f} s > {spec.WATCHDOG_S} s khi đế đang chạy: đã dừng bánh, tắt mô-men"
        log.error("[SYS-03] %s", self.watchdog_fault)
        try:
            self.bus.sync_write(P.GOAL_SPEED, {self.ids[n]: 0 for n in WHEELS})
        except Exception:                      # noqa: BLE001
            pass
        self._wheel_cmd = np.zeros(3)
        try:
            self.emergency_stop()
        except Exception:                      # noqa: BLE001
            pass

    def _poll(self, dt: float) -> np.ndarray | None:
        """Mỗi step(): MỘT sync_read 8 byte (vị trí, tốc độ, tải, điện áp, nhiệt) mọi servo. Khi enabled: bit lỗi -> ServoBaoLoi /
        DienApThap ngay; servo im hoặc gói hỏng spec.SERVO_MISS_CYCLES chu kỳ liền -> ServoMat (BUS-03/BUS-02); cả bus im -> BusMat
        (BUS-04); điện áp (trung vị các servo) dưới V_CANH cảnh báo một lần, dưới V_CAT V_LOC_CHU_KY chu kỳ liền -> DienApThap. Trả tốc độ bánh
        (rad/s) đọc được, hoặc None nếu thiếu."""
        ids = list(self.ids.values())
        res = self.bus.sync_read_all(P.PRESENT_POSITION[0], 8, ids)
        self.last_poll = res
        if self.enabled:
            self._check_poll(res, ids)
        if all(self.ids[n] in res.data for n in WHEELS):
            return np.array([self.cal[n]["sign"] * P.decode_signed(P.from_u16(res.data[self.ids[n]][2:4])) * 2 * math.pi
                             / P.STEPS_PER_REV for n in WHEELS])
        return None

    def _check_poll(self, res, ids) -> None:
        n_max = spec.SERVO_MISS_CYCLES
        skip = set()
        if self.monitor_pairs:                 # servo của cặp ghép đôi: check_pairs() lo (PairFault)
            for j, b in self.pairs.items():
                skip |= {self.ids[j], self.ids[b]}
        name = self._name_of
        errs = {sid: e for sid, e in res.err.items() if sid not in skip}
        if errs:
            volt = [sid for sid, e in errs.items() if e & 1]
            if volt:
                self._halt(DienApThap, "PWR-02", "servo bật bit lỗi điện áp: " + ", ".join(f"{name[i]} (ID {i})" for i in volt)
                           + "; kiểm pin và dây nguồn servo", khop=name[volt[0]])
            txt = "; ".join(f"{name[i]} (ID {i}) báo lỗi: {P.error_text(e)}" for i, e in errs.items())
            self._halt(ServoBaoLoi, "SER-01", f"dừng an toàn: {txt}", khop=name[next(iter(errs))])
        bad = set(res.missing) | set(res.corrupt)
        self._silent = self._silent + 1 if (bad and not res.data) else 0
        if self._silent >= n_max:
            self._halt(BusMat, "BUS-04", f"bus im lặng: không servo nào trả lời {self._silent} chu kỳ liền (cáp servo đứt, mất nguồn "
                       "servo hoặc đứt đường về); đã cắt chốt E-stop", cut_latch=True)
        over = []
        for sid in ids:
            if sid in skip:
                continue
            if sid in bad:
                self._miss[sid] = self._miss.get(sid, 0) + 1
                self._miss_kind[sid] = "hong" if sid in res.corrupt else "mat"
                if self._miss[sid] >= n_max:
                    over.append(sid)
            else:
                self._miss[sid] = 0
        if over:
            hong = any(self._miss_kind[i] == "hong" for i in over)
            txt = ", ".join(f"{name[i]} (ID {i})" for i in over)
            what = "trả gói hỏng (sai checksum)" if hong else "không trả lời"
            self._halt(ServoMat, "BUS-02" if hong else "BUS-03", f"servo {txt} {what} {n_max} chu kỳ liền: dừng an toàn, đã cắt chốt "
                       "E-stop (servo mất liên lạc có thể không nhận lệnh tắt mô-men)", khop=name[over[0]], cut_latch=True)
        volts = [d[6] / 10.0 for sid, d in res.data.items() if len(d) > 6 and sid not in skip]
        if volts:
            self._check_volts(statistics.median(volts))

    def _check_volts(self, v: float) -> None:
        if v < spec.V_CANH:
            self._v_warn_n += 1
            if self._v_warn_n >= spec.V_LOC_CHU_KY and not self._v_warned:
                self._v_warned = True
                log.warning("[PWR-00] điện áp servo thấp: %.1f V < %.1f V (cảnh báo một lần; dưới %.1f V robot sẽ dừng)", v,
                            spec.V_CANH, spec.V_CAT)
        else:
            self._v_warn_n = 0
            if v >= spec.V_CANH + 0.3:
                self._v_warned = False
        self._v_low = self._v_low + 1 if v < spec.V_CAT else 0
        if self._v_low >= spec.V_LOC_CHU_KY:
            self._halt(DienApThap, "PWR-01", f"điện áp servo {v:.1f} V < {spec.V_CAT} V trong {self._v_low} chu kỳ liền: dừng an toàn; "
                       "sạc/thay pin rồi clear_fault(), enable()")

    def _check_voltage_enable(self) -> None:
        """enable() từ chối khi điện áp (trung vị các servo) dưới spec.V_CAT."""
        res = self.bus.sync_read_all(P.PRESENT_VOLTAGE[0], 1, list(self.ids.values()))
        volts = [d[0] / 10.0 for d in res.data.values() if d]
        if volts and statistics.median(volts) < spec.V_CAT:
            v = statistics.median(volts)
            raise DienApThap(f"enable: điện áp servo {v:.1f} V < {spec.V_CAT} V: sạc/thay pin trước khi bật mô-men", ma="PWR-01")

    def _check_latch(self, action: str) -> None:
        if self._closed:
            raise RobotDaDong(f"{action}: robot đã đóng (close())")
        lt = self._latch
        if lt is None:
            if self.allow_no_latch:
                return
            raise EStopActive(f"{action}: không có chốt E-stop (GPIO thật chưa có). Truyền gpio=/latch=, "
                              "hoặc allow_no_latch=True cho bench test")
        if not lt.monitor_alive:
            if self.enabled:
                self.emergency_stop()
            raise EStopActive(f"{action}: luồng giám sát E-stop không chạy ({lt.monitor_error or 'đã dừng'})")
        if not lt.ok:
            if self.enabled:
                self.emergency_stop()
            raise EStopActive(f"{action}: chốt E-stop chưa reset: nhả E-stop rồi gọi latch.reset()")

    # ---------- khoá trạm hai tay (S2, R3b 5.7) ----------
    def dock_state(self) -> dict:
        """{"DOCK_L": bool, "DOCK_R": bool, "ok": bool}; chưa khai chân DOCK: {"ok": None} (phần cứng vẫn chặn qua vòng E-stop)."""
        return dock_state(self.dock)

    def dock_required(self) -> bool:
        return bool(spec.ROBOT_CONFIGS.get(getattr(self, "config", "di_dong"), {}).get("dock_required"))

    def _check_dock(self, action: str) -> None:
        if self.dock_fault:
            raise DockOpen(f"{action}: đang dừng do khoá trạm: {self.dock_fault} (đóng khoá, kiểm rồi gọi clear_fault())")
        if self.dock is None:
            if self.dock_required() and not self.allow_no_dock:
                raise DockOpen(f"{action}: cấu hình {self.config} cần khoá trạm nhưng chưa khai chân DOCK_L/DOCK_R "
                               "(FAIBOTIC_GPIO_PINS hoặc dock=); thử có giám sát: allow_no_dock=True")
            return
        try:
            st = self.dock.state()
        except Exception as e:                 # noqa: BLE001 - đọc lỗi coi như khoá mở
            raise DockOpen(f"{action}: không đọc được khoá trạm ({e}): coi như mở") from e
        if st["ok"] is False:
            open_ = [p for p in spec.DOCK_PINS if not st.get(p)]
            raise DockOpen(f"{action}: khoá trạm chưa đóng: {', '.join(open_)}")

    def _watch_dock(self) -> None:
        """Mỗi step(): khoá rớt khi đang chạy -> phần cứng đã cắt K1; SDK tắt mô-men, ghi lỗi, không tự chạy lại."""
        if self.dock is None or not self.enabled:
            return
        try:
            st = self.dock.state()
        except Exception as e:                 # noqa: BLE001 - QC2 S2 m1: lỗi đọc GPIO = khoá mở (fail-safe)
            st, err = {"ok": False}, e
        else:
            err = None
        if not st["ok"]:
            self.dock_fault = (f"không đọc được khoá trạm khi đang chạy ({err})" if err is not None else
                               "khoá trạm mở khi đang chạy: " + ", ".join(p for p in spec.DOCK_PINS if not st.get(p)))
            self.emergency_stop()
            raise DockOpen(self.dock_fault)

    # ---------- bật/tắt ----------
    def enable(self) -> None:
        """Kiểm chốt E-stop và tư thế, đặt chế độ, giới hạn mô-men vai, rồi bật mô-men.

        Tay ngoài giới hạn mềm quá soft_limit_tol (vd nằm ở chặn cơ khí vai -88° sau mất điện): ném OutOfSoftLimits,
        KHÔNG bật mô-men (bật thì servo kẹp về MIN/MAX_ANGLE và giật). Đưa tay về dải bằng tay rồi enable() lại.
        """
        self._guard("enable", motion=False)
        if self.watchdog_fault:
            raise WatchdogHet(f"chưa xoá lỗi: {self.watchdog_fault} (kiểm tra máy chủ rồi gọi clear_fault())")
        if self.run_fault:
            cls, ma, msg = self.run_fault
            raise cls(f"chưa xoá lỗi: {msg} (kiểm tra rồi gọi clear_fault())", ma=ma)
        if self.fault:
            raise PairFault(f"chưa xoá lỗi: {self.fault} (kiểm tra rồi gọi clear_fault())")
        if self.lift_fault:
            raise LiftFault(f"chưa xoá lỗi trục nâng: {self.lift_fault} (kiểm tra rồi gọi clear_fault())")
        self._check_latch("enable")
        self._check_dock("enable")
        self._check_voltage_enable()
        if not spec.LIFT_LIMIT_COVERS_DESIGN_LOAD:
            warnings.warn(f"lift: giới hạn mô-men cho {spec.LIFT_FORCE_LIMIT_N:.1f} N < {spec.LIFT_HOLD_FORCE_N:.1f} N tải thiết kế; "
                          + ("cho phép chạy lift theo allow_lift_underpowered=True (chỉ thử có giám sát)" if self.allow_lift_underpowered
                             else "lệnh lift sẽ bị từ chối"), RuntimeWarning, stacklevel=2)
        budget = self.current_budget()
        if not budget["ok"]:
            raise CurrentBudgetExceeded(f"tổng dòng ở giới hạn mô-men {budget['allowed_a']:.1f} A > {budget['budget_a']:.0f} A "
                                        "(F1 MEGA 80 A): hạ TORQUE_LIMIT_PCT trong spec.py")
        q = self.read_joints()
        bad = limits.violations({n: q[n] for n in ARM_NAMES}, tol=self.soft_limit_tol)
        if bad:
            raise OutOfSoftLimits("tay ngoài giới hạn mềm, không bật mô-men: " + "; ".join(bad)
                                  + ". Đưa tay về trong dải bằng tay (mô-men đang tắt) rồi gọi enable() lại")
        for n, sid in self.ids.items():
            mode = P.MODE_WHEEL if n in WHEELS else P.MODE_POSITION
            if self.bus.read(sid, P.MODE) != mode:
                self.bus.write(sid, P.TORQUE_ENABLE, 0)
                self.bus.unlock_eeprom(sid)
                try:
                    self.bus.write(sid, P.MODE, mode)
                finally:
                    self.bus.lock_eeprom(sid)
        # giữ nguyên vị trí hiện tại trước khi bật để không giật
        cur = self.bus.sync_read(P.PRESENT_POSITION, [self.ids[n] for n in POS_NAMES + self.pair_b])
        self.bus.sync_write(P.GOAL_POSITION, cur)
        self.bus.sync_write(P.TORQUE_LIMIT, self.torque_limits())
        bad = {n: (self.bus.read(i, P.TORQUE_LIMIT), want) for n, i, want in self._torque_checks()}
        bad = {n: v for n, v in bad.items() if v[0] != v[1]}
        if bad:    # lift: không ghi/đọc lại được thì không bật (răng PETG gãy ở mô-men chặn, R2 7.2)
            raise RuntimeError(f"TORQUE_LIMIT không khớp sau khi ghi (đọc, ghi): {bad}")
        self.bus.write(self.ids["lift"], P.GOAL_SPEED, self.lift_speed_raw())
        self.base_ramp.reset()
        acc = wheel_acc_raw()
        self.bus.sync_write(P.ACCELERATION, {self.ids[n]: acc for n in WHEELS})   # chặn phụ gia tốc bánh (RAM)
        bad = {n: self.bus.read(self.ids[n], P.ACCELERATION) for n in WHEELS}
        bad = {n: v for n, v in bad.items() if v != acc}
        if bad:
            raise RuntimeError(f"ACCELERATION bánh không khớp sau khi ghi (đọc, ghi {acc}): {bad}")
        self._cmd = self.read_joints()
        on = {sid: 1 for sid in self.ids.values()}
        if self.lift_latch is not None:          # chốt đang cài giữ tải: lift chỉ bật mô-men khi rút chốt (lift_unlock)
            on.pop(self.ids["lift"], None)
        self.bus.sync_write(P.TORQUE_ENABLE, on)
        self._lift_torque = self.lift_latch is None
        self._lift_idle_s, self._lift_jam = 0.0, 0
        self._reset_monitors()
        self.enabled = True
        # QC2 vòng 2 (race): E-stop có thể đến trong lúc enable() — luồng nền tắt mô-men rồi lệnh trên lại bật.
        # Kiểm lại chốt SAU lần ghi cuối; nếu đang dừng thì tắt mô-men lần nữa và báo lỗi.
        try:
            self._check_latch("enable")
        except Exception:
            self.emergency_stop()
            raise

    def disable(self) -> None:
        """Dừng thường. Có chốt trục nâng: cài chốt trước (servo giữ -> cài -> cảm biến -> tắt mô-men lift), rồi tắt mô-men
        mọi servo. Chốt không cài: servo lift VẪN GIỮ, các servo khác tắt, ném LiftFault."""
        self.set_base_velocity(0, 0, 0)
        lift_err = None
        if self.lift_latch is not None and self._lift_torque:
            try:
                self.lift_lock()
            except LiftFault as e:
                lift_err = e
        off = {sid: 0 for sid in self.ids.values()}
        if lift_err is not None:
            off.pop(self.ids["lift"], None)
        self.bus.sync_write(P.TORQUE_ENABLE, off)
        self.enabled = False
        if lift_err is not None:
            raise lift_err
        self._lift_torque = False

    def emergency_stop(self) -> None:
        """Tắt mô-men mọi servo ngay (dùng làm on_stop của SafetyLatch) và cắt cuộn chốt trục nâng (phần cứng cũng cắt cùng
        SERVO+). Không gửi lệnh bánh trước để nhanh nhất. Lift rơi tối đa một nấc chốt (spec.LIFT_NOTCH_PITCH)."""
        try:
            if self.lift_latch is not None:
                self.lift_latch.cut()          # cuộn chốt trước: chốt bắt đầu ra trong lúc servo còn giữ (QC2 vòng 1)
        finally:
            try:
                if not self.bus.try_call("sync_write", P.TORQUE_ENABLE, {sid: 0 for sid in self.ids.values()},
                                         timeout=spec.ESTOP_BUS_LOCK_S):
                    log.error("[SAF-01] dừng khẩn: khoá bus bận quá %.2f s, bỏ ghi TORQUE_ENABLE=0 (chốt phần cứng cắt nguồn servo)",
                              spec.ESTOP_BUS_LOCK_S)
            except (BusMat, RobotDaDong):
                pass                           # bus đã mất/đã đóng: chốt E-stop phần cứng là đường dừng
            finally:
                self.enabled = False
                self._lift_torque = False
                self.base_ramp.reset()
                self._wheel_cmd = np.zeros(3)

    # ---------- trục nâng: chốt hãm (R2 7.2) ----------
    def _lift_hold(self) -> None:
        """Servo lift giữ vị trí hiện tại (đích = vị trí đọc), bật mô-men riêng lift."""
        sid = self.ids["lift"]
        raw = self.bus.read(sid, P.PRESENT_POSITION)
        self.bus.write(sid, P.GOAL_POSITION, raw)
        self.bus.write(sid, P.TORQUE_ENABLE, 1)
        self._lift_torque = True
        self._cmd["lift"] = self.from_raw("lift", P.decode_signed(raw))

    def _lift_torque_off(self) -> None:
        self.bus.write(self.ids["lift"], P.TORQUE_ENABLE, 0)
        self._lift_torque = False

    def _lift_error(self, why: str, engage: bool = True) -> None:
        """Dừng lift tại chỗ và chặn lệnh tới clear_fault(). Đang có mô-men: servo giữ, rồi (engage=True) thử cài chốt theo trình
        tự (cuộn tắt -> cảm biến "cài" -> mới tắt mô-men lift) để xe không trượt dần khi servo yếu/kẹt. Chốt không cài: servo
        VẪN GIỮ, lý do ghi vào lỗi."""
        self.lift_fault = why
        if self._lift_torque:
            try:
                self._lift_hold()
            except Exception:                  # noqa: BLE001 - lỗi bus khi giữ: vẫn báo lỗi gốc
                pass
            if engage and self.lift_latch is not None:
                try:
                    self.lift_latch.engage(self._lift_hold, self._lift_torque_off)
                except Exception as e:         # noqa: BLE001
                    why = f"{why}; chốt KHÔNG cài ({e}), servo lift vẫn giữ"
                    self.lift_fault = why
        raise LiftFault(f"trục nâng dừng: {why}")

    def _estop_ok(self) -> bool:
        lt = self._latch
        if lt is None:
            return self.allow_no_latch
        return lt.monitor_alive and lt.ok

    def lift_unlock(self) -> None:
        """Rút chốt theo R2 7.2: servo lift giữ vị trí hiện tại -> rút chốt -> chờ cảm biến "rút". Gọi tự động khi lệnh lift đổi."""
        if self.lift_latch is None:
            return
        self._check_latch("lift_unlock")
        self._check_dock("lift_unlock")        # QC2 S2 m2: không rút chốt lift khi trạm chưa khoá
        try:
            self.lift_latch.release(self._lift_hold, self._estop_ok)
        except LiftLatchError as e:
            if not self._estop_ok():           # E-stop giữa chừng: hold() có thể đã bật lại mô-men lift -> tắt hết lần nữa
                self.emergency_stop()
                raise EStopActive(f"lift_unlock: dừng khẩn trong lúc rút chốt ({e})") from e
            self._lift_error(str(e))
        self._lift_idle_s = 0.0

    def lift_lock(self) -> None:
        """Cài chốt: servo giữ -> cài -> cảm biến "cài" -> tắt mô-men lift. Tự gọi khi lift nghỉ quá spec.LIFT_LATCH_IDLE_S."""
        if self.lift_latch is None:
            return
        try:
            self.lift_latch.engage(self._lift_hold, self._lift_torque_off)
        except LiftLatchError as e:
            self._lift_error(str(e), engage=False)

    def _check_lift(self, dt: float) -> None:
        """Mỗi step(): chốt rơi khi chạy, kẹt/quá mô-men (servo đẩy vào chốt đang ra), cài chốt khi nghỉ lâu."""
        lt = self.lift_latch
        if lt is None or not self.enabled or self.lift_fault:      # R-23: lỗi đã báo một lần, không ném lại mỗi chu kỳ
            return
        if lt.state == "retracted":
            lt.motion_allowed()                # cảm biến rớt khi đang cho chạy -> lt.fault
        if lt.fault:
            self._lift_error(lt.fault)
        if self._lift_torque:
            load = abs(P.decode_signed(self.bus.read(self.ids["lift"], P.PRESENT_LOAD), P.LOAD_SIGN_BIT))
            thr = spec.LIFT_JAM_FRACTION * spec.LIFT_TORQUE_LIMIT_PERMILLE
            if load >= thr and lt.state != "retracted":
                self._lift_error(f"servo lift đẩy vào chốt chưa rút: tải {load}‰ >= {thr:.0f}‰ (kẹt)")
            self._lift_jam = self._lift_jam + 1 if load >= thr else 0
            if self._lift_jam >= spec.LIFT_JAM_CYCLES:
                self._lift_error(f"quá mô-men lift {self._lift_jam} chu kỳ liền (tải {load}‰ >= {thr:.0f}‰): kẹt răng/chốt hoặc vật cản")
        if lt.state == "retracted":
            self._lift_idle_s += dt
            if self._lift_idle_s >= spec.LIFT_LATCH_IDLE_S:
                self.lift_lock()

    # ---------- Backend ----------
    def _sync_read(self, reg, names: list[str]) -> dict[int, int]:
        """sync_read; servo của cặp ghép đôi im lặng thì dừng an toàn cả cặp (PairFault)."""
        try:
            return self.bus.sync_read(reg, [self.ids[n] for n in names])
        except SyncReadTimeout as e:
            got = set()
            name_of = {self.ids[n]: n for n in names}
            missing = [name_of[i] for i in e.missing if i in name_of]
            for j, b in self.pairs.items():
                if j in missing or b in missing:
                    got.add(j)
            if got:
                self._trip(sorted(got), f"servo không phản hồi: {missing}")
            raise
        except P.ServoError as e:
            name = {self.ids[n]: n for n in names}.get(e.sid)
            joint = next((j for j, b in self.pairs.items() if name in (j, b)), None)
            if joint is not None:
                self._trip([joint], f"{name} báo lỗi: {P.error_text(e.err)}")
            raise
        except P.ProtocolError as e:
            # / CH-8: gói hỏng (sai checksum) làm mất cả lần đọc, không biết chắc servo nào: coi mọi cặp có trong lần đọc
            # như servo im lặng, tắt mô-men các cặp đó (không đọc lại).
            got = sorted({j for j, b in self.pairs.items() if j in names or b in names})
            if got:
                self._trip(got, f"gói hỏng khi đọc ({e})")
            raise

    def read_joints(self) -> dict[str, float]:
        """Vị trí mọi khớp. Servo im/gói hỏng/bit lỗi -> lỗi có mã (ServoMat BUS-03/BUS-02, ServoBaoLoi); không tự dừng (step() lo)."""
        self._guard("read_joints", motion=False)
        names = POS_NAMES + self.pair_b
        try:
            raw = self._sync_read(P.PRESENT_POSITION, names)
        except SyncReadTimeout as e:
            txt = ", ".join(f"{self._name_of.get(i, i)} (ID {i})" for i in e.missing)
            raise ServoMat(f"read_joints: servo không trả lời: {txt}", ma="BUS-03",
                           khop=self._name_of.get(e.missing[0]) if e.missing else None) from e
        except P.ServoError as e:
            raise ServoBaoLoi(f"read_joints: {self._name_of.get(e.sid, e.sid)} (ID {e.sid}) báo lỗi: {P.error_text(e.err)}",
                              khop=self._name_of.get(e.sid)) from e
        except P.ProtocolError as e:
            raise ServoMat(f"read_joints: gói trả hỏng ({e})", ma="BUS-02") from e
        val = {}
        for n in names:
            r = raw[self.ids[n]]
            if n == "lift":
                r = P.decode_signed(r)
            val[n] = self.from_raw(n, r)
        out = {n: val[n] for n in POS_NAMES}
        for j, b in self.pairs.items():
            out[j] = 0.5 * (val[j] + val[b])
        return out

    def commanded(self) -> dict[str, float]:
        """Đích đã ra lệnh gần nhất (gốc mà write_joints dùng để kẹp và chặn đoạn)."""
        return dict(self._cmd)

    def write_joints(self, targets: dict[str, float]) -> dict[str, float]:
        self._guard("write_joints")
        if self.fault:
            raise PairFault(f"đang dừng do lỗi: {self.fault} (kiểm tra rồi gọi clear_fault())")
        if self.lift_fault:
            raise LiftFault(f"đang dừng do lỗi trục nâng: {self.lift_fault} (kiểm tra rồi gọi clear_fault())")
        self._check_latch("write_joints")
        targets = dict(targets)
        if "lift" in targets and abs(targets["lift"] - self._cmd["lift"]) > 1e-9:
            self._check_dock("lift")
        if "lift" in targets:    # công tắc hành trình: chạm đầu nào thì không đi tiếp chiều đó (giữ tại vị trí lệnh gần nhất)
            lt = self.lift_latch
            if abs(targets["lift"] - self._cmd["lift"]) > 1e-9 and not (spec.LIFT_LIMIT_COVERS_DESIGN_LOAD
                                                                         or self.allow_lift_underpowered):
                raise LiftUnderpowered(
                    f"giới hạn mô-men lift {spec.LIFT_TORQUE_LIMIT_PERMILLE}‰ cho {spec.LIFT_FORCE_LIMIT_N:.1f} N < "
                    f"{spec.LIFT_HOLD_FORCE_N:.1f} N cần cho tải thiết kế {spec.LIFT_DESIGN_LOAD_KG} kg (F_đo răng chưa đo, "
                    "DK-N18/N20): không chạy lift. Thử không tải có giám sát: FeetechBackend(allow_lift_underpowered=True)")
            if lt is not None and abs(targets["lift"] - self._cmd["lift"]) > 1e-9:
                if not lt.motion_allowed():    # chốt đang cài: rút theo trình tự trước khi cho chạy
                    if lt.fault:
                        self._lift_error(lt.fault)
                    self.lift_unlock()
                self._lift_idle_s = 0.0
            targets["lift"] = limits.lift_gate(self._cmd["lift"], targets["lift"], self.lift_switches, lt)
        sent = limits.safe_step(self._cmd, limits.clamp(targets, self._cmd))
        vals = {}
        for n, v in sent.items():
            for sname in ((n, self.pairs[n]) if n in self.pairs else (n,)):
                r = self.to_raw(sname, v)
                vals[self.ids[sname]] = P.encode_signed(r) if n == "lift" else max(0, min(4095, r))
        if vals:
            self.bus.sync_write(P.GOAL_POSITION, vals)
        self._cmd.update(sent)
        return sent

    # ---------- cặp servo ghép đôi ----------
    def read_pair_loads(self) -> dict[str, tuple[float, float]]:
        """Tải hai servo mỗi cặp, N·m quy về khung khớp (nhân sign hiệu chuẩn). Tải = ‰ mô-men chặn [giả định tuyến tính]."""
        names = list(self.pairs) + self.pair_b
        raw = self._sync_read(P.PRESENT_LOAD, names)
        out = {}
        for j, b in self.pairs.items():
            stall = spec.ARM_SERVO[j.partition("_")[2]].stall_nm
            la, lb = (self.cal[n]["sign"] * P.decode_signed(raw[self.ids[n]], P.LOAD_SIGN_BIT) / 1000.0 * stall for n in (j, b))
            out[j] = (la, lb)
        return out

    def check_pairs(self) -> dict:
        """Một chu kỳ giám sát cặp: chênh vị trí (rad) và chênh tải (tỉ lệ). Vượt ngưỡng pair_fault_cycles chu kỳ liên tiếp
        (hoặc một servo im lặng: ngay) thì tắt mô-men cả cặp, ghi self.fault và ném PairFault."""
        names = list(self.pairs) + self.pair_b
        pos = self._sync_read(P.PRESENT_POSITION, names)
        loads = self.read_pair_loads()
        bad, metrics = [], {}
        for j, b in self.pairs.items():
            qa, qb = (self.from_raw(n, pos[self.ids[n]]) for n in (j, b))  # public-scan: ok
            la, lb = loads[j]
            stall = spec.ARM_SERVO[j.partition("_")[2]].stall_nm
            ref = max(0.5 * (abs(la) + abs(lb)), self.pair_load_floor * stall)
            m = {"pos_diff_deg": math.degrees(abs(qa - qb)), "load_diff": abs(la - lb) / ref, "load_nm": (la, lb)}  # public-scan: ok
            metrics[j] = m
            over = abs(qa - qb) > self.pair_max_pos or m["load_diff"] > self.pair_max_load_diff  # public-scan: ok
            self._pair_bad[j] = self._pair_bad[j] + 1 if over else 0
            if self._pair_bad[j] >= self.pair_fault_cycles:
                bad.append(j)
        self.pair_metrics = metrics
        if bad:
            why = "; ".join(f"{j}: chênh góc {metrics[j]['pos_diff_deg']:.1f}°, chênh tải {100 * metrics[j]['load_diff']:.0f}%" for j in bad)
            self._trip(bad, why)
        return metrics

    def _trip(self, joints: list[str], why: str) -> None:
        ids = {}
        for j in joints:
            ids[self.ids[j]] = 0
            ids[self.ids[self.pairs[j]]] = 0
        self.bus.sync_write(P.TORQUE_ENABLE, ids)   # broadcast: servo còn sống vẫn nhận
        self.fault = f"{', '.join(joints)}: {why}"
        raise PairFault(f"dừng an toàn (tắt mô-men cả cặp) {self.fault}")

    def clear_fault(self) -> None:
        """Xoá lỗi cặp, lỗi khi chạy (servo/điện áp/watchdog) sau khi đã kiểm tra; phải gọi enable() lại. Bus mất (BUS-01) không xoá được."""
        self.fault = None
        self.run_fault = None
        self.watchdog_fault = None
        self._reset_monitors()
        self._pair_bad = {j: 0 for j in self.pairs}
        self.lift_fault = None
        self.dock_fault = None
        self._lift_jam = 0
        if self.lift_latch is not None:
            self.lift_latch.clear_fault()
        self.enabled = False

    # ---------- giới hạn mềm trong servo ----------
    def lift_speed_raw(self) -> int:
        """GOAL_SPEED (bước/s) của servo lift ứng với spec.LIFT_SPEED_MAX."""
        return int(round(spec.LIFT_SPEED_MAX * self.cal["lift"]["counts_per_m"]))

    def _torque_checks(self):
        for n, per in self.torque_limits_by_name().items():
            yield n, self.ids[n], per

    def torque_limits_by_name(self) -> dict[str, int]:
        """TORQUE_LIMIT (‰ mô-men chặn) cho MỌI servo trên bus theo nhóm (spec.TORQUE_LIMIT_PCT; vai còn bị SHOULDER_TORQUE_LIMIT_KGCM
        chia cho số servo của khớp; lift = spec.LIFT_TORQUE_LIMIT_PERMILLE). Trước v0.5 chỉ đặt vai và lift."""
        return limits.torque_limits(self.ids)

    def current_budget(self) -> dict:
        """Ngân sách dòng (A) với giới hạn mô-men đang đặt: limits.current_budget trên các servo của bus."""
        return limits.current_budget(self.ids, limits_permille=self.torque_limits_by_name())

    def torque_limits(self) -> dict[int, int]:
        """TORQUE_LIMIT theo ID bus (xem torque_limits_by_name)."""
        return {self.ids[n]: v for n, v in self.torque_limits_by_name().items()}

    def _arm_servos(self):
        for n in ARM_NAMES:
            yield n, n
            if n in self.pairs:
                yield n, self.pairs[n]

    def load_soft_limits(self) -> None:
        """Ghi MIN/MAX_ANGLE (EEPROM) theo spec.ARM_LIMITS cho mọi servo tay (cả servo B). Chỉ khi mô-men đang tắt.
        Giới hạn ghép vai-khuỷu không nạp được vào servo (servo không biết góc khớp khác): SDK áp ở write_joints."""
        if self.enabled:
            raise RuntimeError("tắt mô-men (disable) trước khi ghi EEPROM giới hạn")
        for j, n in self._arm_servos():
            lo, hi = spec.ARM_LIMITS[j.partition("_")[2]]
            a, b = sorted((self.to_raw(n, lo), self.to_raw(n, hi)))
            sid = self.ids[n]
            self.bus.unlock_eeprom(sid)
            try:
                self.bus.write(sid, P.MIN_ANGLE, max(0, a))
                self.bus.write(sid, P.MAX_ANGLE, min(4095, b))
            finally:
                self.bus.lock_eeprom(sid)

    def read_soft_limits(self) -> dict:
        """Đọc lại MIN/MAX_ANGLE từ từng servo tay (độ, khung khớp) + TORQUE_LIMIT vai + giới hạn ghép SDK đang áp."""
        out = limits.loaded()
        servo = {}
        for j, n in self._arm_servos():
            sid = self.ids[n]
            a, b = self.bus.read(sid, P.MIN_ANGLE), self.bus.read(sid, P.MAX_ANGLE)
            servo[n] = tuple(sorted(round(math.degrees(self.from_raw(n, r)), 2) for r in (a, b)))
        out["servo_deg"] = servo
        out["lift_torque_limit_permille"] = self.bus.read(self.ids["lift"], P.TORQUE_LIMIT)
        out["torque_limit_permille"] = {n: self.bus.read(i, P.TORQUE_LIMIT) for n, i in self.ids.items()}
        out["shoulder_torque_limit_permille"] = {n: self.bus.read(self.ids[n], P.TORQUE_LIMIT)
                                                 for s in spec.SIDES for n in (spec.joint_name(s, "shoulder"), spec.pair_name(spec.joint_name(s, "shoulder")))
                                                 if n in self.ids}
        return out

    def verify_soft_limits(self, tol_deg: float = 0.2) -> list[str]:
        """So giới hạn đọc lại với spec.ARM_LIMITS; rỗng = đạt (ghi vào phiếu IPQC)."""
        got = self.read_soft_limits()["servo_deg"]
        bad = []
        for j, n in self._arm_servos():
            want = tuple(math.degrees(v) for v in spec.ARM_LIMITS[j.partition("_")[2]])
            if any(abs(g - w) > tol_deg for g, w in zip(got[n], want)):
                bad.append(f"{n}: servo {got[n]} khác spec {tuple(round(w, 2) for w in want)}")
        return bad

    def set_base_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Đích vận tốc đế: kẹp |v| <= BASE_MAX_V và ω theo chống lật; bánh đi theo dốc BaseRamp, mỗi step() một bước
        (đặt đích nhiều lần trong một chu kỳ không làm dốc nhanh hơn). Không gọi step() thì bánh giữ lệnh cũ."""
        if vx or vy or wz:
            self._guard("đế")
            self._check_dock("đế")
        elif self._closed:
            raise RobotDaDong("đế: robot đã đóng (close())")
        self.base_ramp.set_goal(vx, vy, wz)

    def base_velocity(self) -> np.ndarray:
        """Vận tốc đế đang lệnh (sau dốc), khung robot."""
        return self.base_ramp.cur.copy()

    def _write_wheels(self, v) -> None:
        if self.watchdog_fault:                # luồng watchdog vừa dừng bánh: không ghi đè tốc độ cũ
            raise WatchdogHet(f"bánh: {self.watchdog_fault}")
        w = kinematics.wheel_speeds(*v)
        if not np.any(self._wheel_cmd) and np.any(w):
            self._last_step = time.monotonic()  # QC2 M1: bánh vừa bắt đầu quay, mốc watchdog tính từ đây
        self._wheel_cmd = w
        steps = {self.ids[n]: P.encode_signed(int(round(self.cal[n]["sign"] * w[i] * P.STEPS_PER_REV / (2 * math.pi))))
                 for i, n in enumerate(WHEELS)}
        self.bus.sync_write(P.GOAL_SPEED, steps)

    def read_wheel_speeds(self) -> np.ndarray:
        raw = self.bus.sync_read(P.PRESENT_SPEED, [self.ids[n] for n in WHEELS])
        return np.array([self.cal[n]["sign"] * P.decode_signed(raw[self.ids[n]]) * 2 * math.pi / P.STEPS_PER_REV
                         for n in WHEELS])

    def base_pose(self) -> np.ndarray:
        return self._pose.copy()

    def _check_dt(self, dt) -> float:
        """QC2 m4: dt ở lớp backend (core plugin gọi thẳng): số thực hữu hạn > 0. Trần dt chỉ ở Robot.step (backend giữ dt lớn cho test/tích luỹ)."""
        if isinstance(dt, bool) or not isinstance(dt, numbers.Real) or not math.isfinite(dt) or dt <= 0:
            raise LenhKhongHopLe(f"step: dt phải là số giây hữu hạn > 0, nhận {dt!r}")
        return float(dt)

    def step(self, dt: float) -> None:
        """Một chu kỳ: chờ dt, chốt E-stop (luồng giám sát chết -> EStopActive; chốt không 'run' thì Robot.step báo, R-16), cặp, lift, khoá trạm, dốc đế -> bánh, rồi đọc mọi servo
        một lần (_poll: lỗi servo, mất liên lạc, điện áp; tốc độ bánh cho odometry)."""
        self._guard("step")
        dt = self._check_dt(dt)
        self._last_step = time.monotonic()      # QC2 M1: mốc watchdog ở ĐẦU step() và sau lúc chờ, không chỉ ở cuối
        self._sleep(dt)
        self._last_step = time.monotonic()
        if self._latch is not None:
            self._latch.poll()
            if not self._latch.monitor_alive and self.enabled:
                self.emergency_stop()
                raise EStopActive(f"luồng giám sát E-stop không chạy ({self._latch.monitor_error or 'đã dừng'})")
        if self.monitor_pairs and self.enabled:
            self.check_pairs()
        self._check_lift(dt)
        self._watch_dock()
        if not self.base_ramp.settled:
            self._write_wheels(self.base_ramp.step(dt))
        w = self._poll(dt)
        if w is None:                          # thiếu gói bánh chu kỳ này: odometry theo lệnh
            w = self._wheel_cmd
        vx, vy, wz = kinematics.base_twist(w)
        yaw = self._pose[2]
        self._pose += np.array([math.cos(yaw) * vx - math.sin(yaw) * vy,
                                math.sin(yaw) * vx + math.cos(yaw) * vy, wz]) * dt
        self._last_step = time.monotonic()

    def camera(self, name: str = "head"):
        import cv2  # tuỳ chọn: pip install opencv-python
        idx = {"head": 0, "left_wrist_cam": 1, "right_wrist_cam": 2}[name]
        cap = cv2.VideoCapture(idx)
        ok, frame = cap.read()
        cap.release()
        if not ok:
            raise RuntimeError(f"không đọc được camera {name}")
        return frame[:, :, ::-1]

    def close(self) -> None:
        """Dừng (disable, hoặc dừng khẩn nếu bus đã mất), đánh dấu đã đóng (luồng khác đang chạy lệnh nhận RobotDaDong ở giao dịch
        bus kế tiếp), dừng luồng chốt, hạ chân cho phép, đóng cổng. Gọi lại nhiều lần không lỗi; lỗi bus khi đóng không ném."""
        if self._closed:
            return
        err = None
        try:
            with self.bus._lock:               # luồng khác không chen gói giữa lúc tắt mô-men và lúc đánh dấu đóng
                try:
                    if self.enabled and not self.bus_lost:
                        try:
                            self.disable()
                        except (BusMat, RobotDaDong):
                            self.emergency_stop()
                        except BaseException as e:
                            self.emergency_stop()      # chốt không cài: vẫn tắt hết khi đóng (lift rơi tối đa một nấc)
                            err = e
                    elif self.enabled:
                        self.emergency_stop()
                finally:
                    self._closed = True
        finally:
            try:
                if self._latch is not None:
                    self._latch.stop_monitor()
                    self._latch.on_tick = None
                self._release_gpio()
            finally:
                self._close_port()
        if err is not None:
            raise err

    def _release_gpio(self) -> None:
        """Hạ chân cho phép (K5_EN / RELAY_EN) qua chốt, kể cả khi chốt do người gọi truyền vào; đóng GPIO nếu backend tự mở."""
        lt = getattr(self, "_latch", None)
        pin = getattr(lt, "enable_pin", None) or getattr(lt, "relay_pin", None)
        try:
            if lt is not None and pin is not None:
                with lt._lock:
                    lt.gpio.write(pin, False)
                    if hasattr(lt, "_en"):
                        lt._en = False
                    if lt.state in ("run", "arming", "resetting"):
                        lt.state = "stopped"
                        lt._event("đóng backend: hạ chân cho phép")
        finally:
            g = getattr(self, "_owned_gpio", None)
            if g is not None and hasattr(g, "close"):
                g.close()
