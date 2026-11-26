# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""API thống nhất `Robot` cho M1: cùng một script chạy với backend "sim" hoặc "feetech".

Tầng 1b: mọi đầu vào được kiểm trước khi đổi trạng thái (NaN/inf -> LenhKhongHopLe, kiểu sai -> KieuLenhSai, tên khớp sai ->
KhopKhongCo, đều có mã .ma); một chủ sở hữu (luồng đầu tiên ra lệnh; luồng khác -> RobotBan, close() thì luồng nào gọi cũng được);
sau close() mọi lệnh -> RobotDaDong.
"""
from __future__ import annotations

import math
import numbers
import threading
from collections.abc import Mapping

import numpy as np

from . import kinematics, limits, spec
from .khay import KhayTai
from .loi import KhopKhongCo, KieuLenhSai, LenhKhongHopLe, M1Loi, RobotBan, RobotDaDong

_JOINTS = spec.all_joint_names()
_JOINT_SET = frozenset(_JOINTS)


def _smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def _so(v, what: str) -> float:
    """Số thực (int, float, số numpy, mảng numpy 0 chiều) -> float; kiểu khác -> KieuLenhSai nêu `what`."""
    if isinstance(v, np.ndarray):
        if v.ndim == 0 and v.dtype.kind in "fiu":
            v = v.item()
        else:
            raise KieuLenhSai(f"{what}: cần một số thực, nhận mảng {v.shape}")
    if isinstance(v, bool) or not isinstance(v, numbers.Real):
        raise KieuLenhSai(f"{what}: cần một số thực, nhận kiểu {type(v).__name__}")
    return float(v)


def _huu_han(v, what: str) -> float:
    x = _so(v, what)
    if not math.isfinite(x):
        raise LenhKhongHopLe(f"{what}: giá trị {x} không hữu hạn")
    return x


def _vec3(p, what: str) -> np.ndarray:
    if p is None or isinstance(p, (str, bytes, Mapping)):
        raise KieuLenhSai(f"{what}: cần 3 số (x, y, z), nhận kiểu {type(p).__name__}")
    try:
        items = list(p)
    except TypeError:
        raise KieuLenhSai(f"{what}: cần 3 số (x, y, z), nhận kiểu {type(p).__name__}") from None
    if len(items) != 3:
        raise LenhKhongHopLe(f"{what}: cần đúng 3 số (x, y, z), nhận {len(items)}")
    return np.array([_huu_han(x, f"{what}[{i}]") for i, x in enumerate(items)])


class Robot:
    """Robot Faibotic M1.

    >>> robot = Robot(backend="sim")
    >>> robot.move_joints({"lift": 0.35}, duration=1.5)   # trục vai 0,90 m
    >>> robot.move_tcp("left", [0.35, 0.10, 0.80], duration=2)
    """

    def __init__(self, backend: str = "sim", lift_speed_max="auto", max_joint_step_deg: float | None = None,
                 khay: KhayTai | None = None, **kwargs):
        """lift_speed_max: m/s kẹp tốc độ trục nâng. "auto": spec.LIFT_SPEED_MAX cho phần cứng (feetech), không kẹp ở sim nhanh (demo/test);
        sim thật theo thiết kế: Robot("sim", lift_speed_max=spec.LIFT_SPEED_MAX, realistic_lift=True).
        max_joint_step_deg: bước đích khớp tay lớn nhất mỗi chu kỳ điều khiển của move_joints/move_tcp (None = spec.ARM_MAX_STEP_DEG);
        lệnh ngắn hơn mức này thì tự kéo dài thời gian (S1C: test tự chơi yêu cầu <= 2 độ/khung).
        khay: vật trên khay đế (KhayTai), như set_khay_tai()."""
        if backend == "sim":
            from .backends.sim import SimBackend
            self.backend = SimBackend(**kwargs)
        elif backend == "feetech":
            from .backends.feetech import FeetechBackend
            self.backend = FeetechBackend(**kwargs)
        else:
            raise ValueError(f'backend phải là "sim" hoặc "feetech", nhận "{backend}"')
        self.dt = self.backend.control_dt
        self._targets = dict(self.backend.read_joints())
        self.lift_vmax = (spec.LIFT_SPEED_MAX if backend == "feetech" else None) if lift_speed_max == "auto" else lift_speed_max
        self._lift_goal = self._targets["lift"]
        self.max_joint_step = math.radians(spec.ARM_MAX_STEP_DEG if max_joint_step_deg is None else max_joint_step_deg)
        # G1/E3 (24/11): vùng cấm trạm (đầu/cột camera, bệ, khay hoặc mặt bàn) chặn ở lớp Robot; cấu hình theo backend (feetech tự nhận), sim = di_dong
        self.config = getattr(self.backend, "config", "di_dong")
        self.keepout = True
        self._closed = False
        self._owner: threading.Thread | None = None      # R-37: một chủ sở hữu (luồng đầu tiên ra lệnh)
        self._owner_lock = threading.Lock()
        self.khay: KhayTai | None = None
        if khay is not None:
            self.set_khay_tai(khay)

    def _steps_for(self, dq_max: float, n: int, peak: float = 1.5) -> int:
        """Số chu kỳ để bước lớn nhất (hồ sơ trơn, đỉnh = peak x trung bình) không vượt max_joint_step."""
        return max(n, int(math.ceil(peak * dq_max / self.max_joint_step))) if self.max_joint_step > 0 else n

    # ---------- canh cổng: đã đóng, một chủ sở hữu ----------
    def _guard(self, action: str) -> None:
        if self._closed:
            raise RobotDaDong(f"{action}: robot đã đóng (close()); tạo Robot mới để chạy tiếp")
        me = threading.current_thread()
        with self._owner_lock:
            own = self._owner
            if own is None or own is me or not own.is_alive():
                self._owner = me
                return
        raise RobotBan(f"{action}: robot đang do luồng '{own.name}' điều khiển; mỗi robot chỉ một luồng ra lệnh "
                       "(gọi release_owner() ở luồng đó để chuyển, hoặc gom lệnh về một luồng)")

    def release_owner(self) -> None:
        """Nhả quyền điều khiển của luồng hiện tại (luồng khác ra lệnh kế tiếp sẽ thành chủ)."""
        with self._owner_lock:
            if self._owner is threading.current_thread():
                self._owner = None

    def _duration(self, d, what: str) -> float:
        d = _huu_han(d, f"{what}: duration")
        if d < 0:
            raise LenhKhongHopLe(f"{what}: duration phải >= 0 giây, nhận {d}")
        if d > spec.MAX_MOVE_DURATION_S:
            raise LenhKhongHopLe(f"{what}: duration {d:g} s vượt trần {spec.MAX_MOVE_DURATION_S:g} s (spec.MAX_MOVE_DURATION_S)")
        return d

    def _check_targets(self, targets, what: str = "set_joint_positions") -> dict[str, float]:
        """Kiểm lệnh khớp trước khi đổi bất kỳ trạng thái nào: dict, tên khớp có thật, giá trị là số thực hữu hạn."""
        if not isinstance(targets, Mapping):
            raise KieuLenhSai(f"{what}: cần dict {{tên khớp: giá trị}}, nhận kiểu {type(targets).__name__}")
        out = {}
        for k, v in targets.items():
            if not isinstance(k, str) or k not in _JOINT_SET:
                raise KhopKhongCo(f"{what}: khớp không có: {k!r}; khớp hợp lệ: {', '.join(_JOINTS)}", khop=str(k))
            out[k] = _huu_han(v, f"{what}: khớp {k}")
        return out

    # ---------- vòng đời ----------
    def close(self) -> None:
        """Dừng đế theo dốc (nếu được), tắt mô-men, đóng backend. Gọi từ luồng nào cũng được, gọi lại không lỗi;
        lỗi khi dừng đế (bus mất, luồng khác đang điều khiển) không chặn việc đóng."""
        if self._closed:
            return
        try:
            self.stop_base()
        except (M1Loi, OSError):
            pass
        finally:
            self._closed = True
            self.backend.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    # ---------- đọc ----------
    def get_state(self) -> dict:
        j = self.backend.read_joints()
        return {"joints": j, "lift": j["lift"],
                "gripper": {s: j[spec.joint_name(s, "gripper")] for s in spec.SIDES},
                "base_pose": self.backend.base_pose().tolist()}

    def arm_q(self, side: str, source: str = "state") -> np.ndarray:
        self._check_side(side, "arm_q")
        j = self.backend.read_joints() if source == "state" else self._targets
        return np.array([j[spec.joint_name(side, n)] for n in spec.ARM_JOINTS])

    def tcp_position(self, side: str, source: str = "state") -> np.ndarray:
        """Vị trí điểm kẹp trong khung robot (từ FK)."""
        j = self.backend.read_joints() if source == "state" else self._targets
        p, _ = kinematics.arm_fk(self.arm_q(side, source), side)
        return kinematics.arm_mount(side, j["lift"]) + p

    def world_to_robot(self, p) -> np.ndarray:
        if p is None or isinstance(p, (str, bytes, Mapping)):
            raise KieuLenhSai(f"world_to_robot: cần 2 hoặc 3 số, nhận kiểu {type(p).__name__}")
        try:
            items = list(p)
        except TypeError:
            raise KieuLenhSai(f"world_to_robot: cần 2 hoặc 3 số, nhận kiểu {type(p).__name__}") from None
        if len(items) not in (2, 3):
            raise LenhKhongHopLe(f"world_to_robot: cần 2 hoặc 3 số, nhận {len(items)}")
        p = [_huu_han(v, f"world_to_robot[{i}]") for i, v in enumerate(items)]
        x, y, yaw = self.backend.base_pose()
        d = np.asarray(p, float)[:2] - [x, y]
        c, s = math.cos(yaw), math.sin(yaw)
        out = [c * d[0] + s * d[1], -s * d[0] + c * d[1]]
        return np.array(out + ([float(p[2])] if len(p) > 2 else []))

    def camera(self, name: str = "head") -> np.ndarray:
        return self.backend.camera(name)

    # ---------- lệnh cấp thấp (không chờ) ----------
    def set_joint_positions(self, targets: dict[str, float]) -> dict[str, float]:
        """Đặt đích (không chờ). Kẹp theo giới hạn mềm (limits.clamp); backend chặn thêm đoạn cắt vùng cấm ghép.

        Đầu vào sai -> lỗi có mã, trạng thái không đổi. Vùng cấm trạm kiểm cả đích cuối lẫn tư thế thực gửi xuống backend
        (sau khi backend chặn đoạn ghép safe_step và lift đi theo tốc độ): vi phạm -> KeepoutError, không gửi gì.
        Trả về đích đã thực sự gửi xuống backend.
        """
        self._guard("set_joint_positions")
        clamped = self._clamp(self._check_targets(targets))
        self._check_keepout(clamped)
        send = dict(clamped)
        if "lift" in send and self.lift_vmax:      # đích lift đi theo step() với tốc độ <= lift_vmax; ở đây chỉ phát bước đầu
            send["lift"] = limits.lift_slew(self._targets["lift"], clamped["lift"], self.dt, self.lift_vmax)
        base = {**self._targets, **(self.backend.commanded() or {})}
        pred = limits.safe_step(base, limits.clamp(send, base))
        if pred != clamped:
            self._check_keepout(pred, "tư thế gửi xuống (sau chặn đoạn ghép/tốc độ lift)", base)
        try:
            sent = self.backend.write_joints(send)
        except M1Loi:
            self._lift_goal = self._targets["lift"]     # lỗi (lift, E-stop...): không đuổi đích lift cũ ở step() sau (R-23)
            raise
        if "lift" in clamped:
            self._lift_goal = clamped["lift"]
        sent = send if sent is None else sent
        self._targets.update(sent)
        return sent

    def _check_keepout(self, targets: dict[str, float], what: str = "đích", current: dict | None = None) -> None:
        """Từ chối đích có tay trong vùng cấm trạm (limits.keepout_violations): ném KeepoutError, không gửi gì xuống backend."""
        if not self.keepout:
            return
        bad = limits.keepout_check(targets, self._targets if current is None else current, self.config, self._khay_dinh_z())
        if bad:
            raise limits.KeepoutError(f"{what} trong vùng cấm trạm ({self.config}): " + "; ".join(bad))

    # ---------- khay đế ----------
    def set_khay_tai(self, khay: KhayTai | None) -> None:
        """Khai báo vật trên khay đế (KhayTai) hoặc None khi khay trống. Vật ngoài định mức/vùng cho phép đã bị KhayTai từ chối
        (CauHinhSai, CFG-02). Có vật: lift không xuống dưới khay.lift_min (giới hạn mềm, lệnh thấp hơn bị kẹp), vùng cấm tâm kẹp
        trên khay lấy đỉnh vật + KEEPOUT_TRAY_MARGIN. Dốc đế mặc định giữ nguyên (đã đủ chống lật >= 1,5 trong vùng cho phép).
        Lift đang thấp hơn lift_min thì chỉ được nâng lên."""
        if khay is not None and not isinstance(khay, KhayTai):
            raise KieuLenhSai(f"set_khay_tai: cần KhayTai hoặc None, nhận kiểu {type(khay).__name__}")
        self.khay = khay

    def lift_min(self) -> float:
        """Giới hạn mềm dưới của lift đang áp (m)."""
        return max(spec.LIFT_RANGE[0], self.khay.lift_min) if self.khay is not None else spec.LIFT_RANGE[0]

    def _khay_dinh_z(self) -> float | None:
        return self.khay.dinh_z if self.khay is not None else None

    def _clamp(self, targets: dict[str, float]) -> dict[str, float]:
        out = limits.clamp(targets, self._targets)
        if "lift" in out and self.khay is not None:
            lo = self.lift_min()
            if out["lift"] < lo:      # lift đang thấp hơn mức sàn (vừa khai báo): không cho hạ thêm, chỉ nâng
                out["lift"] = max(self._targets["lift"], out["lift"]) if self._targets["lift"] < lo else lo
        return out

    def soft_limits(self) -> dict:
        """Giới hạn mềm đã nạp (sim: trong SDK + mô hình; feetech: đọc lại từ EEPROM servo)."""
        return self.backend.read_soft_limits()

    def set_lift(self, height: float) -> None:
        self.set_joint_positions({"lift": height})

    def set_gripper(self, side: str, value: float) -> None:
        """0 = đóng, 100 = mở hết."""
        self._check_side(side, "set_gripper")
        self.set_joint_positions({spec.joint_name(side, "gripper"): value})

    @staticmethod
    def _check_side(side, what: str) -> None:
        if not isinstance(side, str):
            raise KieuLenhSai(f"{what}: side phải là 'left' hoặc 'right', nhận kiểu {type(side).__name__}")
        if side not in spec.SIDES:
            raise LenhKhongHopLe(f"{what}: side phải là 'left' hoặc 'right', nhận {side!r}")

    def set_base_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Đích vận tốc đế (khung robot). |(vx, vy)| <= BASE_MAX_V (co giữ hướng, không kẹp từng trục), ω theo chống lật
        (limits.base_limit_goal); backend đưa vận tốc thật tới đích theo dốc BaseRamp (BASE_MAX_ACC, BASE_MAX_ALPHA) qua step().
        NaN/inf/kiểu sai -> lỗi có mã, đích cũ giữ nguyên."""
        self._guard("set_base_velocity")
        v = [_huu_han(x, f"set_base_velocity: {k}") for k, x in (("vx", vx), ("vy", vy), ("wz", wz))]
        self.backend.set_base_velocity(*limits.base_limit_goal(*v))

    def stop_base(self, timeout: float | None = None) -> None:
        """Dừng đế theo dốc (không phanh gấp: phanh cũng gây lật) và chờ tới khi vận tốc lệnh về 0."""
        if timeout is not None:
            timeout = self._duration(timeout, "stop_base: timeout")
        self.set_base_velocity(0, 0, 0)
        vel = getattr(self.backend, "base_velocity", None)
        if vel is None:
            return
        v = vel()
        t = timeout if timeout is not None else 1.2 * max(math.hypot(v[0], v[1]) / spec.BASE_MAX_ACC,
                                                           abs(v[2]) / spec.BASE_MAX_ALPHA) + 2 * self.dt
        for _ in range(int(math.ceil(t / self.dt))):
            if not np.any(vel()):
                break
            self.step()

    def step(self, dt: float | None = None) -> None:
        """Một chu kỳ điều khiển. dt: giây (> 0, hữu hạn); lớn hơn spec.MAX_STEP_DT_CYCLES chu kỳ thì bị chặn ở mức đó."""
        self._guard("step")
        if dt is None:
            dt = self.dt
        else:
            dt = _huu_han(dt, "step: dt")
            if dt <= 0:
                raise LenhKhongHopLe(f"step: dt phải > 0 giây, nhận {dt}")
            dt = min(dt, spec.MAX_STEP_DT_CYCLES * self.dt)
        try:
            if self.lift_vmax and abs(self._lift_goal - self._targets["lift"]) > 1e-9:
                nxt = limits.lift_slew(self._targets["lift"], self._lift_goal, dt, self.lift_vmax)
                sent = self.backend.write_joints({"lift": nxt})
                self._targets["lift"] = (sent or {"lift": nxt}).get("lift", nxt)
            self.backend.step(dt)
            lt = getattr(self.backend, "latch", None)
            if lt is not None and not getattr(lt, "ok", True):     # R-16: vòng teleop/ghi phải thoát khi đang dừng khẩn
                from .backends.feetech import EStopActive
                raise EStopActive("step: chốt E-stop không ở trạng thái chạy (dừng khẩn): nhả E-stop, latch.reset() rồi enable()")
        except M1Loi:
            self._lift_goal = self._targets["lift"]     # R-23: sau lỗi không đẩy tiếp đích lift mỗi chu kỳ
            raise

    def wait(self, seconds: float) -> None:
        self._guard("wait")
        seconds = self._duration(seconds, "wait")
        for _ in range(max(1, int(round(seconds / self.dt)))):
            self.step()

    # ---------- lệnh cấp cao (chờ xong) ----------
    def move_joints(self, targets: dict[str, float], duration: float = 1.0) -> None:
        """Nội suy khớp tới đích. Đường thẳng cắt vùng cấm ghép vai-khuỷu thì chèn điểm trung gian (limits.plan)."""
        self._guard("move_joints")
        duration = self._duration(duration, "move_joints")
        goal = self._clamp(self._check_targets(targets, "move_joints"))
        self._check_keepout(goal)          # kiểm trước khi đi: không dừng giữa đường vì đích cấm
        legs = limits.plan(self._targets, goal)
        arm = {spec.joint_name(sd, j) for sd in spec.SIDES for j in spec.ARM_JOINTS}
        for leg in legs:
            start = {k: self._targets[k] for k in leg}
            n = max(1, int(round(duration / len(legs) / self.dt)))
            n = self._steps_for(max([abs(leg[k] - start[k]) for k in leg if k in arm] or [0.0]), n)
            for i in range(1, n + 1):
                s = _smooth(i / n)
                self.set_joint_positions({k: start[k] + (leg[k] - start[k]) * s for k in leg})
                self.step()

    def solve_ik(self, side: str, p_robot, pitch: float = kinematics.TOOL_DOWN,
                 grip_yaw: float = 0.0, lift: float | None = None, roll: float | None = None) -> np.ndarray | None:
        """IK tâm kẹp (khung robot). roll: đặt thẳng góc cuộn cổ tay thay cho grip_yaw (tâm kẹp lệch trục cuộn nên vị trí phụ thuộc cuộn).
        hai nghiệm cuộn (trục đóng xoay 180 độ) thì chọn nghiệm gần góc cuộn đích hiện tại, để đường đi liên tục và giữ phía ngón cố định."""
        self._check_side(side, "solve_ik")
        p_robot = _vec3(p_robot, "solve_ik: điểm")
        pitch, grip_yaw = _huu_han(pitch, "solve_ik: pitch"), _huu_han(grip_yaw, "solve_ik: grip_yaw")
        lift = self._targets["lift"] if lift is None else _huu_han(lift, "solve_ik: lift")
        roll = None if roll is None else _huu_han(roll, "solve_ik: roll")
        p = np.asarray(p_robot, float) - kinematics.arm_mount(side, lift)
        return kinematics.arm_ik(p, pitch=pitch, grip_yaw=grip_yaw, side=side, roll=roll,
                                 ref_roll=self._targets[spec.joint_name(side, "wrist_roll")])

    def grasp_point(self, side: str, center_robot, half_width: float, grip_yaw: float = 0.0, margin: float = 0.002,
                    lift: float | None = None) -> np.ndarray | None:
        """(kẹp AM-ARM200, một ngón cố định): TCP (khung robot) để kẹp vật tâm center_robot, nửa bề rộng half_width theo trục đóng
        (dụng cụ chĩa xuống, hướng grip_yaw). Dùng cùng grip_yaw khi gọi move_tcp/set_tcp. None nếu ngoài tầm."""
        self._check_side(side, "grasp_point")
        center_robot = _vec3(center_robot, "grasp_point: tâm vật")
        half_width, grip_yaw = _huu_han(half_width, "grasp_point: half_width"), _huu_han(grip_yaw, "grasp_point: grip_yaw")
        margin = _huu_han(margin, "grasp_point: margin")
        lift = self._targets["lift"] if lift is None else _huu_han(lift, "grasp_point: lift")
        m = kinematics.arm_mount(side, lift)
        t = kinematics.grasp_tcp(np.asarray(center_robot, float) - m, half_width, side=side, grip_yaw=grip_yaw, margin=margin,
                                 ref_roll=self._targets[spec.joint_name(side, "wrist_roll")])
        return None if t is None else t + m

    def set_tcp(self, side: str, p_robot, grip_yaw: float = 0.0) -> bool:
        """Đặt đích điểm kẹp (khung robot, dụng cụ chĩa xuống). Trả False nếu ngoài tầm."""
        self._check_side(side, "set_tcp")
        p_robot = _vec3(p_robot, f"set_tcp: điểm tay {side}")
        grip_yaw = _huu_han(grip_yaw, "set_tcp: grip_yaw")
        q = self.solve_ik(side, p_robot, grip_yaw=grip_yaw)
        if q is None:
            return False
        self.set_joint_positions({spec.joint_name(side, j): v for j, v in zip(spec.ARM_JOINTS, q)})
        return True

    def _arm_dict(self, side: str, q) -> dict[str, float]:
        return {spec.joint_name(side, j): float(v) for j, v in zip(spec.ARM_JOINTS, q)}

    def move_tcp(self, side: str | dict, p_robot=None, duration: float = 1.0, grip_yaw: float = 0.0) -> None:
        """Đi thẳng trong không gian Descartes. side có thể là dict {side: điểm} để chạy hai tay cùng lúc.
        S1C: nếu hướng kẹp ở điểm đầu khác tư thế hiện tại (vd từ tư thế gốc, hoặc đổi grip_yaw) thì nội suy khớp tới đó trước;
        số chu kỳ tự tăng để bước khớp mỗi chu kỳ <= max_joint_step.
        Tầng 1b: đầu vào kiểm trước; R-21: CẢ đường Descartes (mọi điểm nội suy) được kiểm vùng cấm trạm TRƯỚC lệnh đầu tiên."""
        self._guard("move_tcp")
        pairs = side.items() if isinstance(side, dict) else [(side, p_robot)]
        goals = {}
        for s, g in pairs:
            self._check_side(s, "move_tcp")
            goals[s] = _vec3(g, f"move_tcp: điểm tay {s}")
        if not goals:
            raise LenhKhongHopLe("move_tcp: không có tay nào trong lệnh")
        duration = self._duration(duration, "move_tcp")
        grip_yaw = _huu_han(grip_yaw, "move_tcp: grip_yaw")
        starts = {s: self.tcp_position(s, source="target") for s in goals}
        for s, g in goals.items():
            qg = self.solve_ik(s, g, grip_yaw=grip_yaw)
            if qg is None:
                raise LenhKhongHopLe(f"move_tcp: tay {s}: điểm {list(np.round(g, 3))} ngoài tầm với")
            self._check_keepout(self._arm_dict(s, qg), f"tay {s}: điểm {list(np.round(g, 3))}")

        def point(s, i, n):
            return starts[s] + (goals[s] - starts[s]) * _smooth(i / n)

        n0 = n = max(1, int(round(duration / self.dt)))
        pre = {}
        first = {}
        for s in goals:   # tư thế IK ở điểm đầu tiên giải được trên đường (điểm đầu có thể ngoài IK, vd tư thế gốc)
            q0 = None
            for i in range(0, n + 1):
                q0 = self.solve_ik(s, point(s, i, n), grip_yaw=grip_yaw)
                if q0 is not None:
                    break
            first[s] = self.arm_q(s, "target")
            if q0 is not None and np.max(np.abs(q0 - first[s])) > self.max_joint_step:
                pre.update(self._arm_dict(s, q0))
                first[s] = q0
        for _ in range(3):   # kiểm trước đường khớp của nội suy Descartes, kéo dài nếu bước quá lớn (tối đa 10 lần thời gian gốc)
            worst = 0.0
            for s in goals:
                prev = first[s]
                for i in range(1, n + 1):
                    q = self.solve_ik(s, point(s, i, n), grip_yaw=grip_yaw)
                    if q is None:
                        continue
                    worst = max(worst, float(np.max(np.abs(q - prev))))
                    prev = q
            if worst <= self.max_joint_step or self.max_joint_step <= 0:
                break
            n = min(10 * n0, int(math.ceil(n * worst / self.max_joint_step * 1.05)))
        if self.keepout:      # R-21: kiểm vùng cấm trên mọi điểm của đường (và tư thế đầu) trước khi gửi lệnh nào
            if pre:
                self._check_keepout(pre, "move_tcp: tư thế đầu đường")
            for s, g in goals.items():
                for i in range(1, n + 1):
                    q = self.solve_ik(s, point(s, i, n), grip_yaw=grip_yaw)
                    if q is None:
                        continue
                    bad = limits.keepout_check(self._arm_dict(s, q), self._targets, self.config, self._khay_dinh_z())
                    if bad:
                        raise limits.KeepoutError(f"move_tcp: tay {s}: đường thẳng tới {list(np.round(g, 3))} cắt vùng cấm trạm "
                                                  f"({self.config}) tại điểm {list(np.round(point(s, i, n), 3))}: " + "; ".join(bad))
        if pre:
            self.move_joints(pre, duration=0.0)
        for i in range(1, n + 1):
            for s in goals:
                self.set_tcp(s, point(s, i, n), grip_yaw=grip_yaw)   # điểm giữa ngoài IK: giữ đích cũ (S1C: trước đây nhảy thẳng tới đích cuối)
            self.step()
        for s, g in goals.items():
            self.set_tcp(s, g, grip_yaw=grip_yaw)

    def move_base(self, dx: float = 0.0, dy: float = 0.0, dyaw: float = 0.0, duration: float = 2.0) -> None:
        """Dịch đế (khung robot lúc bắt đầu) theo hồ sơ vận tốc trơn. Thật: vòng hở theo odometry bánh.
        duration tự kéo dài (limits.base_move_time) để đỉnh vận tốc, gia tốc không chạm trần chống lật, đế đi đủ quãng.
        Kết thúc bằng stop_base(): dốc về 0 hẳn (không để bánh quay chậm còn dư khi hàm trả về)."""
        self._guard("move_base")
        dx, dy, dyaw = (_huu_han(v, f"move_base: {k}") for k, v in (("dx", dx), ("dy", dy), ("dyaw", dyaw)))
        duration = self._duration(duration, "move_base")
        duration = max(duration, limits.base_move_time(dx, dy, dyaw))
        if duration > spec.MAX_MOVE_DURATION_S:
            raise LenhKhongHopLe(f"move_base: quãng ({dx:g}, {dy:g}, {dyaw:g}) cần {duration:.0f} s > trần {spec.MAX_MOVE_DURATION_S:g} s; "
                                 "chia thành nhiều đoạn")
        n = max(1, int(round(duration / self.dt)))
        for i in range(n):
            ds = _smooth((i + 1) / n) - _smooth(i / n)
            self.set_base_velocity(dx * ds / self.dt, dy * ds / self.dt, dyaw * ds / self.dt)
            self.step()
        self.stop_base()

    def home(self, duration: float = 1.5) -> None:
        t = {spec.joint_name(s, j): v for s in spec.SIDES for j, v in spec.HOME_ARM.items()}
        self.move_joints(t, duration)
