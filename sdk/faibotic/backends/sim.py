"""Backend mô phỏng MuJoCo cho M1."""
from __future__ import annotations

import math
import time
from pathlib import Path

import mujoco
import numpy as np

from .. import kinematics, limits, model as m1model, spec
from .base import Backend

ARM_JOINT_NAMES = [spec.joint_name(s, j) for s in spec.SIDES for j in spec.ARM_JOINTS]


class SimBackend(Backend):
    name = "sim"

    def __init__(self, viewer: bool = False, key_callback=None, realtime: bool | None = None,
                 scene: bool = True, xml_path: str | Path | None = None, realistic_lift: bool = False):
        """realistic_lift: tốc độ lift <= spec.LIFT_SPEED_MAX và lực <= spec.LIFT_FORCE_LIMIT_N (lực răng ở TORQUE_LIMIT lift, v0.5 thanh răng ~60 N; v0.4 vitme 97 N);
        mặc định tắt vì demo/test chạy nhanh (trục nâng sim nhanh gấp ~14 lần phần cứng: không dùng làm thời gian chu kỳ)."""
        self.realistic_lift = realistic_lift
        # file (vd sim/m1_scene.xml) nạp theo đường dẫn để mesh trong sim/assets/ tìm theo thư mục của file
        self.model = (mujoco.MjModel.from_xml_path(str(xml_path)) if xml_path
                      else mujoco.MjModel.from_xml_string(m1model.robot_mjcf(scene)))
        self.data = mujoco.MjData(self.model)
        self.realtime = viewer if realtime is None else realtime
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}
        self._act = {mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
                     for i in range(self.model.nu)}
        self._qadr = {}
        for i in range(self.model.njnt):
            n = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self._qadr[n] = (self.model.jnt_qposadr[i], self.model.jnt_dofadr[i])
        self.viewer = None
        self.lift_switches = limits.PositionLiftSwitches(lambda: self._q("lift"))
        if realistic_lift:
            a = self._act["lift"]
            self.model.actuator_forcerange[a] = (-spec.LIFT_FORCE_LIMIT_N, spec.LIFT_FORCE_LIMIT_N)
            self.model.actuator_forcelimited[a] = 1
        self.reset()
        if viewer:
            import mujoco.viewer as mjv
            self.viewer = mjv.launch_passive(self.model, self.data, key_callback=key_callback)
            self._wall_t0 = time.perf_counter()
            self._sim_t0 = self.data.time

    # ---------- trạng thái ----------
    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        for s in spec.SIDES:
            for j, v in spec.HOME_ARM.items():
                self.data.qpos[self._qadr[spec.joint_name(s, j)][0]] = v
        self._targets = {n: 0.0 for n in spec.all_joint_names()}
        for s in spec.SIDES:
            for j, v in spec.HOME_ARM.items():
                self._targets[spec.joint_name(s, j)] = v
        self._base_target = np.zeros(3)
        self._base_vel = np.zeros(3)          # vận tốc đã qua dốc, là thứ bánh thật nhận
        self.base_ramp = limits.BaseRamp()
        self._lift_cmd = 0.0
        self._apply_ctrl()
        mujoco.mj_forward(self.model, self.data)
        if getattr(self, "viewer", None) is not None:
            self._wall_t0 = time.perf_counter()
            self._sim_t0 = self.data.time

    def _q(self, name: str) -> float:
        return float(self.data.qpos[self._qadr[name][0]])

    def read_joints(self) -> dict[str, float]:
        out = {n: self._q(n) for n in ARM_JOINT_NAMES}
        for s in spec.SIDES:
            out[spec.joint_name(s, "gripper")] = 100.0 * self._q(f"{s}_gripper") / spec.GRIPPER_OPEN_RAD   # góc mở hàm từ chỗ khép
        out["lift"] = self._q("lift")
        return out

    def write_joints(self, targets: dict[str, float]) -> dict[str, float]:
        for k in targets:
            if k not in self._targets:
                raise KeyError(f"khớp không có: {k}")
        targets = dict(targets)
        if "lift" in targets:
            targets["lift"] = limits.lift_gate(self._q("lift"), targets["lift"], self.lift_switches)
        sent = limits.safe_step(self._targets, limits.clamp(targets, self._targets))
        for k, v in sent.items():
            self._targets[k] = float(v)
        if not self.realistic_lift:
            self._lift_cmd = self._targets["lift"]
        self._apply_ctrl()
        return sent

    def commanded(self) -> dict[str, float]:
        """Đích đã ra lệnh gần nhất (gốc mà write_joints dùng để kẹp và chặn đoạn)."""
        return dict(self._targets)

    def read_soft_limits(self) -> dict:
        """Giới hạn trong SDK + dải khớp của mô hình MuJoCo (độ) đọc lại từ model."""
        out = limits.loaded()
        out["model_deg"] = {n: tuple(round(math.degrees(float(v)), 3) for v in self.model.joint(n).range)
                            for n in ARM_JOINT_NAMES}
        return out

    def set_base_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Đích vận tốc; kẹp |v| và ω theo chống lật. Vận tốc thực đi theo dốc BaseRamp ở step() (như backend feetech)."""
        self.base_ramp.set_goal(vx, vy, wz)

    def base_velocity(self) -> np.ndarray:
        """Vận tốc đế đang lệnh (sau dốc), khung robot."""
        return self._base_vel.copy()

    def base_pose(self) -> np.ndarray:
        return np.array([self._q("base_x"), self._q("base_y"), self._q("base_yaw")])

    def _apply_ctrl(self) -> None:
        c = self.data.ctrl
        for n in ARM_JOINT_NAMES:
            c[self._act[n]] = self._targets[n]
        for s in spec.SIDES:
            g = self._targets[spec.joint_name(s, "gripper")]
            # kẹp AM-ARM200, actuator trên khớp hàm (rad mở từ chỗ khép, 0..GRIPPER_OPEN_RAD)
            c[self._act[f"{s}_gripper"]] = spec.GRIPPER_OPEN_RAD * min(max(g, 0.0), 100.0) / 100.0
        c[self._act["lift"]] = self._lift_cmd
        c[self._act["base_x"]], c[self._act["base_y"]], c[self._act["base_yaw"]] = self._base_target
        w = kinematics.wheel_speeds(*self._base_vel)
        for i in range(3):
            c[self._act[f"wheel_{i}"]] = w[i]

    def step(self, dt: float) -> None:
        if isinstance(dt, bool) or not isinstance(dt, (int, float, np.floating)) or not math.isfinite(dt) or dt <= 0:
            from ..loi import LenhKhongHopLe
            raise LenhKhongHopLe(f"step: dt phải là số giây hữu hạn > 0, nhận {dt!r}")   # QC2 m4: lớp backend cũng kiểm
        n = max(1, int(round(dt / self.model.opt.timestep)))
        h = n * self.model.opt.timestep
        # dốc vận tốc rồi tích phân đích tư thế đế từ vận tốc trong khung robot
        self._base_vel = self.base_ramp.step(h)
        yaw = self._base_target[2]
        vx, vy, wz = self._base_vel
        self._base_target += np.array([math.cos(yaw) * vx - math.sin(yaw) * vy,
                                       math.sin(yaw) * vx + math.cos(yaw) * vy, wz]) * h
        if self.realistic_lift:
            self._lift_cmd = limits.lift_slew(self._lift_cmd, self._targets["lift"], h)
        else:
            self._lift_cmd = self._targets["lift"]
        self._apply_ctrl()
        if self.viewer is not None:
            with self.viewer.lock():
                mujoco.mj_step(self.model, self.data, nstep=n)
            self.viewer.sync()
            if self.realtime:
                lag = (self.data.time - self._sim_t0) - (time.perf_counter() - self._wall_t0)
                if lag > 0:
                    time.sleep(lag)
                elif lag < -0.5:  # chậm quá thì bỏ qua, không cố đuổi
                    self._wall_t0 = time.perf_counter() - (self.data.time - self._sim_t0)
        else:
            mujoco.mj_step(self.model, self.data, nstep=n)

    # ---------- camera ----------
    def camera(self, name: str, width: int = 640, height: int = 480) -> np.ndarray:
        key = (width, height)
        if key not in self._renderers:
            self._renderers[key] = mujoco.Renderer(self.model, height, width)
        r = self._renderers[key]
        r.update_scene(self.data, camera=name)
        return r.render()

    # ---------- chỉ sim: ground truth ----------
    def body_pos(self, name: str) -> np.ndarray:
        return self.data.body(name).xpos.copy()

    def site_pos(self, name: str) -> np.ndarray:
        return self.data.site(name).xpos.copy()

    def contacts(self) -> list[tuple[str, str]]:
        out = []
        for c in self.data.contact[: self.data.ncon]:
            g1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or f"#{c.geom1}"
            g2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or f"#{c.geom2}"
            out.append((g1, g2))
        return out

    def is_running(self) -> bool:
        return self.viewer is None or self.viewer.is_running()

    def close(self) -> None:
        for r in self._renderers.values():
            r.close()
        self._renderers.clear()
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
