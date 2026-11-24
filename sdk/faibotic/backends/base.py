"""Giao diện backend chung. Mọi backend (sim, feetech) cài đủ các hàm dưới đây."""
from __future__ import annotations

import abc

import numpy as np


class Backend(abc.ABC):
    name = "base"
    control_dt = 0.02  # 50 Hz

    @abc.abstractmethod
    def read_joints(self) -> dict[str, float]:
        """Vị trí khớp hiện tại: tay (rad), kẹp (0–100, 0 = đóng), lift (m)."""

    @abc.abstractmethod
    def write_joints(self, targets: dict[str, float]) -> dict[str, float]:
        """Đặt vị trí đích cho các khớp có trong dict. Backend tự kẹp lại theo giới hạn mềm (limits.clamp)
        và chặn đoạn cắt vùng cấm ghép (limits.safe_step); trả về đích thực sự đã ghi."""

    def commanded(self) -> dict[str, float] | None:
        """Đích đã ra lệnh gần nhất (None nếu backend không lưu)."""
        return None

    def read_soft_limits(self) -> dict:
        """Giới hạn mềm đã nạp, đọc lại (cho bước "nạp và đọc lại giới hạn mềm" ở 06-lap-rap)."""
        from .. import limits
        return limits.loaded()

    @abc.abstractmethod
    def set_base_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Vận tốc đế trong khung robot (m/s, m/s, rad/s)."""

    @abc.abstractmethod
    def base_pose(self) -> np.ndarray:
        """(x, y, yaw) của đế trong khung thế giới/odometry."""

    @abc.abstractmethod
    def step(self, dt: float) -> None:
        """Cho thời gian trôi dt giây (sim: tích phân vật lý; thật: chờ + cập nhật odometry)."""

    def camera(self, name: str) -> np.ndarray:
        raise NotImplementedError(f"backend {self.name} chưa có camera {name}")

    def close(self) -> None:
        pass
