"""Bộ điều khiển teleop bàn phím cho M1 (không phụ thuộc cửa sổ, test được không cần màn hình).

Mã phím theo GLFW. Phím giữ (đế, trục nâng, di chuyển kẹp) đọc từ tập phím đang giữ;
phím bấm một lần (chọn tay, kẹp, về gốc, reset, demo, camera) đọc từ hàng sự kiện.
"""
from __future__ import annotations

import math

import numpy as np

from . import kinematics, spec

# Mã phím GLFW
K = {c: ord(c) for c in "ABCDEFHIJKLOPQRSUWX"}
K.update({"1": 49, "2": 50, "SPACE": 32, "BACKSPACE": 259, "ESC": 256, "TAB": 258,
          "RIGHT": 262, "LEFT": 263, "DOWN": 264, "UP": 265, "PGUP": 266, "PGDN": 267})

HELP = [
    ("W / S", "đế tiến / lùi"),
    ("A / D", "đế sang trái / phải"),
    ("Q / E", "đế xoay trái / phải"),
    ("R / F", "trục nâng lên / xuống"),
    ("1 / 2", "chọn tay trái / phải"),
    ("mũi tên  hoặc  I K J L", "kẹp tới / lui / trái / phải"),
    ("PgUp PgDn  hoặc  U O", "kẹp lên / xuống"),
    ("Space", "đóng / mở kẹp"),
    ("H", "về tư thế gốc"),
    ("Backspace", "reset cảnh"),
    ("P", "chạy demo gắp đặt tự động"),
    ("C", "đổi camera"),
    ("Tab", "ẩn / hiện bảng phím"),
    ("Esc", "thoát"),
]

BASE_V = 0.25    # m/s
BASE_W = 0.8     # rad/s
LIFT_V = spec.LIFT_SPEED_MAX    # m/s theo spec (QC2 cột nâng: 0,12 cũ gấp 8 lần tốc độ thiết kế): dùng cho phần cứng và sim realistic_lift
LIFT_V_SIM_FAST = 0.12          # sim mặc định (demos/selftest.py cần nâng > 0,08 m trong 5 s): KHÔNG phản ánh phần cứng
TCP_V = 0.12     # m/s


class Teleop:
    def __init__(self, robot):
        self.robot = robot
        self.side = "left"
        self.reset_state()

    def reset_state(self) -> None:
        self.grip_open = {s: False for s in spec.SIDES}
        # đích TCP trong khung gốc tay (đi theo trục nâng)
        self.tcp = {s: kinematics.arm_fk(self.robot.arm_q(s, "target"), s)[0] for s in spec.SIDES}
        self.ik_fail = 0
        self.lift = self.robot._targets["lift"]

    # ---------- phím bấm một lần ----------
    def press(self, key: int) -> str | None:
        """Xử lý phím bấm; trả về lệnh cho vòng ngoài ('reset', 'demo', 'camera', 'help', 'quit') nếu có."""
        if key == K["1"]:
            self.side = "left"
        elif key == K["2"]:
            self.side = "right"
        elif key == K["SPACE"]:
            self.grip_open[self.side] = not self.grip_open[self.side]
            self.robot.set_gripper(self.side, 100 if self.grip_open[self.side] else 0)
        elif key == K["H"]:
            t = {spec.joint_name(s, j): v for s in spec.SIDES for j, v in spec.HOME_ARM.items()}
            self.robot.set_joint_positions(t)
            self.tcp = {s: kinematics.arm_fk([spec.HOME_ARM[j] for j in spec.ARM_JOINTS], s)[0]
                        for s in spec.SIDES}
        elif key == K["BACKSPACE"]:
            return "reset"
        elif key == K["P"]:
            return "demo"
        elif key == K["C"]:
            return "camera"
        elif key == K["TAB"]:
            return "help"
        elif key == K["ESC"]:
            return "quit"
        return None

    # ---------- phím giữ ----------
    def update(self, down: set[int], dt: float) -> None:
        def ax(pos, neg):
            return float(any(K[k] in down for k in pos)) - float(any(K[k] in down for k in neg))

        self.robot.set_base_velocity(BASE_V * ax("W", "S"), BASE_V * ax("A", "D"), BASE_W * ax("Q", "E"))
        fast = self.robot.backend.name == "sim" and not getattr(self.robot.backend, "realistic_lift", False)
        dl = (LIFT_V_SIM_FAST if fast else LIFT_V) * ax("R", "F") * dt
        if dl:
            self.lift = float(np.clip(self.lift + dl, *spec.LIFT_RANGE))
            self.robot.set_lift(self.lift)
        v = np.array([ax(("UP", "I"), ("DOWN", "K")), ax(("LEFT", "J"), ("RIGHT", "L")),
                      ax(("PGUP", "U"), ("PGDN", "O"))]) * TCP_V * dt
        if np.any(v):
            new = self.tcp[self.side] + v
            q = kinematics.arm_ik(new, side=self.side)
            if q is None:
                self.ik_fail += 1
            else:
                self.ik_fail = 0
                self.tcp[self.side] = new
                self.robot.set_joint_positions(
                    {spec.joint_name(self.side, j): x for j, x in zip(spec.ARM_JOINTS, q)})

    def status(self) -> str:
        st = self.robot.get_state()
        x, y, yaw = st["base_pose"]
        p = self.robot.tcp_position(self.side)
        return (f"tay: {'TRÁI' if self.side == 'left' else 'PHẢI'}  kẹp: {'mở' if self.grip_open[self.side] else 'đóng'}\n"
                f"đế: x={x:+.2f} y={y:+.2f} yaw={np.degrees(yaw):+.0f}°\n"
                f"trục nâng: {st['lift']:.2f} m\n"
                f"kẹp (khung robot): {p[0]:+.2f} {p[1]:+.2f} {p[2]:+.2f}"
                + ("\nngoài tầm với!" if self.ik_fail else ""))


# ---------------- Teleop bằng tay leader (SO-101 / AM-ARM200 leader), 1:1 ----------------
# Tay leader đọc qua LeRobot (độ, kẹp 0..100). Khớp 1:1: pan, vai, khuỷu, gập, yaw, cuộn, kẹp (follower 6+1 như leader AM-ARM200;
# leader SO-101 không có wrist_yaw thì yaw giữ đích cũ).
# Góc leader coi như góc URDF upstream AM-ARM200 (leader hiệu chuẩn cùng tư thế 0 với follower); đổi sang quy ước SDK bằng spec.URDF_TO_SDK,
# cộng bù hiệu chuẩn riêng từng bộ (độ, đo ở bước lắp ráp). SO-101 leader: tư thế 0 khác, phải đo bù (offset_deg) trước khi dùng [chưa đo].
LEADER_JOINTS = {"shoulder_pan": "pan", "shoulder_lift": "shoulder", "elbow_flex": "elbow", "wrist_flex": "wrist_flex",
                 "wrist_yaw": "wrist_yaw", "wrist_roll": "wrist_roll", "gripper": "gripper"}
LEADER_IGNORED: tuple[str, ...] = ()   # trước đây bỏ wrist_yaw (5+1)


class LeaderMap:
    """Đổi trạng thái tay leader (dict tên LeRobot -> độ; kẹp 0..100) thành đích khớp M1 (rad; kẹp 0..100) cho một bên.

    offset_deg: bù hiệu chuẩn theo khớp SDK (độ, cộng sau khi đổi dấu); sign: đổi dấu thêm theo khớp (lắp ngược). Kết quả kẹp về
    ARM_LIMITS và giới hạn ghép (spec.elbow_max, spec.wrist_max); bước mỗi chu kỳ do limits.safe_step của backend chặn tiếp."""

    def __init__(self, side: str = "left", offset_deg: dict | None = None, sign: dict | None = None):
        self.side = side
        self.offset = {j: math.radians(v) for j, v in (offset_deg or {}).items()}
        self.sign = dict(sign or {})

    def __call__(self, leader: dict) -> dict:
        out = {}
        for ln, j in LEADER_JOINTS.items():
            if ln not in leader:
                continue
            if j == "gripper":
                out[spec.joint_name(self.side, "gripper")] = float(np.clip(leader[ln], 0.0, 100.0))
                continue
            sg, off = spec.URDF_TO_SDK[j]
            q = sg * self.sign.get(j, 1.0) * math.radians(leader[ln]) + off + self.offset.get(j, 0.0)
            q = math.atan2(math.sin(q), math.cos(q))
            lo, hi = spec.ARM_LIMITS[j]
            out[spec.joint_name(self.side, j)] = float(np.clip(q, lo, hi))
        sh, el, wf = (spec.joint_name(self.side, j) for j in ("shoulder", "elbow", "wrist_flex"))
        if el in out and sh in out:
            out[el] = min(out[el], spec.elbow_max(out[sh]))
            if wf in out:
                out[wf] = min(out[wf], spec.wrist_max(out[sh], out[el]))
        return out
