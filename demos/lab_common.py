"""Phần dùng chung cho các demo phòng lab (sim/lab/lab.xml): nạp robot, đi tới trạm, kẹp, bám theo quỹ đạo bằng đế, kiểm hoàn thành, chụp ảnh.

Chỉ dùng API `Robot` của SDK (IK, kẹp, đế); không đặt trực tiếp vị trí vật. Ngoại lệ duy nhất: `scatter()` xáo vị trí ban đầu của vật
theo seed TRƯỚC khi robot chạy (bố trí cảnh, như người dọn bàn), và các hàm kiểm/ảnh đọc ground truth của sim.

Ghi chú kẹp: kẹp gốc AM-ARM200, một ngón cố định + hàm động quay về phía nó, đóng theo trục y khớp cuộn. Vật kẹp tựa ngón cố định nên
TCP đặt lệch khỏi tâm vật về phía ngón cố định (Robot.grasp_point, nửa bề rộng vật theo trục đóng). Trước đây (S1C): kẹp song song cũ.
"""
from __future__ import annotations

import math
import os
import struct
import sys
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))
sys.path.insert(0, str(ROOT / "sim" / "lab"))

from faibotic import Robot, spec  # noqa: E402
import build_lab as L  # noqa: E402

LAB_XML = ROOT / "sim" / "lab" / "lab.xml"
SNAP_DIR = ROOT / "media" / "sim-snapshots" / "lab"
SIDE = "left"                     # tay làm việc; vai trái ở y +0,17 trong khung robot
ARM_Y = spec.SHOULDER_Y          # S1G: 0,18726 (CAD P-L)
WORK_X = L.TABLE_EDGE_X - L.ROBOT_STANDOFF   # x tâm đế khi làm việc ở bàn
CLEAR_Z = L.TABLE_H + 0.08        # 24/11: 0,10 -> 0,08 vì trần gập 70 hạ đỉnh tầm kẹp chĩa xuống (x 0,39 lift 0,45: z <= 0,845). Độ cao TCP khi di chuyển giữa các điểm (S1C P-L: 0,12 -> 0,10, đầu ngón còn cách mặt bàn 9 cm)
SKIP_RENDER = os.environ.get("M1_SKIP_RENDER") == "1"


def make_robot(viewer: bool = False) -> Robot:
    return Robot(backend="sim", viewer=viewer, xml_path=LAB_XML)


def scatter(robot: Robot, seed: int | None) -> None:
    """Xáo nhẹ vị trí/hướng vật trên bàn và lon trong tủ lạnh theo seed (±1 cm, ±0,2 rad) lúc bố trí cảnh, trước khi robot làm gì."""
    if seed is None:
        return
    import mujoco
    b = robot.backend
    sc = L.objects(seed, can_jitter=True)
    for it in sc:
        ln = it["links"][0]
        a = b.model.joint(it["id"]).qposadr[0]
        b.data.qpos[a:a + 3] = ln["pos"]
        y = ln.get("yaw", 0.0)
        b.data.qpos[a + 3:a + 7] = [math.cos(y / 2), 0, 0, math.sin(y / 2)]
    mujoco.mj_forward(b.model, b.data)


# ---------------- trạng thái sim (ground truth, chỉ để kiểm) ----------------

def obj_pos(robot: Robot, oid: str) -> np.ndarray:
    return robot.backend.body_pos(oid)


def obj_speed(robot: Robot, oid: str) -> float:
    b = robot.backend
    a = b.model.joint(oid).dofadr[0]
    return float(np.linalg.norm(b.data.qvel[a:a + 6]))


def joint_q(robot: Robot, name: str) -> float:
    b = robot.backend
    return float(b.data.qpos[b.model.joint(name).qposadr[0]])


def obj_yaw(robot: Robot, oid: str) -> float:
    b = robot.backend
    a = b.model.joint(oid).qposadr[0]
    w, x, y, z = b.data.qpos[a + 3:a + 7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def obj_half_w(oid: str) -> float:
    """Nửa bề rộng vật theo trục x riêng của vật (trục đóng ngón khi kẹp với grip_yaw = hướng vật)."""
    for i, s, size, _ in L.OBJECTS + [L.FRIDGE_OBJ]:
        if i == oid:
            return size[0] / 2 if s == "box" else size[0]
    raise KeyError(oid)


def obj_half_h(oid: str) -> float:
    for i, s, size, _ in L.OBJECTS + [L.FRIDGE_OBJ]:
        if i == oid:
            return (size[2] if s == "box" else size[1]) / 2
    raise KeyError(oid)


def in_zone(robot: Robot, oid: str, center_xy, half_xy, settle_speed: float = 0.02) -> dict:
    """Vật nằm trong vùng chữ nhật (hoặc tròn nếu half_xy là số), đáy trên mặt bàn/đáy khay (±1,5 cm), đứng yên."""
    p = obj_pos(robot, oid)
    d = p[:2] - np.asarray(center_xy)
    inside = (np.linalg.norm(d) <= half_xy) if np.isscalar(half_xy) else bool(np.all(np.abs(d) <= np.asarray(half_xy)))
    z_ok = abs(p[2] - (L.TABLE_H + obj_half_h(oid))) <= 0.015
    v = obj_speed(robot, oid)
    return {"id": oid, "pos": [round(float(x), 4) for x in p], "inside": bool(inside), "z_ok": bool(z_ok),
            "speed": round(v, 4), "ok": bool(inside and z_ok and v <= settle_speed)}


# ---------------- di chuyển ----------------

def goto_base(robot: Robot, x: float, y: float, speed: float = 0.25) -> None:
    """Đi tới (x, y) thế giới, hướng giữ 0. Đế omni chạy vòng hở theo vận tốc (như odometry), tịnh tiến thẳng."""
    bx, by, _ = robot.backend.base_pose()
    dist = math.hypot(x - bx, y - by)
    if dist < 1e-3:
        return
    robot.move_base(dx=x - bx, dy=y - by, duration=max(0.8, 1.6 * dist / speed))


def raise_to_work(robot: Robot, lift: float = L.LIFT_WORK) -> None:
    robot.move_joints({"lift": lift}, duration=1.5)


def tcp_world_to_robot(robot: Robot, p) -> np.ndarray:
    return robot.world_to_robot(p)


def move_tcp_world(robot: Robot, p, duration: float = 1.0, grip_yaw: float = 0.0, side: str = SIDE) -> None:
    robot.move_tcp(side, robot.world_to_robot(p), duration=duration, grip_yaw=grip_yaw)


def safe_carry(robot: Robot, side: str = SIDE) -> None:
    """Nhấc TCP lên độ cao an toàn tại chỗ."""
    p = robot.tcp_position(side, source="target")
    robot.move_tcp(side, [p[0], p[1], CLEAR_Z], duration=0.8)


def gripper(robot: Robot, value: float, duration: float = 0.6, side: str = SIDE) -> None:
    robot.move_joints({f"{side}_gripper": value}, duration=duration)


def follow_with_base(robot: Robot, path, duration: float, side: str = SIDE) -> None:
    """Giữ TCP ở điểm hiện tại trong khung robot (trong tầm với) và dời đế để TCP đi theo path(s) -> (điểm thế giới xyz, grip_yaw), s 0..1.
    Dùng khi quỹ đạo dài hơn tầm với của tay (cửa tủ lạnh, kéo ngăn kéo, nắp hộp). Đế hướng 0 nên khung robot chỉ tịnh tiến.
    duration tự kéo dài để vận tốc, gia tốc đế của hồ sơ không vượt BASE_MAX_V, BASE_MAX_ACC (dốc SDK không phải cắt, TCP bám path)."""
    from faibotic import spec
    m = 200
    xy = np.array([np.asarray(path(0.5 - 0.5 * math.cos(math.pi * i / m))[0][:2], float) for i in range(m + 1)])
    v = np.diff(xy, axis=0) * m                  # theo thời gian chuẩn hoá (T = 1)
    a = np.diff(v, axis=0) * m
    vmax, amax = float(np.max(np.linalg.norm(v, axis=1))), float(np.max(np.linalg.norm(a, axis=1), initial=0.0))
    duration = max(duration, 1.1 * vmax / spec.BASE_MAX_V, 1.1 * math.sqrt(amax / spec.BASE_MAX_ACC))
    n = max(2, int(round(duration / robot.dt)))
    pr = robot.world_to_robot(path(0.0)[0])
    prev = np.asarray(path(0.0)[0][:2], float) - pr[:2]
    for i in range(1, n + 1):
        s = 0.5 - 0.5 * math.cos(math.pi * i / n)
        w, yaw = path(s)
        g = np.asarray(w[:2], float) - pr[:2]
        v = (g - prev) / robot.dt
        prev = g
        robot.set_base_velocity(v[0], v[1], 0.0)
        robot.set_tcp(side, [pr[0], pr[1], w[2]], grip_yaw=yaw)
        robot.step()
    robot.set_base_velocity(0, 0, 0)


def move_tcp_level(robot: Robot, side: str, p_robot, duration: float = 1.0, roll: float = 0.0) -> None:
    """Đi thẳng Descartes với dụng cụ nằm ngang (pitch 0, hướng ra theo bán kính tay), cuộn cổ tay = roll (0: hai ngón đóng theo phương ngang).
    Dùng solve_ik(pitch=0, roll=roll) của SDK (tâm kẹp lệch trục cuộn nên IK phải biết cuộn); dùng khi phải với vào trong ngăn có nóc (tủ lạnh)."""
    from faibotic import spec
    start = robot.tcp_position(side, source="target")
    goal = np.asarray(p_robot, float)

    def q_at(p):
        return robot.solve_ik(side, p, pitch=0.0, roll=roll)
    if q_at(goal) is None:
        raise ValueError(f"tay {side}: điểm {list(np.round(goal, 3))} ngoài tầm với (dụng cụ nằm ngang)")
    n = max(1, int(round(duration / robot.dt)))
    for i in range(1, n + 1):
        a = 0.5 - 0.5 * math.cos(math.pi * i / n)
        q = q_at(start + (goal - start) * a)
        if q is not None:
            robot.set_joint_positions({spec.joint_name(side, j): v for j, v in zip(spec.ARM_JOINTS, q)})
        robot.step()


def move_level_joint(robot: Robot, side: str, p_robot, duration: float = 1.5, roll: float = 0.0) -> None:
    """Đưa tay (nội suy khớp) tới tư thế dụng cụ nằm ngang tại p_robot: dùng để vào tư thế ban đầu từ tư thế gốc."""
    from faibotic import spec
    q = robot.solve_ik(side, p_robot, pitch=0.0, roll=roll)
    if q is None:
        raise ValueError(f"tay {side}: điểm {list(np.round(p_robot, 3))} ngoài tầm với (dụng cụ nằm ngang)")
    robot.move_joints({spec.joint_name(side, j): v for j, v in zip(spec.ARM_JOINTS, q)}, duration=duration)


def site_world(robot: Robot, name: str) -> np.ndarray:
    """Vị trí tay nắm/núm (site trong lab.xml). Thực tế lấy từ thị giác hoặc bản đồ cảnh; ở đây đọc từ cảnh."""
    return robot.backend.site_pos(name)


# ---------------- gắp đặt một vật ----------------

def grasp_z(oid: str) -> float:
    """TCP ngang tâm vật theo chiều cao (đầu ngón trước TCP spec.TIP_LEN), đầu ngón cách mặt bàn ít nhất 6 mm."""
    from faibotic import spec
    return L.TABLE_H + max(obj_half_h(oid), spec.TIP_LEN + 0.006)


def approach_y(robot: Robot, y_world: float, side: str = SIDE) -> None:
    """Đi ngang để vai tay làm việc thẳng hàng với y cần kẹp."""
    goto_base(robot, WORK_X, y_world - (ARM_Y if side == "left" else -ARM_Y))


def grasp_world(robot: Robot, center, half_w: float, grip_yaw: float = 0.0, side: str = SIDE) -> np.ndarray:
    """TCP (thế giới) để kẹp vật tâm `center` (thế giới) nửa bề rộng half_w: lệch về phía ngón cố định (Robot.grasp_point).
    Ngoài tầm thì trả tâm vật (IK báo lỗi ở bước sau)."""
    c = robot.world_to_robot(center)
    t = robot.grasp_point(side, c, half_w, grip_yaw=grip_yaw)
    return np.asarray(center, float) if t is None else np.asarray(center, float) + (t - c)


def pick(robot: Robot, oid: str, log=print) -> None:
    p = obj_pos(robot, oid)
    yaw = obj_yaw(robot, oid)
    log(f"  gắp {oid} tại {np.round(p, 3).tolist()}")
    approach_y(robot, p[1])
    z = grasp_z(oid)
    gy = yaw  # trục đóng ngón theo trục x riêng của vật
    move_tcp_world(robot, [p[0], p[1], CLEAR_Z], 1.2, gy)          # tới trên tâm vật trước: chốt nghiệm cuộn (phía ngón cố định)
    g = grasp_world(robot, [p[0], p[1], z], obj_half_w(oid), gy)
    move_tcp_world(robot, [g[0], g[1], CLEAR_Z], 0.4, gy)
    gripper(robot, 100, 0.5)
    move_tcp_world(robot, g, 1.0, gy)
    robot.wait(0.15)
    gripper(robot, 0, 0.7)
    robot.wait(0.3)
    move_tcp_world(robot, [g[0], g[1], CLEAR_Z], 1.0, gy)


RELEASE_S = 0.9        # thời gian mở kẹp khi nhả vật (trước: 0,5 s)


def place(robot: Robot, oid: str, xy, drop_h: float = 0.004, open_to: float = 100, log=print) -> None:
    """open_to: độ mở khi thả (0-100); mở vừa đủ khi thả trong ngăn hẹp để ngón không chạm vách."""
    log(f"  đặt {oid} vào {np.round(xy, 3).tolist()}")
    approach_y(robot, xy[1])
    move_tcp_world(robot, [xy[0], xy[1], CLEAR_Z], 1.2)
    g = grasp_world(robot, [xy[0], xy[1], grasp_z(oid) + drop_h], obj_half_w(oid))
    move_tcp_world(robot, [g[0], g[1], CLEAR_Z], 0.4)
    move_tcp_world(robot, g, 1.0)
    robot.wait(0.15)
    gripper(robot, open_to, RELEASE_S)          # hàm quay ra xa vật; mở chậm cho vật đứng yên
    robot.wait(0.2)
    move_tcp_world(robot, [g[0], g[1], CLEAR_Z], 0.8)
    gripper(robot, 0, 0.3)


def park(robot: Robot) -> None:
    """Tay về tư thế gốc, đế lùi khỏi bàn."""
    robot.home(duration=1.2)
    bx, by, _ = robot.backend.base_pose()
    goto_base(robot, WORK_X - 0.15, by)
    robot.wait(0.8)


# ---------------- ảnh ----------------

def write_png(path: Path, img: np.ndarray) -> None:
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))
    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) + \
        chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


class Snapper:
    """Chụp khung theo tên vào media/sim-snapshots/lab/<prefix>-<tên>.png (tắt khi M1_SKIP_RENDER=1 hoặc snap=False)."""

    def __init__(self, robot: Robot, prefix: str, camera: str, enabled: bool = True, w: int = 960, h: int = 600):
        self.robot, self.prefix, self.camera, self.w, self.h = robot, prefix, camera, w, h
        self.enabled = enabled and not SKIP_RENDER
        self.saved: list[Path] = []
        m = robot.backend.model
        for i in range(m.nsite):   # ẩn chấm TCP và điểm neo lò xo khi chụp (chỉ đổi màu hiển thị, không đổi vật lý)
            m.site_rgba[i, 3] = 0.0

    def __call__(self, name: str, camera: str | None = None) -> None:
        if not self.enabled:
            return
        img = self.robot.backend.camera(camera or self.camera, self.w, self.h)
        p = SNAP_DIR / f"{self.prefix}-{name}.png"
        write_png(p, img)
        self.saved.append(p)


def headless_env(headless: bool) -> None:
    if headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
