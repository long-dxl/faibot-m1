"""Demo M1: đi tới bàn, nâng trục, hai tay gắp hộp, đi ngang sang trái/phải, đặt vào đích, lùi ra.

Kịch bản chỉ dùng API `Robot`, nên cùng file chạy được với backend "sim" (mặc định)
và, khi có phần cứng và được phép, backend "feetech".

    python demos/demo_pick_place.py                 # chạy sim không cửa sổ
    python demos/demo_pick_place.py --video media/m1_pick_place.mp4
    mjpython demos/demo_pick_place.py --viewer      # macOS, xem trực tiếp
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))

from faibotic import Robot, spec  # noqa: E402
from faibotic import model as scene  # noqa: E402

# Nhiệm vụ (toạ độ thế giới, lấy từ bố cục cảnh; chưa dùng thị giác)
STOP_X = scene.TABLE_EDGE_X - 0.215     # dừng đế cách mép bàn 1,5 cm (thân đế bán kính 0,20 m). 24/11: 0,25 -> 0,215 cùng RAISE 0,04 (trần gập 70); thử 0,21-0,22 đạt, 0,20 và 0,23-0,25 trượt
LIFT_H = 0.45                            # 22/11/2026: 0,31 -> 0,355, trục vai SDK hạ 45 mm về đúng CAD (0,55 + lift); tay giữ nguyên độ cao so với bàn
                                         # 0,355 -> 0,45: giới hạn gập 87,5 + bảng ghép theo cuộn, kẹp chĩa xuống ở x 0,37 chỉ tới z 0,81 (cần 0,865 ở điểm trên hộp)
GRASP_DY = 0.11                          # hai điểm kẹp cách tâm hộp theo trục dài (hộp dài 26 cm). S2: 0,095 -> 0,11 vì vai rộng 0,475 m: điểm 0,095 ra ngoài tầm kẹp chĩa xuống ở x 0,37
GRASP_Z = scene.TABLE_HEIGHT + scene.BOX_HALF[2] + 0.015    # kẹp AM-ARM200 ngón 34 mm, thân kẹp ngay sau ngón: kẹp cao hơn 10 mm để mặt hộp không chạm thân kẹp
RAISE = 0.04                             # 24/11: 0,08 -> 0,04: trần gập 70 (đáy vỏ cổ tay chạm cẳng tay ở 73,4), kẹp chĩa xuống ở x 0,37 lift 0,45 chỉ tới z 0,84


def run_pick_place(robot: Robot, log=print, on_step=None) -> None:
    """Chạy hết kịch bản. on_step(robot) được gọi sau mỗi bước điều khiển (dùng để ghi video/kiểm tra)."""
    if on_step is not None:
        orig = robot.step

        def step(dt=None):
            orig(dt)
            on_step(robot)
        robot.step = step

    bx, by = scene.BOX_START
    tx, ty = scene.BOX_TARGET
    log("1/7 về tư thế gốc, trục nâng thấp")
    robot.set_gripper("left", 0)
    robot.set_gripper("right", 0)
    robot.wait(0.5)
    log("2/7 đi tới gần bàn, nâng trục, áp sát bàn")
    robot.move_base(dx=STOP_X - 0.2, duration=3.5)
    robot.move_joints({"lift": LIFT_H}, duration=max(2.0, 1.05 * (LIFT_H - robot._targets["lift"]) / spec.LIFT_SPEED_MAX))   # lift 0,45 cần ~31 s ở LIFT_SPEED_MAX (backend feetech hạn tốc)
    robot.move_base(dx=0.2, duration=1.5)
    robot.wait(0.3)
    log("3/7 đưa hai tay lên trên hộp, mở kẹp")
    above = {}
    grasp = {}
    for side, sgn in (("left", 1), ("right", -1)):
        c = robot.world_to_robot([bx, by + sgn * GRASP_DY, GRASP_Z])
        p = robot.grasp_point(side, c, scene.BOX_HALF[0])          # vật tựa ngón cố định của kẹp AM-ARM200
        p = c if p is None else p
        grasp[side] = p
        above[side] = p + [0, 0, RAISE]
    robot.move_tcp(above, duration=2.5)
    robot.move_joints({"left_gripper": 100, "right_gripper": 100}, duration=0.6)
    log("4/7 hạ xuống, đóng kẹp")
    robot.move_tcp(grasp, duration=1.5)
    robot.wait(0.2)
    robot.move_joints({"left_gripper": 0, "right_gripper": 0}, duration=0.8)
    robot.wait(0.4)
    log("5/7 nhấc hộp, đi ngang tới đích")
    robot.move_tcp({s: p + [0, 0, RAISE] for s, p in grasp.items()}, duration=1.5)
    robot.move_base(dx=tx - bx, dy=ty - by, duration=3.5)
    robot.wait(0.3)
    log("6/7 đặt hộp, mở kẹp")
    place = {}
    for side, sgn in (("left", 1), ("right", -1)):
        c = robot.world_to_robot([tx, ty + sgn * GRASP_DY, GRASP_Z + 0.003])
        p = robot.grasp_point(side, c, scene.BOX_HALF[0])
        place[side] = c if p is None else p
    robot.move_tcp({s: p + [0, 0, RAISE] for s, p in place.items()}, duration=0.8)
    robot.move_tcp(place, duration=1.5)
    robot.move_joints({"left_gripper": 100, "right_gripper": 100}, duration=0.6)
    robot.move_tcp({s: p + [0, 0, RAISE] for s, p in place.items()}, duration=1.0)
    log("7/7 về tư thế gốc, lùi ra")
    robot.move_joints({"left_gripper": 0, "right_gripper": 0}, duration=0.4)
    robot.home(duration=1.5)
    robot.move_base(dx=-0.4, duration=2.5)
    robot.move_joints({"lift": 0.0}, duration=1.5)
    robot.wait(0.5)


class VideoWriter:
    """Ghi mp4 qua ffmpeg (pipe khung RGB thô), H.264 yuv420p."""

    def __init__(self, path: str, width=1280, height=720, fps=30, threads=2):
        if not shutil.which("ffmpeg"):
            raise RuntimeError("cần ffmpeg để ghi video")
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.proc = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{width}x{height}", "-r", str(fps), "-i", "-", "-c:v", "libx264",
             "-preset", "medium", "-crf", "22", "-pix_fmt", "yuv420p", "-threads", str(threads),
             "-movflags", "+faststart", path], stdin=subprocess.PIPE)
        self.frames = 0

    def write(self, rgb) -> None:
        self.proc.stdin.write(rgb.tobytes())
        self.frames += 1

    def close(self) -> None:
        self.proc.stdin.close()
        self.proc.wait()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", help="đường dẫn mp4 cần ghi")
    ap.add_argument("--camera", default="overview")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--viewer", action="store_true", help="mở cửa sổ MuJoCo (macOS: chạy bằng mjpython)")
    a = ap.parse_args()

    robot = Robot(backend="sim", viewer=a.viewer)
    writer = None
    on_step = None
    stats = {"render_s": 0.0}
    if a.video:
        writer = VideoWriter(a.video, a.width, a.height, a.fps)
        b = robot.backend
        b.model.vis.global_.offwidth = max(a.width, 640)
        b.model.vis.global_.offheight = max(a.height, 480)
        next_t = [0.0]

        def on_step(r):
            while b.data.time + 1e-9 >= next_t[0]:
                t0 = time.perf_counter()
                writer.write(b.camera(a.camera, a.width, a.height))
                stats["render_s"] += time.perf_counter() - t0
                next_t[0] += 1.0 / a.fps
    t0 = time.perf_counter()
    run_pick_place(robot, on_step=on_step)
    wall = time.perf_counter() - t0
    box = robot.backend.body_pos("box")
    print(f"sim {robot.backend.data.time:.1f} s, wall {wall:.1f} s, hộp cuối {box.round(3).tolist()}")
    if writer:
        writer.close()
        print(f"video {a.video}: {writer.frames} khung, render {stats['render_s'] / max(writer.frames, 1):.3f} s/khung")
    robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
