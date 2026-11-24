"""Teleop bàn phím M1 trong mô phỏng: mở cửa sổ MuJoCo, điều khiển bằng phím.

    python demos/teleop.py              # hoặc ./run.sh
    python demos/teleop.py --seconds 3  # mở rồi tự đóng sau 3 s (kiểm cửa sổ)
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))
sys.path.insert(0, str(ROOT / "demos"))

from faibotic import Robot  # noqa: E402
from faibotic.glviewer import GLViewer  # noqa: E402
from faibotic.teleop import HELP, Teleop  # noqa: E402

CAMERAS = ["free", "overview", "track", "head", "left_wrist_cam", "right_wrist_cam"]


class Abort(Exception):
    pass


def print_help() -> None:
    print("\nFaibotic M1, teleop trong mô phỏng. Phím (bấm vào cửa sổ trước):")
    for k, v in HELP:
        print(f"  {k:<24} {v}")
    print("Chuột: kéo trái xoay, kéo phải dời, cuộn thu phóng.\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=0, help="tự đóng sau N giây (0 = chạy tới khi đóng)")
    ap.add_argument("--screenshot", help="lưu ảnh cửa sổ khi tự đóng (dùng với --seconds)")
    a = ap.parse_args()

    robot = Robot(backend="sim")
    b = robot.backend
    viewer = GLViewer(b.model, b.data, "Faibotic M1 (mô phỏng)")
    tele = Teleop(robot)
    cam_i = 0
    show_help = True
    print_help()
    help_l = "\n".join(k for k, _ in HELP)
    help_r = "\n".join(v for _, v in HELP)
    t_start = time.perf_counter()

    def overlay(extra: str = ""):
        texts = [("bl", "Faibotic M1  |  robot dạng số", tele.status() if not extra else extra)]
        if show_help:
            texts.insert(0, ("tl", help_l, help_r))
        texts.append(("br", "camera", CAMERAS[cam_i]))
        return texts

    def apply_camera():
        name = CAMERAS[cam_i]
        if name == "free":
            viewer.set_free_camera()
        elif name == "track":
            viewer.track_body("base")
        else:
            viewer.set_fixed_camera(name)

    def run_demo():
        from demo_pick_place import run_pick_place
        b.reset()
        robot._targets = dict(b.read_joints())
        wall0, sim0 = time.perf_counter(), b.data.time
        last_render = [0.0]
        stage = ["demo"]

        def on_step(r):
            if viewer.should_close() or 256 in viewer.keys_down:  # Esc dừng demo
                raise Abort
            lag = (b.data.time - sim0) - (time.perf_counter() - wall0)
            if lag > 0:
                time.sleep(lag)
            if time.perf_counter() - last_render[0] > 1 / 60:
                viewer.render(overlay(f"DEMO TỰ ĐỘNG: {stage[0]}\n(Esc để dừng)"))
                last_render[0] = time.perf_counter()

        orig_step = robot.step
        try:
            run_pick_place(robot, log=lambda m: stage.__setitem__(0, m), on_step=on_step)
        except Abort:
            print("demo dừng giữa chừng")
        finally:
            robot.step = orig_step
            robot.set_base_velocity(0, 0, 0)
            viewer.pop_events()
            tele.reset_state()

    wall0, sim0 = time.perf_counter(), b.data.time
    try:
        while not viewer.should_close():
            if a.seconds and time.perf_counter() - t_start > a.seconds:
                if a.screenshot:
                    viewer.screenshot(a.screenshot)
                break
            for key in viewer.pop_events():
                cmd = tele.press(key)
                if cmd == "quit":
                    return 0
                if cmd == "reset":
                    b.reset()
                    robot._targets = dict(b.read_joints())
                    tele.reset_state()
                elif cmd == "help":
                    show_help = not show_help
                elif cmd == "camera":
                    cam_i = (cam_i + 1) % len(CAMERAS)
                    apply_camera()
                elif cmd == "demo":
                    run_demo()
                    wall0, sim0 = time.perf_counter(), b.data.time
            # đuổi theo thời gian thực bằng các bước điều khiển 20 ms
            target_sim = sim0 + (time.perf_counter() - wall0)
            n = 0
            while b.data.time < target_sim and n < 10:
                tele.update(viewer.keys_down, robot.dt)
                robot.step()
                n += 1
            if n == 10:  # máy chậm: không cố đuổi
                wall0, sim0 = time.perf_counter(), b.data.time
            viewer.render(overlay())
    finally:
        viewer.close()
        robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
