"""Điều khiển M1 bằng code, có cửa sổ xem trực tiếp.

    ./run.sh --repl                 (macOS dùng mjpython -i, Linux dùng python -i)

Gõ lệnh SDK ở dấu >>> và nhìn robot chạy trong cửa sổ MuJoCo.
"""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))
sys.path.insert(0, str(ROOT / "demos"))

from faibotic import Robot  # noqa: E402

robot = Robot(backend="sim", viewer=os.environ.get("M1_NO_VIEWER") != "1")

print("""
Đã có biến `robot` (Robot backend "sim"). Thử:
  robot.move_base(dx=0.5, duration=2)                 # đế tiến 0,5 m
  robot.move_joints({"lift": 0.35}, duration=1.5)     # nâng trục lên 0,35 m (trục vai cao 0,90 m)
  robot.move_tcp("left", [0.35, 0.10, 0.80], duration=2)  # đưa kẹp trái tới điểm (khung robot, m); cần nâng trục trước
  robot.set_gripper("left", 100); robot.wait(0.5)     # mở kẹp (0 = đóng)
  robot.get_state()                                   # đọc khớp, trục nâng, kẹp, tư thế đế
  robot.home(); robot.backend.reset()                 # về tư thế gốc / đặt lại cảnh
  from demo_pick_place import run_pick_place; run_pick_place(robot)   # demo gắp đặt
""")
