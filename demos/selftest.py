"""Tự kiểm không mở cửa sổ: nạp mô hình, mô phỏng 5 s với phím giả, IK tới 3 điểm (sai số < 1 cm).

    ./run.sh --selftest      (thoát mã 0 nếu đạt, in JSON)
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))

from faibotic import Robot  # noqa: E402
from faibotic.teleop import K, Teleop  # noqa: E402

# (giây, phím giữ, phím bấm lúc bắt đầu đoạn)
SCRIPT = [
    (1.0, {"W"}, []),
    (0.5, {"A"}, []),
    (0.5, {"Q"}, []),
    (1.0, {"R"}, ["SPACE"]),
    (0.5, {"UP"}, []),
    (0.5, {"PGDN"}, ["2"]),
    (1.0, set(), ["H"]),
]


def main() -> int:
    res = {"checks": {}}
    robot = Robot(backend="sim")
    b = robot.backend
    res["model"] = {"nq": int(b.model.nq), "nu": int(b.model.nu), "nbody": int(b.model.nbody)}
    tele = Teleop(robot)
    nan = 0
    t = 0.0
    for dur, hold, presses in SCRIPT:
        for p in presses:
            tele.press(K[p])
        down = {K[k] for k in hold}
        for _ in range(int(round(dur / robot.dt))):
            tele.update(down, robot.dt)
            robot.step()
            if not (np.all(np.isfinite(b.data.qpos)) and np.all(np.isfinite(b.data.qvel))):
                nan += 1
        t += dur
    st = robot.get_state()
    x, y, yaw = st["base_pose"]
    res["teleop"] = {"sim_s": round(float(b.data.time), 2), "base_pose": [round(v, 3) for v in st["base_pose"]],
                     "lift_m": round(st["lift"], 3), "left_gripper": round(st["gripper"]["left"], 1),
                     "nan_steps": nan}
    res["checks"]["sim_5s"] = b.data.time >= 5.0 - 1e-6
    res["checks"]["no_nan"] = nan == 0
    # đế tăng tốc theo dốc chống lật (0,05 m/s², 0,15 rad/s²), giữ phím 0,5–1 s chỉ đi vài cm
    res["checks"]["base_forward"] = x > 0.05
    res["checks"]["base_left"] = y > 0.01
    res["checks"]["base_turn"] = yaw > 0.02
    res["checks"]["lift_up"] = st["lift"] > 0.08
    res["checks"]["gripper_open"] = st["gripper"]["left"] > 80

    # IK tới 3 điểm trong khung robot, đo bằng site TCP thật của MuJoCo
    b.reset()
    robot._targets = dict(b.read_joints())
    robot.move_joints({"lift": 0.288}, 1.0)   # S1G: giữ trục vai 0,85 m như trước (0,55 + 0,30)
    # v0.3: hai điểm trái cũ (0,35; 0,20; 0,85) và (0,30; 0,05; 0,95) cần khuỷu > 130 độ hoặc vai < -90 độ, ngoài giới hạn mới; thay bằng hai điểm xa vai hơn
    # (cổ tay theo URDF upstream): (0,40; 0,20; 0,85) ngoài tầm khi kẹp chĩa xuống (trục gập cách vai 0,38 m > 0,338 m); hạ còn 0,78 m
    # (giới hạn gập 87,5 + bảng ghép quét theo cuộn): ở lift 0,3 kẹp chĩa xuống chỉ tới z 0,74 (x 0,40) / 0,76 (x 0,38); hạ 6 cm
    # S1G: trục pan CAD P-L (0,0599; ±0,18726) thay (0,12; ±0,17): điểm dời theo để giữ nguyên so với trục pan
    # 24/11: trần gập 87,5 -> 70 (đáy vỏ cổ tay chạm cẳng tay ở 73,4): kẹp chĩa xuống ở x 0,32-0,34 chỉ tới z 0,72; hạ cả ba điểm về 0,70
    pts = {"left": [[0.34, 0.217, 0.70], [0.32, 0.137, 0.70]], "right": [[0.32, -0.137, 0.70]]}
    errs = []
    for side, plist in pts.items():
        for p in plist:
            robot.move_tcp(side, p, duration=1.5)
            robot.wait(0.8)
            x, y, yaw = b.base_pose()
            site = b.site_pos(f"{side}_tcp")
            # site trong khung thế giới -> khung robot
            c, s = np.cos(yaw), np.sin(yaw)
            d = site[:2] - [x, y]
            pr = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], site[2]])
            errs.append(round(float(np.linalg.norm(pr - p)), 4))
    res["ik_errors_m"] = errs
    res["checks"]["ik_under_1cm"] = max(errs) < 0.01
    res["pass"] = all(res["checks"].values())
    print(json.dumps(res, ensure_ascii=False, indent=2))
    robot.close()
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
