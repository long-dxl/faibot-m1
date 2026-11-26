# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Follow a list of tool poses, the way a learned policy outputs them: one IK solve per step.

    python sdk/examples/03_tool_poses.py
"""
import numpy as np

from faibotic import Robot

SIDE = "left"
START = np.array([0.34, 0.217, 0.70])      # tool point, robot frame, metres (gripper pointing down reaches z 0.72 here at lift 0.288)
END = np.array([0.32, 0.137, 0.70])


def main(steps: int = 30) -> int:
    with Robot("sim") as robot:
        robot.move_joints({"lift": 0.288}, duration=1.0)
        robot.move_tcp(SIDE, START, duration=1.5)
        worst = 0.0
        for k in range(1, steps + 1):
            target = START + (END - START) * k / steps
            if not robot.set_tcp(SIDE, target):                  # False: the point is out of reach, nothing is sent
                print("out of reach:", target.round(3).tolist())
                return 1
            robot.step()
            worst = max(worst, float(np.linalg.norm(robot.tcp_position(SIDE, source="target") - target)))
        robot.wait(0.5)
        print("worst commanded-pose error (mm):", round(worst * 1000, 2))
        print("reached (m):", [round(float(v), 3) for v in robot.tcp_position(SIDE)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
