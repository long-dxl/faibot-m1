# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Your own teleoperation loop: send joint targets at a fixed rate and always stop safely.

The targets here come from a sine wave; replace `read_leader()` with a leader arm or a keyboard.

    python sdk/examples/02_teleop_loop.py
"""
import math
import time

from faibotic import Robot

RATE_HZ = 50


def read_leader(t: float) -> dict[str, float]:
    """Stand-in for a leader arm: swing the left wrist roll by 30 degrees."""
    return {"left_wrist_roll": math.radians(30) * math.sin(2 * math.pi * 0.25 * t)}


def main(seconds: float = 3.0) -> int:
    robot = Robot("sim")
    try:
        t0 = time.monotonic()
        for i in range(int(seconds * RATE_HZ)):
            sent = robot.set_joint_positions(read_leader(i / RATE_HZ))   # clamped to the soft limits
            robot.step()                                          # one control cycle (0.02 s)
            if i % RATE_HZ == 0:
                roll = math.degrees(robot.get_state()["joints"]["left_wrist_roll"])
                print(f"t={i / RATE_HZ:.1f} s  wrist roll: sent {math.degrees(sent['left_wrist_roll']):6.1f} deg, measured {roll:6.1f} deg")
        print(f"ran {seconds:.0f} s of simulated time in {time.monotonic() - t0:.1f} s")
    finally:
        robot.set_base_velocity(0, 0, 0)
        robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
