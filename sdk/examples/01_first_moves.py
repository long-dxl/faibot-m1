"""First moves on the simulation backend: raise the lift, reach with the left arm, close the gripper.

    python sdk/examples/01_first_moves.py
"""
from faibotic import Robot


def main() -> int:
    with Robot("sim") as robot:
        robot.move_joints({"lift": 0.288}, duration=1.0)             # lift height in metres
        robot.move_tcp("left", [0.34, 0.217, 0.72], duration=1.5)    # tool centre point, robot frame, metres
        robot.wait(0.5)
        robot.set_gripper("left", 0)                                 # 0 = closed, 100 = open
        robot.wait(0.5)
        state = robot.get_state()
        print("lift (m):", round(state["lift"], 3))
        print("left gripper (0-100):", round(state["gripper"]["left"], 1))
        print("left tool point (m):", [round(float(v), 3) for v in robot.tcp_position("left")])
        robot.home()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
