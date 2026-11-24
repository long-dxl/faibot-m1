import os
import sys

import numpy as np
import pytest

from faibotic import Robot, spec


@pytest.fixture
def robot():
    r = Robot("sim")
    yield r
    r.close()


def test_bad_backend():
    with pytest.raises(ValueError):
        Robot("abc")


def test_state_keys(robot):
    st = robot.get_state()
    assert set(st["joints"]) == set(spec.all_joint_names())
    assert len(st["base_pose"]) == 3


def test_move_base_omni(robot):
    robot.move_base(dx=0.4, duration=2.0)
    robot.move_base(dy=-0.2, duration=1.5)
    robot.move_base(dyaw=0.5, duration=1.5)
    robot.wait(0.5)
    x, y, yaw = robot.get_state()["base_pose"]
    assert x == pytest.approx(0.4, abs=0.01)
    assert y == pytest.approx(-0.2, abs=0.01)
    assert yaw == pytest.approx(0.5, abs=0.02)


def test_lift_and_gripper(robot):
    robot.move_joints({"lift": 0.4}, 1.5)
    robot.set_gripper("left", 100)
    robot.wait(1.0)
    st = robot.get_state()
    assert st["lift"] == pytest.approx(0.4, abs=0.01)
    assert st["gripper"]["left"] > 95


def test_move_tcp_accuracy(robot):
    robot.move_joints({"lift": 0.333}, 1.0)   # S1G: trục vai 0,5619 + lift (CAD P-L); 0,333 giữ trục vai 0,895 m như trước (0,55 + 0,345)
    target = np.array([0.34, 0.137, 0.78])  # S1G: trục pan 0,0599 / ±0,18726 (trước 0,12 / ±0,17): giữ nguyên điểm so với trục pan (trước 0,40; 0,12)
    robot.move_tcp("left", target, duration=1.5)
    robot.wait(1.0)
    assert np.linalg.norm(robot.tcp_position("left") - target) < 0.01
    with pytest.raises(ValueError):
        robot.move_tcp("right", [2.0, 0, 0.8])


def test_world_to_robot(robot):
    robot.move_base(dx=0.5, duration=2.0)
    robot.move_base(dyaw=np.pi / 2, duration=2.0)
    robot.wait(0.5)
    p = robot.world_to_robot([0.5, 1.0, 0.7])
    assert np.allclose(p, [1.0, 0.0, 0.7], atol=0.02)


@pytest.mark.skipif(os.environ.get("M1_SKIP_RENDER") == "1", reason="không có OpenGL offscreen")
def test_camera(robot):
    for cam in ("head", "left_wrist_cam", "overview"):
        img = robot.backend.camera(cam, 160, 120)
        assert img.shape == (120, 160, 3) and img.dtype == np.uint8 and img.std() > 1
