# faibot-m1

Python SDK and command-line tool for the Faibot M1, a two-arm mobile robot with a lifting column. It
runs in a MuJoCo simulation today and drives the real robot over a Feetech STS servo bus from Linux
or macOS. Both backends sit behind one `Robot` class, so a script written against the simulation
runs unchanged on hardware.

## Install

Run this from the root of the repository. The install is editable because the simulation reads its
meshes from `sim/assets` next to the package.

```bash
python3 -m venv .venv
. .venv/bin/activate               # Windows: .venv\Scripts\activate
pip install -e "sdk[dev]"
faibot-m1 doctor                    # checks the computer; talks to no servo
```

To talk to the real robot you need a USB-serial adapter for the servo bus:

| Adapter | How to use it |
|---|---|
| USB bus-servo board, CH340 chip | Shows up as `/dev/ttyUSB0` on Linux, `/dev/cu.usbserial-*` on macOS, `COM3` on Windows. Pass it as `--port`. |
| USB bus-servo board, CP210x chip | Same ports. macOS may need the vendor driver. |

On a Raspberry Pi, `pip install -e "sdk[hw]"` adds the GPIO library behind the E-stop and DOCK inputs.

```bash
# Linux, once: allow your user to open serial ports, then log in again
sudo usermod -aG dialout "$USER"
```

## First run

The arms have no brakes. If servo power is cut while an arm is away from its rest pose, it falls,
so keep the workspace clear. The lifting column is held by a latch, but only after it engages.

```bash
faibot-m1 sim                           # 1. keyboard teleoperation in the simulation window
faibot-m1 selftest                      # 2. headless check of model, teleop and IK; exit 0 = pass
faibot-m1 ports                         # 3. which serial ports are plugged in
faibot-m1 scan --port /dev/ttyUSB0      # 4. ping servo IDs 1-16 and 21-22; sends PING and READ only
faibot-m1 status --port /dev/ttyUSB0    # 5. position, load, voltage and temperature per servo
```

Steps 4 and 5 read only: they never switch torque on. Without a robot, add `--port mock` to run
them against a simulated bus (simulated bus only; nothing here has been run on a finished robot yet).

## Commands

Commands that print a report take `--json`. Every command takes `--help`.

**Look, without touching a servo**

| Command | What it does |
|---|---|
| `doctor` | Verify Python, the packages, the repository checkout and that the simulation model builds. |
| `ports` | List serial ports with their USB vendor and product IDs. |
| `model` | Print the joint table: servo ID, servo model, bus and limits. |

**Read servos (never enables torque)**

| Command | What it does |
|---|---|
| `scan --port P` | Ping every ID. Exit 1 if any servo is silent, 2 if the port cannot be opened. |
| `status --port P` | Position in degrees, load in percent, voltage and temperature for each servo. |

**Simulate**

| Command | What it does |
|---|---|
| `sim` | Open the simulation window with keyboard teleoperation. `--seconds N` closes it after N seconds. |
| `selftest` | Headless run of teleop and IK against the model, printed as JSON. |
| `demo` | Pick-and-place with coordinate checks; exit 0 when the box lands on target. |
| `lab pick\|sort\|open` | Lab scene tasks: pick three objects, sort by colour, open a drawer, lid and fridge door. |

Moving the real robot is not part of the command line yet; use the Python API below.

## Python

```python
from faibotic import Robot

with Robot("sim") as robot:                                    # "feetech" for the real robot
    robot.move_joints({"lift": 0.288}, duration=1.0)           # lift height, metres
    robot.move_tcp("left", [0.34, 0.217, 0.72], duration=1.5)  # tool point in the robot frame, metres
    robot.set_gripper("left", 0)                               # 0 closed .. 100 open
    robot.wait(0.5)
    print(robot.get_state()["lift"])
    print(robot.tcp_position("left"))
    robot.home()
```

Arm joints are in radians, the lift in metres, grippers on a 0 to 100 scale, base velocities in
m/s and rad/s. Joint names are `left_pan`, `left_shoulder`, `left_elbow`, `left_wrist_flex`,
`left_wrist_roll`, `left_gripper`, the same with `right_`, and `lift`.

| You want to | Call |
|---|---|
| Stream targets from a loop you wrote | `robot.set_joint_positions({...})`, then `robot.step()` |
| Move and wait | `robot.move_joints({...}, duration)`, `robot.move_tcp(side, xyz, duration)` |
| Move both arms at once | `robot.move_tcp({"left": p1, "right": p2}, duration)` |
| Read the robot | `get_state()`, `arm_q(side)`, `tcp_position(side)`, `soft_limits()` |
| Check a point is reachable | `robot.solve_ik(side, xyz)` returns joint angles, or `None` |
| Drive the base | `robot.move_base(dx, dy, dyaw, duration)`, `robot.set_base_velocity(vx, vy, wz)` |
| Return to the start pose | `robot.home()` |
| Let simulated time pass | `robot.wait(seconds)` |

Targets outside the soft limits are clamped, and a joint move that would pass through a forbidden
shoulder and elbow combination is split into safe legs, so a call never asks for a pose the model
forbids.

### Your own teleoperation loop

Write targets at a fixed rate and put the shutdown in `finally`, so the base stops even if your
code raises.

```python
import time
from faibotic import Robot

robot = Robot("sim")
try:
    while True:
        targets = read_my_leader()                  # your function: {"left_wrist_roll": radians, ...}
        robot.set_joint_positions(targets)          # clamped to the soft limits
        robot.step()                                # one 50 Hz control cycle
        time.sleep(robot.dt)                        # the simulation can run faster; hardware cannot
finally:
    robot.set_base_velocity(0, 0, 0)
    robot.close()
```

`faibotic.teleop.Teleop` is the keyboard version used by `faibot-m1 sim`, and `LeaderMap` converts a
leader arm's readings into M1 joint targets. A runnable loop is in
[`examples/02_teleop_loop.py`](examples/02_teleop_loop.py).

### Tool poses from a policy

A policy usually outputs where the tool should be, not joint angles. Solve each pose with
`set_tcp`, which runs the inverse kinematics and sends the result. It returns `False` and sends
nothing when the point is out of reach.

```python
for xyz in policy_tool_points:                      # robot frame, metres
    if not robot.set_tcp("left", xyz):
        break                                       # out of reach: stop instead of guessing
    robot.step()
```

While the policy runs, keep the base still and the lift at a fixed height, because the poses are
expressed in the robot frame and a moving base or lift shifts the whole workspace. See
[`examples/03_tool_poses.py`](examples/03_tool_poses.py).

### Cameras

```python
frame = robot.camera("head")                        # RGB image, height x width x 3
```

The simulation provides `head`, `left_wrist_cam`, `right_wrist_cam`, `overview` and `side`, rendered
off-screen at 640 x 480. On the real robot, `head`, `left_wrist_cam` and `right_wrist_cam` map to
USB camera indices 0, 1 and 2 and need `pip install -e "sdk[cameras]"`.

Complete scripts are in [`examples/`](examples).

## Safety

- **No brakes on the arms.** When servo power drops, an arm falls unless it is at rest or supported.
- **E-stop.** A hardware button cuts servo power through a contactor. The SDK watches the E-stop
  input: it refuses `enable()` until the latch has been reset, switches torque off the moment the
  button is pressed, and does not restart by itself. Reset by hand after the cause is cleared.
- **DOCK interlock.** The two-arm station is held to the base by two lock switches, `DOCK_L` and
  `DOCK_R`, wired in series with the E-stop loop. With either lock open the SDK raises `DockOpen`
  and refuses to enable torque, move the lift or drive the base. If a lock opens while the robot is
  running, power is cut in hardware and the SDK turns torque off and stays faulted until the lock
  is closed and `clear_fault()` is called. Skipping the check is possible only with
  `allow_no_dock=True`, for supervised bench work.
- **Lifting column latch.** A solenoid latch catches the column when power is lost; it can still
  drop by one notch, about 5 mm.
- **Soft limits and keep-out zones** apply to every command on both backends.
- **Not verified on a finished robot.** The hardware paths are tested against a simulated servo
  bus and a simulated GPIO. No safety certification is claimed.

Assembly and first power-up steps: [docs/vi/assembly-guide.md](../docs/vi/assembly-guide.md) and
[product/06-lap-rap.md](../product/06-lap-rap.md).

## Hardware

<!-- joints-table:start -->
| Joint | Servo ID | Model | Bus | Limits |
|---|---|---|---|---|
| left_pan | 1 | STS3095-C002 | BUS_L | -70 .. 70 deg |
| left_shoulder | 2 | STS3095-C002 | BUS_L | -80 .. 48.5 deg |
| left_elbow | 3 | STS3095-C002 | BUS_L | -10 .. 127 deg |
| left_wrist_flex | 4 | STS3215-C018 | BUS_L | -77.5 .. 87.5 deg |
| left_wrist_roll | 5 | STS3215-C018 | BUS_L | -160 .. 160 deg |
| left_gripper | 6 | STS3215-C018 | BUS_L | 0 .. 100 % |
| right_pan | 7 | STS3095-C002 | BUS_R | -70 .. 70 deg |
| right_shoulder | 8 | STS3095-C002 | BUS_R | -80 .. 48.5 deg |
| right_elbow | 9 | STS3095-C002 | BUS_R | -10 .. 127 deg |
| right_wrist_flex | 10 | STS3215-C018 | BUS_R | -77.5 .. 87.5 deg |
| right_wrist_roll | 11 | STS3215-C018 | BUS_R | -160 .. 160 deg |
| right_gripper | 12 | STS3215-C018 | BUS_R | 0 .. 100 % |
| wheel_0 | 13 | STS3215-C018 | BUS_B | continuous (wheel) |
| wheel_1 | 14 | STS3215-C018 | BUS_B | continuous (wheel) |
| wheel_2 | 15 | STS3215-C018 | BUS_B | continuous (wheel) |
| lift | 16 | STS3095-C002 | BUS_B | 0 .. 450 mm |
| left_wrist_yaw | 21 | STS3215-C018 | BUS_L | -81 .. 80 deg |
| right_wrist_yaw | 22 | STS3215-C018 | BUS_R | -81 .. 80 deg |
<!-- joints-table:end -->

Feetech STS servos on a half-duplex serial bus at 1 Mbit/s, with the TTL protocol rewritten from
the manufacturer's description. The servos are split over three buses: `BUS_L` and `BUS_R` ride on
the two-arm station, `BUS_B` on the base plate (three wheels and the lift). Limits are the soft
limits in [`faibotic/spec.py`](faibotic/spec.py); the table is generated from that file by
`sdk/tools/sinh_bang_khop.py`, never typed by hand.

The pose that `robot.home()` returns to is `HOME_ARM` in the same file. The CAD, the part list and the URDF are in
[`../hardware`](../hardware).

## Development

```bash
pip install -e "sdk[dev]"
python -m pytest -q tests sdk/tests              # M1_SKIP_RENDER=1 skips camera tests without OpenGL
python3 scripts/kiem_cheo.py                     # cross-checks CAD, simulation, web assets and docs
python3 scripts/check_ban_cu.py                  # scans text for retired product names
python3 sdk/tools/sinh_bang_khop.py --ghi        # regenerate the joint table above
./run.sh --selftest                              # same as faibot-m1 selftest
```

`sdk/faibotic/spec.py` is the single source of numbers: the website and the Studio read it too.
After changing it, run `scripts/kiem_cheo.py` before committing.
