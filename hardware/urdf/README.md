# Faibot M1 URDF

`m1.urdf` is the robot for visual and kinematic work. It is a copy of `sim/m1.urdf`, written by `sdk/faibotic/model.py`. Keep `assets/` (STL meshes) beside it. Joint angles follow the SDK convention.

## Joints

| Joint | Type | Axis | Limits |
| --- | --- | --- | --- |
| `wheel_0`, `wheel_1`, `wheel_2` | continuous | z | none |
| `lift` | prismatic | z | 0 to 0.45 m |
| `{left,right}_pan` | revolute | z | -70 to 70 deg |
| `{left,right}_shoulder` | revolute | y | -80 to 48.5 deg |
| `{left,right}_elbow` | revolute | y | -10 to 127 deg |
| `{left,right}_wrist_flex` | revolute | y | -77.5 to 87.5 deg |
| `{left,right}_wrist_roll` | revolute | x | -160 to 160 deg |
| `{left,right}_gripper` | revolute | z | 0 to 64 deg |

`left_tcp_fixed` and `right_tcp_fixed` mark the tool centre points.

## Not yet verified

- Masses and inertias are estimates from the CAD, not measurements.
- The wheel joints are for completeness; the omni base is handled outside the URDF.
- Meshes are reduced for display. Collision shapes are boxes and cylinders.
- Nothing here has been checked on a real robot.

Apache License 2.0 applies, see [LICENSE](LICENSE). The meshes derive from the third-party designs listed in [`../../NOTICE`](../../NOTICE).
