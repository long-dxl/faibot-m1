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
| `{left,right}_wrist_flex` | revolute | y | -77.5 to 70 deg |
| `{left,right}_wrist_roll` | revolute | x | -160 to 160 deg |
| `{left,right}_gripper` | revolute | z | 0 to 64 deg |

`left_tcp_fixed` and `right_tcp_fixed` mark the tool centre points.

## Not yet verified

- Masses and inertias are estimates from the CAD, not measurements.
- The wheel joints are for completeness; the omni base is handled outside the URDF.
- Meshes are reduced for display. Collision shapes are boxes and cylinders.
- Nothing here has been checked on a real robot.

Copyright (c) 2026 Leo Dao (Faibotic). `m1.urdf` and this README are under the Apache License 2.0, see [LICENSE](LICENSE). The meshes in `assets/` carry their own licence per file: the arm, gripper and servo meshes derived from third-party designs are Apache License 2.0, the meshes that contain parts Faibotic drew (base, lift carriage, wrist camera) are CC BY-NC-SA 4.0. The file-by-file list and the third-party sources are in [`../../NOTICE`](../../NOTICE).
