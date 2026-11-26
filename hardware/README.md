# Faibot M1 hardware

CAD for the Faibot M1: the whole robot as STEP, one STEP file for every part, and a URDF for simulation.

The Faibot M1 is a mobile bimanual robot from [Faibotic](https://www.faibotic.com). It has eighteen powered joints: three drive wheels, a lifting column and two arms, each with six joints and a parallel gripper (6+1, as in the original AM-ARM200). The frame is mostly printed in PETG, with cut aluminium and steel where it carries load.

- Product page: https://www.faibotic.com/san-pham/faibot-m1
- Python SDK: [`../sdk`](../sdk)

## What this CAD shows

- **Build:** v0.5. Seven STS3095 servos (arm pan, shoulder and elbow on both arms, plus the lift) and eleven STS3215 servos (wrist flex, wrist yaw, wrist roll and gripper on both arms, three drive wheels).
- **Configuration:** the mobile robot with the lift at its lowest point and both arms in the `ready` pose, grippers closed, head camera on its mast.
- **Source:** built from `cad/faibotic/assembly_pl.py` at repository commit `e81a130`, exported on 2026-11-24 (`assembly/`).
- **Units:** millimetres.
- **Not in the CAD:** screws, nuts and inserts (see `media/hardware/bang-oc.csv`), cables, the battery cells, the Pi 5 and the bus boards. The battery and electronics bays are plain blocks. The wrist flex, yaw and roll servos (STS3215) and the six arm STS3095 servos are inside the arm meshes, not separate parts.

## Folder layout

```
assembly/faibot-m1-v0.5-mobile-*.step     the full robot, in four files that share one frame:
                                          base, lift, station (two-arm mount, head camera) and arm
parts/faibotic/                           parts Faibotic designed: printed, laser cut or cut to length
parts/inherited/                          parts taken unchanged from open designs, byte for byte
parts/purchased/                          simplified models of bought parts: servos, wheels, switches, cameras
parts.csv                                 one row per part file: number, name, qty, type, group, licence, origin, source
guide-part-map.json                       which part files each step of the assembly guide uses
urdf/m1.urdf                              the robot for simulators and kinematics, meshes in urdf/assets/
tools/export_parts.py                     the script that makes assembly/, parts/, parts.csv and the table below
tools/part-names.en.csv                   part key to English name and group
tools/README.md                           how to run it
```

## Parts

Every distinct part has a single file, however often it is used; the Qty column says how many the robot needs. Quantities are per robot, counted in the assembly. The left and right copies of a part share a file (it holds the left one).

<!-- parts-table:start -->
| File | Part number | Name | Qty | Type | Group | License |
| --- | --- | --- | ---: | --- | --- | --- |
| [`M1-BS-006-aluminium-2040-tray-rail-190-mm.step`](parts/faibotic/M1-BS-006-aluminium-2040-tray-rail-190-mm.step) | M1-BS-006 | Aluminium 2040 tray rail (190 mm) | 1 | cut | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-031-aluminium-spoke-20-x-40-front.step`](parts/faibotic/M1-BS-031-aluminium-spoke-20-x-40-front.step) | M1-BS-031 | Aluminium spoke 20 x 40 (front) | 1 | cut | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-031-aluminium-spoke-20-x-40-side.step`](parts/faibotic/M1-BS-031-aluminium-spoke-20-x-40-side.step) | M1-BS-031 | Aluminium spoke 20 x 40 (side) | 2 | cut | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-032-counterweight-plate-main.step`](parts/faibotic/M1-BS-032-counterweight-plate-main.step) | M1-BS-032 | Counterweight plate (main) | 1 | cut | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-035-omni-wheel-hub.step`](parts/purchased/M1-BS-035-omni-wheel-hub.step) | M1-BS-035 | Omni wheel hub | 3 | purchased | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-035-omni-wheel-roller.step`](parts/purchased/M1-BS-035-omni-wheel-roller.step) | M1-BS-035 | Omni wheel roller | 54 | purchased | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-035-omni-wheel-side-plate.step`](parts/purchased/M1-BS-035-omni-wheel-side-plate.step) | M1-BS-035 | Omni wheel side plate | 6 | purchased | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-036-wheel-servo-mount-cap.step`](parts/faibotic/M1-BS-036-wheel-servo-mount-cap.step) | M1-BS-036 | Wheel servo mount cap | 3 | printed | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-036-wheel-servo-mount-wall.step`](parts/faibotic/M1-BS-036-wheel-servo-mount-wall.step) | M1-BS-036 | Wheel servo mount wall | 3 | printed | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-037-base-deck.step`](parts/faibotic/M1-BS-037-base-deck.step) | M1-BS-037 | Base deck | 1 | printed | Base | CC-BY-NC-SA-4.0 |
| [`M1-BS-039-counterweight-plate-second-layer.step`](parts/faibotic/M1-BS-039-counterweight-plate-second-layer.step) | M1-BS-039 | Counterweight plate (second layer) | 1 | cut | Base | CC-BY-NC-SA-4.0 |
| [`M1-ST-011-tray-bracket-short.step`](parts/faibotic/M1-ST-011-tray-bracket-short.step) | M1-ST-011 | Tray bracket (short) | 1 | printed | Base | CC-BY-NC-SA-4.0 |
| [`M1-ST-012-tray-piece-00-rear-right.step`](parts/faibotic/M1-ST-012-tray-piece-00-rear-right.step) | M1-ST-012 | Tray piece 00 (rear right) | 1 | printed | Base | CC-BY-NC-SA-4.0 |
| [`M1-ST-012-tray-piece-01-rear-left.step`](parts/faibotic/M1-ST-012-tray-piece-01-rear-left.step) | M1-ST-012 | Tray piece 01 (rear left) | 1 | printed | Base | CC-BY-NC-SA-4.0 |
| [`M1-ST-012-tray-piece-10-front-right.step`](parts/faibotic/M1-ST-012-tray-piece-10-front-right.step) | M1-ST-012 | Tray piece 10 (front right) | 1 | printed | Base | CC-BY-NC-SA-4.0 |
| [`M1-ST-012-tray-piece-11-front-left.step`](parts/faibotic/M1-ST-012-tray-piece-11-front-left.step) | M1-ST-012 | Tray piece 11 (front left) | 1 | printed | Base | CC-BY-NC-SA-4.0 |
| [`M1-LF-002-lift-gear-20-teeth.step`](parts/faibotic/M1-LF-002-lift-gear-20-teeth.step) | M1-LF-002 | Lift gear (20 teeth) | 1 | printed | Lift | CC-BY-NC-SA-4.0 |
| [`O_T_Connector_Dowel_Pin_12_25.stp`](parts/inherited/alohamini2/O_T_Connector_Dowel_Pin_12_25.stp) | M1-LF-004 | Dowel pin 12 x 25 | 1 | inherited | Lift | Apache-2.0 |
| [`M1-LF-041-latch-pawl.step`](parts/purchased/M1-LF-041-latch-pawl.step) | M1-LF-041 | Latch pawl | 1 | purchased | Lift | CC-BY-NC-SA-4.0 |
| [`M1-LF-041-travel-limit-switch.step`](parts/purchased/M1-LF-041-travel-limit-switch.step) | M1-LF-041 | Travel limit switch | 2 | purchased | Lift | CC-BY-NC-SA-4.0 |
| [`M1-LF-050-latch-solenoid.step`](parts/purchased/M1-LF-050-latch-solenoid.step) | M1-LF-050 | Latch solenoid | 1 | purchased | Lift | CC-BY-NC-SA-4.0 |
| [`OB_Main_Assembly_Post1.stp`](parts/inherited/alohamini2/OB_Main_Assembly_Post1.stp) | M1-LF-051 | Column section 1 | 1 | inherited | Lift | Apache-2.0 |
| [`OB_Main_Assembly_Post2.stp`](parts/inherited/alohamini2/OB_Main_Assembly_Post2.stp) | M1-LF-052 | Column section 2 | 1 | inherited | Lift | Apache-2.0 |
| [`OB_Main_Assembly_Post3.stp`](parts/inherited/alohamini2/OB_Main_Assembly_Post3.stp) | M1-LF-053 | Column section 3 | 1 | inherited | Lift | Apache-2.0 |
| [`O_Main_Assembly_Post4.stp`](parts/inherited/alohamini2/O_Main_Assembly_Post4.stp) | M1-LF-054 | Column section 4 (foot) | 1 | inherited | Lift | Apache-2.0 |
| [`O_POST4_Connector_Base.stp`](parts/inherited/alohamini2/O_POST4_Connector_Base.stp) | M1-LF-055 | Column foot connector | 1 | inherited | Lift | Apache-2.0 |
| [`OB_T_Connector_Middle.stp`](parts/inherited/alohamini2/OB_T_Connector_Middle.stp) | M1-LF-057 | Middle T frame | 1 | inherited | Lift | Apache-2.0 |
| [`M1-LF-060-latch-bracket.step`](parts/faibotic/M1-LF-060-latch-bracket.step) | M1-LF-060 | Latch bracket | 1 | printed | Lift | CC-BY-NC-SA-4.0 |
| [`M1-LF-061-latch-sensor-switch.step`](parts/purchased/M1-LF-061-latch-sensor-switch.step) | M1-LF-061 | Latch sensor switch | 1 | purchased | Lift | CC-BY-NC-SA-4.0 |
| [`M1-TR-001-station-front-rail-2040.step`](parts/faibotic/M1-TR-001-station-front-rail-2040.step) | M1-TR-001 | Station front rail 2040 | 1 | cut | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-002-station-ear-2040.step`](parts/faibotic/M1-TR-002-station-ear-2040.step) | M1-TR-002 | Station ear 2040 | 2 | cut | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-003-arm-base-bracket.step`](parts/faibotic/M1-TR-003-arm-base-bracket.step) | M1-TR-003 | Arm base bracket | 2 | printed | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-004-bushing-block.step`](parts/faibotic/M1-TR-004-bushing-block.step) | M1-TR-004 | Bushing block | 2 | printed | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-004b-steel-bushing.step`](parts/purchased/M1-TR-004b-steel-bushing.step) | M1-TR-004b | Steel bushing | 2 | purchased | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-008-draw-latch.step`](parts/purchased/M1-TR-008-draw-latch.step) | M1-TR-008 | Draw latch | 2 | purchased | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-009-latch-limit-switch.step`](parts/purchased/M1-TR-009-latch-limit-switch.step) | M1-TR-009 | Latch limit switch | 2 | purchased | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-014-handle.step`](parts/faibotic/M1-TR-014-handle.step) | M1-TR-014 | Handle | 2 | printed | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-020-dock-mount-carriage-side.step`](parts/faibotic/M1-TR-020-dock-mount-carriage-side.step) | M1-TR-020 | Dock mount (carriage side) | 1 | printed | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-021-dock-taper-pin.step`](parts/purchased/M1-TR-021-dock-taper-pin.step) | M1-TR-021 | Dock taper pin | 2 | purchased | Station | CC-BY-NC-SA-4.0 |
| [`M1-TR-022-latch-hook.step`](parts/purchased/M1-TR-022-latch-hook.step) | M1-TR-022 | Latch hook | 2 | purchased | Station | CC-BY-NC-SA-4.0 |
| [`OB_F_J4_Elbow.stp`](parts/inherited/am-arm200/OB_F_J4_Elbow.stp) | M1-AR-001 | J4 elbow | 2 | inherited | Arm | Apache-2.0 |
| [`OB_F_J5_Elbow_Wrist_Bracket.stp`](parts/inherited/am-arm200/OB_F_J5_Elbow_Wrist_Bracket.stp) | M1-AR-002 | J5 elbow-wrist bracket | 2 | inherited | Arm | Apache-2.0 |
| [`OB_F_J56_Wrist_Housing.stp`](parts/inherited/am-arm200/OB_F_J56_Wrist_Housing.stp) | M1-AR-003 | Wrist housing (J5/J6) | 2 | inherited | Arm | Apache-2.0 |
| [`OB_F_J56_Bottom.stp`](parts/inherited/am-arm200/OB_F_J56_Bottom.stp) | M1-AR-004 | Wrist bottom (J5/J6) | 2 | inherited | Arm | Apache-2.0 |
| [`OB_F_J56_Top.stp`](parts/inherited/am-arm200/OB_F_J56_Top.stp) | M1-AR-005 | Wrist top (J5/J6) | 2 | inherited | Arm | Apache-2.0 |
| [`OB_F_J5_Pin_Bearing.stp`](parts/inherited/am-arm200/OB_F_J5_Pin_Bearing.stp) | M1-AR-011 | J5 pin with bearing | 2 | inherited | Arm | Apache-2.0 |
| [`O_F_J1_Base.stp`](parts/inherited/am-arm200/O_F_J1_Base.stp) | M1-SH-005 | J1 base | 2 | inherited | Arm | Apache-2.0 |
| [`OB_F_J3_Shoulder.stp`](parts/inherited/am-arm200/OB_F_J3_Shoulder.stp) | M1-SH-006 | J3 shoulder | 2 | inherited | Arm | Apache-2.0 |
| [`O_F_J2_Rotation.stp`](parts/inherited/am-arm200/O_F_J2_Rotation.stp) | M1-SH-019 | J2 rotation frame | 2 | inherited | Arm | Apache-2.0 |
| [`OB_F_J7_Gripper.stp`](parts/inherited/am-arm200/OB_F_J7_Gripper.stp) | M1-AR-007 | Gripper body (J7) | 2 | inherited | Gripper | Apache-2.0 |
| [`OB_F_J7_Jaw.stp`](parts/inherited/am-arm200/OB_F_J7_Jaw.stp) | M1-AR-008 | Gripper jaw (J7) | 2 | inherited | Gripper | Apache-2.0 |
| [`OB_F_Camera.stp`](parts/inherited/am-arm200/OB_F_Camera.stp) | M1-AR-027 | Wrist camera bracket | 2 | inherited | Wrist camera | Apache-2.0 |
| [`M1-TR-005-camera-mast-bridge.step`](parts/faibotic/M1-TR-005-camera-mast-bridge.step) | M1-TR-005 | Camera mast bridge | 1 | printed | Head camera | CC-BY-NC-SA-4.0 |
| [`M1-TR-006-camera-mast-2020.step`](parts/faibotic/M1-TR-006-camera-mast-2020.step) | M1-TR-006 | Camera mast 2020 | 1 | cut | Head camera | CC-BY-NC-SA-4.0 |
| [`M1-TR-007-pan-tilt-bracket.step`](parts/faibotic/M1-TR-007-pan-tilt-bracket.step) | M1-TR-007 | Pan-tilt bracket | 1 | printed | Head camera | CC-BY-NC-SA-4.0 |
| [`M1-TR-007b-locking-knob-m5.step`](parts/purchased/M1-TR-007b-locking-knob-m5.step) | M1-TR-007b | Locking knob M5 | 2 | purchased | Head camera | CC-BY-NC-SA-4.0 |
| [`M1-CM-001-usb-camera-module.step`](parts/purchased/M1-CM-001-usb-camera-module.step) | M1-CM-001 | USB camera module | 3 | purchased | Cameras | CC-BY-NC-SA-4.0 |
| [`M1-SV-001-sts3215-servo.step`](parts/purchased/M1-SV-001-sts3215-servo.step) | M1-SV-001 | STS3215 servo | 7 | purchased | Actuators | CC-BY-NC-SA-4.0 |
| [`M1-SV-002-sts3095-servo-lift.step`](parts/purchased/M1-SV-002-sts3095-servo-lift.step) | M1-SV-002 | STS3095 servo (lift) | 1 | purchased | Actuators | CC-BY-NC-SA-4.0 |
| [`M1-BS-038-battery-bay-envelope.step`](parts/purchased/M1-BS-038-battery-bay-envelope.step) | M1-BS-038 | Battery bay envelope | 1 | purchased | Power | CC-BY-NC-SA-4.0 |
| [`M1-BS-038-electronics-bay-envelope.step`](parts/purchased/M1-BS-038-electronics-bay-envelope.step) | M1-BS-038 | Electronics bay envelope | 1 | purchased | Electronics | CC-BY-NC-SA-4.0 |
| [`M1-TR-010-d-sub-combo-connector-plug-half.step`](parts/purchased/M1-TR-010-d-sub-combo-connector-plug-half.step) | M1-TR-010 | D-sub combo connector (plug half) | 1 | purchased | Electronics | CC-BY-NC-SA-4.0 |
| [`M1-TR-011-connector-holder.step`](parts/faibotic/M1-TR-011-connector-holder.step) | M1-TR-011 | Connector holder | 1 | printed | Electronics | CC-BY-NC-SA-4.0 |
| [`M1-TR-012-emergency-stop-box.step`](parts/faibotic/M1-TR-012-emergency-stop-box.step) | M1-TR-012 | Emergency stop box | 1 | printed | Electronics | CC-BY-NC-SA-4.0 |
| [`M1-TR-012b-emergency-stop-button.step`](parts/purchased/M1-TR-012b-emergency-stop-button.step) | M1-TR-012b | Emergency stop button | 1 | purchased | Electronics | CC-BY-NC-SA-4.0 |
| [`M1-TR-013-station-electronics-box.step`](parts/faibotic/M1-TR-013-station-electronics-box.step) | M1-TR-013 | Station electronics box | 1 | printed | Electronics | CC-BY-NC-SA-4.0 |
| [`M1-TR-023-d-sub-combo-connector-socket-half.step`](parts/purchased/M1-TR-023-d-sub-combo-connector-socket-half.step) | M1-TR-023 | D-sub combo connector (socket half) | 1 | purchased | Electronics | CC-BY-NC-SA-4.0 |
<!-- parts-table:end -->

The CSV twin of this table, [`parts.csv`](parts.csv), adds origin, source file, material, process, status and notes.

About the part files:

- Faibotic and purchased parts are saved with their origin at the minimum corner of their bounding box, not where they sit in the assembly. Use the four assembly files for placement.
- Each file names its STEP product after its own file name.
- Inherited parts keep the name and the content of the file they came from. The folder has its own [README](parts/inherited/README.md) with the sources.
- Purchased parts are plain solids that show size and mounting points. They are not manufacturer models.
- Material, process and status come from the part register. Where it has no entry, `parts.csv` says "unconfirmed".

## URDF

[`urdf/m1.urdf`](urdf/m1.urdf) describes the robot with three continuous wheel joints, one prismatic lift joint (0 to 0.45 m) and six revolute joints per arm. Keep `urdf/assets/` beside it. Use it for display and kinematics; dynamics are unchecked on hardware. Joint limits and open points are in [urdf/README.md](urdf/README.md). It is a copy of `sim/m1.urdf`, which the Python package in `sdk/faibotic/` generates.

## Regenerating the part files

After a change in `cad/faibotic/` or `sim/`, run on a machine with build123d:

```
python hardware/tools/export_parts.py export
python hardware/tools/export_parts.py tables
```

The first command rebuilds `assembly/` and `parts/` and re-opens every file to compare it with the assembly. The second rewrites `parts.csv`, `guide-part-map.json`, `urdf/` and the table above. `python hardware/tools/export_parts.py --check-only` needs only Python and verifies the result. Details, and the steps after a part is renamed, are in [tools/README.md](tools/README.md).

## Large files

STEP files are stored as ordinary Git objects, without Git LFS. The biggest file is 37 MB, under the 50 MB limit that GitHub warns about, and a plain clone is enough. The assembly is split into four files for the same reason. If a single file passes 50 MB, or this folder passes 200 MB, the plan is to move it into a repository of its own with LFS enabled from the first commit.

## License

Copyright (c) 2026 Leo Dao (Faibotic). The full list of files per licence is in [`../NOTICE`](../NOTICE).

- The Faibotic designs in this folder (`assembly/`, `parts/faibotic/`, `parts/purchased/`, `parts.csv`, `guide-part-map.json`, this README) are licensed under [CC BY-NC-SA 4.0](LICENSE). For commercial use, contact founder@faibotic.com.
- `parts/inherited/` holds Apache License 2.0 material from other authors. It stays under that licence; the sources are listed in its README and in [`../NOTICE`](../NOTICE).
- The assembly files in `assembly/` contain unmodified third-party Apache parts listed in [`../NOTICE`](../NOTICE); those parts remain under the Apache License 2.0.
- `urdf/` and `tools/` (including their README files) fall under the Apache License 2.0, text in [urdf/LICENSE](urdf/LICENSE). The meshes in `urdf/assets/` carry their own licence per file, listed in [`../NOTICE`](../NOTICE).

The models in `parts/purchased/` are simplified envelopes generated by Faibotic from datasheet dimensions (`cad/faibotic/`). They stand in for bought components so that nothing is missing from the robot; each product is still the design of its maker.
