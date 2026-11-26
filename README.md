# Faibot M1

An open mobile robot with two arms and a lifting column, built for university robotics labs. This repository holds its CAD, simulation model, Python SDK, wiring diagrams and assembly guide.

![Faibot M1](media/renders-web/v05f/faibot-m1-v05f-hero.webp)

**Status:** design v0.5, before the first engineering build. No physical robot has been assembled yet; everything here is checked in simulation and against the CAD.

## Simulation demos

Three lab tasks, played at 2× speed (60 fps). Each task was measured on 20 randomised seeds before recording.

| Sorting | Pick & place | Opening & closing |
| --- | --- | --- |
| ![Sorting](media/video/m1-sorting-2x.gif) | ![Pick and place](media/video/m1-pick_place-2x.gif) | ![Opening and closing](media/video/m1-open_close-2x.gif) |
| Both arms pick at the same time and the base carries the objects to the tray. A zone lock keeps only one arm inside the tray area at a time. [MP4](media/video/m1-sorting-60fps-2x.mp4) · [WebM](media/video/m1-sorting-60fps-2x.webm) | Objects are picked from the table and placed in a box while the base moves between them. [MP4](media/video/m1-pick_place-60fps-2x.mp4) · [WebM](media/video/m1-pick_place-60fps-2x.webm) | The drawer is pulled open, an object is placed inside and the drawer is closed. At the fridge, the right arm opens the door and the left arm takes the can out. [MP4](media/video/m1-open_close-60fps-2x.mp4) · [WebM](media/video/m1-open_close-60fps-2x.webm) |

## What's in this repository

| Folder | What it holds |
| --- | --- |
| [`hardware/`](hardware) | CAD as STEP: the whole robot, one file per part, a parts list and a URDF |
| [`sdk/`](sdk) | The `faibot-m1` Python package and command line: simulation, servo bus, kinematics, teleoperation |
| [`sim/`](sim) | MuJoCo model and meshes, plus a lab scene for pick-and-place tasks |
| [`electrical/`](electrical) | Wiring tables and cable drawings |
| [`product/`](product) | Specification, part register and assembly notes |
| [`docs/vi/`](docs/vi) | Documentation in Vietnamese: specifications, bill of materials, assembly guide |

## Quick start

```
./run.sh              # simulation window with keyboard teleoperation
./run.sh --selftest   # headless check, exit code 0 means it passed
```

The first run creates a virtual environment and installs the dependencies. Python 3.10 or newer is needed. Command-line details are in [`sdk/README.md`](sdk/README.md).

## Robot at a glance

- Two arms, each with six joints and a gripper (6+1, as in the original AM-ARM200), on a lifting column and a three-wheel omni base.
- 18 servos: 7 STS3095 and 11 STS3215, all on 12 V. Wrist yaw servos use IDs 21 and 22.
- Payload per arm (calculated, not measured): peak 2.0 kg at 0.25 m reach, continuous 0 kg at full reach, recommended 0.05 kg or less.
- Base speed up to 0.40 m/s and turn rate up to 0.25 rad/s, with an acceleration ramp that keeps the tip-over margin above 1.5 (static 1.64, dynamic 1.50).

## Platform

Faibotic is building a browser-based platform for this robot: calibration, teleoperation, dataset recording and policy training in one place. It is in development; follow it at [faibotic.com](https://www.faibotic.com).

## Documentation

The written documentation is currently in Vietnamese, starting at [`docs/vi/README.md`](docs/vi/README.md). The hardware and SDK READMEs are in English.

## License

| What | License |
| --- | --- |
| Software (`sdk/`, `scripts/`, `tests/`, `demos/`, `examples/`), the generated MJCF/URDF, and the meshes derived only from AM-ARM200 and MuJoCo Menagerie | [Apache License 2.0](LICENSE) |
| Parts Faibotic drew, as CAD (`cad/`, `hardware/`), as sim meshes (the base, lift carriage and wrist camera meshes in `sim/assets/`) and in `web-assets/m1.glb`; electrical drawings, documentation, images | [CC BY-NC-SA 4.0](LICENSE-hardware). For commercial use, contact founder@faibotic.com |
| Third-party designs in `cad/upstream/`, `hardware/parts/inherited/`, and the third-party parts inside the files above | Apache License 2.0, by their original authors |

The file-by-file list is in [`NOTICE`](NOTICE). Releases v0.5 and earlier published the Faibotic-drawn meshes under the Apache License 2.0. Copyright (c) 2026 Leo Dao (Faibotic). "Faibotic" and "Faibot" are trade names; no trademark rights are granted.

## Credits

The arm and the base frame build on open designs by liyiteng (AM-ARM and AlohaMini), and the servo mesh comes from MuJoCo Menagerie; sources and commits are listed in [`NOTICE`](NOTICE).
