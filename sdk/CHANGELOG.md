# Changelog

## Unreleased

- Tier 1b robustness (decision): every error the SDK raises now derives from `faibotic.loi.M1Loi` with an error code `.ma` (for example `BUS-01`) while keeping its usual base class. New errors: `BusMat`, `BusCauHinhSai`, `ServoMat`, `ServoBaoLoi`, `DienApThap`, `RobotDaDong`, `RobotBan`, `WatchdogHet`, `LenhKhongHopLe`, `KieuLenhSai`, `KhopKhongCo`, `HieuChuanSai`, `CauHinhSai`.
- Feetech backend: one sync read per `step()` watches every servo (error bits, lost replies for 3 cycles, bus silence, supply voltage against `spec.V_CANH`/`spec.V_CAT`) and goes to the safe state; a lost serial port raises `BusMat` and cuts the E-stop latch; a watchdog in the latch thread stops the wheels when `step()` is not called for `spec.WATCHDOG_S`; reads are capped at `spec.BUS_READ_DEADLINE_S`; calibration, ID table and servo model are checked at connect; the port is opened exclusively.
- `Robot`: inputs are validated before any state changes (NaN/inf, types, joint names, `step(dt)`, durations); keep-out zones are checked on the pose actually sent and along the whole `move_tcp` path; one owning thread per robot; commands after `close()` raise `RobotDaDong`; `step()` raises `EStopActive` while the latch is not running; `move_base` ends with `stop_base()`.

- Base motion limits for dynamic tip-over (decision): the translational speed limit `BASE_MAX_V` now applies to the magnitude of `(vx, vy)` (diagonal commands are scaled, keeping direction); base velocity follows an acceleration ramp (`limits.BaseRamp`: 0.05 m/s², 0.15 rad/s²; yaw rate ≤ 0.25 rad/s and |v|·|ω| ≤ 0.01 m/s²) in both the sim and Feetech backends; the Feetech backend also writes the wheel servos' ACCELERATION register on `enable()`. `Robot.move_base` lengthens its duration to stay inside the limits, and `Robot.stop_base()` / `close()` ramp the base down.

## 0.5.0

- First packaged release: `pip install -e "sdk[dev]"` from the repository root installs the `faibotic` package and the `faibot-m1` command.
- New `faibot-m1` command: `doctor`, `ports`, `model`, `scan`, `status`, `sim`, `selftest`, `demo`, `lab`.
- `scan` and `status` only read from the servo bus and never switch torque on. `--port mock` runs them against a simulated bus.
- Examples in `sdk/examples/` run on the simulation backend.
- `sdk/tests/` covers the command line, the packaging and the examples.
- The joint table in the README is generated from `faibotic/spec.py` by `sdk/tools/sinh_bang_khop.py`.
