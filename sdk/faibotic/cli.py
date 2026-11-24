"""Command line for the Faibot M1 SDK: `faibot-m1 <command>`.

Look (touches nothing):      doctor, ports, model
Read (never enables torque): scan, status
Simulate (no hardware):      sim, selftest, demo, lab pick|sort|open

`scan` and `status` take `--port mock` to run against a simulated servo bus.
Every command that prints a report takes `--json`.
"""
from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import platform
import subprocess
import sys
from pathlib import Path

from . import __version__, spec

REPO = Path(__file__).resolve().parents[2]     # editable install: sdk/faibotic/cli.py -> repo root
LAB = {"pick": "lab_pick_place.py", "sort": "lab_sorting.py", "open": "lab_open_close.py"}
MIN_PYTHON = (3, 10)


def _emit(data, as_json: bool, lines: list[str]) -> None:
    print(json.dumps(data, indent=2) if as_json else "\n".join(lines))


# ---------------------------------------------------------------- doctor
def _check(name: str, ok: bool, detail: str, required: bool = True) -> dict:
    return {"check": name, "ok": bool(ok), "required": required, "detail": detail}


def run_doctor() -> list[dict]:
    out = []
    out.append(_check("python", sys.version_info >= MIN_PYTHON,
                      f"{platform.python_version()} (need {MIN_PYTHON[0]}.{MIN_PYTHON[1]} or newer)"))
    for mod, required in (("numpy", True), ("mujoco", True), ("serial", True), ("cv2", False), ("gpiod", False)):
        try:
            m = importlib.import_module(mod)
            out.append(_check(mod, True, getattr(m, "__version__", getattr(m, "VERSION", "installed")), required))
        except ImportError:
            hint = {"cv2": "optional: pip install -e 'sdk[cameras]'", "gpiod": "optional: pip install -e 'sdk[hw]' (on a Pi)"}.get(mod, "missing")
            out.append(_check(mod, False, hint, required))
    assets = REPO / "sim" / "assets"
    out.append(_check("repository checkout", assets.is_dir() and (REPO / "demos").is_dir(),
                      f"{REPO} (the SDK reads meshes from sim/assets, so install with `pip install -e`)"))
    try:
        import mujoco
        from . import model
        mujoco.MjModel.from_xml_string(model.robot_mjcf(True))
        out.append(_check("simulation model", True, "builds"))
    except Exception as e:  # noqa: BLE001 - report, do not crash the doctor
        out.append(_check("simulation model", False, f"{type(e).__name__}: {e}"))
    if sys.platform.startswith("linux"):
        try:
            import grp
            groups = {grp.getgrgid(g).gr_name for g in os.getgroups()}
            ok = bool(groups & {"dialout", "uucp"})
            out.append(_check("serial permission", ok, "user is in dialout" if ok else
                              "user is not in the dialout group: sudo usermod -aG dialout $USER, then log in again", required=False))
        except (ImportError, KeyError):
            pass
    return out


def cmd_doctor(args) -> int:
    res = run_doctor()
    lines = [f"faibot-m1 {__version__}"]
    for r in res:
        mark = "ok  " if r["ok"] else ("FAIL" if r["required"] else "warn")
        lines.append(f"  {mark} {r['check']}: {r['detail']}")
    bad = [r for r in res if r["required"] and not r["ok"]]
    lines.append("doctor: all required checks passed" if not bad else f"doctor: {len(bad)} required check(s) failed")
    _emit({"version": __version__, "checks": res, "ok": not bad}, args.json, lines)
    return 1 if bad else 0


# ---------------------------------------------------------------- ports
def list_serial_ports() -> list[dict]:
    from serial.tools import list_ports
    rows = []
    for p in sorted(list_ports.comports(), key=lambda p: p.device):
        if p.hwid == "n/a":       # built-in UART with nothing behind it (Linux /dev/ttyS*)
            continue
        vidpid = f"{p.vid:04x}:{p.pid:04x}" if p.vid is not None and p.pid is not None else ""
        rows.append({"device": p.device, "description": p.description, "vid_pid": vidpid})
    return rows


def cmd_ports(args) -> int:
    rows = list_serial_ports()
    lines = [f"{r['device']:<20} {r['vid_pid']:<10} {r['description']}" for r in rows] or ["no serial ports found"]
    _emit({"ports": rows}, args.json, lines)
    return 0


# ---------------------------------------------------------------- model
def joint_table() -> list[dict]:
    rows = []
    for name in spec.POS_NAMES:
        lo, hi = spec.limits(name)
        unit = "mm" if name == "lift" else ("%" if name.endswith("gripper") else "deg")
        k = 1000.0 if name == "lift" else (1.0 if unit == "%" else 180.0 / math.pi)
        rows.append({"joint": name, "servo_id": spec.SERVO_IDS[name], "servo": spec.PL_SERVO_MODELS[name].model.split()[0],
                     "bus": spec.SERVO_BUS[name], "min": round(lo * k, 1), "max": round(hi * k, 1), "unit": unit})
    for name in ("wheel_0", "wheel_1", "wheel_2"):
        rows.append({"joint": name, "servo_id": spec.SERVO_IDS[name], "servo": spec.PL_SERVO_MODELS[name].model.split()[0],
                     "bus": spec.SERVO_BUS[name], "min": None, "max": None, "unit": "continuous"})
    return rows


def cmd_model(args) -> int:
    rows = joint_table()
    lines = [f"Faibot M1, arm variant {spec.ARM_VARIANT}: {len(rows)} servos"]
    for r in rows:
        lim = "continuous" if r["min"] is None else f"{r['min']:g} .. {r['max']:g} {r['unit']}"
        lines.append(f"  {r['joint']:<18} id {r['servo_id']:<3} {r['bus']}  {r['servo']:<22} {lim}")
    _emit({"variant": spec.ARM_VARIANT, "joints": rows}, args.json, lines)
    return 0


# ---------------------------------------------------------------- scan, status (read only)
def open_bus(port: str):
    """Servo bus for `port`; 'mock' is a simulated bus with every servo answering."""
    from .feetech.bus import FeetechBus
    if port == "mock":
        from .feetech.mockbus import FakeServoPort
        return FeetechBus(FakeServoPort(list(spec.SERVO_IDS.values())))
    return FeetechBus(port)


def _load_pct(raw: int) -> float:
    return round(100.0 * (raw & 0x3FF) / 1000.0 * (-1 if raw & (1 << 10) else 1), 1)


def read_servos(bus, full: bool) -> list[dict]:
    """Ping every servo of the robot. With full=True also read position, load, voltage, temperature.
    Only READ and PING instructions are sent: torque is never switched on."""
    from .feetech import protocol as P
    rows = []
    for name, sid in spec.SERVO_IDS.items():
        row = {"joint": name, "servo_id": sid, "found": bus.ping(sid)}
        if row["found"] and full:
            try:
                raw = bus.read(sid, P.PRESENT_POSITION)
                row["position_steps"] = raw
                row["position_deg"] = round(raw * 360.0 / P.STEPS_PER_REV, 1)
                row["load_pct"] = _load_pct(bus.read(sid, P.PRESENT_LOAD))
                row["voltage_v"] = bus.read(sid, P.PRESENT_VOLTAGE) / 10.0
                row["temperature_c"] = bus.read(sid, P.PRESENT_TEMPERATURE)
            except (TimeoutError, P.ProtocolError) as e:
                row["error"] = str(e)
        rows.append(row)
    return rows


def _servo_cmd(args, full: bool) -> int:
    try:
        bus = open_bus(args.port)
    except Exception as e:  # noqa: BLE001 - port busy, missing or no permission
        print(f"cannot open {args.port}: {e}", file=sys.stderr)
        return 2
    try:
        rows = read_servos(bus, full)
    finally:
        bus.close()
    missing = [r["joint"] for r in rows if not r["found"]]
    lines = []
    for r in rows:
        if not r["found"]:
            lines.append(f"  {r['joint']:<18} id {r['servo_id']:<3} no answer")
        elif full and "error" not in r:
            lines.append(f"  {r['joint']:<18} id {r['servo_id']:<3} {r['position_deg']:>6.1f} deg  load {r['load_pct']:>5.1f} %  "
                         f"{r['voltage_v']:.1f} V  {r['temperature_c']} C")
        else:
            lines.append(f"  {r['joint']:<18} id {r['servo_id']:<3} found" + (f" ({r['error']})" if "error" in r else ""))
    lines.append(f"{len(rows) - len(missing)} of {len(rows)} servos answered" + (f"; missing: {', '.join(missing)}" if missing else ""))
    _emit({"port": args.port, "servos": rows, "missing": missing}, args.json, lines)
    return 1 if missing else 0


def cmd_scan(args) -> int:
    return _servo_cmd(args, full=False)


def cmd_status(args) -> int:
    return _servo_cmd(args, full=True)


# ---------------------------------------------------------------- sim, selftest, demo, lab
def _run_script(script: str, extra: list[str]) -> int:
    path = REPO / "demos" / script
    if not path.is_file():
        print(f"{path} not found: run from a repository checkout installed with `pip install -e \"sdk[dev]\"`", file=sys.stderr)
        return 2
    return subprocess.call([sys.executable, str(path), *extra])


def cmd_sim(args) -> int:
    return _run_script("teleop.py", args.rest)


def cmd_selftest(args) -> int:
    return _run_script("selftest.py", args.rest)


def cmd_demo(args) -> int:
    return _run_script("check_demo.py", args.rest)


def cmd_lab(args) -> int:
    return _run_script(LAB[args.task], args.rest)


# ---------------------------------------------------------------- parser
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="faibot-m1", description="Faibot M1 SDK command line.")
    p.add_argument("--version", action="version", version=f"faibot-m1 {__version__}")
    sub = p.add_subparsers(dest="command", required=True, metavar="command")

    def add(name, fn, help_, json_=False, port=False, rest=False, task=False):
        s = sub.add_parser(name, help=help_, description=help_)
        s.set_defaults(fn=fn)
        if task:
            s.add_argument("task", choices=sorted(LAB), help="pick, sort or open")
        if json_:
            s.add_argument("--json", action="store_true", help="print JSON")
        if port:
            s.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyUSB0 or COM3; 'mock' = simulated bus")
        if rest:
            s.epilog = "Any further options go to the script unchanged (try `--help` after the command name once, or read demos/)."
        return s

    add("doctor", cmd_doctor, "Check Python, packages and the simulation model. Talks to no servo.", json_=True)
    add("ports", cmd_ports, "List serial ports.", json_=True)
    add("model", cmd_model, "Print the joint table: servo ID, model, bus and limits.", json_=True)
    add("scan", cmd_scan, "Ping every servo ID. Read only.", json_=True, port=True)
    add("status", cmd_status, "Read position, load, voltage and temperature of every servo. Read only.", json_=True, port=True)
    add("sim", cmd_sim, "Open the simulation window with keyboard teleoperation.", rest=True)
    add("selftest", cmd_selftest, "Headless check of model, teleop and IK; exit 0 = pass.", rest=True)
    add("demo", cmd_demo, "Pick-and-place demo with coordinate checks; prints JSON.", rest=True)
    add("lab", cmd_lab, "Run a lab scene task in the simulation: lab pick|sort|open.", rest=True, task=True)
    return p


SCRIPT_COMMANDS = {"sim": cmd_sim, "selftest": cmd_selftest, "demo": cmd_demo}


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    # Script commands hand every later option to the demo script, so argparse must not see them.
    if argv and argv[0] in SCRIPT_COMMANDS and "-h" not in argv[1:2] and "--help" not in argv[1:2]:
        return int(SCRIPT_COMMANDS[argv[0]](argparse.Namespace(rest=argv[1:])))
    if len(argv) > 1 and argv[0] == "lab" and argv[1] in LAB:
        return int(cmd_lab(argparse.Namespace(task=argv[1], rest=argv[2:])))
    args = build_parser().parse_args(argv)
    return int(args.fn(args))


if __name__ == "__main__":
    raise SystemExit(main())
