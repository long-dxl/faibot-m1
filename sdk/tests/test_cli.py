# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
import json

import pytest

from faibotic import cli, spec
from faibotic.feetech import protocol as P
from faibotic.feetech.bus import FeetechBus
from faibotic.feetech.mockbus import FakeServoPort


def run(capsys, *argv):
    code = cli.main(list(argv))
    return code, capsys.readouterr().out


def test_doctor_exits_zero_and_reports_json(capsys):
    code, out = run(capsys, "doctor", "--json")
    data = json.loads(out)
    assert code == 0 and data["ok"] is True
    names = {c["check"] for c in data["checks"]}
    assert {"python", "numpy", "mujoco", "simulation model"} <= names


def test_model_lists_all_18_servos(capsys):
    code, out = run(capsys, "model", "--json")
    rows = json.loads(out)["joints"]
    assert code == 0
    assert sorted(r["servo_id"] for r in rows) == list(range(1, 17)) + [21, 22]   # yaw 21, 22
    assert {r["joint"]: r["servo_id"] for r in rows} == spec.SERVO_IDS
    arm = next(r for r in rows if r["joint"] == "left_shoulder")
    assert arm["min"] == pytest.approx(-80.0) and arm["unit"] == "deg"


def test_ports_runs_without_hardware(capsys):
    code, out = run(capsys, "ports", "--json")
    assert code == 0 and isinstance(json.loads(out)["ports"], list)


def test_scan_mock_finds_every_servo(capsys):
    code, out = run(capsys, "scan", "--port", "mock", "--json")
    data = json.loads(out)
    assert code == 0 and data["missing"] == [] and len(data["servos"]) == 18


def test_status_mock_reads_values(capsys):
    code, out = run(capsys, "status", "--port", "mock", "--json")
    row = json.loads(out)["servos"][0]
    assert code == 0
    assert row["position_steps"] == 2048 and row["voltage_v"] == 12.0 and row["temperature_c"] == 30


def test_scan_reports_missing_servo_with_exit_1(monkeypatch, capsys):
    port = FakeServoPort(list(spec.SERVO_IDS.values()))
    port.dead.add(5)
    monkeypatch.setattr(cli, "open_bus", lambda p: FeetechBus(port, retries=0))
    code, out = run(capsys, "scan", "--port", "x", "--json")
    assert code == 1 and json.loads(out)["missing"] == ["left_wrist_roll"]


def test_scan_and_status_never_enable_torque(monkeypatch, capsys):
    port = FakeServoPort(list(spec.SERVO_IDS.values()))
    monkeypatch.setattr(cli, "open_bus", lambda p: FeetechBus(port))
    run(capsys, "scan", "--port", "x")
    run(capsys, "status", "--port", "x")
    instructions = {pkt[4] for pkt in port.log}
    assert instructions <= {P.PING, P.READ}, "scan/status may only send PING and READ"
    assert all(m[P.TORQUE_ENABLE[0]] == 0 for m in port.mem.values())


def test_unopenable_port_exits_2(capsys):
    code = cli.main(["scan", "--port", "/dev/does-not-exist"])
    assert code == 2 and "cannot open" in capsys.readouterr().err


def test_unknown_command_is_a_usage_error():
    with pytest.raises(SystemExit) as e:
        cli.main(["fly"])
    assert e.value.code == 2


def test_lab_requires_known_task():
    with pytest.raises(SystemExit):
        cli.main(["lab", "dance"])
    assert set(cli.LAB) == {"pick", "sort", "open"}


def test_script_commands_pass_options_through(monkeypatch):
    seen = []
    monkeypatch.setattr(cli.subprocess, "call", lambda cmd: seen.append(cmd) or 0)
    assert cli.main(["lab", "pick", "--seed", "3"]) == 0
    assert cli.main(["selftest"]) == 0
    assert seen[0][1].endswith("lab_pick_place.py") and seen[0][2:] == ["--seed", "3"]
    assert seen[1][1].endswith("selftest.py")
