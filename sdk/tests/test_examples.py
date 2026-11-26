# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
import importlib.util
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


def load(name: str):
    spec = importlib.util.spec_from_file_location(name, EXAMPLES / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize("name", ["01_first_moves", "02_teleop_loop", "03_tool_poses"])
def test_example_runs_on_sim_backend(name, capsys):
    assert load(name).main() == 0
    assert capsys.readouterr().out.strip()


def test_every_example_is_tested():
    assert sorted(p.stem for p in EXAMPLES.glob("*.py")) == ["01_first_moves", "02_teleop_loop", "03_tool_poses"]
