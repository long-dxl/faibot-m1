import re
import sys
from pathlib import Path

import pytest

import faibotic
import sinh_bang_khop

SDK = Path(__file__).resolve().parents[1]
REPO = SDK.parent


def pyproject() -> str:
    return (SDK / "pyproject.toml").read_text(encoding="utf-8")


def test_distribution_name_script_and_version():
    text = pyproject()
    assert re.search(r'^name = "faibot-m1"', text, re.M)
    assert 'faibot-m1 = "faibotic.cli:main"' in text
    assert re.search(r'^version = "(.+)"', text, re.M).group(1) == faibotic.__version__
    assert f"## {faibotic.__version__}" in (SDK / "CHANGELOG.md").read_text(encoding="utf-8")


def test_extras_and_python_floor():
    text = pyproject()
    for extra in ("dev", "hw", "cameras"):
        assert re.search(rf"^{extra} = \[", text, re.M)
    assert 'requires-python = ">=3.10"' in text
    assert sys.version_info >= (3, 10)


def test_license_files_in_place():
    assert "Apache License" in (SDK / "LICENSE").read_text(encoding="utf-8")[:200]
    assert (SDK / "NOTICE").read_text(encoding="utf-8").strip()


def test_readme_joint_table_is_current():
    text = (SDK / "README.md").read_text(encoding="utf-8")
    assert sinh_bang_khop.render(text) == text, "run: python3 sdk/tools/sinh_bang_khop.py --ghi"


def test_readme_sections_in_order():
    heads = re.findall(r"^#{2,3} (.+)$", (SDK / "README.md").read_text(encoding="utf-8"), re.M)
    wanted = ["Install", "First run", "Commands", "Python", "Your own teleoperation loop", "Tool poses from a policy",
              "Cameras", "Safety", "Hardware", "Development"]
    idx = [heads.index(w) for w in wanted]
    assert idx == sorted(idx)


def test_readme_commands_exist():
    from faibotic import cli
    text = (SDK / "README.md").read_text(encoding="utf-8")
    parser = cli.build_parser()
    names = set(parser._subparsers._group_actions[0].choices)
    used = set(re.findall(r"faibot-m1 (\w[\w-]*)", text))
    assert used <= names | {"doctor"}, used - names


def test_repo_layout_the_editable_install_relies_on():
    assert (REPO / "sim" / "assets").is_dir() and (REPO / "demos" / "selftest.py").is_file()
