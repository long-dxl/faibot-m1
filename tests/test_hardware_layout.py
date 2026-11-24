"""Bố cục hardware/ (đầu ra sinh từ cad/ và sim/): kiểm của hardware/tools/export_parts.py --check-only
và vài kiểm cấu trúc, cộng thử cài lỗi giả để chắc kiểm bắt được lệch. Không cần build123d."""
import csv
import importlib.util
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
HW = ROOT / "hardware"
_spec = importlib.util.spec_from_file_location("export_parts", HW / "tools" / "export_parts.py")
EP = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(EP)


def _rows():
    with open(HW / "parts.csv", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


@pytest.mark.parametrize("ten,ok,ct", [pytest.param(*r, id=r[0]) for r in EP.check()])
def test_kiem_con(ten, ok, ct):
    assert ok, f"{ten}: {ct}"


def test_cau_truc():
    for f in ("README.md", "LICENSE", ".gitattributes", "parts.csv", "guide-part-map.json", "urdf/m1.urdf", "urdf/README.md", "urdf/LICENSE",
              "parts/inherited/README.md", "parts/inherited/LICENSE", "tools/export_parts.py", "tools/part-names.en.csv", "tools/README.md"):
        assert (HW / f).exists(), f
    assert len(list((HW / "assembly").glob("*.step"))) == 4


def test_khong_lfs():
    assert "filter=lfs" not in (HW / ".gitattributes").read_text(encoding="utf-8")


def test_khong_gia_khong_nha_ban():
    cot = set(_rows()[0])
    assert not cot & {"price", "gia", "url", "supplier", "vendor"}  # public-scan: ok


def test_inherited_la_ban_chep():
    for r in _rows():
        if r["type"] == "inherited":
            assert r["license"] == "Apache-2.0"
            assert EP.sha256(HW / r["file"]) == EP.sha256(ROOT / r["source"])


def test_bat_loi_qty(monkeypatch, tmp_path):
    """Đổi qty trong một bản sao parts.csv thì kiểm qty phải báo lỗi."""
    fake = tmp_path / "hardware"
    shutil.copytree(HW, fake, ignore=shutil.ignore_patterns("*.step", "*.stl"))
    rows = _rows()
    rows[0]["qty"] = str(int(rows[0]["qty"]) + 1)
    with open(fake / "parts.csv", "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, EP.CSV_COLS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    monkeypatch.setattr(EP, "HW", fake)
    monkeypatch.setattr(EP, "REPORT", fake / "tools" / "export-report.json")
    bad = [t for t, ok, _ in EP.check() if not ok]
    assert any("qty" in t for t in bad)
