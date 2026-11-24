"""Chống lật (cad/faibotic/tipover.py): tái lập bảng cũ 1,03 và bảng R1b/R2 cho đế v0.5 (R0,25 m, một bánh ra trước).
Số đích lấy nguyên từ docs (không chỉnh mô hình cho khớp); dung sai ±0,03 theo yêu cầu G2."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cad" / "faibotic"))
import tipover as T  # noqa: E402


def test_old_table_1_03():
    r = T.old_table()
    assert r["lat"] == pytest.approx(1.81, abs=0.01) and r["giu"] == pytest.approx(1.85, abs=0.01)
    assert r["he_so"] == pytest.approx(1.03, abs=0.01)


@pytest.mark.parametrize("cw,expect", [(0, (1.16, 1.37, 1.61)), (3, (1.57, 1.77, 2.01)), (5, (1.83, 2.04, 2.28))])
def test_r1b_counterweight_table(cw, expect):
    """R1b mục 4.3: tay 1,30 kg, không khay."""
    got = [T.r1b_min(b, cw, False, 1.30) for b in ("B1", "B2", "B3")]
    assert got == pytest.approx(list(expect), abs=0.03)


def test_r1b_front_and_side_b2():
    """R1b mục 4.2, B2: trước 3,17, nhỏ nhất 1,37."""
    c = T.r1b_cfg("B2")
    E = T.edges(0.25, (0, 120, 240))
    arms = T.arm_points(c["pan_xy"], 1, 0, 1.30, 0.182, 1.0, 0.584) + T.arm_points(c["pan_xy"], -1, 0, 1.30, 0.182, 1.0, 0.584)
    assert T.factor(c["fixed"] + arms, E)[0] == pytest.approx(3.17, abs=0.03)
    assert T.r1b_min("B2") == pytest.approx(1.37, abs=0.03)


def test_r1b_tray_tables():
    """R1b mục 5.2 (tay 1,30): khay 1 kg góc xa, đối trọng 0/3/5 kg."""
    for cw, expect in ((0, (1.01, 1.19, 1.40)), (3, (1.36, 1.54, 1.75)), (5, (1.59, 1.77, 1.98))):
        assert [T.r1b_min(b, cw, True, 1.30) for b in ("B1", "B2", "B3")] == pytest.approx(list(expect), abs=0.03)


def test_r2_paired_arm_tray_cw5():
    """R2 mục 3.3: tay ghép đôi 1,75 kg, khay 1 kg góc xa, đối trọng 5 kg: 1,47 / 1,65 / 1,86 (R2 suy bằng 'R1b - 0,12').
    Chạy thật: 1,499 / 1,666 / 1,862 (B1 sát biên dung sai). Không khay, đối trọng 3 kg: 1,47 / 1,66 / 1,88 (R1b)."""
    got = [T.r1b_min(b, 5, True, 1.75) for b in ("B1", "B2", "B3")]
    assert got == pytest.approx([1.47, 1.65, 1.86], abs=0.03)
    assert [T.r1b_min(b, 3, False, 1.75) for b in ("B1", "B2", "B3")] == pytest.approx([1.47, 1.66, 1.88], abs=0.03)


def test_pl_arm_from_upstream_urdf():
    """P-L: tay AM-ARM200 theo URDF upstream (bỏ link camera giữ chỗ) 1,27 kg + camera cổ tay 0,03; duỗi ngang: đầu kẹp 0,58 m trước trục pan (R1b 0,584)."""
    a = T.arm_pl()
    assert a["m"] == pytest.approx(1.30, abs=0.01)
    assert a["tcp"] == pytest.approx(0.584, abs=0.01)
    assert 0.17 <= a["r"] <= 0.19


def test_pl_cad_masses_report():
    """Khối lượng lấy từ CAD P-L (cad/export/M1-PL-masses.json, sinh bởi assembly_pl.py --masses). Kiểm số tính lại khớp số đã lưu;
    ngưỡng 1,5 là điều kiện mở của đánh giá thân đế, không ép trong test."""
    p = ROOT / "cad" / "export" / "M1-PL-masses.json"
    if not p.exists():
        pytest.skip("chưa sinh M1-PL-masses.json")
    M = json.loads(p.read_text())
    saved = M["he_so_lat"]
    assert T.pl_min(M)[0] == pytest.approx(saved["co_khay_1kg"], abs=1e-3)
    assert T.pl_min(M, tray_load=False)[0] == pytest.approx(saved["khong_tai_khay"], abs=1e-3)
    need = saved["doi_trong_can_cho_1_5_kg"]
    if need is not None:
        assert T.pl_min(M, cw=need + 0.01)[0] >= 1.5 - 1e-3


def test_di_dong_with_station_meets_1_5():
    """S2 (G1b): chế độ di động có trạm hai tay (vai 0,475 m, đối trọng giữ 8,23 kg) phải đạt >= 1,5 với khay 1 kg góc xa, tay URDF."""
    p = ROOT / "cad" / "export" / "M1-PL-masses.json"
    if not p.exists():
        pytest.skip("chưa sinh M1-PL-masses.json")
    M = json.loads(p.read_text())
    assert "tram_hai_tay" in M["co_dinh"] and "xe_nang_khung_T" not in M["co_dinh"]
    assert M["pan_xy"][1] == pytest.approx(0.2375, abs=1e-4)
    assert 8.0 <= M["co_dinh"][M["doi_trong_key"]][0] <= 8.5
    assert T.pl_min(M)[0] >= 1.5


def test_dynamic_tipover_ramp():
    """(E2, nghiệm thu T1.08): chống lật động >= 1,5 ở biên dốc đế SDK (BASE_MAX_ACC + |v|·|ω|, BASE_MAX_ALPHA, BASE_MAX_W_TIP),
    ca xấu nhất: lift đỉnh, khay 1,3 kg góc xa, hai tay duỗi tải 1 kg/tay, khối tay cận trên spec + dự phòng servo cổ tay.
    Mô hình: tests/tipover_dong.py (khối của tipover.py + gia tốc từng khối a + α×p − ω²p). Chi tiết dốc: tests/test_base_ramp.py."""
    sys.path.insert(0, str(ROOT / "sdk"))
    import tipover_dong as TD
    from faibotic import spec
    assert TD.envelope() >= spec.BASE_TIP_MIN
    assert TD.worst(0.5, 0, 0, spec.BASE_TIP_ARM_EXTRA_KG) < spec.BASE_TIP_MIN     # giả định cũ 0,5 m/s² không đạt: dốc là cần


def test_cad_arm_mass_matches_spec_d62():
    """khối lượng tay 6+1 xuất từ CAD (assembly_pl.py --masses, tay_cad: STEP + 7 servo) khớp spec.ARM_MASS_CAD, ROBOT_MASS_CAD;
    servo yaw không bị tính vào thân (tong_khong_tay giữ 16,91)."""
    p = ROOT / "cad" / "export" / "M1-PL-masses.json"
    if not p.exists():
        pytest.skip("chưa sinh M1-PL-masses.json")
    sys.path.insert(0, str(ROOT / "sdk"))
    from faibotic import spec
    M = json.loads(p.read_text())
    assert M["tay_cad"]["kg"] == pytest.approx(spec.ARM_MASS_CAD, abs=0.005)
    assert M["tay_cad"]["r_m"] == pytest.approx(spec.ARM_COM_R_CAD, abs=0.002)
    assert M["tong_robot_kg"] == pytest.approx(spec.ROBOT_MASS_CAD, abs=0.01)
    assert not any("wrist_yaw" in k for k in M["chi_tiet"])
    assert {r["ten"] for r in M["tay_cad"]["chi_tiet"]} >= {"servo yaw STS3215", "servo cuộn STS3215", "OB_F_J56_Wrist_Housing"}
