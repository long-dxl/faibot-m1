"""Khay đế là sàn thiết bị (laptop HOẶC Jetson + phụ kiện), khai báo qua faibotic.KhayTai.

Kiểm: API khai báo (vùng cho phép, định mức, mã CFG-02), giới hạn mềm lift, vùng cấm tâm kẹp theo đỉnh vật, và chống lật với tay theo CAD
ở mọi góc vùng tâm cho phép: tĩnh và động (biên dốc đế mặc định) >= spec.BASE_TIP_MIN, không cần siết thêm dốc đế hay thêm đối trọng.
Mô hình lật: tests/tipover_dong.py worst_khay (khối lượng khay đế theo CAD: 4 mảnh M1-ST-012, cột đỡ, thanh 2040; không còn tải 1,3 kg ở góc xa).
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))
sys.path.insert(0, str(ROOT / "tests"))
from faibotic import KhayTai, Robot, spec  # noqa: E402
from faibotic.khay import MAT_KHAY  # noqa: E402
from faibotic.loi import CauHinhSai  # noqa: E402

MASSES = ROOT / "cad" / "export" / "M1-PL-masses.json"


# ---------- API ----------
@pytest.mark.parametrize("loai,kg", [("laptop", 2.5), ("jetson-nano", 0.75), ("jetson-agx", 1.25)])
def test_mac_dinh_theo_loai(loai, kg):
    k = KhayTai(loai)
    assert k.kg == pytest.approx(kg) and k.cao > 0 and k.lift_min >= spec.KHAY_DE_LIFT_MIN


@pytest.mark.parametrize("kw", [dict(x=0.21), dict(x=0.07), dict(y=0.05), dict(y=-0.05), dict(kg=3.01), dict(kg=0.0),
                                dict(kg=float("nan")), dict(x=float("inf")), dict(loai="ghe"), dict(loai="khac", kg=1.0),
                                dict(cao=0.5), dict(kg=True)])
def test_ngoai_vung_hoac_dinh_muc_bi_tu_choi(kw):
    with pytest.raises(CauHinhSai) as e:
        KhayTai(**{"loai": "laptop", **kw})
    assert e.value.ma == "CFG-02" and isinstance(e.value, ValueError)


def test_robot_khai_bao_va_bo():
    r = Robot("sim")
    assert r.khay is None and r.lift_min() == spec.LIFT_RANGE[0]
    r.set_khay_tai(KhayTai("laptop", kg=2.0, x=0.19))
    assert r.lift_min() == pytest.approx(spec.KHAY_DE_LIFT_MIN)
    with pytest.raises(TypeError):
        r.set_khay_tai({"loai": "laptop"})
    r.set_khay_tai(None)
    assert r.lift_min() == spec.LIFT_RANGE[0]


def test_lift_khong_xuong_duoi_san_khi_co_vat():
    r = Robot("sim", khay=KhayTai("laptop"))
    r.move_joints({"lift": 0.10}, 0.3)
    r.move_joints({"lift": 0.0}, 0.3)
    assert r._targets["lift"] == pytest.approx(spec.KHAY_DE_LIFT_MIN, abs=1e-9)
    r.set_khay_tai(None)
    r.move_joints({"lift": 0.0}, 0.3)
    assert r._targets["lift"] == pytest.approx(0.0, abs=1e-9)


def test_lift_dang_thap_chi_duoc_nang():
    """Khai báo lúc lift dưới sàn: không bị kéo lên tự động, không được hạ thêm, nâng thì được."""
    r = Robot("sim")
    r.move_joints({"lift": 0.01}, 0.2)
    r.set_khay_tai(KhayTai("laptop"))
    r.move_joints({"lift": 0.0}, 0.2)
    assert r._targets["lift"] == pytest.approx(0.01, abs=1e-6)
    r.move_joints({"lift": 0.2}, 0.3)
    assert r._targets["lift"] == pytest.approx(0.2, abs=1e-6)


def test_laptop_mo_nap_can_lift_cao_hon():
    k = KhayTai("laptop", cao=0.26)
    assert k.lift_min > 0.2 and k.lift_min == pytest.approx(MAT_KHAY + 0.26 + spec.KHAY_DE_KHE - (spec.ARM_BASE_Z0 + spec.TMID_Z[0]))


def test_vung_cam_tam_kep_theo_dinh_vat():
    """Tâm kẹp ở trên khay, cao hơn mặt khay + 3 cm nhưng thấp hơn đỉnh laptop mở nắp + 3 cm: chỉ vi phạm khi đã khai báo."""
    from faibotic import limits
    k = KhayTai("laptop", cao=0.26)
    lift = 0.10
    r = Robot("sim")
    z_tcp = MAT_KHAY + 0.008 + 0.06          # khung robot: trên mặt khay + 6 cm, dưới đỉnh vật
    q = r.solve_ik("left", [0.24, 0.12, z_tcp], lift=lift)
    assert q is not None
    q = list(q)
    assert not [b for b in limits.keepout_violations("left", q, "di_dong", lift) if "khay" in b or "vật" in b]
    bad = limits.keepout_violations("left", q, "di_dong", lift, k.dinh_z)
    assert any("đỉnh vật" in b for b in bad)


# ---------- chống lật (tay theo CAD) ----------
def _td():
    if not MASSES.exists():
        pytest.skip("chưa sinh M1-PL-masses.json")
    import tipover_dong as TD
    return TD


def _items(k: KhayTai):
    return [(k.kg, k.x, k.y, MAT_KHAY + k.cao / 2)]


CORNERS = [(x, y) for x in spec.KHAY_DE_TAM_X for y in (spec.KHAY_DE_TAM_Y, -spec.KHAY_DE_TAM_Y)]


def test_khay_trong_tay_cad_dat():
    TD = _td()
    assert TD.worst_khay() >= spec.BASE_TIP_MIN
    assert TD.envelope_khay() >= spec.BASE_TIP_MIN


@pytest.mark.parametrize("loai", ["laptop", "jetson-agx"])
@pytest.mark.parametrize("x,y", CORNERS)
def test_vat_khai_bao_trong_vung_giu_1_5(loai, x, y):
    """Laptop 2,5 kg và Jetson AGX ở mọi góc vùng tâm cho phép, lift đỉnh và lift thấp: tĩnh và động >= 1,5."""
    TD = _td()
    it = _items(KhayTai(loai, x=x, y=y))
    for lt in (True, False):
        assert TD.worst_khay(it, lift_top=lt, pan_step=20.0) >= spec.BASE_TIP_MIN
        assert TD.envelope_khay(it, lift_top=lt, pan_step=20.0) >= spec.BASE_TIP_MIN


def test_tai_dinh_muc_3kg_goc_xau_nhat():
    TD = _td()
    k = KhayTai("khac", kg=spec.KHAY_DE_TAI_MAX, cao=0.04, x=spec.KHAY_DE_TAM_X[1], y=spec.KHAY_DE_TAM_Y)
    assert TD.envelope_khay(_items(k)) >= spec.BASE_TIP_MIN


def test_vat_ngoai_vung_lam_tut_duoi_1_5():
    """Vì sao phải chặn vùng: laptop 2,5 kg lệch trước + bên (x 0,27, y 0,08) hoặc 1 kg dồn góc trước khay thì động < 1,5."""
    TD = _td()
    assert TD.envelope_khay([(2.5, 0.27, 0.08, MAT_KHAY + 0.0125)]) < spec.BASE_TIP_MIN
    assert TD.envelope_khay([(1.0, 0.30, 0.15, MAT_KHAY + 0.01)]) < spec.BASE_TIP_MIN

