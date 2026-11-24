"""Kiểm chéo CAD - SDK - sim - web - BOM - sổ mã - điện - tài liệu (scripts/kiem_cheo.py, DoD 1).

Mỗi kiểm của script là một test. Lệch đã biết nhưng chưa sửa được trong phạm vi file này: xfail strict
(sửa xong thì test báo XPASS để gỡ dòng). Thêm vài test cài lỗi giả để chắc kiểm bắt được lệch."""
import copy
import importlib.util
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("kiem_cheo", ROOT / "scripts" / "kiem_cheo.py")
KC = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(KC)

# Lệch thật, chờ sửa ở thư mục khác.
DA_BIET = {
}


@pytest.fixture(scope="module")
def du():
    return KC.Du()


@pytest.fixture(scope="module")
def ket_qua():
    t = time.perf_counter()
    kq = KC.chay()
    return kq, time.perf_counter() - t


def test_chay_nhanh(ket_qua):
    assert ket_qua[1] < 30.0


def test_du_kiem(ket_qua):
    ids = [r["id"] for r in ket_qua[0]]
    assert len(ids) == len(set(ids)) >= 25


@pytest.mark.parametrize("cid", [k[3].__name__ for k in KC.KIEM])
def test_kiem(cid, ket_qua, request):
    if cid in DA_BIET:
        request.applymarker(pytest.mark.xfail(reason=DA_BIET[cid], strict=True))
    r = next(x for x in ket_qua[0] if x["id"] == cid)
    assert r["ket_qua"] == "ĐẠT" or r["ket_qua"].startswith(("CÒN MỞ", "BỎ QUA")), f"{r['kiem']}: {r['chi_tiet']}"


def _sai(du, fn, sua):
    d = copy.deepcopy(du)
    sua(d)
    ok, ct = fn(d)
    return ok, ct


def test_bat_lech_gioi_han_web(du):
    def sua(d):
        d.joints["joints"]["left_elbow"]["limit_soft"][1] += 0.01
    ok, ct = _sai(du, KC.k_web_joints, sua)
    assert ok is False and "left_elbow" in ct


def test_bat_lech_so_servo_bom(du):
    def sua(d):
        next(r for r in d.bom if "STS3215" in r["khoan"] and r["nhom"] == "Servo")["so_luong_1_robot"] = "10"   # số đúng là 11
    ok, ct = _sai(du, KC.k_bom_servo, sua)
    assert ok is False and "STS3215" in ct


def test_bat_ma_trung_va_ma_la(du):
    def sua(d):
        d.reg.append(dict(d.reg[0]))
        d.lap_rap += "\nChi tiết M1-ZZ-999."
    assert _sai(du, KC.k_reg, sua)[0] is False
    ok, ct = _sai(du, KC.k_reg_dung, sua)
    assert ok is False and "M1-ZZ-999" in ct


def test_bat_servo_chung_cau_chi(du):
    def sua(d):
        for r in d.day:
            if r["cau_chi"] == "FL2":
                r["cau_chi"] = "FL1"
    ok, ct = _sai(du, KC.k_dien, sua)
    assert ok is False and "chung cầu chì" in ct


def test_bat_lech_kich_thuoc_cad(du):
    def sua(d):
        d.dims["COLUMN_TOP"] += 0.002
    ok, ct = _sai(du, KC.k_cad_dims, sua)
    assert ok is False and "COLUMN_TOP" in ct


def test_bat_lech_id_tai_lieu(du):
    def sua(d):
        d.lap_rap = d.lap_rap.replace("| lift | STS3095 C002 12 V | 16 |", "| lift | STS3095 C002 12 V | 17 |")
    ok, ct = _sai(du, KC.k_doc_id, sua)
    assert ok is False and "lift" in ct


def test_glb_bbox_doc_duoc(du):
    bb = KC._glb_bbox_theo_cum(du.glb)
    assert {"base", "lift", "arm_L", "gripper_R"} <= set(bb)
    assert bb["base"][2] == pytest.approx(0.0, abs=KC.TOL_BBOX)
