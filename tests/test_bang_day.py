"""Bảng dây từng sợi chuỗi L (electrical/bang-day.csv, vat-tu-day.csv) khớp day_L.py và tinh_toan.py."""
import csv
import os
import shutil
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
EL = ROOT / "electrical"
sys.path.insert(0, str(EL))
sys.path.insert(0, str(ROOT / "sdk"))
import day_L as D  # noqa: E402
from faibotic import spec  # noqa: E402


def _csv(name):
    with open(EL / name, encoding="utf-8") as f:
        return list(csv.DictReader(f))


ROWS = _csv("bang-day.csv")
VT = _csv("vat-tu-day.csv")


def _num(x):
    return float(x) if x not in ("", None) else None


def test_csv_khop_day_L():
    gen = D.rows()
    assert [r["ma"] for r in ROWS] == [r["ma"] for r in gen]
    for a, b in zip(ROWS, gen):
        assert a == {k: str(v) for k, v in b.items()}, a["ma"]


def test_ma_day_duy_nhat():
    ids = [r["ma"] for r in ROWS]
    assert len(ids) == len(set(ids))


def test_moi_dau_noi_ton_tai():
    for r in ROWS:
        for ep in (r["tu"], r["den"]):
            c, p = D.split_ep(ep)
            assert c is not None, f"{r['ma']}: không có đầu nối cho {ep}"
            assert p in D.CONN[c]["chan"], f"{r['ma']}: {c} không có chân {p}"
        for t in (r["dau_tu"], r["dau_den"]):
            assert t.partition("@")[0] in D.VT, f"{r['ma']}: đầu cos {t} không có trong vật tư"


def test_moi_servo_co_cau_chi():
    for n in spec.PL_SERVO_MODELS:
        feed = [r for r in ROWS if r["tin_hieu"] == f"V+ {n}" or (n.startswith("wheel") and r["den"] == f"SV.{n}.A.V")]
        assert feed, f"{n} không có dây V+"
        for r in feed:
            assert r["cau_chi"] in D.FUSE and _num(r["cau_chi_A"]) > 0, f"{n}: {r['ma']} không có cầu chì"
        a = _num(feed[0]["cau_chi_A"])
        assert a >= 1.25 * D.allowed(n) - 1e-9, f"{n}: cầu chì {a} A < 1,25 × dòng cho phép"
    fids = {r["cau_chi"] for r in ROWS if r["tin_hieu"].startswith("V+ ") and "cổng" not in r["tin_hieu"]}
    assert len([f for f in fids if f in D.FUSE]) == len(spec.PL_SERVO_MODELS), "mỗi servo một cầu chì riêng"


def test_cau_chi_khong_vuot_chiu_dong_day():
    grp = defaultdict(float)
    for r in ROWS:
        if r["song_song"]:
            grp[r["song_song"]] += _num(r["ampacity_x0_82_A"])
    for r in ROWS:
        fa = _num(r["cau_chi_A"])
        if fa is None or r["ampacity_x0_82_A"] == "":
            continue
        amp = grp[r["song_song"]] if r["song_song"] else _num(r["ampacity_x0_82_A"])
        assert fa <= amp + 1e-9, f"{r['ma']}: cầu chì {fa} A > dây {amp} A"
        assert r["dat_dong_lien_tuc"] == "co", r["ma"]


@pytest.mark.xfail(strict=True, reason=" 6+1: dòng chặn đồng thời một tay 40,2 A > F3/F4 40 A và tiếp điểm 21W4 40 A; chờ kỹ sư điện quyết (nâng nấc cầu chì/tiếp điểm hoặc chấp nhận theo TORQUE_LIMIT 22,3 A)")
def test_cau_chi_cat_du_cho_hop_FB():
    """F3/F4 ≥ dòng chặn nhánh tới hộp FB; F1 ≥ dòng liên tục trục chính."""
    by = {r["ma"]: r for r in ROWS}
    assert D.FUSE["F3"]["A"] >= _num(by["BW07"]["dong_dinh_A"])
    assert D.FUSE["F4"]["A"] >= _num(by["BW08"]["dong_dinh_A"])
    assert D.FUSE["F1"]["A"] >= _num(by["BW01"]["dong_lien_tuc_A"])


def test_tong_khop_tinh_toan():
    by = {r["ma"]: r for r in ROWS}
    stall = 7 * spec.STS3095.stall_a + 11 * spec.STS3215.stall_a      # 6+1: 11 STS3215
    pk = stall + spec.PI_CURRENT_A + spec.CTRL_CURRENT_A + spec.LIFT_LATCH_COIL_A + D.AUX12_A
    assert abs(_num(by["BW01"]["dong_dinh_A"]) - round(pk, 1)) < 1e-6
    assert abs(D.STALL_SERVO - stall) < 1e-9
    fbl = 3 * spec.STS3095.stall_a + 4 * spec.STS3215.stall_a        # S2: FB-L chỉ còn tay trái: 4 STS3215/tay
    assert abs(_num(by["BW07"]["dong_dinh_A"]) - round(fbl, 1)) < 1e-6
    # servo: số dây V+ tới servo = số servo P-L
    assert sum(1 for n in spec.PL_SERVO_MODELS if any(r["den"] in (f"SV.{n}.A.V", f"SP.{n}.+") for r in ROWS)) == 18
    md = (EL / "tinh-toan.md").read_text(encoding="utf-8")
    assert f"{len(ROWS)} sợi" in md
    assert f"**{pk:.1f} A**".replace(".", ",") in md


def test_vat_tu_khop_bang_day():
    tot = defaultdict(int)
    for r in ROWS:
        if r["AWG"] in ("USB2.0", "20") or "FH-LEAD" in (r["dau_tu"], r["dau_den"]) or r["nhom"] == "cân bằng":
            continue
        tot[f"DAY-{r['AWG']}AWG-{r['mau']}"] += int(r["cat_mm"])
    vt = {v["ma_vt"]: v for v in VT}
    for k, mm in tot.items():
        assert abs(float(vt[k]["so_luong"]) - mm / 1000) < 1e-6, k
        assert float(vt[k]["mua"]) >= mm / 1000
    fuses = defaultdict(int)
    for f in D.FUSE.values():
        if f["loai"].startswith("ATO"):
            fuses[f["A"]] += 1
    for a, n in fuses.items():
        assert int(vt[f"ATO-{a:g}"]["so_luong"]) == n
    for v in VT:
        assert v["link"] or v["ma_vt"].startswith("DAY-"), f"{v['ma_vt']} thiếu link"


@pytest.mark.parametrize("name", ["bang-day.csv", "vat-tu-day.csv", "tinh-toan.md", "bang-di-day-evt.md"])
def test_khong_con_thieu(name):
    t = (EL / name).read_text(encoding="utf-8").lower()
    assert "còn thiếu" not in t and "con thieu" not in t  # public-scan: ok


def test_chay_lai_tinh_toan_khong_doi(tmp_path):
    """Chạy lại tinh_toan.py trên bản sao: bang-day.csv, vat-tu-day.csv, tinh-toan.md trùng từng byte."""
    el = tmp_path / "electrical"
    el.mkdir()
    for f in ("tinh_toan.py", "day_L.py"):
        shutil.copy(EL / f, el / f)
    os.symlink(ROOT / "sdk", tmp_path / "sdk")
    subprocess.run([sys.executable, str(el / "tinh_toan.py")], check=True, capture_output=True)
    for f in ("bang-day.csv", "vat-tu-day.csv", "tinh-toan.md", "bang-day-de-ban.csv", "vat-tu-de-ban.csv"):
        assert (el / f).read_bytes() == (EL / f).read_bytes(), f


# ---------------- S2 trạm hai tay ----------------
BY = {r["ma"]: r for r in ROWS}


def test_lift_tren_bus_de():
    """Servo trục nâng theo spec.SERVO_BUS (BUS_B cổng S2), nguồn và cuộn chốt từ tấm điện, không qua FB-L."""
    assert spec.SERVO_BUS["lift"] == "BUS_B"
    assert BY["D.LIFT"]["tu"] == "BUS_B.S2.D"
    assert BY["P.LIFT+"]["tu"] == "HFLIFT.out" and BY["P.LIFT+"]["AWG"] == "16" and BY["P.LIFT+"]["cau_chi"] == "FLIFT"
    assert BY["P.LIFT-"]["den"] == "GNDS.stud"
    assert BY["L01"]["tu"] == "HFLAT.out" and _num(BY["L01"]["cau_chi_A"]) == 2
    assert not any(r["tu"].startswith("FBL") and "lift" in r["tin_hieu"] for r in ROWS)
    for m in ("P.LIFT+", "P.LIFT-", "D.LIFT", "L03", "L04"):
        assert int(BY[m]["vong_dich_vu_mm"]) > 0, m


@pytest.mark.xfail(strict=True, reason=" 6+1: dòng chặn đồng thời một tay 40,2 A > F3/F4 40 A và tiếp điểm 21W4 40 A; chờ kỹ sư điện quyết (nâng nấc cầu chì/tiếp điểm hoặc chấp nhận theo TORQUE_LIMIT 22,3 A)")
def test_giac_21W4():
    """Nguồn hai tay qua 4 tiếp điểm 40 A, F3/F4 40 A, đuôi 8 AWG; tín hiệu theo bảng chân."""
    assert D.FUSE["F3"]["A"] == D.FUSE["F4"]["A"] == 40 <= D.DSUB_A
    nguon = {"A1": "BW07T", "A2": "BW07TN", "A3": "BW08T", "A4": "BW08TN"}
    for pin, m in nguon.items():
        r = BY[m]
        assert f"J21F.{pin}" in (r["tu"], r["den"]) and r["AWG"] == "8", m
        assert _num(r["dong_dinh_A"]) <= D.DSUB_A
    for m in ("BW07S", "BW07SN", "BW08S", "BW08SN"):
        assert BY[m]["AWG"] == "8" and _num(BY[m]["ampacity_x0_82_A"]) >= 40
    chan = {"1": "E01", "3": "E09", "5": "E10", "7": "E11", "9": "AX01", "16": "AX02", "13": "U03"}
    for pin, m in chan.items():
        assert f"J21F.{pin}" in (BY[m]["tu"], BY[m]["den"]), (pin, m)
    dung = defaultdict(list)
    for r in ROWS:
        for ep in (r["tu"], r["den"]):
            if ep.startswith(("J21F.", "J21M.")) and r["ma"] not in ("GD1", "GD2", "GD3", "GD4"):
                dung[ep].append(r["ma"])
    assert all(len(v) == 1 for v in dung.values()), {k: v for k, v in dung.items() if len(v) > 1}
    assert set(D.PIN_21W4) == set(D.DSUB_PINS)


def test_giac_21W4_khong_ke_chan_vong():
    """QC2 M1: theo bố trí hàng 21W4, không cặp chân kề nào cùng trong nhóm vòng + 12V_AUX; chân kề nút sau khoá không là VBUS/12 V;
    chân chắn nối GND ở cả nửa cái xe và đế bàn."""
    ke = D.dsub_adjacent()
    assert len(ke) == 8 + 7 + 16
    nhom = set(D.LOOP_PINS)
    assert not [tuple(sorted(c, key=int)) for c in ke if c <= nhom]
    xau = {D.USB_VBUS_PIN} | nhom
    for p in D.DOWNSTREAM_PINS:
        for c in ke:
            if p in c:
                (q,) = c - {p}
                assert q not in xau, (p, q)
                assert D.PIN_21W4[q][0].startswith(("chắn GND", "dự phòng", "USB D", "USB GND", "GND_AUX")), (p, q)
    for g in D.GUARD_PINS:
        assert D.PIN_21W4[g][0] == "chắn GND"
    gd = {(r["tu"], r["den"]) for r in ROWS if r["ma"].startswith("GD")}
    assert gd == {("J21F.2", "J21F.4"), ("J21F.4", "J21F.6"), ("J21F.6", "J21F.8"), ("J21F.8", "J21F.16")}
    assert by_ban_guard() == 4
    for m in ("E01", "E09", "E10", "E11", "AX01"):
        assert "ống bọc riêng" in BY[m]["ghi_chu"], m


def by_ban_guard():
    return sum(1 for r in _csv("bang-day-de-ban.csv") if r["ma"].startswith("BGD") and r["tu"].startswith("J21B."))


def test_vong_estop_qua_tram():
    """+12V_CTRL → 21W4.1 → khoá trái → khoá phải → E-stop trạm → 21W4.3 → E-stop đỉnh cột (nối tiếp)."""
    chuoi = [("TB1.1", "J21F.1"), ("J21M.1", "SWK_L.COM"), ("SWK_L.NO", "SWK_R.COM"), ("SWK_R.NO", "EST.NC1a"),
             ("EST.NC1b", "EST.NC2a"), ("EST.NC2b", "J21M.3"), ("J21F.3", "TB3.1"), ("TB3.1", "ES.NC1a"), ("ES.NC2b", "TB1.2")]
    cap = {(r["tu"], r["den"]) for r in ROWS}
    for c in chuoi:
        assert c in cap, c
    assert not any(r["tu"] == "TB1.1" and r["den"] == "ES.NC1a" for r in ROWS), "E-stop đỉnh cột không còn lấy thẳng từ TB1.1"
    assert ("SWK_L.NO", "J21M.5") in cap and ("SWK_R.NO", "J21M.7") in cap


def test_de_ban_tach_rieng():
    """Mạch đế bàn ở bảng riêng, không lẫn vào robot chuẩn; cầu chì ≤ chịu dòng dây."""
    rb = _csv("bang-day-de-ban.csv")
    assert [r["ma"] for r in rb] == [w["ma"] for w in D.WIRES_BAN]
    assert not any(r["ma"].startswith("B") and r["nhom"] == "đế bàn" for r in ROWS)
    for r in rb:
        fa = _num(r["cau_chi_A"])
        if fa is not None and r["ampacity_x0_82_A"] != "":
            assert fa <= _num(r["ampacity_x0_82_A"]) + 1e-9, r["ma"]
    assert D.BAN_I + D.AUX12_A <= 0.8 * 15 + 1e-9                     # đề xuất 11,4 A (spec còn 12,0: lượt SDK)
    assert D.FUSE_BAN["BF1"]["A"] == D.FUSE_BAN["BF2"]["A"] == 15


def test_bang_di_day_evt_khop_nguon():
    """E1: bảng đi dây EVT không giữ số cũ (F3/F4 60 A, 160 sợi, 52,2 A); số khớp bang-day.csv và tinh-toan.md."""
    from faibotic import limits as _lim
    md = (EL / "bang-di-day-evt.md").read_text(encoding="utf-8")
    assert f"{len(ROWS)} sợi" in md
    bud = _lim.current_budget(spec.PL_SERVO_MODELS, models=spec.PL_SERVO_MODELS)
    assert f"{bud['allowed_a']:.1f} A".replace(".", ",") in md
    for f in ("F3", "F4"):
        assert f"{f} MEGA {D.FUSE[f]['A']} A" in md
    for cu in ("MEGA 60 A", "60 A (F3, F4)", "160 sợi", "52,2 A"):
        assert cu not in md, cu


def test_review_dien_giu_dong_cam_pin_that():
    """REVIEW điện còn dòng 'chưa duyệt, không đấu pin thật' tới khi có chữ ký."""
    files = list(EL.glob("REVIEW*"))
    assert files
    for p in files:
        t = p.read_text(encoding="utf-8").lower()
        assert "chưa duyệt" in t and "không đấu pin thật" in t, p.name
