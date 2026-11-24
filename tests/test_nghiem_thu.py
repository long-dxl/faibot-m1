"""scripts/nghiem_thu.py: bảng nghiệm thu khớp yaml, mỗi dòng Tầng 1 có kiểm, cách gộp trạng thái đúng."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import nghiem_thu as N  # noqa: E402


def test_tai_lieu_khop_yaml():
    assert N.sinh(kiem_thoi=True) == 0


def test_moi_dong_tier1_co_kiem_va_tieng_anh():
    d = N.nap_yaml()
    assert len(d["tier1"]) >= 10 and len(d["tier2"]) == 8
    ids = [r["id"] for r in d["tier1"] + d["tier2"]]
    assert len(ids) == len(set(ids))
    for r in d["tier1"]:
        assert r["kiem"] and {"nhom", "chi_tieu", "nguong"} <= set(r["en"]), r["id"]


def test_kiem_cheo_id_co_that_hoac_cho_merge():
    kc = N.kc_ids()
    for r in N.nap_yaml()["tier1"]:
        for c in r["kiem"]:
            if "kc" in c:
                assert c["kc"] in kc or c.get("cho_merge"), c


def test_gop_trang_thai():
    ch = N.Chay(nhanh=True)
    assert ch.dong({"id": "y", "cham": True, "kiem": []})[0] == N.CHO
    assert ch.dong({"id": "z", "kiem": [{"chua": "x"}]})[0] == N.CHO
    assert ch.dong({"id": "e", "kiem": [{"evt": 1}]})[0] == N.EVT


def test_tom_tat_dem_khong_dat():
    kq = [dict(id="a", tier=1, nhom="", chi_tieu="", ket_qua=N.KHONG, chi_tiet=""),
          dict(id="b", tier=2, nhom="", chi_tieu="", ket_qua=N.EVT, chi_tiet="")]
    t1, tat = N.tom_tat(kq)
    assert t1[N.KHONG] == 1 and tat[N.EVT] == 1


# ---------------- Tầng 1b ----------------
def test_tier1b_du_dong_co_kiem_va_tieng_anh():
    d = N.nap_yaml()
    rows = d["tier1b"]
    assert [r["id"] for r in rows[:6]] == [f"T1b.0{i}" for i in range(1, 7)]
    ids = [r["id"] for r in d["tier1"] + rows + d["tier2"]]
    assert len(ids) == len(set(ids))
    for r in rows:
        assert r["kiem"] and {"nhom", "chi_tieu", "nguong"} <= set(r["en"]), r["id"]
    vi, en = N.DOC_VI.read_text(encoding="utf8"), N.DOC_EN.read_text(encoding="utf8")
    for r in rows:
        assert r["id"] in vi and r["id"] in en
    assert "## Tầng 1b" in vi and "## Tier 1b" in en


def test_kiem_tier1b_tro_toi_file_co_that_hoac_cho_merge():
    for r in N.nap_yaml()["tier1b"]:
        for c in r["kiem"]:
            f = c.get("pytest", c.get("script", "")).split("::")[0]
            assert (N.ROOT / f).exists() or c.get("cho_merge"), (r["id"], f)


def test_xfail_cho_khong_dem_la_dat(monkeypatch):
    ch = N.Chay(nhanh=False)
    monkeypatch.setattr(N, "_chay", lambda *a, **k: (0, "..x.\n12 passed, 3 xfailed in 1.0s"))
    assert ch.mot({"pytest": "t.py", "xfail_cho": True})[0] == N.CHO
    assert ch.mot({"pytest": "t.py"})[0] == N.DAT                    # không đánh dấu: giữ hành vi cũ của Tầng 1
    monkeypatch.setattr(N, "_chay", lambda *a, **k: (0, "15 passed in 1.0s"))
    assert ch.mot({"pytest": "t.py", "xfail_cho": True})[0] == N.DAT
    monkeypatch.setattr(N, "_chay", lambda *a, **k: (1, "1 failed, 3 xfailed in 1.0s"))
    assert ch.mot({"pytest": "t.py", "xfail_cho": True})[0] == N.KHONG


def test_tom_tat_1b_tach_khoi_tier1():
    kq = [dict(id="a", tier=1, nhom="", chi_tieu="", ket_qua=N.DAT, chi_tiet=""),
          dict(id="b", tier="1b", nhom="", chi_tieu="", ket_qua=N.KHONG, chi_tiet=""),
          dict(id="c", tier="1b", nhom="", chi_tieu="", ket_qua=N.CHO, chi_tiet="")]
    t1, tat = N.tom_tat(kq)
    t1b = N.tom_tat_1b(kq)
    assert t1[N.KHONG] == 0 and t1b[N.KHONG] == 1 and t1b[N.CHO] == 1 and tat[N.KHONG] == 1
    assert "Tầng 1b (2 dòng)" in N.bang(kq)
