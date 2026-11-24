"""Kịch bản thử EVT (product/evt/*.py) chạy được ở chế độ khô và ghi kết quả JSON/CSV; các nhánh kết luận quan trọng."""
import csv
import importlib.util
import json
import sys
from pathlib import Path

import pytest

EVT = Path(__file__).resolve().parents[1] / "product" / "evt"
sys.path.insert(0, str(EVT))


def load(name):
    spec = importlib.util.spec_from_file_location(f"evt_{name}", EVT / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def run(name, tmp_path, *argv, dry=True):
    import evtlib
    mod = load(name)
    args = list(argv) + (["--dry-run"] if dry else []) + ["--out", str(tmp_path)]
    rc = evtlib.run(mod.main, argv=args, add_args=getattr(mod, "add_args", None))
    files = sorted((f for f in (tmp_path / "FM1-0000-000").glob(f"{name}_*.json") if "_dang-chay" not in f.name),
                   key=lambda f: f.stat().st_mtime_ns) if dry and (tmp_path / "FM1-0000-000").exists() else []
    return rc, (json.loads(files[-1].read_text(encoding="utf-8")) if files else None), files


CASES = [
    ("quet_bus", []),
    ("quet_bus", ["--che-do", "16"]),
    ("quet_bus", ["--che-do", "rieng", "--id", "17", "--quay", "--quet-vi-tri"]),
    ("estop_jig", ["--che-do", "pi"]),
    ("estop_jig", ["--che-do", "jig", "--chu-ky", "3"]),
    ("estop_jig", ["--mach", "pcb", "--che-do", "pi"]),
    ("estop_jig", ["--mach", "pcb", "--che-do", "jig", "--chu-ky", "3"]),
    ("estop_jig", ["--mach", "pcb", "--che-do", "phan-mem"]),
    ("quay_nguoc", []),
    ("quay_nguoc", ["--doi-tuong", "cap", "--khop", "right_elbow", "--kieu", "khoi-luong"]),
    ("cap_ghep_doi", ["--giai-doan", "3A"]),
    ("cap_ghep_doi", ["--giai-doan", "3B", "--pha", "tat"]),
    ("gioi_han_mem", []),
    ("lift_da_vong", []),
    ("cong_tac_hanh_trinh", []),
    ("dong_dien", ["--mau-s", "1", "--teleop-s", "2"]),
    ("dong_dien", ["--nguon", "dong-ho"]),
    ("dong_dien", ["--mach", "pcb", "--mau-s", "1", "--teleop-s", "2"]),
    ("tai_nhiet", ["--phut", "10"]),
    ("lap_lai_tcp", ["--lan", "4"]),
    ("burnin", ["--phut", "0.5"]),
    ("ghi_tay", []),
]


@pytest.mark.parametrize("name,argv", CASES, ids=[f"{n}-{'-'.join(a[:4])}" if a else n for n, a in CASES])
def test_dry_run_writes_results(tmp_path, name, argv):
    rc, out, files = run(name, tmp_path, *argv)
    assert rc == 0, out and [c for c in out["tieu_chi"] if c["trang_thai"] == "khong_dat"]
    assert out["serial"] == "FM1-0000-000" and "MÔ PHỎNG" in out["che_do"] and out["tieu_chi"]
    assert all(c["trang_thai"] in ("dat", "ghi_nhan") for c in out["tieu_chi"])
    rows = list(csv.DictReader(files[-1].with_suffix(".csv").open(encoding="utf-8")))
    assert len(rows) == len(out["tieu_chi"]) and rows[0]["serial"] == "FM1-0000-000"


def test_lift_multiturn_branches(tmp_path):
    _, a, _ = run("lift_da_vong", tmp_path / "a", "--mock-da-vong", "khong")
    assert a["du_lieu"]["ho_tro_da_vong"] is False and a["du_lieu"]["ket_luan"].startswith("KHÔNG đa vòng")
    assert a["du_lieu"]["mode0_dich_5000_doc"] <= 4095
    _, b, _ = run("lift_da_vong", tmp_path / "b", "--mock-giu", "co", "--toan-dai")
    assert b["du_lieu"]["ho_tro_da_vong"] and b["du_lieu"]["giu_dem_sau_mat_dien"]
    assert [t["dich"] for t in b["du_lieu"]["mode3_thu"]][-2:] == [32000, -32000]
    _, c, _ = run("lift_da_vong", tmp_path / "c")
    assert c["du_lieu"]["ho_tro_da_vong"] and not c["du_lieu"]["giu_dem_sau_mat_dien"] and "homing" in c["du_lieu"]["ket_luan"]


def test_lift_restores_mode(tmp_path):
    import evtlib
    seen = {}
    orig = evtlib.Ctx.finish

    def finish(self):
        seen["mode"] = self.port.mem[16][evtlib.P.MODE[0]]
        return orig(self)
    evtlib.Ctx.finish = finish
    try:
        run("lift_da_vong", tmp_path)
    finally:
        evtlib.Ctx.finish = orig
    assert seen["mode"] == 0


def test_backdrive_below_threshold_blocks_spring(tmp_path):
    ans = tmp_path / "a.json"
    ans.write_text(json.dumps({k: 10.0 for k in ("g0_duong", "g0_am", "g120_duong", "g120_am", "g240_duong", "g240_am")} | {"r_m": 0.1, "t_tay_don": 0.0}))
    rc, out, _ = run("quay_nguoc", tmp_path, "--doi-tuong", "cap", "--khop", "left_elbow", "--answers", str(ans))
    assert rc == 1     # 10 N x 0,1 m = 1,0 N·m < 2,5 N·m khuỷu
    bad = [c for c in out["tieu_chi"] if c["trang_thai"] == "khong_dat"]
    assert bad and "KHÔNG lắp lò xo" in bad[0]["ghi_chu"]


def test_missing_measurement_is_pending(tmp_path):
    ans = tmp_path / "a.json"
    ans.write_text(json.dumps({"khoi_luong_kg": None}))
    rc, out, _ = run("ghi_tay", tmp_path, "--ma", "EVT-13", "--answers", str(ans))
    st = {c["hang_muc"]: c["trang_thai"] for c in out["tieu_chi"]}
    assert st["khối lượng robot"] == "chua_kiem" and rc == 0


def test_estop_sim_detects_k1_drop_time(tmp_path):
    _, out, _ = run("estop_jig", tmp_path, "--mach", "pcb", "--che-do", "pi", "--chu-ky", "2")
    ms = [r["nha_k1_ms"] for r in out["du_lieu"]["chu_ky"]]
    assert all(25 <= m <= 40 for m in ms)          # SimPCB nhả K1 sau 30 ms ảo


def test_estop_bang_di_day_pi_sequence(tmp_path):
    """Bộ #0: K5 bật trước khi người bấm Reset, chạy chỉ sau ARM_SENSE; ESTOP_OK -> ARM_SENSE ~30 ms (tụ giả)."""
    rc, out, _ = run("estop_jig", tmp_path, "--che-do", "pi", "--chu-ky", "2")
    assert rc == 0
    rows = out["du_lieu"]["chu_ky"]
    assert all(r["dat"] and 25 <= r["nha_k1_ms"] <= 40 for r in rows)
    log = out["du_lieu"]["nhat_ky_chot"]
    assert any("K5 bật, chờ người" in m for m in log) and any("Reset đang nhấn" in m for m in log)
    assert all("RELAY_EN" not in m and "LATCH_SENSE" not in m for m in log)


def test_estop_bang_di_day_rejects_phan_mem(tmp_path):
    with pytest.raises(SystemExit, match="pcb"):
        run("estop_jig", tmp_path, "--che-do", "phan-mem")


@pytest.mark.parametrize("mode", ["pi", "jig"])
@pytest.mark.parametrize("loi", ["khong-chot", "k1-dinh"])
def test_estop_bang_di_day_faults_fail(tmp_path, mode, loi):
    rc, out, _ = run("estop_jig", tmp_path, "--che-do", mode, "--chu-ky", "2", "--mock-loi", loi)
    assert rc == 1
    assert any(c["ma"] == "Q-IPQC-09" and c["trang_thai"] == "khong_dat" for c in out["tieu_chi"])


def test_real_mode_requires_serial_operator_and_pins(tmp_path):
    with pytest.raises(SystemExit, match="serial"):
        run("estop_jig", tmp_path, dry=False)
    with pytest.raises(SystemExit, match="nguoi-do"):
        run("estop_jig", tmp_path, "--serial", "FM1-0000-000", dry=False)
    ans = tmp_path / "a.json"
    ans.write_text(json.dumps({"_xac_nhan_tat_ca": True}))
    with pytest.raises(SystemExit, match="K5_EN"):
        run("estop_jig", tmp_path, "--serial", "FM1-0000-000", "--nguoi-do", "A", "--answers", str(ans), dry=False)
    with pytest.raises(SystemExit, match="RELAY_EN"):
        run("estop_jig", tmp_path, "--serial", "FM1-0000-000", "--nguoi-do", "A", "--answers", str(ans), "--mach", "pcb", dry=False)


def test_pair_calibration_file_used_by_later_scripts(tmp_path):
    run("cap_ghep_doi", tmp_path, "--giai-doan", "3A")
    cal = json.loads((tmp_path / "FM1-0000-000" / "calib.json").read_text())
    assert cal["left_shoulder"]["sign"] == -cal["left_shoulder_b"]["sign"]
    rc, out, _ = run("gioi_han_mem", tmp_path)
    assert rc == 0


def _answers(tmp_path, d):
    f = tmp_path / "ans.json"
    f.write_text(json.dumps(d))
    return str(f)


def test_hold_phase_refuses_without_pair_backdrive(tmp_path):
    rc, out, _ = run("cap_ghep_doi", tmp_path, "--giai-doan", "3B", "--pha", "giu", "--giu-s", "2")
    assert rc == 3 and out is None                       # từ chối, không bật mô-men, không ghi kết quả
    run("quay_nguoc", tmp_path, "--doi-tuong", "cap", "--khop", "right_shoulder")   # khớp khác: vẫn từ chối
    rc, _, _ = run("cap_ghep_doi", tmp_path, "--giai-doan", "3B", "--pha", "giu", "--giu-s", "2")
    assert rc == 3


def test_hold_phase_refuses_when_backdrive_failed(tmp_path):
    low = {k: 5.0 for k in ("g0_duong", "g0_am", "g120_duong", "g120_am", "g240_duong", "g240_am")} | {"r_m": 0.1, "t_tay_don": 0.0}
    rc, _, _ = run("quay_nguoc", tmp_path, "--doi-tuong", "cap", "--khop", "left_shoulder", "--answers", _answers(tmp_path, low))
    assert rc == 1
    rc, _, _ = run("cap_ghep_doi", tmp_path, "--giai-doan", "3B", "--pha", "giu", "--giu-s", "2")
    assert rc == 3


def test_hold_phase_after_backdrive_pass_and_exception(tmp_path):
    run("cap_ghep_doi", tmp_path / "a", "--giai-doan", "3B", "--pha", "tat")
    run("quay_nguoc", tmp_path / "a", "--doi-tuong", "cap", "--khop", "left_shoulder")
    rc, out, _ = run("cap_ghep_doi", tmp_path / "a", "--giai-doan", "3B", "--pha", "giu", "--giu-s", "5")
    assert rc == 0 and "đạt" in out["du_lieu"]["quay_nguoc"]
    assert any(c["ma"] == "Q-FQC-C4" and c["trang_thai"] == "dat" for c in out["tieu_chi"])
    rc, out, _ = run("cap_ghep_doi", tmp_path / "b", "--giai-doan", "3B", "--pha", "giu", "--giu-s", "2", "--ngoai-le-duyet", "Người Duyệt")
    assert rc == 0 and out["du_lieu"]["ngoai_le_duyet"] == "Người Duyệt"
    assert any("NGOẠI LỆ" in c["hang_muc"] for c in out["tieu_chi"])


def test_torque_off_phase_never_enables_torque(tmp_path, monkeypatch):
    import evtlib
    seen = []
    orig = evtlib.EvtPort._set

    def spy(self, sid, addr, data):
        if addr == evtlib.P.TORQUE_ENABLE[0] and data[0]:
            seen.append(sid)
        return orig(self, sid, addr, data)
    monkeypatch.setattr(evtlib.EvtPort, "_set", spy)
    rc, _, _ = run("cap_ghep_doi", tmp_path, "--giai-doan", "3B", "--pha", "tat")
    assert rc == 0 and seen == []


@pytest.mark.parametrize("mode", ["pi", "jig"])
def test_estop_faulty_pcb_fails(tmp_path, mode):
    rc, out, _ = run("estop_jig", tmp_path, "--mach", "pcb", "--che-do", mode, "--chu-ky", "3", "--mock-loi", "khong-chot")
    assert rc == 1
    st = {(c["ma"], c["hang_muc"]): c["trang_thai"] for c in out["tieu_chi"]}
    assert any(k[0] == "Q-IPQC-09" and v == "khong_dat" for k, v in st.items())


def test_estop_pcb_without_hw_latch_fails_pi_mode(tmp_path, monkeypatch):
    mod = load("estop_jig")
    orig = mod.SimPCB.__init__
    monkeypatch.setattr(mod.SimPCB, "__init__", lambda self, ctx, hw_latch=True, reclose=False: orig(self, ctx, False, reclose))
    import evtlib
    rc = evtlib.run(mod.main, argv=["--dry-run", "--out", str(tmp_path), "--mach", "pcb", "--che-do", "pi", "--chu-ky", "2"], add_args=mod.add_args)
    assert rc == 1


def test_operator_refusal_aborts(tmp_path, monkeypatch):
    monkeypatch.setattr("builtins.input", lambda *a: "khong")
    rc, _, _ = run("estop_jig", tmp_path, "--serial", "FM1-0000-000", "--nguoi-do", "A", dry=False)
    assert rc == 3


def test_soft_limits_abort_when_torque_on(tmp_path, monkeypatch):
    import evtlib

    class Hot(evtlib.EvtPort):
        def __init__(self, **kw):
            super().__init__(**kw)
            self.mem[2][evtlib.P.TORQUE_ENABLE[0]] = 1
    monkeypatch.setattr(evtlib, "EvtPort", Hot)
    rc, out, _ = run("gioi_han_mem", tmp_path)
    assert rc == 3 and out is None


def test_pair_3a_same_direction_fails(tmp_path):
    rc, out, _ = run("cap_ghep_doi", tmp_path, "--giai-doan", "3A",
                     "--answers", _answers(tmp_path, {"chieu_left_shoulder": 1, "chieu_left_shoulder_b": 1}))
    assert rc == 1 and any(c["trang_thai"] == "khong_dat" and "ngược chiều" in c["hang_muc"] for c in out["tieu_chi"])


@pytest.mark.parametrize("name,argv", [("quet_bus", ["--che-do", "rieng", "--id", "16", "--quay"]), ("cap_ghep_doi", ["--giai-doan", "3A", "--khop", "left_elbow"])])
def test_refuse_torque_when_mode_not_zero(tmp_path, monkeypatch, name, argv):
    import evtlib

    class Step(evtlib.EvtPort):
        def __init__(self, **kw):
            super().__init__(**kw)
            for m in self.mem.values():
                m[evtlib.P.MODE[0]] = evtlib.MODE_STEP
    monkeypatch.setattr(evtlib, "EvtPort", Step)
    rc, _, _ = run(name, tmp_path, *argv)
    assert rc == 3


def test_burnin_checkpoint_and_raw_current(tmp_path):
    run("burnin", tmp_path, "--phut", "0.2")
    part = list((tmp_path / "FM1-0000-000").glob("burnin_*_dang-chay.json"))
    assert part and json.loads(part[0].read_text())["du_lieu"]["chu_trinh"] >= 1
    _, out, _ = run("quet_bus", tmp_path, "--che-do", "rieng", "--id", "17", "--quay")
    assert out["du_lieu"]["dong_khong_tai_raw"]["trung_vi"] > 0


def test_xa_tu_measure_and_gate(tmp_path):
    """Đo xả tụ (có servo, Reset giữ) trước lần bật mô-men đầu; số quá dài thì kịch bản bật mô-men dừng (mã 3)."""
    rc, out, _ = run("estop_jig", tmp_path / "a", "--che-do", "xa-tu", "--tu-giu-s", "0.25")
    assert rc == 0 and all(200 <= m <= 300 for m in out["du_lieu"]["xa_tu_ms"])
    rc, out, _ = run("estop_jig", tmp_path / "b", "--che-do", "xa-tu", "--tu-giu-s", "0.8")
    assert rc == 1
    rc, out, _ = run("burnin", tmp_path / "c", "--phut", "0.1", "--xa-tu-ms", "800")
    assert rc == 3 and out is None
    rc, _, _ = run("burnin", tmp_path / "d", "--phut", "0.1", "--xa-tu-ms", "300")
    assert rc == 0


def test_pigpio_requires_hw():
    import evtlib
    with pytest.raises(ValueError, match="hw"):
        evtlib.PiGPIO({"K5_EN": 22})


@pytest.mark.parametrize("ms", ["0.3", "0", "-5", "nan", "abc"])
def test_xa_tu_ms_invalid_refuses(tmp_path, ms):
    """M2 (QC2-dk-dien-sdk): --xa-tu-ms sai đơn vị/giá trị -> SafetyLatch từ chối reset, kịch bản bật mô-men dừng (mã 3)."""
    rc, out, _ = run("burnin", tmp_path, "--phut", "0.1", "--xa-tu-ms", ms)
    assert rc == 3 and out is None


@pytest.mark.parametrize("muc", ["cao", "thap"])
def test_cong_tac_nc_default_and_no(tmp_path, muc):
    """M1: mặc định --muc-cham cao = công tắc NC + kéo lên (bảng đi dây); thap = NO. Chạy khô theo đúng cách đi dây."""
    mod = load("cong_tac_hanh_trinh")
    import argparse
    ap = argparse.ArgumentParser()
    mod.add_args(ap)
    assert ap.parse_args([]).muc_cham == "cao"
    rc, out, _ = run("cong_tac_hanh_trinh", tmp_path, "--muc-cham", muc)
    st = [c["trang_thai"] for c in out["tieu_chi"] if c["ma"] == "QC2-cot-nang #4"]
    assert rc == 0 and st and all(s == "dat" for s in st)
