"""Tầng 1b, nhóm (d) soak, R-34..R-36 (docs/S4/KE-HOACH-TANG-1B.md mục 3).

- R-34 / R-35: `@pytest.mark.soak`, gọi scripts/soak.py (bản 2 giờ chạy bằng `python scripts/soak.py --phut 120`, nightly). Ở đây chỉ
  bản ngắn: SOAK_PHUT phút (mặc định 1) với ngưỡng nới. Mặc định pytest bỏ qua mark soak (pytest.ini: addopts -m "not soak");
  chạy: `pytest -m soak tests/tier1b/test_soak.py`.
- R-36: máy trạng thái enable -> chạy -> E-stop/DOCK/dừng thường -> reset -> enable. Không mark soak: CI chạy SOAK_CHU_KY=200 chu kỳ,
  nightly đặt SOAK_CHU_KY=10000.

Chỉ viết test, không sửa SDK.
"""
from __future__ import annotations

import gc
import os
import random
import sys
import threading
import warnings
from pathlib import Path

import numpy as np
import pytest

from faibotic import spec
from faibotic.backends.feetech import DockOpen, FeetechBackend
from faibotic.feetech import protocol as P
from faibotic.feetech.mockbus import FakeServoPort
from faibotic.safety import DockSwitches, FakeGPIO, SafetyLatch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import soak as S  # noqa: E402

pytestmark = pytest.mark.t1b
ALL_IDS = list(spec.SERVO_IDS.values())
SOAK_PHUT = float(os.environ.get("SOAK_PHUT", "1"))
SOAK_CHU_KY = int(os.environ.get("SOAK_CHU_KY", "200"))


def _chay_ngan(backend: str, tmp_path):
    bc = S.chay(backend, SOAK_PHUT, chu_ky_mau=5.0, bao_cao=str(tmp_path / f"soak-{backend}.json"), log=lambda s: None)
    msg = "\n".join(f"{'ĐẠT' if v['dat'] else 'HỎNG'} {k}: {v['chi_tiet']}" for k, v in bc["tieu_chi"].items())
    return bc, msg


@pytest.mark.soak
def test_r34_soak_sim_ngan(tmp_path):
    """Sim chạy SOAK_PHUT phút: 0 ngoại lệ, RSS không phình, p99 không tăng dần, không trôi, luồng/fd đứng yên, log nhỏ."""
    bc, msg = _chay_ngan("sim", tmp_path)
    assert bc["so_vong"] >= 5 and bc["mau"], "chạy quá ít vòng để kết luận"
    assert bc["dat"], msg
    assert (tmp_path / "soak-sim.json").stat().st_size > 0


@pytest.mark.soak
def test_r35_soak_bus_gia_chot_ngan(tmp_path):
    """FeetechBackend + bus giả + FakeGPIO + luồng giám sát E-stop (lỗi thoáng 0,1% chờ lớp chèn lỗi của Lane A; ở đây 0%)."""
    bc, msg = _chay_ngan("bus-gia", tmp_path)
    assert bc["so_vong"] >= 5 and bc["mau"]
    assert bc["dat"], msg


def test_soak_bao_cao_danh_gia_dung_chieu():
    """Bộ chấm của scripts/soak.py phải bắt được rò RSS, p99 tăng dần, trôi, rò luồng: kiểm bộ chấm bằng dữ liệu dựng."""
    ng = S.NGUONG["dai"]

    class H:
        byte, dong = 10, 1

        @staticmethod
        def lap_max():
            return 1
    mau = [{"t_s": 60.0 * i, "rss_kb": 100_000, "step_p99_ms": 0.3} for i in range(1, 13)]
    vong = [{"troi_deg": 0.2}] * 5
    ok = S.danh_gia(mau, vong, [], (2, 4), (2, 4), H, ng, 120.0)
    assert all(v["dat"] for v in ok.values())
    ro = S.danh_gia([{**m, "rss_kb": m["rss_kb"] + 2000 * i} for i, m in enumerate(mau)], vong, [], (2, 4), (2, 4), H, ng, 120.0)
    assert not ro["rss"]["dat"]
    cham = S.danh_gia([{**m, "step_p99_ms": 0.3 + 0.1 * i} for i, m in enumerate(mau)], vong, [], (2, 4), (2, 4), H, ng, 120.0)
    assert not cham["p99_khong_tang_dan"]["dat"]
    assert not S.danh_gia(mau, [{"troi_deg": 0.2}, {"troi_deg": 0.9}], [], (2, 4), (2, 4), H, ng, 120.0)["troi"]["dat"]
    assert not S.danh_gia(mau, vong, [], (2, 4), (3, 4), H, ng, 120.0)["luong_fd_khong_doi"]["dat"]
    assert not S.danh_gia(mau, vong, ["vòng 3: ValueError: x"], (2, 4), (2, 4), H, ng, 120.0)["ngoai_le"]["dat"]


# ------------------------------------------------------------------ R-36 máy trạng thái
def torque_off(port) -> bool:
    return all(port.mem[i][P.TORQUE_ENABLE[0]] == 0 for i in ALL_IDS)


def _fd() -> int:
    return S.so_fd()


def test_r36_may_trang_thai_enable_estop_dock_reset():
    """SOAK_CHU_KY chu kỳ enable -> chạy 1 s -> sự cố ngẫu nhiên (E-stop / khoá trạm rớt / dừng thường) -> khôi phục: luôn về trạng
    thái an toàn (mô-men tắt, enabled False, dốc đế 0), không rò luồng/fd, số cảnh báo mỗi chu kỳ không tăng dần."""
    rng = random.Random(20261124)
    port = FakeServoPort(ALL_IDS)
    port.log = __import__("collections").deque(maxlen=1000)
    g, dock_g = FakeGPIO(), FakeGPIO(DOCK_L=True, DOCK_R=True)
    lt = SafetyLatch(g, settle_s=0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        be = FeetechBackend(port=port, latch=lt, dock=DockSwitches(dock_g), sleep=lambda s: None, allow_lift_underpowered=True)
    gc.collect()
    base_thr, base_fd = threading.active_count(), _fd()
    canh_bao, hong = [], []
    try:
        for k in range(SOAK_CHU_KY):
            with warnings.catch_warnings(record=True) as w:
                warnings.simplefilter("always")
                if not lt.ok:
                    lt.reset()
                be.enable()
                assert be.enabled, f"chu kỳ {k}: enable() xong mà enabled False"
                for i in range(50):
                    if i % 10 == 0:
                        be.set_base_velocity(0.05, 0.0, 0.0)
                        be.write_joints({"left_pan": 0.05 * ((-1) ** i)})
                    be.step(0.02)
                sk = rng.choice(["estop", "dock", "dung"])
                try:
                    if sk == "estop":
                        g.levels["ESTOP_SENSE"] = False
                        be.step(0.02)
                    elif sk == "dock":
                        dock_g.levels["DOCK_L"] = False
                        be.step(0.02)
                    else:
                        be.disable()
                except DockOpen:
                    pass
                # trạng thái an toàn
                if not (torque_off(port) and not be.enabled):
                    hong.append(f"chu kỳ {k} ({sk}): mô-men chưa tắt hoặc enabled còn True")
                if sk != "dung" and np.any(be.base_velocity() != 0):
                    hong.append(f"chu kỳ {k} ({sk}): dốc đế chưa về 0 {be.base_velocity()}")
                # khôi phục: phải clear_fault / reset rõ ràng mới chạy tiếp
                if sk == "estop":
                    with pytest.raises(Exception):
                        be.enable()                           # chốt chưa reset: từ chối
                    g.levels["ESTOP_SENSE"] = True
                elif sk == "dock":
                    with pytest.raises(DockOpen):
                        be.enable()                           # chưa clear_fault: từ chối
                    dock_g.levels["DOCK_L"] = True
                    be.clear_fault()
                canh_bao.append(sum(issubclass(x.category, RuntimeWarning) for x in w))
            if len(hong) >= 5:
                break
    finally:
        be.close()
    assert not hong, "\n".join(hong[:5])
    gc.collect()
    assert threading.active_count() <= base_thr, \
        f"rò luồng: {base_thr} -> {threading.active_count()}"
    assert _fd() <= base_fd, f"rò fd: {base_fd} -> {_fd()}"
    n = max(1, len(canh_bao) // 10)
    assert sum(canh_bao[-n:]) / n <= sum(canh_bao[:n]) / n + 1e-9, f"cảnh báo mỗi chu kỳ tăng dần: {canh_bao[:5]} ... {canh_bao[-5:]}"
