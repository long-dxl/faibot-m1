"""Tầng 1b, nhóm (f) cấu hình, R-43..R-51 (docs/S4/KE-HOACH-TANG-1B.md mục 3).

Chỉ viết test. Ca nào SDK hiện tại trượt thì đánh xfail(strict=True, reason="T1b.R-xx: <sửa cần có>"): khi bản sửa vào mà chưa
bỏ đánh dấu, test tự đỏ. Bus giả: FakeServoPort; chốt E-stop và DOCK giả dựng ngay trong file này (không dùng conftest chung).
"""
from __future__ import annotations

import copy
import math
import warnings

import pytest

from faibotic import Robot, spec
from faibotic.backends.feetech import DockOpen, FeetechBackend, default_calibration
from faibotic.feetech.mockbus import FakeServoPort
from faibotic.gpio_pi import GPIOConfigError, GPIOUnavailable, PiGPIO, config_from_env
from faibotic.safety import DockSwitches, FakeGPIO

pytestmark = [pytest.mark.t1b, pytest.mark.filterwarnings("ignore::RuntimeWarning")]

ALL_IDS = list(spec.SERVO_IDS.values())
BUS_B = list(spec.BUS_B_IDS)
NOSLEEP = dict(sleep=lambda s: None, allow_lift_underpowered=True)


class RunLatch:
    """Chốt E-stop giả đang ở 'run' để kiểm riêng khoá trạm mà không cần allow_no_latch."""
    on_stop = None
    monitor_alive, ok, monitor_error = True, True, None

    def start_monitor(self, period):
        pass

    def stop_monitor(self, timeout=1.0):
        pass

    def poll(self):
        return True


def dock_ok():
    return DockSwitches(FakeGPIO(DOCK_L=True, DOCK_R=True))


def bench(ids=None, **kw):
    """Backend di_dong trên bus giả, khoá trạm đóng, không chốt phần mềm (allow_no_latch)."""
    port = FakeServoPort(ALL_IDS if ids is None else ids)
    return FeetechBackend(port=port, dock=kw.pop("dock", dock_ok()), allow_no_latch=True, **NOSLEEP, **kw)


def xf(ma: str, sua: str):
    return pytest.mark.xfail(strict=True, reason=f"T1b.R-{ma}: {sua}")


# ------------------------------------------------------------------ R-43
def test_r43_di_dong_du():
    """Đủ servo + DOCK đóng: nhận di_dong, vùng cấm dau_camera + be_tram, enable chạy."""
    be = bench()
    assert be.config == "di_dong"
    assert set(spec.ROBOT_CONFIGS["di_dong"]["keepout"]) == {"dau_camera", "be_tram"}
    be.enable()
    assert be.enabled
    be.close()
    r = Robot("feetech", port=FakeServoPort(ALL_IDS), dock=dock_ok(), allow_no_latch=True, **NOSLEEP)
    assert r.config == "di_dong"
    r.close()


# ------------------------------------------------------------------ R-44
def test_r44_tram_ban_tu_choi_ro_rang():
    ids = [i for i in ALL_IDS if i not in BUS_B]
    with pytest.raises(ConnectionError) as e:
        FeetechBackend(port=FakeServoPort(ids), allow_no_latch=True, **NOSLEEP)
    assert "tram_ban" in str(e.value) and "chưa hỗ trợ" in str(e.value)


@xf("44", "(ngoài phạm vi lô sửa 24/11) chờ core S3: backend nhận tram_ban, lệnh đế/lift bị từ chối có kiểu, vùng cấm có mat_ban")
def test_r44_tram_ban_chay_duoc_cho_s3():
    ids = [i for i in ALL_IDS if i not in BUS_B]
    r = Robot("feetech", port=FakeServoPort(ids), allow_no_latch=True, **NOSLEEP)
    assert r.config == "tram_ban"
    assert "mat_ban" in spec.ROBOT_CONFIGS["tram_ban"]["keepout"]
    with pytest.raises(ValueError):
        r.set_base_velocity(0.1, 0, 0)
    with pytest.raises(ValueError):
        r.set_lift(0.1)


# ------------------------------------------------------------------ R-45
@pytest.mark.parametrize("thieu", [[13], [14], [15], [16], [13, 14], [15, 16], [13, 14, 15], [14, 15, 16]])
def test_r45_bus_b_thieu_mot_phan(thieu):
    ids = [i for i in ALL_IDS if i not in thieu]
    with pytest.raises(ConnectionError) as e:
        FeetechBackend(port=FakeServoPort(ids), allow_no_latch=True, **NOSLEEP)
    msg = str(e.value)
    assert "BUS_B" in msg
    seen = [i for i in BUS_B if i not in thieu]
    assert str(seen) in msg and str(BUS_B) in msg, msg      # ID thấy và ID cần


# ------------------------------------------------------------------ R-46
def test_r46_cau_hinh_chon_tay_khong_khop_bus():
    with pytest.raises(ConnectionError, match="tram_ban"):
        FeetechBackend(port=FakeServoPort(ALL_IDS), config="tram_ban", allow_no_latch=True, **NOSLEEP)
    ids = [i for i in ALL_IDS if i not in BUS_B]
    with pytest.raises(ConnectionError, match="di_dong"):
        FeetechBackend(port=FakeServoPort(ids), config="di_dong", allow_no_latch=True, **NOSLEEP)


@pytest.mark.parametrize("ten", ["xyz", "", "DI_DONG", "di dong", None, 3])
def test_r46_cau_hinh_la_bi_tu_choi(ten):
    with pytest.raises((ValueError, TypeError)):
        FeetechBackend(port=FakeServoPort(ALL_IDS), config=ten, allow_no_latch=True, **NOSLEEP)


# ------------------------------------------------------------------ R-47
def make_no_dock(**kw):
    return FeetechBackend(port=FakeServoPort(ALL_IDS), latch=RunLatch(), **NOSLEEP, **kw)


def test_r47_thieu_dock_tu_choi_enable_lift_de():
    be = make_no_dock()
    assert be.dock is None
    with pytest.raises(DockOpen):
        be.enable()
    assert not be.enabled
    with pytest.raises(DockOpen):
        be.set_base_velocity(0.1, 0, 0)
    with pytest.raises(DockOpen):
        be.write_joints({"lift": be._cmd["lift"] + 0.01})
    be.close()


def test_r47_allow_no_dock_canh_bao_mot_lan():
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        be = make_no_dock(allow_no_dock=True)
        be.enable()
        for _ in range(50):
            be.step(0.02)
        be.set_base_velocity(0.05, 0, 0)
        be.disable()
        be.enable()
        be.close()
    dem = [x for x in w if issubclass(x.category, RuntimeWarning) and "allow_no_dock" in str(x.message)]
    assert len(dem) == 1, [str(x.message)[:60] for x in dem]


# ------------------------------------------------------------------ R-48
def _env(pins: str, **kw):
    return {"FAIBOTIC_GPIO_PINS": pins, **kw}


GPIO_HONG = {
    "sai_cu_phap_thieu_bang": "K5_EN22,ESTOP_OK=17",
    "gia_tri_khong_phai_so": "K5_EN=abc,ESTOP_OK=17,ARM_SENSE=27,RESET_SENSE=23",
    "thieu_chan_bat_buoc": "K5_EN=22,ESTOP_OK=17",
    "trung_chan": "K5_EN=22,ESTOP_OK=22,ARM_SENSE=27,RESET_SENSE=23",
    "ngo_ra_tren_bcm_0_8": "K5_EN=4,ESTOP_OK=17,ARM_SENSE=27,RESET_SENSE=23",
    "ngoai_header": "K5_EN=22,ESTOP_OK=17,ARM_SENSE=27,RESET_SENSE=99",
    "chot_tren_bcm_0_8": "K5_EN=22,ESTOP_OK=17,ARM_SENSE=27,RESET_SENSE=23,LIFT_LATCH_EN=3,LIFT_LATCH_RETRACTED=25",
    "chot_trung_k5": "K5_EN=22,ESTOP_OK=17,ARM_SENSE=27,RESET_SENSE=23,LIFT_LATCH_EN=24,LIFT_LATCH_RETRACTED=22",
    "dock_trung_lift": "K5_EN=22,ESTOP_OK=17,ARM_SENSE=27,RESET_SENSE=23,LIFT_BOT=5,LIFT_TOP=6,DOCK_L=5,DOCK_R=26",
}


@pytest.mark.parametrize("ten", sorted(GPIO_HONG))
def test_r48_gpio_pins_hong_bi_tu_choi_co_kieu(ten, monkeypatch):
    """Cấu hình chân hỏng: lỗi có kiểu (GPIOConfigError, hoặc GPIOUnavailable khi chưa có thư viện), backend KHÔNG tạo được."""
    monkeypatch.setenv("FAIBOTIC_GPIO_PINS", GPIO_HONG[ten])
    for k in ("FAIBOTIC_GPIO_ACTIVE_LOW", "FAIBOTIC_GPIO_PULL", "FAIBOTIC_GPIO_CHIP", "FAIBOTIC_SAFETY_HW"):
        monkeypatch.delenv(k, raising=False)
    with pytest.raises((GPIOConfigError, GPIOUnavailable)):
        FeetechBackend(port=FakeServoPort(ALL_IDS), **NOSLEEP)


def test_r48_trung_chan_bi_bat_o_buoc_kiem_khong_phai_o_thu_vien():
    """Trùng chân phải bị chặn bởi kiểm khai báo (trước khi mở thư viện GPIO): thông báo nêu số BCM trùng."""
    with pytest.raises(GPIOConfigError, match="trùng"):
        PiGPIO({"K5_EN": 22, "ESTOP_OK": 22, "ARM_SENSE": 27, "RESET_SENSE": 23})
    with pytest.raises(GPIOConfigError, match="thiếu chân"):
        PiGPIO({"K5_EN": 22})
    with pytest.raises(GPIOConfigError):
        config_from_env(_env("K5_EN=22,ESTOP_OK"))


def test_r48_khong_khai_chan_thi_khong_co_chot_va_tu_choi_enable(monkeypatch):
    monkeypatch.delenv("FAIBOTIC_GPIO_PINS", raising=False)
    be = FeetechBackend(port=FakeServoPort(ALL_IDS), **NOSLEEP)
    assert be.latch is None
    with pytest.raises(Exception) as e:
        be.enable()
    assert "chốt" in str(e.value)
    assert not be.enabled


# ------------------------------------------------------------------ R-49
def cal(**sua) -> dict:
    """Hiệu chuẩn mặc định rồi áp sửa: 'khop.truong' = giá trị; giá trị DEL xoá trường, khoá 'khop' = DEL xoá khớp."""
    c = copy.deepcopy(default_calibration())
    for k, v in sua.items():
        if "." in k:
            j, f = k.split(".", 1)
            if v is DEL:
                c[j].pop(f)
            else:
                c[j][f] = v
        elif v is DEL:
            c.pop(k)
        else:
            c[k] = v
    return c


DEL = object()
CAL_HONG = {
    "lift_counts_per_m_0": ({"lift.counts_per_m": 0}, "lift"),
    "lift_counts_per_m_nan": ({"lift.counts_per_m": math.nan}, "lift"),
    "lift_counts_per_m_inf": ({"lift.counts_per_m": math.inf}, "lift"),
    "kep_open_bang_closed": ({"left_gripper.open": 2048}, "left_gripper"),
    "kep_nan": ({"right_gripper.open": math.nan}, "right_gripper"),
    "sign_2": ({"left_shoulder.sign": 2}, "left_shoulder"),
    "sign_0": ({"right_elbow.sign": 0}, "right_elbow"),
    "sign_chuoi": ({"left_pan.sign": "x"}, "left_pan"),
    "thieu_khop": ({"right_wrist_flex": DEL}, "right_wrist_flex"),
    "thieu_truong_zero": ({"left_elbow.zero": DEL}, "left_elbow"),
    "zero_nan": ({"left_wrist_roll.zero": math.nan}, "left_wrist_roll"),
    "zero_5000": ({"left_shoulder.zero": 5000}, "left_shoulder"),
    "zero_am": ({"right_shoulder.zero": -1}, "right_shoulder"),
    "khop_la": ({"khop_la_xyz": {"zero": 2048, "sign": 1}}, "khop_la_xyz"),
}


@pytest.mark.parametrize("ten", sorted(CAL_HONG))
def test_r49_hieu_chuan_hong_bi_tu_choi_ngay_khi_khoi_tao(ten):
    sua, khop = CAL_HONG[ten]
    with pytest.raises(ValueError) as e:
        FeetechBackend(port=FakeServoPort(ALL_IDS), calibration=cal(**sua), allow_no_latch=True, **NOSLEEP)
    assert khop in str(e.value)


def test_r49_hieu_chuan_mac_dinh_hop_le_van_chay():
    be = FeetechBackend(port=FakeServoPort(ALL_IDS), calibration=cal(), dock=dock_ok(), allow_no_latch=True, **NOSLEEP)
    assert all(math.isfinite(v) for v in be.read_joints().values())
    be.close()


# ------------------------------------------------------------------ R-50
def test_r50_bang_id_trung():
    ids = dict(spec.SERVO_IDS)
    ids["right_pan"] = ids["left_pan"]
    with pytest.raises(ValueError) as e:
        FeetechBackend(port=FakeServoPort(ALL_IDS), ids=ids, allow_no_latch=True, **NOSLEEP)
    assert "left_pan" in str(e.value) and "right_pan" in str(e.value)




# ------------------------------------------------------------------ R-51
def test_r51_servo_sai_model():
    port = FakeServoPort(ALL_IDS)
    sid = spec.SERVO_IDS["lift"]
    port.mem[sid][3], port.mem[sid][4] = 0x34, 0x12            # model lạ ở servo lift
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        try:
            FeetechBackend(port=port, dock=dock_ok(), allow_no_latch=True, **NOSLEEP)
            loi = None
        except (ConnectionError, ValueError) as e:
            loi = str(e)
    nd = loi or " ".join(str(x.message) for x in w)
    assert "lift" in nd and ("model" in nd.lower() or "4660" in nd or "0x1234" in nd.lower()), nd
