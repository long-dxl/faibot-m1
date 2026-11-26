import json
import subprocess
import sys
from pathlib import Path

import pytest

from faibotic import spec
from static_torque import reach_map, static_torque

ROOT = Path(__file__).resolve().parents[1]


def test_check_demo_passes():
    p = subprocess.run([sys.executable, str(ROOT / "demos/check_demo.py")], capture_output=True, text=True)
    res = json.loads(p.stdout)
    assert res["pass"], res
    assert res["dist_xy_m"] <= 0.03 and res["bad_contact_steps"] == 0 and res["nan_steps"] == 0
    assert p.returncode == 0


def test_static_torque_fully_extended_under_stall():
    """P-L (S1C): duỗi hết, tải 0,5 kg/tay, khối lượng cận trên: mô-men tĩnh mọi khớp <= 60% mô-men chặn (= TORQUE_LIMIT), không cần lò xo."""
    t = static_torque(spec.PAYLOAD_TARGET)
    for j, row in t.items():
        assert row["torque_kgcm"] <= row["stall_kgcm"], (j, row)
        assert row["stall_ratio"] <= 0.65, (j, row)
    print(json.dumps(t, ensure_ascii=False))
    assert t["shoulder"]["torque_nm"] > t["elbow"]["torque_nm"] > t["wrist_flex"]["torque_nm"]


SOR002_PL = ("P-L (vai và khuỷu một STS3095, không lò xo): duỗi hết, tải 0,5 kg cần vai ~60 kg·cm = 267% định mức 22,5 "
             "(không tải đã 33 kg·cm). SOR-002 (<= 80% định mức) không đạt; số và tải công bố: docs/M1-LITE/tai-cong-bo-PL.md")


@pytest.mark.xfail(strict=True, reason=SOR002_PL)
def test_sor002_shoulder_under_80pct_rated():
    """SOR-002: mô-men vai duỗi hết (khuỷu, cổ tay = 0), tải 0,5 kg, mọi góc vai trong giới hạn <= 80% định mức servo vai.
    P-A (ghép đôi + lò xo) đạt với 2 x STS3095; P-L không đạt (xfail strict: báo ngay nếu mô hình đổi làm test đạt)."""
    import math
    lo, hi = spec.ARM_LIMITS["shoulder"]
    worst = max(static_torque(spec.PAYLOAD_TARGET, q_arm=[0.0, math.radians(d), 0.0, 0.0, 0.0])["shoulder"]["rated_ratio"]
                for d in range(int(math.degrees(lo)), int(math.degrees(hi)) + 1, 5))
    assert worst <= 0.8, worst


@pytest.mark.xfail(strict=True, reason=SOR002_PL)
def test_elbow_under_80pct_rated():
    """Khuỷu một STS3095: duỗi hết, tải 0,5 kg <= 80% định mức 22,5 kg·cm."""
    t = static_torque(spec.PAYLOAD_TARGET)["elbow"]
    assert t["servo_count"] == 1 and t["rated_ratio"] <= 0.8, t


def test_pl_peak_under_torque_limit_grid():
    """Thay SOR-002 cho P-L: lưới tư thế hợp lệ (vai -80..48,5 x khuỷu x cổ tay, trong giới hạn ghép), tải 0,3 kg (tải đỉnh kẹp ngang
    công bố được tới 0,44 kg ở tầm lớn nhất): vai, khuỷu, gập <= 60% mô-men chặn (TORQUE_LIMIT, servo không cắt)."""
    import math
    worst = {"shoulder": 0.0, "elbow": 0.0, "wrist_flex": 0.0}
    for qs in range(-80, 49, 16):
        for qe in (-10, 30, 60, 90, 120, 127):
            if math.radians(qe) > spec.elbow_max(math.radians(qs)) + 1e-9:
                continue
            for qw in (-77, -45, 0, 45, 87):
                if qw > math.degrees(spec.wrist_max(math.radians(qs), math.radians(qe))) + 1e-9:
                    continue
                t = static_torque(0.3, q_arm=[0.0, math.radians(qs), math.radians(qe), math.radians(qw), 0.0])
                for j in worst:
                    worst[j] = max(worst[j], t[j]["stall_ratio"])
    assert all(v <= 0.60 for v in worst.values()), worst


def test_pl_load_table_matches_doc():
    """Bảng tải công bố (docs/M1-LITE/tai-cong-bo-PL.md) sinh từ demos/static_torque.py --md: file phải chứa đúng bảng hiện hành."""
    from static_torque import load_table, markdown
    doc = (ROOT / "docs/M1-LITE/tai-cong-bo-PL.md").read_text(encoding="utf-8")
    assert markdown(load_table()) in doc
    rows = [r for r in load_table() if r["reachable"]]
    full = rows[0]
    assert full["pose"] == "duỗi hết" and full["tren"]["peak_kg"] <= 0.6     # trần kẹp ngang 0,6 kg (QC2 kẹp cũ; kẹp AM-ARM200: 0,50 kg)


def test_joint_limits_v03():
    """Giới hạn v0.4.1 vòng 2: vai >= -80 (đệm chạm -80,5), khuỷu >= -10, khuỷu <= 130, nên giới hạn ghép khuỷu <= 90 khi vai < -85 không còn tới được; Robot kẹp vai về -80."""
    import math
    assert math.degrees(spec.ARM_LIMITS["shoulder"][0]) == -80.0 and abs(math.degrees(spec.ARM_LIMITS["elbow"][1]) - 127.0) < 1e-9
    # ràng buộc v0.3 (vai < -85 thì khuỷu <= 90) đã bỏ khỏi spec (mã chết vì vai >= -80): khuỷu <= 130 suốt dải vai thấp
    assert not hasattr(spec, "SHOULDER_COUPLE_DEG")
    # bảng quét theo cuộn: vai -80 chỉ bị tự va chạm (87; trần gập 70 từ 24/11 trên cẳng tay J4 upstream), khuỷu 127: 84,5; v0.4.5 (DK-W4, camera + giá M1-AR-009): 74,5
    assert abs(spec.wrist_max(math.radians(-80), math.radians(100)) - math.radians(70)) < 1e-12 and abs(spec.wrist_max(math.radians(-80), math.radians(127)) - math.radians(70)) < 1e-12
    from faibotic import Robot
    r = Robot("sim")
    r.keepout = False   # G1/E3: test kẹp giới hạn khớp; tư thế cực nằm trong vùng cấm bệ (kiểm ở test_tram)
    out = r.set_joint_positions({"left_shoulder": math.radians(-90), "left_elbow": math.radians(120)})
    assert out["left_shoulder"] >= math.radians(-80) - 1e-9 and out["left_elbow"] <= math.radians(127) + 1e-9
    r.close()
    assert abs(math.degrees(spec.ARM_LIMITS["elbow"][0]) + 10.0) < 1e-9
    # tư thế home (vai -80, khuỷu 90) phải hợp lệ
    assert spec.HOME_ARM["wrist_flex"] <= spec.wrist_max(spec.HOME_ARM["shoulder"], spec.HOME_ARM["elbow"]) + 1e-9


def test_reach_table_heights_060_to_080():
    rm = reach_map()
    for h in ("0.60", "0.70", "0.80"):
        assert rm[h] is not None, h
        lo, hi = rm[h]["depth_from_edge_m"]
        assert hi - lo >= 0.10, (h, rm[h])  # với được ít nhất dải 10 cm tính từ mép bàn


@pytest.mark.xfail(strict=True, reason="24/11: trần gập cổ tay 87,5 -> 70 (đáy vỏ cổ tay chạm cẳng tay J4 ở 73,4): bàn 0,85 m chỉ còn dải 5,5 cm (0,055-0,11 m) ở lift 0,45; trước 13 cm")
def test_reach_table_height_085():
    """kẹp AM-ARM200 ngắn hơn kẹp cũ 32 mm: bàn 0,85 m với được 0,055-0,185 m từ mép (S1G với kẹp cũ: 7 cm, xfail). Trần gập 70: 0,055-0,11 m."""
    rm = reach_map((0.85,))
    lo, hi = rm["0.85"]["depth_from_edge_m"]
    assert hi - lo >= 0.10, rm


@pytest.mark.xfail(strict=True, reason="P-L: trục vai đỉnh 1,012 m + vai >= -80: bàn 0,90 m chỉ còn dải 6,5 cm (0,07-0,135 m) ở lift 0,45 (kẹp AM-ARM200; kẹp cũ 5,5 cm)")
def test_reach_table_height_090():
    rm = reach_map((0.90,))
    lo, hi = rm["0.90"]["depth_from_edge_m"]
    assert hi - lo >= 0.10, rm


def test_selftest_script():
    p = subprocess.run([sys.executable, str(ROOT / "demos/selftest.py")], capture_output=True, text=True)
    assert p.returncode == 0, p.stdout + p.stderr
