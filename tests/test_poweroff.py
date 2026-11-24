"""v0.4.1: an toàn mất điện vai + khuỷu. Servo tắt, lưới tư thế xấu, đệm PU có đường lực-ép (poweroff.pad_force) và giới hạn cứng.
Số ma sát/giảm chấn khớp, độ cứng đệm, giới hạn bánh răng 9,3 N·m đều là giả định chưa đo (cad/bao-cao-cad.md v0.4.1).
Test đạt = ở "mô hình sim" và các lỗi đơn (lò xo đứt, tải 0,5 kg), mô-men đỉnh lên khớp <= 9,3 N·m và không chạm giới hạn cứng.
Ca cần số đo phần cứng (ma sát thấp, ngưỡng quay ngược) hoặc thiết kế còn hở là xfail strict có chủ đích."""
import json
import math
from pathlib import Path

import pytest

from faibotic import poweroff as po, spec

# S1C (P-L): tay AM-ARM200 gốc không lò xo, không đệm PU/chặn của P-A; mô hình mất điện ở đây vẫn dùng đệm P-A (spec.FAILSAFE_PADS)
# lên tay P-L, nên chỉ là chỉ báo. Phân tích mất điện P-L (chặn cơ khí thật của AM-ARM200, có cần đệm không) CHƯA LÀM.
PL_POWEROFF = ("P-L (S1C): tay không lò xo, cẳng tay + kẹp nặng hơn: vượt ngưỡng mô hình P-A (vai trên 13,75 N·m > 9,3 ở ngưỡng khuỷu 3,0; "
               "giữ tĩnh cần 5,5 N·m quay ngược > 4,0). Chưa có phân tích mất điện P-L: mở, không nới ngưỡng")

ROOT = Path(__file__).resolve().parents[1]
GRID = dict(shoulders=(-80, -45, -10, 25, 49), elbows=(-10, 30, 80, 130))
T_GEAR = spec.FAILSAFE_T_GEAR_NM
PAD_KEYS = tuple(spec.FAILSAFE_PADS)
_cache: dict = {}


def worst(ds, fl, payload, spring):
    k = (ds, fl, payload, spring)
    if k not in _cache:
        _cache[k] = po.sweep(po.build_model(ds, fl, payload, spring), **GRID)
    return _cache[k]


def pad_problems(w, keys=PAD_KEYS):
    """Chặn bị vượt: mô-men đỉnh > giới hạn bánh răng hoặc chạm giới hạn cứng (chỉ các đệm, không tính gá khuỷu đập chặn dưới)."""
    return {k: (round(float(v["tau_nm"]), 2), v["hard"]) for k, v in w.items() if k in keys and (v["tau_nm"] > T_GEAR or v["hard"])}


def real_hits(w):
    """Va chạm thật (động năng >= 1 mJ); bỏ trượt vài phần nghìn độ khi Coulomb vừa đủ giữ (E ~ 0,1-0,3 mJ)."""
    return {k: round(v["E_J"], 3) for k, v in w.items() if v["E_J"] >= 1e-3}


NO_REV = PAD_KEYS   # v0.4.1 vòng 2: chặn khuỷu ngược M1-AR-023 có hành trình 8,5 độ, đạt như các chặn khác


@pytest.fixture(scope="module")
def scan():
    return json.loads((ROOT / "cad/export/failsafe_scan.json").read_text())


@pytest.fixture(scope="module")
def scan42():
    return json.loads((ROOT / "cad/export/failsafe_v042_scan.json").read_text())


@pytest.fixture(scope="module")
def scan42low():
    return json.loads((ROOT / "cad/export/failsafe_v042_low_scan.json").read_text())["B3_r45_qc50_h62"]


@pytest.fixture(scope="module")
def scan41():
    return json.loads((ROOT / "cad/export/failsafe_v041_scan.json").read_text())


def test_no_hard_intersection_in_mechanical_range(scan):
    """Mọi ô lưới CAD trong hộp cơ khí [chặn vai trên, chặn dưới] x [khuỷu ngược, khuỷu 130] không giao cứng (đệm PU được phép ép)."""
    lo, hi = spec.FAILSAFE_SHOULDER_STOP_DEG, spec.FAILSAFE_SHOULDER_UP_DEG
    elo, ehi = spec.FAILSAFE_ELBOW_LOW_DEG, 130
    cells = [r for r in scan["rows"] if "q_khuyu" in r and lo <= r["q_vai"] <= hi and elo <= r["q_khuyu"] <= ehi]
    assert len(cells) > 150                                   # quét đủ dày
    bad = [(r["q_vai"], r["q_khuyu"], r["chi_tiet"]) for r in cells if r["giao_cung_mm3"] > 0]
    assert not bad, bad
    for r in scan["rows"]:
        if r.get("kind") == "vai" and lo <= r["q_vai"] <= hi:
            assert r["link_cradle"] == 0 and r["link_bar"] == 0 and r["pin_cradle"] == 0 and r["pin_bar"] == 0 and r["pin_servo"] == 0, r


def test_elbow_contact_with_link_is_the_hard_stop(scan):
    """Khuỷu 140: gá khuỷu chạm link (45,7 mm3), 150: giao 1.475 mm3. Đệm M1-AR-022 (dựng CAD v0.4.1) phải thắng trước."""
    assert spec.FAILSAFE_ELBOW_STOP_DEG < 138 < 140
    r140 = [r for r in scan["rows"] if r.get("q_khuyu") == 140 and r["q_vai"] in (-85, 0, 90)]
    assert r140 and all(0 < r["giao_cung_mm3"] < 100 for r in r140)


def test_pad_engages_after_soft_limit_and_not_overcompressed(scan):
    # giới hạn mềm vai nằm trước chân đệm, đệm ép hết cỡ ở chặn không quá 50% bề dày 16 mm
    assert spec.ARM_LIMITS["shoulder"][0] >= math.radians(spec.FAILSAFE_SHOULDER_CONTACT_DEG)
    dep = {float(k): v for k, v in scan["pad_depth"].items()}
    assert dep[-85.0] == 0 and dep[-85.5] > 0
    row = next(r for r in scan["rows"] if r.get("kind") == "vai" and r["q_vai"] == spec.FAILSAFE_SHOULDER_STOP_DEG)
    assert row["link_dem_mm3"] / (8.0 * 16.0) <= 8.0          # mm ép = thể tích / (8 mm dày link x 16 mm cao đệm)


def test_v041_new_stops_have_no_hard_intersection(scan41):
    """Tab/đệm khuỷu M1-AR-022/023 không giao cứng với gá vai và thanh chặn (chặn dưới cũ M1-SH-017 v1/v2 đã thay bằng v3, test_v042_low_stop_*)."""
    assert all(r["tab_cradle"] == 0 and r["tab_bar"] == 0 for r in scan41["tab_vs_gavai"])
    k = {r["q_khuyu"]: r for r in scan41["khuyu"]}
    assert all(k[q]["chan_nguoc_cung"] == 0 for q in k)                           # tab nhôm không bị gá khuỷu chạm trước đệm
    v43 = {r["q_khuyu"]: r["dem_ar022"] for r in json.load(open(Path(__file__).parent.parent / "cad" / "export" / "failsafe_v043_scan.json"))["khuyu_dem"]}
    assert v43[spec.ELBOW_SOFT_MAX_DEG] == 0 and v43[127.5] == 0 and v43[128.1] > 0 and v43[133] > 0     # v0.4.3: đệm (PU 7 + đế 5) chạm ở 128, sau giới hạn mềm 127
    assert spec.FAILSAFE_PADS["elbow_low"]["contact_deg"] < math.degrees(spec.ARM_LIMITS["elbow"][0])


def test_v042_soft_limit_stops(scan42):
    """giới hạn mềm vai -80 / khuỷu ngược -10. Đệm vai trên (dày 20) chạm link từ -80,5 và không giao cứng thanh/gá khuỷu; đệm khuỷu ngược chạm từ ~-23,5, tab nhôm không bị chạm."""
    assert math.degrees(spec.ARM_LIMITS["shoulder"][0]) == -80.0 and abs(math.degrees(spec.ARM_LIMITS["elbow"][0]) + 10.0) < 1e-9
    rows = {r["q_vai"]: r for r in scan42["vai_tren"]}
    assert rows[-80.5]["link_dem"] == 0 and rows[-82]["link_dem"] > 0 and rows[-90]["link_dem"] > rows[-88]["link_dem"] > rows[-85]["link_dem"]
    assert all(r["link_thanh"] == 0 and all(v == 0 for v in r["khuyu_vs_thanh"].values()) and all(v == 0 for v in r["khuyu_vs_dem"].values()) for r in rows.values())
    kr = {r["q_khuyu"]: r for r in scan42["khuyu_nguoc"]}
    assert all(r["tab_cung"] == 0 for r in kr.values())
    assert kr[-20]["dem_ar023"] == 0 and kr[-24]["dem_ar023"] > 0 and kr[-10]["dem_ar023"] == 0 and kr[0]["dem_ar023"] == 0
    assert all(kr[q]["dem_ar023"] < 10 for q in (120, 130, 131, 136))               # đệm chỉ sượt gá khuỷu khi gập (< 10 mm3)


def test_v042_low_stop_link_hits_pad_first(scan42low):
    """-2: chặn dưới dời xuống đáy má (M1-SH-017 v3 + đệm M1-SH-018 v3, r 45): link chạm đệm từ vai 50 (sau giới hạn mềm 49,5) và KHÔNG giao cứng giá, chốt, lò xo, gá vai, servo, thanh trên."""
    assert math.degrees(spec.ARM_LIMITS["shoulder"][1]) < spec.FAILSAFE_PADS["shoulder_up"]["contact_deg"] == scan42low["qc"] == 50.0
    rows = {r["q_vai"]: r for r in scan42low["link_gia"]}
    assert all(r["link_gia"] == r["pin"] == r["spring"] == r["gia_cradle"] == r["gia_servo"] == r["gia_thanh_tren"] == 0 for r in rows.values()), rows
    assert rows[50.0]["link_dem"] == 0 and rows[52.0]["link_dem"] > 0 and rows[62.0]["link_dem"] > rows[54.0]["link_dem"]
    assert scan42low["hard"] == spec.FAILSAFE_SHOULDER_UP_DEG == spec.FAILSAFE_PADS["shoulder_up"]["hard_deg"]


def test_v042_low_stop_elbow_never_strikes_before_pad(scan42low):
    """Mọi góc khuỷu cơ khí tới 133 độ x mọi vai -80..62: gá khuỷu 0 mm3 với giá chặn và đệm (lưới 8 độ + lưới mịn 124-133). Khuỷu 136 (cứng, sau đệm khuỷu 131) còn giao 69-464 mm3: ghi là hở."""
    g = scan42low["gakhuyu_vs_gia"]
    assert g["so_o_giao"] > 0 and all(c[1] == 136 for c in g["cac_o_giao"]), g["cac_o_giao"]
    fine = scan42low["gakhuyu_min"]
    assert all(v == 0 for k, v in fine.items() if float(k.split("|")[1]) <= 133), {k: v for k, v in fine.items() if v}
    assert any(v > 100 for k, v in fine.items() if k.endswith("|136"))
    assert spec.FAILSAFE_ELBOW_STOP_DEG == 133.0                                      # v0.4.3: chặn cứng khuỷu 133 nên ô 136 (hở) không còn tới được


@pytest.mark.parametrize("spring,payload", [(1.0, 0.0), (1.0, spec.PAYLOAD_TARGET), (0.5, spec.PAYLOAD_TARGET), (0.0, spec.PAYLOAD_TARGET), (2.0, spec.PAYLOAD_TARGET)])
def test_poweroff_pads_hold_at_model_friction_incl_single_faults(spring, payload):
    """Mô hình sim (damping 1,5 N·m·s/rad, ma sát 0,05 N·m [giả định]); lò xo bình thường, đứt một (0,5), đứt cả hai (0), kẹt/căng x2; tải 0 và 0,5 kg:
    mô-men đỉnh đệm lên khớp <= 9,3 N·m [giả định] và không chạm giới hạn cứng, cả bốn chặn (vai trên/dưới, khuỷu trước/ngược)."""
    w = worst(1.0, None, payload, spring)
    assert w and set(PAD_KEYS) & set(w)
    assert not pad_problems(w, NO_REV), pad_problems(w, NO_REV)


def test_pad_capacity_is_consistent_with_force_curve():
    """Khả năng hấp thụ tính từ đường lực-ép tới 9,3 N·m: vai trên ~0,35 J (hành trình 8,8 mm, đệm dày 20), vai dưới (v0.4.2: chạm 50, cứng 62, r 45, đệm 20 x 96 mm2) ~0,70 J, khuỷu trước ~0,20 J,
    khuỷu ngược ~0,34 J (70A). Mô-men ở cuối luôn <= 9,3 N·m [giả định]."""
    for k in PAD_KEYS:
        assert po.pad_capacity(k)["tau_nm"] <= T_GEAR * 1.01, k
    cap = po.stop_capacity()
    assert 0.30 < cap["shoulder_low"] < 0.40 and 0.60 < cap["shoulder_up"] < 0.80 and 0.17 < cap["elbow_high"] < 0.23 and 0.28 < cap["elbow_low"] < 0.40
    assert not po.pad_capacity("shoulder_up")["bottom"] and not po.pad_capacity("elbow_high")["bottom"] and not po.pad_capacity("shoulder_low")["bottom"]
    for k in PAD_KEYS:      # dung sai độ cứng +-20% (Shore +-5): hấp thụ không đổi quá 40%
        base = po.pad_capacity(k)["cap_J"]
        for sc in (0.8, 1.2):
            assert abs(po.pad_capacity(k, e_scale=sc)["cap_J"] - base) / base < 0.4, (k, sc)


def test_pad_peak_force_matches_hand_calc():
    """Đối chiếu tay đệm vai trên 50A (Gent E 2,46 MPa), bắt 8 x 6 = 48 mm2, dày 20: ép 8,8 mm (e 0,44) -> lực = E/3 (1/(1-e)^2 - (1-e)) A ~ 103 N = 9,3 N·m."""
    assert abs(po.gent_E_mpa(50) - 2.46) < 0.05 and abs(po.gent_E_mpa(70) - 5.5) < 0.1
    e = 8.8 / 20.0
    expect = 2.46 / 3 * (1 / (1 - e) ** 2 - (1 - e)) * 48.0
    f = po.pad_force("shoulder_low", 8.8e-3)
    assert abs(f - expect) / expect < 0.02 and 8.8 < f * 0.09 < 9.8, (f, expect)


@pytest.mark.xfail(strict=True, reason=PL_POWEROFF)
def test_poweroff_static_hold_when_backdrive_torque_enough():
    """Nếu mô-men quay ngược mỗi khớp >= mô-men không cân bằng (vai 2,96 N·m, khuỷu), tay không dịch chuyển: không chạm đệm nào."""
    h = po.static_hold_torque()
    assert 2.0 < h["hold_up_nm"] < 4.0 and 2.0 < h["hold_down_nm"] < 4.0
    w = worst(0.0, 3.3, spec.PAYLOAD_TARGET, 1.0)
    assert pad_problems(w) == {} and not real_hits(w), w     # trượt vài phần nghìn độ, không phải rơi


# (cổ tay theo URDF upstream, 23/11/2026): khuỷu 2,5 N·m nay giữ được ở sim (đệm khuỷu trước 0,3 mJ < 1 mJ, 0,98 N·m; trước 0,11 J ở 3,8 N·m, xfail).
# Vẫn là số mô hình: ngưỡng thật chờ số đo quay ngược STS3095.
# (22/11/2026): cổ tay xuất phát nay kẹp trong giới hạn mềm -77,5..87,5 (trước không kẹp, lưới -100/120): ở 2,5 N·m tư thế (48, -9,5, -77) chạm đệm vai trên
# 4,7 mJ ở 1,05 N·m (vượt ngưỡng va chạm 1 mJ, xa giới hạn bánh răng 9,3): ngưỡng mô hình về lại 3,0 N·m (c: vách ngưỡng sát).
@pytest.mark.parametrize("coulomb", [pytest.param(3.0, marks=pytest.mark.xfail(strict=True, reason=PL_POWEROFF)),
                                     pytest.param(2.5, marks=pytest.mark.xfail(strict=True, reason="cổ tay xuất phát -77 (giới hạn mềm mới), vai 48, khuỷu -9,5: đệm vai trên 4,7 mJ ở 1,05 N·m; v0.4.5 cộng ống cẳng tay 81,9 g: chạm cứng vai trên 0,116 J (trước 0,005 J), khuỷu trên 0,006 J; chờ số đo quay ngược")),
                                     pytest.param(2.0, marks=pytest.mark.xfail(strict=True, reason="chưa có số liệu số đo quay ngược: khuỷu 2,0 N·m: đệm M1-AR-022 nhận 0,38 J ở 5,6 N·m (đạt mô-men) và đệm vai trên 0,015 J (trước: 0,64 J ở 6,9 N·m, chặn dưới 0,01 J); không còn gá khuỷu đập chặn dưới từ v0.4.2"))])
def test_poweroff_elbow_threshold_sweep(coulomb):
    """Ngưỡng khuỷu (ma sát Coulomb thuần, tải 0,5 kg): 3,0 N·m tay đứng yên; 2,5 (đạt ở, hụt) và 2,0 cần phần cứng đo (xfail). Ghi số trong bao-cao-cad.md v0.4.1."""
    w = worst(0.0, coulomb, spec.PAYLOAD_TARGET, 1.0)
    assert not pad_problems(w) and not real_hits(w), w


def test_poweroff_folded_elbow_does_not_strike_lowstop():
    """-2 đóng: chặn dưới dời (link chạm đệm trước); ở ma sát mô hình không còn ca gá khuỷu đập giá (0 J tiếp xúc cứng)."""
    assert "lowstop_hit" not in worst(1.0, None, spec.PAYLOAD_TARGET, 1.0)


@pytest.mark.xfail(strict=True, reason="chưa có số liệu: nếu ma sát khớp chỉ bằng 0,3 lần giả định (0,45 N·m·s/rad; 0,015 N·m) tải 0,5 kg, lò xo đứt một thì chặn vai trên (đệm chặn dưới) vượt: 13,9 N·m, chạm cứng 3,4 J (poweroff.json sinh lại v0.4.3 vòng 2; số cũ 11,9 N·m ở vai dưới là của v0.4.1); "
                                       "đóng bằng số đo mô-men quay ngược STS3095, hoặc phương án b/d")
def test_poweroff_low_friction_needs_hardware_data():
    assert not pad_problems(po.sweep(po.build_model(0.3, None, spec.PAYLOAD_TARGET, 0.5), shoulders=(-80, -55, -30, -5, 20, 49), elbows=range(-10, 131, 28)))


@pytest.mark.xfail(strict=True, reason="chưa có số liệu: lò xo kẹt/căng (x2) cùng ma sát 0,3 lần: khuỷu trước 21,3 N·m (vượt 9,3), vai dưới 5,5 N·m (đạt) (matrix poweroff.json sinh lại v0.4.3 vòng 2; 20,4/10,6 là số cũ)")
def test_poweroff_spring_double_at_low_friction_needs_hardware_data():
    assert not pad_problems(po.sweep(po.build_model(0.3, None, spec.PAYLOAD_TARGET, 2.0), shoulders=(-80, -55, -30, -5, 20, 49), elbows=range(-10, 131, 28)))


# ---------------- v0.5: trục nâng thanh răng không tự hãm, chốt đỡ trong một nấc ----------------
def test_lift_locked_only_in_arm_model_not_self_locking():
    """v0.4 khoá lift vì vitme tự hãm; v0.5 chỉ khoá trong mô phỏng tay (chốt đỡ <= 1 nấc), lift_free=True thả lift."""
    assert "lift" not in po._LOCK and po._LOCK_LIFT_FOR_ARM_SIM == ("lift",)
    a = po.build_model().joint("lift").dofadr[0]
    assert po.build_model().dof_frictionloss[a] >= 1e5
    assert po.build_model(lift_free=True).dof_frictionloss[a] < 1e3


@pytest.mark.parametrize("v0", [0.0, spec.LIFT_SPEED_MAX, 0.065])
def test_lift_drop_one_notch_energy_matches_hand_calc(v0):
    """Không giảm chấn: động năng lúc chạm chốt = m g d + ½ m v0² (bảo toàn năng lượng, sai số bước tích phân <= 3%).
    Có giảm chấn khớp của MJCF: không vượt cận trên. d = một bước răng mô-đun 1,5 [giả định]."""
    free = po.lift_drop(v0=v0, damping_scale=0.0)
    assert free["drop_m"] == pytest.approx(spec.LIFT_NOTCH_PITCH)
    assert free["E_J"] == pytest.approx(free["E_hand_J"], rel=0.03)
    damped = po.lift_drop(v0=v0)
    assert damped["E_J"] <= free["E_hand_J"] * 1.01


def test_lift_drop_design_load_hand_calc_r2():
    """R2 7.2: tải 6,1 kg, E-stop khi hạ ~65 mm/s [tự suy ra], rơi một nấc: ~0,28 J thế năng + 0,013 J động năng.
    Số cho phép của chốt/răng: Chưa có số liệu (đo thử rơi DK-N4), test chỉ giữ phép tính."""
    h = po.lift_drop_hand(6.1, 0.065)
    assert h["E_pot_J"] == pytest.approx(6.1 * 9.81 * math.pi * 1.5e-3)
    assert h["E_kin0_J"] == pytest.approx(0.5 * 6.1 * 0.065 ** 2)
    assert h["E_J"] < 0.31
