"""Mất điện: thả tay (servo tắt, đệm PU tác dụng) từ lưới tư thế; năng lượng/tốc độ/mô-men đỉnh tới từng chặn, so khả năng hấp thụ.
v0.4.1: quét lỗi đơn (lò xo đứt một/cả hai = hệ số 0, lò xo x2; tải 0 và 0,5 kg; ma sát mô hình/0,3x/0), khuỷu ngược, ngưỡng Coulomb.
v0.5: trục nâng thanh răng không tự hãm: thêm mục "lift_drop" (rơi tối đa một nấc chốt, poweroff.lift_drop / lift_drop_hand).

    python demos/poweroff_drop.py            # in JSON (đủ ma trận, vài phút)
    python demos/poweroff_drop.py --write    # ghi cad/export/poweroff.json (dùng cho cad/bao-cao-cad.md v0.4.1)
    python demos/poweroff_drop.py --lift     # chỉ trục nâng rơi một nấc chốt (v0.5)
"""
from __future__ import annotations

import json
import sys
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))

from faibotic import poweroff as po, spec  # noqa: E402

CASES = [("mo_hinh_sim", 1.0, None), ("ma_sat_0,3x", 0.3, None), ("khong_ma_sat_can_tren", 0.0, 0.0)]
SPRINGS = [("lo_xo_1", 1.0), ("lo_xo_dut_1_(0,5)", 0.5), ("lo_xo_dut_ca_hai_(0)", 0.0), ("lo_xo_x2_(kẹt/căng)", 2.0)]
GRID = dict(shoulders=(-80, -55, -30, -5, 20, 49), elbows=range(-10, 131, 28))
FRICTIONS = [0.05, 0.2, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]   # N·m mỗi khớp, Coulomb thuần (không giảm chấn nhớt), tải 0,5 kg
PL = (0.0, spec.PAYLOAD_TARGET)


def _fmt(w, cap):
    return {k: dict(E_J=round(v["E_J"], 3), v_tcp=round(v["v_tcp"], 2), tau_nm=round(v["tau_nm"], 2), hard=bool(v["hard"]), start=v["start"],
                    cap_J=round(cap.get(k, 0.0), 3), ok=bool(v["tau_nm"] <= spec.FAILSAFE_T_GEAR_NM and not v["hard"])) for k, v in w.items()}


def _job(a):
    kind, name, ds, fl, sp, pl, es = a
    cap = po.stop_capacity()
    return kind, name, _fmt(po.sweep(po.build_model(ds, fl, pl, sp), e_scale=es, **GRID), cap)


def run(workers: int = 2) -> dict:
    cap = po.stop_capacity()
    jobs = []
    for sn, sp in SPRINGS:
        for name, ds, fl in CASES:
            for pl in PL:
                jobs.append(("cases", f"{name}|{sn}|tai={pl}", ds, fl, sp, pl, 1.0))
    for es in (0.8, 1.2):       # dung sai độ cứng đệm +-20% (Shore +-5 điểm)
        for pl in PL:
            jobs.append(("pad_tol", f"mo_hinh_sim|lo_xo_dut_ca_hai_(0)|tai={pl}|E_dem_x{es}", 1.0, None, 0.0, pl, es))
    for fr in FRICTIONS:
        jobs.append(("friction_sweep", str(fr), 0.0, fr, 1.0, spec.PAYLOAD_TARGET, 1.0))
    with Pool(workers) as p:
        res = p.map(_job, jobs)
    out = {"capacity_J": {k: round(v, 3) for k, v in cap.items() if k in spec.FAILSAFE_PADS}, "T_allow_nm": cap["T_allow_nm"], "link_alu": cap["link_alu"],
           "pad_cap_detail": {k: {a: (round(b, 3) if isinstance(b, float) else b) for a, b in po.pad_capacity(k).items()} for k in spec.FAILSAFE_PADS},
           "pads": spec.FAILSAFE_PADS, "static_hold_nm": po.static_hold_torque(), "cases": {}, "pad_tol": {}, "friction_sweep": {}}
    for kind, name, r in res:
        out[kind][name] = r
    out["lift_drop"] = lift_drop_cases()
    return out


def lift_drop_cases() -> dict:
    """v0.5: E-stop khi lift đứng yên, hạ ở LIFT_SPEED_MAX và ở ~65 mm/s [R2 7.2, tự suy ra]; mô hình MJCF và tính tay với tải thiết kế."""
    out = {"notch_m": spec.LIFT_NOTCH_PITCH, "design_load_kg": spec.LIFT_DESIGN_LOAD_KG, "cases": {}}
    for v0 in (0.0, spec.LIFT_SPEED_MAX, 0.065):
        sim = po.lift_drop(v0=v0)
        hand = po.lift_drop_hand(spec.LIFT_DESIGN_LOAD_KG, v0)
        out["cases"][f"v0={v0}"] = {"mjcf": {k: round(v, 4) for k, v in sim.items()},
                                    "tay_tai_thiet_ke": {k: round(v, 4) for k, v in hand.items()}}
    return out


if __name__ == "__main__":
    if "--lift" in sys.argv:          # chỉ phần trục nâng (nhanh)
        print(json.dumps(lift_drop_cases(), ensure_ascii=False, indent=1))
        sys.exit(0)
    res = run()
    txt = json.dumps(res, ensure_ascii=False, indent=1, default=str)
    if "--write" in sys.argv:
        (ROOT / "cad/export/poweroff.json").write_text(txt, encoding="utf-8")
    print(txt)
