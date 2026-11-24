"""Hệ số chống lật động cho giới hạn đế (D-E2). Dùng mô hình và khối lượng của cad/faibotic/tipover.py (M1-PL-masses.json),
thêm gia tốc của từng khối khi đế vừa tăng tốc vừa quay:

    a_i = a + α × p_i − ω² p_i        (a: gia tốc tịnh tiến đều, gồm cả ω × v khi đi cong)

Ca xấu nhất (thận trọng hơn assembly_pl `gia_toc_0_5_lift_thap_cao`): lift ở đỉnh (xe nâng + trạm nâng theo LIFT_RANGE[1], tay ở cao
trục vai), khay 1,3 kg ở góc xa (khay 0,3 + tải 1), hai tay duỗi ngang tải 1 kg/tay ở TCP, tổ hợp pan ±70°, mọi hướng gia tốc,
cả hai chiều α. Khối lượng tay = max(URDF AM-ARM200, spec.ARM_MASS_MOVING cận trên) + extra_arm_kg (dự phòng servo cổ tay khi tay
lên 6 DOF, spec.BASE_TIP_ARM_EXTRA_KG), đặt ở tâm khối lượng tay duỗi.
"""
from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "cad" / "faibotic"), str(ROOT / "sdk")]
import tipover as TP  # noqa: E402
from faibotic import spec  # noqa: E402

G = TP.G


def _cfg(lift_top: bool = True):
    M = json.load(open(ROOT / "cad" / "export" / "M1-PL-masses.json", encoding="utf-8"))
    M = copy.deepcopy(M)
    z_sh = spec.SHOULDER_Z0 + (spec.LIFT_RANGE[1] if lift_top else 0.0)
    if lift_top:
        for k in ("xe_nang", "tram_hai_tay"):
            if k in M["co_dinh"]:
                M["co_dinh"][k][3] += spec.LIFT_RANGE[1]
    cfg, _ = TP.pl_cfg(M)
    return M, cfg, z_sh


def _factor(ms: np.ndarray, edges, A: float, alpha: float, w: float, ndir: int) -> float:
    """ms: (N, 4) [m, x, y, z]. Nhỏ nhất theo cạnh, hướng a (ndir), chiều α."""
    m, p, z = ms[:, 0], ms[:, 1:3], ms[:, 3]
    th = np.linspace(0, 2 * math.pi, ndir, endpoint=False)
    best = math.inf
    for n, d in edges:
        s = p @ n - d                                       # >0 ngoài cạnh
        tip0 = float(np.sum(m * np.clip(s, 0, None)))
        hold0 = float(np.sum(m * np.clip(-s, 0, None)))
        jp = np.stack([-p[:, 1], p[:, 0]], 1) @ n           # (α × p)·n / α
        an = A * np.cos(th - math.atan2(n[1], n[0]))        # a·n theo hướng
        for sa in (1.0, -1.0):
            comp = -an[:, None] - sa * alpha * jp[None, :] + w * w * (p @ n)[None, :]   # quán tính ra ngoài cạnh
            mom = m[None, :] * comp / G * z[None, :]
            tip = tip0 + np.sum(np.clip(mom, 0, None), 1)
            hold = hold0 + np.sum(np.clip(-mom, 0, None), 1)
            best = min(best, float(np.min(hold / np.maximum(tip, 1e-12))))
    return best


def worst(A: float, alpha: float = 0.0, w: float = 0.0, extra_arm_kg: float = 0.0, lift_top: bool = True,
          pan_step: float = 10.0, ndir: int = 24) -> float:
    """Hệ số nhỏ nhất ở gia tốc tịnh tiến A (m/s²), gia tốc góc alpha (rad/s²), tốc độ quay w (rad/s)."""
    M, cfg, z_sh = _cfg(lift_top)
    E = TP.edges(cfg["R"], cfg["angles"])
    x, y, z = M["khay_goc_xa"]
    pans = np.arange(-70.0, 70.0 + 1e-9, pan_step)
    load = cfg["load_m"]
    arm_m = max(cfg["arm_m"], spec.ARM_MASS_MOVING) + extra_arm_kg     # cận trên spec (tự cập nhật khi tay đổi) + dự phòng
    res = math.inf
    for sy in (1, -1):
        fixed = list(cfg["fixed"]) + [(1.3, x, sy * y, z)]
        for pl in pans:
            L = TP.arm_points(cfg["pan_xy"], 1, pl, arm_m, cfg["arm_r"], load, cfg["load_r"], z_sh)
            for pr in pans:
                R = TP.arm_points(cfg["pan_xy"], -1, pr, arm_m, cfg["arm_r"], load, cfg["load_r"], z_sh)
                ms = np.array([tuple(v) + ((z_sh,) if len(v) == 3 else ()) for v in fixed + L + R], float)
                res = min(res, _factor(ms, E, A, alpha, w, ndir))
    return res


def envelope(extra_arm_kg: float | None = None, **kw) -> float:
    """Hệ số ở biên giới hạn đế SDK: A = BASE_MAX_ACC + BASE_TURN_ACC_MAX (ω × v), α = BASE_MAX_ALPHA, ω = BASE_MAX_W_TIP."""
    extra = spec.BASE_TIP_ARM_EXTRA_KG if extra_arm_kg is None else extra_arm_kg
    return worst(spec.BASE_MAX_ACC + spec.BASE_TURN_ACC_MAX, spec.BASE_MAX_ALPHA, spec.BASE_MAX_W_TIP, extra, **kw)


if __name__ == "__main__":
    for a in sys.argv[1:]:
        A, al, w, ex = (float(t) for t in a.split(","))
        print(a, round(worst(A, al, w, ex), 3), flush=True)


def worst_khay(items=(), A: float = 0.0, alpha: float = 0.0, w: float = 0.0, arm: tuple[float, float] | None = None,
               lift_top: bool = True, deck=(), pan_step: float = 10.0,
               ndir: int = 24) -> float:
    """Khay đế là sàn thiết bị (không còn tải 1,3 kg ở góc xa). Khối lượng khay lấy từ CAD (M1-PL-masses.json: 4 mảnh M1-ST-012, cột đỡ
    M1-ST-011, thanh 2040); deck: khối phụ thêm [m, x, y, z] (mặc định không có); items: vật trên khay [(kg, x, y, z)]. arm: (kg, tâm cách trục pan m), mặc định tay theo CAD
    (spec.ARM_MASS_CAD, spec.ARM_COM_R_CAD). Hai tay duỗi tải 1 kg/tay, pan ±70°, mọi hướng gia tốc như worst()."""
    M, cfg, z_sh = _cfg(lift_top)
    cd = dict(M["co_dinh"])
    E = TP.edges(cfg["R"], cfg["angles"])
    arm_m, arm_r = arm if arm is not None else (spec.ARM_MASS_CAD, spec.ARM_COM_R_CAD)
    fixed = [tuple(v) for v in cd.values()] + [tuple(v) for v in deck] + [tuple(v) for v in items]
    pans = np.arange(-70.0, 70.0 + 1e-9, pan_step)
    res = math.inf
    for pl in pans:
        L = TP.arm_points(cfg["pan_xy"], 1, pl, arm_m, arm_r, cfg["load_m"], cfg["load_r"], z_sh)
        for pr in pans:
            R = TP.arm_points(cfg["pan_xy"], -1, pr, arm_m, arm_r, cfg["load_m"], cfg["load_r"], z_sh)
            ms = np.array([tuple(v) + ((z_sh,) if len(v) == 3 else ()) for v in fixed + L + R], float)
            res = min(res, _factor(ms, E, A, alpha, w, ndir))
    return res


def envelope_khay(items=(), **kw) -> float:
    """worst_khay ở biên dốc đế SDK (như envelope())."""
    return worst_khay(items, spec.BASE_MAX_ACC + spec.BASE_TURN_ACC_MAX, spec.BASE_MAX_ALPHA, spec.BASE_MAX_W_TIP, **kw)
