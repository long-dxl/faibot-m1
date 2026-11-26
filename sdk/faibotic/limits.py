# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Giới hạn mềm M1: ARM_LIMITS + giới hạn ghép cổ tay-khuỷu-vai (spec.wrist_max) và yaw-gập (spec.wrist_yaw_range), kiểm cả đường đi giữa hai đích
(yaw-gập: kiểm tại đích ở Robot và backend, chưa kiểm đoạn nội suy: DK-N66 ghi chú).

Dùng chung cho lớp Robot và cả hai backend (sim, feetech): backend là lớp chặn cuối, không tin lớp trên.
Vùng cấm ghép (cổ tay gập quá spec.wrist_max(vai, khuỷu): kẹp J7 hoặc vỏ cổ tay giao giá chặn/gá vai, quét CAD v0.4.3) không lồi, nên hai đích hợp lệ
vẫn có thể nối bằng một đoạn thẳng cắt qua vùng cấm; plan() hạ cổ tay xuống mức an toàn trên cả đoạn vai-khuỷu, đổi vai/khuỷu, rồi nâng cổ tay lên đích.
"""
from __future__ import annotations

import math

import numpy as np

from . import spec
from .loi import LenhKhongHopLe, M1Loi

STEP_RAD = math.radians(1.0)   # bước lấy mẫu khi kiểm đoạn nội suy
TOL = 1e-9


def clamp(targets: dict[str, float], current: dict[str, float]) -> dict[str, float]:
    """Kẹp đích vào ARM_LIMITS (và giới hạn kẹp, lift) rồi áp giới hạn ghép tại điểm đích.

    Cổ tay vượt wrist_max(vai, khuỷu) thì kẹp cổ tay; lệnh chỉ vai hoặc khuỷu đưa tư thế vào vùng ghép thì hạ cổ tay theo.
    """
    out = {}
    for k, v in targets.items():
        if not math.isfinite(v):           # lớp thứ hai (Robot kiểm trước): NaN/inf không bao giờ tới servo/MuJoCo
            raise LenhKhongHopLe(f"khớp {k}: giá trị {v} không hữu hạn", khop=k)
        lo, hi = spec.limits(k)
        out[k] = float(min(max(v, lo), hi))
    merged = {**current, **out}
    for side in spec.SIDES:
        ks, ke, kw = (spec.joint_name(side, j) for j in ("shoulder", "elbow", "wrist_flex"))
        if (ks in out or ke in out) and ks in merged and ke in merged:       # khuỷu theo vai (cột nâng, QC2): lệnh vai làm khuỷu vượt thì hạ khuỷu
            emax = spec.elbow_max(merged[ks])
            if merged[ke] > emax + TOL:
                out[ke] = emax
                merged[ke] = emax
        if (ks in out or ke in out or kw in out) and ks in merged and ke in merged and kw in merged:
            wmax = spec.wrist_max(merged[ks], merged[ke])
            if merged[kw] > wmax + TOL:
                out[kw] = wmax
                merged[kw] = wmax
        ky = spec.joint_name(side, "wrist_yaw")        # yaw theo gập (vỏ/đáy J56 chạm cẳng tay J4), lệnh gập làm yaw vượt thì đưa yaw về
        if (kw in out or ky in out) and kw in merged and ky in merged:
            ylo, yhi = spec.wrist_yaw_range(merged[kw])
            if not ylo - TOL <= merged[ky] <= yhi + TOL:
                out[ky] = float(min(max(merged[ky], ylo), yhi))
    return out


def violations(joints: dict[str, float], tol: float = 1e-6) -> list[str]:
    """Danh sách vi phạm (rỗng = hợp lệ) của một tư thế."""
    bad = []
    for k, v in joints.items():
        lo, hi = spec.limits(k)
        if not lo - tol <= v <= hi + tol:
            bad.append(f"{k}={v:.4f} ngoài [{lo:.4f}, {hi:.4f}]")
    for side in spec.SIDES:
        ks, ke, kw = (spec.joint_name(side, j) for j in ("shoulder", "elbow", "wrist_flex"))
        if ks in joints and ke in joints and joints[ke] > spec.elbow_max(joints[ks]) + tol:
            bad.append(f"{side}: khuỷu {math.degrees(joints[ke]):.1f}° > {math.degrees(spec.elbow_max(joints[ks])):.1f}° khi vai {math.degrees(joints[ks]):.1f}° (chạm cột/xe)")
        if ks in joints and ke in joints and kw in joints:
            wmax = spec.wrist_max(joints[ks], joints[ke])
            if joints[kw] > wmax + tol:
                bad.append(f"{side}: cổ tay {math.degrees(joints[kw]):.1f}° > {math.degrees(wmax):.1f}° "
                           f"khi vai {math.degrees(joints[ks]):.1f}°, khuỷu {math.degrees(joints[ke]):.1f}°")
        ky = spec.joint_name(side, "wrist_yaw")
        if kw in joints and ky in joints:
            ylo, yhi = spec.wrist_yaw_range(joints[kw])
            if not ylo - tol <= joints[ky] <= yhi + tol:
                bad.append(f"{side}: yaw cổ tay {math.degrees(joints[ky]):.1f}° ngoài [{math.degrees(ylo):.1f}, {math.degrees(yhi):.1f}]° "
                           f"khi gập {math.degrees(joints[kw]):.1f}° (vỏ J56 chạm cẳng tay/giá J5)")
    return bad


def _triples(q0: dict, q1: dict):
    for side in spec.SIDES:
        ks, ke, kw = (spec.joint_name(side, j) for j in ("shoulder", "elbow", "wrist_flex"))
        if ks in q0 and ke in q0 and kw in q0 and (ks in q1 or ke in q1 or kw in q1):
            yield ks, ke, kw, (q0[ks], q0[ke], q0[kw]), (q1.get(ks, q0[ks]), q1.get(ke, q0[ke]), q1.get(kw, q0[kw]))


def _n(a, b) -> int:
    return max(1, math.ceil(max(abs(b[0] - a[0]), abs(b[1] - a[1]), abs(b[2] - a[2])) / STEP_RAD))


def _seg_ok(a, b) -> bool:
    n = _n(a, b)
    for i in range(n + 1):
        t = i / n
        s, e, w = (a[k] + (b[k] - a[k]) * t for k in range(3))
        if w > spec.wrist_max(s, e) + TOL:
            return False
    return True


def segment_ok(q0: dict[str, float], q1: dict[str, float]) -> bool:
    """True nếu đoạn nội suy tuyến tính q0 -> q1 (q1 có thể chỉ chứa một phần khớp) không cắt vùng cấm ghép."""
    return all(_seg_ok(a, b) for _, _, _, a, b in _triples(q0, q1))


def safe_step(q0: dict[str, float], q1: dict[str, float]) -> dict[str, float]:
    """Lớp chặn cuối ở backend: nếu đoạn q0 -> q1 cắt vùng cấm thì giữ vai và khuỷu ở q0, chỉ đi cổ tay (đã kẹp theo vai/khuỷu cũ).

    Gọi lại lệnh ở chu kỳ sau sẽ đi tiếp phần vai/khuỷu (cổ tay đã ra khỏi vùng nguy hiểm). Robot.move_joints tự chèn điểm trung gian.
    """
    out = dict(q1)
    for ks, ke, kw, a, b in _triples(q0, q1):
        if not _seg_ok(a, b):
            out[ks], out[ke] = a[0], a[1]
            out[kw] = min(b[2], spec.wrist_max(a[0], a[1]))
    return out


def plan(q0: dict[str, float], q1: dict[str, float]) -> list[dict[str, float]]:
    """Các điểm đi qua (không gồm q0, gồm q1) sao cho mọi đoạn nội suy đều hợp lệ.

    Đoạn thẳng hợp lệ thì trả [q1]. Không thì đi ba chặng: hạ cổ tay xuống mức an toàn trên cả đoạn (vai, khuỷu), đổi vai và khuỷu, rồi nâng cổ tay lên đích.
    """
    w1, w2 = {}, {}
    for ks, ke, kw, a, b in _triples(q0, q1):
        if _seg_ok(a, b):
            continue
        n = _n((a[0], a[1], a[2]), (b[0], b[1], a[2]))
        w_safe = min(a[2], b[2])
        for i in range(n + 1):
            t = i / n
            w_safe = min(w_safe, spec.wrist_max(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        w_safe = max(w_safe, spec.ARM_LIMITS["wrist_flex"][0])
        w1.update({ks: a[0], ke: a[1], kw: w_safe})
        w2.update({ks: b[0], ke: b[1], kw: w_safe})
    if not w1:
        return [dict(q1)]
    base = {k: q1[k] for k in q1}
    # điểm 1: khớp khác đứng yên ở q0, chỉ hạ cổ tay; điểm 2: đổi vai và khuỷu; điểm 3: đích
    p1 = {k: q0[k] for k in q1 if k in q0}
    p1.update(w1)
    p2 = dict(p1)
    p2.update(w2)
    return [p1, p2, base]


def loaded() -> dict:
    """Giới hạn mềm đang áp trong SDK (độ), để in và so với giá trị đọc lại từ servo."""
    return {"arm_deg": {j: tuple(round(math.degrees(v), 3) for v in spec.ARM_LIMITS[j]) for j in spec.ARM_JOINTS},
            "lift_m": tuple(spec.LIFT_RANGE), "lift_speed_max": spec.LIFT_SPEED_MAX, "lift_torque_limit_permille": spec.LIFT_TORQUE_LIMIT_PERMILLE,
            "torque_limit_pct": dict(spec.TORQUE_LIMIT_PCT), "current_budget_a": spec.CURRENT_BUDGET_A,
            "couple": {"wrist_max_deg_min": round(min(min(r) for r in spec.WRIST_COUPLE_MAX_DEG), 3),
                       "wrist_table_nodes": [len(spec.WRIST_COUPLE_SHOULDER_DEG), len(spec.WRIST_COUPLE_ELBOW_DEG)]}}


# ---------- trục nâng: tốc độ, công tắc hành trình (QC2 cột nâng vòng 2) ----------
def lift_slew(cur: float, target: float, dt: float, vmax: float | None = None) -> float:
    """Bước tiếp theo của lift tiến tới target, tốc độ <= vmax (mặc định spec.LIFT_SPEED_MAX)."""
    vmax = spec.LIFT_SPEED_MAX if vmax is None else vmax
    step = vmax * dt
    return cur + max(-step, min(step, target - cur))


def base_wz_max(v_norm: float) -> float:
    """|ω| lớn nhất cho phép ở tốc độ tịnh tiến v_norm: min(BASE_MAX_W, BASE_MAX_W_TIP, BASE_TURN_ACC_MAX / |v|)."""
    w = min(spec.BASE_MAX_W, spec.BASE_MAX_W_TIP)
    return min(w, spec.BASE_TURN_ACC_MAX / v_norm) if v_norm > 1e-9 else w


def base_limit_goal(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    """Kẹp đích vận tốc đế (khung robot): co (vx, vy) giữ hướng để |v| <= BASE_MAX_V; |ω| theo base_wz_max(|v|)
    (ưu tiên giữ tịnh tiến, giảm quay)."""
    vx, vy, wz = float(vx), float(vy), float(wz)
    n = math.hypot(vx, vy)
    if n > spec.BASE_MAX_V:
        vx, vy, n = vx * spec.BASE_MAX_V / n, vy * spec.BASE_MAX_V / n, spec.BASE_MAX_V
    wm = base_wz_max(n)
    return vx, vy, max(-wm, min(wm, wz))


class BaseRamp:
    """Dốc vận tốc đế (chống lật động). set_goal() nhận lệnh, step(dt) trả vận tốc gửi xuống bánh, sao cho mỗi chu kỳ:
    |Δv|/dt <= BASE_MAX_ACC (vector), |Δω|/dt <= BASE_MAX_ALPHA, |ω| <= BASE_MAX_W_TIP, |v|·|ω| <= BASE_TURN_ACC_MAX, |v| <= BASE_MAX_V.
    Thứ tự: ω trước (đích ω kẹp theo |v| hiện tại), rồi v (đích v kẹp theo ω mới), nên bất biến |v|·|ω| giữ được qua mọi bước.
    Dừng khẩn không đi qua đây: E-stop cắt nguồn servo bằng phần cứng."""

    def __init__(self):
        self.cur = np.zeros(3)
        self.goal = np.zeros(3)

    def set_goal(self, vx: float, vy: float, wz: float) -> np.ndarray:
        for k, v in (("vx", vx), ("vy", vy), ("wz", wz)):      # lớp thứ hai (Robot kiểm trước): dốc luôn hữu hạn
            if not math.isfinite(v):
                raise LenhKhongHopLe(f"vận tốc đế {k} = {v} không hữu hạn")
        self.goal = np.array(base_limit_goal(vx, vy, wz))
        return self.goal.copy()

    def reset(self) -> None:
        self.cur[:] = 0.0
        self.goal[:] = 0.0

    @property
    def settled(self) -> bool:
        return bool(np.allclose(self.cur, self.goal, atol=1e-12))

    def step(self, dt: float) -> np.ndarray:
        v = self.cur[:2]
        wm = base_wz_max(float(np.linalg.norm(v)))
        wg = max(-wm, min(wm, float(self.goal[2])))
        dw = spec.BASE_MAX_ALPHA * dt
        w = float(self.cur[2]) + max(-dw, min(dw, wg - float(self.cur[2])))
        vg = self.goal[:2].copy()
        vmax = spec.BASE_TURN_ACC_MAX / abs(w) if abs(w) > 1e-12 else math.inf
        gn = float(np.linalg.norm(vg))
        if gn > vmax:
            vg *= vmax / gn
        d = vg - v
        dn = float(np.linalg.norm(d))
        dv = spec.BASE_MAX_ACC * dt
        if dn > dv:
            d *= dv / dn
        self.cur = np.array([v[0] + d[0], v[1] + d[1], w])
        return self.cur.copy()


def base_move_time(dx: float, dy: float, dyaw: float, peak: float = 1.5, margin: float = 1.05) -> float:
    """Thời gian nhỏ nhất của hồ sơ trơn smoothstep (đỉnh vận tốc = 1,5 trung bình, đỉnh gia tốc = 6 d/T²) để không chạm trần,
    nên BaseRamp không phải cắt và đế đi đủ quãng (move_base vòng hở)."""
    d, th = math.hypot(dx, dy), abs(dyaw)
    t = [peak * d / spec.BASE_MAX_V, math.sqrt(6 * d / spec.BASE_MAX_ACC),
         peak * th / min(spec.BASE_MAX_W, spec.BASE_MAX_W_TIP), math.sqrt(6 * th / spec.BASE_MAX_ALPHA),
         peak * math.sqrt(d * th / spec.BASE_TURN_ACC_MAX)]
    return margin * max(t)


class LiftSwitches:
    """Đầu vào công tắc hành trình trừu tượng: read() -> (dưới, trên), True = đang đóng (chạm). Bản thật nối GPIO (chưa chọn chân: Chưa có số liệu)."""
    def read(self) -> tuple[bool, bool]:
        raise NotImplementedError


class FakeLiftSwitches(LiftSwitches):
    """Bản giả cho test."""
    def __init__(self, bottom: bool = False, top: bool = False):
        self.bottom, self.top = bottom, top

    def read(self):
        return self.bottom, self.top


class PositionLiftSwitches(LiftSwitches):
    """Công tắc suy từ vị trí lift (sim): đóng khi trong 0,5 mm quanh hai đầu."""
    def __init__(self, get_lift, eps: float = 0.0005):
        self.get_lift, self.eps = get_lift, eps

    def read(self):
        q = self.get_lift()
        return q <= spec.LIFT_RANGE[0] + self.eps, q >= spec.LIFT_RANGE[1] - self.eps


def lift_gate(cur: float, target: float, switches: LiftSwitches | None, latch=None) -> float:
    """Công tắc dưới đóng: không cho đi xuống; công tắc trên đóng: không cho đi lên (giữ tại cur, chỉ cho chiều ngược lại).

    latch (safety.LiftLatch, v0.5 thanh răng): khi có, lift chỉ được đổi vị trí lúc chốt đã rút và cảm biến đang báo "rút"
    (latch.motion_allowed()); không thì giữ tại cur. Việc rút chốt theo đúng thứ tự R2 7.2 là của backend (lift_unlock)."""
    if latch is not None and abs(target - cur) > 1e-12 and not latch.motion_allowed():
        return cur
    if switches is None:
        return target
    bottom, top = switches.read()
    if (bottom and target < cur) or (top and target > cur):
        return cur
    return target


# ---------- giới hạn mô-men và ngân sách dòng (R2 7.4, DK-N19) ----------
def servo_group(name: str) -> str:
    """Nhóm servo theo tên bus: left_shoulder_b -> shoulder, wheel_0 -> wheel, lift -> lift."""
    if name == "lift":
        return "lift"
    if name.startswith("wheel"):
        return "wheel"
    j = name.partition("_")[2]
    return j[: -len(spec.PAIR_SUFFIX)] if j.endswith(spec.PAIR_SUFFIX) else j


def servo_model(name: str) -> spec.Servo:
    g = servo_group(name)
    if g == "lift":
        return spec.STS3095
    if g == "wheel":
        return spec.STS3215
    return spec.ARM_SERVO[g]


def torque_limits(names) -> dict[str, int]:
    """TORQUE_LIMIT (‰ mô-men chặn) cho từng servo trong names. Vai: min(% nhóm, SHOULDER_TORQUE_LIMIT_KGCM / số servo của khớp)."""
    names = list(names)
    out = {}
    for n in names:
        g = servo_group(n)
        if g == "lift":
            out[n] = spec.LIFT_TORQUE_LIMIT_PERMILLE
            continue
        per = 10 * spec.TORQUE_LIMIT_PCT[g]
        if g == "shoulder":
            joint = n[: -len(spec.PAIR_SUFFIX)] if n.endswith(spec.PAIR_SUFFIX) else n
            k = sum(1 for x in names if x in (joint, spec.pair_name(joint)))
            per = min(per, int(round(1000 * spec.SHOULDER_TORQUE_LIMIT_KGCM / k / spec.STS3095.stall_kgcm)))
        out[n] = per
    return out


def current_budget(names, pi_a: float | None = None, limits_permille: dict | None = None, extra_a: dict | None = None,
                   models: dict | None = None) -> dict:
    """Ngân sách dòng: mỗi servo = dòng chặn x TORQUE_LIMIT/1000 [giả định dòng tỉ lệ mô-men]. Trả rows theo nhóm, tổng chặn,
    tổng cho phép (gồm phụ tải extra_a: Pi, mạch điều khiển, cuộn chốt lift), budget và ok (tổng cho phép <= spec.CURRENT_BUDGET_A).
    Phụ tải tính bằng dòng lớn nhất ở cả hai cột (không có giới hạn mô-men).
    models: {tên: spec.Servo} ghi đè loại servo theo tên (vd spec.PL_SERVO_MODELS: pan là STS3095 ở AM-ARM200); None = theo spec.ARM_SERVO."""
    if models is not None and not isinstance(names, (list, tuple)):
        names = list(names)
    if extra_a is None:
        extra_a = {"pi": spec.PI_CURRENT_A if pi_a is None else pi_a, "dieu_khien": spec.CTRL_CURRENT_A,
                   "cuon_chot_lift": spec.LIFT_LATCH_COIL_A if "lift" in list(names) else 0.0}
    pi_a = sum(extra_a.values())
    names = list(names)
    lim = torque_limits(names) if limits_permille is None else limits_permille
    rows: dict[str, dict] = {}
    for n in names:
        g = servo_group(n)
        a = (models[n] if models is not None and n in models else servo_model(n)).stall_a
        r = rows.setdefault(g, dict(n=0, stall_a=0.0, allowed_a=0.0, permille=set()))
        r["n"] += 1
        r["stall_a"] += a
        r["allowed_a"] += a * lim[n] / 1000
        r["permille"].add(lim[n])
    stall = sum(r["stall_a"] for r in rows.values()) + pi_a
    allowed = sum(r["allowed_a"] for r in rows.values()) + pi_a
    return dict(rows=rows, pi_a=pi_a, extra_a=dict(extra_a), stall_a=stall, allowed_a=allowed, budget_a=spec.CURRENT_BUDGET_A,
                ok=allowed <= spec.CURRENT_BUDGET_A)


# ---------------- S2: vùng cấm trạm hai tay (luật mềm theo vị trí, spec.KEEPOUT_*) ----------------
def arm_points_carriage(side: str, q) -> dict:
    """Khuỷu, trục gập (trên trục cuộn), TCP của một tay trong khung xe nâng (x, y như khung robot; z = 0 ở đáy J1), m."""
    import numpy as np
    from . import kinematics as K
    q1, q2 = q[0], q[1]
    a1 = -q2
    r, h = spec.SHOULDER_DR + spec.L1 * math.cos(a1), spec.D0 + spec.L1 * math.sin(a1)
    base = np.array([spec.SHOULDER_X, spec.SHOULDER_Y if side == "left" else -spec.SHOULDER_Y, 0.0])
    elbow = base + np.array([r * math.cos(q1), r * math.sin(q1), h])
    wrist = base + K._arm_point(q, side, 0.0)[0]
    tcp = base + K._arm_point(q, side, spec.LT)[0]
    return {"khuỷu": elbow, "cổ tay": wrist, "tâm kẹp": tcp}


class KeepoutError(M1Loi, ValueError):
    """Đích tay rơi vào vùng cấm trạm (keepout_violations); Robot từ chối lệnh, không gửi xuống backend (G1/E3, 24/11)."""
    ma = "SAF-04"
    muc = 20      # logging.INFO: từ chối lệnh, không phải sự cố


def keepout_check(targets: dict[str, float], current: dict[str, float], config: str = "di_dong", khay_dinh_z: float | None = None) -> list[str]:
    """Vi phạm vùng cấm của tư thế đích (gộp với tư thế hiện tại) cho mọi tay có khớp tay hoặc lift trong targets.
    khay_dinh_z: đỉnh vật đã khai báo trên khay đế (khay.KhayTai.dinh_z) hoặc None."""
    merged = {**current, **targets}
    lift = merged.get("lift", 0.0) if config == "di_dong" else 0.0
    bad = []
    for side in spec.SIDES:
        names = [spec.joint_name(side, j) for j in spec.ARM_JOINTS]
        if not (any(n in targets for n in names) or "lift" in targets) or not all(n in merged for n in names):
            continue
        bad += keepout_violations(side, [merged[n] for n in names], config, lift, khay_dinh_z)
    return bad


def keepout_violations(side: str, q, config: str = "di_dong", lift: float = 0.0, khay_dinh_z: float | None = None) -> list[str]:
    """Vi phạm vùng cấm của trạm (rỗng = hợp lệ), q = [pan, vai, khuỷu, gập, yaw, cuộn] (rad; nhận cả q 5 khớp cũ với yaw 0):
    - cầu bán kính KEEPOUT_HEAD_R quanh trục tilt đầu camera (khuỷu, cổ tay, tâm kẹp không được vào);
    - trụ đứng bán kính KEEPOUT_MAST_R quanh cột camera, dưới trục tilt (cùng ba điểm);
    - mặt bệ: cổ tay, tâm kẹp có x (khung trạm, tính từ trục pan) < KEEPOUT_DECK_X thì z (trên đáy J1) phải >= KEEPOUT_DECK_Z;
    - di_dong: tâm kẹp trên khay không thấp hơn mặt khay + KEEPOUT_TRAY_MARGIN (lift: độ cao trục nâng SDK, m);
      có khai báo vật (khay_dinh_z): hộp khay kéo về mép sau KHAY_DE_X[0], mốc là đỉnh vật nếu cao hơn mặt khay;
    - tram_ban: tâm kẹp không thấp hơn mặt bàn + KEEPOUT_TABLE_MARGIN.
    Số ở spec.py, đối chiếu quét mesh pl_lift_scan.py --tay (cad/export/M1-PL-tram-scan-tay.json). Nối vào Robot.set_joint_positions,
    move_joints, move_tcp (24/11, G1/E3): đích vi phạm -> KeepoutError (keepout_check)."""
    import numpy as np
    pts = arm_points_carriage(side, q)
    bad = []
    c = np.array(spec.HEAD_CAM_AXIS)
    for k, p in pts.items():
        d = float(np.linalg.norm(p - c))
        if d < spec.KEEPOUT_HEAD_R:
            bad.append(f"{side}: {k} cách đầu camera {d * 1000:.0f} mm < {spec.KEEPOUT_HEAD_R * 1000:.0f}")
    mx = (spec.HEAD_MAST_XR[0] + spec.HEAD_MAST_XR[1]) / 2
    for k, p in pts.items():
        d = math.hypot(p[0] - mx, p[1])
        if d < spec.KEEPOUT_MAST_R and p[2] < spec.HEAD_CAM_AXIS[2]:
            bad.append(f"{side}: {k} cách trục cột camera {d * 1000:.0f} mm < {spec.KEEPOUT_MAST_R * 1000:.0f}")
    t = pts["tâm kẹp"]
    for k in ("cổ tay", "tâm kẹp"):
        p = pts[k]
        if p[0] - spec.SHOULDER_X < spec.KEEPOUT_DECK_X and p[2] < spec.KEEPOUT_DECK_Z:
            bad.append(f"{side}: {k} sát bệ trạm (x {(p[0] - spec.SHOULDER_X) * 1000:.0f} mm, z {p[2] * 1000:.0f} mm)")
    if config == "di_dong":
        (tx, ty, tz), (hx, hy, hz) = spec.TRAY_POS, spec.TRAY_HALF
        m = 0.03
        top, what = tz + hz, "mặt khay"
        if khay_dinh_z is not None:
            x0, x1 = spec.KHAY_DE_X
            tx, hx, hy = (x0 + x1) / 2, (x1 - x0) / 2, max(hy, spec.KHAY_DE_Y)
            if khay_dinh_z > top:
                top, what = khay_dinh_z, "đỉnh vật trên khay"
        if abs(t[0] - tx) < hx + m and abs(t[1] - ty) < hy + m and spec.ARM_BASE_Z0 + lift + t[2] < top + spec.KEEPOUT_TRAY_MARGIN:
            bad.append(f"{side}: tâm kẹp thấp hơn {what} + {spec.KEEPOUT_TRAY_MARGIN * 1000:.0f} mm")
    if config == "tram_ban" and t[2] < -spec.H_J1_ABOVE_TABLE + spec.KEEPOUT_TABLE_MARGIN:
        bad.append(f"{side}: tâm kẹp dưới mặt bàn + {spec.KEEPOUT_TABLE_MARGIN * 1000:.0f} mm")
    return bad
