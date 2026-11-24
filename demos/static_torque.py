"""Mô-men tĩnh, tải công bố và vùng với tới của M1 (P-L: AM-ARM200 gốc, vai/khuỷu một STS3095, kẹp gốc AM-ARM200, không lò xo).

    python demos/static_torque.py            # in JSON (bảng tải công bố + mô-men duỗi hết + vùng với)
    python demos/static_torque.py --md       # in bảng Markdown cho docs/M1-LITE/tai-cong-bo-PL.md

Khối lượng: đủ cả chuỗi theo spec (phần in URDF upstream khối đặc 1,0 g/cm³ + servo + vỏ J56 + servo cuộn + kẹp gốc AM-ARM200 173 g + camera cổ tay),
lấy từ mô hình MuJoCo (model.robot_mjcf). Mô-men giữ = qfrc_bias (vận tốc 0) + J_tcp^T m g; không gia tốc, không ma sát.
Ngưỡng (quy tắc dự án "ghi đỉnh/liên tục và tầm với"):
  - liên tục: <= định mức. STS3095 22,5 kg·cm [trang bán]. STS3215 chưa có định mức hãng: dùng cùng tỉ lệ định mức/chặn của STS3095 (23,7%),
    tức 7,1 kg·cm [giả định, tự suy ra; chờ số hãng hoặc đo nhiệt DK-N9];
  - đỉnh: <= 60% mô-men chặn (= TORQUE_LIMIT nhóm vai/khuỷu/gập, spec.TORQUE_LIMIT_PCT).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))

from faibotic import kinematics, model as m1model, spec  # noqa: E402

PEAK_FRACTION = 0.60
RATED_RATIO_3215 = spec.STS3095.rated_kgcm / spec.STS3095.stall_kgcm    # [giả định] định mức STS3215 theo tỉ lệ STS3095
LOAD_JOINTS = ("shoulder", "elbow", "wrist_flex")   # khớp chịu trọng lực khi tay trong mặt phẳng đứng (pan, cuộn ~0)


def rated_nm(j: str) -> float:
    sv = spec.ARM_SERVO[j]
    r = sv.rated_nm if sv.rated_nm is not None else sv.stall_nm * RATED_RATIO_3215
    return r * spec.ARM_SERVO_COUNT[j]


def peak_nm(j: str) -> float:
    return PEAK_FRACTION * spec.ARM_SERVO[j].stall_nm * spec.ARM_SERVO_COUNT[j]


_M: dict = {}
# "tren": phần in khối đặc 1,0 g/cm³ như URDF (cận trên, mô hình sim mặc định); "in": phần in PETG 4 thành 15% (spec.PRINT_FACTOR_EST)
CASES = {"tren": None, "in": spec.PRINT_FACTOR_EST}


def _model(case: str = "tren"):
    if case not in _M:
        m = mujoco.MjModel.from_xml_string(m1model.robot_mjcf(scene=False, print_scale=CASES[case]))
        _M[case] = (m, mujoco.MjData(m))
    return _M[case]


def torques(q_arm, side: str = "left", case: str = "tren") -> tuple[dict, dict]:
    """(mô-men giữ không tải, mô-men thêm cho 1 kg ở TCP) theo khớp (N·m, dấu giữ)."""
    m, d = _model(case)
    d.qpos[:] = m.qpos0
    for j, v in zip(spec.ARM_JOINTS, q_arm):
        d.qpos[m.joint(f"{side}_{j}").qposadr[0]] = v
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jacSite(m, d, jacp, None, m.site(f"{side}_tcp").id)
    per_kg = jacp.T @ np.array([0, 0, spec.G])
    t0 = {j: float(d.qfrc_bias[m.joint(f"{side}_{j}").dofadr[0]]) for j in spec.ARM_JOINTS}
    t1 = {j: float(per_kg[m.joint(f"{side}_{j}").dofadr[0]]) for j in spec.ARM_JOINTS}
    return t0, t1


def static_torque(payload: float = spec.PAYLOAD_TARGET, q_arm=None, case: str = "tren") -> dict:
    """Mô-men giữ từng khớp tay trái với tải `payload` kg tại TCP. Mặc định duỗi hết nằm ngang (mọi khớp 0): xấu nhất cho vai."""
    q_arm = [0.0] * 5 if q_arm is None else q_arm
    t0, t1 = torques(q_arm, case=case)
    out = {}
    for j in spec.ARM_JOINTS:
        t = abs(t0[j] + payload * t1[j])
        sv = spec.ARM_SERVO[j]
        n = spec.ARM_SERVO_COUNT[j]
        out[j] = {"servo": sv.model, "servo_count": n, "torque_nm": round(t, 3), "torque_kgcm": round(t / spec.KGCM, 2),
                  "stall_kgcm": sv.stall_kgcm * n, "stall_ratio": round(t / (sv.stall_nm * n), 3),
                  "rated_kgcm": round(rated_nm(j) / spec.KGCM, 2), "rated_ratio": round(t / rated_nm(j), 3),
                  "rated_source": "trang bán" if sv.rated_kgcm else "giả định: tỉ lệ STS3095"}
    return out


def max_payload(q_arm, limit: str, case: str = "tren") -> tuple[float, str]:
    """Tải lớn nhất (kg) ở TCP để mọi khớp chịu tải <= ngưỡng ('continuous' hoặc 'peak'); kèm khớp giới hạn."""
    t0, t1 = torques(q_arm, case=case)
    best, who = math.inf, ""
    for j in LOAD_JOINTS:
        lim = rated_nm(j) if limit == "continuous" else peak_nm(j)
        a, b = t0[j], t1[j]
        if abs(a) > lim:
            return 0.0, j
        if abs(b) < 1e-9:
            continue
        # |a + m b| <= lim với m >= 0
        m = (lim - a) / b if b > 0 else (-lim - a) / b
        if m < best:
            best, who = m, j
    return max(0.0, best), who


def pose_for(reach: float, tool: str, z_rel: float) -> np.ndarray | None:
    """Tư thế tay trái (khung tay, pan 0) đặt TCP ở tầm ngang `reach` m tính từ trục pan, cao `z_rel` m so với trục vai.
    tool 'ngang': dụng cụ nằm ngang (pitch 0, cuộn 0); 'xuong': chĩa thẳng xuống."""
    p = [reach, spec.ROLL_DY, spec.D0 + z_rel]
    if tool == "ngang":
        return kinematics.arm_ik(p, pitch=0.0, roll=0.0)
    return kinematics.arm_ik(p, pitch=kinematics.TOOL_DOWN, roll=0.0)


REACH_MAX = spec.SHOULDER_DR + spec.L1 + spec.L2 + spec.LT      # TCP khi duỗi hết, từ trục pan


def load_table(reaches=(0.30, 0.35, 0.40, 0.45, 0.50, 0.55)) -> list[dict]:
    """Bảng tải công bố: duỗi hết nằm ngang (tầm lớn nhất); kẹp ngang ở tầm r, TCP ngang trục vai và thấp hơn 0,15 m;
    kẹp chúi xuống ở tầm 0,25-0,35 m, TCP thấp hơn trục vai 0,20 và 0,30 m. Điểm ngoài tầm IK (giới hạn khớp, giới hạn ghép) ghi "ngoài tầm"."""
    rows = []

    def row(name, tool, reach, z_rel, q):
        if q is None:
            rows.append(dict(pose=name, tool=tool, reach_m=round(reach, 3), z_rel_m=z_rel, reachable=False))
            return
        r = dict(pose=name, tool=tool, reach_m=round(reach, 3), z_rel_m=z_rel, reachable=True, q_deg=[round(math.degrees(v), 1) for v in q[:4]])
        for case in CASES:
            mc, jc = max_payload(q, "continuous", case)
            mp, jp = max_payload(q, "peak", case)
            t = static_torque(0.0, q, case)
            r[case] = dict(continuous_kg=round(mc, 2), continuous_limit=jc, peak_kg=round(mp, 2), peak_limit=jp,  # public-scan: ok
                           no_load_kgcm={j: t[j]["torque_kgcm"] for j in LOAD_JOINTS})
        rows.append(r)

    row("duỗi hết", "ngang", REACH_MAX, 0.0, np.zeros(5))
    for z in (0.0, -0.15):
        for r in reaches:
            row("kẹp ngang", "ngang", r, z, pose_for(r, "ngang", z))
    for z in (-0.20, -0.30):          # kẹp chúi xuống chỉ tới được tầm <= ~0,36 m (vai >= -80)
        for r in (0.25, 0.30, 0.35):
            row("kẹp chúi xuống", "xuống", r, z, pose_for(r, "xuong", z))
    return rows


def reach_map(table_heights=(0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90), clearance=0.05, obj_h=0.03) -> dict:
    """Với mỗi độ cao bàn: dải độ sâu (từ mép bàn) mà kẹp chĩa xuống chạm được điểm cao `obj_h` trên mặt bàn, ngay trước mỗi tay
    (y = vai), với trục nâng tối ưu trong 0–0,5 m. Mép bàn cách tâm đế = bán kính đế + clearance."""
    edge = spec.BASE_RADIUS + clearance
    res = {}
    lifts = np.linspace(*spec.LIFT_RANGE, 51)
    for th in table_heights:
        z = th + obj_h
        best = None
        for lift in lifts:
            depths = []
            for depth in np.arange(0.0, 0.40, 0.005):
                p = np.array([edge + depth, spec.SHOULDER_Y, z]) - kinematics.arm_mount("left", lift)
                q = kinematics.arm_ik(p)
                if q is None:
                    continue
                a1 = -q[1]   # khuỷu phải trên mặt bàn nếu nằm trên bàn
                er = spec.SHOULDER_DR + spec.L1 * math.cos(a1)
                ez = kinematics.arm_mount("left", lift)[2] + spec.D0 + spec.L1 * math.sin(a1)
                if spec.SHOULDER_X + er > edge and ez < th + 0.03:
                    continue
                depths.append(depth)
            if depths and (best is None or (max(depths) - min(depths)) > (best[2] - best[1])):
                best = (float(lift), float(min(depths)), float(max(depths)))
        res[f"{th:.2f}"] = None if best is None else {
            "lift_m": round(best[0], 3), "depth_from_edge_m": [round(best[1], 3), round(best[2], 3)]}
    return res


def markdown(rows) -> str:
    out = ["| Tư thế | Kẹp | Tầm TCP từ trục pan (m) | TCP so trục vai (m) | Liên tục, cận trên (kg) | Đỉnh, cận trên (kg) | Liên tục, ước tính in (kg) | Đỉnh, ước tính in (kg) | Khớp giới hạn (cận trên) |",
           "|---|---|---:|---:|---:|---:|---:|---:|---|"]
    vi = {"shoulder": "vai", "elbow": "khuỷu", "wrist_flex": "gập", "": "-"}
    f = lambda v: f"{v:.2f}".replace(".", ",")
    for r in rows:
        if not r["reachable"]:
            continue
        a, b = r["tren"], r["in"]
        lim = "/".join(dict.fromkeys(vi[x] for x in (a["continuous_limit"], a["peak_limit"])))
        out.append(f'| {r["pose"]} | {r["tool"]} | {f(r["reach_m"])} | {f(r["z_rel_m"])} | {f(a["continuous_kg"])} | {f(a["peak_kg"])} | '
                   f'{f(b["continuous_kg"])} | {f(b["peak_kg"])} | {lim} |')
    miss = [f'{r["pose"]} {f(r["reach_m"])} m (TCP {f(r["z_rel_m"])})' for r in rows if not r["reachable"]]
    out.append("")
    out.append("Ngoài tầm IK: " + "; ".join(miss))
    return "\n".join(out)


def main() -> int:
    rows = load_table()
    if "--md" in sys.argv:
        print(markdown(rows))
        return 0
    out = {"payload_kg_per_arm": spec.PAYLOAD_TARGET, "reach_max_m": round(REACH_MAX, 4),
           "arm_mass_moving_kg": round(spec.ARM_MASS_MOVING, 4),
           "thresholds_kgcm": {j: {"continuous": round(rated_nm(j) / spec.KGCM, 2), "peak": round(peak_nm(j) / spec.KGCM, 2)} for j in LOAD_JOINTS},
           "load_table": rows,
           "fully_extended_0.5kg": static_torque(0.5),
           "fully_extended_no_payload": static_torque(0.0),
           "reach_map_tool_down": reach_map()}
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
