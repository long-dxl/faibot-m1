# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Mất điện (servo tắt) ở tư thế xấu: tốc độ, năng lượng tới chặn cơ khí, khả năng hấp thụ (v0.4).

Mô hình: cùng MJCF của model.py nhưng bỏ lực servo (mjDSBL_ACTUATION), khoá trục nâng/đế, tắt va chạm hình học
(giao cắt kiểm riêng bằng CAD: cad/faibotic/failsafe.py), giới hạn khớp = chặn CƠ KHÍ (spec.FAILSAFE_*), không phải giới hạn mềm.
Năng lượng tới chặn = động năng toàn tay (kể cả quán tính rotor quy đổi, armature 0,05 [giả định]) ở lần chạm giới hạn đầu tiên.
Ma sát/giảm chấn khớp là ẩn số lớn nhất: số đo STS3095 mô-men quay ngược chưa có số liệu. Mặc định = mô hình sim (damping 1,5 N·m·s/rad,
ma sát 0,05 N·m [giả định, chưa đo]); damping_scale=0, friction_nm=0 là cận trên không ma sát.
v0.4.1: chặn = ĐỆM PU có đường lực-ép (pad_force, neo-Hooke không nén được, E theo Gent từ Shore A) thay cho giới hạn khớp cứng;
giới hạn cứng chỉ còn ở hard_deg (kết cấu). Kết quả mỗi chặn: động năng lúc chạm đệm, mô-men đỉnh lên khớp, có chạm giới hạn cứng không.
Mọi số đệm là [giả định/tự tính], chưa đo.
v0.5 (thanh răng KHÔNG tự hãm): mất điện thì cuộn chốt (lấy từ SERVO+) mất điện, lò xo đẩy chốt vào răng, lift rơi tối đa
MỘT nấc (spec.LIFT_NOTCH_PITCH, mô-đun 1,5 [giả định] -> ~4,71 mm) rồi chốt đỡ. v0.4 (vitme tự hãm) coi lift đứng yên. Hai mô hình:
- drop()/sweep() (tay): vẫn khoá lift. Lift rơi <= 1 nấc trong vài chục ms, nhỏ so với tay rơi tới đệm; ảnh hưởng của cú giật
  lên tay chưa mô hình (Chưa có số liệu).
- lift_drop(): mô hình riêng lift + hai tay (khoá khớp tay) rơi từ q0 tới nấc chốt bên dưới, bắt đầu với vận tốc hạ v0 (E-stop khi
  đang hạ): năng lượng chốt và răng phải đỡ. Giả định chốt đã cài kịp (thời gian cài so với SERVO+ xả: chưa đo, DK-N17).
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from . import model as m1model, spec

_LOCK = ("base_x", "base_y", "base_yaw", "wheel_0", "wheel_1", "wheel_2")   # luôn khoá (đế đứng yên)
_LOCK_LIFT_FOR_ARM_SIM = ("lift",)   # tay: lift coi như đứng yên (chốt đỡ trong <= 1 nấc, xem lift_drop); KHÔNG phải tự hãm như v0.4
_HUGE = 1e6


def build_model(damping_scale: float = 1.0, friction_nm: float | None = None, payload: float = 0.0, spring_scale: float = 1.0,
                lift_free: bool = False):
    """friction_nm: ma sát Coulomb mỗi khớp tay (None = giá trị sim 0,05 x damping_scale).
    lift_free: True = lift không khoá (dùng cho lift_drop); mặc định khoá cho mô phỏng tay rơi."""
    m = mujoco.MjModel.from_xml_string(m1model.robot_mjcf(scene=False))
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION
    m.geom_contype[:] = 0
    m.geom_conaffinity[:] = 0
    m.tendon_stiffness[:] *= spring_scale
    for n in _LOCK + (() if lift_free else _LOCK_LIFT_FOR_ARM_SIM):
        m.dof_frictionloss[m.joint(n).dofadr[0]] = _HUGE
    for s in spec.SIDES:
        for j in spec.ARM_JOINTS:
            a = m.joint(f"{s}_{j}").dofadr[0]
            m.dof_damping[a] *= damping_scale
            m.dof_frictionloss[a] = m.dof_frictionloss[a] * damping_scale if friction_nm is None else friction_nm
        m.jnt_range[m.joint(f"{s}_shoulder").id] = [math.radians(spec.FAILSAFE_PADS["shoulder_low"]["hard_deg"]), math.radians(spec.FAILSAFE_PADS["shoulder_up"]["hard_deg"])]
        m.jnt_range[m.joint(f"{s}_elbow").id] = [math.radians(spec.FAILSAFE_PADS["elbow_low"]["hard_deg"]), math.radians(spec.FAILSAFE_PADS["elbow_high"]["hard_deg"])]
    m.body_mass[m.body("left_hand").id] += payload
    return m


def gent_E_mpa(shore: float) -> float:
    """Mô đun đàn hồi từ Shore A theo Gent (1958), MPa: Shore 70 -> 5,5; 60 -> 3,6; 50 -> 2,5."""
    return 0.0981 * (56 + 7.62336 * shore) / (0.137505 * (254 - 2.54 * shore))


def pad_force(name: str, pen_m: float, e_scale: float = 1.0) -> float:
    """Lực đệm (N) khi ép pen_m (m): ứng suất danh nghĩa neo-Hooke không nén được sigma = E/3 (1/(1-e)^2 - (1-e)), e = ép/dày.
    Bỏ hệ số hình dạng (dán một mặt, bảo thủ về độ cứng). e_scale nhân E (dung sai độ cứng Shore +-20%)."""
    p = spec.FAILSAFE_PADS[name]
    if pen_m <= 0:
        return 0.0
    e = min(pen_m * 1e3 / p["t_mm"], 0.9)
    lam = 1.0 - e
    return e_scale * gent_E_mpa(p["shore"]) / 3.0 * (1.0 / lam**2 - lam) * p["area_mm2"]


def pad_capacity(name: str, t_allow: float | None = None, e_scale: float = 1.0) -> dict:
    """Năng lượng (J) đệm hấp thụ cho tới khi mô-men lên khớp đạt t_allow (mặc định giới hạn bánh răng) hoặc tới giới hạn cứng.
    Trả về cap_J, tau_nm ở cuối, bottom (True nếu tới giới hạn cứng trước khi chạm t_allow), pen_mm."""
    t_allow = spec.FAILSAFE_T_GEAR_NM if t_allow is None else t_allow
    p = spec.FAILSAFE_PADS[name]
    pen_hard = abs(math.radians(p["hard_deg"] - p["contact_deg"])) * p["r_m"]
    E = 0.0
    x, dx = 0.0, 1e-5
    while x < pen_hard:
        f = pad_force(name, x + dx / 2, e_scale)
        if f * p["r_m"] > t_allow:
            return dict(cap_J=E, tau_nm=f * p["r_m"], bottom=False, pen_mm=x * 1e3, pen_hard_mm=pen_hard * 1e3)
        E += f * dx
        x += dx
    return dict(cap_J=E, tau_nm=pad_force(name, pen_hard, e_scale) * p["r_m"], bottom=True, pen_mm=pen_hard * 1e3, pen_hard_mm=pen_hard * 1e3)


_PAD_SIGN = {"shoulder_low": ("shoulder", -1), "shoulder_up": ("shoulder", 1), "elbow_high": ("elbow", 1), "elbow_low": ("elbow", -1)}


def drop(m, q_shoulder: float, q_elbow: float, q_wrist: float = 0.0, t_end: float = 1.5, e_scale: float = 1.0) -> dict:
    """Thả tay trái (độ) từ nghỉ, servo tắt, đệm PU tác dụng. Với mỗi chặn bị chạm: E_J (động năng toàn tay lúc chạm đệm),
    v_tcp (m/s), tau_nm (mô-men đỉnh đệm lên khớp trong cả lần thả), hard (True nếu tới giới hạn cứng), cùng tốc độ vai/khuỷu."""
    d = mujoco.MjData(m)
    jn_ = {"shoulder": m.joint("left_shoulder"), "elbow": m.joint("left_elbow")}
    soft = {"shoulder": spec.ARM_LIMITS["shoulder"], "elbow": spec.ARM_LIMITS["elbow"]}
    for jn, v in (("shoulder", q_shoulder), ("elbow", q_elbow), ("wrist_flex", q_wrist)):
        j = m.joint(f"left_{jn}")
        lo, hi = spec.ARM_LIMITS[jn]     # xuất phát trong giới hạn MỀM (v0.4.1 vòng 2: vai -80, khuỷu ngược -10; v0.4.4 cả cổ tay -77,5..87,5, trước đó cổ tay không kẹp vì range MJCF -100..120 phủ lưới)
        d.qpos[j.qposadr[0]] = float(np.clip(math.radians(v), lo + math.radians(0.5), hi - math.radians(0.5)))
    mujoco.mj_forward(m, d)
    jp = np.zeros((3, m.nv))
    Mv = np.zeros(m.nv)
    ev: dict = {}
    for _ in range(int(t_end / m.opt.timestep)):
        d.qfrc_applied[:] = 0
        for k, (jn, sg) in _PAD_SIGN.items():
            p = spec.FAILSAFE_PADS[k]
            j = jn_[jn]
            pen = sg * (d.qpos[j.qposadr[0]] - math.radians(p["contact_deg"])) * p["r_m"]
            if pen > 0:
                tau = pad_force(k, pen, e_scale) * p["r_m"]
                d.qfrc_applied[j.dofadr[0]] += -sg * tau
                if k not in ev:
                    mujoco.mj_mulM(m, d, Mv, d.qvel)
                    mujoco.mj_jacSite(m, d, jp, None, m.site("left_tcp").id)
                    ev[k] = dict(E_J=0.5 * float(d.qvel @ Mv), v_tcp=float(np.linalg.norm(jp @ d.qvel)), tau_nm=0.0, hard=False,
                                 w_shoulder=float(d.qvel[jn_["shoulder"].dofadr[0]]), w_elbow=float(d.qvel[jn_["elbow"].dofadr[0]]))
                ev[k]["tau_nm"] = max(ev[k]["tau_nm"], tau)
                if sg * (d.qpos[j.qposadr[0]] - m.jnt_range[j.id][0 if sg < 0 else 1]) >= -1e-3:
                    ev[k]["hard"] = True
        qs_deg, qe_deg = math.degrees(d.qpos[jn_["shoulder"].qposadr[0]]), math.degrees(d.qpos[jn_["elbow"].qposadr[0]])
        if "lowstop_hit" not in ev and qe_deg > spec.lowstop_elbow_clear_deg(qs_deg) + 1.5:
            mujoco.mj_mulM(m, d, Mv, d.qvel)   # gá khuỷu đập thanh chặn dưới (cứng, CAD) trước đệm: động năng lúc đó
            ev["lowstop_hit"] = dict(E_J=0.5 * float(d.qvel @ Mv), v_tcp=0.0, tau_nm=0.0, hard=True, w_shoulder=0.0, w_elbow=0.0)
        mujoco.mj_step(m, d)
    return ev


def sweep(m, shoulders=(-80, -55, -30, -5, 20, 49), elbows=range(-10, 131, 35), wrists=(-77.5, 0, 87.5), t_end: float = 1.5, e_scale: float = 1.0) -> dict:
    """Lưới tư thế xuất phát (độ). Trả về xấu nhất theo từng chặn: {chặn: {E_J, v_tcp, tau_nm, hard, start}}; tau_nm và hard lấy xấu nhất riêng."""
    worst: dict = {}
    for qs in shoulders:
        for qe in elbows:
            for qw in wrists:
                for k, v in drop(m, qs, qe, qw, t_end, e_scale).items():
                    w = worst.get(k)
                    if w is None:
                        worst[k] = dict(v, start=(qs, qe, qw))
                        continue
                    tau, hard = max(w["tau_nm"], v["tau_nm"]), w["hard"] or v["hard"]
                    if v["E_J"] > w["E_J"]:
                        w = dict(v, start=(qs, qe, qw))
                    worst[k] = dict(w, tau_nm=tau, hard=hard)
    return worst


def stop_capacity(alu_link: bool | None = None, e_scale: float = 1.0) -> dict:
    """Khả năng hấp thụ (J) từng chặn tới khi mô-men lên khớp đạt giới hạn bánh răng (spec.FAILSAFE_T_GEAR_NM = 9,3 N·m [giả định]).
    Chỉ tính được khi link nhôm (PETG: cổ link 7,8 N·m / SF 2 = 3,9 N·m thấp hơn, tính lại bằng t_allow)."""
    alu = spec.FAILSAFE_LINK_ALU if alu_link is None else alu_link
    t_allow = spec.FAILSAFE_T_GEAR_NM if alu else min(7.8 / 2.0, spec.FAILSAFE_T_GEAR_NM)
    out = {k: pad_capacity(k, t_allow, e_scale)["cap_J"] for k in spec.FAILSAFE_PADS}
    out.update(T_allow_nm=t_allow, T_gear_nm=spec.FAILSAFE_T_GEAR_NM, link_alu=alu)
    return out


def static_hold_torque(payload: float = spec.PAYLOAD_TARGET, shoulders=range(-80, 91, 15), elbows=range(-10, 131, 15), wrists=(-100, 0, 120)) -> dict:
    """Mô-men quay ngược mà hộp số vai phải chịu để tay đứng yên khi mất điện (trọng lực + lò xo + tải, khuỷu và cổ tay cố định):
    xấu nhất theo chiều nâng (lò xo thắng) và chiều hạ. N·m, cho cả cặp servo."""
    m = build_model(payload=payload)
    d = mujoco.MjData(m)
    sh = m.joint("left_shoulder")
    up = down = 0.0
    for qs in shoulders:
        for qe in elbows:
            for qw in wrists:
                for jn, v in (("shoulder", qs), ("elbow", qe), ("wrist_flex", qw)):
                    d.qpos[m.joint(f"left_{jn}").qposadr[0]] = math.radians(v)
                d.qvel[:] = 0
                mujoco.mj_forward(m, d)
                t = float(d.qfrc_bias[sh.dofadr[0]] - d.qfrc_passive[sh.dofadr[0]])   # >0: trọng lực kéo xuống (dương = hạ)
                down, up = max(down, t), max(up, -t)
    return dict(hold_down_nm=down, hold_up_nm=up)


def lift_drop_hand(mass_kg: float = spec.LIFT_DESIGN_LOAD_KG, v0: float = 0.0, notch_m: float = spec.LIFT_NOTCH_PITCH) -> dict:
    """Tính tay (R2 7.2): rơi tối đa một nấc từ vận tốc hạ v0. E = m g d + ½ m v0² (bỏ ma sát, cận trên)."""
    e_pot = mass_kg * spec.G * notch_m
    e_kin = 0.5 * mass_kg * v0 ** 2
    return dict(drop_m=notch_m, E_pot_J=e_pot, E_kin0_J=e_kin, E_J=e_pot + e_kin,
                v_impact=math.sqrt(v0 ** 2 + 2 * spec.G * notch_m), mass_kg=mass_kg)


def lift_drop(q0: float = 0.25, v0: float = 0.0, notch_m: float = spec.LIFT_NOTCH_PITCH, payload: float = 0.0,
              damping_scale: float = 1.0, t_end: float = 0.5) -> dict:
    """Mất điện khi lift ở q0 (m), đang hạ với vận tốc v0 (m/s, dương = hạ): servo tắt, chốt cài, xe + hai tay (khớp tay khoá)
    rơi tới nấc bên dưới. Trường hợp xấu nhất: đầu chốt vừa qua một nấc nên rơi đủ một bước răng notch_m.
    Trả về drop_m, v_impact (m/s), E_J (động năng toàn hệ lúc chạm chốt, gồm quán tính quy đổi armature của MJCF), mass_kg
    (khối lượng cây con xe nâng), E_hand_J (m g d + ½ m v0² với mass_kg). damping_scale 0 = không giảm chấn (cận trên)."""
    m = build_model(damping_scale=1.0, friction_nm=_HUGE, payload=payload, lift_free=True)   # khoá khớp tay bằng ma sát lớn
    j = m.joint("lift")
    a = j.dofadr[0]
    m.dof_damping[a] *= damping_scale
    m.dof_frictionloss[a] = 0.0
    m.jnt_range[j.id] = [q0 - notch_m, spec.LIFT_RANGE[1] + 1.0]
    m.opt.timestep = 1e-4                    # bước nhỏ: rơi một nấc chỉ ~30 ms
    d = mujoco.MjData(m)
    d.qpos[j.qposadr[0]] = q0
    d.qvel[a] = -v0
    mujoco.mj_forward(m, d)
    mass = float(m.body_subtreemass[m.body("carriage").id])
    Mv = np.zeros(m.nv)
    out = None
    for _ in range(int(t_end / m.opt.timestep)):
        mujoco.mj_step(m, d)
        if d.qpos[j.qposadr[0]] <= q0 - notch_m + 1e-5:
            mujoco.mj_mulM(m, d, Mv, d.qvel)
            out = dict(drop_m=notch_m, v_impact=float(-d.qvel[a]), E_J=0.5 * float(d.qvel @ Mv), mass_kg=mass,
                       t_s=float(d.time))
            break
    if out is None:
        out = dict(drop_m=q0 - float(d.qpos[j.qposadr[0]]), v_impact=0.0, E_J=0.0, mass_kg=mass, t_s=t_end)
    out["E_hand_J"] = lift_drop_hand(mass, v0, notch_m)["E_J"]
    return out
