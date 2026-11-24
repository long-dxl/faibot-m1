"""Động học của M1: tay 6+1 (như AM-ARM200 gốc: gập -> yaw -> cuộn) FK đủ 6 khớp, IK giải tích khoá yaw = 0; đế 3 bánh omni.

IK: khớp yaw giữ 0 (chuỗi khi đó trùng 5+1 cũ, nghiệm giải tích ổn định cho demo/teleop Descartes). Yaw dùng qua điều khiển khớp
(teleop leader AM-ARM200 6+1, set_joint_positions). arm_ik trả 6 góc với wrist_yaw = 0.

Khung tay: gốc ở đáy đế J1 trên trục pan (trục vai cao hơn gốc D0, lệch ra trước SHOULDER_DR), x hướng ra trước robot, z hướng lên.
P-L (S1C): tay AM-ARM200 gốc theo URDF upstream (spec.py); kẹp gốc AM-ARM200, tâm kẹp lệch trục cuộn (TCP_DY, TCP_DZ).
Khung robot: gốc ở tâm đế trên sàn, x tiến, y sang trái, z lên.
"""
from __future__ import annotations

import math

import numpy as np

from . import spec

TOOL_DOWN = -math.pi / 2  # góc ngẩng của dụng cụ: -90° = chĩa thẳng xuống


def arm_mount(side: str, lift: float) -> np.ndarray:
    """Vị trí gốc tay (mặt đế xoay vai, khung robot) theo độ cao trục nâng. Trục vai ở trên gốc D0, tức SHOULDER_Z0 + lift."""
    y = spec.SHOULDER_Y if side == "left" else -spec.SHOULDER_Y
    return np.array([spec.SHOULDER_X, y, spec.ARM_BASE_Z0 + lift])


def _side_sign(side: str) -> float:
    return 1.0 if side == "left" else -1.0


def _tcp_offset(q5: float, side: str, dy: float | None = None) -> tuple[float, float]:
    """Lệch tâm kẹp khỏi trục cuộn sau khi cuộn q5: (ngang theo trục khuỷu, vuông góc trục kẹp trong mặt phẳng tay). dy: thay TCP_DY."""
    ty, tz = _side_sign(side) * (spec.TCP_DY if dy is None else dy), spec.TCP_DZ
    c, s = math.cos(q5), math.sin(q5)
    return ty * c - tz * s, ty * s + tz * c


def _rx(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s_], [0, s_, c]])


def _ry(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, 0, s_], [0, 1.0, 0], [-s_, 0, c]])


def _rz(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])


def arm_frames(q, side: str = "left") -> dict:
    """Khung (R, p) khung tay của từng body tay 6+1, cùng cây body MJCF (model._arm_mjcf): shoulder, upper, fore, wrist, yaw, hand.
    q = [pan, shoulder, elbow, wrist_flex, wrist_yaw, wrist_roll]; chấp nhận q 5 phần tử cũ (yaw = 0)."""
    q = list(q)
    if len(q) == 5:
        q = q[:4] + [0.0] + q[4:]
    sg = _side_sign(side)
    F = {}
    R, p = _rz(q[0]), np.zeros(3)
    F["shoulder"] = (R, p)
    p = p + R @ np.array([spec.SHOULDER_DR, 0.0, spec.D0]); R = R @ _ry(q[1]); F["upper"] = (R, p)
    p = p + R @ np.array([spec.L1, 0.0, 0.0]); R = R @ _ry(q[2]); F["fore"] = (R, p)
    p = p + R @ np.array([spec.L2, 0.0, spec.WF_DZ]); R = R @ _ry(q[3]); F["wrist"] = (R, p)
    p = p + R @ np.array([spec.YAW_DX, sg * spec.ROLL_DY, spec.YAW_DZ]); R = R @ _rz(q[4]); F["yaw"] = (R, p)
    p = p + R @ np.array([spec.LW - spec.YAW_DX, 0.0, spec.ROLL_DZ - spec.YAW_DZ]); R = R @ _rx(q[5]); F["hand"] = (R, p)
    return F


def _arm_point(q, side: str, along: float, dy: float | None = None) -> tuple[np.ndarray, float]:
    """Điểm trên trục kẹp cách trục gập `along` (dọc trục cuộn), cộng lệch tâm kẹp; trả (điểm khung tay, góc ngẩng trục cuộn).
    chuỗi 6+1 (yaw giữa gập và cuộn); yaw 0 trùng hình học 5+1 cũ."""
    q = list(q)
    if len(q) == 5:
        q = q[:4] + [0.0] + q[4:]
    R, p = arm_frames(q, side)["hand"]
    ty = _side_sign(side) * (spec.TCP_DY if dy is None else dy)
    pt = p + R @ np.array([along - spec.LW, ty, spec.TCP_DZ])
    if q[4] == 0.0:
        pitch = -(q[1] + q[2] + q[3])
    else:
        d = R[:, 0]
        pitch = math.atan2(d[2], math.hypot(d[0], d[1]))
    return pt, pitch


def arm_fk(q, side: str = "left") -> tuple[np.ndarray, float]:
    """q = [pan, shoulder, elbow, wrist_flex, wrist_yaw, wrist_roll] -> (TCP trong khung tay, góc ngẩng dụng cụ). 6+1.

    Hình học cổ tay (spec.py): trục gập cách khuỷu L2, trục cuộn lệch (LW, ROLL_DY, ROLL_DZ) so với trục gập,
    tâm kẹp lệch (TCP_DY, TCP_DZ) khỏi trục cuộn và quay theo cuộn. Tay phải lấy gương theo y."""
    return _arm_point(q, side, spec.LT)


def fingertip_fk(q, side: str = "left") -> np.ndarray:
    """Đầu ngón (giữa hai ngón, trước tâm kẹp TIP_LEN) trong khung tay."""
    return _arm_point(q, side, spec.LT + spec.TIP_LEN)[0]


def fixed_side_dir(q, side: str = "left") -> np.ndarray:
    """Hướng đơn vị (khung tay) từ tâm kẹp sang ngón cố định của kẹp AM-ARM200 (trục đóng hàm, ngược chiều hàm động khép)."""
    a = _arm_point(q, side, spec.LT)[0]
    b = _arm_point(q, side, spec.LT, spec.TCP_DY + 0.01)[0]
    return (b - a) / np.linalg.norm(b - a)


def grasp_tcp(center, half_width: float, side: str = "left", grip_yaw: float = 0.0, margin: float = 0.002,
              pitch: float = None, ref_roll: float | None = None) -> np.ndarray | None:
    """kẹp AM-ARM200 có một ngón cố định, hàm động quay về phía nó. Để kẹp vật tâm `center` (khung tay) có nửa bề rộng `half_width`
    theo trục đóng, đặt TCP lệch khỏi tâm vật về phía ngón cố định sao cho mặt trong ngón cố định cách mặt vật `margin`:
    TCP = tâm + hướng_ngón_cố_định x (TCP_DY - FIXED_FACE_Y + half_width + margin). Hướng lấy theo nghiệm IK ở chính điểm TCP (lặp).
    ref_roll: như arm_ik (chọn nghiệm cuộn gần tư thế hiện tại, để phía ngón cố định khớp với lúc tay đi tới). Trả None nếu ngoài tầm."""
    pitch = TOOL_DOWN if pitch is None else pitch
    c = np.asarray(center, float); off = spec.TCP_DY - spec.FIXED_FACE_Y + half_width + margin
    t = c.copy()
    for _ in range(6):
        q = arm_ik(t, pitch=pitch, grip_yaw=grip_yaw, side=side, ref_roll=ref_roll)
        if q is None:
            return None
        t_new = c + fixed_side_dir(q, side) * off
        if np.linalg.norm(t_new - t) < 1e-6:
            return t_new
        t = t_new
    q = arm_ik(t, pitch=pitch, grip_yaw=grip_yaw, side=side, ref_roll=ref_roll)
    return t if q is not None and np.linalg.norm(c + fixed_side_dir(q, side) * off - t) < 1e-4 else None


def within_limits(q, tol: float = 1e-6) -> bool:
    for j, v in zip(spec.ARM_JOINTS, q):
        lo, hi = spec.ARM_LIMITS[j]
        if v < lo - tol or v > hi + tol:
            return False
    # v0.4.3: giới hạn ghép cổ tay-khuỷu-vai (spec.wrist_max) tránh kẹp/vỏ cổ tay giao giá chặn và gá vai
    if q[3] > spec.wrist_max(q[1], q[2]) + tol:
        return False
    return True


def _ik_fixed_roll(x: float, y: float, z: float, pitch: float, side: str, roll_of_pan) -> np.ndarray | None:
    """IK với cuộn cổ tay cho bởi roll_of_pan(q1). Lệch ngang làm pan phụ thuộc cuộn: lặp điểm bất động (lệch vài mm, hội tụ nhanh)."""
    rt = math.hypot(x, y)
    q1 = math.atan2(y, x) if rt > 1e-9 else 0.0
    for _ in range(30):
        q5 = roll_of_pan(q1)
        oy, oz = _tcp_offset(q5, side)
        lat = _side_sign(side) * spec.ROLL_DY + oy
        if rt <= abs(lat) + 1e-9:
            return None
        q1n = math.atan2(y, x) - math.asin(lat / rt)
        if abs(q1n - q1) < 1e-13:
            q1 = q1n
            break
        q1 = q1n
    q5 = roll_of_pan(q1)
    oy, oz = _tcp_offset(q5, side)
    lat = _side_sign(side) * spec.ROLL_DY + oy
    r = math.sqrt(rt * rt - lat * lat)
    dz = spec.ROLL_DZ + oz
    # trục gập cổ tay
    fr = r - spec.SHOULDER_DR - spec.LT * math.cos(pitch) + dz * math.sin(pitch)
    fh = z - spec.D0 - spec.LT * math.sin(pitch) - dz * math.cos(pitch)
    l2 = math.hypot(spec.L2, spec.WF_DZ)        # khuỷu -> trục gập (gồm lệch WF_DZ)
    phi = math.atan2(spec.WF_DZ, spec.L2)
    d = math.hypot(fr, fh)
    if d > spec.L1 + l2 - 1e-9 or d < abs(spec.L1 - l2) + 1e-9:
        return None
    beta = math.atan2(fh, fr)
    alpha = math.acos((spec.L1**2 + d**2 - l2**2) / (2 * spec.L1 * d))
    a1 = beta + alpha
    er, eh = spec.L1 * math.cos(a1), spec.L1 * math.sin(a1)
    a12 = math.atan2(fh - eh, fr - er) - phi
    q2 = -a1
    q3 = a1 - a12
    q4 = -(pitch) - q2 - q3
    return np.array([_wrap(q1), q2, q3, q4, 0.0, _wrap(q5)])   # yaw khoá 0 trong IK


def arm_ik(p, pitch: float = TOOL_DOWN, grip_yaw: float = 0.0, side: str = "left",
           roll: float | None = None, ref_roll: float | None = None) -> np.ndarray | None:
    """IK giải tích (nhánh khuỷu hướng lên) cho tâm kẹp.

    p: TCP mong muốn trong khung tay. pitch: góc ngẩng dụng cụ (mặc định chĩa xuống).
    grip_yaw: hướng trục đóng ngón trong mặt phẳng ngang (rad, 0 = theo trục x robot),
    chỉ có nghĩa khi dụng cụ chĩa xuống. roll: đặt thẳng góc cuộn cổ tay (bỏ qua grip_yaw).
    Hai ngón đóng theo trục y của khớp cuộn (ngang khi cuộn 0, như kẹp AM-ARM200).
    ref_roll: thử trước nghiệm cuộn gần ref_roll (giữ liên tục khi đi theo đường: kẹp có ngón cố định nên trục đóng xoay 180 độ
    là đổi phía ngón cố định). None: thứ tự cũ.
    Trả None nếu ngoài tầm với hoặc vượt giới hạn khớp.
    """
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    if roll is not None:
        cands = [lambda q1: roll]
    else:
        # dụng cụ chĩa xuống: trục y khớp cuộn nằm ở góc q1 + pi/2 - q5 trong mặt phẳng ngang
        base = lambda q1: q1 + math.pi / 2 - grip_yaw
        cands = [lambda q1: _wrap(base(q1)), lambda q1: _wrap(base(q1) + math.pi)]   # trục đóng ngón đối xứng: thử xoay thêm 180°
    sols = []
    for f in cands:
        q = _ik_fixed_roll(x, y, z, pitch, side, f)
        if q is not None and within_limits(q):
            if ref_roll is None or roll is not None:
                return q
            sols.append(q)
    if not sols:
        return None
    return min(sols, key=lambda q: abs(_wrap(q[5] - ref_roll)))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------- Đế omni ----------------

def wheel_speeds(vx: float, vy: float, wz: float) -> np.ndarray:
    """Vận tốc góc 3 bánh (rad/s) từ vận tốc đế trong khung robot."""
    out = []
    for th in spec.WHEEL_ANGLES:
        v = -math.sin(th) * vx + math.cos(th) * vy + spec.WHEEL_MOUNT_RADIUS * wz
        out.append(v / spec.WHEEL_RADIUS)
    return np.array(out)


def _wheel_matrix() -> np.ndarray:
    return np.array([[-math.sin(th), math.cos(th), spec.WHEEL_MOUNT_RADIUS]
                     for th in spec.WHEEL_ANGLES]) / spec.WHEEL_RADIUS


def base_twist(w) -> np.ndarray:
    """Ngược lại: (vx, vy, wz) từ vận tốc 3 bánh."""
    return np.linalg.solve(_wheel_matrix(), np.asarray(w, dtype=float))
