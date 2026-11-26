# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Sinh mô hình M1 (MJCF cho MuJoCo, URDF cho ROS/khác) từ spec.py.

P-L (S1C): 2 tay AM-ARM200 gốc 6+1 (khớp yaw mở, body `<bên>_yaw`) (khung body theo URDF upstream, quy ước góc SDK), kẹp gốc AM-ARM200 (thân kẹp trong body
`<bên>_hand`, hàm động body `<bên>_jaw` quay quanh khớp `<bên>_gripper` = góc mở từ chỗ khép), đế 3 omni R0,25 + đối trọng + khay (S1G: hình va chạm theo CAD thân P-L, mesh hiển thị từ CAD),
trục nâng thanh răng (slide `lift`). Không lò xo cân bằng (P-L bỏ).
Va chạm: hình khối đơn giản (hộp, trụ, viên nang). Khối lượng/quán tính: phần tử <inertial> theo spec.LINK_PRINT + spec.ARM_POINT_MASSES.
Hiển thị: mesh trong sim/assets/ (sim/assets/meshes.json, source.variant == "P-L"): geom mesh không va chạm, không khối lượng, group 2;
hình khối va chạm chuyển sang group 3. Mesh của biến thể cũ (P-A, ghép đôi) bị bỏ qua: dùng hình khối làm hình hiển thị.
Vật lý cho MuJoCo WASM: timestep 2 ms, implicitfast, cone elliptic; kẹp giữ vật bằng ma sát (không weld).
Chạy `python -m faibotic.model --out sim/` để xuất lại file trong sim/.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from . import spec

# Bố cục cảnh pick-place (toạ độ thế giới; robot xuất phát ở gốc, nhìn theo +x)
TABLE_EDGE_X = 1.25
TABLE_DEPTH = 0.60
TABLE_HALF_Y = 0.60
TABLE_HEIGHT = 0.75
BOX_HALF = (0.025, 0.13, 0.03)   # hộp 5 x 26 x 6 cm, trục dài theo y
BOX_MASS = 0.20
BOX_START = (1.37, 0.0)
BOX_TARGET = (1.37, -0.25)

WHEEL_MASS = 0.20   # mỗi bánh omni (MJCF 0,19 vành + 0,01 nan; URDF trước 24/11 ghi 0,3: sửa theo MJCF, kiem_cheo so tổng URDF = MJCF = spec)
ROBOT_GEOM_PREFIXES = ("base", "column", "head", "carriage", "left_", "right_", "wheel_")

MESH_DIR = Path(__file__).resolve().parents[2] / "sim" / "assets"
MESH_VARIANT = "P-L"
# Bảng màu bộ render (media/renders-web/src/render_scene.py; tokens ui-tokens + brand navy #1E3A5F). Không cam.
# (hex sRGB, specular, shininess, reflectance)
MATERIALS = {"ivory": ("#f1efe8", 0.25, 0.4, 0.0), "ivory_hi": ("#f6f5f1", 0.25, 0.4, 0.0), "navy": ("#1e3a5f", 0.35, 0.5, 0.0),
             "alu": ("#b3b7bd", 0.7, 0.7, 0.05), "steel": ("#8d939b", 0.8, 0.8, 0.08), "servo": ("#1b1d21", 0.2, 0.3, 0.0),
             "servo_body": ("#1a1a1a", 0.3, 0.4, 0.0),
             "pad": ("#33363d", 0.1, 0.2, 0.0), "rubber": ("#26282d", 0.1, 0.2, 0.0), "hub": ("#6d727a", 0.6, 0.6, 0.04)}

# ---------------- kẹp gốc AM-ARM200 (khung tay = gốc khớp cuộn, mét; cad/export/AM_gripper_PL.json) ----------------
# Thân kẹp: hộp bao thân J7 trừ ngón (x -2,8..62 mm, bỏ phần ngón để lòng kẹp trống), ngón cố định x 70..104, mặt trong y 10,0 (tay trái).
# Hàm động: hộp xoay theo hướng hàm (beta 18,28 độ quanh z, từ trục kẹp tới đầu hàm), phần ngón u 51,6..85,9 mm từ trục; mặt trong lùi 1,77 mm
# để hai hộp không lấn nhau khi khép (khép = góc khớp 0). Camera và giá camera: chỉ mesh hiển thị, không va chạm.
GRIP_TCP_X = spec.LT - spec.LW                     # 0,09382
PALM_BOX = ((-0.00277, -0.0301, -0.02795), (0.062, 0.03512, 0.02406))      # (min, max) khung tay, tay trái
FIXED_FINGER_BOX = ((0.070, 0.0100, -0.00963), (0.104, 0.02771, 0.00979))
JAW_BETA = math.radians(18.282)
JAW_U, JAW_W, JAW_Z = (0.051634, 0.085898), (-0.008827, 0.003222), (-0.008047, 0.011203)   # khung trục kẹp: u dọc hàm, w ngang (về ngón cố định), z
WRIST_CAM_POS = (0.0112, -0.0018, 0.0630)          # đầu ống kính, khung left_camera URDF + 18 mm dọc trục quang [vị trí đề xuất]
WRIST_CAM_LOOK = (0.9118, 0.0, -0.4107)            # trục quang: chúc 24,2 độ xuống phía ngón (URDF left_camera)
GRIP_TORQUE_NM = spec.ARM_SERVO["gripper"].stall_nm * spec.TORQUE_LIMIT_PCT["gripper"] / 100   # 1,47 N.m -> ~17 N ở đầu hàm
GRIP_FORCE_N = GRIP_TORQUE_NM / spec.GRIPPER_TIP_R


def _box(lo, hi, sg=1.0):
    """(tâm, nửa cạnh) của hộp (min, max) khung tay trái; tay phải gương y."""
    lo, hi = np.array(lo), np.array(hi); c = (lo + hi) / 2; h = (hi - lo) / 2
    return (c[0], sg * c[1], c[2]), tuple(h)


def jaw_geom(sg=1.0):
    """(tâm, nửa cạnh, quat) hộp ngón hàm động trong khung body jaw (gốc = trục kẹp, trục song song khung tay); tay phải gương y."""
    b = sg * JAW_BETA; d = np.array([math.cos(b), math.sin(b), 0.0]); n = np.array([-math.sin(b), math.cos(b), 0.0])
    u, w, z = (sum(JAW_U) / 2, sg * sum(JAW_W) / 2, sum(JAW_Z) / 2)
    c = u * d + w * n + np.array([0, 0, z])
    h = ((JAW_U[1] - JAW_U[0]) / 2, (JAW_W[1] - JAW_W[0]) / 2, (JAW_Z[1] - JAW_Z[0]) / 2)
    return tuple(c), h, (math.cos(b / 2), 0.0, 0.0, math.sin(b / 2))


def wrist_cam_xyaxes(sg=1.0) -> str:
    L = np.array(WRIST_CAM_LOOK); L[1] *= sg; x = np.array([0.0, -sg, 0.0]); y = np.cross(-L, x)
    return f(*x, *y)


# Hình va chạm tay (khung body, tay trái; tay phải gương y): (body, tên, kiểu, tham số)
# Hình khối theo hộp bao thô URDF/STEP, chỉ để va chạm; mesh hiển thị nằm riêng.


def _rgba(h: str) -> str:
    return f(*(int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)), 1)


def load_meshes() -> list[dict]:
    """Danh sách mesh hiển thị (sim/assets/meshes.json, chỉ nhận source.variant == MESH_VARIANT); rỗng nếu chưa xuất.
    Mesh xuất sẵn trong khung body nên place_body = body, place_pos = 0."""
    p = MESH_DIR / "meshes.json"
    if not p.exists():
        return []
    d = json.loads(p.read_text())
    if d.get("source", {}).get("variant") != MESH_VARIANT:
        return []
    return [dict(m, place_body=m["body"], place_pos=(0.0, 0.0, 0.0)) for m in d["meshes"]]


def mesh_place(m: dict, frames: dict | None = None) -> tuple[str, tuple]:
    """(body gắn, vị trí gốc mesh trong body). P-L: mesh xuất theo khung body nên luôn (body, 0)."""
    return m["body"], (0.0, 0.0, 0.0)


def _vis(body: str, meshes: list[dict]) -> str:
    out = []
    for m in meshes:
        if m.get("place_body", m["body"]) != body:
            continue
        out.append(f'\n{" " * 8}<geom name="{m["name"]}" class="vis" mesh="{m["name"]}" material="m1_{m["material"]}"/>')
    return "".join(out)


def f(*v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def lookat_xyaxes(pos, target) -> str:
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    fwd = target - pos
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return f(*right, *up)


# ---------------- khối lượng ----------------

def _mirror_I(I6, sg):
    ixx, iyy, izz, ixy, ixz, iyz = I6
    return (ixx, iyy, izz, sg * ixy, ixz, sg * iyz)


def body_inertial(body: str, side: str = "left", print_scale: dict | None = None) -> tuple[float, np.ndarray, np.ndarray]:
    """(khối lượng, trọng tâm, ma trận quán tính 3x3 tại trọng tâm) của body tay = phần in URDF + khối điểm; tay phải gương y.
    print_scale: {body: hệ số} nhân khối lượng/quán tính phần in (vd spec.PRINT_FACTOR_EST: in thật PETG thay khối đặc 1,0 g/cm³)."""
    sg = 1.0 if side == "left" else -1.0
    items = []
    if body in spec.LINK_PRINT:
        m, c, I6 = spec.LINK_PRINT[body]
        k = (print_scale or {}).get(body, 1.0)
        m, I6 = m * k, tuple(v * k for v in I6)
        ixx, iyy, izz, ixy, ixz, iyz = _mirror_I(I6, sg)
        items.append((m, np.array([c[0], sg * c[1], c[2]]), np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])))
    for b, _, m, c in spec.ARM_POINT_MASSES:
        if b == body:   # khối điểm: quán tính hộp 30 mm cho khỏi suy biến
            items.append((m, np.array([c[0], sg * c[1], c[2]]), np.eye(3) * m * 0.03 ** 2 / 6))
    M = sum(x[0] for x in items)
    C = sum(x[0] * x[1] for x in items) / M
    I = np.zeros((3, 3))
    for m, c, Ii in items:
        d = c - C
        I += Ii + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, C, I


def _inertial_mjcf(body: str, side: str, print_scale: dict | None = None) -> str:
    m, c, I = body_inertial(body, side, print_scale)
    return (f'<inertial pos="{f(*c)}" mass="{m:.6g}" '
            f'fullinertia="{f(I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])}"/>')


# ---------------- MJCF ----------------

def _arm_mjcf(side: str, meshes: list[dict] | None = None, print_scale: dict | None = None) -> str:
    meshes = meshes or []
    mb = {m["place_body"] for m in meshes}
    cgb = lambda b: ' group="3"' if f"{side}_{b}" in mb else ""   # body có mesh: hình khối va chạm ẩn
    cg = cgb("hand")
    v = lambda b: _vis(f"{side}_{b}", meshes)
    sy = spec.SHOULDER_Y if side == "left" else -spec.SHOULDER_Y
    s = side
    sg = 1.0 if side == "left" else -1.0
    palm, fix, jg = _box(*PALM_BOX, sg), _box(*FIXED_FINGER_BOX, sg), jaw_geom(sg)
    tcp = (GRIP_TCP_X, sg * spec.TCP_DY, spec.TCP_DZ)
    yaw_pos = (spec.YAW_DX, sg * spec.ROLL_DY, spec.YAW_DZ)                     # khớp yaw (6+1) trong body cổ tay
    hand_pos = (spec.LW - spec.YAW_DX, 0.0, spec.ROLL_DZ - spec.YAW_DZ)       # gốc khớp cuộn trong body yaw
    ink = "0.95 0.94 0.91 1"   # ngà (không mesh)
    dark = "0.12 0.23 0.37 1" if side == "left" else "0.74 0.75 0.78 1"
    return f"""
      <geom name="{s}_mount_geom"{' group="3"' if "carriage" in mb else ""} type="box" size="0.045 0.045 0.02" pos="{f(spec.SHOULDER_X, sy, 0.02)}" mass="0" rgba="{ink}"/>
      <body name="{s}_shoulder" pos="{f(spec.SHOULDER_X, sy, 0)}">
        <joint name="{s}_pan" class="arm" axis="0 0 1" range="{f(*spec.ARM_LIMITS['pan'])}"/>
        {_inertial_mjcf("shoulder", s, print_scale)}
        <geom name="{s}_shoulder_geom"{cgb("shoulder")} type="box" size="0.035 0.04 0.045" pos="{f(0.03, 0, 0.105)}" rgba="{ink}"/>{v("shoulder")}
        <body name="{s}_upper" pos="{f(spec.SHOULDER_DR, 0, spec.D0)}">
          <joint name="{s}_shoulder" class="arm" axis="0 1 0" range="{f(*spec.ARM_LIMITS['shoulder'])}"/>
          {_inertial_mjcf("upper", s, print_scale)}
          <geom name="{s}_upper_geom"{cgb("upper")} type="capsule" size="0.026" fromto="{f(0.03, 0, 0.012, spec.L1 - 0.035, 0, 0.012)}" rgba="{ink}"/>{v("upper")}
          <body name="{s}_fore" pos="{f(spec.L1, 0, 0)}">
            <joint name="{s}_elbow" class="arm" axis="0 1 0" range="{f(*spec.ARM_LIMITS['elbow'])}"/>
            {_inertial_mjcf("fore", s, print_scale)}
            <geom name="{s}_fore_geom"{cgb("fore")} type="capsule" size="0.022" fromto="{f(0.035, 0, 0, spec.L2 - 0.03, 0, spec.WF_DZ)}" rgba="{dark}"/>{v("fore")}
            <body name="{s}_wrist" pos="{f(spec.L2, 0, spec.WF_DZ)}">
              <joint name="{s}_wrist_flex" class="arm_small" axis="0 1 0" range="{f(*spec.ARM_LIMITS['wrist_flex'])}"/>
              {_inertial_mjcf("wrist", s, print_scale)}{v("wrist")}
              <body name="{s}_yaw" pos="{f(*yaw_pos)}">
              <joint name="{s}_wrist_yaw" class="arm_small" axis="0 0 1" range="{f(*spec.ARM_LIMITS['wrist_yaw'])}"/>
              {_inertial_mjcf("yaw", s, print_scale)}
              <geom name="{s}_wrist_geom"{cgb("yaw")} type="capsule" size="0.02" fromto="{f(0.03 - spec.YAW_DX, 0, -spec.YAW_DZ, spec.LW - 0.012 - spec.YAW_DX, 0, spec.ROLL_DZ - spec.YAW_DZ)}" rgba="{ink}"/>{v("yaw")}
              <body name="{s}_hand" pos="{f(*hand_pos)}">
                <joint name="{s}_wrist_roll" class="arm_small" axis="1 0 0" range="{f(*spec.ARM_LIMITS['wrist_roll'])}"/>
                {_inertial_mjcf("hand", s, print_scale)}
                <geom name="{s}_palm_geom"{cg} type="box" size="{f(*palm[1])}" pos="{f(*palm[0])}" rgba="0.25 0.25 0.28 1"/>{v("hand")}
                <geom name="{s}_finger_fixed_geom" class="finger" size="{f(*fix[1])}" pos="{f(*fix[0])}"/>
                <camera name="{s}_wrist_cam" pos="{f(WRIST_CAM_POS[0], sg * WRIST_CAM_POS[1], WRIST_CAM_POS[2])}" xyaxes="{wrist_cam_xyaxes(sg)}" fovy="75"/>
                <site name="{s}_tcp" pos="{f(*tcp)}" size="0.006" rgba="1 0 0 0.6"/>
                <body name="{s}_jaw" pos="{f(spec.JAW_PIVOT[0], sg * spec.JAW_PIVOT[1], spec.JAW_PIVOT[2])}">
                  <joint name="{s}_gripper" class="gripper" axis="{f(0, 0, -sg)}" range="0 {spec.GRIPPER_OPEN_RAD:.6g}"/>
                  {_inertial_mjcf("jaw", s, print_scale)}
                  <geom name="{s}_finger_jaw_geom" class="finger" size="{f(*jg[1])}" pos="{f(*jg[0])}" quat="{f(*jg[2])}"/>{v("jaw")}
                </body>
              </body>
              </body>
            </body>
          </body>
        </body>
      </body>"""


def robot_mjcf_body(meshes: list[dict] | None = None, print_scale: dict | None = None) -> str:
    """Đế + cột + xe nâng + trạm hai tay + 2 tay (cấu hình di_dong). Hình va chạm thân lấy từ CAD P-L (spec.py, nhóm "Đế", "Cột"): sàn, nan,
    đối trọng 2 tấm, khay, trụ AlohaMini2, khung T giữa, đế ngàm, ray + tai trạm, cột + gá camera đầu. Khi có mesh thân (body base/carriage)
    thì các hình này ẩn (group 3), vẫn va chạm."""
    meshes = meshes or []
    cg = ' group="3"' if any(m["place_body"] == "base" for m in meshes) else ""
    cgc = ' group="3"' if any(m["place_body"] == "carriage" for m in meshes) else ""
    wheels, spokes = [], []
    for i, th in enumerate(spec.WHEEL_ANGLES):
        c, sn = math.cos(th), math.sin(th)
        p = (spec.WHEEL_MOUNT_RADIUS * c, spec.WHEEL_MOUNT_RADIUS * sn, spec.WHEEL_RADIUS)
        wheels.append(f"""
      <body name="wheel_{i}" pos="{f(*p)}" zaxis="{f(c, sn, 0)}">
        <joint name="wheel_{i}" type="hinge" axis="0 0 1" damping="0.001"/>
        <geom name="wheel_{i}_geom" type="cylinder" size="{spec.WHEEL_RADIUS} {spec.WHEEL_WIDTH/2}" class="visual"{cg} rgba="0.15 0.16 0.18 1" mass="{WHEEL_MASS - 0.01:.6g}"/>
        <geom type="box" size="0.012 {spec.WHEEL_RADIUS*0.9:.4g} 0.021" class="visual"{cg} rgba="0.6 0.6 0.6 1" mass="0.01"/>
      </body>""")
        # nan nhôm 20x40 (M1-BS-031) trên tấm thép, từ moay-ơ tới giá servo bánh
        r0, r1 = spec.SPOKE_R
        z0, z1 = spec.SPOKE_Z
        spokes.append(f'<geom name="base_spoke_{i}"{cg} type="box" size="{f((r1 - r0) / 2, 0.02, (z1 - z0) / 2)}" '
                      f'pos="{f((r0 + r1) / 2 * c, (r0 + r1) / 2 * sn, (z0 + z1) / 2)}" euler="0 0 {th:.6g}" mass="{spec.SPOKE_MASS}" rgba="0.70 0.72 0.74 1"/>')
    base_h = (spec.BASE_Z1 - spec.BASE_Z0) / 2
    cw, cwp = spec.COUNTERWEIGHT_HALF, spec.COUNTERWEIGHT_POS
    th, tp = spec.TRAY_HALF, spec.TRAY_POS
    (cx0, cx1), cyh, (cz0, cz1) = spec.COLUMN_XR, spec.COLUMN_YH, (spec.BASE_Z1, spec.COLUMN_TOP)
    (fx0, fx1), fyh, (fz0, fz1) = spec.COLUMN_FOOT_XR, spec.COLUMN_FOOT_YH, spec.COLUMN_FOOT_Z
    (tx0, tx1), tyh, (tz0, tz1) = spec.STATION_RAIL_XR, spec.STATION_RAIL_YH, spec.STATION_RAIL_Z
    (mx0, mx1), myh, (mz0, mz1) = spec.TMID_XR, spec.TMID_YH, spec.TMID_Z
    car_z = spec.ARM_BASE_Z0
    cam_p, cam_x = spec.HEAD_CAM_POS, spec.HEAD_CAM_XYAXES

    def bx(name, xr, yr, zr, rgba="0.92 0.92 0.94 1"):
        return (f'<geom name="{name}"{cgc} type="box" size="{f((xr[1] - xr[0]) / 2, (yr[1] - yr[0]) / 2, (zr[1] - zr[0]) / 2)}" '
                f'pos="{f((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2)}" mass="0" rgba="{rgba}"/>')
    ey = spec.STATION_EAR_Y
    station = "\n        ".join([
        bx("carriage_ear_l", spec.STATION_EAR_XR, ey, spec.STATION_EAR_Z, "0.70 0.72 0.74 1"),
        bx("carriage_ear_r", spec.STATION_EAR_XR, (-ey[1], -ey[0]), spec.STATION_EAR_Z, "0.70 0.72 0.74 1"),
        bx("carriage_dock", spec.DOCK_XR, (-spec.DOCK_YH, spec.DOCK_YH), spec.DOCK_Z, "0.12 0.23 0.37 1"),
        bx("head_mast", spec.HEAD_MAST_XR, (-spec.HEAD_MAST_YH, spec.HEAD_MAST_YH), spec.HEAD_MAST_Z, "0.70 0.72 0.74 1"),
        bx("head_gimbal", spec.HEAD_GIMBAL_XR, (-spec.HEAD_GIMBAL_YH, spec.HEAD_GIMBAL_YH), spec.HEAD_GIMBAL_Z, "0.12 0.23 0.37 1")])
    return f"""
    <body name="base" pos="0 0 0">
      <joint name="base_x" type="slide" axis="1 0 0" damping="5"/>
      <joint name="base_y" type="slide" axis="0 1 0" damping="5"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1" damping="2"/>
      <geom name="base_geom"{cg} type="cylinder" size="{spec.BASE_RADIUS} {base_h:.4g}" pos="{f(0, 0, spec.BASE_Z0 + base_h)}" mass="{spec.BASE_MASS}" rgba="0.92 0.92 0.94 1"/>{_vis("base", meshes)}
      {''.join(spokes)}
      <geom name="base_counterweight"{cg} type="box" size="{f(*cw)}" pos="{f(*cwp)}" mass="{spec.COUNTERWEIGHT_MASS}" rgba="0.35 0.36 0.38 1"/>
      <geom name="base_tray"{cg} type="box" size="{f(*th)}" pos="{f(*tp)}" mass="{spec.TRAY_MASS}" rgba="0.95 0.94 0.91 1"/>
      <geom name="base_logo"{cg} type="box" size="0.002 0.06 0.02" pos="{f(spec.BASE_RADIUS, 0, spec.BASE_Z1 + 0.035)}" class="visual" rgba="0.12 0.23 0.37 1"/>
      <geom name="column_geom"{cg} type="box" size="{f((cx1 - cx0) / 2, cyh, (cz1 - cz0) / 2)}" pos="{f((cx0 + cx1) / 2, 0, (cz0 + cz1) / 2)}" mass="{spec.COLUMN_MASS}" rgba="0.95 0.94 0.91 1"/>
      <geom name="column_foot_geom"{cg} type="box" size="{f((fx1 - fx0) / 2, fyh, (fz1 - fz0) / 2)}" pos="{f((fx0 + fx1) / 2, 0, (fz0 + fz1) / 2)}" mass="0" rgba="0.95 0.94 0.91 1"/>
      {''.join(wheels)}
      <body name="carriage" pos="{f(0, 0, car_z)}">  <!-- gốc xe nâng = đáy J1 (gốc tay, mặt trên gá tay của trạm); trục vai cao hơn D0. Trạm hai tay (di_dong) gắn cứng vào xe nâng -->
        <joint name="lift" type="slide" axis="0 0 1" range="{f(*spec.LIFT_RANGE)}" damping="50" armature="1"/>
        <geom name="carriage_geom"{cgc} type="box" size="{f((tx1 - tx0) / 2, tyh, (tz1 - tz0) / 2)}" pos="{f((tx0 + tx1) / 2, 0, (tz0 + tz1) / 2)}" mass="{spec.CARRIAGE_MASS + spec.STATION_MASS + 2 * spec.LINK_MASS['mount']:.6g}" rgba="0.70 0.72 0.74 1"/>{_vis("carriage", meshes)}
        <geom name="carriage_rack"{cgc} type="box" size="{f((mx1 - mx0) / 2, myh, (mz1 - mz0) / 2)}" pos="{f((mx0 + mx1) / 2, 0, (mz0 + mz1) / 2)}" mass="0" rgba="0.3 0.3 0.33 1"/>
        {station}
        <camera name="head" pos="{f(*cam_p)}" xyaxes="{f(*cam_x)}" fovy="{spec.HEAD_CAM_FOVY:g}"/>  <!-- camera đầu trên cột trạm, chúi {spec.HEAD_CAM_TILT_DEG:g}° -->
        {_arm_mjcf('left', meshes, print_scale)}
        {_arm_mjcf('right', meshes, print_scale)}
      </body>
    </body>"""


def arm_kp(j: str) -> tuple[float, float]:
    """(kp, forcerange) của actuator vị trí: forcerange = TORQUE_LIMIT nhóm (spec.TORQUE_LIMIT_PCT, % mô-men chặn), kp để đạt forcerange ở
    lệch ARM_KP_SAT_DEG (servo STS hộp số 1:345 giữ vị trí cứng; [giả định, chưa đo độ võng servo thật])."""
    tau = spec.ARM_SERVO[j].stall_nm * spec.TORQUE_LIMIT_PCT[j] / 100 * spec.ARM_SERVO_COUNT[j]
    return tau / math.radians(spec.ARM_KP_SAT_DEG), tau


def _actuators() -> str:
    out = []
    for s in spec.SIDES:
        for j in spec.ARM_JOINTS:
            kp, tau = arm_kp(j)
            lo, hi = spec.ARM_LIMITS[j]
            out.append(f'<position name="{s}_{j}" joint="{s}_{j}" kp="{kp:.4g}" kv="{kp * 0.06:.3g}" '
                       f'ctrlrange="{f(lo, hi)}" forcerange="{f(-tau, tau)}"/>')
        kpg = GRIP_TORQUE_NM / 0.10   # bão hoà khi lệch 0,10 rad (~8 mm ở đầu hàm)
        out.append(f'<position name="{s}_gripper" joint="{s}_gripper" kp="{kpg:.4g}" kv="{kpg * 0.05:.3g}" '
                   f'ctrlrange="0 {spec.GRIPPER_OPEN_RAD:.6g}" forcerange="{f(-GRIP_TORQUE_NM, GRIP_TORQUE_NM)}"/>')
    out.append(f'<position name="lift" joint="lift" kp="20000" kv="2000" ctrlrange="{f(*spec.LIFT_RANGE)}" forcerange="-600 600"/>')
    out.append('<position name="base_x" joint="base_x" kp="20000" kv="3000"/>')
    out.append('<position name="base_y" joint="base_y" kp="20000" kv="3000"/>')
    out.append('<position name="base_yaw" joint="base_yaw" kp="3000" kv="400"/>')
    for i in range(3):
        out.append(f'<velocity name="wheel_{i}" joint="wheel_{i}" kv="0.05"/>')
    return "\n    ".join(out)


def _contact_excludes() -> str:
    out = []
    for s in spec.SIDES:
        out.append(f'<exclude body1="{s}_jaw" body2="{s}_wrist"/>')
        out.append(f'<exclude body1="{s}_jaw" body2="{s}_yaw"/>')
        out.append(f'<exclude body1="{s}_hand" body2="{s}_wrist"/>')
        out.append(f'<exclude body1="{s}_yaw" body2="{s}_fore"/>')
        out.append(f'<exclude body1="{s}_hand" body2="{s}_fore"/>')
        out.append(f'<exclude body1="carriage" body2="{s}_upper"/>')
    return "\n    ".join(out)


def robot_mjcf(scene: bool = True, meshdir: str | None = None, print_scale: dict | None = None) -> str:
    """MJCF đầy đủ. scene=True: kèm sàn, bàn, hộp, đích (cảnh demo).
    meshdir: thư mục mesh ghi vào <compiler>; None = đường dẫn tuyệt đối tới sim/assets (để from_xml_string chạy ở mọi thư mục);
    file trong sim/ ghi "assets" (tương đối theo file)."""
    meshes = load_meshes()
    meshdir = str(MESH_DIR) if meshdir is None else meshdir
    compiler = f' meshdir="{meshdir}"' if meshes else ""
    assets = "".join(f'\n    <mesh name="{m["name"]}" file="{m["file"]}"/>' for m in meshes)
    if meshes:
        assets += "".join(f'\n    <material name="m1_{k}" rgba="{_rgba(h)}" specular="{sp}" shininess="{sh}" reflectance="{rf}"/>'
                          for k, (h, sp, sh, rf) in MATERIALS.items())
    fcg = ' group="3"' if meshes else ""
    world = ""
    if scene:
        tx = TABLE_EDGE_X + TABLE_DEPTH / 2
        th = TABLE_HEIGHT
        legs = "".join(
            f'<geom name="table_leg{i}" type="box" size="0.02 0.02 {th/2 - 0.015:.4g}" pos="{f(dx, dy, th/2 - 0.015)}" rgba="0.45 0.3 0.2 1"/>'
            for i, (dx, dy) in enumerate([(-0.27, -0.57), (-0.27, 0.57), (0.27, -0.57), (0.27, 0.57)]))
        cam = lookat_xyaxes((0.95, -1.75, 1.45), (0.9, -0.05, 0.7))
        world = f"""
    <geom name="floor" type="plane" size="6 6 0.1" material="grid"/>
    <geom name="wall_back" type="box" size="0.05 4 0.8" pos="3.2 0 0.8" rgba="0.85 0.86 0.9 1"/>
    <geom name="wall_side" type="box" size="4 0.05 0.8" pos="0 2.2 0.8" rgba="0.88 0.88 0.92 1"/>
    <camera name="overview" pos="0.95 -1.75 1.45" xyaxes="{cam}" fovy="50"/>
    <camera name="side" pos="1.1 -2.3 1.25" xyaxes="{lookat_xyaxes((1.1, -2.3, 1.25), (1.0, 0.0, 0.75))}" fovy="45"/>
    <body name="table" pos="{f(tx, 0, 0)}">
      <geom name="table_top" type="box" size="{f(TABLE_DEPTH/2, TABLE_HALF_Y, 0.015)}" pos="{f(0, 0, th - 0.015)}" rgba="0.75 0.6 0.42 1"/>
      {legs}
    </body>
    <geom name="target_marker" type="cylinder" size="0.04 0.0008" pos="{f(BOX_TARGET[0], BOX_TARGET[1], th + 0.0008)}" class="visual" rgba="0.9 0.1 0.1 0.8"/>
    <body name="box" pos="{f(BOX_START[0], BOX_START[1], th + BOX_HALF[2])}">
      <freejoint name="box"/>
      <geom name="box_geom" type="box" size="{f(*BOX_HALF)}" mass="{BOX_MASS}" rgba="0.1 0.7 0.3 1" friction="1.5 0.02 0.001"/>
    </body>"""
    return f"""<mujoco model="faibotic_m1">
  <!-- SPDX-License-Identifier: Apache-2.0. Copyright (c) 2026 Leo Dao (Faibotic). Meshes in assets/ carry their own licence per file, listed in NOTICE at the repository root. -->
  <!-- Faibotic M1 v0.5 P-L. File sinh tự động bởi faibotic.model; sửa spec.py rồi xuất lại. -->
  <compiler angle="radian" autolimits="true"{compiler}/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="8192" offsamples="8"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.82 0.84 0.86" rgb2="0.74 0.76 0.78"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.95 0.96 0.98" rgb2="0.75 0.8 0.88" width="256" height="256"/>{assets}
  </asset>
  <default>
    <geom friction="1 0.01 0.001" solref="0.004 1"/>
    <default class="visual"><geom contype="0" conaffinity="0" group="1"/></default>
    <default class="vis"><geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/></default>
    <default class="arm"><joint damping="1.5" armature="0.05" frictionloss="0.05"/></default>
    <default class="arm_small"><joint damping="0.3" armature="0.01" frictionloss="0.02"/></default>
    <default class="gripper"><joint type="hinge" damping="0.05" armature="0.005" frictionloss="0.005"/></default>
    <default class="finger">
      <geom type="box" friction="1.5 0.02 0.001" condim="4" mass="0" rgba="0.15 0.15 0.15 1"{fcg}/>
    </default>
  </default>
  <worldbody>
    <light pos="1 -1 3" dir="-0.2 0.3 -1" directional="true" diffuse="0.8 0.8 0.8"/>
    <light pos="-1 1 3" dir="0.3 -0.2 -1" directional="true" diffuse="0.3 0.3 0.3" castshadow="false"/>
    {world}
    {robot_mjcf_body(meshes, print_scale)}
  </worldbody>
  <contact>
    {_contact_excludes()}
  </contact>
  <actuator>
    {_actuators()}
  </actuator>
</mujoco>
"""


# ---------------- URDF ----------------

def _inertial(m: float, s: float = 0.03, com=(0.0, 0.0, 0.0)) -> str:
    i = m * s * s / 6
    return f'<inertial><origin xyz="{f(*com)}"/><mass value="{m:.4g}"/><inertia ixx="{i:.3g}" iyy="{i:.3g}" izz="{i:.3g}" ixy="0" ixz="0" iyz="0"/></inertial>'


def _inertial_urdf(body: str, side: str) -> str:
    m, c, I = body_inertial(body, side)
    return (f'<inertial><origin xyz="{f(*c)}"/><mass value="{m:.6g}"/><inertia ixx="{I[0, 0]:.6g}" iyy="{I[1, 1]:.6g}" izz="{I[2, 2]:.6g}" '
            f'ixy="{I[0, 1]:.6g}" ixz="{I[0, 2]:.6g}" iyz="{I[1, 2]:.6g}"/></inertial>')


def robot_urdf() -> str:
    L = []
    J = []
    base_h = spec.BASE_Z1 - spec.BASE_Z0
    (cx0, cx1), cyh = spec.COLUMN_XR, spec.COLUMN_YH
    col_h = spec.COLUMN_TOP - spec.BASE_Z1
    cw, cwp = spec.COUNTERWEIGHT_HALF, spec.COUNTERWEIGHT_POS
    th, tp = spec.TRAY_HALF, spec.TRAY_POS
    base_m = spec.BASE_MASS + spec.COLUMN_MASS + spec.COUNTERWEIGHT_MASS + spec.TRAY_MASS + 3 * spec.SPOKE_MASS
    L.append(f"""<link name="base_link">{_inertial(base_m, 0.2, (cwp[0] * spec.COUNTERWEIGHT_MASS / base_m, 0, 0.15))}
    <visual><origin xyz="{f(0, 0, spec.BASE_Z0 + base_h/2)}"/><geometry><cylinder radius="{spec.BASE_RADIUS}" length="{base_h:.4g}"/></geometry></visual>
    <collision><origin xyz="{f(0, 0, spec.BASE_Z0 + base_h/2)}"/><geometry><cylinder radius="{spec.BASE_RADIUS}" length="{base_h:.4g}"/></geometry></collision>
    <visual><origin xyz="{f(*cwp)}"/><geometry><box size="{f(*(2 * np.array(cw)))}"/></geometry></visual>
    <collision><origin xyz="{f(*cwp)}"/><geometry><box size="{f(*(2 * np.array(cw)))}"/></geometry></collision>
    <visual><origin xyz="{f(*tp)}"/><geometry><box size="{f(*(2 * np.array(th)))}"/></geometry></visual>
    <collision><origin xyz="{f(*tp)}"/><geometry><box size="{f(*(2 * np.array(th)))}"/></geometry></collision>
    <visual><origin xyz="{f((cx0 + cx1) / 2, 0, spec.BASE_Z1 + col_h/2)}"/><geometry><box size="{f(cx1 - cx0, 2 * cyh, col_h)}"/></geometry></visual>
    <collision><origin xyz="{f((cx0 + cx1) / 2, 0, spec.BASE_Z1 + col_h/2)}"/><geometry><box size="{f(cx1 - cx0, 2 * cyh, col_h)}"/></geometry></collision>
  </link>""")
    for i, a in enumerate(spec.WHEEL_ANGLES):
        c, sn = math.cos(a), math.sin(a)
        L.append(f'<link name="wheel_{i}">{_inertial(WHEEL_MASS)}<visual><geometry><cylinder radius="{spec.WHEEL_RADIUS}" length="{spec.WHEEL_WIDTH}"/></geometry></visual></link>')
        J.append(f"""<joint name="wheel_{i}" type="continuous"><parent link="base_link"/><child link="wheel_{i}"/>
    <origin xyz="{f(spec.WHEEL_MOUNT_RADIUS*c, spec.WHEEL_MOUNT_RADIUS*sn, spec.WHEEL_RADIUS)}" rpy="{f(0, math.pi/2, a)}"/><axis xyz="0 0 1"/></joint>""")
    car_z = spec.ARM_BASE_Z0
    (tx0, tx1), tyh, (tz0, tz1) = spec.STATION_RAIL_XR, spec.STATION_RAIL_YH, spec.STATION_RAIL_Z
    tbox = f'<origin xyz="{f((tx0 + tx1) / 2, 0, (tz0 + tz1) / 2)}"/><geometry><box size="{f(tx1 - tx0, 2 * tyh, tz1 - tz0)}"/></geometry>'
    L.append(f"""<link name="carriage">{_inertial(spec.CARRIAGE_MASS + spec.STATION_MASS + 2 * spec.LINK_MASS['mount'], 0.15)}
    <visual>{tbox}</visual>
    <collision>{tbox}</collision></link>""")
    J.append(f"""<joint name="lift" type="prismatic"><parent link="base_link"/><child link="carriage"/>
    <origin xyz="{f(0, 0, car_z)}"/><axis xyz="0 0 1"/><limit lower="{spec.LIFT_RANGE[0]}" upper="{spec.LIFT_RANGE[1]}" effort="600" velocity="0.2"/></joint>""")

    def cap_x(p0, p1, r):
        p0, p1 = np.array(p0, float), np.array(p1, float)
        d = p1 - p0
        ln = float(np.linalg.norm(d))
        pitch = math.atan2(math.hypot(d[0], d[1]), d[2])
        yaw = math.atan2(d[1], d[0])
        return f'<origin xyz="{f(*((p0 + p1) / 2))}" rpy="{f(0, pitch, yaw)}"/><geometry><cylinder radius="{r}" length="{ln:.4g}"/></geometry>'

    for s in spec.SIDES:
        sy = spec.SHOULDER_Y if s == "left" else -spec.SHOULDER_Y
        sg = 1.0 if s == "left" else -1.0
        chain = [
            ("pan", "carriage", f"{s}_shoulder", (spec.SHOULDER_X, sy, 0), "0 0 1", "shoulder",
             '<origin xyz="0.03 0 0.105"/><geometry><box size="0.07 0.08 0.09"/></geometry>'),
            ("shoulder", f"{s}_shoulder", f"{s}_upper", (spec.SHOULDER_DR, 0, spec.D0), "0 1 0", "upper", cap_x((0.03, 0, 0.012), (spec.L1 - 0.035, 0, 0.012), 0.026)),
            ("elbow", f"{s}_upper", f"{s}_fore", (spec.L1, 0, 0), "0 1 0", "fore", cap_x((0.035, 0, 0), (spec.L2 - 0.03, 0, spec.WF_DZ), 0.022)),
            ("wrist_flex", f"{s}_fore", f"{s}_wrist", (spec.L2, 0, spec.WF_DZ), "0 1 0", "wrist",
             cap_x((0.005, sg * spec.ROLL_DY, 0), (spec.YAW_DX, sg * spec.ROLL_DY, spec.YAW_DZ), 0.015)),
            # khớp yaw (6+1) như URDF upstream left_wrist_yaw_joint
            ("wrist_yaw", f"{s}_wrist", f"{s}_yaw", (spec.YAW_DX, sg * spec.ROLL_DY, spec.YAW_DZ), "0 0 1", "yaw",
             cap_x((0.03 - spec.YAW_DX, 0, -spec.YAW_DZ), (spec.LW - 0.012 - spec.YAW_DX, 0, spec.ROLL_DZ - spec.YAW_DZ), 0.02)),
            ("wrist_roll", f"{s}_yaw", f"{s}_hand", (spec.LW - spec.YAW_DX, 0, spec.ROLL_DZ - spec.YAW_DZ), "1 0 0", "hand",
             (lambda b: f'<origin xyz="{f(*b[0])}"/><geometry><box size="{f(*(2 * np.array(b[1])))}"/></geometry>')(_box(*PALM_BOX, sg))),
        ]
        for j, parent, child, xyz, axis, body, geo in chain:
            lo, hi = spec.ARM_LIMITS[j]
            eff = arm_kp(j)[1]
            L.append(f'<link name="{child}">{_inertial_urdf(body, s)}<visual>{geo}</visual><collision>{geo}</collision></link>')
            J.append(f"""<joint name="{s}_{j}" type="revolute"><parent link="{parent}"/><child link="{child}"/>
    <origin xyz="{f(*xyz)}"/><axis xyz="{axis}"/><limit lower="{lo:.6g}" upper="{hi:.6g}" effort="{eff:.4g}" velocity="3.0"/></joint>""")
        tcp = (GRIP_TCP_X, sg * spec.TCP_DY, spec.TCP_DZ)
        # hàm động quay quanh trục kẹp (góc mở từ chỗ khép); hộp ngón cố định trong link hand
        jc, jh, jq = jaw_geom(sg)
        fc, fh = _box(*FIXED_FINGER_BOX, sg)
        fgeo = f'<origin xyz="{f(*fc)}"/><geometry><box size="{f(*(2 * np.array(fh)))}"/></geometry>'
        L[-1] = L[-1].replace("</link>", f"<collision>{fgeo}</collision></link>")
        jgeo = f'<origin xyz="{f(*jc)}" rpy="0 0 {sg * JAW_BETA:.6g}"/><geometry><box size="{f(*(2 * np.array(jh)))}"/></geometry>'
        L.append(f'<link name="{s}_jaw">{_inertial_urdf("jaw", s)}<visual>{jgeo}</visual><collision>{jgeo}</collision></link>')
        J.append(f"""<joint name="{s}_gripper" type="revolute"><parent link="{s}_hand"/><child link="{s}_jaw"/>
    <origin xyz="{f(spec.JAW_PIVOT[0], sg * spec.JAW_PIVOT[1], spec.JAW_PIVOT[2])}"/><axis xyz="{f(0, 0, -sg)}"/><limit lower="0" upper="{spec.GRIPPER_OPEN_RAD:.6g}" effort="{GRIP_TORQUE_NM:.4g}" velocity="5.0"/></joint>""")
        L.append(f'<link name="{s}_tcp"/>')
        J.append(f'<joint name="{s}_tcp_fixed" type="fixed"><parent link="{s}_hand"/><child link="{s}_tcp"/><origin xyz="{f(*tcp)}"/></joint>')
    meshes = load_meshes()
    mj = ""
    if meshes:   # hình hiển thị = mesh (sim/assets/); hình va chạm giữ hình khối
        import re
        rgba = {k: _rgba(h) for k, (h, *_r) in MATERIALS.items()}
        mats = "".join(f'<material name="m1_{k}"><color rgba="{c}"/></material>' for k, c in rgba.items())
        for i, ln in enumerate(L):
            name = re.match(r'<link name="([^"]+)"', ln).group(1)
            ms = [m for m in meshes if m["place_body"] == ("base" if name == "base_link" else name)]
            if ms:
                ln = re.sub(r"<visual>.*?</visual>", "", ln, flags=re.S)
                vis = "".join(f'<visual><geometry><mesh filename="assets/{m["file"]}"/></geometry><material name="m1_{m["material"]}"/></visual>' for m in ms)
                L[i] = ln.replace("</link>", vis + "</link>")
        L.insert(0, mats)
        mj = '<mujoco><compiler meshdir="assets" discardvisual="true"/></mujoco>\n  '
    body = mj + "\n  ".join(L + J)
    return f"""<?xml version="1.0"?>
<!-- SPDX-License-Identifier: Apache-2.0. Copyright (c) 2026 Leo Dao (Faibotic). Meshes in assets/ carry their own licence per file, listed in NOTICE at the repository root. -->
<!-- Faibotic M1 v0.5 P-L. File sinh tự động bởi faibotic.model từ spec.py. Đế di động: base_link gắn sàn (đế omni xử lý ngoài URDF).
     Góc khớp theo quy ước SDK (spec.URDF_TO_SDK đổi từ góc URDF upstream AM-ARM200). -->
<robot name="faibotic_m1">
  {body}
</robot>
"""


def write_all(out: Path) -> list[Path]:
    out.mkdir(parents=True, exist_ok=True)
    files = {"m1_scene.xml": robot_mjcf(True, meshdir="assets"), "m1.xml": robot_mjcf(False, meshdir="assets"), "m1.urdf": robot_urdf()}
    paths = []
    for name, txt in files.items():
        p = out / name
        p.write_text(txt, encoding="utf-8")
        paths.append(p)
    return paths


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="sim")
    a = ap.parse_args()
    for p in write_all(Path(a.out)):
        print(p)
