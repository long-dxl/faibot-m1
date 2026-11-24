# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Thân servo thật cho lắp ráp Faibot M1 (v0.4.6, 23/11/2026): hình học, tư thế đặt, mesh hiển thị. build123d + numpy.
Phần mã và mô hình STS3095 là của Faibotic. Mesh STS3215 lấy từ MuJoCo Menagerie robotstudio_so101 (Apache-2.0),
xem cad/upstream/menagerie-so101/README.md.

Hai loại servo, cùng một quy ước "hệ servo": trục ra là trục z, còi ở phía +z, thân kéo dài theo -x tính từ trục.
- STS3215 (M1-SV-001): hệ servo = hệ mesh Menagerie sts3215_03a_v1.stl (mm). Trục ra tại (12,5; 0), mặt tựa còi z 16,2, còi Ø20 tới z 18,7, đầu vít tới 20,2.
  Thân 45,4 x 24,8, z -15,8..16,2; nắp đáy 39,9 x 20 tới z -17,3; vấu đáy tới -18,3 (đo mặt cắt mesh, archive/v0.4/cad/CHANGES.md v0.4.6).
  Khối bao sạch sts3215_env() bám các mặt cắt đó (lệch mesh <= 1 mm ngoài các gờ nhỏ) và là hình dùng cho --check; mesh chỉ để hiển thị.
- STS3095 (M1-SV-002): dựng sạch bằng build123d trong đúng hộp bao đang dùng (params.py: 65 x 30 x 48 [trang bên bán], trục cách đầu 10 [giả định]):
  thân, hai tai bắt dọc cạnh dài (lỗ bắt không vẽ), cổng cáp lõm, còi R12,5 x 3,5 có 4 lỗ M3 PCD 20 (shoulder.py). Hình dạng tai và vị trí cổng cáp [giả định], chưa có bản vẽ Feetech.
Tư thế đặt: pose(P, h, b) với P = điểm trên trục ra tại mặt tựa còi, h = hướng còi, b = hướng từ trục về giữa thân (dọc cạnh dài)."""
import os
import numpy as np
from build123d import Box, Cylinder, Pos, Align, Location, Plane

HERE = os.path.dirname(os.path.abspath(__file__))
MEN = os.path.join(HERE, "..", "upstream", "menagerie-so101")
MESH_FULL = os.path.join(MEN, "sts3215_03a_v1.stl")                       # nguyên bản Menagerie (render)
MESH_SIM = os.path.join(MEN, "derived", "sts3215_03a_v1_sim.stl")          # rút gọn (sim, GLB): xem README
MIN = (Align.MIN,) * 3

# ---------------- STS3215 ----------------
S15_SHAFT_X = 12.5          # trục ra (x) trong hệ mesh
S15_SEAT_Z = 16.2           # mặt tựa còi (đỉnh thân)
S15_HORN_Z1, S15_SCREW_Z1 = 18.7, 20.2
S15_HORN_R, S15_SCREW_R = 10.0, 2.7

def sts3215_env():
    """Khối bao STS3215 trong hệ servo (mm): thân + nắp đáy + vấu đáy + còi + đầu vít."""
    body = Pos(-22.7, -12.4, -15.8) * Box(45.4, 24.8, S15_SEAT_Z + 15.8, align=MIN)
    cap = Pos(-17.4, -10.0, -17.3) * Box(39.9, 20.0, 1.5, align=MIN)
    boss = Pos(-2.6, -8.6, -18.3) * Box(18.1, 17.2, 1.0, align=MIN)
    horn = Pos(S15_SHAFT_X, 0, S15_SEAT_Z) * Cylinder(S15_HORN_R, S15_HORN_Z1 - S15_SEAT_Z, align=(Align.CENTER, Align.CENTER, Align.MIN))
    screw = Pos(S15_SHAFT_X, 0, S15_HORN_Z1) * Cylinder(S15_SCREW_R, S15_SCREW_Z1 - S15_HORN_Z1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body + cap + boss + horn + screw

# ---------------- STS3095 ----------------
S95_L, S95_W, S95_H, S95_E = 65.0, 30.0, 48.0, 10.0      # = params.SERVO_L, SERVO_W, SERVO_H, SHAFT_FROM_END
S95_HORN_R, S95_HORN_T, S95_PCD, S95_HOLE = 12.5, 3.5, 20.0, 3.4
EAR_T, EAR_Z = 4.0, -17.0     # tai bắt dày 4 (dọc trục), mặt dưới tai cách mặt tựa còi 17 [giả định]
EAR_IN = 3.5                  # tai lồi 3,5 mỗi bên cạnh 48 (thân chính 41 x 30, tai tới 48) [giả định]

def sts3095_body():
    """Thân STS3095 trong hệ servo (mm, không gồm còi): x -55..10, y ±24, z -30..0 (mặt tựa còi z 0). Nằm trọn trong hộp 65 x 48 x 30 của params.py."""
    x0 = S95_E - S95_L
    core = Pos(x0, -(S95_H / 2 - EAR_IN), -S95_W) * Box(S95_L, S95_H - 2 * EAR_IN, S95_W, align=MIN)
    ears = Pos(x0, -S95_H / 2, EAR_Z) * Box(S95_L, S95_H, EAR_T, align=MIN)     # lỗ bắt trên tai không vẽ (giữ lưới nhẹ cho GLB)
    port = Pos(x0, -6.0, -S95_W + 6.0) * Box(3.0, 12.0, 6.0, align=MIN)     # cổng cáp lõm ở đầu sau [giả định]
    return core + ears - port

def sts3095_horn():
    h = Cylinder(S95_HORN_R, S95_HORN_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for k in range(4):
        a = np.radians(45 + 90 * k)
        h -= Pos(S95_PCD / 2 * np.cos(a), S95_PCD / 2 * np.sin(a), 0) * Cylinder(S95_HOLE / 2, S95_HORN_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return h - Cylinder(3.0, S95_HORN_T, align=(Align.CENTER, Align.CENTER, Align.MIN))

def sts3095():
    return sts3095_body() + sts3095_horn()

# ---------------- tư thế ----------------
def pose(model, P, h, b):
    """Ma trận 4x4 (mm) hệ servo -> hệ đặt. P: điểm trên trục ra ở mặt tựa còi; h: hướng còi; b: từ trục về giữa thân (vuông góc h)."""
    h = np.asarray(h, float) / np.linalg.norm(h); b = np.asarray(b, float); b = b - h * (b @ h); b /= np.linalg.norm(b)
    x = -b; z = h; y = np.cross(z, x)
    R = np.column_stack([x, y, z]); M = np.eye(4); M[:3, :3] = R
    s0 = np.array([S15_SHAFT_X, 0.0, S15_SEAT_Z]) if model == "sts3215" else np.zeros(3)
    M[:3, 3] = np.asarray(P, float) - R @ s0
    return M

def m2loc(M):
    return Location(Plane(origin=tuple(M[:3, 3]), x_dir=tuple(M[:3, 0]), z_dir=tuple(M[:3, 2])))

def loc2m(loc):
    t = loc.wrapped.Transformation(); M = np.eye(4)
    for i in range(3):
        for j in range(4):
            M[i, j] = t.Value(i + 1, j + 1)
    return M

MIRROR_XZ = np.diag([1.0, -1.0, 1.0, 1.0])
def mirror_proper(M):
    """Gương qua mặt y = 0 (tay phải) nhưng giữ phép quay đúng (đổi dấu trục y servo): servo thật không lật gương được.
    Khối bao và mesh gần đối xứng theo y servo nên hình vẫn trùng gương trong +-0,1 mm (khối bao) / chi tiết nhỏ (mesh)."""
    R = MIRROR_XZ @ M; R[:3, 1] *= -1; return R

def solid(model, M):
    return m2loc(M) * (sts3215_env() if model == "sts3215" else sts3095())

_MESH = {}
def mesh(model, M, detail="sim"):
    """Mesh hiển thị (V mm, F) đặt theo M; None nếu loại servo không có mesh (STS3095: dùng tessellate của solid)."""
    if model != "sts3215":
        return None
    if detail not in _MESH:
        import trimesh
        m = trimesh.load(MESH_FULL if detail == "full" else MESH_SIM, process=True)
        _MESH[detail] = (np.asarray(m.vertices) * 1000.0, np.asarray(m.faces, dtype=np.int64))
    V, F = _MESH[detail]
    return (M[:3, :3] @ V.T).T + M[:3, 3], F.copy()
