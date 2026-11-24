"""Mesh hiển thị của sim (sim/assets/, P-L: `cad/faibotic/export_sim_meshes.py --pl`) khớp tay AM-ARM200 gốc + kẹp gốc AM-ARM200.

1. Đặt chi tiết: tâm khối từng chi tiết STEP (trừ t) so tâm khối link URDF upstream (meshes.json source.com_check_mm) <= 8 mm với link một chi tiết
   (STEP và URDF lệch nhau ~5 mm theo z ở cổ tay, cad/CHANGES.md v0.4.6).
2. Đổi góc: tâm mesh cánh tay/cẳng tay đặt theo FK MuJoCo ở nhiều tư thế trùng điểm đó đi theo FK URDF upstream (spec.URDF_TO_SDK) <= 0,5 mm.
3. Mesh không đổi vật lý: geom mesh không va chạm, không khối lượng; kích thước, NOTICE."""
import hashlib
import json
import struct
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

from faibotic import model as M
from faibotic import spec

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "sim" / "assets"
pytestmark = pytest.mark.skipif(not (ASSETS / "meshes.json").exists(), reason="chưa xuất mesh sim")
MJ = json.loads((ASSETS / "meshes.json").read_text()) if (ASSETS / "meshes.json").exists() else {}


@pytest.fixture(scope="module")
def mdl():
    return mujoco.MjModel.from_xml_string(M.robot_mjcf(scene=False))


def _stl(name):
    b = (ASSETS / name).read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    return np.frombuffer(b[84:], dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n)["v"].reshape(-1, 3).astype(float)


def test_meshes_are_pl_and_hashes_match():
    assert MJ["source"]["variant"] == "P-L" and len(M.load_meshes()) == len(MJ["meshes"])
    for m in MJ["meshes"]:
        assert hashlib.sha256((ASSETS / m["file"]).read_bytes()).hexdigest() == m["sha256"], m["file"]


def test_step_parts_match_urdf_com():
    chk = MJ["source"]["com_check_mm"]
    for n in ("OB_F_J3_Shoulder", "OB_F_J4_Elbow", "OB_F_J5_Elbow_Wrist_Bracket"):   # link chỉ gồm một chi tiết STEP
        a, b = np.array(chk[n]["com_step_minus_t"]), np.array(chk[n]["com_urdf_link"])
        assert np.linalg.norm(a - b) <= 8.0, (n, a, b)


@pytest.mark.parametrize("q", [[0.3, -0.5, 1.2, 0.6, 0.0, 0.4], [-0.6, -1.2, 1.6, 1.4, 0.0, -1.0],
                               [0.3, -0.5, 1.2, 0.6, 0.7, 0.4], [-0.6, -1.2, 1.6, 1.4, -1.2, -1.0]])   # 6+1: có yaw khác 0
def test_mesh_follows_urdf_fk(mdl, q):
    """Điểm cố định trên mesh (tâm đỉnh) đi theo FK MuJoCo bằng FK URDF upstream sau đổi góc spec.URDF_TO_SDK.
    (6+1): gồm vỏ J56 (body yaw, link URDF left_wrist_yaw) để kiểm dấu và vị trí khớp yaw."""
    sys.path.insert(0, str(ROOT / "sim" / "tools"))
    import am_arm200_pl as A
    d = mujoco.MjData(mdl)
    lift = 0.2
    d.qpos[mdl.joint("lift").qposadr[0]] = lift
    for j, v in zip(spec.ARM_JOINTS, q):
        d.qpos[mdl.joint(f"left_{j}").qposadr[0]] = v
    mujoco.mj_forward(mdl, d)
    a = np.radians(spec.UPPER_URDF_DEG)
    T0 = A.fk({"left_shoulder_lift": -a, "left_elbow_flex": a})          # q_sdk = 0
    names = {"pan": "left_shoulder_pan", "shoulder": "left_shoulder_lift", "elbow": "left_elbow_flex", "wrist_flex": "left_wrist_flex", "wrist_yaw": "left_wrist_yaw_joint"}
    Tq = A.fk({names[j]: (v - spec.URDF_TO_SDK[j][1]) / spec.URDF_TO_SDK[j][0] for j, v in zip(spec.ARM_JOINTS, q) if j in names})
    mount = np.array([spec.SHOULDER_X, spec.SHOULDER_Y, spec.ARM_BASE_Z0 + lift])
    origin0 = {"upper": np.array([spec.SHOULDER_DR, 0, spec.D0]), "fore": np.array([spec.SHOULDER_DR + spec.L1, 0, spec.D0])}
    origin0["yaw"] = origin0["fore"] + [spec.L2 + spec.YAW_DX, spec.ROLL_DY, spec.WF_DZ + spec.YAW_DZ]
    for body, link, fn in (("upper", "left_Upper_Arm", "left_upper__ivory.stl"), ("fore", "left_Lower_Arm", "left_fore__navy.stl"),
                           ("yaw", "left_wrist_yaw", "left_yaw__navy.stl")):
        c_b = _stl(fn).mean(0)                                  # khung body
        w0 = A.R_SDK.T @ (origin0[body] + c_b)                  # hệ URDF ở q_sdk = 0
        R0, p0 = T0[link]
        loc = R0.T @ (w0 - p0)
        Rq, pq = Tq[link]
        ref = mount + A.R_SDK @ (pq + Rq @ loc)
        bid = mdl.body(f"left_{body}").id
        sim = d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ c_b
        assert np.linalg.norm(sim - ref) <= 5e-4, (body, sim, ref)


def test_visual_meshes_do_not_change_physics(mdl, monkeypatch):
    vis = [i for i in range(mdl.ngeom) if mdl.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH]
    assert len(vis) == len(MJ["meshes"])
    for i in vis:
        assert mdl.geom_contype[i] == 0 and mdl.geom_conaffinity[i] == 0 and mdl.geom_group[i] == 2
    monkeypatch.setattr(M, "load_meshes", lambda: [])      # mô hình hình khối
    m0 = mujoco.MjModel.from_xml_string(M.robot_mjcf(scene=False))
    assert mdl.nbody == m0.nbody and mdl.njnt == m0.njnt
    assert mdl.body_mass == pytest.approx(m0.body_mass)
    assert mdl.body_inertia == pytest.approx(m0.body_inertia)
    assert mdl.body_ipos == pytest.approx(m0.body_ipos)


def test_mesh_assets_size_and_notice():
    size = sum(p.stat().st_size for p in ASSETS.glob("*.stl"))
    assert size <= 20e6
    assert (ASSETS / "NOTICE").exists() and (ASSETS / "LICENSE").exists()
    t = (ASSETS / "NOTICE").read_text()
    assert "AM-ARM" in t and "CERN-OHL-P" not in t      # không còn hình kẹp cũ
