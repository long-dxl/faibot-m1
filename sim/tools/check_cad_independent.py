"""Đối chiếu độc lập mesh sim với mesh CAD xuất riêng (không dùng cad_check.json của script xuất mesh sim).
Nguồn so: thư mục STL do `cad/faibotic/assembly_full.py --pose ready --lift 200 --mesh DIR` xuất (mm, hệ thế giới, mỗi chi tiết một file + manifest.json).
Sim: nạp sim/m1.xml bằng MuJoCo, đặt tư thế ready (vai -30, khuỷu 70, cổ tay 10 độ, hai tay), lift 0,2 m, mj_forward, lấy pose từng geom mesh
(geom_xpos, geom_xmat; MuJoCo tự dời gốc mesh nên phải dùng pose geom, không dùng pose body).
So theo từng geom mesh sim = nhóm (body, vật liệu): gom các chi tiết CAD cùng nhóm (luật gán viết lại ở đây, không import export_sim_meshes.py),
so tâm bề mặt (trọng số diện tích tam giác, ít phụ thuộc mật độ lưới) và hộp bao (lệch lớn nhất của 6 mặt). Lò xo cân bằng không có mesh sim: bỏ.
Chạy: python sim/tools/check_cad_independent.py DIR [--xml sim/m1.xml] [--out sim/assets/cad_check_doc_lap.json]"""
import argparse, json, math, re, struct, sys
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[2]
POSE_DEG = {"shoulder": -30.0, "elbow": 70.0, "wrist_flex": 10.0}
LIFT = 0.20


def read_stl(p):
    b = Path(p).read_bytes()
    if len(b) >= 84 and 84 + 50 * struct.unpack("<I", b[80:84])[0] == len(b):
        n = struct.unpack("<I", b[80:84])[0]
        rec = np.frombuffer(b[84:], dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n)
        return rec["v"].astype(float)
    v = [list(map(float, ln.split()[1:4])) for ln in b.decode(errors="ignore").splitlines() if ln.strip().startswith("vertex")]
    return np.array(v, float).reshape(-1, 3, 3)


def surf_stats(tri):
    a = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    c = tri.mean(1)
    v = tri.reshape(-1, 3)
    return (c * a[:, None]).sum(0) / a.sum(), v.min(0), v.max(0), a.sum()


def group_of(name, mat):
    """Chi tiết CAD -> (body MJCF, vật liệu); None = không có trong sim."""
    if "spring" in name:
        return None
    side = {"L": "left", "R": "right"}.get(name[-1]) if name[-2:] in ("_L", "_R") else None
    if name.startswith(("M1-LF-block", "M1-LF-carriage_", "M1-LF-024_pan_hub")):
        body = "carriage"
    elif name.startswith(("M1-BS-", "M1-LF-")) or side is None:
        body = "base"
    elif re.match(r"(M1-SH-003|servo_|M1-SH-01[4-8])", name):
        body = side + "_shoulder"
    elif re.match(r"(M1-SH-004|M1-SH-012|M1-AR-02[234]|M1-HW-008)", name):
        body = side + "_upper"
    elif re.match(r"(M1-AR-020|elbow_servo|M1-AR-021|M1-AR-026|wrist_flex_servo|OB_F_J5_)", name):
        body = side + "_fore"
    elif name.startswith(("OB_F_J56_", "M1-AR-025", "wrist_roll_servo")):
        body = side + "_wrist"
    elif name.startswith(("OB_F_J7_Gripper", "M1-CM-001_wrist_cam", "M1-AR-009", "gripper_servo")):
        body = side + "_hand"
    elif name.startswith("OB_F_J7_Jaw"):
        body = side + "_finger_a"
    else:
        raise SystemExit("chua gan: " + name)
    return body, mat


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("cad_dir"); ap.add_argument("--xml", default=str(ROOT / "sim" / "m1.xml"))
    ap.add_argument("--out", default=str(ROOT / "sim" / "assets" / "cad_check_doc_lap.json")); a = ap.parse_args()
    m = mujoco.MjModel.from_xml_path(a.xml); d = mujoco.MjData(m)
    d.qpos[m.joint("lift").qposadr[0]] = LIFT
    for s in ("left", "right"):
        for j, v in POSE_DEG.items():
            d.qpos[m.joint(f"{s}_{j}").qposadr[0]] = math.radians(v)
    mujoco.mj_forward(m, d)
    cad = {}
    man = json.loads((Path(a.cad_dir) / "manifest.json").read_text())
    for n, mat in man:
        g = group_of(n, mat)
        if g is None:
            continue
        cad.setdefault(f"{g[0]}__{g[1]}", []).append((n, read_stl(Path(a.cad_dir) / f"{n}.stl") * 1e-3))
    rows, worst_c, worst_b = [], 0.0, 0.0
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
        mi = m.geom_dataid[g]; va, vn, fa, fn = m.mesh_vertadr[mi], m.mesh_vertnum[mi], m.mesh_faceadr[mi], m.mesh_facenum[mi]
        v = d.geom_xpos[g] + m.mesh_vert[va:va + vn] @ d.geom_xmat[g].reshape(3, 3).T
        tri_sim = v[m.mesh_face[fa:fa + fn]]
        parts = cad.pop(name, None)
        if parts is None:
            rows.append(dict(geom=name, loi="khong co chi tiet CAD")); continue
        tri_cad = np.concatenate([t for _, t in parts])
        cs, lo_s, hi_s, _ = surf_stats(tri_sim); cc, lo_c, hi_c, _ = surf_stats(tri_cad)
        dc = float(np.linalg.norm(cs - cc)) * 1e3; db = float(np.abs(np.r_[lo_s - lo_c, hi_s - hi_c]).max()) * 1e3
        worst_c, worst_b = max(worst_c, dc), max(worst_b, db)
        rows.append(dict(geom=name, n_chi_tiet=len(parts), lech_tam_mm=round(dc, 3), lech_hop_bao_mm=round(db, 3)))
    res = dict(nguon_cad=str(a.cad_dir), xml=str(a.xml), tu_the=dict(q_deg=POSE_DEG, lift_m=LIFT), so_geom=len(rows),
               so_chi_tiet=sum(r.get("n_chi_tiet", 0) for r in rows), lech_tam_lon_nhat_mm=round(worst_c, 3), lech_hop_bao_lon_nhat_mm=round(worst_b, 3),
               cad_khong_co_trong_sim=sorted(cad), theo_geom=rows)
    Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1))
    print("geom", len(rows), "chi tiet", res["so_chi_tiet"], "lech tam max mm", res["lech_tam_lon_nhat_mm"], "hop bao max mm", res["lech_hop_bao_lon_nhat_mm"],
          "thieu", sorted(cad), [r for r in rows if "loi" in r])


if __name__ == "__main__":
    main()
