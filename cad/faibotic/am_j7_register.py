# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""đăng ký kẹp gốc AM-ARM200 (STEP follower OB_F_J7_Gripper + OB_F_Camera, OB_F_J7_Jaw) vào khung URDF upstream.
STEP follower đặt kẹp J7 theo bố cục trưng bày (lệch khỏi chỗ lắp, xem assembly_full GRIP_UP), nên không dùng được phép STEP = URDF + t như các chi tiết khác.
Lấy mesh URDF left_Fixed_Jaw / left_Moving_Jaw (cad/upstream/AM-ARM/am-arm200/urdf/am-arm200-meshes, commit 5b03af1, Apache-2.0) đặt bằng FK tư thế 0,
khớp đám điểm STEP vào đó bằng ICP cứng (khởi đầu: tâm hộp bao). Kết quả: ma trận 4x4 (mm) hệ STEP -> khung left_Base URDF ở tư thế 0, sai số RMS.
Chạy (venv build123d): python am_j7_register.py -> ../export/AM_J7_registration.json"""
import json, os, struct, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import amarm as AM

MESH = os.path.join(AM.UPA, "urdf", "am-arm200-meshes")
OUT = os.path.join(HERE, "..", "export", "AM_J7_registration.json")
PARTS = {"OB_F_J7_Gripper": "left_Fixed_Jaw", "OB_F_Camera": "left_Fixed_Jaw", "OB_F_J7_Jaw": "left_Moving_Jaw"}   # mesh left_Fixed_Jaw = thân kẹp + giá camera


def read_stl(p):
    b = open(p, "rb").read(); n = struct.unpack("<I", b[80:84])[0]
    if 84 + 50 * n == len(b):
        rec = np.frombuffer(b[84:], dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n)
        return rec["v"].reshape(-1, 3).astype(float)
    return np.array([list(map(float, l.split()[1:4])) for l in b.decode(errors="ignore").splitlines() if l.strip().startswith("vertex")])


def step_pts(name, tol=0.1):
    from build123d import import_step
    s = import_step(os.path.join(AM.STP, name + ".stp")); v, _ = s.tessellate(tol, 0.2)
    return np.array([[p.X, p.Y, p.Z] for p in v])


def icp(src, dst, R=np.eye(3), t=np.zeros(3), it=80):
    from scipy.spatial import cKDTree
    tree = cKDTree(dst)
    for _ in range(it):
        p = src @ R.T + t; d, i = tree.query(p); q = dst[i]
        keep = d < max(np.percentile(d, 90), 0.5)
        a, b = p[keep], q[keep]; ca, cb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb)); D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        dR = Vt.T @ D @ U.T; R = dR @ R; t = dR @ t + cb - dR @ ca
    d, _ = tree.query(src @ R.T + t)
    return R, t, float(np.sqrt((d ** 2).mean())), float(np.percentile(d, 95))


def main():
    """Từng chi tiết khớp riêng vào mesh link của nó (mesh thân kẹp gồm cả giá camera: đám STEP là tập con). Khởi đầu: lưới tịnh tiến 12 mm quanh
    phép khớp tâm hộp bao theo x, mặt trước (y nhỏ nhất) và mặt trên/dưới; ICP thô 3.000 điểm, rồi tinh 20.000 điểm từ khởi đầu tốt nhất."""
    T0 = AM.fk({}); out = dict(ghi_chu="M (mm): điểm STEP follower -> khung left_Base URDF (mm) ở tư thế 0 của URDF; mesh URDF đặt bằng amarm.fk({})", chi_tiet={})
    rng = np.random.default_rng(0)
    for n, link in PARTS.items():
        V = read_stl(os.path.join(MESH, link + ".STL")) * 1000.0
        V = V @ T0[link][:3, :3].T + T0[link][:3, 3] * 1000.0
        S = step_pts(n); src = S[rng.choice(len(S), min(len(S), 20000), replace=False)]; coarse = src[:3000]
        Vs = V[rng.choice(len(V), min(len(V), 20000), replace=False)]
        base = []
        for zmode in ("min", "max", "mid"):
            tz = {"min": V.min(0)[2] - S.min(0)[2], "max": V.max(0)[2] - S.max(0)[2], "mid": (V.min(0)[2] + V.max(0)[2] - S.min(0)[2] - S.max(0)[2]) / 2}[zmode]
            for ymode in ("min", "max"):
                ty = V.min(0)[1] - S.min(0)[1] if ymode == "min" else V.max(0)[1] - S.max(0)[1]
                base.append(np.array([(V.min(0)[0] + V.max(0)[0] - S.min(0)[0] - S.max(0)[0]) / 2, ty, tz]))
        best = None
        for b in base:
            for dx in (-12, 0, 12):
                for dy in (-12, 0, 12):
                    for dz in (-12, 0, 12):
                        R, t, rms, _ = icp(coarse, Vs, np.eye(3), b + (dx, dy, dz), it=15)
                        if best is None or rms < best[2]: best = (R, t, rms)
        R, t, rms, p95 = icp(src, V, best[0], best[1], it=60)
        M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t
        ang = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
        out["chi_tiet"][n] = dict(link=link, M=M.round(6).tolist(), rms_mm=round(rms, 3), p95_mm=round(p95, 3), quay_deg=round(ang, 3))
        print(n, link, "rms", round(rms, 3), "p95", round(p95, 3), "quay", round(ang, 2), "t", t.round(2), flush=True)
    json.dump(out, open(OUT, "w"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
