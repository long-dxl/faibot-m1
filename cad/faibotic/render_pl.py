# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Ảnh cổng hình P-L: 3 góc (trước, bên, 3/4) cùng thước, chiếu song song, từ thư mục mesh của assembly_pl.py --mesh. matplotlib, không GPU.
 Chạy: python render_pl.py MESH_DIR OUT_DIR [nhãn]  -> OUT_DIR/{front,side,34}_<nhãn>.png"""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

COL = {"ivory": (0.93, 0.91, 0.85), "ivory_hi": (0.98, 0.96, 0.90), "navy": (0.16, 0.23, 0.42), "alu": (0.72, 0.74, 0.77), "steel": (0.45, 0.47, 0.50),
       "servo_body": (0.10, 0.10, 0.10), "servo": (0.15, 0.15, 0.16), "rubber": (0.12, 0.12, 0.12), "hub": (0.60, 0.62, 0.66), "pad": (0.85, 0.45, 0.10)}
VIEWS = {"front": (0, 0), "side": (0, -90), "34": (20, -50)}


def read_stl(f):
    """STL nhị phân -> (n, 3, 3) mm (không cần trimesh)."""
    b = open(f, "rb").read()
    if b[:5] == b"solid" and b"facet" in b[:400]:
        v = [list(map(float, l.split()[1:4])) for l in b.decode().splitlines() if l.strip().startswith("vertex")]
        return np.array(v).reshape(-1, 3, 3)
    n = int(np.frombuffer(b[80:84], "<u4")[0])
    d = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return d["v"].astype(float)


def load(d):
    man = json.load(open(os.path.join(d, "manifest.json"))); tris = []; cols = []
    for n, m in man:
        f = os.path.join(d, n + ".stl")
        if not os.path.exists(f): continue
        tr = read_stl(f)
        tr = tr[np.abs(tr).max(axis=(1, 2)) < 2000]          # bỏ thực thể lạc (STEP Post1 có điểm ở gốc toạ độ bố trí)
        tris.append(tr); cols.append(np.repeat([COL.get(m, (0.6, 0.6, 0.6))], len(tr), 0))
    T = np.concatenate(tris); C = np.concatenate(cols)
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    return T, C, nrm


def draw(T, C, nrm, view, png, title, lim):
    el, az = view
    e, a = np.radians(el), np.radians(az)
    eye = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    light = eye + np.array([0.3, 0.2, 0.6]); light /= np.linalg.norm(light)
    sh = np.clip(np.abs(nrm @ light), 0, 1) * 0.65 + 0.35
    fc = np.clip(C * sh[:, None], 0, 1)
    fig = plt.figure(figsize=(6, 7.5), dpi=110); ax = fig.add_subplot(111, projection="3d", proj_type="ortho")
    ax.add_collection3d(Poly3DCollection(T, facecolors=fc, linewidths=0, zsort="average"))
    (x0, x1), (y0, y1), (z0, z1) = lim
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_zlim(z0, z1); ax.set_box_aspect((x1 - x0, y1 - y0, z1 - z0))
    ax.view_init(el, az); ax.set_axis_off(); fig.patch.set_facecolor("white")
    ax.set_title(title, fontsize=8)
    fig.savefig(png, bbox_inches="tight", facecolor="white"); plt.close(fig)


if __name__ == "__main__":
    src, dst = sys.argv[1], sys.argv[2]; tag = sys.argv[3] if len(sys.argv) > 3 else "pl"
    os.makedirs(dst, exist_ok=True)
    T, C, nrm = load(src)
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 1.02
    lim = ((c[0] - r, c[0] + r), (c[1] - r, c[1] + r), (lo[2] - 5, lo[2] + 2 * r))
    for k, v in VIEWS.items():
        draw(T, C, nrm, v, os.path.join(dst, f"{k}_{tag}.png"), f"Faibot M1 v0.5 P-L {tag}  cao {hi[2]:.0f} mm, rộng {hi[1]-lo[1]:.0f} mm", lim)
    print("ok", len(T), [round(v) for v in lo], [round(v) for v in hi])
