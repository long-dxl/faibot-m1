# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Render PNG từ STL bằng trimesh + matplotlib (không GPU)."""
import sys, os, numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
src, dst = sys.argv[1], sys.argv[2]
os.makedirs(dst, exist_ok=True)
def draw(stl, png, views=((25, -60), (25, 30))):
    m = trimesh.load(stl); fig = plt.figure(figsize=(5 * len(views), 5))
    light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
    tri = m.triangles; shade = np.clip(m.face_normals @ light, 0, 1) * 0.6 + 0.35
    cols = np.stack([shade * 0.35, shade * 0.6, shade * 0.9, np.ones_like(shade)], 1)
    for i, (el, az) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(tri, facecolors=cols, linewidths=0))
        b = m.bounds; c = b.mean(0); r = (b[1] - b[0]).max() / 2
        ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(el, az); ax.set_axis_off()
    fig.suptitle(os.path.basename(stl) + "  [mm]", fontsize=9)
    fig.savefig(png, dpi=90, bbox_inches="tight"); plt.close(fig)
for n in ("shoulder_assembly", "power_assembly", "cradle", "link", "power_deck"):
    draw(f"{src}/{n}.stl", f"{dst}/{n}.png")
