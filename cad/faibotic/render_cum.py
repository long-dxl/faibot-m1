# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Ảnh tô màu theo cụm cho tờ HARDWARE: trạm hai tay, trụ nâng, đế (và nơi đặt điện).

Dùng lại bộ vẽ của render_buoc_lap.py (thuật toán người thợ sơn trên PIL, cùng góc nhìn, cùng đọc mesh).
Khác render_buoc_lap: nền trong suốt (dùng được cho bản sáng và tối), khung chữ nhật cố định cho mọi ảnh.

  python assembly_pl.py --mesh MESH_DIR --lift 0 --pose ready      (cần build123d)
  python render_cum.py MESH_DIR OUT_DIR
  -> OUT_DIR/cum-tong-quan.png (ba cụm ba màu), cum-tram.png, cum-truc-nang.png, cum-de.png, cum-dien.png
"""
import json, os, re, sys
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_buoc_lap import chieu  # noqa: E402
from render_pl import read_stl  # noqa: E402

CUM = [
    ("tram", r"M1-TR-|M1-SH-|M1-AR-|M1-CM-|M1-EL-|M1-SV-001_gripper"),
    ("truc-nang", r"M1-LF-"),
    ("de", r"M1-BS-|M1-ST-|M1-SV-001_wheel"),
]
# nơi đặt điện: hộp điện trạm, hộp nút, khoang điện đế, công tắc và khoá hãm
DIEN = r"M1-BS-038|M1-TR-01[23]|M1-TR-009|M1-LF-041_switch|M1-LF-050|M1-LF-061"
# bảng màu cho cả nền sáng lẫn nền tối, phân biệt được cả khi mù màu đỏ/lục: xanh dương, cam, xanh ngọc
MAU = {"tram": (31, 111, 235), "truc-nang": (240, 140, 22), "de": (16, 150, 135), "dien": (31, 111, 235)}
XAM = (176, 178, 182)
XANH = (31, 111, 235)                # cụm đang xem trên từng tờ; ba màu riêng chỉ dùng ở ảnh tổng quan
W, H, SS = 760, 920, 2


def nhom(ten):
    for k, bt in CUM:
        if re.search(bt, ten):
            return k
    return None


def tai(d):
    man = json.load(open(os.path.join(d, "manifest.json")))
    ds = []
    for n, _ in man:
        f = os.path.join(d, n + ".stl")
        if not os.path.exists(f):
            continue
        g = nhom(n)
        if g is None:
            print("không thuộc cụm:", n)
            continue
        t = read_stl(f)
        t = t[np.abs(t).max(axis=(1, 2)) < 2000]      # bỏ thực thể lạc ở gốc toạ độ
        if len(t):
            ds.append((n, g, t))
    return ds


def ve(tris, cols, eye, right, up, khung):
    T = np.concatenate(tris)
    C = np.concatenate([np.repeat([c], len(t), 0) for t, c in zip(tris, cols)]).astype(float) / 255
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    light = eye + np.array([0.3, 0.2, 0.6]); light /= np.linalg.norm(light)
    sh = np.clip(np.abs(nrm @ light), 0, 1) * 0.60 + 0.40
    col = (np.clip(C * sh[:, None], 0, 1) * 255).astype(np.uint8)
    order = np.argsort(T.mean(axis=1) @ eye)
    x0, y1, s = khung
    px = (T @ right - x0) * s; py = (y1 - T @ up) * s
    im = Image.new("RGBA", (W * SS, H * SS), (0, 0, 0, 0)); dr = ImageDraw.Draw(im)
    for k in order:
        c = tuple(int(v) for v in col[k]) + (255,)
        p = [(px[k, 0], py[k, 0]), (px[k, 1], py[k, 1]), (px[k, 2], py[k, 2])]
        dr.polygon(p, fill=c, outline=c)
    return im.resize((W, H), Image.LANCZOS)


def luu(im, path):
    """PNG bảng màu (có kênh trong suốt), nhẹ cho README."""
    im.quantize(colors=96, method=Image.Quantize.FASTOCTREE, dither=Image.Dither.NONE).save(path, optimize=True)


def main(src, dst):
    os.makedirs(dst, exist_ok=True)
    ds = tai(src)
    eye, right, up = chieu()
    tat = np.concatenate([t for _, _, t in ds]).reshape(-1, 3)
    u, v = tat @ right, tat @ up
    ru, rv = u.max() - u.min(), v.max() - v.min()
    s = min(W * SS / (ru * 1.10), H * SS / (rv * 1.10))
    x0 = (u.min() + u.max()) / 2 - W * SS / s / 2
    y1 = (v.min() + v.max()) / 2 + H * SS / s / 2
    khung = (x0, y1, s)
    tris = [t for _, _, t in ds]
    cum = [g for _, g, _ in ds]
    ten = [n for n, _, _ in ds]
    out = {}
    out["cum-tong-quan"] = [MAU[g] for g in cum]
    for k in ("tram", "truc-nang", "de"):
        out["cum-" + k] = [XANH if g == k else XAM for g in cum]
    out["cum-dien"] = [MAU["dien"] if re.search(DIEN, n) else XAM for n in ten]
    # vẽ phần xám trước, phần màu sau (thuật toán sắp theo độ sâu chung nên chỉ cần một lượt)
    for name, cols in out.items():
        im = ve(tris, cols, eye, right, up, khung)
        p = os.path.join(dst, name + ".png")
        luu(im, p)
        print(name, os.path.getsize(p) // 1024, "KB", flush=True)
    json.dump({"so_chi_tiet": {k: sum(1 for g in cum if g == k) for k in ("tram", "truc-nang", "de")}, "dien": [n for n in ten if re.search(DIEN, n)]},
              open(os.path.join(dst, "cum.json"), "w"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
