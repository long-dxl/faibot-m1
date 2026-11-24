# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Ảnh từng bước lắp P-L: mỗi ảnh cộng thêm một nhóm chi tiết, nhóm mới màu xanh, phần đã lắp màu xám, cùng một góc nhìn đẳng cự.

Hai bước:
  1. Xuất mesh từ assembly_pl.py (cần build123d):
       python assembly_pl.py --mesh MESH_DIR --lift 0 --pose ready
  2. Vẽ (chỉ cần numpy + Pillow; dùng lại read_stl của render_pl.py):
       python render_buoc_lap.py MESH_DIR OUT_DIR
     -> OUT_DIR/buoc-01.png ... buoc-10.png (1200 x 1200, PNG bảng màu, <= 300 KB) và OUT_DIR/buoc-lap.json (nhóm, số chi tiết).

Nhóm theo tên chi tiết trong manifest.json của assembly_pl.py (mã M1-xx). Chi tiết không khớp nhóm nào bị liệt kê và không vẽ.
Vẽ bằng thuật toán người thợ sơn (sắp theo độ sâu) trên PIL, siêu lấy mẫu 2x; không cần GPU.
"""
import json, os, re, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_pl import read_stl  # noqa: E402

# (nhãn, biểu thức tên, bên). Thứ tự = thứ tự lắp. Mỗi chi tiết thuộc nhóm đầu tiên khớp.
BUOC = [
    ("Đế và đối trọng", r"M1-BS-03[12]|M1-BS-039|M1-BS-037|M1-BS-038"),
    ("Bánh, servo bánh, giá servo", r"M1-BS-035|M1-BS-036|M1-SV-001_wheel"),
    ("Trụ nâng", r"M1-LF-05[1-5]_"),
    ("Xe nâng: khung T, bánh răng, chốt hãm, công tắc, servo", r"M1-LF-|M1-BS-006(?!_2040_do_khay)"),
    ("Trạm hai tay: ray, tai, gá, khoá, giắc, hộp, cột", r"M1-TR-|M1-TB-|M1-EL-"),
    ("Tay trái", r"M1-(SH|AR)-\d+.*_L$"),
    ("Tay phải", r"M1-(SH|AR)-\d+.*_R$"),
    ("Kẹp (thân J7, hàm, servo kẹp)", r"M1-AR-00[78]|M1-SV-001_gripper"),
    ("Camera (giá cổ tay, camera cổ tay, camera đầu)", r"M1-AR-027|M1-CM-"),
    ("Khay và cột đỡ khay", r"M1-ST-|M1-BS-006_2040_do_khay"),
]
# Các nhóm gom theo tên trước khi áp BUOC: kẹp và camera phải bắt trước nhóm tay (tên cùng đuôi _L/_R).
ƯU_TIÊN = (7, 8)

EL, AZ = 20.0, -50.0           # cùng góc "34" của render_pl.py
N = 1200                       # cạnh ảnh, px
SS = 2                         # siêu lấy mẫu
XAM = np.array([0.74, 0.75, 0.77]); XANH = np.array([0.10, 0.42, 0.92]); NEN = (255, 255, 255)
MAX_KB = 300


def nhom_cua(ten):
    for i in ƯU_TIÊN:
        if re.search(BUOC[i][1], ten):
            return i
    for i, (_, bt) in enumerate(BUOC):
        if i in ƯU_TIÊN:
            continue
        if re.search(bt, ten):
            return i
    return None


def tai(d):
    man = json.load(open(os.path.join(d, "manifest.json")))
    tris = [[] for _ in BUOC]; le = []
    for n, _ in man:
        f = os.path.join(d, n + ".stl")
        if not os.path.exists(f):
            continue
        g = nhom_cua(n)
        if g is None:
            le.append(n); continue
        t = read_stl(f)
        t = t[np.abs(t).max(axis=(1, 2)) < 2000]      # bỏ thực thể lạc ở gốc toạ độ (STEP Post1)
        if len(t):
            tris[g].append(t)
    return [np.concatenate(t) if t else np.zeros((0, 3, 3)) for t in tris], le


def font(size):
    cands = ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]
    try:
        import matplotlib
        cands.append(os.path.join(matplotlib.get_data_path(), "fonts/ttf/DejaVuSans.ttf"))
    except Exception:
        pass
    for c in cands:
        if os.path.exists(c):
            return ImageFont.truetype(c, size)
    return ImageFont.load_default()


def chieu():
    e, a = np.radians(EL), np.radians(AZ)
    eye = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    right = np.array([-np.sin(a), np.cos(a), 0.0])
    up =np.array([-np.sin(e) * np.cos(a), -np.sin(e) * np.sin(a), np.cos(e)])
    return eye, right, up


def ve(trs, mau, khung, eye, right, up):
    """trs: danh sách (mảng tam giác, màu gốc). Trả ảnh PIL N x N."""
    T = np.concatenate([t for t, _ in trs]); C = np.concatenate([np.repeat([c], len(t), 0) for t, c in trs])
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    light = eye + np.array([0.3, 0.2, 0.6]); light /= np.linalg.norm(light)
    sh = np.clip(np.abs(nrm @ light), 0, 1) * 0.65 + 0.35
    col = (np.clip(C * sh[:, None], 0, 1) * 255).astype(np.uint8)
    depth = (T.mean(axis=1) @ eye)
    order = np.argsort(depth)
    (x0, x1, y0, y1) = khung; s = N * SS / max(x1 - x0, y1 - y0)
    px = (T @ right - x0) * s; py = (y1 - T @ up) * s
    im = Image.new("RGB", (N * SS, N * SS), NEN); dr = ImageDraw.Draw(im)
    for k in order:
        dr.polygon([(px[k, 0], py[k, 0]), (px[k, 1], py[k, 1]), (px[k, 2], py[k, 2])], fill=tuple(int(v) for v in col[k]))
    return im.resize((N, N), Image.LANCZOS)


def luu(im, path):
    for nc in (96, 64, 48, 32, 24):
        q = im.quantize(colors=nc, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
        q.save(path, optimize=True)
        if os.path.getsize(path) <= MAX_KB * 1024:
            return nc
    return nc


def main(src, dst):
    os.makedirs(dst, exist_ok=True)
    tris, le = tai(src)
    eye, right, up = chieu()
    tat = np.concatenate([t for t in tris if len(t)]).reshape(-1, 3)
    u = tat @ right; v = tat @ up
    cx, cy = (u.min() + u.max()) / 2, (v.min() + v.max()) / 2
    r = max(u.max() - u.min(), v.max() - v.min()) / 2 * 1.08
    khung = (cx - r, cx + r, cy - r, cy + r)
    f1, f2 = font(34), font(24)
    out = []
    for i, (nhan, _) in enumerate(BUOC):
        trs = [(tris[j], XAM) for j in range(i) if len(tris[j])]
        if len(tris[i]):
            trs.append((tris[i], XANH))
        im = ve(trs, None, khung, eye, right, up)
        d = ImageDraw.Draw(im)
        d.text((36, 28), f"Bước {i + 1:02d}", fill=(20, 60, 140), font=f1)
        d.text((36, 76), nhan, fill=(60, 60, 60), font=f2)
        p = os.path.join(dst, f"buoc-{i + 1:02d}.png")
        nc = luu(im, p)
        out.append(dict(buoc=i + 1, nhan=nhan, so_tam_giac=int(len(tris[i])), mau=nc, kb=round(os.path.getsize(p) / 1024)))
        print(out[-1], flush=True)
    json.dump(dict(goc=dict(el=EL, az=AZ), buoc=out, khong_khop_nhom=le), open(os.path.join(dst, "buoc-lap.json"), "w"), ensure_ascii=False, indent=1)
    if le:
        print("KHÔNG KHỚP NHÓM:", le, flush=True)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
