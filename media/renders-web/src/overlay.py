"""Vẽ thước kích thước lên ảnh trực giao cạnh (thuoc). python overlay.py <ảnh.png> <thư_mục_mesh_straight> <ra.png> -> in JSON số đo. Số lấy từ mesh CAD (STL mm)."""
import sys, struct, re, json, glob, os
from PIL import Image, ImageDraw, ImageFont
img, mesh, out = sys.argv[1:4]
def verts(f):
    d = open(f, "rb").read()
    if d[:5] == b"solid": return [tuple(map(float, m)) for m in re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", d)]
    n = struct.unpack("<I", d[80:84])[0]; return [struct.unpack("<3f", d[84 + i * 50 + 12 + 12 * k: 84 + i * 50 + 24 + 12 * k]) for i in range(n) for k in range(3)]
def bb(pat):
    v = [p for f in glob.glob(os.path.join(mesh, pat)) for p in verts(f)]; xs, ys, zs = zip(*v); return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)
col = bb("M1-LF-010_column.stl"); shell = bb("M1-BS-00[12]_base_*.stl"); arm = [bb("OB_F_J7_Gripper_*.stl"), bb("OB_F_J7_Jaw_*.stl")]
H = col[5]; D = shell[1] - shell[0]; reach = max(a[1] for a in arm); ztop_arm = max(a[5] for a in arm)
m = {"cao_tong_mm": round(H, 1), "duong_kinh_de_mm": round(D, 1), "tam_voi_tu_tam_de_mm": round(reach, 1), "z_kep_mm": [round(min(a[4] for a in arm), 1), round(ztop_arm, 1)]}
im = Image.open(img).convert("RGB"); d = ImageDraw.Draw(im); W, Hh = im.size
ppm = W / 2.55; cx, cz = 0.2, 0.60
px = lambda x: W / 2 + (x / 1000 - cx) * ppm; pz = lambda z: Hh / 2 - (z / 1000 - cz) * ppm
NAVY = (30, 58, 95); f1 = ImageFont.truetype(next(p for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf") if os.path.exists(p)), int(46 * W / 2560)); lw = max(2, int(4 * W / 2560)); t = int(14 * W / 2560)
def hline(x0, x1, z, label):
    d.line([(px(x0), pz(z)), (px(x1), pz(z))], fill=NAVY, width=lw)
    for x in (x0, x1): d.line([(px(x), pz(z) - t), (px(x), pz(z) + t)], fill=NAVY, width=lw)
    tw = d.textlength(label, font=f1); d.text(((px(x0) + px(x1)) / 2 - tw / 2, pz(z) - 2.4 * t - 20), label, fill=NAVY, font=f1)
def vline(x, z0, z1, label):
    d.line([(px(x), pz(z0)), (px(x), pz(z1))], fill=NAVY, width=lw)
    for z in (z0, z1): d.line([(px(x) - t, pz(z)), (px(x) + t, pz(z))], fill=NAVY, width=lw)
    d.text((px(x) - d.textlength(label, font=f1) - 2 * t, (pz(z0) + pz(z1)) / 2 - 20), label, fill=NAVY, font=f1)
fmt = lambda v: f"{v:,.0f} mm".replace(",", " ")
vline(-300, 0, H, fmt(H)); hline(-D / 2, D / 2, -45, "Ø " + fmt(D)); hline(0, reach, ztop_arm + 110, fmt(reach))
for x in (0, reach):   # đường dóng
    for k in range(0, 14): 
        z0 = ztop_arm + 110 - t * 0 - k * 40 - 0; 
    d.line([(px(x), pz(ztop_arm + 110)), (px(x), pz(ztop_arm + 5))], fill=NAVY, width=max(1, lw - 1))
im.save(out); print(json.dumps(m))
