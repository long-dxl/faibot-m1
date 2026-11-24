# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Hình từng bước cho M1_assembly_A5_v0.5.pdf: nét kỹ thuật xám, chi tiết mới màu cam, hình nổ có đường nét đứt về vị trí lắp.

Một script, hai chế độ (cùng file):
  1. Xuất mesh (cần build123d, thường chạy trên máy dựng CAD):
       python assembly_pl.py --mesh MESH_DIR --lift 0 --pose ready
  2. Vẽ (cần Blender >= 3.6 có Freestyle, và Pillow + numpy + PyYAML ở python hệ thống):
       python render_pdf_buoc.py MESH_DIR OUT_DIR [--yaml trinh_tu_lap.yaml] [--blender blender] [--chi bia,the,buoc]
     Script tự gọi `blender -b -P render_pdf_buoc.py` để vẽ nét (Freestyle: silhouette, border, crease 35 độ) ra PNG trong suốt,
     rồi ghép nền trắng và nét đứt hình nổ bằng Pillow.

Đầu ra trong OUT_DIR:
  buoc-<id>.png   hình từng bước (id trong trinh_tu_lap.yaml)
  the-<ma>.png    hình thẻ chi tiết (một chi tiết, màu cam)
  bia.png         hình bìa (cả robot, nét đen, nền trắng)
  buoc.json       toạ độ chiếu, danh sách chi tiết không khớp nhóm nào

Trình tự lắp, nhóm chi tiết và độ lệch nổ nằm ở trinh_tu_lap.yaml; không vẽ tay. Mesh lấy từ assembly_pl.py hiện tại: chi tiết chưa có
trong assembly_pl.py (ví dụ thân servo của tay, nếu nhánh khác chưa thêm) sẽ chưa hiện trong hình. Chạy lại script khi assembly_pl.py đổi.
Chạy nặng: `flock /tmp/heavy.lock nice -n 10 python render_pdf_buoc.py ...`.
"""
import json, math, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
CAM = (0.89, 0.25, 0.013)      # cam #F28A1E (giá trị tuyến tính, Blender đổi sang sRGB khi vẽ)
XAM = (0.66, 0.67, 0.68)       # xám: đã lắp
RES = (900, 1400)              # px (rộng, cao), ảnh bước: dọc, vừa vùng hình của trang A5
RES_THE = 420                  # px, thẻ chi tiết


# ============================================================== phần chạy trong Blender
def trong_blender():
    import bpy, mathutils
    argv = sys.argv[sys.argv.index("--") + 1:]
    mesh_dir, out_dir, yaml_path, chi = argv[0], argv[1], argv[2], argv[3].split(",")
    cfg = json.load(open(yaml_path, encoding="utf-8"))      # main() đã đổi yaml -> json (Python của Blender không có PyYAML)
    man = [n for n, _ in json.load(open(os.path.join(mesh_dir, "manifest.json")))]

    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    objs = {}
    for n in man:
        f = os.path.join(mesh_dir, n + ".stl")
        if not os.path.exists(f):
            continue
        bpy.ops.wm.stl_import(filepath=f)
        o = bpy.context.selected_objects[0]
        o.name = n
        o.scale = (0.001,) * 3           # mm -> m
        if max(abs(v) for c in o.bound_box for v in c) > 5000:   # thực thể lạc ở gốc toạ độ (STEP Post1): bỏ
            bpy.data.objects.remove(o); continue
        objs[n] = o
    bpy.context.view_layer.update()

    sc.render.engine = "BLENDER_WORKBENCH"
    sc.render.film_transparent = True
    sc.view_settings.view_transform = "Standard"      # không tông màu điện ảnh: giữ cam và xám đúng màu
    sc.render.image_settings.file_format = "PNG"; sc.render.image_settings.color_mode = "RGBA"
    sh = sc.display.shading
    sh.light = "FLAT"; sh.color_type = "OBJECT"; sh.show_object_outline = False
    sc.render.use_freestyle = True
    sc.render.line_thickness_mode = "ABSOLUTE"; sc.render.line_thickness = 1.7
    vl = bpy.context.view_layer; vl.use_freestyle = True
    fs = vl.freestyle_settings
    while len(fs.linesets):
        fs.linesets.remove(fs.linesets[0])
    ls = fs.linesets.new("nét")
    ls.select_silhouette = True; ls.select_border = True; ls.select_crease = True
    fs.crease_angle = math.radians(35)
    ls.linestyle = bpy.data.linestyles.new("den"); ls.linestyle.color = (0.05, 0.05, 0.05); ls.linestyle.thickness = 1.5

    camd = bpy.data.cameras.new("c"); camd.type = "ORTHO"
    cam = bpy.data.objects.new("c", camd); bpy.context.collection.objects.link(cam); sc.camera = cam

    def dat_cam(el, az, pts, res, pad=1.05):
        """Đặt camera đẳng cự nhìn từ (el, az) và vừa khít các điểm pts (m, hệ thế giới)."""
        e, a = math.radians(el), math.radians(az)
        d = mathutils.Vector((math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)))
        c0 = sum(pts, mathutils.Vector()) / len(pts)
        cam.location = c0 + d * 20
        cam.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
        bpy.context.view_layer.update()
        inv = cam.matrix_world.inverted()
        xy = [(inv @ p) for p in pts]
        x0, x1 = min(p.x for p in xy), max(p.x for p in xy); y0, y1 = min(p.y for p in xy), max(p.y for p in xy)
        rw, rh = (res, res) if isinstance(res, int) else res
        big = max(rw, rh)          # ortho_scale và shift tính theo cạnh dài của ảnh
        s = max((x1 - x0) * big / rw, (y1 - y0) * big / rh) * pad
        camd.ortho_scale = s
        camd.shift_x = ((x0 + x1) / 2) / s; camd.shift_y = ((y0 + y1) / 2) / s
        sc.render.resolution_x, sc.render.resolution_y = rw, rh
        bpy.context.view_layer.update()

    def corners(names):
        pts = []
        for n in names:
            o = objs[n]
            pts += [o.matrix_world @ mathutils.Vector(c) for c in o.bound_box]
        return pts

    def proj(p):
        from bpy_extras.object_utils import world_to_camera_view
        v = world_to_camera_view(sc, cam, p)
        return [v.x * sc.render.resolution_x, (1 - v.y) * sc.render.resolution_y]

    def mau(o, c):
        o.color = (c[0], c[1], c[2], 1.0)

    def ve(path):
        sc.render.filepath = path
        bpy.ops.render.render(write_still=True)

    # ---- nhóm theo trình tự
    gan = {}                # tên -> (chỉ số bước, chỉ số nhóm)
    for bi, b in enumerate(cfg["buoc"]):
        for gi, g in enumerate(b["nhom"]):
            for n in objs:
                if "re" in g and n not in gan and re.search(g["re"], n):
                    gan[n] = (bi, gi)
    le = [n for n in objs if n not in gan]
    info = {"khong_khop_nhom": le, "buoc": {}}
    goc0 = cfg.get("goc_mac_dinh", [20, -50])

    if "buoc" in chi:
        for bi, b in enumerate(cfg["buoc"]):
            ids = [x["id"] for x in cfg["buoc"]]
            hien_cu = [n for n, (i, _) in gan.items() if i < bi]
            moi = [[n for n, (i, g) in gan.items() if i == bi and g == gi] for gi in range(len(b["nhom"]))]
            for gi, g in enumerate(b["nhom"]):      # nhóm "cua_buoc": nhấc lại chi tiết đã lắp ở các bước đó
                if "cua_buoc" in g:
                    ib = {ids.index(x) for x in g["cua_buoc"]}
                    moi[gi] = [n for n, (i, _) in gan.items() if i in ib]
                    hien_cu = [n for n in hien_cu if n not in moi[gi]]
            for o in objs.values():
                o.hide_render = True
                o.location = (0, 0, 0)
            for n in hien_cu:
                objs[n].hide_render = False; mau(objs[n], XAM)
            pt_goc = []
            nhom_info = []
            for gi, (g, ten_nhom) in enumerate(zip(b["nhom"], moi)):
                if not ten_nhom:
                    continue
                c_lap = corners(ten_nhom)
                if "radial" in g:
                    # lệch theo hướng từ trục robot (x, y) = (0, 0) ra tới tâm nhóm
                    cc = sum(c_lap, mathutils.Vector()) / len(c_lap)
                    h = mathutils.Vector((cc.x, cc.y, 0)); h = h.normalized() if h.length > 1e-6 else mathutils.Vector((1, 0, 0))
                    v = h * g["radial"] / 1000
                else:
                    v = mathutils.Vector(g["lech"]) / 1000
                for n in ten_nhom:
                    objs[n].hide_render = False; objs[n].location = v; mau(objs[n], CAM)
                pt_goc += c_lap + [c + v for c in c_lap]
                nhom_info.append((ten_nhom, v, c_lap))
            if not pt_goc:
                continue
            el, az = b.get("goc", goc0)
            pts = pt_goc + corners(hien_cu) if hien_cu else pt_goc
            dat_cam(el, az, pts, RES)
            ve(os.path.join(out_dir, f"_raw-{b['id']}.png"))
            ni = []
            for ten_nhom, v, c_lap in nhom_info:
                ni.append({"lap": [proj(c) for c in c_lap], "no": [proj(c + v) for c in c_lap]})
            info["buoc"][b["id"]] = {"nhom": ni, "so_ct": len(pt_goc) // 16}
            print("buoc", b["id"], flush=True)

    # ---- thẻ chi tiết
    if "the" in chi:
        done = set()
        for b in cfg["buoc"]:
            for ct in b.get("chi_tiet", []):
                key = ct["ma"] + "|" + ct.get("hinh", "")
                if key in done:
                    continue
                done.add(key)
                ten = [n for n in objs if re.search(ct.get("hinh", "^" + ct["ma"] + "_"), n)]
                if not ten:
                    continue
                for o in objs.values():
                    o.hide_render = True; o.location = (0, 0, 0)
                for n in ten:
                    objs[n].hide_render = False; mau(objs[n], CAM)
                dat_cam(24, -50, corners(ten), RES_THE, pad=1.12)
                sc.render.line_thickness = 1.3
                ve(os.path.join(out_dir, f"_the-{ct['ma']}.png"))
                sc.render.line_thickness = 1.5
                # thẻ có nhiều biến thể hinh cho cùng mã: ghi thêm dưới tên mã đầu tiên là đủ
        print("the", len(done), flush=True)

    # ---- bìa
    if "bia" in chi:
        for o in objs.values():
            o.hide_render = False; o.location = (0, 0, 0); mau(o, (0.97, 0.97, 0.97))
        dat_cam(18, -50, corners(list(objs)), 1600, pad=1.06)
        sc.render.line_thickness = 2.0
        ve(os.path.join(out_dir, "_raw-bia.png"))
        print("bia", flush=True)

    json.dump(info, open(os.path.join(out_dir, "buoc.json"), "w"), ensure_ascii=False)


# ============================================================== phần chạy ở python hệ thống (ghép ảnh)
def hull(pts):
    pts = sorted(set(map(tuple, pts)))
    def cr(o, a, b): return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cr(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def net_dut(dr, a, b, dash=11, gap=8, w=2, fill=(70, 70, 70)):
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    if L < 1: return
    ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
    t = 0.0
    while t < L:
        e = min(t + dash, L)
        dr.line([(a[0] + ux * t, a[1] + uy * t), (a[0] + ux * e, a[1] + uy * e)], fill=fill, width=w)
        t += dash + gap


def mui_ten(dr, a, b, size=16, fill=(70, 70, 70)):
    """Đầu mũi tên tại b, hướng từ a tới b."""
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    if L < 1: return
    ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
    px, py = -uy, ux
    dr.polygon([b, (b[0] - ux * size + px * size * 0.4, b[1] - uy * size + py * size * 0.4),
                (b[0] - ux * size - px * size * 0.4, b[1] - uy * size - py * size * 0.4)], fill=fill)


def ghep(out_dir, chi):
    from PIL import Image, ImageDraw
    info = json.load(open(os.path.join(out_dir, "buoc.json")))
    for bid, bi in info["buoc"].items():
        raw = Image.open(os.path.join(out_dir, f"_raw-{bid}.png")).convert("RGBA")
        nen = Image.new("RGBA", raw.size, (255, 255, 255, 255))
        # nét đứt nằm dưới hình: chỉ lộ ở khoảng trống giữa vị trí nổ và vị trí lắp
        dr = ImageDraw.Draw(nen)
        for g in bi["nhom"]:
            lap, no = g["lap"], g["no"]
            hl = hull(no)
            for i, p in enumerate(no):
                if tuple(p) in hl or p in [list(h) for h in hl]:
                    net_dut(dr, p, lap[i])
        nen.alpha_composite(raw)
        # mũi tên: ở đầu nét đứt phía vị trí lắp, vẽ trên cùng nhưng nhỏ
        dr = ImageDraw.Draw(nen)
        for g in bi["nhom"]:
            lap, no = g["lap"], g["no"]
            hl = [list(h) for h in hull(no)]
            xs = [i for i, p in enumerate(no) if p in hl]
            if xs:
                i = xs[len(xs) // 2]
                a, b = no[i], lap[i]
                L = math.hypot(b[0] - a[0], b[1] - a[1])
                if L > 60:
                    ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
                    mui_ten(dr, a, (a[0] + ux * L * 0.55, a[1] + uy * L * 0.55), size=22)
        nen = nen.convert("RGB")
        nen.save(os.path.join(out_dir, f"buoc-{bid}.png"), optimize=True)
        os.remove(os.path.join(out_dir, f"_raw-{bid}.png"))
    for f in sorted(os.listdir(out_dir)):
        if f.startswith("_the-") or f == "_raw-bia.png":
            im = Image.open(os.path.join(out_dir, f)).convert("RGBA")
            nen = Image.new("RGBA", im.size, (255, 255, 255, 255)); nen.alpha_composite(im)
            bb = Image.eval(im.split()[3], lambda v: 255 if v > 8 else 0).getbbox()
            nen = nen.convert("RGB")
            if bb:
                m = 8
                nen = nen.crop((max(bb[0] - m, 0), max(bb[1] - m, 0), min(bb[2] + m, im.width), min(bb[3] + m, im.height)))
            nen.save(os.path.join(out_dir, f[1:].replace("raw-", "")), optimize=True)
            os.remove(os.path.join(out_dir, f))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh"); ap.add_argument("out")
    ap.add_argument("--yaml", default=os.path.join(HERE, "trinh_tu_lap.yaml"))
    ap.add_argument("--blender", default="blender")
    ap.add_argument("--chi", default="bia,the,buoc")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    import yaml
    cfgj = os.path.join(a.out, "_cfg.json")
    json.dump(yaml.safe_load(open(a.yaml, encoding="utf-8")), open(cfgj, "w"), ensure_ascii=False)
    a.yaml = cfgj
    cmd = [a.blender, "-b", "-P", os.path.abspath(__file__), "--", os.path.abspath(a.mesh), os.path.abspath(a.out), os.path.abspath(a.yaml), a.chi]
    r = subprocess.run(cmd, stdout=open(os.path.join(a.out, "blender.log"), "w"), stderr=subprocess.STDOUT)
    if r.returncode != 0 or "Traceback" in open(os.path.join(a.out, "blender.log")).read():
        sys.exit("Blender lỗi, xem " + os.path.join(a.out, "blender.log"))
    ghep(a.out, a.chi)
    info = json.load(open(os.path.join(a.out, "buoc.json")))
    if info["khong_khop_nhom"]:
        print("Chi tiết không thuộc bước nào (không vẽ):", len(info["khong_khop_nhom"]), info["khong_khop_nhom"][:12])


if __name__ == "__main__":
    try:
        import bpy  # noqa: F401
        TRONG = True
    except ImportError:
        TRONG = False
    trong_blender() if TRONG else main()
