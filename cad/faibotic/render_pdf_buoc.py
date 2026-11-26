# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Hình từng bước cho M1_assembly_A5_v0.5.pdf: nét kỹ thuật xám, chi tiết mới màu cam, hình nổ có đường nét đứt về vị trí lắp.

Một script, hai chế độ (cùng file):
  1. Xuất mesh (cần build123d, thường chạy trên máy dựng CAD):
       python assembly_pl.py --mesh MESH_DIR --lift 0 --pose ready
  2. Vẽ (cần Blender >= 3.6 có Freestyle, và Pillow + numpy + PyYAML ở python hệ thống):
       python render_pdf_buoc.py MESH_DIR OUT_DIR [--yaml trinh_tu_lap.yaml] [--blender blender] [--chi bia,the,buoc]
     Script tự gọi `blender -b -P render_pdf_buoc.py` để vẽ nét (Freestyle: silhouette, border, crease 35 độ) ra PNG trong suốt,
     rồi ghép nền trắng và nét đứt hình nổ bằng Pillow.

  Thêm `--style web --chi buoc,cum [--only id,...]`: hình cho web (xám đổ bóng, cam #C2410C, nền trắng, 1600x1000, WebP 1600/800 px
  vào OUT_DIR/web: buoc-NN, cum-<cụm>), một góc camera cho cả cụm (web_goc trong yaml). Xem web-assets/lap-rap/README.md.

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
# --- kiểu "web" (hình lắp ráp cho wiki/trang sản phẩm): chi tiết đổ bóng xám nhạt, nét mềm, chi tiết mới cam #C2410C, nền trắng
CAM_WEB = (0.39, 0.038, 0.0027)   # cam #C2410C (tuyến tính); chỉnh nếu ánh sáng làm lệch màu (đo trung vị ảnh)
XAM_WEB = (0.52, 0.52, 0.53)        # xám trung tính: đã lắp
RES_WEB = (1600, 1200)              # px, ảnh bước (4:3, cắt sát cụm đang lắp); bản 800 px thu từ bản 1600
BG_WEB = (245, 245, 247)            # nền #F5F5F7, dùng chung mọi ảnh web
NO_WEB = 1.4                        # độ nổ tối đa = 1,4 lần kích thước nhóm theo phương nổ
CAM_WEB_HEX = (194, 65, 12)


# ============================================================== phần chạy trong Blender
def trong_blender():
    import bpy, mathutils
    argv = sys.argv[sys.argv.index("--") + 1:]
    mesh_dir, out_dir, yaml_path, chi = argv[0], argv[1], argv[2], argv[3].split(",")
    style = argv[4] if len(argv) > 4 else "pdf"
    only = set(x for x in (argv[5].split(",") if len(argv) > 5 else []) if x)
    web = style == "web"
    CAMc, XAMc = (CAM_WEB, XAM_WEB) if web else (CAM, XAM)
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
        if web:     # gộp đỉnh trùng + làm mịn bóng theo góc 35 độ (STL chỉ có mặt tam giác)
            import bmesh
            bm = bmesh.new(); bm.from_mesh(o.data); bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-3); bm.to_mesh(o.data); bm.free()
            o.data.polygons.foreach_set("use_smooth", [True] * len(o.data.polygons))
            o.data.use_auto_smooth = True; o.data.auto_smooth_angle = math.radians(35)
        objs[n] = o
    bpy.context.view_layer.update()

    sc.render.engine = "BLENDER_WORKBENCH"
    sc.render.film_transparent = True
    sc.view_settings.view_transform = "Standard"      # không tông màu điện ảnh: giữ cam và xám đúng màu
    sc.render.image_settings.file_format = "PNG"; sc.render.image_settings.color_mode = "RGBA"
    sh = sc.display.shading
    sh.light = "FLAT"; sh.color_type = "OBJECT"; sh.show_object_outline = False
    if web:     # đổ bóng studio mềm + cavity nhẹ, giữ màu vật thể; nét Freestyle xám mỏng
        sh.light = "STUDIO"; sh.studio_light = "paint.sl"; sh.show_cavity = True; sh.cavity_type = "BOTH"
        sh.cavity_ridge_factor = 0.5; sh.cavity_valley_factor = 0.6; sh.show_shadows = True; sh.shadow_intensity = 0.25
        sh.show_specular_highlight = False
        sc.display.render_aa = "32"
        sc.display.shading.single_color = (1, 1, 1)
        # Cycles CPU (máy dựng không có EGL cho Workbench): 24 mẫu + khử nhiễu OIDN, nền trong suốt, đèn studio mềm
        sc.render.engine = "CYCLES"; cy = sc.cycles; cy.device = "CPU"; cy.samples = int(os.environ.get("WEB_SAMPLES", "20")); cy.use_adaptive_sampling = True
        cy.use_denoising = True; cy.denoiser = "OPENIMAGEDENOISE"; cy.max_bounces = 3; cy.diffuse_bounces = 2; cy.glossy_bounces = 1
        cy.transmission_bounces = 0; cy.sample_clamp_indirect = 4.0
        sc.render.threads_mode = "AUTO"
        w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True
        w.node_tree.nodes["Background"].inputs[0].default_value = (1, 1, 1, 1); w.node_tree.nodes["Background"].inputs[1].default_value = 0.75
        tg = bpy.data.objects.new("tg", None); sc.collection.objects.link(tg)
        def luz(nm, loc, size, en):
            d = bpy.data.lights.new(nm, "AREA"); d.size = size; d.energy = en
            o = bpy.data.objects.new(nm, d); sc.collection.objects.link(o); o.location = loc
            c = o.constraints.new("TRACK_TO"); c.target = tg; c.track_axis = "TRACK_NEGATIVE_Z"; c.up_axis = "UP_Y"
        luz("key", (3.0, -2.5, 4.0), 5.0, 260); luz("fill", (-3.0, 2.5, 2.0), 6.0, 90)
        def matweb(nm, c):
            m = bpy.data.materials.new(nm); m.use_nodes = True; b = m.node_tree.nodes["Principled BSDF"]
            b.inputs["Base Color"].default_value = (c[0], c[1], c[2], 1); b.inputs["Roughness"].default_value = 0.55
            b.inputs["Specular IOR Level"].default_value = 0.25
            return m
        MATW = {"cam": matweb("cam", CAM_WEB), "xam": matweb("xam", XAM_WEB)}
    sc.render.use_freestyle = True
    sc.render.line_thickness_mode = "ABSOLUTE"; sc.render.line_thickness = 1.7 if not web else 1.6
    vl = bpy.context.view_layer; vl.use_freestyle = True
    fs = vl.freestyle_settings
    while len(fs.linesets):
        fs.linesets.remove(fs.linesets[0])
    ls = fs.linesets.new("nét")
    ls.select_silhouette = True; ls.select_border = True; ls.select_crease = True
    fs.crease_angle = math.radians(35)
    ls.linestyle = bpy.data.linestyles.new("den"); ls.linestyle.color = (0.05, 0.05, 0.05); ls.linestyle.thickness = 1.5
    if web:
        ls.linestyle.color = (0.28, 0.28, 0.30); ls.linestyle.alpha = 0.75; ls.linestyle.thickness = 1.4
        fs.crease_angle = math.radians(40)

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

    def verts(names, k=500):
        """Đỉnh thật (lấy mẫu tối đa k đỉnh mỗi chi tiết, hệ thế giới): khung camera sát hơn hộp bao khi chi tiết nằm chéo."""
        import numpy as np
        pts = []
        for n in names:
            o = objs[n]; nv = len(o.data.vertices)
            a = np.empty(nv * 3, dtype=np.float32); o.data.vertices.foreach_get("co", a); a = a.reshape(-1, 3)
            a = a[:: max(1, nv // k)]
            m = np.array(o.matrix_world)
            w = a @ m[:3, :3].T + m[:3, 3]
            pts += [mathutils.Vector(r) for r in w.tolist()]
        return pts

    def proj(p):
        from bpy_extras.object_utils import world_to_camera_view
        v = world_to_camera_view(sc, cam, p)
        return [v.x * sc.render.resolution_x, (1 - v.y) * sc.render.resolution_y]

    def mau(o, c):
        o.color = (c[0], c[1], c[2], 1.0)
        if web:
            o.data.materials.clear(); o.data.materials.append(MATW["cam"] if c is CAM_WEB else MATW["xam"])

    def cam_cua(b, mac_dinh):
        """Góc camera: kiểu web dùng một góc cho cả cụm (web_goc trong yaml), PDF dùng góc từng bước."""
        if web and b["cum"] in cfg.get("web_goc", {}):
            return cfg["web_goc"][b["cum"]]
        return b.get("goc", mac_dinh)

    def ve(path, ns=None):
        sc.render.filepath = path
        if web and ns:
            sc.cycles.samples = ns
        bpy.ops.render.render(write_still=True)
        if web:
            sc.cycles.samples = int(os.environ.get("WEB_SAMPLES", "20"))

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

    def vnp(names, k=600):
        """Đỉnh thật của các chi tiết (np, Nx3, m, hệ thế giới)."""
        import numpy as np
        out = []
        for n in names:
            o = objs[n]; nv = len(o.data.vertices)
            a = np.empty(nv * 3, dtype=np.float32); o.data.vertices.foreach_get("co", a); a = a.reshape(-1, 3)
            a = a[:: max(1, nv // k)]
            m = np.array(o.matrix_world)
            out.append(a @ m[:3, :3].T + m[:3, 3])
        return np.concatenate(out) if out else np.zeros((0, 3))

    def web_step(b, bi, hien_cu, moi):
        """Hình bước kiểu web: camera sát nhóm đang lắp, nổ ngắn (<= NO_WEB x kích thước), bóng mờ ở vị trí lắp;
        ghi _raw (tất cả), _E-<gi> (nhóm đã nổ, một mình), _G-<gi> (nhóm ở vị trí lắp, một mình) để ghép nét đứt/mũi tên."""
        import numpy as np
        for o in objs.values():
            o.hide_render = True; o.location = (0, 0, 0)
        bpy.context.view_layer.update()
        grp = []         # (gi, tên, d, s, P_lap)
        boxes = []
        for gi, (g, ten) in enumerate(zip(b["nhom"], moi)):
            if not ten:
                continue
            P = vnp(ten)
            if "radial" in g:
                cc = P.mean(0); h = np.array([cc[0], cc[1], 0.0]); h = h / np.linalg.norm(h) if np.linalg.norm(h) > 1e-6 else np.array([1.0, 0, 0])
                d, L0 = h, g["radial"] / 1000
            else:
                v = np.array(g["lech"], float) / 1000; L0 = float(np.linalg.norm(v)); d = v / L0 if L0 > 0 else v
            if L0 <= 0:
                grp.append((gi, ten, d, 0.0, P)); continue
            ext = max(float(np.ptp(P @ d)), 0.4 * float(np.ptp(P, axis=0).max()))      # tấm mỏng: lấy theo cạnh lớn, không theo độ dày
            sh = min(L0, NO_WEB * ext)
            for _ in range(10):         # không để hai nhóm đã nổ chồng nhau
                Pe = P + d * sh; lo, hi = Pe.min(0), Pe.max(0); mg = 0.1 * (hi - lo)
                if any(np.all(lo + mg < bh) and np.all(hi - mg > bl) for bl, bh in boxes):
                    sh += 0.3 * ext
                else:
                    break
            Pe = P + d * sh; boxes.append((Pe.min(0), Pe.max(0)))
            grp.append((gi, ten, d, sh, P))
        if not grp:
            return
        for n in hien_cu:
            objs[n].hide_render = False; mau(objs[n], XAMc)
        for gi, ten, d, sh, P in grp:
            for n in ten:
                objs[n].hide_render = False; objs[n].location = tuple(d * sh); mau(objs[n], CAMc)
        pts = np.concatenate([np.concatenate([P, P + d * sh]) for gi, ten, d, sh, P in grp])
        pts = pts[:: max(1, len(pts) // 5000)]
        el, az = cam_cua(b, goc0)
        dat_cam(el, az, [mathutils.Vector(r) for r in pts.tolist()], RES_WEB, pad=1.10)
        ve(os.path.join(out_dir, f"_raw-{b['id']}.png"))
        ng = []
        for gi, ten, d, sh, P in grp:
            for o in objs.values():
                o.hide_render = True; o.location = (0, 0, 0)
            for n in ten:
                objs[n].hide_render = False; mau(objs[n], CAMc); objs[n].location = tuple(d * sh)
            ve(os.path.join(out_dir, f"_E-{b['id']}-{gi}.png"), 12)
            if sh > 0:
                for n in ten:
                    objs[n].location = (0, 0, 0)
                ve(os.path.join(out_dir, f"_G-{b['id']}-{gi}.png"), 4)
            ng.append({"gi": gi, "no": sh > 0, "sh_mm": round(sh * 1000, 1)})
        info["buoc"][b["id"]] = {"web": ng}
        print("buoc", b["id"], [x["sh_mm"] for x in ng], flush=True)

    if "buoc" in chi:
        for bi, b in enumerate(cfg["buoc"]):
            if only and b["id"] not in only:
                continue
            ids = [x["id"] for x in cfg["buoc"]]
            hien_cu = [n for n, (i, _) in gan.items() if i < bi]
            moi = [[n for n, (i, g) in gan.items() if i == bi and g == gi] for gi in range(len(b["nhom"]))]
            for gi, g in enumerate(b["nhom"]):      # nhóm "cua_buoc": nhấc lại chi tiết đã lắp ở các bước đó
                if "cua_buoc" in g:
                    ib = {ids.index(x) for x in g["cua_buoc"]}
                    moi[gi] = [n for n, (i, _) in gan.items() if i in ib]
                    hien_cu = [n for n in hien_cu if n not in moi[gi]]
            if web:
                web_step(b, bi, hien_cu, moi)
                continue
            for o in objs.values():
                o.hide_render = True
                o.location = (0, 0, 0)
            bpy.context.view_layer.update()      # đưa matrix_world về vị trí lắp trước khi lấy góc/đỉnh
            for n in hien_cu:
                objs[n].hide_render = False; mau(objs[n], XAMc)
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
                    objs[n].hide_render = False; objs[n].location = v; mau(objs[n], CAMc)
                pt_goc += c_lap + [c + v for c in c_lap]
                nhom_info.append((ten_nhom, v, c_lap))
            if not pt_goc:
                continue
            el, az = cam_cua(b, goc0)
            if web:      # khung theo chi tiết mới (cả vị trí nổ), phần đã lắp ở quanh bị cắt theo khung
                pv = []
                for ten_nhom, v, c_lap in nhom_info:
                    q = verts(ten_nhom); pv += q + [x + v for x in q]
                dat_cam(el, az, pv, RES_WEB, pad=1.18)
            else:
                pts = pt_goc + corners(hien_cu) if hien_cu else pt_goc
                dat_cam(el, az, pts, RES)
            ve(os.path.join(out_dir, f"_raw-{b['id']}.png"))
            ni = []
            for ten_nhom, v, c_lap in nhom_info:
                ni.append({"lap": [proj(c) for c in c_lap], "no": [proj(c + v) for c in c_lap]})
            info["buoc"][b["id"]] = {"nhom": ni, "so_ct": len(pt_goc) // 16}
            print("buoc", b["id"], flush=True)

    # ---- thẻ chi tiết
    if "the" in chi and not web:
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
    if "bia" in chi and not web:
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


def ghep_web(out_dir, bid, bi, thu_tu):
    """Ghép ảnh bước kiểu web: nền #F5F5F7, bóng mờ ở vị trí lắp, ảnh chính (phần xa mờ dần), nét đứt cam trong khoảng trống
    giữa nhóm đã nổ và vị trí lắp, mũi tên cam kết thúc đúng mép bóng (điểm lắp), nhóm đã nổ vẽ lên trên cùng. WebP 1600 và 800 px."""
    import numpy as np
    from PIL import Image, ImageDraw, ImageFilter
    P = lambda n: os.path.join(out_dir, n)
    raw = Image.open(P(f"_raw-{bid}.png")).convert("RGBA")
    W, H = raw.size
    cam = CAM_WEB_HEX + (255,)
    nen = Image.new("RGBA", raw.size, BG_WEB + (255,))
    Es, Gs = {}, {}
    for g in bi["web"]:
        Es[g["gi"]] = Image.open(P(f"_E-{bid}-{g['gi']}.png")).convert("RGBA")
        if g["no"]:
            Gs[g["gi"]] = Image.open(P(f"_G-{bid}-{g['gi']}.png")).convert("RGBA")
    # bóng mờ ở vị trí lắp: viền cam mảnh dưới ảnh chính
    for gi, G in Gs.items():
        a = G.split()[3].point(lambda v: 255 if v > 40 else 0)
        edge = ImageChops_sub(a.filter(ImageFilter.MaxFilter(5)), a)
        ov = Image.new("RGBA", raw.size, CAM_WEB_HEX + (0,)); ov.putalpha(edge.point(lambda v: int(v * 0.55)))
        nen.alpha_composite(ov)
    # phần xa nhóm đang lắp mờ dần về nền
    un = np.zeros((H, W), bool)
    for gi in Es:
        un |= np.array(Es[gi].split()[3]) > 40
    for gi in Gs:
        un |= np.array(Gs[gi].split()[3]) > 40
    ys, xs = np.nonzero(un)
    x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    X, Y = np.meshgrid(np.arange(W), np.arange(H))
    dd = np.hypot(np.maximum(np.maximum(x0 - X, X - x1), 0), np.maximum(np.maximum(y0 - Y, Y - y1), 0))
    fade = np.clip(1.0 - (dd - 0.05 * W) / (0.30 * W), 0.0, 1.0)
    fa = np.array(raw.split()[3]).astype(float) * fade
    raw.putalpha(Image.fromarray(fa.astype(np.uint8)))
    nen.alpha_composite(raw)
    # nét đứt + mũi tên trên một lớp riêng, cắt bỏ phần nằm dưới nhóm đã nổ
    lay = Image.new("RGBA", raw.size, (0, 0, 0, 0)); dr = ImageDraw.Draw(lay)
    cen = lambda im: np.array(np.nonzero(np.array(im.split()[3]) > 40))[::-1].mean(1)
    for g in bi["web"]:
        if not g["no"]:
            continue
        E, G = Es[g["gi"]], Gs[g["gi"]]
        ce, cg = cen(E), cen(G)
        L = float(np.hypot(*(cg - ce)))
        if L < 20:
            continue
        u = (cg - ce) / L
        ea, ga = np.array(E.split()[3]) > 40, np.array(G.split()[3]) > 40
        # tia theo hướng nổ bắt đầu từ vài điểm của nhóm đã nổ; chọn tia có khoảng trống lớn nhất giữa mép nhóm nổ và mép bóng
        ys, xs = np.nonzero(ea)
        rng = np.random.default_rng(1)
        sel = rng.choice(len(xs), min(250, len(xs)), replace=False)
        cands = [ce] + [np.array([xs[i], ys[i]], float) for i in sel]
        best = None
        ts = np.arange(0, 2.0 * L + 300, 2.0)
        for p0 in cands:
            px = np.clip(np.round(p0[0] + u[0] * ts).astype(int), 0, W - 1); py = np.clip(np.round(p0[1] + u[1] * ts).astype(int), 0, H - 1)
            ine, ing = ea[py, px], ga[py, px]
            if not ine[0]:
                continue
            k0 = int(np.argmax(~ine))                    # ra khỏi nhóm đã nổ
            if ine[k0]:
                continue
            h = np.nonzero(ing[k0:])[0]
            if len(h) == 0:
                continue
            k1 = k0 + int(h[0])                          # chạm mép bóng
            gap = ts[k1] - ts[k0]
            sc_ = gap - 0.3 * float(np.hypot(*(p0 - ce)))
            if gap >= 24 and (best is None or sc_ > best[0]):
                best = (sc_, p0, ts[k0], ts[k1])
        if best is None:
            continue
        _, p0, t0, t1 = best
        a, b = p0 + u * t0, p0 + u * t1
        net_dut(dr, tuple(a), tuple(b), dash=16, gap=11, w=3, fill=cam)
        if t1 - t0 > 70:
            hl = min(44.0, (t1 - t0) * 0.5)
            dr.line([tuple(b - u * (hl + 70)) if t1 - t0 > hl + 70 else tuple(a), tuple(b - u * hl * 0.5)], fill=cam, width=5)
            mui_ten(dr, tuple(b - u * hl), tuple(b), size=hl, fill=cam)
    m = np.zeros((H, W), bool)
    for gi in Es:
        m |= np.array(Es[gi].split()[3]) > 40
    la = np.array(lay.split()[3]).astype(float); la[m] = 0
    lay.putalpha(Image.fromarray(la.astype(np.uint8)))
    nen.alpha_composite(lay)
    for gi in Es:
        nen.alpha_composite(Es[gi])
    nen = nen.convert("RGB")
    d = os.path.join(out_dir, "web"); os.makedirs(d, exist_ok=True)
    ten = "buoc-%02d" % thu_tu
    nen.save(os.path.join(d, ten + ".webp"), quality=90, method=6)
    nen.resize((800, round(800 * H / W)), Image.LANCZOS).save(os.path.join(d, ten + "-800.webp"), quality=90, method=6)
    nen.save(os.path.join(out_dir, ten + ".png"), optimize=True)
    box = (int(x0), int(y0), int(x1), int(y1))
    for f in os.listdir(out_dir):
        if f.startswith((f"_raw-{bid}.", f"_E-{bid}-", f"_G-{bid}-")):
            os.remove(P(f))
    return {"bbox_px": box, "fill_w": round((x1 - x0) / W, 3), "fill_h": round((y1 - y0) / H, 3)}


def ImageChops_sub(a, b):
    from PIL import ImageChops
    return ImageChops.subtract(a, b)


def ghep(out_dir, chi, style="pdf", yaml_path=None):
    from PIL import Image, ImageDraw
    info = json.load(open(os.path.join(out_dir, "buoc.json")))
    if style == "web":
        cfg = json.load(open(yaml_path, encoding="utf-8"))
        ids = [b["id"] for b in cfg["buoc"]]
        fills = {}
        for bid, bi in info["buoc"].items():
            r = ghep_web(out_dir, bid, bi, ids.index(bid) + 1)
            print("fill", bid, r)
            fills[bid] = r
        json.dump(fills, open(os.path.join(out_dir, "web", "fill.json"), "w"), indent=1)
        return
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


def ghi_sel(mesh, out, cfg):
    """Danh sách chi tiết cho ảnh cụm (render_scene.py --sel): cụm đang xét giữ màu sản phẩm, chi tiết của bước cuối có chi tiết riêng
    màu cam #C2410C, phần còn lại của robot xám nhạt. Khung theo cụm (cộng thêm chi tiết khớp web_cum_khung của cụm nếu có)."""
    man = [n for n, _ in json.load(open(os.path.join(mesh, "manifest.json")))]
    gan = {}
    for bi, b in enumerate(cfg["buoc"]):
        for gi, g in enumerate(b["nhom"]):
            for n in man:
                if "re" in g and n not in gan and re.search(g["re"], n):
                    gan[n] = bi
    for ck in cfg["cum"]:
        idx = [i for i, b in enumerate(cfg["buoc"]) if b["cum"] == ck]
        prod = [n for n in man if gan.get(n) in idx]
        co = [i for i in idx if any(gan.get(n) == i for n in man)]
        hi = [n for n in man if gan.get(n) == max(co)]
        kh = cfg.get("web_cum_khung", {}).get(ck)
        subj = prod + ([n for n in man if re.search(kh, n) and n not in prod] if kh else [])
        json.dump({"prod": prod, "hi": hi, "hide": [], "subject": subj, "buoc_cuoi": cfg["buoc"][max(co)]["id"]},
                  open(os.path.join(out, f"sel-{ck}.json"), "w"), ensure_ascii=False, indent=1)
        print("sel", ck, len(prod), "cam:", cfg["buoc"][max(co)]["id"], len(hi))
    # ảnh sản phẩm (render_scene.py): robot đủ bộ, trạm đứng riêng, đế ngàm có chốt côn + khoá kéo, nút E-stop
    arm = re.compile(r"^M1-(AR|SH|SV-001_(gripper|wrist)|CM-001_wrist)")
    tr = re.compile(r"^M1-TR-0(0[1-9]|1[0-4])")
    pick = lambda rx: [n for n in man if re.search(rx, n)]
    SP = {"sau": dict(prod=man, hi=[], hide=[], subject=[]),
          "tram-u": dict(prod=[n for n in man if tr.search(n) or arm.search(n) or n.startswith("M1-CM-001_head")], hi=[], hide=[], subject=[]),
          "tram-gan": dict(prod=[n for n in man if not arm.search(n)], hi=pick(r"^M1-TR-(021|008)"), hide=[n for n in man if arm.search(n)], subject=pick(r"^M1-TR-(02[0-2]|008)")),
          "estop": dict(prod=[n for n in man if not arm.search(n)], hi=pick(r"^M1-TR-012b"), hide=[n for n in man if arm.search(n)], subject=pick(r"^M1-TR-(012|002_tai_2040_L)"))}
    SP["tram-u"]["hide"] = [n for n in man if n not in SP["tram-u"]["prod"]]
    for k, v in SP.items():
        json.dump(v, open(os.path.join(out, f"sel-{k}.json"), "w"), ensure_ascii=False, indent=1)
        print("sel", k, len(v["prod"]), "cam:", len(v["hi"]))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh"); ap.add_argument("out")
    ap.add_argument("--yaml", default=os.path.join(HERE, "trinh_tu_lap.yaml"))
    ap.add_argument("--blender", default="blender")
    ap.add_argument("--chi", default="bia,the,buoc")
    ap.add_argument("--style", default="pdf", choices=["pdf", "web"], help="web: xám đổ bóng + cam #C2410C, nền trắng, WebP 1600/800 px (cần --chi buoc,cum)")
    ap.add_argument("--only", default="", help="chỉ vẽ các id bước / cụm này, phân cách bằng dấu phẩy")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    import yaml
    cfgj = os.path.join(a.out, "_cfg.json")
    json.dump(yaml.safe_load(open(a.yaml, encoding="utf-8")), open(cfgj, "w"), ensure_ascii=False)
    a.yaml = cfgj
    if "sel" in a.chi.split(","):
        ghi_sel(a.mesh, a.out, json.load(open(cfgj, encoding="utf-8")))
        a.chi = ",".join(c for c in a.chi.split(",") if c != "sel")
        if not a.chi:
            return
    cmd = [a.blender, "-b", "-P", os.path.abspath(__file__), "--", os.path.abspath(a.mesh), os.path.abspath(a.out), os.path.abspath(a.yaml), a.chi, a.style, a.only]
    r = subprocess.run(cmd, stdout=open(os.path.join(a.out, "blender.log"), "w"), stderr=subprocess.STDOUT)
    if r.returncode != 0 or "Traceback" in open(os.path.join(a.out, "blender.log")).read():
        sys.exit("Blender lỗi, xem " + os.path.join(a.out, "blender.log"))
    ghep(a.out, a.chi, a.style, a.yaml)
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
