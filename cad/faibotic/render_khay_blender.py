# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""R4 khay: hiện tại / --de-xuat (khay B gắn xe nâng, ẩn khay đế) / --hai-tang (giữ khay đế + laptop đóng nắp + khay B). Gốc: m1-r11/rw/web2.py.
Render ảnh web Faibot M1 v0.5 từ m1.glb (4aaaa56): tư thế theo joints.json, sân khấu + camera theo render_scene.py/views.json.
Thêm --khay-de trong|laptop|jetson [--khay-de-cad cad/export/khay-de] (view "khay" trong media/renders-web/src/views.json): khay đế M1-ST-012 + vật mô phỏng đúng kích thước.
Chạy: blender -b -P render_glb_views.py -- DIR VIEW OUT.png [--pose ready] [--lift M] [--samples N] [--scale k]
DIR chứa m1.glb, joints.json, views.json. Vật liệu lấy từ PBR trong GLB. Không chữ, không nhãn."""
import bpy, sys, os, math, json
from mathutils import Vector, Quaternion
argv = sys.argv[sys.argv.index("--") + 1:]
DIR, VIEW, OUT = argv[0], argv[1], argv[2]
opt = lambda k, d: type(d)(argv[argv.index(k) + 1]) if k in argv else d
SAMPLES, SCALE, POSE = opt("--samples", 48), opt("--scale", 1.0), opt("--pose", "ready")
LIFT = opt("--lift", -1.0)
def lin(h):
    h = h.lstrip("#"); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return [(x / 12.92 if x <= .04045 else ((x + .055) / 1.055) ** 2.4) for x in c] + [1]
bpy.ops.wm.read_factory_settings(use_empty=True); sc = bpy.context.scene
STL = argv[argv.index("--stl") + 1] if "--stl" in argv else None
MAT = {"ivory": ("#f1efe8", .5, 0), "ivory_hi": ("#f6f5f1", .45, 0), "navy": ("#1e3a5f", .42, 0), "alu": ("#b3b7bd", .40, .85), "steel": ("#8d939b", .35, 1.0), "servo": ("#1b1d21", .62, 0), "servo_body": ("#1a1a1a", .45, 0), "pad": ("#33363d", .75, 0), "rubber": ("#26282d", .8, 0), "hub": ("#6d727a", .42, .9)}
BAN = "--ban" in argv
if BAN: bpy.ops.import_scene.gltf(filepath=os.path.join(DIR, "m1.glb"))
if STL:
    mats = {}
    for n, m in json.load(open(os.path.join(STL, "manifest.json"))):
        f = os.path.join(STL, n + ".stl")
        if not os.path.exists(f) or (BAN and n.startswith(("M1-AR", "M1-SH", "M1-SV", "M1-CM-001_wrist"))): continue
        bpy.ops.wm.stl_import(filepath=f); o = bpy.context.selected_objects[0]
        o.scale = (0.001,) * 3; o.rotation_euler = (0, 0, math.pi / 2)
        if m not in mats:
            h, r, me = MAT.get(m, MAT["ivory"]); mt = bpy.data.materials.new(m); mt.use_nodes = True; b = mt.node_tree.nodes["Principled BSDF"]
            b.inputs["Base Color"].default_value = lin(h); b.inputs["Roughness"].default_value = r; b.inputs["Metallic"].default_value = me; mats[m] = mt
        o.data.materials.append(mats[m]); o.data.polygons.foreach_set("use_smooth", [False] * len(o.data.polygons))
else:
    bpy.ops.import_scene.gltf(filepath=os.path.join(DIR, "m1.glb"))
J = json.load(open(os.path.join(DIR, "joints.json"))); q = dict(J["poses"][POSE]["q"])
if LIFT >= 0: q["lift"] = LIFT
conv = lambda a: Vector((a[0], -a[2], a[1]))
obj = {o.name: o for o in bpy.data.objects}
if BAN: q["lift"] = 0.16      # chế độ bàn: khung trạm ở lift CAD 0 = lift SDK 0,16 (LIFT_CAD_OFFSET)
for k, j in ([] if (STL and not BAN) else J["joints"].items()):
    v = q.get(k, 0.0); o = obj[j["node"]]; ax = conv(j["axis"])
    if j["type"] == "revolute":
        v = v / 100.0 * j.get("open_rad", 0) if k.endswith("gripper") else v
        o.rotation_mode = "QUATERNION"; o.rotation_quaternion = o.rotation_quaternion @ Quaternion(ax, v)
    else:
        d = v if k == "lift" else 0.0; o.location = o.location + ax * d
        if "mimic_node" in j: ob = obj[j["mimic_node"]]; ob.location = ob.location + conv(j["mimic_axis"]) * d
# GLB: trước robot = +Y Blender. Xoay -90 độ quanh Z để trước = +X như hệ CAD của render_scene.py
if BAN:   # giữ trạm hai tay từ GLB (cụm vai, tay, kẹp, camera cổ tay); bỏ đế, trụ, xe nâng của robot di động
    for o in list(bpy.data.objects):
        if o.type == "MESH" and not o.name.startswith(("shoulder_", "arm_", "gripper_", "camera.left_wrist", "camera.right_wrist")) and o.name not in [x.name for x in bpy.data.objects if x.data and x.data.name.endswith(".stl")] and not o.name.startswith("M1-"):
            bpy.data.objects.remove(o, do_unlink=True)
rot = bpy.data.objects.new("rot", None); sc.collection.objects.link(rot); rot.rotation_euler = (0, 0, -math.pi / 2)
for o in list(bpy.data.objects):
    if o is not rot and o.parent is None: o.parent = rot
bpy.context.view_layer.update()
# --- khay (R4): --de-xuat: bỏ khay trên đế (cắt đỉnh lưới base trong hộp khay+giá) và thêm khay gắn xe nâng (phương án B) ---
HAI_TANG = "--hai-tang" in argv   # hai tầng: GIỮ khay đế, thêm laptop đóng nắp 0,33x0,23x0,02 m (x 0,18, y 0) + khay B
if "--de-xuat" in argv or HAI_TANG:
    import bmesh
    for o in ([] if HAI_TANG else [o for o in bpy.data.objects if o.type == "MESH" and o.name.startswith("base")]):
        bm = bmesh.new(); bm.from_mesh(o.data); M = o.matrix_world
        dv = [v for v in bm.verts if (lambda p: p.z > 0.11 and p.x > 0.09 and abs(p.y) < 0.17 and p.z < 0.26)(M @ v.co)]
        if dv: print("cat", o.name, len(dv), flush=True); bmesh.ops.delete(bm, geom=dv, context="VERTS"); bm.to_mesh(o.data)
        bm.free()
    L = q.get("lift", 0.0); zt = 0.4137 + L - 0.17
    pm = bpy.data.materials.new("khay_b"); pm.use_nodes = True; pb = pm.node_tree.nodes["Principled BSDF"]
    pb.inputs["Base Color"].default_value = lin("#e8762b"); pb.inputs["Roughness"].default_value = 0.5
    def box(n, x0, x1, y0, y1, z0, z1):
        bpy.ops.mesh.primitive_cube_add(size=1, location=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
        b = bpy.context.active_object; b.name = n; b.scale = (x1 - x0, y1 - y0, z1 - z0); b.data.materials.append(pm); return b
    box("khayB_tam", 0.20, 0.32, -0.13, 0.13, zt - 0.006, zt)
    box("khayB_go_truoc", 0.314, 0.32, -0.13, 0.13, zt, zt + 0.008)
    box("khayB_go_trai", 0.20, 0.32, 0.124, 0.13, zt, zt + 0.008)
    box("khayB_go_phai", 0.20, 0.32, -0.13, -0.124, zt, zt + 0.008)
    box("khayB_go_sau", 0.20, 0.206, -0.13, 0.13, zt, zt + 0.008)
    box("khayB_gia", 0.18, 0.20, -0.02, 0.02, zt - 0.006, 0.4137 + L)
    if HAI_TANG:
        lm = bpy.data.materials.new("laptop"); lm.use_nodes = True; lb = lm.node_tree.nodes["Principled BSDF"]
        lb.inputs["Base Color"].default_value = lin("#4a4f57"); lb.inputs["Roughness"].default_value = 0.45; lb.inputs["Metallic"].default_value = 0.6
        pm_, pm = pm, lm; box("laptop_dong", 0.015, 0.345, -0.115, 0.115, 0.218, 0.238); pm = pm_
    print("khay B mat", round(zt, 4), "lift", L, flush=True)
    bpy.context.view_layer.update()
# --- khay đế mới (M1-ST-012, đã có trong GLB): --khay-de trong|laptop|jetson thêm khối vật mô phỏng đúng kích thước/vị trí (R5), cùng góc máy (view "khay") ---
KD = argv[argv.index("--khay-de") + 1] if "--khay-de" in argv else None
KDC = argv[argv.index("--khay-de-cad") + 1] if "--khay-de-cad" in argv else None   # thư mục cad/export/khay-de: thay khay đơn giản của GLB bằng STL CAD đủ chi tiết
if KDC:
    import bmesh
    for o_ in [o for o in bpy.data.objects if o.type == "MESH" and o.name.startswith("base")]:
        bm = bmesh.new(); bm.from_mesh(o_.data); M_ = o_.matrix_world
        dv = [v for v in bm.verts if (lambda p: 0.06 < p.x < 0.33 and abs(p.y) < 0.2 and 0.1865 < p.z < 0.225)(M_ @ v.co)]
        if dv: bmesh.ops.delete(bm, geom=dv, context="VERTS"); bm.to_mesh(o_.data)
        bm.free()
    _mats = {}
    for fn_, mk in (("M1-ST-012_khay_de_00", "ivory"), ("M1-ST-012_khay_de_01", "ivory"), ("M1-ST-012_khay_de_10", "ivory"), ("M1-ST-012_khay_de_11", "ivory"),
                    ("M1-ST-011_cot_do_cat", "navy"), ("2040_200_tham_chieu", "alu")):
        bpy.ops.wm.stl_import(filepath=os.path.join(KDC, fn_ + ".stl")); o_ = bpy.context.selected_objects[0]; o_.name = "kdc_" + fn_
        o_.scale = (0.001,) * 3
        if mk not in _mats:
            h_, r_, me_ = MAT[mk]; mt_ = bpy.data.materials.new("kdc_" + mk); mt_.use_nodes = True; b_ = mt_.node_tree.nodes["Principled BSDF"]
            b_.inputs["Base Color"].default_value = lin(h_); b_.inputs["Roughness"].default_value = r_; b_.inputs["Metallic"].default_value = me_; _mats[mk] = mt_
        o_.data.materials.append(_mats[mk]); o_.data.polygons.foreach_set("use_smooth", [False] * len(o_.data.polygons))
    bpy.context.view_layer.update()
if KD in ("laptop", "jetson"):
    bpy.context.view_layer.update()
    def blk(n, x0, x1, y0, y1, z0, z1, hexc, rough, metal):
        m_ = bpy.data.materials.new(n); m_.use_nodes = True; b_ = m_.node_tree.nodes["Principled BSDF"]
        b_.inputs["Base Color"].default_value = lin(hexc); b_.inputs["Roughness"].default_value = rough; b_.inputs["Metallic"].default_value = metal
        bpy.ops.mesh.primitive_cube_add(size=1, location=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
        o_ = bpy.context.active_object; o_.name = n; o_.scale = (x1 - x0, y1 - y0, z1 - z0); o_.data.materials.append(m_); return o_
    ZT = 0.210 + 0.002        # mặt da khay + dải cao su 2 mm
    if KD == "laptop":      # laptop 16 in gập: 0,355 x 0,250 x 0,025, tựa gờ sau (mặt trong x 0,067)
        objs = [blk("kd_laptop", 0.067, 0.317, -0.1775, 0.1775, ZT, ZT + 0.025, "#8d939b", 0.35, 0.9),
                blk("kd_laptop_logo", 0.15, 0.23, -0.04, 0.04, ZT + 0.0248, ZT + 0.0252, "#c9ccd1", 0.3, 0.9)]
    else:                   # Jetson AGX + đệm 80 mm tại (0,150; 0) + DC-DC 0,075 x 0,045 x 0,03 cạnh phải
        objs = [blk("kd_jetson", 0.150 - 0.0575, 0.150 + 0.0575, -0.0575, 0.0575, ZT, ZT + 0.080, "#1d2a24", 0.5, 0.2),
                blk("kd_jetson_tan", 0.150 - 0.045, 0.150 + 0.045, -0.045, 0.045, ZT + 0.080, ZT + 0.0805, "#7b8088", 0.4, 0.8),
                blk("kd_dcdc", 0.235, 0.310, -0.0225, 0.0225, ZT, ZT + 0.030, "#2f4f7f", 0.5, 0.1)]
    bpy.context.view_layer.update()
mesh = [o for o in bpy.data.objects if o.type == "MESH"]
_lo_all = min((o.matrix_world @ Vector(c)).z for o in mesh if not o.name.startswith(("kd_", "kdc_")) for c in o.bound_box)
_sub = json.load(open(os.path.join(DIR, "views.json")))[VIEW].get("subject")
if _sub == "armL": mesh = [o for o in mesh if o.name.startswith(("shoulder_L", "arm_L", "gripper_L"))]
if _sub == "khay":    # khung cố định quanh khay + đáy xe nâng (không phụ thuộc vật trên khay) để ba ảnh trống/laptop/Jetson cùng góc máy
    corners = [Vector((x, y, z)) for x in (0.0, 0.36) for y in (-0.22, 0.22) for z in (_lo_all + 0.10, _lo_all + 0.40)]
else:
    corners = [o.matrix_world @ Vector(c) for o in mesh for c in o.bound_box]
lo = Vector((min(p.x for p in corners), min(p.y for p in corners), min(p.z for p in corners)))
hi = Vector((max(p.x for p in corners), max(p.y for p in corners), max(p.z for p in corners)))
print("bbox", [round(x, 3) for x in lo], [round(x, 3) for x in hi], "pose", POSE, "lift", q.get("lift"), flush=True)
cen = sum(corners, Vector()) / len(corners)
# phông studio: sàn + tường cong (đặt tại đáy robot)
zf = _lo_all
prof = [(0, 0)] + [(r, 0) for r in (2, 4, 7)] + [(7 + 2.5 * math.sin(a), 2.5 - 2.5 * math.cos(a)) for a in [i * math.pi / 2 / 12 for i in range(1, 13)]] + [(9.5, 8), (9.5, 14)]
NS = 120; V = []; F = []
for r, z in prof:
    for i in range(NS): a = 2 * math.pi * i / NS; V.append((r * math.cos(a), r * math.sin(a), z + zf))
for j in range(len(prof) - 1):
    for i in range(NS): F.append((j * NS + i, j * NS + (i + 1) % NS, (j + 1) * NS + (i + 1) % NS, (j + 1) * NS + i))
me = bpy.data.meshes.new("cyc"); me.from_pydata(V, [], F); me.update(); cyc = bpy.data.objects.new("cyc", me); sc.collection.objects.link(cyc)
me.polygons.foreach_set("use_smooth", [True] * len(F))
fm = bpy.data.materials.new("floor"); fm.use_nodes = True; fm.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = lin("#d6d3cc"); fm.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = .9
cyc.data.materials.append(fm); cyc.is_shadow_catcher = True; sc.render.film_transparent = True
w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True; bg = w.node_tree.nodes["Background"]; bg.inputs[0].default_value = lin("#d6d3cc"); bg.inputs[1].default_value = 0.55
tgt = bpy.data.objects.new("tgt", None); sc.collection.objects.link(tgt); tgt.location = (0.0, 0, 0.65 + zf)
def area(name, loc, size, energy, color=(1, 1, 1)):
    d = bpy.data.lights.new(name, "AREA"); d.size = size; d.energy = energy; d.color = color
    o = bpy.data.objects.new(name, d); sc.collection.objects.link(o); o.location = loc
    c = o.constraints.new("TRACK_TO"); c.target = tgt; c.track_axis = "TRACK_NEGATIVE_Z"; c.up_axis = "UP_Y"
area("key", (3.4, 2.8, 3.6 + zf), 6.0, 150, (1.0, .985, .96)); area("fill", (3.2, -3.4, 2.2 + zf), 7.0, 80, (.95, .97, 1)); area("rim", (-3.4, 0.8, 2.6 + zf), 4.0, 120, (.96, .98, 1))
area("top", (0.3, 0.0, 4.6 + zf), 6.0, 60)
cam = bpy.data.cameras.new("cam"); co = bpy.data.objects.new("cam", cam); sc.collection.objects.link(co); sc.camera = co
ct = bpy.data.objects.new("ct", None); sc.collection.objects.link(ct)
tc = co.constraints.new("TRACK_TO"); tc.target = ct; tc.track_axis = "TRACK_NEGATIVE_Z"; tc.up_axis = "UP_Y"
v = json.load(open(os.path.join(DIR, "views.json")))[VIEW]
sc.render.resolution_x = int(v["res"][0] * SCALE); sc.render.resolution_y = int(v["res"][1] * SCALE); sc.render.resolution_percentage = 100
from bpy_extras.object_utils import world_to_camera_view
cam.lens = v["lens"]; cam.sensor_width = 36
ct.location = cen
dirv = Vector((math.cos(math.radians(v["az"])), math.sin(math.radians(v["az"])), 0)); dist = 4.0
zc = lambda: (v["h"] + zf if "h" in v else ct.location.z + v["dz"])
for it in range(8):
    co.location = ct.location + dirv * dist; co.location.z = zc(); bpy.context.view_layer.update()
    pts = [world_to_camera_view(sc, co, p) for p in corners]
    x0, x1, y0, y1 = min(p.x for p in pts), max(p.x for p in pts), min(p.y for p in pts), max(p.y for p in pts)
    fw, fh = x1 - x0, y1 - y0
    dist *= max(fh / v["fill"], fw / (v["fill"] + 0.12))
    mx, my = (x0 + x1) / 2 - 0.5, (y0 + y1) / 2 - 0.5
    mw = co.matrix_world; right, up = mw.to_3x3() @ Vector((1, 0, 0)), mw.to_3x3() @ Vector((0, 1, 0))
    wv_x = (co.location - ct.location).length * 36.0 / v["lens"]; wv_y = wv_x * sc.render.resolution_y / sc.render.resolution_x
    ct.location += right * mx * wv_x + up * my * wv_y
co.location = ct.location + dirv * dist; co.location.z = zc(); bpy.context.view_layer.update()
MASK = argv[argv.index("--mask") + 1] if "--mask" in argv else None
if MASK:
    sv = sorted(o.name for o in bpy.data.objects if o.type == "MESH" and ".servo_body." in o.name)
    json.dump(sv, open(OUT + ".json", "w"))
    for o in list(bpy.data.objects):
        if o.type != "MESH" or o.name == "cyc":
            if o.name == "cyc": bpy.data.objects.remove(o, do_unlink=True)
            continue
        i = sv.index(o.name) + 1 if o.name in sv else 0
        if MASK == "solo" and not i: bpy.data.objects.remove(o, do_unlink=True); continue
        mt = bpy.data.materials.new("m%d" % i); mt.use_nodes = True; nt_ = mt.node_tree; nt_.nodes.clear()
        e = nt_.nodes.new("ShaderNodeEmission"); e.inputs[0].default_value = ((i * 13 % 256) / 255, i / 255, 0, 1) if i else (0, 0, 0, 1); e.inputs[1].default_value = 1
        out_ = nt_.nodes.new("ShaderNodeOutputMaterial"); nt_.links.new(e.outputs[0], out_.inputs[0])
        o.data.materials.clear(); o.data.materials.append(mt)
        for p_ in o.data.polygons: p_.material_index = 0
    for l in [o for o in bpy.data.objects if o.type == "LIGHT"]: bpy.data.objects.remove(l, do_unlink=True)
    sc.world.node_tree.nodes["Background"].inputs[1].default_value = 0
    sc.render.engine = "CYCLES"; sc.cycles.samples = 1; sc.cycles.use_denoising = False; sc.cycles.pixel_filter_type = "BOX"; sc.cycles.filter_width = 0.01
    sc.view_settings.view_transform = "Raw"; sc.render.film_transparent = False; sc.use_nodes = False
    sc.render.image_settings.file_format = "PNG"; sc.render.image_settings.color_depth = "8"; sc.render.filepath = OUT; bpy.ops.render.render(write_still=True); print("DA GHI", OUT); sys.exit(0)
sc.render.engine = "CYCLES"; cy = sc.cycles; cy.device = "CPU"; cy.samples = SAMPLES; cy.use_adaptive_sampling = True; cy.adaptive_threshold = 0.015
cy.max_bounces = 5; cy.diffuse_bounces = 3; cy.glossy_bounces = 3; cy.transmission_bounces = 0; cy.sample_clamp_indirect = 6.0; cy.caustics_reflective = False; cy.caustics_refractive = False
sc.render.threads_mode = "FIXED"; sc.render.threads = 3
cy.use_denoising = True; cy.denoiser = "OPENIMAGEDENOISE"
sc.view_layers[0].use_pass_normal = True; sc.use_nodes = True; nt = sc.node_tree; nt.nodes.clear()
rl = nt.nodes.new("CompositorNodeRLayers"); cp = nt.nodes.new("CompositorNodeComposite"); ao = nt.nodes.new("CompositorNodeAlphaOver"); bgc = nt.nodes.new("CompositorNodeRGB")
bgc.outputs[0].default_value = (1.72, 1.60, 1.22, 1.0)   # sau AgX ra đúng #D6D3CC (đo ở render_scene.py)
bl = nt.nodes.new("CompositorNodeBilateralblur"); bl.iterations = 3; bl.sigma_color = 0.10; bl.sigma_space = 2.5
nt.links.new(rl.outputs["Image"], bl.inputs["Image"]); nt.links.new(rl.outputs["Normal"], bl.inputs["Determinator"])
nt.links.new(bgc.outputs[0], ao.inputs[1]); nt.links.new(bl.outputs["Image"], ao.inputs[2]); nt.links.new(ao.outputs[0], cp.inputs["Image"])
sc.render.image_settings.file_format = "PNG"; sc.render.image_settings.color_depth = "8"; sc.render.image_settings.color_mode = "RGB"; sc.render.filepath = OUT
bpy.ops.render.render(write_still=True); print("DA GHI", OUT, flush=True)
