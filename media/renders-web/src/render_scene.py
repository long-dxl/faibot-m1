"""Dựng cảnh Faibot M1 từ mesh lắp ráp (cad/faibotic/assembly_full.py --mesh) và render Cycles CPU.
Chạy: blender -b -P render_scene.py -- <thư_mục_mesh> <view> <ra.png> [--samples N] [--scale k]
view: hero | chinh-dien | canh | sau | can-vai-kep | thuoc (chính diện/cạnh trực giao cho ảnh thước) ... Màu theo ui-tokens (ngà, than, navy), nhôm anodized xám nhạt. Không cam."""
import bpy, bmesh, sys, os, math, json
from mathutils import Vector, Matrix
argv = sys.argv[sys.argv.index("--") + 1:]
MESH, VIEW, OUT = argv[0], argv[1], argv[2]
opt = lambda k, d: type(d)(argv[argv.index(k) + 1]) if k in argv else d
SAMPLES, SCALE = opt("--samples", 48), opt("--scale", 1.0)
HI, SUBJ_RX = opt("--hi", ""), opt("--subject-rx", "")    # --hi REGEX: chi tiết khớp đổi sang cam #C2410C (ảnh lắp ráp); --subject-rx REGEX: chỉ khung theo các chi tiết này
import re
# --sel FILE.json: {"hi": [tên...], "prod": [tên...], "hide": [tên...], "subject": [tên...]}. Chi tiết trong hi cam #C2410C, trong prod giữ màu sản phẩm
#   (ngà/navy như GLB), còn lại xám nhạt (mờ dần); hide bỏ khỏi cảnh; subject: chi tiết dùng để lấy khung.
# --el ĐỘ: góc nâng camera (thay cho h/dz của view); --az ĐỘ: ghi đè phương vị; --alpha: ghi PNG RGBA không nền (ghép nền ở Pillow);
# --floor-z M: độ cao sàn studio; --markers FILE.json: chiếu điểm 3D ra pixel -> <ra.png>.markers.json
SEL = json.load(open(opt("--sel", ""))) if "--sel" in argv else None
EL, AZ, FLOORZ, MARK = opt("--el", 999.0), opt("--az", 999.0), opt("--floor-z", 0.0), opt("--markers", "")
ALPHA = "--alpha" in argv
def lin(h):
    h = h.lstrip("#"); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return [(x / 12.92 if x <= .04045 else ((x + .055) / 1.055) ** 2.4) for x in c] + [1]
bpy.ops.wm.read_factory_settings(use_empty=True); sc = bpy.context.scene
def mat(name, hexc, rough, metal=0.0, coat=0.0, aniso=0.0):
    m = bpy.data.materials.new(name); m.use_nodes = True; b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = lin(hexc); b.inputs["Roughness"].default_value = rough; b.inputs["Metallic"].default_value = metal
    b.inputs["Coat Weight"].default_value = coat; b.inputs["Coat Roughness"].default_value = 0.25
    return m
M = {"ivory": mat("ivory", "#f1efe8", .5, 0, .15), "ivory_hi": mat("ivory_hi", "#f6f5f1", .45, 0, .2), "navy": mat("navy", "#1e3a5f", .42, 0, .25),
     "alu": mat("alu", "#b3b7bd", .40, .85), "steel": mat("steel", "#8d939b", .35, 1.0), "servo": mat("servo", "#1b1d21", .62), "servo_body": mat("servo_body", "#1a1a1a", .45, 0, .1), "pad": mat("pad", "#33363d", .75),
     "rubber": mat("rubber", "#26282d", .8), "hub": mat("hub", "#6d727a", .42, .9), "hi": mat("hi", "#c2410c", .45, 0, .2), "ghost": mat("ghost", "#dcdce0", .75)}
man = json.load(open(os.path.join(MESH, "manifest.json")))
objs = {}
# --skip TIEN_TO,...: bỏ chi tiết khỏi cảnh (không sửa CAD). Bộ ảnh 70a98f6 bỏ M1-CM-001_wrist_cam: camera cổ tay ở vị trí đề xuất, chưa có giá
# (M1-AR-009 chưa đặt được), cách vỏ J56 ~16,6 mm nên trên ảnh là vật lơ lửng.
SKIP = tuple(x for x in (argv[argv.index("--skip") + 1].split(",") if "--skip" in argv else []) if x)
for n, m in man:
    if n.startswith(SKIP) if SKIP else False: continue
    if SEL and n in SEL.get("hide", []): continue
    p = os.path.join(MESH, n + ".stl")
    bpy.ops.wm.stl_import(filepath=p); o = bpy.context.object; o.name = n
    bm = bmesh.new(); bm.from_mesh(o.data); bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-3); bm.to_mesh(o.data); bm.free()
    o.scale = (0.001,) * 3; o.data.materials.append(M["hi"] if (HI and re.search(HI, n)) or (SEL and n in SEL["hi"]) else M["ghost"] if (SEL and n not in SEL["prod"]) else M.get(m, M["alu"]))
    o.data.use_auto_smooth = True; o.data.auto_smooth_angle = math.radians(32)
    bpy.context.view_layer.objects.active = o; bpy.ops.object.shade_smooth()
    if m in ("ivory", "navy", "alu", "ivory_hi") and not n.startswith("M1-BS-010"):
        bv = o.modifiers.new("b", "BEVEL"); bv.width = 0.0006; bv.segments = 2; bv.limit_method = "ANGLE"
    objs[n] = o
bpy.context.view_layer.update()
# ---- phông studio: sàn + tường cong, giấy ngà nhạt
prof = [(0, 0)] + [(r, 0) for r in (2, 4, 7)] + [(7 + 2.5 * math.sin(a), 2.5 - 2.5 * math.cos(a)) for a in [i * math.pi / 2 / 12 for i in range(1, 13)]] + [(9.5, 8), (9.5, 14)]
NS = 120; V = []; F = []
for r, z in prof:
    for i in range(NS): a = 2 * math.pi * i / NS; V.append((r * math.cos(a), r * math.sin(a), z))
for j in range(len(prof) - 1):
    for i in range(NS): F.append((j * NS + i, j * NS + (i + 1) % NS, (j + 1) * NS + (i + 1) % NS, (j + 1) * NS + i))
me = bpy.data.meshes.new("cyc"); me.from_pydata(V, [], F); me.update(); cyc = bpy.data.objects.new("cyc", me); sc.collection.objects.link(cyc)
me.polygons.foreach_set("use_smooth", [True] * len(F)); cyc.data.materials.append(mat("floor", "#d6d3cc", .9)); cyc.is_shadow_catcher = True; cyc.location.z = FLOORZ; sc.render.film_transparent = True
w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True; bg = w.node_tree.nodes["Background"]; bg.inputs[0].default_value = lin("#d6d3cc"); bg.inputs[1].default_value = 0.55
tgt = bpy.data.objects.new("tgt", None); sc.collection.objects.link(tgt); tgt.location = (0.0, 0, 0.65)
def area(name, loc, size, energy, color=(1, 1, 1), shape="SQUARE"):
    d = bpy.data.lights.new(name, "AREA"); d.size = size; d.energy = energy; d.color = color; d.shape = shape
    o = bpy.data.objects.new(name, d); sc.collection.objects.link(o); o.location = loc
    c = o.constraints.new("TRACK_TO"); c.target = tgt; c.track_axis = "TRACK_NEGATIVE_Z"; c.up_axis = "UP_Y"
area("key", (3.4, 2.8, 3.6), 6.0, 150, (1.0, .985, .96)); area("fill", (3.2, -3.4, 2.2), 7.0, 80, (.95, .97, 1)); area("rim", (-3.4, 0.8, 2.6), 4.0, 120, (.96, .98, 1))
area("top", (0.3, 0.0, 4.6), 6.0, 60)
SCV = SCALE
cam = bpy.data.cameras.new("cam"); co = bpy.data.objects.new("cam", cam); sc.collection.objects.link(co); sc.camera = co
ct = bpy.data.objects.new("ct", None); sc.collection.objects.link(ct)
tc = co.constraints.new("TRACK_TO"); tc.target = ct; tc.track_axis = "TRACK_NEGATIVE_Z"; tc.up_axis = "UP_Y"
az = lambda deg, d, h: (d * math.cos(math.radians(deg)), d * math.sin(math.radians(deg)), h)
VIEWS = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "views.json")))
v = dict(VIEWS[VIEW])
if EL < 900: v["el"] = EL
if AZ < 900: v["az"] = AZ
sc.render.resolution_x = int(v["res"][0] * SCV); sc.render.resolution_y = int(v["res"][1] * SCV); sc.render.resolution_percentage = 100
from bpy_extras.object_utils import world_to_camera_view
subj = [o for n, o in objs.items() if (v.get("subject") == "wristL" and n.endswith("_L") and n.startswith(("OB_F_", "M1-AR-021"))) or (v.get("subject") == "wristL") is False and (v.get("subject") != "armL" or (n.endswith("_L") and not n.startswith(("M1-LF", "M1-BS")) or n.startswith("M1-LF-024") and n.endswith("_L")))]
if SUBJ_RX: subj = [o for n, o in objs.items() if re.search(SUBJ_RX, n)]
if SEL and SEL.get("subject"): subj = [objs[n] for n in SEL["subject"] if n in objs]
corners = [o.matrix_world @ Vector(c) for o in subj for c in o.bound_box]
cen = sum(corners, Vector()) / len(corners)
if v.get("ortho"):
    cam.type = "ORTHO"; cam.ortho_scale = v["ortho"]; co.location = v["loc"]; ct.location = v["tgt"]
else:
    cam.lens = v["lens"]; cam.sensor_width = 36
    ct.location = Vector(v.get("tgt", cen)) if "tgt" in v else cen
    ce = math.cos(math.radians(v["el"])) if "el" in v else 1.0
    dirv = Vector((ce * math.cos(math.radians(v["az"])), ce * math.sin(math.radians(v["az"])), math.sin(math.radians(v["el"])) if "el" in v else 0)); dist = v.get("d", 4.0)
    zc = lambda: (v["h"] if "h" in v else ct.location.z + v["dz"])
    for it in range(8):
        co.location = ct.location + dirv * dist
        if "el" not in v: co.location.z = zc()
        bpy.context.view_layer.update()
        pts = [world_to_camera_view(sc, co, p) for p in corners]
        x0, x1, y0, y1 = min(p.x for p in pts), max(p.x for p in pts), min(p.y for p in pts), max(p.y for p in pts)
        fw, fh = x1 - x0, y1 - y0
        k = max(fh / v["fill"], fw / (v["fill"] + 0.12))     # >1: quá to -> lùi ra
        dist *= k
        # dời đích để chủ thể ra giữa khung
        mx, my = (x0 + x1) / 2 - 0.5, (y0 + y1) / 2 - 0.5
        mw = co.matrix_world; right, up = mw.to_3x3() @ Vector((1, 0, 0)), mw.to_3x3() @ Vector((0, 1, 0))
        wv = 2 * (co.location - ct.location).length * (18.0 / v["lens"]) * (sc.render.resolution_x / sc.render.resolution_y if False else 1)
        wv_x = (co.location - ct.location).length * 36.0 / v["lens"]; wv_y = wv_x * sc.render.resolution_y / sc.render.resolution_x
        ct.location += right * mx * wv_x + up * my * wv_y
    co.location = ct.location + dirv * dist
    if "el" not in v: co.location.z = zc()
    bpy.context.view_layer.update()
    if v.get("dof"): cam.dof.use_dof = True; cam.dof.focus_distance = (co.location - ct.location).length; cam.dof.aperture_fstop = v["dof"]
sc.render.resolution_x = int(v["res"][0] * SCV); sc.render.resolution_y = int(v["res"][1] * SCV); sc.render.resolution_percentage = 100
sc.render.engine = "CYCLES"; cy = sc.cycles; cy.device = "CPU"; cy.samples = SAMPLES; cy.use_adaptive_sampling = True; cy.adaptive_threshold = 0.015
cy.max_bounces = 5; cy.diffuse_bounces = 3; cy.glossy_bounces = 3; cy.transmission_bounces = 0; cy.sample_clamp_indirect = 6.0; cy.caustics_reflective = False; cy.caustics_refractive = False
sc.render.threads_mode = "FIXED"; sc.render.threads = int(os.environ.get("THREADS", "2"))
cy.use_denoising = False   # bản Blender apt không có OpenImageDenoise: khử nhiễu bằng Bilateral Blur dẫn hướng Normal (compositor)
sc.view_layers[0].use_pass_normal = True; sc.use_nodes = True; nt = sc.node_tree; nt.nodes.clear()
rl = nt.nodes.new("CompositorNodeRLayers"); cp = nt.nodes.new("CompositorNodeComposite"); ao = nt.nodes.new("CompositorNodeAlphaOver"); bgc = nt.nodes.new("CompositorNodeRGB")
# nền ảnh: giá trị tuyến tính đã hiệu chỉnh để SAU view transform AgX mặc định ra đúng #D6D3CC (đo trên Blender 4.0.2: (1,72; 1,60; 1,22) -> 214,211,204);
# lin("#d6d3cc") qua AgX chỉ ra ~#B6B5B2 (QC renders-web M2/DK-R2). Ánh sáng, world, vật liệu robot giữ nguyên.
bgc.outputs[0].default_value = (1.72, 1.60, 1.22, 1.0)
last = rl.outputs["Image"]
if "--no-denoise" not in argv:
    bl = nt.nodes.new("CompositorNodeBilateralblur"); bl.iterations = 3; bl.sigma_color = 0.10; bl.sigma_space = 2.5
    nt.links.new(rl.outputs["Image"], bl.inputs["Image"]); nt.links.new(rl.outputs["Normal"], bl.inputs["Determinator"]); last = bl.outputs["Image"]
if ALPHA: nt.links.new(last, cp.inputs["Image"])
else: nt.links.new(bgc.outputs[0], ao.inputs[1]); nt.links.new(last, ao.inputs[2]); nt.links.new(ao.outputs[0], cp.inputs["Image"])
sc.render.image_settings.file_format = "PNG"; sc.render.image_settings.color_depth = "8"; sc.render.image_settings.color_mode = "RGBA" if ALPHA else "RGB"; sc.render.filepath = OUT
if MARK:
    mk = json.load(open(MARK)); res = {}
    for k, e in mk.items():
        if "xyz_mm" in e: pw = Vector(e["xyz_mm"]) / 1000
        else:
            o = objs[e["mesh"]]; cs = [o.matrix_world @ Vector(c) for c in o.bound_box]; pw = sum(cs, Vector()) / 8
        q = world_to_camera_view(sc, co, pw); res[k] = {"id": e.get("id"), "x": round(q.x * sc.render.resolution_x, 1), "y": round((1 - q.y) * sc.render.resolution_y, 1)}
    json.dump({"size": [sc.render.resolution_x, sc.render.resolution_y], "pos": res}, open(OUT + ".markers.json", "w"), indent=1)
if "--save-blend" in argv: bpy.ops.wm.save_as_mainfile(filepath=OUT.replace(".png", ".blend"))
bpy.ops.render.render(write_still=True); print("DA GHI", OUT)
