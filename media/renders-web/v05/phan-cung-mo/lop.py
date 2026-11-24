"""Render 4 lớp 'phần cứng mở' Faibot M1 từ m1.glb: cùng camera, cùng ánh sáng, nhìn thẳng phía trước.
Chạy: blender -b -P lop.py -- DIR OUTDIR [--samples N] [--pose straight]
Tạo: m1-lop-{1..4}.png (nền trong suốt) và m1-lop-{1..4}-nen.png (nền #F7F7F7)."""
import bpy, sys, os, math, json
import numpy as np
from mathutils import Vector, Quaternion
argv = sys.argv[sys.argv.index("--") + 1:]
DIR, OUTD = argv[0], argv[1]
opt = lambda k, d: type(d)(argv[argv.index(k) + 1]) if k in argv else d
SAMPLES, POSE = opt("--samples", 64), opt("--pose", "straight")
RES = (opt("--w", 1600), opt("--h", 2000))
# Ánh xạ vật liệu GLB -> lớp (node lá đặt tên <cum>.<node_khớp>.<vật liệu>).
# in 3D (trắng + xanh navy): ivory, navy. Còn lại là khung/servo/điện: alu, steel, hub (moay-ơ bánh), rubber (lốp),
# servo (còi servo), servo_body (thân servo, extras.servo), ivory_hi (khoang điện tử power), pad.
PRINTED = {"ivory", "navy"}
BODY = {"ivory"}   # lớp 2: thân in trắng; navy (tay xanh) thêm ở lớp 3 với alpha 0.3
def layer_of(o):
    m = o.data.materials[0].name.split(".")[0] if o.data.materials else ""
    return "printed" if m in PRINTED else "frame"
def lin(h):
    h = h.lstrip("#"); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return [(x / 12.92 if x <= .04045 else ((x + .055) / 1.055) ** 2.4) for x in c] + [1]
bpy.ops.wm.read_factory_settings(use_empty=True); sc = bpy.context.scene
bpy.ops.import_scene.gltf(filepath=os.path.join(DIR, "m1.glb"))
J = json.load(open(os.path.join(DIR, "joints.json"))); q = dict(J["poses"][POSE]["q"])
for kv in (argv[argv.index("--q") + 1].split(",") if "--q" in argv else []):
    k_, v_ = kv.split("="); q[k_] = float(v_)
conv = lambda a: Vector((a[0], -a[2], a[1]))
obj = {o.name: o for o in bpy.data.objects}
for k, j in J["joints"].items():
    v = q.get(k, 0.0); o = obj[j["node"]]; ax = conv(j["axis"])
    if j["type"] == "revolute":
        v = v / 100.0 * j.get("open_rad", 0) if k.endswith("gripper") else v
        o.rotation_mode = "QUATERNION"; o.rotation_quaternion = o.rotation_quaternion @ Quaternion(ax, v)
    else:
        d = v if k == "lift" else 0.0; o.location = o.location + ax * d
rot = bpy.data.objects.new("rot", None); sc.collection.objects.link(rot); rot.rotation_euler = (0, 0, -math.pi / 2)
for o in list(bpy.data.objects):
    if o is not rot and o.parent is None: o.parent = rot
bpy.context.view_layer.update()
mesh = [o for o in bpy.data.objects if o.type == "MESH"]
LAY = {o.name: layer_of(o) for o in mesh}
cnt = {"frame": 0, "printed": 0}; sv = 0; by_mat = {}
for o in mesh:
    cnt[LAY[o.name]] += 1; sv += ".servo_body." in o.name
    k = o.data.materials[0].name.split(".")[0] + "->" + LAY[o.name]; by_mat[k] = by_mat.get(k, 0) + 1
print("LAYER_COUNT nodes", cnt, "servo_body", sv, "by_material", by_mat, flush=True)
corners = [o.matrix_world @ Vector(c) for o in mesh for c in o.bound_box]
lo = Vector((min(p.x for p in corners), min(p.y for p in corners), min(p.z for p in corners)))
hi = Vector((max(p.x for p in corners), max(p.y for p in corners), max(p.z for p in corners)))
print("bbox", [round(x, 3) for x in lo], [round(x, 3) for x in hi], "pose", POSE, flush=True)
print("CHIEU_CAO_TOI_DA_m", round(hi.z - lo.z, 4), "lift", q.get("lift"), flush=True)
if "--bbox" in argv: sys.exit(0)
cen = sum(corners, Vector()) / len(corners); zf = lo.z
w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True; bg = w.node_tree.nodes["Background"]; bg.inputs[0].default_value = lin("#e8e6e0"); bg.inputs[1].default_value = 0.7
sc.render.film_transparent = True
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
sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage = RES[0], RES[1], 100
from bpy_extras.object_utils import world_to_camera_view
cam.lens = 85; cam.sensor_width = 36
ct.location = cen; dirv = Vector((1, 0, 0)); dist = 6.0; FILL = 0.92   # camera trên +X = chính diện (robot nhìn về +X sau xoay)
for it in range(10):
    co.location = ct.location + dirv * dist; co.location.z = ct.location.z; bpy.context.view_layer.update()
    pts = [world_to_camera_view(sc, co, p) for p in corners]
    x0, x1, y0, y1 = min(p.x for p in pts), max(p.x for p in pts), min(p.y for p in pts), max(p.y for p in pts)
    dist *= max((y1 - y0) / FILL, (x1 - x0) / FILL)
    mx, my = (x0 + x1) / 2 - 0.5, (y0 + y1) / 2 - 0.5
    mw = co.matrix_world; right, up = mw.to_3x3() @ Vector((1, 0, 0)), mw.to_3x3() @ Vector((0, 1, 0))
    wv_x = (co.location - ct.location).length * 36.0 / cam.lens; wv_y = wv_x * RES[1] / RES[0]
    ct.location += right * mx * wv_x + up * my * wv_y
co.location = ct.location + dirv * dist; co.location.z = ct.location.z; bpy.context.view_layer.update()
sc.render.engine = "CYCLES"; cy = sc.cycles; cy.device = "CPU"; cy.samples = SAMPLES; cy.use_adaptive_sampling = True; cy.adaptive_threshold = 0.015
cy.max_bounces = 6; cy.diffuse_bounces = 3; cy.glossy_bounces = 3; cy.transmission_bounces = 0; cy.transparent_max_bounces = 16
cy.sample_clamp_indirect = 6.0; cy.caustics_reflective = False; cy.caustics_refractive = False
sc.render.threads_mode = "FIXED"; sc.render.threads = 4
cy.use_denoising = True; cy.denoiser = "OPENIMAGEDENOISE"
sc.view_settings.view_transform = "Standard"; sc.view_settings.look = "None"
sc.render.image_settings.file_format = "PNG"; sc.render.image_settings.color_depth = "8"; sc.render.image_settings.color_mode = "RGBA"
mats = {m.name.split(".")[0]: m for m in bpy.data.materials}
def set_alpha(a):
    for n in PRINTED:
        if n in mats: mats[n].node_tree.nodes["Principled BSDF"].inputs["Alpha"].default_value = a
NAMES = {1: "m1-lop-1-khung", 2: "m1-lop-2-than", 3: "m1-lop-3-tay-trong-suot", 4: "m1-lop-4-hoan-chinh"}
def to_srgb(x): return np.where(x <= 0.0031308, x * 12.92, 1.055 * np.power(np.clip(x, 1e-9, None), 1 / 2.4) - 0.055)
for L in (1, 2, 3, 4):
    for o in mesh:
        mn = o.data.materials[0].name.split(".")[0]
        o.hide_render = (LAY[o.name] == "printed") and ((L == 1) or (L == 2 and mn not in BODY))
    set_alpha(1.0)
    if L == 3: mats["navy"].node_tree.nodes["Principled BSDF"].inputs["Alpha"].default_value = 0.3
    p = os.path.join(OUTD, NAMES[L] + ".png"); sc.render.filepath = p
    bpy.ops.render.render(write_still=True); print("DA GHI", p, flush=True)
    im = bpy.data.images.load(p)
    # nạp lại PNG thành mảng; 'pixels' ở không gian màu của ảnh (sRGB, straight alpha). Ghép lên #F7F7F7 trong sRGB.
    im.colorspace_settings.name = "Non-Color"; im.alpha_mode = "STRAIGHT"
    a = np.array(im.pixels[:], dtype=np.float32).reshape(RES[1], RES[0], 4); al = a[..., 3:4]
    out = a[..., :3] * al + (0xF7 / 255.0) * (1 - al)
    nm = bpy.data.images.new("nen%d" % L, RES[0], RES[1], alpha=False); nm.colorspace_settings.name = "Non-Color"
    nm.pixels.foreach_set(np.concatenate([out, np.ones_like(al)], axis=2).reshape(-1))
    nm.filepath_raw = os.path.join(OUTD, NAMES[L] + "-nen.png"); nm.file_format = "PNG"; nm.save()
    print("DA GHI nen", nm.filepath_raw, flush=True)
