"""Cổng hình S1: render m1.glb (đúng tệp web dùng) ở tư thế trong joints.json, 3 góc (truoc, ben, 34), Cycles CPU, WebP q90.
Chạy: blender -b -P 15_render_glb.py -- WEB_DIR OUT_DIR [--pose ready] [--samples 32] [--res 1000 1250]
Đặt khớp theo joints.json: node khớp quay quanh trục local (hệ CAD) của nó; glTF Y-up -> Blender Z-up đổi trục local (x, y, z) -> (x, -z, y).
Màu lấy từ vật liệu PBR trong GLB. Nền #d6d3cc, đèn studio như media/renders-web/src/render_scene.py. Không cam."""
import bpy, sys, os, math, json
from mathutils import Vector, Quaternion
argv = sys.argv[sys.argv.index("--") + 1:]
WEB, OUT = argv[0], argv[1]
opt = lambda k, d: type(d)(argv[argv.index(k) + 1]) if k in argv else d
POSE, SAMPLES = opt("--pose", "ready"), opt("--samples", 32)
RES = (int(argv[argv.index("--res") + 1]), int(argv[argv.index("--res") + 2])) if "--res" in argv else (1000, 1250)
os.makedirs(OUT, exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True); sc = bpy.context.scene
bpy.ops.import_scene.gltf(filepath=os.path.join(WEB, "m1.glb"))
J = json.load(open(os.path.join(WEB, "joints.json"))); q = J["poses"][POSE]["q"]
conv = lambda a: Vector((a[0], -a[2], a[1]))
obj = {o.name: o for o in bpy.data.objects}
for k, j in J["joints"].items():
    v = q.get(k, 0.0); o = obj[j["node"]]; ax = conv(j["axis"])
    if j["type"] == "revolute":
        o.rotation_mode = "QUATERNION"; o.rotation_quaternion = o.rotation_quaternion @ Quaternion(ax, v)
    else:
        d = v if k == "lift" else v / 100.0 * j["travel_m"]; o.location = o.location + ax * d
        if "mimic_node" in j: ob = obj[j["mimic_node"]]; ob.location = ob.location + conv(j["mimic_axis"]) * d
bpy.context.view_layer.update()
mesh = [o for o in bpy.data.objects if o.type == "MESH"]
pts = [o.matrix_world @ Vector(c) for o in mesh for c in o.bound_box]
lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts))); hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
cen = (lo + hi) / 2; H = hi.z - lo.z
print("bbox", [round(x, 3) for x in lo], [round(x, 3) for x in hi])


def lin(h):
    h = h.lstrip("#"); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return [(x / 12.92 if x <= .04045 else ((x + .055) / 1.055) ** 2.4) for x in c] + [1]


# sàn bắt bóng + nền
bpy.ops.mesh.primitive_plane_add(size=20, location=(0, 0, lo.z)); fl = bpy.context.object; fl.is_shadow_catcher = True
w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True; bg = w.node_tree.nodes["Background"]; bg.inputs[0].default_value = lin("#d6d3cc"); bg.inputs[1].default_value = 0.6
tgt = bpy.data.objects.new("tgt", None); sc.collection.objects.link(tgt); tgt.location = cen


def area(name, loc, size, energy):
    d = bpy.data.lights.new(name, "AREA"); d.size = size; d.energy = energy
    o = bpy.data.objects.new(name, d); sc.collection.objects.link(o); o.location = loc
    c = o.constraints.new("TRACK_TO"); c.target = tgt; c.track_axis = "TRACK_NEGATIVE_Z"; c.up_axis = "UP_Y"


# glTF: robot nhìn về -Z_gl = +x CAD; Blender: +x CAD -> Blender? (X_gl = -y, Z_gl = -x) -> Blender (X_gl, -Z_gl, Y_gl) = (-y, x, z): trước robot = +Y Blender
area("key", (2.8, 3.4, 3.6), 6.0, 160); area("fill", (-3.4, 3.2, 2.2), 7.0, 80); area("rim", (0.8, -3.4, 2.6), 4.0, 120); area("top", (0, 0.3, 4.6), 6.0, 60)
cam = bpy.data.cameras.new("cam"); co = bpy.data.objects.new("cam", cam); sc.collection.objects.link(co); sc.camera = co; cam.lens = 70; cam.sensor_width = 36
tc = co.constraints.new("TRACK_TO"); tc.target = tgt; tc.track_axis = "TRACK_NEGATIVE_Z"; tc.up_axis = "UP_Y"
sc.render.resolution_x, sc.render.resolution_y = RES
sc.render.engine = "CYCLES"; cy = sc.cycles; cy.device = "CPU"; cy.samples = SAMPLES; cy.use_adaptive_sampling = True; cy.use_denoising = False
sc.render.threads_mode = "FIXED"; sc.render.threads = 3; sc.render.film_transparent = False
sc.render.image_settings.file_format = "WEBP"; sc.render.image_settings.quality = 90; sc.render.image_settings.color_mode = "RGB"
from bpy_extras.object_utils import world_to_camera_view
VIEWS = {"truoc": 90.0, "ben": 0.0, "34": 50.0}      # góc phương vị quanh z Blender; 90 = nhìn từ phía trước robot (+Y)
for name, az in VIEWS.items():
    d = Vector((math.cos(math.radians(az)), math.sin(math.radians(az)), 0.0)); dist = 4.0; h = 0.35 * H
    for _ in range(6):
        co.location = cen + d * dist + Vector((0, 0, h)); bpy.context.view_layer.update()
        p = [world_to_camera_view(sc, co, x) for x in pts]
        fw = max(x.x for x in p) - min(x.x for x in p); fh = max(x.y for x in p) - min(x.y for x in p)
        dist *= max(fw / 0.86, fh / 0.86)
    co.location = cen + d * dist + Vector((0, 0, h)); bpy.context.view_layer.update()
    sc.render.filepath = os.path.join(OUT, f"m1-pl-{name}.webp"); bpy.ops.render.render(write_still=True); print("DA GHI", sc.render.filepath, flush=True)
