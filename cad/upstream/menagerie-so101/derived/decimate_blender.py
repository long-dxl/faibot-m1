"""Rút gọn mesh STS3215 Menagerie cho sim/GLB (Faibotic, 23/11/2026). Chạy bằng Blender 4.0.2:
blender -b --factory-startup -P decimate_blender.py -- ../sts3215_03a_v1.stl sts3215_03a_v1_sim.stl 1500
Gộp đỉnh trùng (1e-7 m), Decimate kiểu collapse về số tam giác đích, ghi STL nhị phân (đơn vị giữ nguyên: m)."""
import bpy, sys, struct
a = sys.argv[sys.argv.index("--") + 1:]; src, dst, tgt = a[0], a[1], int(a[2])
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.stl_import(filepath=src)
o = bpy.context.selected_objects[0]; bpy.context.view_layer.objects.active = o
o.data = o.data.copy(); n0 = len(o.data.polygons)
bpy.ops.object.mode_set(mode="EDIT"); bpy.ops.mesh.select_all(action="SELECT"); bpy.ops.mesh.remove_doubles(threshold=1e-7); bpy.ops.object.mode_set(mode="OBJECT")
m = o.modifiers.new("d", "DECIMATE"); m.decimate_type = "COLLAPSE"; m.ratio = tgt / len(o.data.polygons); m.use_collapse_triangulate = True
bpy.ops.object.modifier_apply(modifier="d")
print("DEC", n0, "->", len(o.data.polygons))
me = o.data; me.calc_loop_triangles(); W = o.matrix_world
with open(dst, "wb") as fh:
    fh.write(b"decimated".ljust(80, b" ")); fh.write(struct.pack("<I", len(me.loop_triangles)))
    for t in me.loop_triangles:
        fh.write(struct.pack("<3f", 0, 0, 0))
        for vi in t.vertices: fh.write(struct.pack("<3f", *(W @ me.vertices[vi].co)))
        fh.write(b"\0\0")
