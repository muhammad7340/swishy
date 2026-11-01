"""Blender: turn a ffish.asia CC0 fish scan (.glb) into Gazebo parts.

Run by scripts/make_fish_models.py --from-scans DIR. Per photographed
texture: one OBJ (+X forward, +Z up, centred, scaled to LENGTH m,
decimated) and one 512 px texture (re-encoded as JPEG afterwards).

    blender -b --factory-startup --python fish_scan_to_parts.py -- SCAN.glb OUT_DIR LENGTH TARGET_FACES
"""
import bpy, sys, os, math, mathutils
src, out, length, target = sys.argv[sys.argv.index("--") + 1:]
length, target = float(length), int(target)
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=src)
for o in list(bpy.data.objects):
    if o.type != "MESH" or len(o.data.polygons) < 100:   # the scale cube
        bpy.data.objects.remove(o)
parts = [o for o in bpy.data.objects]
bpy.ops.object.select_all(action="DESELECT")
for o in parts: o.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
bpy.ops.object.join()
obj = bpy.context.view_layer.objects.active
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
# head is at -X in the scans: turn to +X, centre on the bounding box, scale
bb = [mathutils.Vector(c) for c in obj.bound_box]
mn = mathutils.Vector([min(v[i] for v in bb) for i in range(3)]); mx = mathutils.Vector([max(v[i] for v in bb) for i in range(3)])
s = length / (mx.x - mn.x)
obj.data.transform(mathutils.Matrix.Translation(-(mn + mx) / 2))
obj.data.transform(mathutils.Matrix.Rotation(math.pi, 4, "Z"))
obj.data.transform(mathutils.Matrix.Scale(s, 4))
# weld the scan's texture-atlas chunks, then decimate the whole fish at once
bpy.ops.object.mode_set(mode="EDIT"); bpy.ops.mesh.select_all(action="SELECT")
bpy.ops.mesh.remove_doubles(threshold=length * 2e-4)
bpy.ops.object.mode_set(mode="OBJECT")
tris = sum(len(p.vertices) - 2 for p in obj.data.polygons)
mod = obj.modifiers.new("dec", "DECIMATE"); mod.ratio = min(1.0, target / tris); mod.delimit = {"UV"}   # keep the photo texture intact across UV seams
bpy.ops.object.modifier_apply(modifier="dec")
bpy.ops.object.shade_smooth()
bpy.ops.object.mode_set(mode="EDIT"); bpy.ops.mesh.separate(type="MATERIAL"); bpy.ops.object.mode_set(mode="OBJECT")
os.makedirs(f"{out}/meshes", exist_ok=True); os.makedirs(f"{out}/materials/textures", exist_ok=True)
k = 0
for o in sorted(bpy.data.objects, key=lambda o: o.name):
    if o.type != "MESH" or not o.data.polygons: continue
    img = next(n.image for n in o.data.materials[0].node_tree.nodes if n.type == "TEX_IMAGE" and n.image)
    img.scale(512, 512)
    img.filepath_raw = f"{out}/materials/textures/part{k}.jpg"; img.file_format = "JPEG"
    bpy.context.scene.render.image_settings.quality = 88
    img.save_render(img.filepath_raw)
    bpy.ops.object.select_all(action="DESELECT"); o.select_set(True); bpy.context.view_layer.objects.active = o
    bpy.ops.wm.obj_export(filepath=f"{out}/meshes/part{k}.obj", export_selected_objects=True,
                          export_materials=False, export_uv=True, export_normals=True,
                          forward_axis="Y", up_axis="Z", apply_modifiers=True)
    bbz = [v.co.z for v in o.data.vertices]
    print(f"PART {k}: {len(o.data.polygons)} faces")
    k += 1
bb = [v.co for o in bpy.data.objects if o.type == "MESH" for v in o.data.vertices]
print("SIZE", [round(max(v[i] for v in bb) - min(v[i] for v in bb), 3) for i in range(3)])
