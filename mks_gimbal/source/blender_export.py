"""Blender assembly, native reference meshes, GLB and explanatory renders.

Printed meshes are already validated by the build script. No bevel/decimation
is applied to them or to the MKS reference. Hardware is a visual envelope.
"""
from pathlib import Path
import hashlib
import json
import math
import struct
import sys

import bpy
import bmesh
from mathutils import Euler, Matrix, Vector

CODE = Path(__file__).resolve().parent
HERE = CODE.parent if CODE.name == "source" and (CODE.parent/"reference").is_dir() else CODE
OUT = HERE / "output"
DOC = json.loads((OUT / "manifest.json").read_text(encoding="utf-8"))
CHECKS = json.loads((OUT / "assembly_checks.json").read_text(encoding="utf-8"))
assert CHECKS["pass"], "Do not present a build with unresolved assembly checks"
assert CHECKS["manifest_sha256"] == hashlib.sha256((OUT/"manifest.json").read_bytes()).hexdigest()
REN = OUT / "renders"
REN.mkdir(exist_ok=True)

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
SC = bpy.context.scene
SC.unit_settings.system = "METRIC"
SC.unit_settings.scale_length = 0.001
SC.unit_settings.length_unit = "MILLIMETERS"

COLS = {}
for name in ("Printed Parts", "Real MKS Servos", "Unchanged Source Reference",
             "Hardware Envelopes", "HERO8 Fit Model", "Studio"):
    c = bpy.data.collections.new(name)
    SC.collection.children.link(c)
    COLS[name] = c


def move_collection(o, name):
    for c in list(o.users_collection):
        c.objects.unlink(o)
    COLS[name].objects.link(o)


def material(name, colour, metal=0, rough=0.5):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*colour, 1)
    bsdf.inputs["Metallic"].default_value = metal
    bsdf.inputs["Roughness"].default_value = rough
    mat.diffuse_color = (*colour, 1)
    return mat


MATS = {
    "base": material("PETG Base", (0.16, 0.23, 0.31)),
    "carrier": material("PETG Bearing Carrier", (0.32, 0.40, 0.48)),
    "frame": material("PETG Frame", (0.09, 0.25, 0.37)),
    "cradle": material("PETG Cradle", (0.13, 0.40, 0.39)),
    "hub": material("PA Spline Hub", (0.95, 0.38, 0.08)),
    "collar": material("PA Clamp Collar", (0.34, 0.24, 0.10)),
    "steel": material("Steel Hardware", (0.54, 0.58, 0.64), 0.8, 0.3),
    "brass": material("M2.5 Brass Insert", (0.72, 0.49, 0.16), 0.75, 0.3),
    "servo": material("MKS Red Aluminium", (0.50, 0.045, 0.055), 0.45, 0.3),
    "black": material("Camera Black", (0.045, 0.055, 0.070), 0.0, 0.5),
    "lens": material("Camera Lens", (0.055, 0.085, 0.105), 0.5, 0.16),
    "screen": material("Camera Front Screen", (0.17, 0.22, 0.22), 0.1, 0.3),
}

LABELS = {
    "01_Base": "01 底座", "02_Pan_Bearing_Carrier": "02 Pan 轴承座",
    "03_Pan_Lower_Cap": "03 下轴承压盖", "04_Pan_Upper_Cap": "04 上轴承压盖",
    "05_Pan_25T_Hub": "05 Pan 25T 驱动轴套", "06_Pan_Clamp_Collar": "06 Pan 夹紧套",
    "07_Pan_Inner_Spacer": "07 Pan 内圈隔套", "08_U_Frame": "08 转台/右支架",
    "09_HERO8_Cradle": "09 HERO8 笼架", "10_HERO8_Top_Clamp": "10 可拆顶压条",
    "11_Tilt_25T_Hub": "11 Tilt 25T 驱动轴套", "12_Tilt_Clamp_Collar": "12 Tilt 夹紧套",
    "13_Left_Bearing_Cap": "13 左轴承压盖", "14_Left_Inner_Spacer": "14 左轴内圈顶帽",
    "15_Removable_Left_Support": "15 可卸左支架", "16_Pan_Front_Mount_Bridge": "16 Pan 前耳安装桥",
    "MKS_Pan": "MKS HBL388 · Pan", "MKS_Tilt": "MKS HBL388 · Tilt",
    "MKS_Pan_Output": "Pan 真实25T输出轴", "MKS_Tilt_Output": "Tilt 真实25T输出轴",
    "HERO8": "GoPro HERO8 Black（包络模型）",
}

ROOT = bpy.data.objects.new("Model_Root", None)
COLS["Printed Parts"].objects.link(ROOT)
ROOT["units"] = "millimeter geometry; GLB root converts to meters"
RECORDS = []


def register(o, pid, category, group, explode):
    o["part_id"] = pid
    o["label"] = LABELS.get(pid, pid)
    o["category"] = category
    o["assembly_group"] = group
    o["explode_mm"] = list(explode)
    o["printable"] = category == "printed"
    o.parent = ROOT
    RECORDS.append({"obj": o, "matrix": o.matrix_world.copy(), "explode": Vector(explode)})
    return o


def set_mat(o, mat):
    o.data.materials.clear()
    o.data.materials.append(mat)


def smooth_display(o):
    # Shading only: actual vertices never change.
    for p in o.data.polygons:
        p.use_smooth = len(p.vertices) == 4


for pid, info in sorted(DOC["printed_parts"].items()):
    bpy.ops.wm.stl_import(filepath=str(OUT / "assembly_meshes" / (pid + ".stl")))
    o = bpy.context.object
    o.name = pid
    move_collection(o, "Printed Parts")
    key = "base"
    if "Carrier" in pid or "Cap" in pid or "Spacer" in pid:
        key = "carrier"
    if "U_Frame" in pid or "Left_Support" in pid:
        key = "frame"
    if "HERO8" in pid:
        key = "cradle"
    if "25T" in pid:
        key = "hub"
    if "Collar" in pid:
        key = "collar"
    set_mat(o, MATS[key])
    register(o, pid, "printed", info["group"], info["explode_mm"])

# Preserve a full, unmodified raw mesh in the Blender document. For animation,
# partition its original triangles into two DISPLAY-ONLY face sets. No vertex
# is moved, no face is decimated, and the raw reference remains available.
assert hashlib.sha256(Path(DOC["servo_source"]).read_bytes()).hexdigest() == DOC["servo_sha256"]
bpy.ops.wm.stl_import(filepath=DOC["servo_source"])
template = bpy.context.object
raw_mesh = template.data
raw_mesh.materials.clear()
raw_mesh.materials.append(MATS["servo"])
raw_mesh.materials.append(MATS["steel"])
template.name = "MKS_Unchanged_STL_Reference"
move_collection(template, "Unchanged Source Reference")
template.hide_render = True
template.hide_viewport = True
template["source_sha256"] = DOC["servo_sha256"]
template["note"] = "Original imported STL: unchanged coordinates, no modifiers, no independent scaling"
vertices = [tuple(v.co) for v in raw_mesh.vertices]
source_bytes = Path(DOC["servo_source"]).read_bytes()
triangle_count = struct.unpack_from("<I", source_bytes, 80)[0]
assert len(source_bytes) == 84+triangle_count*50, "Expected supplied binary STL"
source_triangles = []
for i in range(triangle_count):
    values = struct.unpack_from("<12fH", source_bytes, 84+50*i)
    source_triangles.append(tuple(sorted(tuple(values[j:j+3]) for j in (3, 6, 9))))
imported_triangles = [tuple(sorted(vertices[v] for v in p.vertices)) for p in raw_mesh.polygons]
assert sorted(source_triangles) == sorted(imported_triangles), "Blender import changed source surface geometry"
rotor_ids = {p.index for p in raw_mesh.polygons
             if any(raw_mesh.vertices[v].co.y > 20.40001 for v in p.vertices)}
assert rotor_ids, "Missing actual output shaft faces"
display_meshes = {}
for key, face_ids, mat in (("Stator", set(range(len(raw_mesh.polygons)))-rotor_ids, MATS["servo"]),
                          ("Output", rotor_ids, MATS["steel"])):
    mesh = bpy.data.meshes.new("MKS_Original_Faces_"+key)
    mesh.from_pydata(vertices, [], [tuple(raw_mesh.polygons[i].vertices) for i in sorted(face_ids)])
    mesh.materials.append(mat)
    mesh.update()
    display_meshes[key] = mesh
assert sum(len(me.polygons) for me in display_meshes.values()) == len(raw_mesh.polygons)
for pid, placement in DOC["servo_placements"].items():
    o = bpy.data.objects.new(pid, display_meshes["Stator"])
    COLS["Real MKS Servos"].objects.link(o)
    R = Matrix(placement["rotation_matrix"]).to_4x4()
    o.matrix_world = Matrix.Translation(placement["translation"]) @ R
    displacement = (-100, 0, 55) if pid == "MKS_Pan" else placement["explode_mm"]
    register(o, pid, "servo", placement["group"], displacement)
    o["geometry_note"] = "Original STL faces; display-only stator/rotor partition. Full raw mesh preserved in Blender."
    rotor = bpy.data.objects.new(pid+"_Output", display_meshes["Output"])
    COLS["Real MKS Servos"].objects.link(rotor)
    rotor.matrix_world = Matrix.Translation(placement["translation"]) @ R
    register(rotor, pid+"_Output", "servo_rotor", "pan" if pid == "MKS_Pan" else "tilt", displacement)
    rotor["geometry_note"] = "Unmodified original triangle subset; follows hub rotation about exact CAD shaft axis"


def ring_data(name, outer, inner, h, n=48, hex_af=None):
    verts, faces = [], []
    for r, z in ((outer/2, 0), (inner/2, 0), (outer/2, h), (inner/2, h)):
        for i in range(n):
            theta = math.tau*i/n
            rr = r
            if hex_af is not None and r == outer/2:
                rr = hex_af/2/max(math.cos(theta-math.tau*k/6) for k in range(6))
            verts.append((rr*math.cos(theta), rr*math.sin(theta), z))
    for i in range(n):
        j = (i+1) % n
        faces += [(i, j, 2*n+j, 2*n+i),
                  (n+j, n+i, 3*n+i, 3*n+j),
                  (j, i, n+i, n+j),
                  (2*n+i, 2*n+j, 3*n+j, 3*n+i)]
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    return me


def screw_data(diameter, length, head_diameter, head_height, n=32):
    rh, hh = head_diameter/2, head_height
    stages = [(rh, -hh), (rh, 0), (diameter/2, 0), (diameter/2, length)]
    verts = [(r*math.cos(math.tau*i/n), r*math.sin(math.tau*i/n), z)
             for r, z in stages for i in range(n)]
    faces = [tuple(reversed(range(n))), tuple(range(3*n, 4*n))]
    for level in range(3):
        for i in range(n):
            j = (i+1) % n
            faces.append((level*n+i, level*n+j, (level+1)*n+j, (level+1)*n+i))
    me = bpy.data.meshes.new(f"Screw_M{diameter}x{length}")
    me.from_pydata(verts, [], faces)
    me.update()
    return me


ROTS = {"+Z": (0, 0, 0), "-Z": (math.pi, 0, 0),
        "+X": (0, math.pi/2, 0), "-X": (0, -math.pi/2, 0),
        "+Y": (-math.pi/2, 0, 0), "-Y": (math.pi/2, 0, 0)}


def hw_object(name, data, loc, direction, mat, group, explode):
    o = bpy.data.objects.new(name, data)
    COLS["Hardware Envelopes"].objects.link(o)
    if not data.materials:
        data.materials.append(mat)
    o.matrix_world = Matrix.Translation(loc) @ Euler(ROTS[direction]).to_matrix().to_4x4()
    smooth_display(o)
    register(o, name, "hardware", group, explode)
    o["geometry_note"] = "Hardware envelope: screw threads, bearing balls and knurls simplified"
    return o


brass = ring_data("M2.5_RX_Insert_Envelope", 4.6, 2.05, 5.7)
for i, ins in enumerate(DOC["inserts"], start=1):
    info = DOC["printed_parts"][ins["part"]]
    hw_object(f"Insert_{i:02d}", brass, ins["position"], ins["direction"], MATS["brass"],
              info["group"], info["explode_mm"])

screw_cache = {}
for i, spec in enumerate(DOC["screws"], start=1):
    key = spec["thread_mm"], spec["length_mm"], spec["head_D_mm"], spec["head_H_mm"]
    if key not in screw_cache:
        screw_cache[key] = screw_data(*key)
    hw_object(f"Screw_{i:02d}", screw_cache[key], spec["bearing_surface"], spec["direction"],
              MATS["steel"], spec["group"], spec["explode_mm"])
    if spec["washer_mm"]:
        me = ring_data(f"Washer_{i:02d}", spec["washer_OD_mm"], spec["washer_ID_mm"],
                       spec["washer_mm"])
        hw_object(f"Washer_{i:02d}", me, spec["bearing_surface"], spec["direction"],
                  MATS["steel"], spec["group"], spec["explode_mm"])

for spec in DOC["hardware"]:
    me = ring_data(spec["name"], spec["od"], spec["id"], spec["length"], n=64)
    hw_object(spec["name"], me, spec["position"], "+"+spec["axis"], MATS["steel"],
              spec["group"], spec["explode_mm"])

nut_mesh = ring_data("M3_DIN934_Envelope", 5.5*2/math.sqrt(3), 3.2, 2.4, n=48, hex_af=5.5)
for i, spec in enumerate(DOC["nuts"], 1):
    hw_object(f"M3_Nut_{i:02d}", nut_mesh, spec["position"], spec["direction"], MATS["steel"],
              spec["group"], spec["explode_mm"])

# Fit model keeps the whole camera inside the official 66.3 x 48.6 x 28.4 envelope.
GP = DOC["go_pro"]
gw, gh, gd = GP["size_mm"]
centre = Vector(GP["centre"])
camera_root = bpy.data.objects.new("HERO8_Fit_Model", None)
COLS["HERO8 Fit Model"].objects.link(camera_root)
register(camera_root, "HERO8", "payload", "tilt", (0, 0, 200))


def camera_box(name, dimensions, location, mat, bevel=0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    o = bpy.context.object
    o.name = name
    o.scale = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        md = o.modifiers.new("Visual rounded corners", "BEVEL")
        md.width = bevel
        md.segments = 3
        bpy.ops.object.modifier_apply(modifier=md.name)
    move_collection(o, "HERO8 Fit Model")
    set_mat(o, mat)
    o.parent = camera_root
    return o


body = camera_box("HERO8_Approx_Body", (gw, gd-4, gh), (0, 2, centre.z), MATS["black"], 1.6)
body["note"] = "Visual details are approximate; fit uses the official maximum-size envelope"
camera_box("HERO8_Approx_Lens_Cover", (25, 4, 25), (17, -gd/2+2, centre.z+7), MATS["black"], 1.8)
bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=8, depth=0.3,
                                   location=(17, -gd/2+0.15, centre.z+7),
                                   rotation=(math.pi/2, 0, 0))
o = bpy.context.object
o.name = "HERO8_Approx_Lens"
move_collection(o, "HERO8 Fit Model")
set_mat(o, MATS["lens"])
o.parent = camera_root
camera_box("HERO8_Approx_Front_Screen", (17, 0.15, 14), (-17, -gd/2+3.9, centre.z+6), MATS["screen"], 0.03)

# Compressed foam envelopes fill the intentional camera clearances. Uncompressed
# pads should be slightly thicker; verify grip without loading camera buttons.
foam_mat = material("Compressed Foam", (0.08, 0.09, 0.10), rough=0.9)
foam_specs = [
    ("Bottom", (54, 20, 0.5), (0, 0, centre.z-gh/2-0.25), (0, 0, 145)),
    ("Back", (48, 0.5, 32), (0, gd/2+0.25, centre.z), (0, 0, 145)),
    ("Left", (0.5, 2.0, 34), (-gw/2-0.25, 13.2, centre.z), (0, 0, 145)),
    ("Right", (0.5, 2.0, 34), (gw/2+0.25, 13.2, centre.z), (0, 0, 145)),
    ("TopLeft", (20, 6, 1.5), (-10, 5, centre.z+gh/2+0.75), (0, 0, 235)),
]
for name, dims, loc, displacement in foam_specs:
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.object
    o.name = "Foam_"+name
    o.scale = dims
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    move_collection(o, "Hardware Envelopes")
    set_mat(o, foam_mat)
    register(o, o.name, "hardware", "tilt", displacement)
    o["geometry_note"] = "Compressed foam envelope, not a printable rigid part"

bpy.context.view_layer.update()
# Re-record after parent/depsgraph updates; references preserve exact poses.
for rec in RECORDS:
    rec["matrix"] = rec["obj"].matrix_world.copy()

# GLB explicitly in metres. Blender source stays mm for fabrication.
ROOT.scale = (0.001, 0.001, 0.001)
bpy.context.view_layer.update()
bpy.ops.object.select_all(action="DESELECT")
ROOT.select_set(True)
for rec in RECORDS:
    rec["obj"].select_set(True)
for o in COLS["HERO8 Fit Model"].objects:
    o.select_set(True)
bpy.ops.export_scene.gltf(filepath=str(OUT / "mks_gimbal_assembly.glb"),
                          export_format="GLB", use_selection=True, export_yup=True,
                          export_extras=True, export_apply=False)
ROOT.scale = (1, 1, 1)
bpy.context.view_layer.update()

# Studio, kept OUT of the GLB.
SC.render.engine = "BLENDER_EEVEE_NEXT"
SC.render.resolution_x = 1600
SC.render.resolution_y = 1300
SC.render.resolution_percentage = 100
SC.render.image_settings.file_format = "PNG"
SC.view_settings.look = "AgX - Medium High Contrast"
SC.view_settings.exposure = 0.0
world = bpy.data.worlds.new("Neutral Studio")
world.use_nodes = True
world.node_tree.nodes["Background"].inputs[0].default_value = (0.72, 0.76, 0.82, 1)
world.node_tree.nodes["Background"].inputs[1].default_value = 0.5
SC.world = world
for i, (rotation, energy) in enumerate((((50, 0, 30), 3.5), ((65, 0, -125), 1.5), ((15, 0, 180), 0.8))):
    data = bpy.data.lights.new(f"Studio Sun {i}", "SUN")
    data.energy = energy
    data.angle = math.radians(12)
    o = bpy.data.objects.new(data.name, data)
    COLS["Studio"].objects.link(o)
    o.rotation_euler = [math.radians(v) for v in rotation]
bpy.ops.mesh.primitive_plane_add(size=1400, location=(0, 0, -0.15))
ground = bpy.context.object
ground.name = "Studio_Ground"
move_collection(ground, "Studio")
set_mat(ground, material("Studio Ground", (0.79, 0.82, 0.86), rough=0.8))
cd = bpy.data.cameras.new("Assembly Camera")
cd.type = "ORTHO"
cam = bpy.data.objects.new("Assembly Camera", cd)
COLS["Studio"].objects.link(cam)
SC.camera = cam


def view(location, target, scale):
    cam.location = location
    cam.rotation_euler = (Vector(target)-Vector(location)).to_track_quat("-Z", "Y").to_euler()
    cd.ortho_scale = scale
    cd.clip_start, cd.clip_end = 0.01, 3000
    bpy.context.view_layer.update()


def render(name, loc, target, scale):
    view(loc, target, scale)
    SC.render.filepath = str(REN / (name + ".png"))
    bpy.ops.render.render(write_still=True)
    print("RENDERED", name, flush=True)


def explosion(amount):
    for rec in RECORDS:
        rec["obj"].matrix_world = Matrix.Translation(rec["explode"]*amount) @ rec["matrix"]
    bpy.context.view_layer.update()


render("assembly_iso", (300, -370, 280), (0, 0, 85), 300)
render("assembly_front", (0, -450, 91), (0, 0, 91), 245)
render("assembly_side", (450, 0, 95), (0, 0, 95), 245)
view((300, -370, 280), (0, 0, 85), 300)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT / "mks_gimbal.blend"))

# Exploded presentation retains exact meshes and only changes rigid poses.
explosion(1.0)
render("exploded_iso", (430, -510, 455), (0, 0, 200), 600)
render("exploded_front", (0, -600, 200), (0, 0, 200), 545)
view((430, -510, 455), (0, 0, 200), 600)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT / "mks_gimbal_exploded.blend"))
explosion(0)

# Internal view: only hides shells. Never exported as a printable part.
hidden = [bpy.data.objects[n] for n in ("01_Base", "02_Pan_Bearing_Carrier")]
for o in hidden:
    o.hide_render = True
render("pan_internal_view", (230, -250, 180), (0, 0, 53), 170)
for o in hidden:
    o.hide_render = False

print("BLENDER ASSEMBLY / EXPLODED / GLB / RENDERS OK", flush=True)
report = {"blender_version": bpy.app.version_string, "geometry_units": "mm",
          "GLB_coordinate_convention": "glTF Y-up, metre world dimensions via 0.001 root",
          "reference_sha256": DOC["servo_sha256"], "raw_reference_triangles": len(raw_mesh.polygons),
          "raw_reference_vertices": len(raw_mesh.vertices),
          "raw_surface_triangles_exact_match": True,
          "display_stator_triangles": len(display_meshes["Stator"].polygons),
          "display_rotor_triangles": len(display_meshes["Output"].polygons),
          "reference_vertices_changed": False, "printed_parts": len(DOC["printed_parts"]),
          "hardware_note": "Non-printable envelopes; threads, rolling elements and knurls simplified",
          "assembly_checks_sha256": hashlib.sha256((OUT/"assembly_checks.json").read_bytes()).hexdigest()}
(OUT/"blender_export_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
