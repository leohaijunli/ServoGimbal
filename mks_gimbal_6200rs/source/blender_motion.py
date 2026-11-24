"""Standalone Blender-batch motion copy; never overwrite the static assembly."""
from pathlib import Path
import argparse
import ast
import json
import math
import re
import sys

import bpy
from mathutils import Matrix

parser = argparse.ArgumentParser(description=__doc__)
parser.parse_args(sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else [])
assert bpy.app.background, "Use in a separate Blender -b process, not someone's live scene"
CODE = Path(__file__).resolve().parent
ROOT = CODE.parent if CODE.name == "source" else CODE
OUT = ROOT/"output"
bpy.ops.wm.open_mainfile(filepath=str(OUT/"mks_gimbal.blend"))
scene = bpy.context.scene
scene.frame_set(1)
objects = [o for o in scene.objects if o.get("assembly_group") in ("fixed", "pan", "tilt")]
world = {o.name: o.matrix_world.copy() for o in objects}
mesh_data = {o.name: o.data for o in objects}
root = bpy.data.objects["Model_Root"]
assert "ANIM_Pan_Pivot" not in bpy.data.objects and "ANIM_Tilt_Pivot" not in bpy.data.objects
pan = bpy.data.objects.new("ANIM_Pan_Pivot", None)
tilt = bpy.data.objects.new("ANIM_Tilt_Pivot", None)
root.users_collection[0].objects.link(pan)
root.users_collection[0].objects.link(tilt)
pan.parent = root
pan.matrix_world = Matrix.Identity(4)
tilt.parent = pan
tilt.matrix_world = Matrix.Translation((0, 0, 140))
bpy.context.view_layer.update()
for obj in objects:
    group = obj.get("assembly_group")
    if group == "fixed":
        continue
    parent = tilt if group == "tilt" else pan
    obj.parent = parent
    obj.matrix_parent_inverse = parent.matrix_world.inverted()
    obj.matrix_world = world[obj.name]
keys = [(1, 0, 0), (24, 0, 0), (72, 45, 0), (120, -45, 0),
        (168, 0, 30), (216, 35, -25), (264, -35, 25), (288, 0, 0)]
for frame, p, t in keys:
    pan.rotation_euler = (0, 0, math.radians(p))
    tilt.rotation_euler = (math.radians(t), 0, 0)
    pan.keyframe_insert(data_path="rotation_euler", frame=frame)
    tilt.keyframe_insert(data_path="rotation_euler", frame=frame)
errors = []
for frame in (1, 288):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    errors.append(max(abs(o.matrix_world[i][j]-world[o.name][i][j])
                      for o in objects for i in range(4) for j in range(4)))
assert max(errors) < 0.0001 and all(o.data == mesh_data[o.name] for o in objects)


def pick(owner, field, requested):
    prop = owner.bl_rna.properties[field]
    identifiers = [item.identifier for item in prop.enum_items]
    if requested not in identifiers:
        identifiers = [item.identifier for item in prop.enum_items_static]
    if requested not in identifiers:
        # Same validated dynamic-enum discovery used in the static exporter.
        current = getattr(owner, field)
        try:
            setattr(owner, field, "__ENUM_DISCOVERY_INVALID__")
        except TypeError as error:
            match = re.search(r"not found in (\([^\n]*\))", str(error))
            if match is None:
                raise RuntimeError(f"Cannot discover {field}: {error}") from error
            identifiers = list(ast.literal_eval(match.group(1)))
        else:
            setattr(owner, field, current)
            raise RuntimeError("Enum discovery unexpectedly changed the property")
    return next(identifier for identifier in identifiers if identifier == requested)


render = scene.render
render.resolution_x, render.resolution_y, render.resolution_percentage = 1280, 720, 100
render.fps, render.fps_base = 24, 1
scene.frame_start, scene.frame_end = 1, 288
render.filepath = "//PanTilt_motion_preview.mp4"
render.image_settings.file_format = pick(render.image_settings, "file_format", "FFMPEG")
for field, value in (("format", "MPEG4"), ("codec", "H264"),
                     ("constant_rate_factor", "MEDIUM"), ("ffmpeg_preset", "GOOD"),
                     ("audio_codec", "NONE")):
    setattr(render.ffmpeg, field, pick(render.ffmpeg, field, value))
if scene.camera and scene.camera.data.type == "ORTHO":
    scene.camera.data.ortho_scale = 450
if hasattr(scene, "eevee") and "taa_render_samples" in scene.eevee.bl_rna.properties:
    scene.eevee.taa_render_samples = 32
scene.frame_set(1)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/"MKS_PanTilt_motion.blend"))
print("MOTION_COPY_READY", json.dumps({"frames": [1, 288], "fps": 24,
                                      "neutral_matrix_errors": errors,
                                      "mesh_data_changed": False}))
