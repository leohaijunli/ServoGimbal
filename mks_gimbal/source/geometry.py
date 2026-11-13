"""Native manifold geometry, explicit mm frames, no scene-side transforms."""
from pathlib import Path
import json
import math
import sys

import manifold3d as m
import numpy as np
import trimesh

CODE = Path(__file__).resolve().parent
HERE = CODE.parent if CODE.name == "source" and (CODE.parent/"reference").is_dir() else CODE
SKILL = Path(r"C:\Users\HLi\.config\opencode\skills\3d-print")
sys.path.insert(0, str(CODE/"lib" if (CODE/"lib").is_dir() else SKILL/"scripts"))
from generate_model import export_3mf, export_stl
from validate_mesh import validate_mesh
from build_fit_tests import mesh_islands


def box(w, d, h, x=0, y=0, z=0):
    return m.Manifold.cube([w, d, h], center=True).translate([x, y, z + h/2])


def cyl(r, h, x=0, y=0, z=0, axis="Z", segments=96):
    o = m.Manifold.cylinder(height=h, radius_low=r, circular_segments=segments)
    if axis == "X":
        o = o.rotate([0, 90, 0])
    elif axis == "Y":
        o = o.rotate([-90, 0, 0])
    return o.translate([x, y, z])


def roundplate(w, d, h, r, x=0, y=0, z=0):
    cs = m.CrossSection.square([w - 2*r, d - 2*r], center=True)
    cs = cs.offset(r, m.JoinType.Round, circular_segments=48)
    return m.Manifold.extrude(cs, height=h).translate([x, y, z])


def gusset_xz(points, y0, depth):
    p = np.array(points, dtype=float)
    area = np.sum(p[:, 0]*np.roll(p[:, 1], -1) - p[:, 1]*np.roll(p[:, 0], -1))
    if area < 0:
        p = p[::-1]
    return m.Manifold.extrude(m.CrossSection([p]), height=depth).rotate([90, 0, 0]).translate([0, y0+depth, 0])


def spline_cs(clearance=0.06, print_pose=False):
    evidence = json.loads((HERE / "reference/reference_evidence.json").read_text())
    assert evidence["spline"]["teeth_measured"] == 25
    p = np.array(evidence["spline"]["ordered_polygon"], dtype=float)
    if not print_pose:
        p[:, 1] *= -1  # source X-Z -> Pan X-Y, R_x(+90)
    area = np.sum(p[:, 0]*np.roll(p[:, 1], -1) - p[:, 1]*np.roll(p[:, 0], -1))
    if area < 0:
        p = p[::-1]
    cs = m.CrossSection([p]).simplify(0.003)
    return cs.offset(clearance, m.JoinType.Round, circular_segments=16).simplify(0.003)


def spline_tool(z, height, clearance=0.06):
    return m.Manifold.extrude(spline_cs(clearance), height=height).translate([0, 0, z])


def spline_lead(z, clearance=0.06):
    """0.25 mm lead, wide at the entry; the remaining teeth are unchanged."""
    cs = spline_cs(clearance).scale([1.06, 1.06])
    return m.Manifold.extrude(cs, height=0.25,
                             scale_top=[1/1.06, 1/1.06]).translate([0, 0, z])


def orient(solid, position, direction):
    rotations = {"+Z": [0, 0, 0], "-Z": [180, 0, 0],
                 "+X": [0, 90, 0], "-X": [0, -90, 0],
                 "+Y": [-90, 0, 0], "-Y": [90, 0, 0]}
    return solid.rotate(rotations[direction]).translate(list(position))


def hex_prism(across_flats, height, position=(0, 0, 0), direction="+Z"):
    r = across_flats / math.sqrt(3)
    pts = [[r*math.cos(math.radians(30+60*i)),
            r*math.sin(math.radians(30+60*i))] for i in range(6)]
    return orient(m.Manifold.extrude(m.CrossSection([pts]), height=height),
                  position, direction)


def insert_tool(x, y, z, direction, depth=6.2, through=0):
    # Named RX-M2.5x5.7: OD4.6 x L5.7. Pilot D4.0, small entry chamfer.
    o = cyl(2.0, depth+0.5, z=-depth)
    lead = m.Manifold.cylinder(height=0.4, radius_low=2.0,
                              radius_high=2.25, circular_segments=64).translate([0, 0, -0.25])
    o = o + lead
    if through:
        o = o + cyl(1.4, through+0.5, z=-through)
    # Default tool extends from surface down along -Z.
    rots = {"-Z": [0, 0, 0], "+Z": [180, 0, 0],
            "-X": [0, 90, 0], "+X": [0, -90, 0],
            "-Y": [-90, 0, 0], "+Y": [90, 0, 0]}
    return o.rotate(rots[direction]).translate([x, y, z])


def json_write(path, obj):
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False,
                               default=lambda v: v.item() if hasattr(v, "item") else str(v)), encoding="utf-8")


def export_part(name, solid, out, print_rotation=(0, 0, 0)):
    solid = solid.simplify(0.003)
    assert solid.status() == m.Error.NoError and solid.volume() > 0, (name, solid.status())
    assert len(solid.decompose()) == 1, (name, "islands", len(solid.decompose()))
    assembly_path = out / "assembly_meshes" / (name + ".stl")
    assembly_path.parent.mkdir(parents=True, exist_ok=True)
    export_stl(solid, assembly_path)
    oriented = solid.rotate(list(print_rotation))
    bb = oriented.bounding_box()
    oriented = oriented.translate([-(bb[0]+bb[3])/2, -(bb[1]+bb[4])/2, -bb[2]])
    path = out / "stl" / (name + ".stl")
    export_stl(oriented, path)
    export_3mf(oriented, out / "3mf" / (name + ".3mf"), name=name)
    qa = validate_mesh(path)
    assert all(c["pass"] for c in qa["checks"].values()), (name, qa)
    mesh = trimesh.load(path, force="mesh")
    qa.update({"islands": mesh_islands(mesh), "size_mm": mesh.extents.tolist(),
               "print_rotation_deg": print_rotation, "min_z_mm": float(mesh.bounds[0, 2]),
               "material_volume_mm3": solid.volume()})
    assert qa["islands"] == 1 and abs(qa["min_z_mm"]) < 1e-4
    return solid, qa
