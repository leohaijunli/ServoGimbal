"""Small, independently validated fit samples from the cached STEP section.

All generated geometry uses manifold3d. No STL repair, auto-rescale, guessed
teeth, or repeated reference probing. These are trials, NOT load-rated hubs.
"""
from pathlib import Path
import hashlib
import json
import math
import sys

import manifold3d as m3d
import numpy as np
import trimesh

CODE = Path(__file__).resolve().parent
ROOT = CODE.parent if CODE.name == "source" and (CODE.parent/"reference").is_dir() else CODE
SKILL = Path(r"C:\Users\HLi\.config\opencode\skills\3d-print")
sys.path.insert(0, str(CODE/"lib" if (CODE/"lib").is_dir() else SKILL/"scripts"))
from generate_model import export_3mf, export_stl
from validate_mesh import validate_mesh

OUT = ROOT / "fit_tests"
OUT.mkdir(parents=True, exist_ok=True)


def dump(path, obj):
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False,
                               default=lambda v: v.item() if hasattr(v, "item") else str(v)),
                    encoding="utf-8")


def mesh_islands(mesh):
    parent = list(range(len(mesh.vertices)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b, c in mesh.faces:
        a, b, c = int(a), int(b), int(c)
        parent[find(b)] = find(a)
        parent[find(c)] = find(a)
    return len({find(i) for i in range(len(parent))})


def save(name, part):
    # Eliminate sub-micron construction slivers BEFORE STL float32 export.
    # This is a bounded geometry tolerance, not automatic mesh repair.
    part = part.simplify(0.003)
    assert part.status() == m3d.Error.NoError, (name, part.status())
    assert len(part.decompose()) == 1, (name, "disconnected material")
    assert part.volume() > 0
    export_3mf(part, OUT / (name + ".3mf"), name=name)
    export_stl(part, OUT / (name + ".stl"))
    report = validate_mesh(OUT / (name + ".stl"))
    assert all(x["pass"] for x in report["checks"].values()), report
    mesh = trimesh.load(OUT / (name + ".stl"), force="mesh")
    report["components"] = mesh_islands(mesh)
    report["size_mm"] = mesh.extents.tolist()
    assert report["components"] == 1
    print("VALIDATED", name, "size", mesh.extents, "components", report["components"])
    return report


def main():
    evidence_path = ROOT / "reference" / "reference_evidence.json"
    data = json.loads(evidence_path.read_text(encoding="utf-8"))
    assert data["spline"]["teeth_measured"] == 25
    p = np.asarray(data["spline"]["ordered_polygon"], dtype=float)
    # STEP section is source X-Z. In printed pose the socket opens upward.
    # Flip the finished cup 180 deg about X for a +Z servo installation.
    area = np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1))
    if area < 0:
        p = p[::-1]
    raw = m3d.CrossSection([p]).simplify(0.003)
    assert raw.area() > 30, "Expected the exterior spline profile"
    reports = {}
    nominal_shaft = m3d.Manifold.extrude(raw, height=6.0).translate([0, 0, 2.0])
    for i, allowance in enumerate((0.03, 0.06, 0.10), start=1):
        name = f"T0{i}_25T_socket_clearance_{round(allowance*1000):03d}um"
        profile = raw.offset(allowance, m3d.JoinType.Round,
                             circular_segments=16).simplify(0.003)
        # Bypass the failing upstream revolve helper. Native primitives only.
        cup = m3d.Manifold.cylinder(height=7.7, radius_low=10.0,
                                    circular_segments=128).translate([0, 0, 0.3])
        foot = m3d.Manifold.cylinder(height=0.35, radius_low=9.65,
                                     radius_high=10.0, circular_segments=128)
        cup = cup + foot
        tool = m3d.Manifold.extrude(profile, height=6.5).translate([0, 0, 2.0])
        # Lead-in enlarges only the last 0.4 mm. It must NOT erase the teeth.
        lead = m3d.Manifold.extrude(profile, height=0.5, scale_top=[1.075, 1.075])
        lead = lead.translate([0, 0, 7.6])
        bore = m3d.Manifold.cylinder(height=9, radius_low=1.65, circular_segments=64)
        cup = cup - tool - lead - bore.translate([0, 0, -0.5])
        report = save(name, cup)
        overlap = (cup ^ nominal_shaft).volume()
        half_pitch_clash = (cup ^ nominal_shaft.rotate([0, 0, 7.2])).volume()
        assert abs(overlap) < 1e-6, (name, overlap)
        assert half_pitch_clash > 0.5, "No positive tooth engagement"
        report.update({"normal_clearance_mm": allowance,
                       "nominal_fit_intersection_mm3": overlap,
                       "half_pitch_intersection_mm3": half_pitch_clash,
                       "full_profile_length_mm": 5.6,
                       "physical_fit": "not tested"})
        reports[name] = report
        print("TOOTH FIT", name, "nominal overlap", overlap,
              "half-pitch clash", half_pitch_clash)

    # Widely available named M2.5 insert, not an assumed universal exterior.
    # RX-M2.5x5.7 CAD: knurl OD ~4.6, L5.7. Try three heat-install hole sizes.
    plate = m3d.Manifold.cube([52, 20, 8])
    for x, diameter in ((9, 3.8), (26, 4.0), (43, 4.2)):
        hole = m3d.Manifold.cylinder(height=6.6, radius_low=diameter/2,
                                     circular_segments=80).translate([x, 10, 1.8])
        lip = m3d.Manifold.cylinder(height=0.6, radius_low=diameter/2,
                                    radius_high=diameter/2+0.3,
                                    circular_segments=80).translate([x, 10, 7.5])
        plate = plate - hole - lip
    name = "T04_M2.5_RX_insert_holes_3.8_4.0_4.2"
    reports[name] = save(name, plate)
    reports[name]["insert"] = {"product": "ruthex RX-M2.5x5.7",
                               "thread": "M2.5", "knurl_OD_mm": 4.6,
                               "length_mm": 5.7,
                               "hole_sizes_left_to_right_mm": [3.8, 4.0, 4.2],
                               "heat_installation": "not tested"}
    for prefix, bearing, diameter, thickness in (("T05", "6804", 32.0, 7.0),
                                                ("T06", "6800", 19.0, 5.0)):
        for extra in (0.0, 0.1, 0.2):
            name = f"{prefix}_{bearing}_seat_D{diameter+extra:.1f}"
            ring = m3d.Manifold.cylinder(height=thickness+0.5,
                                         radius_low=diameter/2+4,
                                         circular_segments=128)
            hole = m3d.Manifold.cylinder(height=thickness+1.5,
                                         radius_low=(diameter+extra)/2,
                                         circular_segments=128).translate([0, 0, -0.5])
            reports[name] = save(name, ring-hole)
    for diameter in (9.98, 19.98):
        name = f"T07_journal_D{diameter:.2f}"
        pin = m3d.Manifold.cylinder(height=15, radius_low=diameter/2,
                                    circular_segments=128)
        flange = m3d.Manifold.cylinder(height=2, radius_low=diameter/2+3,
                                       circular_segments=128)
        reports[name] = save(name, pin+flange)
    dump(OUT / "fit_test_validation.json", {
        "reference_evidence_sha256": hashlib.sha256(evidence_path.read_bytes()).hexdigest(),
        "reference_servo_sha256": data["sources"]["servo"]["sha256"],
        "units": "mm", "results": reports})

    # Inspectable section drawing: actual tooth profile, not a smooth circle.
    xy = p * 31
    points = " ".join(f"{200+x:.3f},{200-y:.3f}" for x, y in xy)
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 400 450">
<rect width="400" height="450" fill="#f7f9fc"/>
<circle cx="200" cy="200" r="124" fill="none" stroke="#b8c1cf" stroke-dasharray="4 4"/>
<polygon points="{points}" fill="#dc7939" stroke="#263446" stroke-width="1.1"/>
<circle cx="200" cy="200" r="45" fill="#f7f9fc"/>
<text x="200" y="382" text-anchor="middle" font-size="19" font-family="sans-serif">MKS HBL388 — 25T STEP section</text>
<text x="200" y="409" text-anchor="middle" font-size="16" font-family="sans-serif">Tip Ø8.00 / Root Ø7.00 / Pitch 14.4°</text>
</svg>'''
    (OUT / "spline_section.svg").write_text(svg, encoding="utf-8")
    print("FIT TESTS OK — physical fit and strength NOT yet verified")


if __name__ == "__main__":
    main()
