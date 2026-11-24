"""One deterministic OCC measurement; persist evidence instead of re-probing.

This reads CAD topology and plane sections, not construction-point ranges or
vertex averages. Supplied STLs are checked in their current, unscaled units.
"""
from pathlib import Path
import hashlib
import json
import math

import cadquery as cq
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_REVERSED
from OCP.gp import gp_Dir, gp_Pln, gp_Pnt

HERE = Path(__file__).resolve().parent
SOURCE = Path(r"\\Artemis\Users\HLi\Downloads\seeker")
OUT = HERE / "reference"
OUT.mkdir(parents=True, exist_ok=True)


def bbox(shape):
    b = shape.BoundingBox()
    return {"min": [b.xmin, b.ymin, b.zmin],
            "max": [b.xmax, b.ymax, b.zmax],
            "size": [b.xlen, b.ylen, b.zlen]}


def stl_bbox(path):
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    assert len(data) == 84 + count * 50, "Expected binary STL"
    dtype = np.dtype([("normal", "<f4", (3,)), ("p", "<f4", (3, 3)),
                      ("attr", "<u2")])
    p = np.frombuffer(data, dtype=dtype, count=count, offset=84)["p"]
    p = p.reshape(-1, 3)
    mn, mx = p.min(axis=0), p.max(axis=0)
    return {"triangles": count, "min": mn.tolist(), "max": mx.tolist(),
            "size": (mx - mn).tolist()}


def cylinders(shape):
    out = []
    for f in shape.Faces():
        a = BRepAdaptor_Surface(f.wrapped)
        if a.GetType() != GeomAbs_Cylinder:
            continue
        c = a.Cylinder()
        p, d = c.Location(), c.Axis().Direction()
        out.append({"radius": c.Radius(),
                    "axis_point": [p.X(), p.Y(), p.Z()],
                    "axis_direction": [d.X(), d.Y(), d.Z()],
                    "face_bounds": bbox(f), "area": f.Area()})
    return out


def section_wires(shape, y):
    op = BRepAlgoAPI_Section(shape.wrapped,
                            gp_Pln(gp_Pnt(0, y, 0), gp_Dir(0, 1, 0)), False)
    op.Build()
    assert op.IsDone(), "OCC section failed"
    edges = cq.Shape.cast(op.Shape()).Edges()
    return cq.Wire.combine(edges, tol=1e-6)


def ordered_points(wire, centre=(0, 0)):
    points = []
    ex = BRepTools_WireExplorer(wire.wrapped)
    while ex.More():
        e = ex.Current()
        a = BRepAdaptor_Curve(e)
        lo, hi = a.FirstParameter(), a.LastParameter()
        if e.Orientation() == TopAbs_REVERSED:
            lo, hi = hi, lo
        # Straight flanks retain their endpoints; arcs get ~0.01 mm spacing.
        n = max(2, int(math.ceil(cq.Edge(e).Length() / 0.012)))
        for u in np.linspace(lo, hi, n, endpoint=False):
            p = a.Value(float(u))
            points.append([p.X() - centre[0], p.Z() - centre[1]])
        ex.Next()
    return np.asarray(points)


def radial_profile(points, count=7200):
    p = np.asarray(points)
    q = np.roll(p, -1, axis=0)
    edge = q - p
    angles = np.linspace(0, math.tau, count, endpoint=False)
    radii = []
    for ang in angles:
        d = np.array([math.cos(ang), math.sin(ang)])
        denom = d[0] * edge[:, 1] - d[1] * edge[:, 0]
        ok = np.abs(denom) > 1e-12
        t = np.divide(p[:, 0] * edge[:, 1] - p[:, 1] * edge[:, 0],
                      denom, out=np.full(len(p), -1.0), where=ok)
        u = np.divide(p[:, 0] * d[1] - p[:, 1] * d[0],
                      denom, out=np.full(len(p), -1.0), where=ok)
        hit = t[ok & (t > 0) & (u >= -1e-8) & (u <= 1 + 1e-8)]
        assert len(hit), "Section is not closed or not star-shaped"
        radii.append(float(hit.max()))
    return angles, np.asarray(radii)


def describe_sections(shape, planes, centre):
    out = []
    for y in planes:
        rings = []
        for w in section_wires(shape, y):
            p = ordered_points(w, centre)
            r = np.hypot(p[:, 0], p[:, 1])
            rings.append({"closed": w.IsClosed(), "edges": len(w.Edges()),
                          "length": w.Length(), "rmin": float(r.min()),
                          "rmax": float(r.max()), "bounds": bbox(w)})
        out.append({"plane_y_mm": y, "loops": rings})
    return out


def main():
    report = {"units": "mm", "method": "OpenCascade BREP + exact plane sections",
              "frame": "source STEP, unchanged", "sources": {}}
    shapes = {}
    for key, stem in (("servo", "MKS-X8-HBL388"),
                      ("gear_mount", "MKS-X8-HBL388-Gear Mount")):
        path = SOURCE / (stem + ".STEP")
        shape = cq.importers.importStep(str(path)).val()
        shapes[key] = shape
        report["sources"][key] = {
            "file": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "bounds": bbox(shape), "valid": shape.isValid(),
            "solids": len(shape.Solids()), "cylinders": cylinders(shape),
            "supplied_stl": stl_bbox(SOURCE / (stem + ".stl")),
            "supplied_stl_sha256": hashlib.sha256((SOURCE / (stem + ".stl")).read_bytes()).hexdigest()}
        print(key, "STEP bounds:", bbox(shape))
        print(key, "supplied STL:", report["sources"][key]["supplied_stl"])

    servo = shapes["servo"]
    # Axial surfaces establish the axis directly. This is not a vertex mean.
    axes = [c for c in cylinders(servo) if abs(c["radius"] - 4.0) < 1e-6]
    assert axes and all(abs(c["axis_direction"][1]) > 0.999 for c in axes)
    cx, _, cz = axes[0]["axis_point"]
    report["output_axis"] = {"origin_xz": [cx, cz], "direction": [0, 1, 0],
                             "evidence": axes}
    wires = section_wires(servo, 23.0)
    outer = max(wires, key=lambda w: w.Length())
    points = ordered_points(outer, (cx, cz))
    angles, radii = radial_profile(points)
    # Count separated tooth crests with a threshold between root and tip.
    tooth_band = radii > (radii.min() + radii.max()) / 2
    tooth_count = int(np.count_nonzero(tooth_band & ~np.roll(tooth_band, 1)))
    report["spline"] = {"section_y": 23.0, "teeth_measured": tooth_count,
                        "root_radius": float(radii.min()), "tip_radius": float(radii.max()),
                        "ordered_polygon": points.tolist(),
                        "radial_angles": angles.tolist(), "radial_radii": radii.tolist()}
    print("AXIS", cx, cz, "+Y; measured spline teeth:", tooth_count,
          "root/tip:", radii.min(), radii.max())
    report["servo_sections"] = describe_sections(servo, [8.65, 19.0, 20.8, 23.0, 25.8], (cx, cz))
    report["mount_sections"] = describe_sections(shapes["gear_mount"], [-1.0, 0.0, 2.0, 3.7], (0, 0))
    # Actual mounting-ear slot axes; shell assembly screws are different.
    report["ear_slots"] = [c for c in cylinders(servo)
                            if abs(c["radius"] - 2.0) < 1e-6
                            and abs(abs(c["axis_point"][0]) - 26.25) < 0.001]
    print("EAR SLOT CYLINDERS:")
    for c in report["ear_slots"]:
        print(c)
    for entry in report["mount_sections"]:
        print("GEAR MOUNT SECTION", entry)
    (OUT / "reference_evidence.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    # Preserve exact STEP provenance; structural models later use this section.
    for key, shape in shapes.items():
        cq.exporters.export(shape, str(OUT / (key + ".stl")), tolerance=0.008,
                            angularTolerance=0.05)
    print("REFERENCE EVIDENCE SAVED")


if __name__ == "__main__":
    main()
