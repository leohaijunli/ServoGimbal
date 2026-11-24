"""Validate actual exported reliefs against the preserved baseline/interfaces."""
from pathlib import Path
import argparse
import hashlib
import json

import manifold3d as m
import numpy as np
import trimesh

from geometry import HERE, box, cyl, roundplate, json_write
from build_gimbal import BASE_BOLTS, UPPER_CAP_BOLTS, base_protected_regions


def load(path):
    mesh = trimesh.load(path, force="mesh", process=True)
    solid = m.Manifold(m.Mesh(np.asarray(mesh.vertices, dtype=np.float32),
                              np.asarray(mesh.faces, dtype=np.uint32)))
    assert solid.status() == m.Error.NoError
    return solid


def carrier_interface_regions():
    mask = roundplate(90, 84, 6, 5, z=46)
    mask = mask + (cyl(36.3, 10, z=46)-cyl(33.1, 11, z=45.5))
    mask = mask + cyl(33, 0.8, z=55.6) + cyl(33, 0.8, z=78.6)
    for z in (55, 71):
        mask = mask + cyl(20.1, 9, z=z)
    for x, y in BASE_BOLTS:
        mask = mask + cyl(7, 33, x, y, 46)
    for x in (-19, 19):
        for y in (-19, 19):
            mask = mask + cyl(5.5, 8, x, y, 55.5)
    for x, y in UPPER_CAP_BOLTS:
        mask = mask + cyl(5.5, 8, x, y, 71.5)
    return mask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    args = parser.parse_args()
    out = HERE/"output"
    old_doc = json.loads((args.baseline/"manifest.json").read_text(encoding="utf-8"))
    new_doc = json.loads((out/"manifest.json").read_text(encoding="utf-8"))
    changed = {"01_Base", "02_Pan_Bearing_Carrier"}
    for key in ("go_pro", "servo_placements", "fits", "inserts", "screws", "hardware", "nuts"):
        assert old_doc[key] == new_doc[key], ("Interface parameters changed", key)
    unchanged = []
    for name in new_doc["printed_parts"]:
        if name in changed:
            continue
        for folder, extension in (("stl", ".stl"), ("3mf", ".3mf"), ("assembly_meshes", ".stl")):
            a = args.baseline/folder/(name+extension)
            b = out/folder/(name+extension)
            assert hashlib.sha256(a.read_bytes()).digest() == hashlib.sha256(b.read_bytes()).digest()
        unchanged.append(name)
    rows = {}
    for name, mask in (("01_Base", base_protected_regions()),
                       ("02_Pan_Bearing_Carrier", carrier_interface_regions())):
        before = load(args.baseline/"assembly_meshes"/(name+".stl"))
        after = load(out/"assembly_meshes"/(name+".stl"))
        added = max(0, (after-before).volume())
        lost = max(0, ((before ^ mask)-after).volume())
        bbox_error = float(np.max(np.abs(np.array(before.bounding_box())-after.bounding_box())))
        row = {"before_volume_mm3": before.volume(), "after_volume_mm3": after.volume(),
               "removed_volume_mm3": before.volume()-after.volume(),
               "reduction_percent": (1-after.volume()/before.volume())*100,
               "protected_material_loss_mm3": lost, "added_material_mm3": added,
               "bbox_difference_mm": bbox_error, "connected_components": len(after.decompose()),
               "print_mesh_checks_pass": all(c["pass"] for c in new_doc["quality"][name]["checks"].values())}
        row["pass"] = (lost < 0.1 and added < 0.1 and bbox_error < 0.0031
                       and row["connected_components"] == 1 and row["print_mesh_checks_pass"])
        rows[name] = row
    total_before = sum(v["material_volume_mm3"] for v in old_doc["quality"].values())
    total_after = sum(v["material_volume_mm3"] for v in new_doc["quality"].values())
    report = {"units": "mm", "parts": rows, "all_interface_parameters_unchanged": True,
              "unchanged_printed_parts": unchanged,
              "total_before_mm3": total_before, "total_after_mm3": total_after,
              "total_reduction_percent": (1-total_after/total_before)*100,
              "mesh_simplification_tolerance_mm": 0.003,
              "protected_compare_limit_mm3": 0.1,
              "pass": all(r["pass"] for r in rows.values()),
              "physical_validation": "No FDM fit/load/stiffness acceptance; relief region selection is not an FEA result"}
    json_write(out/"lightweight_report.json", report)
    print(json.dumps(report, indent=2), flush=True)
    if not report["pass"]:
        raise SystemExit("Lightweight preservation check failed; do not deliver")


if __name__ == "__main__":
    main()
