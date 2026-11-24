"""Verify the final CAD evidence and publish an independent, minimal reprint kit.

Never overwrite a prior design or an existing destination. No mesh repair,
rescaling, new reference measurement, git operations or silent file deletion.
"""
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "output"
REUSE = (
    "01_Base", "03_Pan_Lower_Cap", "04_Pan_Upper_Cap", "06_Pan_Clamp_Collar",
    "08_U_Frame", "10_HERO8_Top_Clamp", "11_Tilt_25T_Hub", "12_Tilt_Clamp_Collar",
    "13_Left_Bearing_Cap", "16_Pan_Front_Mount_Bridge",
)
CHANGE = (
    "02_Pan_Bearing_Carrier", "05_Pan_25T_Hub", "07_Pan_Inner_Spacer",
    "09_HERO8_Cradle", "14_Left_Inner_Spacer", "15_Removable_Left_Support",
)
ADD = ("17_Pan_25T_Coupler", "18_6200RS_Outer_Ring_Adapter")


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_csv(path, header, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def audit():
    doc = read_json(OUT / "manifest.json")
    checks = read_json(OUT / "assembly_checks.json")
    baseline = read_json(ROOT / "reference/previous_designs_sha256.json")
    assert checks["pass"] and checks["manifest_sha256"] == sha(OUT / "manifest.json")
    assert checks["printed_static_and_motion_pass"] and checks["bearing_interfaces"]["pass"]
    assert checks["pan_detachable_key"]["pass"]
    assert doc["hardware_evidence_sha256"] == sha(ROOT / "reference/hardware_evidence.json")
    assert len(doc["hardware"]) == 3
    assert all((row["id"], row["od"], row["length"]) == (10.0, 30.0, 9.0)
               and row["name"].startswith("6200RS") for row in doc["hardware"])

    inventory = doc["print_file_inventory"]
    expected = set(REUSE + CHANGE + ADD)
    assert set(inventory) == expected and len(expected) == 18
    for folder, extension in (("stl", ".stl"), ("3mf", ".3mf")):
        assert {path.stem for path in (OUT / folder).glob("*" + extension)} == expected
    assert len(doc["printed_parts"]) == 20
    assert sum(row["quantity"] for row in inventory.values()) == 20
    assert inventory[ADD[1]]["quantity"] == 3
    for name, report in doc["quality"].items():
        assert report["islands"] == 1 and all(row["pass"] for row in report["checks"].values())
        print_id = doc["printed_parts"][name].get("print_file_id", name)
        for folder, extension in (("stl", ".stl"), ("3mf", ".3mf"), ("assembly_meshes", ".stl")):
            file_id = name if folder == "assembly_meshes" else print_id
            assert sha(OUT / folder / (file_id + extension)) == report["file_sha256"][folder]

    differences = {}
    for name in REUSE + CHANGE:
        differences[name] = {}
        for folder, extension in (("stl", ".stl"), ("3mf", ".3mf")):
            before = baseline["files"][f"mks_gimbal/{folder}/{name}{extension}"]
            after = sha(OUT / folder / (name + extension))
            same = before == after
            assert same == (name in REUSE), (name, folder, "Unexpected print-file change/reuse")
            differences[name][folder] = {"baseline_sha256": before, "current_sha256": after,
                                          "byte_identical": same}
    assert doc["printed_parts"]["13_Left_Bearing_Cap"]["reuse_translation_mm"] == [-5.0, 0, 0]
    original_root = Path(baseline["source_root"])
    original_changes = [name for name, value in baseline["files"].items()
                        if not (original_root / name).is_file() or sha(original_root / name) != value]
    assert not original_changes, ("Previous/user design changed; do not overwrite or relabel it", original_changes)

    fit = read_json(ROOT / "fit_tests/fit_test_validation.json")
    assert fit["hardware_evidence_sha256"] == doc["hardware_evidence_sha256"]
    for name, result in fit["results"].items():
        assert all(row["pass"] for row in result["checks"].values())
        for extension, value in result["file_sha256"].items():
            assert sha(ROOT / "fit_tests" / (name + extension)) == value

    full_volume = sum(row["material_volume_mm3"] for row in doc["quality"].values())
    reprint_volume = sum(row["material_volume_mm3"] for name, row in doc["quality"].items()
                        if doc["printed_parts"][name].get("print_file_id", name) in CHANGE + ADD)
    report = {
        "design_version": doc["design_version"], "units": "mm",
        "verified_at_UTC": datetime.now(timezone.utc).isoformat(),
        "build_manifest_sha256": sha(OUT / "manifest.json"),
        "assembly_checks_sha256": sha(OUT / "assembly_checks.json"),
        "original_files_checked": len(baseline["files"]), "original_files_changed": [],
        "original_print_files_reused": list(REUSE), "original_print_files_to_reprint": list(CHANGE),
        "additional_print_files": {ADD[0]: 1, ADD[1]: 3},
        "original_file_hash_comparison": differences,
        "reposed_only": {"13_Left_Bearing_Cap": {"translation_mm": [-5, 0, 0],
                         "print_files_byte_identical": True}},
        "unique_files_in_reprint_kit": 8, "new_printed_pieces": 10,
        "full_unique_print_files": 18, "full_printed_pieces": 20,
        "all_present_bearings": "6200RS 10x30x9; Pan 2, Tilt 1",
        "full_model_solid_volume_mm3": full_volume, "reprint_solid_volume_mm3": reprint_volume,
        "solid_volume_avoided_vs_full_reprint_percent": (1-reprint_volume/full_volume)*100,
        "volume_note": "Actual exported CAD solid volumes, not slicer material/time estimates; supports, infill, purge and scrap excluded",
        "validation": "Manifold mesh, local bearing interfaces, fasteners/tools, assembly routes and independent +/-90deg 5deg motion samples passed",
        "physical_validation": "Not printed or load/precision tested. Unknown-brand seal contact, keyed-joint backlash, stiffness and lifetime remain unverified.",
        "tradeoffs": "Pan centre spacing 16->12 mm; added detachable 8AF joint and three 1mm outer-ring adapters. No unchanged-stiffness or zero-backlash claim.",
    }
    return doc, checks, report


def tables(stage, doc, checks):
    rows = []
    for print_id, item in sorted(doc["print_file_inventory"].items()):
        instance = item["assembly_instances"][0]
        info = doc["quality"][instance]
        inserts = sum(row["part"] in item["assembly_instances"] for row in doc["inserts"])
        groups = "/".join(sorted({doc["printed_parts"][name]["group"] for name in item["assembly_instances"]}))
        rows.append([print_id, item["quantity"], *[round(value, 3) for value in info["size_mm"]],
                     round(info["material_volume_mm3"], 3), inserts, groups,
                     "复用" if print_id in REUSE else "重打" if print_id in CHANGE else "新增"])
    write_csv(stage / "PARTS.csv", ["文件", "数量", "打印X_mm", "打印Y_mm", "打印Z_mm",
                                    "单件CAD体积_mm3", "M2.5铜套数量", "运动组", "本次动作"], rows)
    bolts = Counter(f"M{row['thread_mm']:g}x{row['length_mm']:g} {row['head_standard']}"
                    for row in doc["screws"])
    bolts["ruthex RX-M2.5x5.7"] = len(doc["inserts"])
    bolts["M3 DIN934 AF5.5 H2.4"] = len(doc["nuts"])
    for row in doc["screws"]:
        if row["washer_mm"]:
            bolts[f"M{row['thread_mm']:g} DIN125A {row['washer_ID_mm']:g}x{row['washer_OD_mm']:g}x{row['washer_mm']:g}"] += 1
    bolts["6200RS 10x30x9 (brand/seal faces require physical check)"] = 3
    bolts["MKS-X8-HBL388"] = 2
    write_csv(stage / "BOM.csv", ["物料", "数量"], sorted(bolts.items()))
    write_csv(stage / "FASTENERS.csv", ["ID", "接合", "螺纹_mm", "头下长_mm", "标准",
              "接触面坐标_mm", "方向", "接收零件", "啮合_mm", "孔底余量_mm", "数字结果"],
              [[row["id"], row["joint"], row["thread_mm"], row["length_mm"], row["head_standard"],
                json.dumps(row["bearing_surface"]), row["direction"], row["receiver_part"],
                row["engagement_mm"], row["OEM_thread_bottom_reserve_mm"], "PASS"]
               for row in checks["fasteners"]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only", action="store_true", help="Validate final meshes/reuse/source preservation without publishing")
    parser.add_argument("--destination", type=Path, help="New independent delivery directory; must not already exist")
    args = parser.parse_args()
    if not args.check_only and args.destination is None:
        parser.error("--destination is required unless --check-only")
    doc, checks, report = audit()
    print(json.dumps({key: report[key] for key in ("original_files_checked", "unique_files_in_reprint_kit",
          "new_printed_pieces", "original_print_files_reused", "original_print_files_to_reprint",
          "solid_volume_avoided_vs_full_reprint_percent")}, ensure_ascii=False, indent=2))
    if args.check_only:
        return
    assert not args.destination.exists(), "Destination exists; inspect it instead of overwriting"
    stage = ROOT / "delivery"
    assert not stage.exists(), "Staging directory exists; inspect before creating another delivery"
    for name in ("mks_gimbal.blend", "mks_gimbal_exploded.blend", "mks_gimbal_assembly.glb",
                 "viewer.html", "viewer_report.json", "blender_export_report.json"):
        assert (OUT / name).is_file(), (name, "Final visual export is missing")
    visual = read_json(OUT / "blender_export_report.json")
    assert visual["assembly_checks_sha256"] == report["assembly_checks_sha256"]
    assert visual["raw_surface_triangles_exact_match"] and not visual["reference_vertices_changed"]
    viewer = read_json(OUT / "viewer_report.json")
    assert viewer["embedded_GLB_sha256"] == sha(OUT / "mks_gimbal_assembly.glb")
    assert viewer["html_sha256"] == sha(OUT / "viewer.html")
    if viewer["browser_validation"] == "passed":
        assert viewer["browser_checks"]["pass"] and viewer["browser_checks"]["console_errors"] == 0

    stage.mkdir()
    for folder in ("stl", "3mf", "assembly_meshes", "renders"):
        shutil.copytree(OUT / folder, stage / folder)
    for folder in ("source", "reference", "fit_tests"):
        shutil.copytree(ROOT / folder, stage / folder, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    for name in ("README.md", "REPRINT_GUIDE.md", "STL_PRINT_VALIDATION.md"):
        shutil.copy2(ROOT / name, stage / name)
    for name in ("manifest.json", "quality_report.json", "assembly_checks.json", "blender_export_report.json",
                 "viewer_report.json", "mks_gimbal.blend", "mks_gimbal_exploded.blend",
                 "mks_gimbal_assembly.glb", "viewer.html"):
        shutil.copy2(OUT / name, stage / name)
    if (OUT / "MKS_PanTilt_motion.blend").is_file():
        animation = stage / "animation"
        animation.mkdir()
        shutil.copy2(OUT / "MKS_PanTilt_motion.blend", animation)
        (animation / "README.md").write_text("# 6200RS当前模型运动副本\n\n使用本次总装生成；未复制旧版轴承视频。数字展示不是承载/线缆/实机行程验收。可在Blender中播放或按source说明渲染。\n", encoding="utf-8")
    for folder, extension in (("stl", ".stl"), ("3mf", ".3mf")):
        target = stage / "reprint_only" / folder
        target.mkdir(parents=True)
        for print_id in CHANGE + ADD:
            shutil.copy2(stage / folder / (print_id + extension), target / (print_id + extension))
    report["delivery_directory"] = str(args.destination)
    write_json(stage / "reprint_report.json", report)
    tables(stage, doc, checks)
    sums = {path.relative_to(stage).as_posix(): sha(path) for path in sorted(stage.rglob("*")) if path.is_file()}
    write_json(stage / "SHA256SUMS.json", sums)
    shutil.copytree(stage, args.destination)
    assert all(sha(args.destination / name) == value for name, value in sums.items())
    print("DELIVERED", args.destination, "files", len(sums)+1)


if __name__ == "__main__":
    main()
