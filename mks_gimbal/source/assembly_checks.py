"""Canonical assembly audit: hardware, tools, receivers and installation paths.

Uses the exported meshes, not a second geometry construction or a repeated
measurement of the reference STEP. Reported paths/motion are discrete samples.
Tool cylinders are stated envelopes; no particular user's tool is certified.
"""
from pathlib import Path
import hashlib
import json
import math
import zipfile
import xml.etree.ElementTree as ET

import manifold3d as m
import numpy as np
import trimesh

from geometry import HERE, box, cyl, hex_prism, orient, json_write

OUT = HERE / "output"
DOC = json.loads((OUT / "manifest.json").read_text(encoding="utf-8"))
PZ = DOC["go_pro"]["centre"][2]
DIR = {"+X": np.array([1., 0, 0]), "-X": np.array([-1., 0, 0]),
       "+Y": np.array([0, 1., 0]), "-Y": np.array([0, -1., 0]),
       "+Z": np.array([0, 0, 1.]), "-Z": np.array([0, 0, -1.])}
THRESHOLD = 0.05


def load(path):
    mesh = trimesh.load(path, force="mesh", process=True)
    s = m.Manifold(m.Mesh(np.asarray(mesh.vertices, dtype=np.float32),
                          np.asarray(mesh.faces, dtype=np.uint32)))
    assert s.status() == m.Error.NoError, (path, s.status())
    return s


PARTS = {n: load(OUT / "assembly_meshes" / (n+".stl")) for n in DOC["printed_parts"]}
REFS = {}
raw = load(DOC["servo_source"])
for name, pose in DOC["servo_placements"].items():
    matrix = np.c_[pose["rotation_matrix"], pose["translation"]]
    REFS[name] = raw.transform(matrix.tolist())
ALL = {**PARTS, **REFS}
W, H, D = DOC["go_pro"]["size_mm"]
CAMERA = box(W, D, H, z=PZ-H/2)


def intersection(a, b):
    aa, bb = a.bounding_box(), b.bounding_box()
    if any(aa[i+3] <= bb[i]+1e-6 or bb[i+3] <= aa[i]+1e-6 for i in range(3)):
        return 0.0
    return max(0.0, (a ^ b).volume())


def clashes(solid, obstacles, threshold=THRESHOLD):
    return [[name, round(v, 6)] for name, other in obstacles.items()
            if (v := intersection(solid, other)) > threshold]


def screw_envelope(spec):
    shaft = cyl(spec["thread_mm"]/2, spec["length_mm"])
    head = cyl(spec["head_D_mm"]/2, spec["head_H_mm"], z=-spec["head_H_mm"])
    result = shaft + head
    if spec["washer_mm"]:
        washer = cyl(spec["washer_OD_mm"]/2, spec["washer_mm"])
        washer = washer - cyl(spec["washer_ID_mm"]/2, spec["washer_mm"]+1, z=-0.5)
        result = result + washer
    return orient(result, spec["bearing_surface"], spec["direction"])


def nut_envelope(spec):
    s = hex_prism(spec["across_flats_mm"], spec["length_mm"])
    s = s - cyl(1.6, spec["length_mm"]+1, z=-0.5)
    return orient(s, spec["position"], spec["direction"])


def insert_envelope(spec):
    s = cyl(spec["OD_mm"]/2, spec["length_mm"])
    s = s - cyl(1.025, spec["length_mm"]+1, z=-0.5)
    return orient(s, spec["position"], spec["direction"])


def bearing_envelope(spec):
    s = cyl(spec["od"]/2, spec["length"])
    s = s - cyl(spec["id"]/2, spec["length"]+1, z=-0.5)
    return orient(s, spec["position"], "+"+spec["axis"])


RECEIVERS = {
    "Pan mounting ear": ("nut", "01_Base"),
    "Tilt mounting ear": ("nut", "08_U_Frame"),
    "Carrier to base": ("insert", "01_Base"),
    "Front bridge to base": ("insert", "01_Base"),
    "Lower bearing retainer": ("insert", "02_Pan_Bearing_Carrier"),
    "Upper bearing retainer": ("insert", "02_Pan_Bearing_Carrier"),
    "Pan spline clamp": ("insert", "06_Pan_Clamp_Collar"),
    "U-frame to Pan flange": ("insert", "05_Pan_25T_Hub"),
    "Left support to deck": ("insert", "08_U_Frame"),
    "Left bearing retainer": ("insert", "15_Removable_Left_Support"),
    "Left journal inner-race clamp": ("insert", "09_HERO8_Cradle"),
    "HERO8 removable top clamp": ("insert", "09_HERO8_Cradle"),
    "Tilt flange to cradle": ("insert", "09_HERO8_Cradle"),
    "Tilt spline clamp": ("insert", "12_Tilt_Clamp_Collar"),
    "Pan hub OEM shaft retainer": ("OEM thread", "MKS_Pan"),
    "Tilt hub OEM shaft retainer": ("OEM thread", "MKS_Tilt"),
}

PAN_MODULE = ["02_Pan_Bearing_Carrier", "03_Pan_Lower_Cap", "04_Pan_Upper_Cap",
              "05_Pan_25T_Hub", "06_Pan_Clamp_Collar", "07_Pan_Inner_Spacer"]
TILT_MODULE = ["09_HERO8_Cradle", "11_Tilt_25T_Hub", "12_Tilt_Clamp_Collar"]

STAGES = {
    "Pan mounting ear": (["01_Base", "16_Pan_Front_Mount_Bridge", "MKS_Pan"], "先固定Pan舵机，尚未放入轴承模块"),
    "Front bridge to base": (["01_Base", "16_Pan_Front_Mount_Bridge", "MKS_Pan"], "插入舵机后，从前侧滑入安装桥，再经顶面紧固"),
    "Carrier to base": (["01_Base", "16_Pan_Front_Mount_Bridge", "MKS_Pan"]+PAN_MODULE, "先经四个深工具孔紧固，后装转台"),
    "Lower bearing retainer": (PAN_MODULE, "轴承模块独立装配，未安装到底座"),
    "Upper bearing retainer": (PAN_MODULE, "经Pan法兰四个工具孔拧紧，尚未安装转台"),
    "Pan hub OEM shaft retainer": (["01_Base", "MKS_Pan"]+PAN_MODULE, "转台安装前拧紧轴心螺钉"),
    "Pan spline clamp": (["01_Base", "MKS_Pan"]+PAN_MODULE, "从轴承座侧面窗口拧紧"),
    "U-frame to Pan flange": (["01_Base", "08_U_Frame", "MKS_Pan"]+PAN_MODULE, "笼架未装，从转台上方紧固"),
    "Tilt mounting ear": (["08_U_Frame", "MKS_Tilt"], "先固定Tilt舵机，尚未放入笼架模块"),
    "Tilt flange to cradle": (["09_HERO8_Cradle", "11_Tilt_25T_Hub"], "独立预装；夹紧套和舵机均未安装"),
    "Tilt spline clamp": (list(PARTS)+["MKS_Tilt", "MKS_Pan"], "相机取出，从前/背侧接近夹紧螺钉"),
    "Tilt hub OEM shaft retainer": (list(PARTS)+["MKS_Tilt", "MKS_Pan"], "相机取出，经笼架右侧中心工具孔紧固"),
    "Left support to deck": (list(PARTS)+["MKS_Tilt", "MKS_Pan"], "相机取出，沿左支架外侧拧紧四颗螺钉"),
    "Left bearing retainer": (list(PARTS)+["MKS_Tilt", "MKS_Pan"], "从左侧紧固外圈压盖"),
    "Left journal inner-race clamp": (list(PARTS)+["MKS_Tilt", "MKS_Pan"], "从左侧紧固内圈顶帽"),
    "HERO8 removable top clamp": (list(PARTS)+["MKS_Tilt", "MKS_Pan"], "放入相机后从顶面紧固"),
}


def thread_allowance(spec):
    if spec["joint"] == "Pan hub OEM shaft retainer":
        return orient(cyl(1.8, 6.0), [0, 0, 51.65], "-Z")
    if spec["joint"] == "Tilt hub OEM shaft retainer":
        return orient(cyl(1.8, 6.0), [47.0, 0, PZ], "+X")
    return None


def fastener_audit():
    rows, hardware, failures = [], [], []
    for i, spec in enumerate(DOC["screws"], 1):
        p, axis = np.array(spec["bearing_surface"]), DIR[spec["direction"]]
        kind, owner = RECEIVERS[spec["joint"]]
        if spec["joint"] == "Pan mounting ear" and p[0] < 0:
            owner = "16_Pan_Front_Mount_Bridge"
        candidates = []
        if kind == "OEM thread":
            candidates = [{"part": owner, "position": [0, 0, 51.65] if owner == "MKS_Pan" else [47, 0, PZ],
                           "direction": "-Z" if owner == "MKS_Pan" else "+X", "length_mm": 6.0}]
        else:
            candidates = DOC["nuts" if kind == "nut" else "inserts"]
        aligned = []
        for item in candidates:
            if item["part"] != owner:
                continue
            delta = np.array(item["position"])-p
            if np.linalg.norm(delta-axis*np.dot(delta, axis)) > 0.01:
                continue
            ends = [float(np.dot(delta, axis)),
                    float(np.dot(delta+DIR[item["direction"]]*item["length_mm"], axis))]
            near, far = sorted(ends)
            if far > 0:
                aligned.append((near, far))
        assert len(aligned) == 1, (spec["joint"], p, aligned)
        near, far = aligned[0]
        engagement = max(0, min(far, spec["length_mm"])-max(0, near))
        required = 2.4 if kind == "nut" else 4.0
        reserve = far-spec["length_mm"] if kind == "OEM thread" else None
        env = screw_envelope(spec)
        obs = dict(ALL)
        allowed = thread_allowance(spec)
        if allowed is not None:
            obs[owner] = obs[owner]-allowed
        hits = clashes(env, obs)
        bad = bool(hits) or engagement < required-1e-5 or (reserve is not None and reserve < 1.0)
        row = {"id": f"S{i:02d}", **spec, "receiver_kind": kind, "receiver_part": owner,
               "grip_to_first_thread_mm": round(near, 4),
               "engagement_mm": round(engagement, 4), "required_engagement_mm": required,
               "OEM_thread_bottom_reserve_mm": reserve, "clashes": hits, "pass": not bad}
        rows.append(row)
        hardware.append((row["id"], env, spec["group"]))
        if bad:
            failures.append(row)

    for kind, specs, factory in (("nut", DOC["nuts"], nut_envelope),
                                  ("insert", DOC["inserts"], insert_envelope),
                                  ("bearing", DOC["hardware"], bearing_envelope)):
        for i, spec in enumerate(specs, 1):
            env = factory(spec)
            obs = dict(ALL)
            if kind == "insert":
                # OD4.6 into D4.0 is intentional molten-plastic displacement.
                obs.pop(spec["part"])
            hits = clashes(env, obs)
            if hits:
                failures.append({"kind": kind, "id": i, "clashes": hits})
            group = spec.get("group", DOC["printed_parts"].get(spec.get("part"), {}).get("group"))
            hardware.append((f"{kind}_{i:02d}", env, group))
    return rows, hardware, failures


def tool_audit():
    screws, inserts = [], []
    for i, spec in enumerate(DOC["screws"], 1):
        p = np.array(spec["bearing_surface"])
        outward = -DIR[spec["direction"]]
        # Bare long-arm Allen key plus 0.2mm envelope allowance. A large bit
        # holder is NOT interchangeable in the close servo-ear clearance.
        stem_d = spec["hex_key_mm"]*2/math.sqrt(3)+0.2
        stem = orient(cyl(stem_d/2, 75), p+outward*(spec["head_H_mm"]+0.02),
                      next(k for k, v in DIR.items() if np.array_equal(v, outward)))
        names, stage = STAGES[spec["joint"]]
        hits = clashes(stem, {n: ALL[n] for n in names})
        screws.append({"id": f"S{i:02d}", "joint": spec["joint"], "stage": stage,
                       "stem_OD_mm": stem_d, "stem_length_mm": 75.0,
                       "hex_key_mm": spec["hex_key_mm"], "clashes": hits, "pass": not hits})
    for i, ins in enumerate(DOC["inserts"], 1):
        direction = next(k for k, v in DIR.items() if np.array_equal(v, -DIR[ins["direction"]]))
        p = np.array(ins["position"])-DIR[ins["direction"]]*0.1
        nose = cyl(4.0, 22)+cyl(8.0, 25, z=22)
        tool = orient(nose, p, direction)
        hits = clashes(tool, {ins["part"]: PARTS[ins["part"]]})
        # Verify at least 1 mm radial plastic around the knurl through the
        # useful middle 4.8 mm, excluding the entry chamfer and axial tip.
        ring = cyl(3.3, 4.8, z=0.4)-cyl(2.3, 5.8, z=-0.1)
        ring = orient(ring, ins["position"], ins["direction"])
        missing = max(0, (ring-PARTS[ins["part"]]).volume())
        inserts.append({"id": f"I{i:02d}", "part": ins["part"], "position": ins["position"],
                        "stage": "在独立打印件上装铜套，后组装", "nose_OD_mm": 8.0,
                        "housing_OD_mm": 16.0, "clashes": hits,
                        "radial_wall_checked_mm": 1.0, "wall_missing_volume_mm3": round(missing, 6),
                        "pass": not hits and missing < THRESHOLD})
    return {"screw_access": screws, "heat_insert_access_and_walls": inserts,
            "coverage": "Declared tool envelopes; independent-part heat insertion. No real tool measured.",
            "failures": [r for r in screws+inserts if not r["pass"]]}


def path(name, moving, obstacles, deltas, description, step):
    hits = []
    for delta in deltas:
        for item, solid in moving.items():
            for other, v in clashes(solid.translate(list(delta)), obstacles):
                hits.append({"delta_mm": list(delta), "moving": item, "obstacle": other, "volume_mm3": v})
    return {"name": name, "description": description, "step_mm": step,
            "samples": len(deltas), "failures": hits, "pass": not hits}


def installation_audit():
    paths = []
    paths.append(path("Pan servo", {"MKS_Pan": REFS["MKS_Pan"]}, {"01_Base": PARTS["01_Base"]},
                      [(0, 0, float(z)) for z in range(80, -1, -1)], "从上方插入；前安装桥和轴承模块未装", 1.0))
    paths.append(path("Front mounting bridge", {"16_Bridge": PARTS["16_Pan_Front_Mount_Bridge"]},
                      {"01_Base": PARTS["01_Base"], "MKS_Pan": REFS["MKS_Pan"]},
                      [(float(x), 0, 0) for x in np.linspace(-50, 0, 101)],
                      "舵机先插入，再从左侧沿底面滑入前安装桥，承托左耳", 0.5))
    paths.append(path("Pan bearing module", {n: PARTS[n] for n in PAN_MODULE},
                      {"01_Base": PARTS["01_Base"], "16_Bridge": PARTS["16_Pan_Front_Mount_Bridge"],
                       "MKS_Pan": REFS["MKS_Pan"]},
                      [(0, 0, float(z)) for z in range(50, -1, -1)], "预装轴承模块从上方套入真实25T轴", 1.0))
    paths.append(path("Tilt servo", {"MKS_Tilt": REFS["MKS_Tilt"]}, {"08_U_Frame": PARTS["08_U_Frame"]},
                      [(float(x), 0, 0) for x in range(70, -1, -1)], "从右侧穿入安装窗口，再锁M3耳槽螺钉", 1.0))
    paths.append(path("Tilt module", {n: PARTS[n] for n in TILT_MODULE},
                      {"08_U_Frame": PARTS["08_U_Frame"], "MKS_Tilt": REFS["MKS_Tilt"]},
                      [(float(x), 0, 0) for x in np.linspace(-7, 0, 29)],
                      "左支架未装；笼架/轴套先预紧，再向右套入6mm真实花键", 0.25))
    obs = {"08_U_Frame": PARTS["08_U_Frame"], "MKS_Tilt": REFS["MKS_Tilt"],
           **{n: PARTS[n] for n in TILT_MODULE}}
    paths.append(path("Left support slide", {"15_Left": PARTS["15_Removable_Left_Support"]}, obs,
                      [(float(x), 0, 2.2) for x in np.linspace(-30, 0, 61)],
                      "不装左轴承；支架抬高2.2mm，从左侧沿轴线滑入", 0.5))
    paths.append(path("Left support lower", {"15_Left": PARTS["15_Removable_Left_Support"]}, obs,
                      [(0, 0, float(z)) for z in np.linspace(2.2, 0, 23)],
                      "下落使两个定位柱入槽，后拧四颗M2.5", 0.1))
    bearing = bearing_envelope(DOC["hardware"][-1])
    left_obs = {**obs, "15_Left": PARTS["15_Removable_Left_Support"]}
    paths.append(path("Left bearing", {"6800": bearing}, left_obs,
                      [(float(x), 0, 0) for x in range(-30, 1)], "从左侧沿轴颈套入开式61800轴承", 1.0))
    paths.append(path("Camera", {"HERO8": CAMERA},
                      {n: s for n, s in ALL.items() if n != "10_HERO8_Top_Clamp"},
                      [(0, 0, float(z)) for z in range(60, -1, -1)], "顶压条卸下，折叠GoPro安装脚，从上方放入", 1.0))
    for i, spec in enumerate(DOC["nuts"], 1):
        if spec["part"] in ("01_Base", "16_Pan_Front_Mount_Bridge"):
            sign = 1 if spec["position"][1] > 0 else -1
            deltas = [(0, float(sign*y), 0) for y in np.linspace(40, 0, 81)]
        else:
            deltas = [(float(x), 0, 0) for x in range(-20, 1)]
        paths.append(path(f"M3 nut {i}", {"nut": nut_envelope(spec)},
                          {spec["part"]: PARTS[spec["part"]]}, deltas,
                          "安装舵机和运动模块前先放入M3防转螺母",
                          0.5 if spec["part"] in ("01_Base", "16_Pan_Front_Mount_Bridge") else 1.0))
    return {"paths": paths, "failures": [p for p in paths if not p["pass"]],
            "coverage": "Prescribed assembly order and sampled rigid paths; foam compression, wires, real tools excluded."}


def hardware_motion(hardware):
    failures = []
    tilt_body = REFS["MKS_Tilt"]-orient(cyl(6, 7.6), [53.0, 0, PZ], "-X")
    pan_body = REFS["MKS_Pan"]-cyl(6, 7, z=45.65)
    for angle in range(-90, 91, 5):
        for name, solid, group in hardware:
            if group == "tilt":
                moved = solid.translate([0, 0, -PZ]).rotate([angle, 0, 0]).translate([0, 0, PZ])
                obs = {n: s for n, s in PARTS.items() if DOC["printed_parts"][n]["group"] != "tilt"}
                obs.update({"MKS_Tilt_case": tilt_body, "MKS_Pan_case": pan_body})
                for other, vol in clashes(moved, obs):
                    failures.append(["tilt", angle, name, other, vol])
            if group != "fixed":
                moved = solid.rotate([0, 0, angle])
                obs = {n: s for n, s in PARTS.items() if DOC["printed_parts"][n]["group"] == "fixed"}
                obs["MKS_Pan_case"] = pan_body
                for other, vol in clashes(moved, obs):
                    failures.append(["pan", angle, name, other, vol])
    return {"range_deg": [-90, 90], "step_deg": 5, "failures": failures,
            "coverage": "Screws/washers/nuts/inserts/bearings against stationary printed parts and servo cases. Threads intentional."}


def spline_audit():
    rows = []
    for axis, hub_name, shaft in (
        ("Pan", "05_Pan_25T_Hub", REFS["MKS_Pan"] ^ cyl(4.1, 5.3, z=46.2)),
        ("Tilt", "11_Tilt_25T_Hub", REFS["MKS_Tilt"] ^ orient(cyl(4.1, 5.25), [52.4, 0, PZ], "-X")),
    ):
        hub = PARTS[hub_name]
        def shifted(degrees):
            if axis == "Pan":
                return shaft.rotate([0, 0, degrees])
            return shaft.translate([0, 0, -PZ]).rotate([degrees, 0, 0]).translate([0, 0, PZ])
        nominal = intersection(hub, shaft)
        half_pitch = intersection(hub, shifted(7.2))
        contact_angles = []
        for sign in (-1, 1):
            lo, hi = 0., 7.2
            for _ in range(12):
                mid = (lo+hi)/2
                if intersection(hub, shifted(sign*mid)) > 0.001:
                    hi = mid
                else:
                    lo = mid
            contact_angles.append(sign*hi)
        rows.append({"axis": axis, "nominal_overlap_mm3": nominal,
                     "half_pitch_deg": 7.2, "half_pitch_clash_mm3": half_pitch,
                     "CAD_unclamped_contact_angles_deg": contact_angles,
                     "profile_teeth_source": 25, "normal_allowance_mm": 0.06,
                     "full_tooth_length_bound_mm": 5.40,
                     "split_slot_width_mm": 0.6,
                     "note": "25T source profile with a split interrupting one sector. Not 25 equally loaded teeth; contact angles exclude clamp deformation/FDM error.",
                     "pass": nominal < THRESHOLD and half_pitch > 0.5})
    return {"results": rows, "pass": all(r["pass"] for r in rows),
            "physical_fit_backlash_strength": "Not measured"}


def axial_stop_audit():
    rows = []
    for axis, name, reference, rotor, delta in (
        ("Pan", "05_Pan_25T_Hub", REFS["MKS_Pan"],
         REFS["MKS_Pan"] ^ cyl(6, 7, z=45.65), [0, 0, -0.1]),
        ("Tilt", "11_Tilt_25T_Hub", REFS["MKS_Tilt"],
         REFS["MKS_Tilt"] ^ orient(cyl(6, 7), [53.0, 0, PZ], "-X"), [0.1, 0, 0]),
    ):
        moved = PARTS[name].translate(delta)
        tip_stop = intersection(moved, rotor)
        body_contact = intersection(moved, reference-rotor)
        rows.append({"axis": axis, "nominal_boss_clearance_mm": 0.30,
                     "attempted_seating_overtravel_mm": 0.10,
                     "shaft_tip_stop_interference_mm3": tip_stop,
                     "stationary_case_interference_mm3": body_contact,
                     "pass": tip_stop > 0.5 and body_contact < THRESHOLD})
    return {"results": rows, "left_retainer_to_journal_end_reserve_mm": 0.45,
            "pass": all(r["pass"] for r in rows),
            "coverage": "CAD positive shaft-tip stops engage before journal can rub stationary servo boss; real tightening and FDM still untested."}


def print_pose_audit():
    results = []
    ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
    for name in PARTS:
        mesh = trimesh.load(OUT/"stl"/(name+".stl"), force="mesh")
        tri = mesh.triangles
        bed = np.max(tri[:, :, 2], axis=1) < 0.02
        bed_area = float(mesh.area_faces[bed].sum())
        downward = (mesh.face_normals[:, 2] < -math.sqrt(0.5)) & ~bed
        with zipfile.ZipFile(OUT/"3mf"/(name+".3mf")) as archive:
            model_paths = [n for n in archive.namelist() if n.lower().endswith(".model")]
            assert len(model_paths) == 1, (name, model_paths)
            root = ET.fromstring(archive.read(model_paths[0]))
        verts = root.findall(".//m:vertex", ns)
        v3 = np.array([[float(v.attrib[a]) for a in ("x", "y", "z")] for v in verts])
        size_match = float(np.max(np.abs(np.ptp(v3, axis=0)-mesh.extents))) < 0.01
        row = {"part": name, "bed_contact_area_mm2": round(bed_area, 3),
               "downward_face_area_excluding_bed_mm2": round(float(mesh.area_faces[downward].sum()), 3),
               "print_rotation_deg": DOC["quality"][name]["print_rotation_deg"],
               "3mf_units": root.attrib.get("unit"), "3mf_matches_STL_size": size_match,
               "pass": bed_area > 10 and root.attrib.get("unit") == "millimeter" and size_match}
        results.append(row)
    return {"parts": results, "pass": all(r["pass"] for r in results),
            "coverage": "Actual exported print poses, bed faces and 3MF dimensions. Downward faces may be bridges/supports; no slicer/G-code run."}


def main():
    rows, hardware, fastener_failures = fastener_audit()
    tools = tool_audit()
    installation = installation_audit()
    motion = hardware_motion(hardware)
    spline = spline_audit()
    axial_stops = axial_stop_audit()
    printing = print_pose_audit()
    result = {"units": "mm", "manifest_sha256": hashlib.sha256((OUT/"manifest.json").read_bytes()).hexdigest(),
              "fasteners": rows, "hardware_static_failures": fastener_failures,
              "tools": tools, "installation": installation, "hardware_motion": motion,
              "spline": spline, "print_pose": printing,
              "axial_stops": axial_stops,
              "axial_design": {"pan_inner_stack_mm": 23.0, "pan_nominal_endplay_mm": 0.15,
                               "Pan_inner_contact_OD_mm": 23.2, "Pan_outer_retainer_ID_mm": 30.0,
                               "Tilt_inner_contact_OD_mm": 12.0, "Tilt_outer_retainer_ID_mm": 17.0,
                               "note": "用夹紧套消除明显窜动，保留自由转动，不给轴承施加强预紧；未实测"},
              "physical_validation": "Not printed, not load tested. No rated stall-torque or precision claim."}
    result["pass"] = (not (fastener_failures or tools["failures"] or installation["failures"] or motion["failures"])
                      and spline["pass"] and printing["pass"] and axial_stops["pass"])
    json_write(OUT/"assembly_checks.json", result)
    print("FASTENER FAILURES", json.dumps(fastener_failures, ensure_ascii=False), flush=True)
    print("TOOL/WALL FAILURES", json.dumps(tools["failures"], ensure_ascii=False), flush=True)
    print("INSTALLATION FAILURES", json.dumps(installation["failures"], ensure_ascii=False), flush=True)
    print("HARDWARE MOTION FAILURES", json.dumps(motion["failures"], ensure_ascii=False), flush=True)
    print("SPLINE CHECKS", json.dumps(spline, ensure_ascii=False), flush=True)
    print("AXIAL STOP CHECKS", json.dumps(axial_stops, ensure_ascii=False), flush=True)
    print("PRINT POSE / 3MF CHECKS", printing["pass"], flush=True)
    print("ASSEMBLY CHECKS", "PASS" if result["pass"] else "NEEDS CORRECTION", flush=True)
    return result


if __name__ == "__main__":
    main()
