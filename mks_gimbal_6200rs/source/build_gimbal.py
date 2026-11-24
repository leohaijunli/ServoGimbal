"""MKS HBL388 + HERO8: actual mechanical assembly, explicit mm coordinates.

Printed spline hubs and clamp collars; independent Pan bearings; supported
Tilt trunnion; M2.5 heat-set inserts at every printed-part bolted joint.
References are the supplied, unscaled STLs. These are prototype parts, not a
claim that printed splines can withstand the servo's full stall torque.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math

import manifold3d as m
import numpy as np
import trimesh

from geometry import (HERE, box, cyl, roundplate, gusset_xz, spline_tool,
                      spline_lead, insert_tool, hex_prism, export_part,
                      export_stl, json_write)
from bearing_design import (VERSION, BEARING, BORE, OD, WIDTH, SEAT_D, JOURNAL_D,
                            INNER_CONTACT_D, OUTER_RETAINER_ID, SPACER_BORE_D,
                            PAN_BEARING_Z, PAN_SPACER_Z, PAN_SPACER_LENGTH,
                            PAN_CAP_BOTTOM, PAN_CAP_TOP, PAN_STACK_TOP,
                            PAN_COUPLER_FACE_Z, PAN_KEY_FLOOR_Z,
                            PAN_KEY_MALE_AF, PAN_KEY_FEMALE_AF,
                            OUTER_ADAPTER_H, OUTER_ADAPTER_OD, OUTER_ADAPTER_PRINT_ID,
                            TILT_BEARING_OUTER_X, TILT_SUPPORT_INNER_X,
                            TILT_SUPPORT_THICKNESS, TILT_CAP_OUTER_X,
                            TILT_JOURNAL_END_X, TILT_RETAINER_OUTER_X,
                            TILT_CLAMP_SURFACE_X, assembly_parameters)

OUT = HERE / "output"
OUT.mkdir(parents=True, exist_ok=True)
SOURCE = (HERE/"reference" if (HERE/"reference/MKS-X8-HBL388.stl").is_file()
          else Path(r"\\Artemis\Users\HLi\Downloads\seeker"))

# Source frame: shaft axis (-10.5, Y, 0); no vertex-average alignment.
PAN_T = [10.5, 0, 25.25]
PITCH_Z = 140.0
TILT_T = [73.4, 0, PITCH_Z + 10.5]
RPAN = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
RTILT = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]

# Official HERO8 dimensions, NOT the earlier HERO7-size approximation.
GP_W, GP_H, GP_D = 66.3, 48.6, 28.4
CAM_BOT, CAM_TOP = PITCH_Z-GP_H/2, PITCH_Z+GP_H/2
BACK_F, BACK_B = GP_D/2 + 0.5, GP_D/2 + 3.5
SHELF_TOP, SHELF_BOT = CAM_BOT-0.5, CAM_BOT-3.5
CLAMP_Z = CAM_TOP + 1.5

PARTS, ROTATIONS, META, INSERTS, SCREWS, HARDWARE, NUTS = {}, {}, {}, [], [], [], []
UPPER_CAP_BOLTS = [(27, 0), (-27, 0), (0, 27), (0, -27)]
BASE_BOLTS = [(x, y) for x in (-37, 37) for y in (-35, 35)]
BRIDGE_BOLTS = [(x, y) for x in (-38, -28) for y in (-22, 22)]
LIGHTWEIGHT_INFO = {}


def base_protected_regions():
    """Retained load/interface regions, not an FEA or stall-torque rating."""
    protected = box(104, 88, 10, z=-1)  # full plate beneath the servo holder
    protected = protected + roundplate(100, 84, 6.4, 8, z=39.6)
    for x, y in BASE_BOLTS:
        protected = protected + cyl(8, 40, x, y, 6.8)
    for x, y in BRIDGE_BOLTS:
        protected = protected + cyl(7, 21, x, y, 6.8)
    for y in (-22, 22):
        protected = protected + box(32.4, 14.8, 4, x=-27.3, y=y, z=22.3)
    # Join neighbouring retained posts without trapping tiny blind cavities.
    for y in (-29, 29):
        protected = protected + box(6, 10, 19, x=-33, y=y, z=7.8)
    # Preserve right-ear bearing surfaces and both captive-nut access rails.
    protected = protected + box(14, 26, 30, x=36.75, z=6.8)
    protected = protected + box(9.6, 90, 5.2, x=36.75, z=27.8)
    # Keep 3.2mm guide walls immediately outside the unchanged case pocket.
    for y in (-13.2, 13.2):
        protected = protected + box(44, 3.2, 30, x=10.5, y=y, z=6.8)
    for x in (-72, 72):
        for y in (-52, 52):
            protected = protected + cyl(9, 10, x, y, -1)
    # Full-height ribs divide the through-windows in the base-plate wings.
    for x in (-66, 66):
        for y in (-28, 0, 28):
            protected = protected + box(28, 3.2, 10, x=x, y=y, z=-1)
    for y in (-54, 54):
        for x in (-36, 0, 36):
            protected = protected + box(3.2, 18, 10, x=x, y=y, z=-1)
    return protected


def carrier_structural_regions():
    # Keep the original 6mm seating plate, central ring, four screw towers and
    # four 5mm diagonal webs. Lower cap recess wall remains at least 3.4mm.
    keep = roundplate(90, 84, 6, 5, z=46)
    keep = keep + cyl(36.5, 33, z=46)
    for x, y in BASE_BOLTS:
        keep = keep + cyl(7, 33, x, y, 46)
        angle = math.degrees(math.atan2(y, x))
        keep = keep + box(26, 5, 27.5, x=41.5, z=51.5).rotate([0, 0, angle])
    return keep


def record_lightweight(name, before, after, protected):
    removed_protected = max(0.0, ((before ^ protected)-after).volume())
    addition = max(0.0, (after-before).volume())
    assert removed_protected < 0.05, (name, "protected material removed", removed_protected)
    assert addition < 0.05, (name, "unexpected added material", addition)
    assert after.volume() < before.volume()
    LIGHTWEIGHT_INFO[name] = {
        "before_volume_mm3": before.volume(), "after_volume_mm3": after.volume(),
        "removed_volume_mm3": before.volume()-after.volume(),
        "reduction_percent": (1-after.volume()/before.volume())*100,
        "protected_region_loss_mm3": removed_protected,
        "added_material_mm3": addition,
        "scope": "Subtractive relief outside retained load/interface regions; no strength certification"}
    return after


def add(name, solid, rotate=(0, 0, 0), group="fixed", explode=(0, 0, 0)):
    PARTS[name] = solid
    ROTATIONS[name] = rotate
    META[name] = {"group": group, "explode_mm": explode,
                  "category": "printed", "material": "PETG / PA", "print": True}
    print("BUILT", name, "volume", round(solid.volume(), 1), flush=True)
    return solid


def insert(part, pos, direction, through=0):
    INSERTS.append({"part": part, "position": list(pos), "direction": direction,
                    "thread": "M2.5", "OD_mm": 4.6, "length_mm": 5.7,
                    "pilot_D_mm": 4.0, "pilot_depth_mm": 6.2})
    return insert_tool(*pos, direction, through=through)


def screw(joint, surface, direction, length, thread=2.5, washer=0,
          washer_od=6.0, group="fixed", explode=(0, 0, 0),
          head_standard="DIN 912"):
    if head_standard == "ISO 7380-1":
        head_d, head_h, key = 5.7, 1.65, 2.0
    else:
        head_d, head_h, key = ((4.5, 2.5, 2.0) if thread == 2.5 else (5.5, 3.0, 2.5))
    SCREWS.append({"joint": joint, "bearing_surface": list(surface),
                   "direction": direction, "length_mm": length,
                   "thread_mm": thread, "washer_mm": washer,
                   "washer_OD_mm": washer_od,
                   "washer_ID_mm": 2.7 if thread == 2.5 else 3.2,
                   "head_standard": head_standard, "head_D_mm": head_d,
                   "head_H_mm": head_h, "hex_key_mm": key,
                   "group": group, "explode_mm": explode})


def nut(part, position, direction, group, explode):
    NUTS.append({"part": part, "position": list(position), "direction": direction,
                 "thread": "M3", "across_flats_mm": 5.5, "length_mm": 2.4,
                 "standard": "DIN 934", "group": group, "explode_mm": explode})


def cut_vertical_bolt(solid, x, y, z0, z1, head_face=None):
    solid = solid - cyl(1.4, z1-z0, x, y, z0)
    if head_face is not None:
        solid = solid - cyl(2.6, z1-head_face+0.5, x, y, head_face)
    return solid


def clamp_shape(lip=False):
    # Native C-clamp: split extends out of one side, not two disconnected halves.
    ring = cyl(14.0, 8.0) - cyl(10.05, 9.0, z=-0.5)
    if lip:
        ring = ring + (cyl(11.6, 2.2, z=7.9) - cyl(10.05, 3.0, z=7.5))
    ring = ring + box(14, 4.8, 8, x=15, y=2.8)
    ring = ring + box(14, 6.4, 8, x=15, y=-3.6)
    ring = ring - cyl(10.05, 11, z=-0.5)
    ring = ring - box(26, 0.8, 11.0 if lip else 8.7, x=13, y=0, z=-0.1)
    ring = ring - cyl(1.4, 7.0, x=17, y=-0.5, z=4.0, axis="Y")
    return ring


def build_base():
    name = "01_Base"
    base = roundplate(170, 130, 8, 14)
    base = base + roundplate(100, 84, 38.2, 8, z=7.8)
    # Case pocket: source body remains a real mesh. This is the receiving void.
    base = base - roundplate(44.0, 23.2, 42, 1.0, x=10.5, z=5.5)
    # All ear surfaces rest on Z34.30; no volume overlap with servo ears.
    base = base - box(61.0, 23.2, 13, x=10.5, z=34.3)
    base = base - cyl(25.0, 1.0, z=45.5)  # full-angle rotating collar clearance
    # Connector/wire path on native -X side.
    base = base - box(60, 10, 10, x=-34, z=5)
    # Real connector cannot pass through a closed front wall during top-loading.
    # The front mounting bridge is installed afterwards, below the left ear.
    base = base - box(41.5, 26.6, 42, x=-32.25, z=5.0)
    base = base - box(41.7, 32.4, 21, x=-32.15, z=26.3)
    for y in (-22, 22):
        base = base - box(41.7, 14.4, 21, x=-32.15, y=y, z=26.3)
    for z in (14, 22):
        for x in (-22, -8, 6, 20, 34):
            vent = box(8, 90, 4, x=x, z=z)
            for bx, by in BRIDGE_BOLTS:
                vent = vent - cyl(4, 12, bx, by, 19)
            base = base - vent  # preserve insert bosses while opening 8mm vents
    for x in (-72, 72):
        for y in (-52, 52):
            base = base - cyl(2.25, 10, x, y, -1)
    for x in (36.75,):
        for y in (-5.0, 5.0):
            base = base - cyl(1.7, 8, x, y, 27.0)
            # M3 DIN934 captive nuts slide in from the front/back. They are not
            # printed threads and are not confused with the M2.5 insert joints.
            base = base - hex_prism(5.8, 2.6, (x, y, 31.4), "-Z")
            base = base - box(5.8, 40, 2.6, x=x, y=25 if y > 0 else -25, z=28.8)
            nut(name, (x, y, 31.4), "-Z", "fixed", (0, 0, 28))
            screw("Pan mounting ear", (x, y, 37.3), "-Z", 10, thread=3.0,
                  washer=0.5, washer_od=7.0, explode=(0, 0, 28))
    for x, y in BASE_BOLTS:
        base = base - insert(name, (x, y, 46), "-Z")
    for x, y in BRIDGE_BOLTS:
        base = base - insert(name, (x, y, 26.3), "-Z")
    before = base
    protected = base_protected_regions()
    cutters = None
    # Side windows have a 45-degree roof; 4mm vertical webs remain between them.
    # The first edge must not be exactly tangent to the retained post cylinders
    # at X=-45, which would merge into four-face edges in an STL.
    for x in (-38.4, -20, -2, 16, 34):
        window = gusset_xz([(x-7, 11), (x+7, 11), (x+7, 26),
                            (x, 33), (x-7, 26)], -45, 90)
        cutters = window if cutters is None else cutters+window
    for x in (-66, 66):
        cutters = cutters + roundplate(24, 88, 10, 3, x=x, z=-1)
    for y in (-54, 54):
        cutters = cutters + roundplate(108, 14, 10, 3, y=y, z=-1)
    base = record_lightweight(name, before, before-(cutters-protected), protected)
    add(name, base)

    name = "16_Pan_Front_Mount_Bridge"
    bridge = box(31.6, 32, 8.0, x=-27.3, z=26.3)
    for y in (-22, 22):
        bridge = bridge + box(31.6, 14, 8.0, x=-27.3, y=y, z=26.3)
    for y in (-5, 5):
        bridge = bridge - cyl(1.7, 9.0, -15.75, y, 25.8)
        bridge = bridge - hex_prism(5.8, 2.6, (-15.75, y, 31.4), "-Z")
        bridge = bridge - box(5.8, 25, 2.6, x=-15.75, y=17.5 if y > 0 else -17.5, z=28.8)
        nut(name, (-15.75, y, 31.4), "-Z", "fixed", (-45, 0, 20))
        screw("Pan mounting ear", (-15.75, y, 37.3), "-Z", 10, thread=3.0,
              washer=0.5, washer_od=7.0, explode=(0, 0, 28))
    for x, y in BRIDGE_BOLTS:
        bridge = cut_vertical_bolt(bridge, x, y, 25.8, 34.8, head_face=31.8)
        screw("Front bridge to base", (x, y, 31.8), "-Z", 10, explode=(-45, 0, 20))
    add(name, bridge, explode=(-45, 0, 20))


def build_pan_support():
    name = "02_Pan_Bearing_Carrier"
    carrier = roundplate(90, 84, 6.0, 5, z=46.0)
    # Four connected outer columns support the bearing block above the lower
    # retainer. The retainer must fit through the bottom opening, not into solid.
    carrier = carrier + roundplate(90, 84, 27.5, 5, z=51.5)
    carrier = carrier - cyl(OUTER_RETAINER_ID/2, 35, z=45.5)
    carrier = carrier - cyl(33.1, 10.5, z=45.5)  # removable cap + C-clamp access
    for z in PAN_BEARING_Z:
        carrier = carrier - cyl(SEAT_D/2, WIDTH+0.05, z=z-0.025)
    for z in (PAN_CAP_BOTTOM, PAN_CAP_TOP-OUTER_ADAPTER_H):
        carrier = carrier - cyl(OUTER_ADAPTER_OD/2+0.05, OUTER_ADAPTER_H+0.05, z=z-0.025)
    # Clamp screw can be reached before/after placing this carrier on the servo.
    carrier = carrier - box(14, 44, 6.8, x=17, y=22, z=46.8)
    for x, y in BASE_BOLTS:
        carrier = cut_vertical_bolt(carrier, x, y, 45.5, 52.5)
        carrier = carrier - cyl(3.4, 28.0, x, y, 52.0)
        screw("Carrier to base", (x, y, 52.0), "-Z", 10, explode=(0, 0, 40))
    for x in (-19, 19):
        for y in (-19, 19):
            carrier = carrier - insert(name, (x, y, 56.0), "+Z", through=7.0)
    for x, y in UPPER_CAP_BOLTS:
        carrier = carrier - insert(name, (x, y, 79.0), "-Z", through=9.0)
    before = carrier
    keep = carrier_structural_regions()
    carrier = record_lightweight(name, before, before ^ keep, keep)
    add(name, carrier, rotate=(180, 0, 0), group="fixed", explode=(0, 0, 40))

    name = "03_Pan_Lower_Cap"
    cap = cyl(33, 2.0, z=52.0) - cyl(24.0, 3.0, z=51.5)
    web = cyl(33, 2.1, z=53.9) - cyl(15.0, 3.0, z=53.5)
    cap = cap + web
    for x in (-19, 19):
        for y in (-19, 19):
            cap = cut_vertical_bolt(cap, x, y, 51.5, 56.5)
            screw("Lower bearing retainer", (x, y, 52), "+Z", 10, explode=(0, 0, 28))
    add(name, cap, rotate=(180, 0, 0), explode=(0, 0, 28))

    name = "04_Pan_Upper_Cap"
    cap = cyl(33, 3.0, z=79) - cyl(15.0, 4.0, z=78.5)
    for x, y in UPPER_CAP_BOLTS:
        cap = cut_vertical_bolt(cap, x, y, 78.5, 82.5, head_face=79.5)
        screw("Upper bearing retainer", (x, y, 79.5), "-Z", 8, explode=(0, 0, 60))
    add(name, cap, explode=(0, 0, 60))

    spacer = cyl(INNER_CONTACT_D/2, PAN_SPACER_LENGTH, z=PAN_SPACER_Z)
    spacer = spacer - cyl(SPACER_BORE_D/2, PAN_SPACER_LENGTH+1.0, z=PAN_SPACER_Z-0.5)
    add("07_Pan_Inner_Spacer", spacer, group="pan", explode=(0, 0, 50))


def build_pan_drive():
    name = "05_Pan_25T_Hub"
    # A detachable flanged shaft lets CLOSED ID10 bearings slide in from below.
    # The original thick OD20 25T socket becomes the small separate part 17.
    # Naming 05 is retained so the U-frame/flange interface stays unchanged.
    journal_start = PAN_COUPLER_FACE_Z + 0.05
    hub = cyl(JOURNAL_D/2, PAN_STACK_TOP+0.1-journal_start, z=journal_start)
    hub = hub + hex_prism(PAN_KEY_MALE_AF, journal_start+0.15-PAN_KEY_FLOOR_Z,
                          (0, 0, PAN_KEY_FLOOR_Z))
    hub = hub + cyl(INNER_CONTACT_D/2, 82.4-PAN_STACK_TOP, z=PAN_STACK_TOP)
    hub = hub + cyl(34, 6.0, z=82.3)
    hub = hub - cyl(1.65, 38, z=51.5)
    # M3x40: 4.65 mm nominal engagement and 1.35 mm reserve to thread bottom.
    hub = hub - cyl(3.25, 2.0, z=87.0)
    for x, y in UPPER_CAP_BOLTS:
        hub = hub - cyl(2.6, 7.0, x, y, 81.8)
    for x in (-23, 23):
        for y in (-17, 17):
            hub = hub - insert(name, (x, y, 88.3), "-Z", through=6.5)
    add(name, hub, (180, 0, 0), group="pan", explode=(0, 0, 68))
    screw("Pan hub OEM shaft retainer", (0, 0, 87), "-Z", 40, thread=3.0,
          group="pan", explode=(0, 0, 68), head_standard="ISO 7380-1")

    name = "06_Pan_Clamp_Collar"
    collar = clamp_shape(lip=True).translate([0, 0, 45.75])
    collar = collar - insert(name, (17, -6.8, 49.75), "+Y", through=6.5)
    add(name, collar, group="pan", explode=(28, 0, 38))
    screw("Pan spline clamp", (17, 5.7, 49.75), "-Y", 12, washer=0.5,
           washer_od=6.0, group="pan", explode=(28, 0, 38))

    name = "17_Pan_25T_Coupler"
    # Preserve OD20 beneath BOTH the original clamp body and its contact lip.
    # This lets old 06 act over 3.45mm of the new 4.45mm key, rather than relying
    # on an unclamped narrow neck to remove the keyed joint's clearance.
    coupler = cyl(9.99, 9.90, z=45.95)
    coupler = coupler + cyl(INNER_CONTACT_D/2, PAN_COUPLER_FACE_Z-55.8, z=55.8)
    # Original measured 25T profile, original positive stop and boss clearance.
    coupler = coupler - spline_tool(45.25, 6.4) - spline_lead(45.9)
    coupler = coupler - hex_prism(PAN_KEY_FEMALE_AF, PAN_COUPLER_FACE_Z-PAN_KEY_FLOOR_Z+0.5,
                                  (0, 0, PAN_KEY_FLOOR_Z))
    coupler = coupler - cyl(1.65, 7, z=51.5)
    coupler = coupler - box(22, 0.6, PAN_COUPLER_FACE_Z-45.5+0.1, x=11, z=45.5)
    # Socket entrance on the bed, 8AF entrance upwards: bridge only the small
    # socket roof; never generate support in either tooth/key profile.
    add(name, coupler, group="pan", explode=(0, 0, 28))


def build_outer_adapters():
    # All three are the SAME print file, not three cosmetically different STLs.
    local = cyl(OUTER_ADAPTER_OD/2, OUTER_ADAPTER_H)
    local = local-cyl(OUTER_RETAINER_ID/2, OUTER_ADAPTER_H+1, z=-0.5)
    for name, z, explode in (
        ("18A_Pan_Lower_Outer_Adapter", PAN_CAP_BOTTOM, (0, 0, 28)),
        ("18B_Pan_Upper_Outer_Adapter", PAN_CAP_TOP-OUTER_ADAPTER_H, (0, 0, 60)),
    ):
        add(name, local.translate([0, 0, z]), group="fixed", explode=explode)
        META[name]["print_file_id"] = OUTER_ADAPTER_PRINT_ID
    name = "18C_Tilt_Outer_Adapter"
    add(name, local.rotate([0, 90, 0]).translate([TILT_BEARING_OUTER_X-OUTER_ADAPTER_H, 0, PITCH_Z]),
        (0, -90, 0), group="pan", explode=(-32, 0, 90))
    META[name]["print_file_id"] = OUTER_ADAPTER_PRINT_ID


def build_frame():
    name = "08_U_Frame"
    floor = roundplate(140, 66, 8.0, 9, z=88.3)
    right = box(6.0, 32, 85.0, x=58.85, z=96.1)
    frame = floor + right
    for y0 in (-20, 12):
        frame = frame + gusset_xz([(44, 95.8), (61.85, 95.8), (61.85, 119)], y0, 8)
    # Servo case slides in from +X. Ears bear on X61.85, outside the window.
    frame = frame - box(9.0, 23.2, 44.2, x=58.85, z=PITCH_Z-11.6)
    for y in (-5, 5):
        for z in (PITCH_Z-15.75, PITCH_Z+36.75):
            frame = frame - cyl(1.7, 8.0, x=54.0, y=y, z=z, axis="X")
            frame = frame - hex_prism(5.8, 3.0, (58.55, y, z), "-X")
            nut(name, (58.55, y, z), "-X", "pan", (0, 0, 90))
            screw("Tilt mounting ear", (64.85, y, z), "-X", 10, thread=3.0,
                  washer=0.5, washer_od=7.0, group="pan", explode=(45, 0, 95))
    for x in (-62, -46):
        for y in (-23, 23):
            frame = frame - insert(name, (x, y, 96.3), "-Z")
    for y in (-10, 10):
        frame = frame - cyl(2.15, 2.7, -58, y, 94.1)
    for x in (-23, 23):
        for y in (-17, 17):
            frame = cut_vertical_bolt(frame, x, y, 87.8, 96.8, head_face=93.8)
            screw("U-frame to Pan flange", (x, y, 93.8), "-Z", 10,
                  group="pan", explode=(0, 0, 90))
    frame = frame - cyl(5.0, 9.0, z=88.0)
    add(name, frame, group="pan", explode=(0, 0, 90))

    # The left support MUST be removable. Otherwise the pre-bolted cradle/hub
    # cannot slide onto the real 6 mm spline without hitting an integral tower.
    name = "15_Removable_Left_Support"
    support = box(24, 56, 6.0, x=-54, z=96.3)
    support = support + box(TILT_SUPPORT_THICKNESS, 40, 62.1,
                            x=(TILT_BEARING_OUTER_X+TILT_SUPPORT_INNER_X)/2, z=102.1)
    for y0 in (-20, 12):
        support = support + gusset_xz([(-65, 102.1), (-44, 102.1), (-44, 123)], y0, 8)
    for y in (-10, 10):
        support = support + cyl(2, 2.2, -58, y, 94.3)
    for x in (-62, -46):
        for y in (-23, 23):
            support = cut_vertical_bolt(support, x, y, 95.8, 102.8)
            screw("Left support to deck", (x, y, 102.3), "-Z", 10,
                  group="pan", explode=(-32, 0, 90))
    support = support - cyl(SEAT_D/2, WIDTH+0.05,
                            x=TILT_BEARING_OUTER_X-0.025, z=PITCH_Z, axis="X")
    support = support - cyl(OUTER_RETAINER_ID/2, TILT_SUPPORT_THICKNESS+1.0,
                            x=TILT_BEARING_OUTER_X-0.5, z=PITCH_Z, axis="X")
    for y in (-14, 14):
        for z in (PITCH_Z-14, PITCH_Z+14):
            support = support - insert(name, (TILT_BEARING_OUTER_X, y, z), "+X", through=12.5)
    add(name, support, (0, -90, 0), group="pan", explode=(-32, 0, 90))

    name = "13_Left_Bearing_Cap"
    # Reuse the ORIGINAL cap, only move it outwards by 5mm in the assembly.
    # Adapter 18 keeps its ID17 face 1mm clear of the bearing seal/inner ring.
    cap = box(2.0, 40, 40, x=-53.0, z=PITCH_Z-20)
    cap = cap - cyl(8.5, 3.0, x=-54.5, z=PITCH_Z, axis="X")
    for y in (-14, 14):
        for z in (PITCH_Z-14, PITCH_Z+14):
            cap = cap - cyl(1.4, 3.0, x=-54.5, y=y, z=z, axis="X")
            screw("Left bearing retainer", (TILT_CAP_OUTER_X, y, z), "+X", 8,
                  group="pan", explode=(-32, 0, 90))
    delta = TILT_CAP_OUTER_X-(-54.0)
    add(name, cap.translate([delta, 0, 0]), (0, 90, 0), group="pan", explode=(-32, 0, 90))
    META[name]["reuse_translation_mm"] = [delta, 0, 0]

    # OD14.4 contact lip follows the named SKF 6200 abutment range; the unknown
    # user-brand seal face must still be physically checked before tightening.
    retainer = cyl(7.75, 2.05, x=TILT_RETAINER_OUTER_X, z=PITCH_Z, axis="X")
    tube = cyl(7.75, 1.5, x=TILT_BEARING_OUTER_X-1.9, z=PITCH_Z, axis="X")
    tube = tube + cyl(INNER_CONTACT_D/2, 0.5,
                      x=TILT_BEARING_OUTER_X-0.5, z=PITCH_Z, axis="X")
    tube = tube - cyl(SPACER_BORE_D/2, 2.5,
                      x=TILT_BEARING_OUTER_X-2.0, z=PITCH_Z, axis="X")
    retainer = retainer + tube
    retainer = retainer - cyl(1.4, 5.0, x=TILT_CLAMP_SURFACE_X, z=PITCH_Z, axis="X")
    add("14_Left_Inner_Spacer", retainer, (0, -90, 0), group="tilt", explode=(-24, 0, 120))


def build_cradle():
    name = "09_HERO8_Cradle"
    cradle = box(GP_W+2.5, 3.0, CLAMP_Z-SHELF_BOT, y=(BACK_F+BACK_B)/2, z=SHELF_BOT)
    cradle = cradle + box(GP_W+1.0, 32.8, 3.0, y=-0.3, z=SHELF_BOT)
    cradle = cradle + box(28, 2.0, 5.0, y=-15.7, z=SHELF_TOP-1.5)
    for s in (-1, 1):
        cradle = cradle + box(1.4, 5.7, 40.0, x=s*(GP_W/2+1.2), y=14.85,
                              z=PITCH_Z-20)
    # Both webs overlap the back plate; none extend inside the camera envelope.
    cradle = cradle + cyl(19, 6.5, x=34.0, z=PITCH_Z, axis="X")
    cradle = cradle + box(6.7, 9.7, 24, x=37.15, y=12.85, z=PITCH_Z-12)
    cradle = cradle + cyl(16, 6.5, x=-40.5, z=PITCH_Z, axis="X")
    cradle = cradle + box(6.7, 9.7, 24, x=-37.15, y=12.85, z=PITCH_Z-12)
    # Extend the OD10 journal 4mm outwards for the thicker 6200RS, preserving
    # the camera centre, inner shoulder face, servo pose and support foot holes.
    cradle = cradle + cyl(INNER_CONTACT_D/2, 6.7, x=-47.0, z=PITCH_Z, axis="X")
    cradle = cradle + cyl(JOURNAL_D/2, -46.8-TILT_JOURNAL_END_X,
                          x=TILT_JOURNAL_END_X, z=PITCH_Z, axis="X")
    cradle = cradle - insert(name, (TILT_JOURNAL_END_X, 0, PITCH_Z), "+X", through=13.5)
    cradle = cradle - cyl(3.4, 7.5, x=33.5, z=PITCH_Z, axis="X")
    # Right flange: 4 actual threaded joints, not a loose polygonal socket.
    a = 14/math.sqrt(2)
    for y in (-a, a):
        for dz in (-a, a):
            cradle = cradle - insert(name, (40.5, y, PITCH_Z+dz), "-X", through=6.5)
    for x in (-20, 20):
        cradle = cradle + box(12, 9, 13.0, x=x, y=20.8, z=CLAMP_Z-13.0)
        cradle = cradle - insert(name, (x, 20.8, CLAMP_Z), "-Z", through=9)
    # Right flange on bed, left bearing journal printed vertically for roundness.
    add(name, cradle, (0, 90, 0), group="tilt", explode=(0, 0, 145))
    screw("Left journal inner-race clamp", (TILT_CLAMP_SURFACE_X, 0, PITCH_Z), "+X", 10,
          washer=0.5, washer_od=6.0, group="tilt", explode=(-24, 0, 120))

    name = "10_HERO8_Top_Clamp"
    clamp = roundplate(56, 29.3, 4, 1.5, y=10.65, z=CLAMP_Z)
    # Conservative shutter-button keepout: top/front/right remains open.
    clamp = clamp - box(31, 15.2, 5, x=24.5, y=2.4, z=CLAMP_Z-0.5)
    for x in (-20, 20):
        clamp = cut_vertical_bolt(clamp, x, 20.8, CLAMP_Z-0.5, CLAMP_Z+4.5,
                                  head_face=CLAMP_Z+1.5)
        screw("HERO8 removable top clamp", (x, 20.8, CLAMP_Z+1.5), "-Z", 10,
              group="tilt", explode=(0, 0, 235))
    add(name, clamp, group="tilt", explode=(0, 0, 235))


def build_tilt_drive():
    name = "11_Tilt_25T_Hub"
    local = cyl(9.99, 8.6, z=0.2) + cyl(19.0, 4.0, z=8.4)
    # Global shaft tip X47.0 is the axial stop; journal entry X52.7 remains
    # 0.30mm away from the stationary bearing boss at X53.0.
    local = local - spline_tool(-0.5, 6.4) - spline_lead(0.15)
    local = local - cyl(1.65, 8.0, z=5.7)
    local = local - cyl(3.25, 3.2, z=9.4)
    local = local - box(22, 0.6, 7.3, x=11, z=-0.1)
    a = 14/math.sqrt(2)
    for x in (-a, a):
        for y in (-a, a):
            local = local - cyl(1.4, 5.0, x, y, 8.0)
            local = local - cyl(2.6, 2.55, x, y, 8.35)
    hub = local.rotate([0, -90, 0]).translate([52.9, 0, PITCH_Z])
    add(name, hub, (0, -90, 0), group="tilt", explode=(28, 0, 145))
    for y in (-a, a):
        for dz in (-a, a):
            screw("Tilt flange to cradle", (42.0, y, PITCH_Z+dz), "-X", 6,
                  group="tilt", explode=(28, 0, 145))
    screw("Tilt hub OEM shaft retainer", (43.5, 0, PITCH_Z), "+X", 8,
          thread=3.0, group="tilt", explode=(28, 0, 145), head_standard="ISO 7380-1")

    name = "12_Tilt_Clamp_Collar"
    local = clamp_shape(lip=False)
    # Insert transform: rotate source -Y clamp position with the same rigid frame.
    local = local - insert_tool(17, -6.8, 4, "+Y", through=6.5)
    INSERTS.append({"part": name, "position": [48.7, -6.8, PITCH_Z+17],
                    "direction": "+Y", "thread": "M2.5", "OD_mm": 4.6,
                    "length_mm": 5.7, "pilot_D_mm": 4.0, "pilot_depth_mm": 6.2})
    collar = local.rotate([0, -90, 0]).translate([52.7, 0, PITCH_Z])
    add(name, collar, (0, -90, 0), group="tilt", explode=(42, 0, 145))
    screw("Tilt spline clamp", (48.7, 5.7, PITCH_Z+17), "-Y", 12,
          washer=0.5, washer_od=6.0, group="tilt", explode=(42, 0, 145))


def load_reference(path, rotation, translation):
    mesh = trimesh.load(path, force="mesh", process=True)
    mesh.vertices = np.asarray(mesh.vertices) @ np.asarray(rotation).T + translation
    # Analysis-only conversion; original STL and Blender mesh stay unchanged.
    solid = m.Manifold(m.Mesh(np.asarray(mesh.vertices, dtype=np.float32),
                              np.asarray(mesh.faces, dtype=np.uint32)))
    if solid.status() != m.Error.NoError:
        return None, {"status": str(solid.status()), "mesh_watertight": bool(mesh.is_watertight)}
    return solid, {"status": "OK", "components": len(solid.decompose())}


def hardware_specs():
    # Separate inner/outer bearing envelopes prevent false binding in QA.
    for index, z in enumerate(PAN_BEARING_Z):
        HARDWARE.append({"name": f"6200RS_Pan_{'lower' if index == 0 else 'upper'}",
                         "kind": "bearing", "id": BORE, "od": OD, "length": WIDTH,
                         "axis": "Z", "position": [0, 0, z],
                         "product": BEARING["product"],
                         "inner_shoulder_D_mm": BEARING["inner_shoulder_d2_approx"],
                         "outer_recess_D_mm": BEARING["outer_recess_D2_approx"],
                         "group": "fixed", "explode_mm": [0, 0, 45 if index == 0 else 55]})
    HARDWARE.append({"name": "6200RS_Tilt_left", "kind": "bearing", "id": BORE,
                     "od": OD, "length": WIDTH, "axis": "X",
                     "position": [TILT_BEARING_OUTER_X, 0, PITCH_Z],
                     "product": BEARING["product"],
                     "inner_shoulder_D_mm": BEARING["inner_shoulder_d2_approx"],
                     "outer_recess_D_mm": BEARING["outer_recess_D2_approx"],
                     "group": "pan", "explode_mm": [-24, 0, 90]})


def mesh_qa_and_interference(refs):
    # Excludes purposely contacting bearing envelopes / thermal insert material.
    allowed_contact = set()
    failures = []
    results = []
    names = list(PARTS)
    for i, a in enumerate(names):
        for b in names[i+1:]:
            v = (PARTS[a] ^ PARTS[b]).volume()
            if v > 0.05 and (a, b) not in allowed_contact:
                failures.append([a, b, float(v)])
            if v > 0.001:
                results.append([a, b, float(v)])
    for rname, ref in refs.items():
        if ref is None:
            continue
        for name, part in PARTS.items():
            v = (part ^ ref).volume()
            if v > 0.05:
                failures.append([name, rname, float(v)])
                results.append([name, rname, float(v)])
    print("STATIC CLASHES", failures, flush=True)

    # Camera envelope is conservative and uses official maximum dimensions.
    camera = box(GP_W, GP_D, GP_H, z=CAM_BOT)
    for name in ("09_HERO8_Cradle", "10_HERO8_Top_Clamp"):
        v = (PARTS[name] ^ camera).volume()
        if v > 0.05:
            failures.append([name, "HERO8_envelope", float(v)])

    tilt_hits = []
    rotating = [n for n in names if META[n]["group"] == "tilt"]
    static = [n for n in names if META[n]["group"] != "tilt"]
    tilt_shaft, tilt_body = None, None
    if refs.get("MKS_Tilt") is not None:
        # The 25T output is a rotor, NOT a stationary collision obstacle.
        tilt_shaft = refs["MKS_Tilt"] ^ box(7.5, 12, 12, x=49.25, z=PITCH_Z-6)
        tilt_body = refs["MKS_Tilt"] - tilt_shaft
    for angle in range(-90, 91, 5):
        shaft_at_angle = None
        if tilt_shaft is not None:
            shaft_at_angle = tilt_shaft.translate([0, 0, -PITCH_Z]).rotate([angle, 0, 0]).translate([0, 0, PITCH_Z])
        for name in rotating + ["HERO8_envelope"]:
            p = camera if name == "HERO8_envelope" else PARTS[name]
            moved = p.translate([0, 0, -PITCH_Z]).rotate([angle, 0, 0]).translate([0, 0, PITCH_Z])
            for other in static:
                v = (moved ^ PARTS[other]).volume()
                if v > 0.05:
                    tilt_hits.append([angle, name, other, float(v)])
            if tilt_body is not None:
                v = (moved ^ tilt_body).volume() + (moved ^ shaft_at_angle).volume()
                if v > 0.05:
                    tilt_hits.append([angle, name, "MKS_Tilt", float(v)])
    print("TILT SWEEP CLASHES", tilt_hits, flush=True)
    pan_hits = []
    pan_rotating = [n for n in names if META[n]["group"] != "fixed"]
    pan_static = [n for n in names if META[n]["group"] == "fixed"]
    pan_shaft, pan_body = None, None
    if refs.get("MKS_Pan") is not None:
        pan_shaft = refs["MKS_Pan"] ^ box(12, 12, 7.0, z=45.65)
        pan_body = refs["MKS_Pan"] - pan_shaft
    for angle in range(-90, 91, 5):
        for name in pan_rotating + ["HERO8_envelope", "MKS_Tilt"]:
            p = camera if name == "HERO8_envelope" else (refs.get(name) if name == "MKS_Tilt" else PARTS[name])
            if p is None:
                continue
            moved = p.rotate([0, 0, angle])
            for other in pan_static:
                v = (moved ^ PARTS[other]).volume()
                if v > 0.05:
                    pan_hits.append([angle, name, other, float(v)])
            if pan_body is not None:
                v = (moved ^ pan_body).volume() + (moved ^ pan_shaft.rotate([0, 0, angle])).volume()
                if v > 0.05:
                    pan_hits.append([angle, name, "MKS_Pan", float(v)])
    print("PAN SWEEP CLASHES", pan_hits, flush=True)
    # Continuous floor clearance from the maximum radial envelope, not sampling.
    rad = 0.0
    for name in rotating + ["HERO8_envelope"]:
        p = camera if name == "HERO8_envelope" else PARTS[name]
        vs = np.asarray(p.to_mesh().vert_properties)[:, :3]
        rad = max(rad, float(np.hypot(vs[:, 1], vs[:, 2]-PITCH_Z).max()))
    floor_clearance = PITCH_Z-rad-96.3
    return {"static_failures": failures, "nonzero_intersections_mm3": results,
            "tilt_sweep": {"range_deg": [-90, 90], "step_deg": 5,
                           "failures": tilt_hits},
            "pan_sweep": {"range_deg": [-90, 90], "step_deg": 5,
                          "failures": pan_hits},
            "kinematics": "Servo cases fixed to their mounts; output splines rotate together with driven hubs",
            "continuous_floor_clearance_bound_mm": floor_clearance,
            "max_tilt_radius_mm": rad}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parts", help="Comma-separated changed parts; reuse validated existing meshes for the rest")
    parser.add_argument("--repose", help="Comma-separated original parts to move rigidly while retaining exact STL/3MF print files")
    args = parser.parse_args()
    previous = json.loads((OUT/"manifest.json").read_text(encoding="utf-8")) if (args.parts or args.repose) else None
    evidence = json.loads((HERE / "reference/reference_evidence.json").read_text())
    stl = SOURCE / "MKS-X8-HBL388.stl"
    assert hashlib.sha256(stl.read_bytes()).hexdigest() == evidence["sources"]["servo"]["supplied_stl_sha256"]
    build_base()
    build_pan_support()
    build_pan_drive()
    build_frame()
    build_cradle()
    build_tilt_drive()
    build_outer_adapters()
    hardware_specs()

    quality = {}
    reposed = set(args.repose.split(",")) if args.repose else set()
    selected = set(args.parts.split(",")) if args.parts else set(PARTS)-reposed
    assert not (selected & reposed), "Reposed parts must not also be selected for print-file export"
    assert selected <= set(PARTS), ("Unknown parts", selected-set(PARTS))
    assert reposed <= set(PARTS)
    printed_ids = set()
    for name in list(PARTS):
        print_id = META[name].get("print_file_id", name)
        if name in selected:
            PARTS[name], quality[name] = export_part(name, PARTS[name], OUT, ROTATIONS[name],
                                                     print_name=print_id, export_print=print_id not in printed_ids)
        elif name in reposed:
            old = previous["quality"][name]
            for folder, extension in (("stl", ".stl"), ("assembly_meshes", ".stl"), ("3mf", ".3mf")):
                assert hashlib.sha256((OUT/folder/(name+extension)).read_bytes()).hexdigest() == old["file_sha256"][folder]
            baseline, status = load_reference(OUT/"assembly_meshes"/(name+".stl"), np.eye(3), [0, 0, 0])
            assert baseline is not None, status
            moved = baseline.translate(META[name]["reuse_translation_mm"])
            difference = (PARTS[name]-moved).volume()+(moved-PARTS[name]).volume()
            assert difference < 0.05, (name, "Repose must not change physical geometry", difference)
            PARTS[name], quality[name] = export_part(name, moved, OUT, ROTATIONS[name], export_print=False)
            quality[name]["validation_reuse"] = "Original STL/3MF bytes retained; original assembly triangles only translated rigidly"
        else:
            quality[name] = previous["quality"][name]
            assert all(c["pass"] for c in quality[name]["checks"].values())
            for folder, extension in (("stl", ".stl"), ("assembly_meshes", ".stl"), ("3mf", ".3mf")):
                file_id = name if folder == "assembly_meshes" else print_id
                actual_hash = hashlib.sha256((OUT/folder/(file_id+extension)).read_bytes()).hexdigest()
                assert actual_hash == quality[name]["file_sha256"][folder], (name, folder, "reused mesh changed")
            quality[name]["file"] = str(OUT/"stl"/(print_id+".stl"))
            quality[name]["validation_reuse"] = "Unchanged baseline files SHA256-checked before reuse"
            PARTS[name], existing_status = load_reference(OUT/"assembly_meshes"/(name+".stl"),
                                                        np.eye(3), [0, 0, 0])
            assert PARTS[name] is not None, (name, existing_status)
        quality[name]["file_sha256"] = {
            folder: hashlib.sha256((OUT/folder/((name if folder == "assembly_meshes" else print_id)+extension)).read_bytes()).hexdigest()
            for folder, extension in (("stl", ".stl"), ("assembly_meshes", ".stl"), ("3mf", ".3mf"))}
        print("PRINT QA", name, "PASS", quality[name]["size_mm"], flush=True)
        printed_ids.add(print_id)

    pan, pqa = load_reference(stl, RPAN, PAN_T)
    tilt, tqa = load_reference(stl, RTILT, TILT_T)
    refs = {"MKS_Pan": pan, "MKS_Tilt": tilt}
    audit = mesh_qa_and_interference(refs)
    manifest = {"units": "mm", "design_version": VERSION,
                "go_pro": {"model": "HERO8 Black", "size_mm": [GP_W, GP_H, GP_D],
                           "source": "https://community.gopro.com/s/article/HERO8-Black-Camera-Size-and-Weight",
                           "centre": [0, 0, PITCH_Z]},
                "servo_source": str(stl), "servo_sha256": evidence["sources"]["servo"]["supplied_stl_sha256"],
                "servo_placements": {
                    "MKS_Pan": {"rotation_matrix": RPAN, "translation": PAN_T,
                                "group": "fixed", "explode_mm": [0, 0, 18]},
                    "MKS_Tilt": {"rotation_matrix": RTILT, "translation": TILT_T,
                                 "group": "pan", "explode_mm": [60, 0, 95]}},
                "printed_parts": META, "quality": quality,
                "inserts": INSERTS, "screws": SCREWS, "hardware": HARDWARE, "nuts": NUTS,
                "fits": {"spline_normal_clearance_mm": 0.06,
                         "nominal_axial_spline_overlap_mm": 5.7,
                         "full_tooth_engagement_bound_mm": 5.40,
                         "spline_lead_mm": 0.25, "insert_pilot_D_mm": 4.0,
                         "servo_boss_to_rotating_hub_clearance_mm": 0.30,
                          "6200RS_seat_D_mm": SEAT_D, "6200RS_journal_D_mm": JOURNAL_D},
                "bearing_design": assembly_parameters(),
                "print_file_inventory": {print_id: {
                    "quantity": sum(META[n].get("print_file_id", n) == print_id for n in PARTS),
                    "assembly_instances": [n for n in PARTS if META[n].get("print_file_id", n) == print_id]}
                    for print_id in sorted(printed_ids)},
                "hardware_evidence_sha256": hashlib.sha256((HERE/"reference/hardware_evidence.json").read_bytes()).hexdigest(),
                "reference_mesh_analysis": {"MKS_Pan": pqa, "MKS_Tilt": tqa},
                "lightweight_changes": LIGHTWEIGHT_INFO,
                "assembly_audit": audit,
                "physical_validation": "Not printed/tested; no full-stall-torque rating"}
    json_write(OUT / "manifest.json", manifest)
    json_write(OUT / "quality_report.json", {"parts": quality, "assembly": audit})
    json_write(OUT / "lightweight_construction_report.json", LIGHTWEIGHT_INFO)
    print("INSERTS", len(INSERTS), "SCREWS", len(SCREWS), flush=True)
    if audit["static_failures"] or audit["tilt_sweep"]["failures"] or audit["pan_sweep"]["failures"]:
        print("ASSEMBLY NEEDS CORRECTION; see output/manifest.json", flush=True)
    else:
        print("PRINTED MESH / STATIC / MOTION QA OK", flush=True)
    from assembly_checks import main as check_assembly
    full_audit = check_assembly()
    if not full_audit["pass"]:
        raise SystemExit("Assembly paths/tools/hardware need correction; reports retained")
    print("BUILD AND ASSEMBLY QA OK", flush=True)


if __name__ == "__main__":
    main()
