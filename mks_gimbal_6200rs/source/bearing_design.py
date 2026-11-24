"""One sourced bearing specification and shared assembly interface parameters."""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parent.parent
VERSION = "MKS-6200RS-MINREPRINT-LW1-prototype-20261124"
EVIDENCE = json.loads((ROOT / "reference/hardware_evidence.json").read_text(encoding="utf-8"))
BEARING = EVIDENCE["bearings"][0]
BORE = float(BEARING["bore"])
OD = float(BEARING["OD"])
WIDTH = float(BEARING["width"])
SEAT_D = OD + 0.10
JOURNAL_D = BORE - 0.02
INNER_CONTACT_D = float(BEARING["design_inner_contact_OD"])
OUTER_RETAINER_ID = float(BEARING["design_outer_retainer_ID"])
SPACER_BORE_D = BORE + 0.20

OUTER_ADAPTER_H = 1.0
OUTER_ADAPTER_OD = 34.0
OUTER_ADAPTER_PRINT_ID = "18_6200RS_Outer_Ring_Adapter"
PAN_CAP_BOTTOM = 56.0
PAN_CAP_TOP = 79.0
PAN_STACK_BOTTOM = PAN_CAP_BOTTOM + OUTER_ADAPTER_H
PAN_STACK_TOP = PAN_CAP_TOP - OUTER_ADAPTER_H
PAN_BEARING_Z = (PAN_STACK_BOTTOM, PAN_STACK_TOP - WIDTH)
PAN_SPACER_Z = PAN_STACK_BOTTOM + WIDTH
PAN_SPACER_LENGTH = PAN_BEARING_Z[1] - PAN_SPACER_Z
PAN_CENTRE_SPAN = PAN_BEARING_Z[1] - PAN_BEARING_Z[0]
PAN_COUPLER_FACE_Z = PAN_STACK_BOTTOM - 0.15
PAN_KEY_FLOOR_Z = 52.4
PAN_KEY_MALE_AF = 8.0
PAN_KEY_FEMALE_AF = PAN_KEY_MALE_AF + 0.12

TILT_BEARING_INNER_X = -47.0
TILT_BEARING_OUTER_X = TILT_BEARING_INNER_X - WIDTH
TILT_SUPPORT_INNER_X = -44.0
TILT_SUPPORT_THICKNESS = TILT_SUPPORT_INNER_X - TILT_BEARING_OUTER_X
TILT_CAP_OUTER_X = TILT_BEARING_OUTER_X - OUTER_ADAPTER_H - 2.0
TILT_JOURNAL_END_X = TILT_BEARING_OUTER_X - 1.4
TILT_RETAINER_OUTER_X = TILT_BEARING_OUTER_X - 3.9
TILT_CLAMP_SURFACE_X = TILT_RETAINER_OUTER_X - 0.5

assert (BORE, OD, WIDTH) == (10.0, 30.0, 9.0)
assert BEARING["shaft_abutment_da_min"] <= INNER_CONTACT_D <= BEARING["shaft_abutment_da_max"]
assert BEARING["outer_recess_D2_approx"] < OUTER_RETAINER_ID <= BEARING["housing_abutment_Da_max"]
assert PAN_SPACER_LENGTH > 0


def assembly_parameters():
    return {
        "product": BEARING["product"], "nominal_size_mm": [BORE, OD, WIDTH],
        "quantity": 3, "allocation": {"Pan": 2, "Tilt": 1},
        "seat_D_mm": SEAT_D, "journal_D_mm": JOURNAL_D,
        "fit_type": "Nominal clearance; +0.10 mm seat and -0.02 mm journal are untested FDM starting values, not guaranteed fits",
        "inner_contact_OD_mm": INNER_CONTACT_D,
        "outer_retainer_ID_mm": OUTER_RETAINER_ID,
        "Pan_bearing_start_Z_mm": list(PAN_BEARING_Z),
        "Pan_bearing_centre_span_mm": PAN_CENTRE_SPAN,
        "Pan_inner_spacer_length_mm": PAN_SPACER_LENGTH,
        "Pan_inner_stack_mm": PAN_STACK_TOP - PAN_STACK_BOTTOM,
        "Pan_upper_inner_shoulder_Z_mm": PAN_STACK_TOP,
        "Pan_lower_inner_stop_Z_mm": PAN_COUPLER_FACE_Z,
        "Pan_key_male_AF_mm": PAN_KEY_MALE_AF,
        "Pan_key_female_AF_mm": PAN_KEY_FEMALE_AF,
        "Pan_key_engagement_mm": PAN_COUPLER_FACE_Z - PAN_KEY_FLOOR_Z,
        "outer_adapter_print_file": OUTER_ADAPTER_PRINT_ID,
        "outer_adapter_quantity": 3,
        "outer_adapter_size_mm": [OUTER_ADAPTER_OD, OUTER_RETAINER_ID, OUTER_ADAPTER_H],
        "Tilt_bearing_faces_X_mm": [TILT_BEARING_OUTER_X, TILT_BEARING_INNER_X],
        "Tilt_support_thickness_mm": TILT_SUPPORT_THICKNESS,
        "Tilt_journal_end_X_mm": TILT_JOURNAL_END_X,
        "Tilt_retainer_to_journal_end_reserve_mm": 0.45,
        "single_bearing_decision": "Tilt has one external bearing and an opposite servo-supported end. Pan retains two separated bearings; no single-external-bearing Pan load rating is claimed.",
        "reprint_policy": "Keep physical files 01,03,04,06,08,10,11,12,13,16. Reprint original 02,05,07,09,14,15; add 17 and three copies of one shared 18 file. Part 13 is reposed only.",
    }
