#!/usr/bin/env python3
"""Record inexpensive bounds checks without running or changing the simulator.

Bounds can establish separation but cannot certify mechanical interference or
manufacturing fit for parametric substitute meshes. No triangle collision test
or physical calibration is implied by this report.
"""
from itertools import combinations
from pathlib import Path
import hashlib
import json
import math
import re

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT / "simulation"


def translate(bounds, offset):
    return [[edge[i] + offset[i] for i in range(3)] for edge in bounds]


def contains(outer, inner):
    return all(outer[0][i] - 1e-9 <= inner[0][i] and inner[1][i] <= outer[1][i] + 1e-9 for i in range(3))


def relation(a, b):
    overlap = [min(a[1][i], b[1][i]) - max(a[0][i], b[0][i]) for i in range(3)]
    gap = [max(0., -value) for value in overlap]
    return {"axis_overlap_m": overlap, "distance_lower_bound_m": math.sqrt(sum(x*x for x in gap))}


def collision_boxes(generator):
    section = generator.split("boundingObject Group", 1)[1]
    numbers = r"([-+\d.eE]+) ([-+\d.eE]+) ([-+\d.eE]+)"
    boxes = re.findall(r"Pose \{\{ translation " + numbers + r" children \[Box \{\{ size " + numbers, section)
    bounds = [[[float(row[i]) - float(row[i+3])/2 for i in range(3)],
               [float(row[i]) + float(row[i+3])/2 for i in range(3)]] for row in boxes[:2]]
    if len(bounds) == 1:
        match = re.search(r"Pose \{\{ translation " + numbers + r" children \[Cylinder \{\{ radius ([-+\d.eE]+) height ([-+\d.eE]+)", section)
        x, y, z, radius, height = map(float, match.groups())
        bounds.append([[x-radius, y-radius, z-height/2], [x+radius, y+radius, z+height/2]])
    return bounds


def main():
    manifest = json.loads((SIM / "assets/manifest.json").read_text())
    config = json.loads((SIM / "config/robot.json").read_text())
    parts = manifest["body"]["parts"]
    offset = config["cad_body_offset_m"]
    main_box, skid_box = collision_boxes((SIM / "tools/build_world.py").read_text())
    body_bounds = {name: translate(part["bounds_m"], offset) for name, part in parts.items()}
    marker_center = [*config["marker_offset_m"], config["marker_height_m"]]
    marker_half_side = config["marker_size_m"] * 1.3 / 2
    carrier = [[marker_center[0]-marker_half_side, marker_center[1]-marker_half_side, marker_center[2]-.0008],
               [marker_center[0]+marker_half_side, marker_center[1]+marker_half_side, marker_center[2]-.0002]]
    pcb_parts = [(name, part) for name, part in parts.items() if part.get("kind") == "PCB component"]
    candidates = []
    for (a, pa), (b, pb) in combinations(pcb_parts, 2):
        value = relation(pa["bounds_m"], pb["bounds_m"])
        if min(value["axis_overlap_m"]) > 1e-8:
            candidates.append({"parts": [a, b], **value,
                               "interpretation": "Bounds of parametric substitute meshes overlap; real-part interference not established."})
    below_envelope = [name for name, bounds in body_bounds.items() if not contains(main_box, bounds)]
    principal = config["body_inertia_kg_m2"]
    inertia_margins = [sum(principal) - 2*value for value in principal]
    provenance = config["provenance"]
    report = {
        "units": "m, kg, s",
        "frame": "Robot base at drive axle; X forward, Y left, Z up",
        "scope": "Bounds and configuration consistency only. Not measured assembled CAD, PCBA clearance certification or a calibrated digital twin.",
        "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in [
            SIM/"assets/robot/body.obj", SIM/"assets/manifest.json", SIM/"config/robot.json", SIM/"tools/build_world.py"]},
        "collision_boxes_from_generator_m": {"main": main_box, "front_support": skid_box},
        "visual_marker_carrier_bounds_m": carrier,
        "marker_carrier_inside_main_box": contains(main_box, carrier),
        "visual_parts_not_fully_inside_main_box": below_envelope,
        "body_visual_bounds_in_base_m": translate(manifest["body"]["bounds_m"], offset),
        "front_support_visual_bounds_in_base_m": body_bounds["Front_skid_estimated"],
        "front_support_contact_in_base_m": parts["Front_skid_estimated"]["physical_base_contact_centre_m"],
        "nominal_clearances_m": {
            "wheel_inner_surface_to_PCB_edge": config["track_width_m"]/2-config["wheel_width_m"]/2-.065,
            "wheel_inner_surface_to_motor_case_outer_surface": .010,
            "wheel_inner_surface_to_bracket_outer_surface": .009,
            "battery_case_bottom_above_highest_PCB_component": manifest["clearances"]["battery_bottom_m"]-manifest["clearances"]["maximum_PCB_component_top_m"],
            "battery_case_bottom_above_motor_encoder_top": .020-parts["Pololu5178_left"]["bounds_m"][1][2],
            "marker_carrier_to_turbine_housing_longitudinal": body_bounds["Turbine_32mm"][0][0]-carrier[1][0],
            "marker_carrier_to_battery_strap_lateral": .0285-carrier[1][1],
            "shaft_axial_engagement_in_wheel_mesh": 0.,
            "support_disc_outside_support_collider_longitudinal_each_side": max(0., .0045-(skid_box[1][0]-skid_box[0][0])/2),
        },
        "PCB_parametric_bounds_overlap_candidates": candidates,
        "estimates": {key: provenance.get(key, "not measured") for key in [
            "mass_kg", "wheel_mass_kg", "body_center_of_mass_m", "body_inertia_kg_m2", "track_width_m", "friction", "skid_friction", "fan_force_n"]},
        "inertia_snapshot_kg_m2": principal,
        "inertia_triangle_margins_kg_m2": inertia_margins,
        "inertia_positive_and_triangle_consistent": min(principal)>0 and min(inertia_margins)>=-1e-12,
        "limitations": [
            "Motor/support locations follow board coordinates; wheel track is inferred, not a mounting measurement.",
            "Shaft meets wheel inner plane with no modelled insertion. Shaft extension, hub offset and attachment require assembled measurements.",
            "Battery case clears electronics; battery leads intentionally terminate at the XT30 region. Cable routes are illustrative.",
            "The main collision box fills cutouts and gaps conservatively; it is not a triangle-accurate collision mesh.",
            "The support collider, tongue and disc are approximate; thin attachment geometry is not exactly covered by both collision boxes.",
            "Marker carrier has no measured mounting legs; its actual support and sensor-bar mounting remain unspecified.",
            "Parametric solder terminals and nominal component boxes can overlap despite a functional assembled board; bounds do not establish real interference.",
            "Reflectance sensor poses and bar dimensions are provisional; visual lens positions are not a calibration of the simulated sensor origins.",
        ],
    }
    destination = SIM/"assets/geometry_audit.json"
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"report": str(destination.relative_to(ROOT)), "marker_inside_main_box": report["marker_carrier_inside_main_box"],
                      "parts_outside_main_box": below_envelope, "parametric_overlap_candidates": len(candidates),
                      "inertia_consistent": report["inertia_positive_and_triangle_consistent"]}, indent=2))


if __name__ == "__main__":
    main()
