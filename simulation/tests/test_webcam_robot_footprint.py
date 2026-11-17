"""Comprueba la huella contra las mallas locales, sin cámara ni motores."""
from __future__ import annotations

import math
from pathlib import Path

import pytest

from simulation.webcam.robot_footprint import (
    ROBOT_FOOTPRINT_MM,
    ROBOT_RADIUS_MM,
    TRACK_WIDTH_MM,
    WHEEL_DIAMETER_MM,
    WHEEL_WIDTH_MM,
    robot_radius_mm,
)


def cross(a, b, point):
    return ((b[0] - a[0]) * (point[1] - a[1])
            - (b[1] - a[1]) * (point[0] - a[0]))


def inside(point, tolerance=1e-5):
    # Tolerancia de distancia en mm, no de área del producto vectorial; cubre
    # el redondeo de coordenadas a float32 al serializar las mallas.
    return all(cross(a, b, point) >= -tolerance * math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in
               zip(ROBOT_FOOTPRINT_MM, ROBOT_FOOTPRINT_MM[1:] + ROBOT_FOOTPRINT_MM[:1]))


def vertices(name):
    path = Path(__file__).resolve().parents[1] / "assets" / "robot" / f"{name}.obj"
    with path.open() as source:
        return {tuple(float(value) * 1000.0 for value in line.split()[1:4])
                for line in source if line.startswith("v ")}


def test_radius_is_derived_and_rounded_up_to_the_next_tenth_mm():
    measured = max(math.hypot(x, y) for x, y in ROBOT_FOOTPRINT_MM)
    assert measured == pytest.approx(math.hypot(83.15, 32.0), abs=1e-8)
    assert ROBOT_RADIUS_MM == robot_radius_mm() == 89.1
    assert measured <= ROBOT_RADIUS_MM < measured + .1


def test_hull_is_convex_and_contains_axle_origin_with_correct_dimensions():
    points = ROBOT_FOOTPRINT_MM
    assert len(points) == 10
    assert all(cross(points[i - 2], points[i - 1], point) > 0
               for i, point in enumerate(points))
    assert inside((0.0, 0.0))
    assert min(x for x, _ in points) == pytest.approx(-52.85)
    assert max(x for x, _ in points) == pytest.approx(83.15)
    assert WHEEL_DIAMETER_MM == 33.0 and WHEEL_WIDTH_MM == 7.0
    assert TRACK_WIDTH_MM == 163.0
    assert max(y for _, y in points) - min(y for _, y in points) == pytest.approx(170.0)
    assert not inside((83.15, 85.0))  # Esquina vacía de la caja delimitadora.


def test_hull_covers_body_mesh_in_its_axle_frame():
    assert all(inside((x + 13.15, y)) for x, y, _ in vertices("body"))
    # El portador del marcador se genera aparte del OBJ del cuerpo.
    assert all(inside((x, y)) for x in (-52.85, -.85) for y in (-26.0, 26.0))


def test_hull_covers_the_full_rotation_of_both_wheel_meshes():
    for x, y, z in vertices("wheel"):
        sweep = math.hypot(x, z)
        for wheel_y in (-81.5, 81.5):
            assert inside((-sweep, y + wheel_y))
            assert inside((sweep, y + wheel_y))
            assert math.hypot(sweep, y + wheel_y) <= ROBOT_RADIUS_MM


def test_physical_marker_carrier_example_uses_calibrated_axle_origin():
    # Ejemplo fijo, no se lee .runtime ni se presupone el montaje del simulador.
    center_tag_m = (-.044136176834403615, .03362232769425596)
    front_angle_rad = -.04239146511652292
    c, s = math.cos(front_angle_rad), math.sin(front_angle_rad)
    projected = []
    for x_tag in (-.026, .026):
        for y_tag in (-.026, .026):
            dx, dy = x_tag - center_tag_m[0], y_tag - center_tag_m[1]
            projected.append((1000 * (c * dx + s * dy),
                              1000 * (-s * dx + c * dy)))
    assert max(math.hypot(x, y) for x, y in projected) == pytest.approx(92.05381719748496)
    # Al reducirse las ruedas, el radio nominal ya no garantiza cubrir una
    # tarjeta desplazada: el planificador debe añadirla desde su calibración.
    assert any(math.hypot(x, y) > ROBOT_RADIUS_MM for x, y in projected)
    assert any(not inside(point) for point in projected)


@pytest.mark.parametrize("marker_size,carrier_size", [(.040, .052), (.052, .065)])
def test_physical_card_keeps_calibrated_center_front_and_known_outer_size(marker_size, carrier_size):
    import cv2
    import numpy as np
    from simulation.webcam.navigation_geometry import Calibration
    from simulation.webcam.pose_obstacle_planner import _Scene

    calibration = Calibration((-.044136176834403615, .03362232769425596),
                              -.04239146511652292, "fixture", 1280, 960, marker_size)
    before = calibration.to_dict()
    corners = [[600, 440], [680, 440], [680, 520], [600, 520]]
    scene = _Scene(corners, calibration, [], ROBOT_FOOTPRINT_MM)
    theta = calibration.front_angle_rad
    rotation = np.array([[math.cos(theta), math.sin(theta)],
                         [-math.sin(theta), math.cos(theta)]])
    half = carrier_size / 2.
    for x in (-half, half):
        for y in (-half, half):
            point = rotation @ (np.array([x, y]) - calibration.center_tag_m)
            assert cv2.pointPolygonTest(scene.footprint.astype(np.float32), tuple(point), True) >= -1e-7
    assert calibration.to_dict() == before
    assert scene.calibration is calibration
