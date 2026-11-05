"""Geometría de navegación con imágenes sintéticas; no accede a motores ni cámara."""
from __future__ import annotations

import json
import math

import numpy as np
import pytest

from simulation.webcam.navigation_geometry import Calibration, calibrate_robot, observe_robot


def project(matrix, point):
    homogeneous = matrix @ np.array([*point, 1.])
    return homogeneous[:2] / homogeneous[2]


def tag_corners(matrix, side=.04):
    half = side / 2
    return np.array([project(matrix, p) for p in
                     [(-half, half), (half, half), (half, -half), (-half, -half)]])


def image_matrix(angle=0., scale=4000., width=1280, height=960, perspective=False):
    c, s = math.cos(angle), math.sin(angle)
    matrix = np.array([[scale * c, -scale * s, width / 2],
                       [-scale * s, -scale * c, height / 2], [0., 0., 1.]])
    if perspective:
        matrix[2, :2] = [1.8, -2.2]
    return matrix


def calibrate(matrix, center=(.026, -.008), theta=.3, side=.04, width=1280, height=960,
              front_distance=.06):
    center = np.asarray(center)
    direction = np.array([math.cos(theta), math.sin(theta)])
    return calibrate_robot(tag_corners(matrix, side), project(matrix, center),
                           project(matrix, center + front_distance * direction),
                           camera_id="webcam-usb", width=width, height=height,
                           marker_size_m=side)


@pytest.mark.parametrize("perspective", [False, True])
@pytest.mark.parametrize("camera_angle", [0., math.pi / 2, math.pi, 3 * math.pi / 2])
def test_center_front_and_target_follow_robot_rotation_and_perspective(camera_angle, perspective):
    matrix = image_matrix(camera_angle, perspective=perspective)
    center = np.array([.026, -.008])
    theta = .3
    calibration = calibrate(matrix, center, theta)
    # The calibration survives a new pose, rather than being tied to image pixels.
    moved_matrix = image_matrix(camera_angle + .25, scale=3200., perspective=perspective)
    moved_matrix[:2, 2] += [30., -15.]
    local_target = np.array([.075, -.018])
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    target = project(moved_matrix, center + rotation @ local_target)
    observation = observe_robot(tag_corners(moved_matrix), calibration, target)
    expected_center = project(moved_matrix, center)
    expected_front = project(moved_matrix, center + rotation @ [.02, 0.])
    delta = expected_front - expected_center
    expected_heading = math.degrees(math.atan2(-delta[1], delta[0])) % 360
    assert calibration.center_tag_m == pytest.approx(center, abs=1e-7)
    assert calibration.front_angle_rad == pytest.approx(theta, abs=1e-6)
    assert observation["center_px"] == pytest.approx(expected_center, abs=.001)
    assert observation["front_px"] == pytest.approx(expected_front, abs=.001)
    assert observation["heading_deg"] == pytest.approx(expected_heading, abs=.001)
    assert observation["target_local_m"] == pytest.approx(local_target, abs=1e-7)
    assert observation["distance_m"] == pytest.approx(np.linalg.norm(local_target), abs=1e-6)
    assert observation["bearing_rad"] == pytest.approx(math.atan2(-.018, .075), abs=1e-6)


def test_projective_center_is_not_the_arithmetic_average_of_corners():
    matrix = image_matrix(perspective=True)
    corners = tag_corners(matrix)
    calibration = calibrate(matrix, center=(0., 0.), theta=0.)
    observation = observe_robot(corners, calibration)
    assert np.linalg.norm(corners.mean(axis=0) - project(matrix, [0., 0.])) > 2.
    assert observation["center_px"] == pytest.approx(project(matrix, [0., 0.]), abs=.001)
    assert observation["distance_m"] is None
    assert observation["bearing_rad"] is None
    assert observation["target_local_m"] is None


@pytest.mark.parametrize("local, bearing", [((.03, 0.), 0.), ((0., .03), math.pi / 2),
                                            ((0., -.03), -math.pi / 2), ((-.03, .01),
                                             math.atan2(.01, -.03))])
def test_bearing_is_positive_to_the_left_in_robot_coordinates(local, bearing):
    matrix = image_matrix()
    calibration = calibrate(matrix, center=(0., 0.), theta=math.pi / 2)
    # Robot points towards tag +Y; robot +Y points towards tag -X.
    target_tag = [-local[1], local[0]]
    result = observe_robot(tag_corners(matrix), calibration, project(matrix, target_tag))
    assert result["target_local_m"] == pytest.approx(local, abs=1e-7)
    assert result["bearing_rad"] == pytest.approx(bearing, abs=1e-6)


@pytest.mark.parametrize("width,height,scale,side", [(640, 480, 1800., .04),
                                                    (1280, 960, 4000., .04),
                                                    (1920, 1080, 6000., .06)])
def test_marker_scale_and_resolution_preserve_metric_distances(width, height, scale, side):
    matrix = image_matrix(width=width, height=height, scale=scale)
    calibration = calibrate(matrix, center=(0., 0.), theta=0., side=side,
                            width=width, height=height)
    result = observe_robot(tag_corners(matrix, side), calibration, project(matrix, [.05, 0.]))
    assert result["distance_m"] == pytest.approx(.05, abs=1e-7)


def test_target_at_center_has_zero_distance_and_json_safe_numbers():
    matrix = image_matrix(perspective=True)
    calibration = calibrate(matrix)
    observation = observe_robot(tag_corners(matrix), calibration,
                                project(matrix, calibration.center_tag_m))
    assert observation["distance_m"] == pytest.approx(0., abs=1e-7)
    assert json.loads(json.dumps(observation, allow_nan=False))["center_px"]


def test_calibration_roundtrips_json_and_normalizes_angle():
    original = Calibration([.02, -.01], 2 * math.pi + .4, "usb", 1280, 960)
    restored = Calibration.from_dict(json.loads(json.dumps(original.to_dict(), allow_nan=False)))
    assert restored == original
    assert restored.front_angle_rad == pytest.approx(.4)
    assert isinstance(restored.center_tag_m, tuple)


@pytest.mark.parametrize("field,value", [
    ("center_tag_m", [float("nan"), 0]), ("center_tag_m", [0, float("inf")]),
    ("center_tag_m", [True, 0]), ("center_tag_m", [".02", 0]),
    ("center_tag_m", {"x": 0, "y": 0}), ("center_tag_m", [0]),
    ("center_tag_m", [.201, 0]), ("center_tag_m", [10 ** 1000, 0]),
    ("front_angle_rad", float("nan")), ("front_angle_rad", True),
    ("front_angle_rad", "0"), ("camera_id", ""), ("camera_id", "   "),
    ("camera_id", 0), ("width", 0), ("width", 1280.), ("width", True),
    ("height", 8193), ("marker_size_m", 0), ("marker_size_m", .251),
    ("marker_size_m", ".04"), ("marker_size_m", True),
])
def test_corrupt_saved_calibration_is_rejected_as_value_error(field, value):
    saved = Calibration((0., 0.), 0., "usb", 1280, 960).to_dict()
    saved[field] = value
    with pytest.raises(ValueError):
        Calibration.from_dict(saved)


@pytest.mark.parametrize("saved", [None, [], {}, {"unexpected": 0}])
def test_saved_calibration_requires_its_complete_schema(saved):
    with pytest.raises(ValueError):
        Calibration.from_dict(saved)


@pytest.mark.parametrize("corners", [
    [[10, 10], [20, 20], [30, 30], [40, 40]],  # collinear
    [[10, 10], [30, 30], [30, 10], [10, 30]],  # self-intersecting
    [[10, 10], [30, 10], [15, 15], [10, 30]],  # concave
    [[10, 10], [30, 10], [30, 10], [10, 30]],  # duplicate
    [[10, 10], [13, 10], [13, 13], [10, 13]],  # too small
    [[10, 10], [30, 10], [30, float("nan")], [10, 30]],
    [[10, 10], [30, 10], [30, float("inf")], [10, 30]],
    [[-1, 10], [30, 10], [30, 30], [10, 30]],
    [[10, 10], [1280, 10], [30, 30], [10, 30]],
    [[10, 10], [30, 10], [30, 960], [10, 30]],
    [["10", "10"], ["30", "10"], ["30", "30"], ["10", "30"]],
    [[10, 10], [30, 10], [30, 30]], None,
])
def test_invalid_quadrilaterals_cannot_generate_motor_geometry(corners):
    calibration = Calibration((0., 0.), 0., "usb", 1280, 960)
    with pytest.raises(ValueError):
        observe_robot(corners, calibration, [100, 100])


@pytest.mark.parametrize("distance", [.01, .25])
def test_front_click_accepts_documented_distance_boundaries(distance):
    matrix = image_matrix(scale=1800.)
    calibration = calibrate(matrix, center=(0., 0.), theta=0., front_distance=distance)
    assert calibration.front_angle_rad == pytest.approx(0., abs=1e-6)


@pytest.mark.parametrize("distance", [0., .005, .009, .251])
def test_front_click_rejects_unreliable_short_or_long_direction(distance):
    matrix = image_matrix(scale=1800.)
    with pytest.raises(ValueError):
        calibrate(matrix, center=(0., 0.), theta=0., front_distance=distance)


def test_calibration_rejects_center_more_than_twenty_centimeters_from_tag():
    matrix = image_matrix(scale=1000.)
    with pytest.raises(ValueError):
        calibrate(matrix, center=(.201, 0.), theta=0.)


@pytest.mark.parametrize("point", [[-1., 100.], [1280., 100.], [100., 960.],
                                   [float("nan"), 100.], [100.], {"x": 5, "y": 5}])
def test_invalid_clicks_are_rejected_for_calibration_and_destination(point):
    matrix = image_matrix()
    corners = tag_corners(matrix)
    for center, front in [(point, [700, 480]), ([640, 480], point)]:
        with pytest.raises(ValueError):
            calibrate_robot(corners, center, front, camera_id="usb", width=1280, height=960)
    with pytest.raises(ValueError):
        observe_robot(corners, Calibration((0., 0.), 0., "usb", 1280, 960), point)


def test_observation_rejects_calibrated_center_or_front_outside_current_image():
    matrix = image_matrix(scale=1000.)
    matrix[0, 2] = 25.
    # Marker remains inside frame, but the axle center or its projected front exits.
    calibration = Calibration((-.03, 0.), 0., "usb", 1280, 960)
    with pytest.raises(ValueError):
        observe_robot(tag_corners(matrix), calibration)
    calibration = Calibration((-.02, 0.), math.pi, "usb", 1280, 960)
    matrix[0, 2] = 19.
    matrix[0, 0] = 500.
    with pytest.raises(ValueError):
        observe_robot(tag_corners(matrix), calibration)


def test_geometry_does_not_open_camera_hardware(monkeypatch):
    import cv2

    def forbidden(*_args, **_kwargs):
        raise AssertionError("La geometría no debe abrir cámaras.")

    monkeypatch.setattr(cv2, "VideoCapture", forbidden)
    matrix = image_matrix()
    calibration = calibrate(matrix)
    assert observe_robot(tag_corners(matrix), calibration)["center_px"]
