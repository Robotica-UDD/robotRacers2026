"""Tamaño explícito del patrón negro: geometría y API sin dispositivos reales."""
from __future__ import annotations

import json

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot
from simulation.tests.test_webcam_navigation_geometry import image_matrix, project, tag_corners
from simulation.webcam.navigation import Navigation, NavigationError, make_navigation_router


HEADERS = {"X-Requested-With": "XMLHttpRequest"}
CALIBRATION = {"center_px": [640, 480], "front_px": [740, 480],
               "frame_width": 1280, "frame_height": 960, "camera_id": "usb-camera"}


@pytest.fixture
def rig():
    camera, robot = FakeCamera(), FakeRobot()
    nav = Navigation(camera, robot)
    try:
        yield camera, robot, nav
    finally:
        nav.close()


@pytest.mark.parametrize("size_mm", [None, 40, 52])
def test_calibration_api_preserves_old_request_and_scales_metric_navigation(rig, size_mm):
    camera, robot, nav = rig
    # Un campo de tamaño ajeno a Calibration no debe sustituir la escala elegida.
    camera.update(marker_size_m=.04)
    app = FastAPI()
    app.include_router(make_navigation_router(nav))
    payload = dict(CALIBRATION)
    if size_mm is not None:
        payload["marker_size_mm"] = size_mm
    expected_mm = 40 if size_mm is None else size_mm
    with TestClient(app) as client:
        response = client.post("/api/navigation/calibrate", json=payload, headers=HEADERS)
        assert response.status_code == 200
        assert response.json()["calibration_geometry"]["marker_size_m"] == pytest.approx(expected_mm / 1000.)
        response = client.post("/api/navigation/target", json={
            "x": 900, "y": 480, "frame_width": 1280, "frame_height": 960,
            "camera_id": "usb-camera"}, headers=HEADERS)
        assert response.status_code == 200
        assert response.json()["remaining_mm"] == pytest.approx(130. * expected_mm / 40., abs=.01)
    assert robot.starts == 0 and robot.drives == [] and robot.stops == 0


def test_reading_and_restoring_a_52_mm_calibration_preserves_its_size(tmp_path):
    camera, robot = FakeCamera(), FakeRobot()
    path = tmp_path / "calibration.json"
    nav = Navigation(camera, robot, path)
    restored = None
    try:
        nav.calibrate(**CALIBRATION, marker_size_mm=52)
        saved = path.read_bytes()
        restored = Navigation(camera, robot, path)
        assert restored.state()["calibration_geometry"]["marker_size_m"] == pytest.approx(.052)
        assert path.read_bytes() == saved
        assert json.loads(saved)["calibration"]["marker_size_m"] == pytest.approx(.052)
        assert robot.starts == 0 and robot.drives == [] and robot.stops == 0
    finally:
        nav.close()
        if restored is not None:
            restored.close()


def test_old_calibration_request_still_selects_40_mm_after_an_explicit_52_mm_request(rig):
    _, robot, nav = rig
    nav.calibrate(**CALIBRATION, marker_size_mm=52)
    assert nav.calibrate(**CALIBRATION)["calibration_geometry"]["marker_size_m"] == pytest.approx(.04)
    assert robot.starts == 0 and robot.drives == []


@pytest.mark.parametrize("size_mm", [True, False, None, "52", [], {}, 0, 4.99, 250.01,
                                   float("nan"), float("inf"), -float("inf"), 10 ** 1000])
def test_invalid_size_does_not_replace_existing_calibration_or_route(rig, size_mm):
    _, robot, nav = rig
    nav.calibrate(**CALIBRATION, marker_size_mm=52)
    nav.select_target(900, 480, 1280, 960, "usb-camera")
    before = nav.state()
    with pytest.raises(NavigationError) as caught:
        nav.calibrate(**CALIBRATION, marker_size_mm=size_mm)
    assert caught.value.status_code == 422
    after = nav.state()
    assert after["calibration_geometry"] == before["calibration_geometry"]
    assert after["waypoints_px"] == before["waypoints_px"]
    assert after["route_revision"] == before["route_revision"]
    assert robot.starts == 0 and robot.drives == [] and robot.stops == 0


@pytest.mark.parametrize("extra", [{"marker_size_mm": True}, {"marker_size_mm": None},
                                  {"marker_size_mm": "52"}, {"marker_size_mm": 4},
                                  {"marker_size_mm": 251}, {"marker_size_mm": 52, "unexpected": 0}])
def test_calibration_api_rejects_invalid_values_and_unrelated_extra_fields(rig, extra):
    _, robot, nav = rig
    app = FastAPI()
    app.include_router(make_navigation_router(nav))
    with TestClient(app) as client:
        response = client.post("/api/navigation/calibrate", json={**CALIBRATION, **extra}, headers=HEADERS)
        assert response.status_code == 422
    assert nav.state()["calibrated"] is False
    assert robot.starts == 0 and robot.drives == [] and robot.stops == 0


@pytest.mark.parametrize("constant", ["NaN", "Infinity", "-Infinity"])
def test_calibration_api_rejects_nonfinite_json_marker_sizes(rig, constant):
    _, _, nav = rig
    app = FastAPI()
    app.include_router(make_navigation_router(nav))
    payload = json.dumps(CALIBRATION)[:-1] + ', "marker_size_mm": ' + constant + '}'
    with TestClient(app) as client:
        response = client.post("/api/navigation/calibrate", content=payload, headers=HEADERS)
        assert response.status_code == 422


@pytest.mark.parametrize("size_mm", [5, 52, 250])
def test_physical_marker_size_preserves_metric_target_after_calibration(rig, size_mm):
    camera, robot, nav = rig
    matrix = image_matrix(scale=3000.)
    camera.update(marker_corners=tag_corners(matrix, side=size_mm / 1000.).tolist())
    nav.calibrate(project(matrix, [.01, -.005]).tolist(),
                  project(matrix, [.04, -.005]).tolist(), 1280, 960, "usb-camera", size_mm)
    target = project(matrix, [.11, -.005]).tolist()
    result = nav.select_target(*target, 1280, 960, "usb-camera")
    assert result["remaining_mm"] == pytest.approx(100., abs=.01)
    assert result["bearing_deg"] == pytest.approx(0., abs=.001)
    assert robot.starts == 0 and robot.drives == [] and robot.stops == 0
