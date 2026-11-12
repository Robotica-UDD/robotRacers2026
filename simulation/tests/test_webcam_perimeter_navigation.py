"""Perímetros trazados como barreras: interior libre y límite infranqueable.

La planta de ruedas y la cámara son dobles locales. Ninguna prueba abre una
cámara, un puerto USB ni una conexión con el robot físico.
"""
from __future__ import annotations

from contextlib import contextmanager
import json
import math

from fastapi import FastAPI
from fastapi.testclient import TestClient
import numpy as np
import pytest

from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot, wait_for
from simulation.webcam.navigation import Navigation, NavigationError, make_navigation_router
from simulation.webcam.pose_obstacle_planner import check_pose
from simulation.webcam.robot_footprint import ROBOT_FOOTPRINT_MM


CAMERA_ID = "usb-camera"
WIDTH, HEIGHT = 1280, 960
SCALE = 600.
ROOM_X, ROOM_Y = .7, .5
ROOM = [(-ROOM_X, -ROOM_Y), (ROOM_X, -ROOM_Y),
        (ROOM_X, ROOM_Y), (-ROOM_X, ROOM_Y)]


def pixel(point):
    return [640. + SCALE * point[0], 480. - SCALE * point[1]]


def perimeter(points=ROOM):
    return [[pixel(point), pixel(points[(index + 1) % len(points)])]
            for index, point in enumerate(points)]


class PerimeterWheelPlant(FakeRobot):
    """Integra arcos de 20 ms y mide la huella frente a límites independientes."""

    def __init__(self, camera):
        super().__init__()
        self.camera = camera
        self.x, self.y, self.yaw = -.3, 0., 0.
        self.trace = []
        self.maximum_boundary_intrusion_mm = 0.
        self.publish()

    def publish(self):
        center = pixel((self.x, self.y))
        self.camera.pose(x=center[0], y=center[1], heading=self.yaw, scale=SCALE)

    def drive_velocity_navigation(self, left, right):
        super().drive_velocity_navigation(left, right)
        speed, angular = (left + right) / 2000., (right - left) / 190.
        origin_x, origin_y, origin_yaw = self.x, self.y, self.yaw
        for fraction in (.25, .5, .75, 1.):
            delta = angular * .020 * fraction
            if abs(angular) > 1e-9:
                x = origin_x + speed / angular * (math.sin(origin_yaw + delta) - math.sin(origin_yaw))
                y = origin_y + speed / angular * (math.cos(origin_yaw) - math.cos(origin_yaw + delta))
            else:
                x = origin_x + speed * .020 * fraction * math.cos(origin_yaw)
                y = origin_y + speed * .020 * fraction * math.sin(origin_yaw)
            yaw = origin_yaw + delta
            c, s = math.cos(yaw), math.sin(yaw)
            body = np.asarray([[x + (c * px - s * py) / 1000.,
                                y + (s * px + c * py) / 1000.]
                               for px, py in ROBOT_FOOTPRINT_MM])
            intrusion = max(0., float(np.max(np.abs(body[:, 0]))) - ROOM_X,
                            float(np.max(np.abs(body[:, 1]))) - ROOM_Y)
            self.maximum_boundary_intrusion_mm = max(self.maximum_boundary_intrusion_mm, intrusion * 1000.)
        self.x, self.y, self.yaw = x, y, yaw
        self.trace.append((self.x, self.y, self.yaw, left, right))
        self.publish()


@contextmanager
def scene(*, plant=False, runtime_path=None):
    camera = FakeCamera()
    camera.pose(x=460., y=480., scale=SCALE)
    camera.update(running=True, opening=False)
    robot = PerimeterWheelPlant(camera) if plant else FakeRobot()
    robot.update(max_velocity_mm_s=1000)
    nav = Navigation(camera, robot, runtime_path)
    nav.CONTROL_INTERVAL = .001
    nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 12000., 18000.
    nav.SETTLE_SECONDS, nav.WAYPOINT_SETTLE_SECONDS = .010, .005
    nav.RESUME_STABLE_SECONDS, nav.RESUME_RETRY_SECONDS = .015, .015
    nav.calibrate(pixel((-.3, 0.)), pixel((-.26, 0.)), WIDTH, HEIGHT, CAMERA_ID)
    nav.set_speeds(240, 90)
    # El seguidor y las ruedas usan el mismo paso virtual de 20 ms. Acelerar el
    # bucle real sin adaptar su reloj falsearía las rampas de la prueba.
    original_command = nav._continuous_command
    logical_time = [0.]

    def logical_command(frame, calibration, path, follower, speeds, tick, **kwargs):
        logical_time[0] += .020
        previous_stamp = nav._ramp_stamp
        rise, fall = nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2
        try:
            nav._ramp_stamp = logical_time[0] - .020
            nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 600., 900.
            if follower._last_now is None:
                follower._last_now = logical_time[0] - .020
            return original_command(frame, calibration, path, follower, speeds, logical_time[0], **kwargs)
        finally:
            nav._ramp_stamp = previous_stamp
            nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = rise, fall

    if plant:
        nav._continuous_command = logical_command
    try:
        yield camera, robot, nav
    finally:
        nav.close()


def save_segments(nav, barriers=None):
    return nav.set_obstacles(perimeter() if barriers is None else barriers,
                             WIDTH, HEIGHT, CAMERA_ID, nav.state()["route_revision"])


def select(nav, point=(.3, 0.)):
    return nav.select_target(*pixel(point), WIDTH, HEIGHT, CAMERA_ID)


def test_a_closed_perimeter_keeps_the_interior_free_and_preview_never_arms_motors():
    with scene() as (camera, robot, nav):
        state = save_segments(nav)
        assert state["obstacles_px"] == perimeter()
        state = select(nav)
        assert state["planning_status"] == "ready", state["planning_message"]
        assert state["planned_path_px"][0] == pytest.approx(pixel((-.3, 0.)), abs=.01)
        assert state["planned_path_px"][-1] == pytest.approx(pixel((.3, 0.)), abs=.01)
        assert check_pose(camera.state()["marker_corners"], nav._calibration,
                          pixel((-.3, 0.)), 0., perimeter())["clear"]
        assert robot.starts == robot.stops == 0 and robot.drives == []


def test_virtual_robot_starts_and_advances_inside_the_closed_perimeter():
    with scene(plant=True) as (_, robot, nav):
        save_segments(nav)
        select(nav)
        assert nav.start()["active"]
        wait_for(lambda: nav.state()["phase"] in {"arrived", "error"}, timeout=8.)
        assert nav.state()["phase"] == "arrived", nav.state()["message"]
        assert robot.x > .1 and len(robot.trace) > 20
        assert any(left > 0 and right > 0 for _, _, _, left, right in robot.trace)
        assert robot.maximum_boundary_intrusion_mm == 0.
        assert robot.starts == 1 and robot.stops == 1


def test_an_open_perimeter_has_a_passage_but_closing_its_last_side_blocks_the_external_target():
    with scene() as (_, robot, nav):
        # El lateral derecho es el único paso entre el interior y el destino.
        barriers = perimeter()
        state = save_segments(nav, [barriers[0], barriers[2], barriers[3]])
        state = select(nav, (.9, 0.))
        assert state["planning_status"] == "ready", state["planning_message"]
        state = save_segments(nav)
        assert state["planning_status"] == "blocked"
        assert state["planned_path_px"] == []
        with pytest.raises(NavigationError):
            nav.start()
        assert robot.starts == robot.stops == 0 and robot.drives == []


def test_the_closed_perimeter_never_reports_success_for_a_target_on_the_other_side():
    with scene() as (_, robot, nav):
        save_segments(nav)
        state = select(nav, (.9, 0.))
        assert state["planning_status"] == "blocked"
        assert state["phase"] != "arrived" and not state["active"]
        with pytest.raises(NavigationError):
            nav.start()
        assert robot.starts == 0 and robot.drives == []


def test_starting_with_the_footprint_over_a_boundary_does_not_authorize_an_escape_outside():
    with scene() as (camera, robot, nav):
        # Centro aún dentro; la esquina delantera invade la barrera de1mm.
        center = pixel((.65, 0.))
        camera.pose(x=center[0], y=center[1], scale=SCALE)
        save_segments(nav)
        state = select(nav, (.95, 0.))
        assert not check_pose(camera.state()["marker_corners"], nav._calibration,
                              center, 0., perimeter())["clear"]
        assert state["planning_status"] == "blocked", state["planned_path_px"]
        with pytest.raises(NavigationError):
            nav.start()
        assert robot.starts == 0 and robot.drives == []


def test_api_and_persistence_keep_segments_as_two_points_and_legacy_quads_as_solids(tmp_path):
    runtime = tmp_path / "navigation-state.json"
    with scene(runtime_path=runtime) as (camera, robot, nav):
        legacy = [pixel((.1, .2)), pixel((.2, .2)), pixel((.2, .3)), pixel((.1, .3))]
        values = [*perimeter(), legacy]
        app = FastAPI()
        app.include_router(make_navigation_router(nav))
        with TestClient(app) as client:
            payload = {"polygons_px": values, "frame_width": WIDTH, "frame_height": HEIGHT,
                       "camera_id": CAMERA_ID, "expected_route_revision": nav.state()["route_revision"]}
            response = client.post("/api/navigation/obstacles", json=payload,
                                   headers={"X-Requested-With": "XMLHttpRequest"})
            assert response.status_code == 200, response.text
        stored = json.loads(runtime.with_name("navigation-obstacles.json").read_text())
        assert stored["version"] == 1 and stored["polygons_px"] == values
        restored = Navigation(camera, FakeRobot(), runtime)
        try:
            assert restored.state()["obstacles_px"] == values
            assert [len(value) for value in restored.state()["obstacles_px"]] == [2, 2, 2, 2, 4]
        finally:
            restored.close()
        assert robot.starts == robot.stops == 0 and robot.drives == []


def test_obstacle_api_accepts_a_perimeter_map_larger_than_the_regular_eight_kilobyte_actions():
    with scene() as (_, robot, nav):
        # Muchas paredes se guardan como segmentos independientes; no se debe
        # perder el último cierre por el antiguo límite de 32 cuadriláteros.
        barriers = [perimeter()[index % 4] for index in range(400)]
        payload = {"polygons_px": barriers, "frame_width": WIDTH, "frame_height": HEIGHT,
                   "camera_id": CAMERA_ID, "expected_route_revision": nav.state()["route_revision"]}
        assert 8192 < len(json.dumps(payload, separators=(",", ":")).encode()) < 65536
        app = FastAPI()
        app.include_router(make_navigation_router(nav))
        with TestClient(app) as client:
            response = client.post("/api/navigation/obstacles", json=payload,
                                   headers={"X-Requested-With": "XMLHttpRequest"})
        assert response.status_code == 200, response.text
        assert len(nav.state()["obstacles_px"]) == 400
        assert robot.starts == robot.stops == 0 and robot.drives == []
