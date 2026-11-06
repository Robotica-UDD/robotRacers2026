"""Integración de obstáculos con cámaras y ruedas virtuales, sin IO físico."""
from __future__ import annotations

import json
import math
import threading
import time

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

import simulation.webcam.navigation as navigation_module
from simulation.webcam.navigation import INT32_MAX, Navigation, NavigationError
from simulation.webcam.obstacle_planner import point_clearance_mm, segment_is_clear
from simulation.webcam.straight_control import StraightControl
from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot, wait_for


QUAD = [[[628., 444.], [652., 444.], [652., 516.], [628., 516.]]]


def calibration(nav):
    return nav.calibrate([472., 480.], [532., 480.], 1280, 960, "usb-camera")


def goal(nav, x=808., y=480.):
    return nav.select_target(x, y, 1280, 960, "usb-camera")


def obstacles(nav, polygons=QUAD):
    return nav.set_obstacles(polygons, 1280, 960, "usb-camera", nav.state()["route_revision"])


def stable(camera, nav, heading=0.):
    camera.update(visible=True, frame_age_ms=0.)
    for _ in range(5):
        camera.pose(x=472., y=480., heading=heading, scale=600.)
        time.sleep(max(nav.CONTROL_INTERVAL * 1.5, nav.RESUME_STABLE_SECONDS / 3))


@pytest.fixture
def scene():
    camera, robot = FakeCamera(), FakeRobot()
    camera.pose(x=472., y=480., scale=600.)
    camera.update(running=True, opening=False)
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .01
    nav.RESUME_STABLE_SECONDS = .03
    nav.SETTLE_SECONDS = .03
    nav.WAYPOINT_SETTLE_SECONDS = .01
    calibration(nav)
    nav.set_obstacle_settings(30, nav.state()["route_revision"])
    try:
        yield camera, robot, nav
    finally:
        nav.close()


def test_preview_detours_without_changing_user_waypoint_or_sending_motor_commands(scene):
    camera, robot, nav = scene
    goal(nav)
    state = obstacles(nav)
    assert state["planning_status"] == "ready", state["planning_message"]
    assert state["target_px"] == [808., 480.] and state["waypoints_px"] == [[808., 480.]]
    assert len(state["planned_path_px"]) >= 4 and state["inflated_obstacles_px"]
    assert state["steering_target_px"] != state["target_px"]
    assert state["obstacle_settings"] == {"robot_radius_mm": 150, "safety_margin_mm": 30, "tracking_margin_mm": 40}
    assert not segment_is_clear(camera.state()["marker_corners"], nav._calibration,
                                [472, 480], [808, 480], QUAD, clearance_mm=180)
    for start, end in zip(state["planned_path_px"], state["planned_path_px"][1:]):
        assert segment_is_clear(camera.state()["marker_corners"], nav._calibration,
                                start, end, QUAD, clearance_mm=220)
    assert robot.starts == robot.stops == 0 and robot.drives == []


def test_blocked_goal_is_rejected_before_arming_and_explicit_clear_keeps_route(scene):
    _, robot, nav = scene
    goal(nav, 640., 480.)
    state = obstacles(nav)
    assert state["planning_status"] == "blocked"
    with pytest.raises(NavigationError):
        nav.start()
    assert robot.starts == 0 and robot.drives == []
    route, speeds, calib = state["waypoints_px"], nav.get_speeds(), nav._calibration
    state = nav.clear_obstacles()
    assert state["obstacles_px"] == [] and state["waypoints_px"] == route
    assert state["planning_status"] == "ready" and state["calibrated"] is True
    assert nav.get_speeds() == speeds and nav._calibration == calib


def test_preview_cache_avoids_astar_for_repeated_polls_and_tiny_pose_noise(scene, monkeypatch):
    camera, _, nav = scene
    calls = []
    original = navigation_module.plan_path

    def count(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    monkeypatch.setattr(navigation_module, "plan_path", count)
    goal(nav)
    obstacles(nav)
    baseline = len(calls)
    for _ in range(5):
        assert nav.state()["planning_status"] == "ready"
    camera.pose(x=475., y=480., scale=600.)  # Cinco milímetros: no requiere otro cálculo.
    nav.state()
    assert len(calls) == baseline
    camera.pose(x=484., y=480., scale=600.)
    nav.state()
    assert len(calls) == baseline + 1


def test_preview_finishing_after_mission_start_cannot_replace_active_subroute(scene, monkeypatch):
    camera, robot, nav = scene
    goal(nav)
    obstacles(nav)
    entered, release = threading.Event(), threading.Event()
    original = navigation_module.plan_path

    def calculate(*args, **kwargs):
        entered.set()
        assert release.wait(1.)
        return original(*args, **kwargs)

    monkeypatch.setattr(navigation_module, "plan_path", calculate)
    camera.pose(x=484., y=480., scale=600.)
    worker = threading.Thread(target=nav.state)
    worker.start()
    assert entered.wait(.5)
    # La vista previa comenzó detenida; durante A* un Go ya puso al controlador
    # en marcha y este consumió parte de su lista de subpuntos.
    with nav._lock:
        nav._active = True
        nav._avoidance_points = [[700., 300.]]
        nav._steering_target = [700., 300.]
    release.set()
    worker.join(timeout=2.)
    assert not worker.is_alive()
    assert nav._avoidance_points == [[700., 300.]] and nav._steering_target == [700., 300.]
    assert nav._planning_status == "ready"
    assert robot.starts == 0 and robot.drives == []


def test_planning_does_not_hold_pose_lock_or_arm_if_vision_disappears_during_calculation(scene, monkeypatch):
    camera, robot, nav = scene
    goal(nav)
    obstacles(nav)
    original = navigation_module.plan_path
    lock_read = []

    def calculate(*args, **kwargs):
        done = threading.Event()

        def reader():
            lock_read.append(nav.saved_camera_id())
            done.set()

        worker = threading.Thread(target=reader)
        worker.start()
        assert done.wait(.2), "A* bloqueó el lock que utiliza el hilo de cámara."
        worker.join()
        result = original(*args, **kwargs)
        camera.update(visible=False)
        return result

    monkeypatch.setattr(navigation_module, "plan_path", calculate)
    with pytest.raises(NavigationError):
        nav.start()
    assert lock_read and robot.starts == 0 and robot.drives == []


def test_obstacles_disable_blind_continuation_and_recovery_replans_same_waypoint(scene, monkeypatch):
    camera, robot, nav = scene
    goal(nav)
    obstacles(nav)
    calls = []
    original = navigation_module.plan_path

    def count(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    monkeypatch.setattr(navigation_module, "plan_path", count)
    nav.start()
    wait_for(lambda: any(command != (0, 0) for command in robot.drives))
    before = len(calls)
    camera.update(visible=False)
    wait_for(lambda: nav.state()["phase"] == "paused_vision")
    assert robot.drives[-1] == (0, 0) and nav.state()["brief_vision_gaps"] == 0
    stable(camera, nav)
    wait_for(lambda: nav.state()["phase"] in {"turning", "advancing"}, timeout=2.)
    assert len(calls) > before
    assert nav.state()["waypoint_index"] == 0 and nav.state()["target_px"] == [808., 480.]


def test_map_settings_and_source_persist_separately_and_corrupt_map_is_fail_closed(tmp_path):
    camera, robot = FakeCamera(), FakeRobot()
    camera.pose(x=472., y=480., scale=600.)
    path = tmp_path / "navigation-state.json"
    nav = Navigation(camera, robot, path)
    calibration(nav)
    goal(nav)
    obstacles(nav)
    nav.set_obstacle_settings(40, nav.state()["route_revision"])
    saved = json.loads(path.with_name("navigation-obstacles.json").read_text())
    assert saved["source"]["camera_id"] == "usb-camera" and saved["safety_margin_mm"] == 40
    assert json.loads(path.read_text())["version"] == 2
    restored = Navigation(camera, robot, path)
    assert restored.state()["obstacles_px"] == QUAD
    assert restored.state()["obstacle_settings"]["safety_margin_mm"] == 40
    path.with_name("navigation-obstacles.json").write_text("{corrupt")
    blocked = Navigation(camera, robot, path)
    try:
        assert blocked.state()["planning_status"] == "blocked"
        with pytest.raises(NavigationError, match="mapa guardado"):
            blocked.start()
        assert robot.starts == 0 and robot.drives == []
        corrupted = path.with_name("navigation-obstacles.json").read_bytes()
        with pytest.raises(NavigationError):
            blocked.set_obstacle_settings(40, blocked.state()["route_revision"])
        blocked.stop()
        assert path.with_name("navigation-obstacles.json").read_bytes() == corrupted
        assert blocked.state()["planning_status"] == "blocked"
        after_restart = Navigation(camera, FakeRobot(), path)
        try:
            assert after_restart.state()["planning_status"] == "blocked"
            with pytest.raises(NavigationError):
                after_restart.start()
        finally:
            after_restart.close()
        assert blocked.clear_obstacles()["planning_status"] == "ready"
    finally:
        nav.close()
        restored.close()
        blocked.close()


def test_saved_map_from_another_source_cannot_silently_become_free_space(tmp_path):
    camera, robot = FakeCamera(), FakeRobot()
    camera.pose(x=472., y=480., scale=600.)
    path = tmp_path / "navigation-state.json"
    nav = Navigation(camera, robot, path)
    calibration(nav)
    goal(nav)
    obstacles(nav)
    map_path = path.with_name("navigation-obstacles.json")
    stored = json.loads(map_path.read_text())
    stored["source"]["camera_id"] = "different-camera"
    map_path.write_text(json.dumps(stored))
    restored = Navigation(camera, robot, path)
    try:
        assert restored.state()["obstacles_px"] == []
        assert restored.state()["planning_status"] == "blocked"
        with pytest.raises(NavigationError):
            restored.start()
        assert robot.starts == 0
    finally:
        nav.close()
        restored.close()


def test_obstacle_settings_api_cas_validation_and_active_clear(scene):
    _, robot, nav = scene
    goal(nav)
    app = FastAPI()
    app.include_router(navigation_module.make_navigation_router(nav))
    headers = {"X-Requested-With": "XMLHttpRequest"}
    with TestClient(app) as client:
        payload = {"polygons_px": QUAD, "frame_width": 1280, "frame_height": 960,
                   "camera_id": "usb-camera", "expected_route_revision": nav.state()["route_revision"]}
        assert client.post("/api/navigation/obstacles", json=payload).status_code == 403
        response = client.post("/api/navigation/obstacles", json=payload, headers=headers)
        assert response.status_code == 200
        assert client.post("/api/navigation/obstacles", json={**payload, "extra": 1}, headers=headers).status_code == 422
        assert client.post("/api/navigation/obstacles", json=payload, headers=headers).status_code == 409
        for value in (True, None, -1, 201, 30.):
            assert client.post("/api/navigation/obstacle-settings", json={"safety_margin_mm": value,
                "expected_route_revision": nav.state()["route_revision"]}, headers=headers).status_code == 422
        nav.start()
        assert client.post("/api/navigation/obstacle-settings", json={"safety_margin_mm": 30,
            "expected_route_revision": nav.state()["route_revision"]}, headers=headers).status_code == 409
        state = client.post("/api/navigation/clear-obstacles", json={}, headers=headers).json()
        wait_for(lambda: not nav.state()["active"])
        assert state["obstacles_px"] == [] and nav.state()["waypoints_px"] == [[808., 480.]]
        starts = robot.starts
        time.sleep(.03)
        assert robot.starts == starts and nav.state()["active"] is False


def test_local_speed_budget_reserves_reaction_and_braking_space_without_changing_turn_profile(scene):
    camera, robot, nav = scene
    robot.update(max_velocity_mm_s=INT32_MAX)
    nav.set_speeds(INT32_MAX, 32)
    goal(nav)
    state = obstacles(nav)
    steer = state["steering_target_px"]
    heading = math.atan2(480. - steer[1], steer[0] - 472.)
    camera.pose(x=472., y=480., heading=heading, scale=600.)
    nav.start()
    current_steer = nav.state()["steering_target_px"]
    camera.pose(x=472., y=480., heading=math.atan2(480. - current_steer[1], current_steer[0] - 472.), scale=600.)
    wait_for(lambda: nav.state()["phase"] == "advancing", timeout=2.)
    time.sleep(.4)
    clearance = point_clearance_mm(camera.state()["marker_corners"], nav._calibration, [472, 480], QUAD)
    limit = nav._obstacle_speed_limit(clearance, 180.)
    forward = [max(left, right) for left, right in robot.drives if left > 0 and right > 0]
    assert forward and max(forward) <= math.floor(limit) < 300
    assert nav._obstacle_speed_limit(190., 180.) == 0.
    assert nav._obstacle_speed_limit(1000., 180.) > nav._obstacle_speed_limit(220., 180.)
    assert nav._wheel_commands("turning", math.pi / 2, .5, 0.) == (-53, 53)


def test_closed_loop_detours_through_subpoints_without_entering_physical_clearance(monkeypatch):
    class VirtualRobot(FakeRobot):
        def __init__(self, camera):
            super().__init__()
            self.camera = camera
            self.x = self.y = self.heading = 0.
            self.clearances = []

        def drive_velocity_navigation(self, left, right):
            super().drive_velocity_navigation(left, right)
            dt = .02
            speed, angular = (left + right) / 2000., (right - left) / 190.
            delta = angular * dt
            if abs(angular) > 1e-9:
                self.x += speed / angular * (math.sin(self.heading + delta) - math.sin(self.heading))
                self.y += speed / angular * (math.cos(self.heading) - math.cos(self.heading + delta))
            else:
                self.x += speed * math.cos(self.heading) * dt
                self.y += speed * math.sin(self.heading) * dt
            self.heading += delta
            self.camera.pose(x=472 + self.x * 600, y=480 - self.y * 600, heading=self.heading, scale=600.)
            self.clearances.append(point_clearance_mm(self.camera.state()["marker_corners"], nav._calibration,
                                                      [472 + self.x * 600, 480 - self.y * 600], QUAD))

    # El reloj físico de la planta avanza veinte veces más rápido en este ensayo.
    monkeypatch.setattr(StraightControl, "BEARING_FILTER_SECONDS", .12 / 20)
    monkeypatch.setattr(StraightControl, "DERIVATIVE_FILTER_SECONDS", .15 / 20)
    monkeypatch.setattr(StraightControl, "KD", .12 / 20)
    monkeypatch.setattr(StraightControl, "MAX_SLEW_RAD_S2", math.radians(25) * 20)
    camera = FakeCamera()
    camera.pose(x=472., y=480., scale=600.)
    camera.update(running=True, opening=False)
    robot = VirtualRobot(camera)
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .001
    nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 12000., 18000.
    nav.WAYPOINT_SETTLE_SECONDS, nav.SETTLE_SECONDS = .005, .01
    nav.TURN_CONFIRM_SECONDS = .25 / 20
    try:
        calibration(nav)
        nav.set_obstacle_settings(30, nav.state()["route_revision"])
        goal(nav)
        state = obstacles(nav)
        assert len(state["planned_path_px"]) >= 4
        nav.start()
        wait_for(lambda: not nav.state()["active"], timeout=8.)
        state = nav.state()
        assert state["phase"] == "arrived", (state["phase"], state["planning_message"], state["message"])
        assert state["waypoint_index"] == state["waypoint_count"] == 1
        assert min(robot.clearances) >= 180. - .1
        assert robot.starts == 1 and robot.stops == 1
        assert math.hypot(robot.x - .56, robot.y) <= .04
    finally:
        nav.close()


def near_border_scene():
    camera, robot = FakeCamera(), FakeRobot()
    camera.update(frame_height=720, running=True, opening=False)
    # Patrón de40 mm a60 píxeles; el centro del robot está55,48 mm a la
    # izquierda del marcador. El borde visible queda a159 mm del centro.
    camera.pose(x=238.5 + 55.48 * 1.5, y=359., scale=1500.)
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .01
    nav.calibrate([238.5, 359.], [298.5, 359.], 1280, 720, "usb-camera")
    nav.set_obstacle_settings(20, nav.state()["route_revision"])
    return camera, robot, nav


def far_border_obstacle(nav):
    quad = [[[850., 200.], [1025., 200.], [1025., 500.], [850., 500.]]]
    return nav.set_obstacles(quad, 1280, 720, "usb-camera", nav.state()["route_revision"])


def test_camera_border_is_not_treated_as_a_solid_robot_radius_obstacle():
    _, robot, nav = near_border_scene()
    try:
        nav.select_target(450., 359., 1280, 720, "usb-camera")
        state = far_border_obstacle(nav)
        info = state["clearance_info"]
        assert state["planning_status"] == "ready", state["planning_message"]
        assert info["view_mm"] == pytest.approx(159., abs=.2)
        assert info["view_required_mm"] == pytest.approx(103.76, abs=.2)
        assert info["obstacle_mm"] > 390 and info["obstacle_required_mm"] == 170
        # La antigua resta del radio físico al borde habría detenido el robot.
        assert nav._obstacle_speed_limit(info["view_mm"], info["obstacle_required_mm"]) == 0.
        assert nav._obstacle_speed_limit(info["view_mm"], info["view_required_mm"]) > 100.
        nav.start()
        wait_for(lambda: nav.state()["phase"] == "advancing")
        time.sleep(.2)
        assert nav.state()["active"] is True and nav.state()["phase"] == "advancing"
        assert any(left > 50 and right > 50 for left, right in robot.drives)
    finally:
        nav.close()


def test_visible_marker_margin_and_real_obstacle_report_different_blocking_reasons():
    _, robot, nav = near_border_scene()
    try:
        nav.select_target(200., 359., 1280, 720, "usb-camera")
        state = far_border_obstacle(nav)
        assert state["planning_status"] == "blocked"
        assert "destino" in state["planning_message"] and "borde" in state["planning_message"]
        assert "210 mm" not in state["planning_message"]  # El borde usa143,8 mm, no el radio físico.
        with pytest.raises(NavigationError):
            nav.start()
        assert robot.starts == 0
        nav.select_target(900., 359., 1280, 720, "usb-camera")
        state = nav.state()
        assert state["planning_status"] == "blocked"
        assert "obstáculo" in state["planning_message"] and "210 mm" in state["planning_message"]
        with pytest.raises(NavigationError):
            nav.start()
        assert robot.starts == 0
    finally:
        nav.close()


def test_clearance_diagnostics_never_expose_infinity_in_json():
    _, _, nav = near_border_scene()
    try:
        state = nav.state()
        assert state["clearance_info"]["obstacle_mm"] is None
        assert state["clearance_info"]["view_mm"] > 0
        json.dumps(state, allow_nan=False)
    finally:
        nav.close()


def test_zero_mode_defaults_and_json_persistence_remove_all_optional_reserves(tmp_path):
    camera, robot = FakeCamera(), FakeRobot()
    camera.pose(x=472., y=480., scale=600.)
    path = tmp_path / "navigation-state.json"
    nav = Navigation(camera, robot, path)
    assert nav.state()["obstacle_settings"] == {
        "robot_radius_mm": 150, "safety_margin_mm": 0, "tracking_margin_mm": 0}
    calibration(nav)
    goal(nav)
    obstacles(nav)
    app = FastAPI()
    app.include_router(navigation_module.make_navigation_router(nav))
    headers = {"X-Requested-With": "XMLHttpRequest"}
    with TestClient(app) as client:
        for value in (False, 0.0):
            assert client.post("/api/navigation/obstacle-settings", json={
                "safety_margin_mm": value, "expected_route_revision": nav.state()["route_revision"]},
                headers=headers).status_code == 422
        positive = nav.set_obstacle_settings(20, nav.state()["route_revision"])
        assert positive["obstacle_settings"]["tracking_margin_mm"] == 40
        result = client.post("/api/navigation/obstacle-settings", json={
            "safety_margin_mm": 0, "expected_route_revision": nav.state()["route_revision"]}, headers=headers)
        assert result.status_code == 200
        assert result.json()["obstacle_settings"]["tracking_margin_mm"] == 0
    base = math.hypot(*nav._calibration.center_tag_m) * 1000 + 20 * math.sqrt(2)
    assert nav._camera_view_margin(nav._calibration, 0) == pytest.approx(base)
    assert nav._camera_view_margin(nav._calibration, 0, planning=True) == pytest.approx(base)
    assert nav._camera_view_margin(nav._calibration, 20) == pytest.approx(base + 20)
    assert nav._camera_view_margin(nav._calibration, 20, planning=True) == pytest.approx(base + 60)
    assert json.loads(path.with_name("navigation-obstacles.json").read_text())["safety_margin_mm"] == 0
    restored = Navigation(camera, robot, path)
    try:
        assert restored.state()["obstacle_settings"] == nav.state()["obstacle_settings"]
        assert restored.state()["obstacles_px"] == QUAD
        assert robot.starts == robot.stops == 0 and robot.drives == []
    finally:
        nav.close()
        restored.close()


def corridor(upper, lower):
    return [[[600., 0.], [700., 0.], [700., upper], [600., upper]],
            [[600., lower], [700., lower], [700., 959.], [600., 959.]]]


def test_zero_margin_fits_a_real_360mm_corridor_while_positive_margin_blocks(scene):
    _, robot, nav = scene
    goal(nav)
    state = obstacles(nav, corridor(372., 588.))  # 216px / 0.6px/mm =360mm.
    assert state["planning_status"] == "blocked"
    zero = nav.set_obstacle_settings(0, nav.state()["route_revision"])
    assert zero["planning_status"] == "ready", zero["planning_message"]
    assert zero["obstacle_settings"] == {"robot_radius_mm": 150, "safety_margin_mm": 0, "tracking_margin_mm": 0}
    assert robot.starts == 0 and robot.drives == []


def test_zero_margin_adaptive_budget_can_command_below_50_without_forcing_the_floor(scene):
    camera, robot, nav = scene
    nav.set_obstacle_settings(0, nav.state()["route_revision"])
    goal(nav)
    state = obstacles(nav, corridor(387., 573.))  #310mm, solo5mm libres por lado al centro.
    assert state["planning_status"] == "ready", state["planning_message"]
    limit = nav._obstacle_speed_limit(155., 150., reserve_mm=0)
    assert 1 < limit < 50
    assert nav._obstacle_speed_limit(155., 150.) == 0  # El perfil positivo conserva reserva15.
    nav.start()
    camera.pose(x=640., y=480., scale=600.)
    wait_for(lambda: nav._avoidance_index == 1, timeout=2.)
    wait_for(lambda: nav.state()["phase"] == "advancing", timeout=2.)
    first = len(robot.drives)
    time.sleep(.2)
    commands = [command for command in robot.drives[first:] if command != (0, 0)]
    assert commands and all(0 < left <= math.floor(limit) and 0 < right <= math.floor(limit)
                            for left, right in commands)
    assert nav.state()["active"] is True and nav.state()["phase"] == "advancing"


def test_all_goals_are_prevalidated_and_occupied_point_two_is_reported_before_astar(scene, monkeypatch):
    _, robot, nav = scene
    nav.set_obstacle_settings(0, nav.state()["route_revision"])
    goal(nav)
    nav.append_target(640., 480., 1280, 960, "usb-camera", nav.state()["route_revision"])

    def forbidden(*args, **kwargs):
        raise AssertionError("No se debe ejecutar A* antes de revisar todos los destinos.")

    monkeypatch.setattr(navigation_module, "plan_path", forbidden)
    state = obstacles(nav)
    assert state["planning_status"] == "blocked"
    assert state["planning_message"].startswith("Punto 2:")
    assert "envolvente del robot" in state["planning_message"] and "150 mm" in state["planning_message"]
    assert state["waypoints_px"] == [[808., 480.], [640., 480.]]
    with pytest.raises(NavigationError):
        nav.start()
    assert robot.starts == 0 and robot.drives == []


def test_first_blocked_leg_is_labelled_one_even_with_two_valid_goals(scene, monkeypatch):
    _, _, nav = scene
    nav.set_obstacle_settings(0, nav.state()["route_revision"])
    goal(nav)
    nav.append_target(808., 600., 1280, 960, "usb-camera", nav.state()["route_revision"])

    def blocked(*args, **kwargs):
        raise ValueError("No hay paso entre los objetos.")

    monkeypatch.setattr(navigation_module, "plan_path", blocked)
    state = obstacles(nav)
    assert state["planning_message"].startswith("Punto 1:")


def test_preflight_geometry_valueerror_is_blocked_not_a_server_error(scene, monkeypatch):
    _, robot, nav = scene
    goal(nav)

    def invalid(*args, **kwargs):
        raise ValueError("La proyección de la imagen no es válida.")

    monkeypatch.setattr(navigation_module, "point_clearances_mm", invalid)
    state = obstacles(nav)
    assert state["planning_status"] == "blocked" and state["planning_message"].startswith("Punto 1:")
    assert state["clearance_info"] is None
    with pytest.raises(NavigationError):
        nav.start()
    assert robot.starts == 0 and robot.drives == []
