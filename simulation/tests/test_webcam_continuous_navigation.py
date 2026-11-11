"""Recorrido continuo y salida de un pasillo, sin acceso al robot físico.

La cámara grabada solo aporta geometría anónima. La planta diferencial integra
los mandos en un plano fijo y vuelve a generar las esquinas del marcador.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import pytest

from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot, ClosedLoopRobot, calibrate, wait_for
from simulation.webcam.navigation import Navigation, INT32_MAX
from simulation.webcam.navigation_geometry import Calibration, _homographies, _project, observe_robot
from simulation.webcam.pose_follower import PoseFollower
from simulation.webcam.pose_obstacle_planner import (
    _Scene, check_pose, check_swept_motion, plan_pose_path,
)
from simulation.webcam.robot_footprint import ROBOT_FOOTPRINT_MM


SCENE = json.loads((Path(__file__).with_name("fixtures") / "webcam_six_wall_scene.json").read_text())
CALIBRATION = Calibration.from_dict(SCENE["calibration"])
CORNERS = SCENE["marker_corners"]
OBSTACLES = SCENE["obstacles_px"]
GOALS = SCENE["waypoints_px"]


def validate_recorded_route(corners, *, start=None, heading=None, minimum_gap_mm=10.):
    current = observe_robot(corners, CALIBRATION)
    start = current["center_px"] if start is None else start
    heading = math.radians(current["heading_deg"]) if heading is None else heading
    report = check_pose(corners, CALIBRATION, start, heading, OBSTACLES)
    assert report["clear"]
    assert minimum_gap_mm < report["clearance_mm"] < 25.
    began = time.monotonic()
    result = plan_pose_path(corners, CALIBRATION, start, GOALS[0], OBSTACLES,
                            start_heading_rad=heading, max_seconds=3.)
    assert time.monotonic() - began < 3.25
    assert result["path_px"][-1] == pytest.approx(GOALS[0], abs=.003)
    assert np.all(np.isfinite(result["path_px"]))
    previous = start
    normal_motion_seen = False
    reverse_distance = 0.
    previous_local = np.asarray(observe_robot(corners, CALIBRATION, start)["target_local_m"])
    for step in result["steps"]:
        escape = step["kind"] == "escape" or step.get("escaping", False)
        end_local = np.asarray(observe_robot(corners, CALIBRATION, step["target_px"])["target_local_m"])
        if step["direction"] < 0:
            assert escape and not normal_motion_seen
            reverse_distance += float(np.linalg.norm(end_local - previous_local))
        if step["kind"] == "drive":
            assert step["direction"] == 1
            normal_motion_seen = True
        swept = check_swept_motion(corners, CALIBRATION, previous, heading,
            step["target_px"], step["heading_rad"], OBSTACLES,
            allow_start_overlap=escape, max_penetration_mm=step.get("max_penetration_mm"))
        assert swept["clear"], (step, swept)
        previous, heading, previous_local = step["target_px"], step["heading_rad"], end_local
    assert reverse_distance <= .08001
    assert check_pose(corners, CALIBRATION, previous, heading, OBSTACLES)["clear"]
    return result


def test_six_wall_free_start_can_recover_instead_of_reporting_no_motion_sequence():
    validate_recorded_route(CORNERS)


def test_free_pose_near_the_corner_can_turn_then_back_up_before_resuming():
    pose = SCENE["free_following_recovery_pose"]
    result = validate_recorded_route(CORNERS, start=pose["center_px"],
                                    heading=pose["heading_rad"], minimum_gap_mm=0.)
    first_reverse = next(i for i, step in enumerate(result["steps"]) if step["direction"] < 0)
    assert any(step["kind"] == "turn" for step in result["steps"][:first_reverse])


@pytest.mark.parametrize("seed", [3, 17])
def test_small_camera_corner_changes_keep_the_free_corridor_navigable(seed):
    corners = np.asarray(CORNERS) + np.random.default_rng(seed).normal(0., .10, (4, 2))
    validate_recorded_route(corners.tolist())


@pytest.mark.parametrize("direction", [-1, 1])
def test_a_five_mm_recovery_is_not_mistaken_for_an_already_completed_step(direction):
    step = {"kind": "escape", "direction": direction, "target_px": [100., 100.],
            "heading_rad": 0., "escaping": True}
    result = PoseFollower().update(step,
        {"distance_m": .005, "bearing_rad": math.pi if direction < 0 else 0.},
        0., 1., {"forward_mm_s": 200, "turn_deg_s": 100})
    assert not result["done"] and not result["replan"]
    assert result["left"] * direction > 0 and result["right"] * direction > 0


class RecordedWheelPlant(FakeRobot):
    """Ruedas ideales separadas 190 mm; proyección fija del marcador de 40 mm."""

    def __init__(self, camera):
        super().__init__()
        self.camera = camera
        self.navigation = None
        self.center = np.array(CALIBRATION.center_tag_m)
        self.yaw = CALIBRATION.front_angle_rad
        _, self.projection, _ = _homographies(CORNERS, .04, 1280, 720)
        self.scene = _Scene(CORNERS, CALIBRATION, OBSTACLES, ROBOT_FOOTPRINT_MM)
        self.trace = []
        self.visited = []

    def image_pose(self, position=None, yaw=None):
        position = self.center if position is None else position
        yaw = self.yaw if yaw is None else yaw
        center = _project(self.projection, position, np.zeros(2))
        front = _project(self.projection,
                        position + .02 * np.array([math.cos(yaw), math.sin(yaw)]),
                        np.zeros(2))
        angle = math.atan2(-(front[1] - center[1]), front[0] - center[0])
        return center.tolist(), angle

    def publish(self):
        angle = self.yaw - CALIBRATION.front_angle_rad
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        origin = self.center - rotation @ np.asarray(CALIBRATION.center_tag_m)
        corners = [_project(self.projection, origin + rotation @ np.array(point), np.zeros(2)).tolist()
                   for point in [(-.02, .02), (.02, .02), (.02, -.02), (-.02, -.02)]]
        self.camera.update(marker_corners=corners)

    def drive_velocity_navigation(self, left, right):
        super().drive_velocity_navigation(left, right)
        dt = .020
        speed, angular = (left + right) / 2000., (right - left) / 190.
        previous, previous_yaw = self.image_pose()
        origin = self.center.copy()
        peak = 0.
        marker_visible = True
        # Las cuatro poses interiores siguen el arco diferencial real, en vez
        # de comprobar solo la posición final o el segmento entre sus extremos.
        for fraction in (.25, .5, .75, 1.):
            angle = angular * dt * fraction
            if abs(angular) > 1e-9:
                position = origin + speed / angular * np.array([
                    math.sin(self.yaw + angle) - math.sin(self.yaw),
                    math.cos(self.yaw) - math.cos(self.yaw + angle)])
            else:
                position = origin + speed * dt * fraction * np.array([math.cos(self.yaw), math.sin(self.yaw)])
            center_px, image_yaw = self.image_pose(position, self.yaw + angle)
            depths, visible, inside = self.scene.evaluate(self.scene.pose(center_px, image_yaw))
            marker_visible = marker_visible and visible and inside
            peak = max(peak, float(np.max(depths, initial=0.)) * 1000.)
        delta = angular * dt
        if abs(angular) > 1e-9:
            self.center += speed / angular * np.array([
                math.sin(self.yaw + delta) - math.sin(self.yaw),
                math.cos(self.yaw) - math.cos(self.yaw + delta)])
        else:
            self.center += speed * dt * np.array([math.cos(self.yaw), math.sin(self.yaw)])
        self.yaw += delta
        center, heading = self.image_pose()
        self.trace.append({"left": left, "right": right, "center": center, "heading": heading,
                           "previous_center": previous, "previous_heading": previous_yaw,
                           "penetration_mm": peak,
                           "marker_visible": marker_visible,
                           "phase": self.navigation._phase,
                           "steering_target_px": self.navigation._steering_target,
                           "pose_step_index": self.navigation._pose_step_index,
                           "waypoint_index": self.navigation._waypoint_index})
        self.publish()


def configure_recorded_navigation(speeds=(200, 100)):
    camera = FakeCamera()
    camera.update(camera_id="recorded-camera", frame_width=1280, frame_height=720,
                  marker_corners=CORNERS, running=True, opening=False)
    robot = RecordedWheelPlant(camera)
    robot.update(max_velocity_mm_s=INT32_MAX)
    nav = Navigation(camera, robot)
    robot.navigation = nav
    nav._calibration = CALIBRATION
    nav._waypoints = [point[:] for point in GOALS]
    nav._sync_target_locked()
    nav._obstacles = [[point[:] for point in polygon] for polygon in OBSTACLES]
    nav._obstacle_source = {"camera_id": "recorded-camera", "frame_width": 1280, "frame_height": 720}
    nav._phase = "selected"
    original_sync = nav._sync_target_locked

    def record_progress():
        original_sync()
        if nav._active and nav._waypoint_index:
            robot.visited.append((nav._waypoint_index, robot.center.copy(), robot.yaw))

    nav._sync_target_locked = record_progress
    nav.CONTROL_INTERVAL = .001
    nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 12000., 18000.
    nav.RESUME_STABLE_SECONDS, nav.RESUME_RETRY_SECONDS = .015, .015
    nav.WAYPOINT_SETTLE_SECONDS, nav.SETTLE_SECONDS = .005, .01
    nav.set_speeds(*speeds)
    # La geometría tarda un tiempo variable de CPU. El reloj del controlador
    # normal y su rampa avanzan exactamente los mismos 20 ms que las ruedas;
    # multiplicar una aceleración por20 usando tiempo real no sería equivalente.
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

    nav._continuous_command = logical_command
    return camera, robot, nav


@pytest.mark.parametrize("speeds", [(200, 100), (600, 1600)])
def test_recorded_six_wall_feedback_completes_all_four_destinations_without_deadlock(speeds, tmp_path):
    _, robot, nav = configure_recorded_navigation(speeds)
    phases = []
    progress = []
    try:
        state = nav.state()
        assert state["planning_status"] == "ready", state["planning_message"]
        nav.start(state["route_revision"])
        deadline = time.monotonic() + 35.
        while nav.state()["active"] and time.monotonic() < deadline:
            phases.append(nav.state()["phase"])
            progress.append(nav.state()["waypoint_index"])
            time.sleep(.003)
        state = nav.state()
        report_path = tmp_path / "six-wall-feedback.json"
        report_path.write_text(json.dumps({"speeds": speeds, "state": state, "phases": phases,
            "progress": progress, "trace": robot.trace,
            "visited": [(index, center.tolist(), yaw) for index, center, yaw in robot.visited]}, ensure_ascii=False))
        assert state["phase"] == "arrived", (state["phase"], state["message"], state["planning_message"], str(report_path))
        assert state["waypoint_index"] == state["waypoint_count"] == 4
        assert [index for index, _, _ in robot.visited] == [1, 2, 3, 4]
        world_from_image = np.linalg.inv(robot.projection)
        for index, center, yaw in robot.visited:
            target = _project(world_from_image, np.asarray(GOALS[index - 1]), np.zeros(2))
            rotation = np.asarray([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
            local = rotation @ (target - center) * 1000.
            assert cv2.pointPolygonTest(np.asarray(ROBOT_FOOTPRINT_MM, dtype=np.float32),
                                       tuple(local), False) >= 0.
        # Un sondeo de interfaz puede saltarse una transición si el hilo de
        # control avanza mientras se calcula el estado visual. Las cuatro
        # visitas y su cobertura se registran de forma síncrona arriba.
        assert progress and set(progress) <= {0, 1, 2, 3, 4}
        assert progress == sorted(progress)
        assert max(row["penetration_mm"] for row in robot.trace) <= .002
        assert all(row["marker_visible"] for row in robot.trace)
        assert "paused_obstacle" not in phases
        # P1→P2 invierte el rumbo sobre un tramo que se solapa. La cobertura
        # anticipada por la carrocería no debe dejar el punto adelantado en el
        # tramo ya cumplido. Tampoco debe invertirse la elección del giro por
        # la velocidad angular residual y provocar vueltas completas.
        for goal_index in range(4):
            accumulated_rotation = sum(abs(row["right"] - row["left"]) / 190. * .020
                                       for row in robot.trace if row["waypoint_index"] == goal_index)
            assert accumulated_rotation <= 2.5 * math.pi, (goal_index, accumulated_rotation, str(report_path))
        # Son llamadas de apertura/cierre de sesión; las maniobras de salida
        # pueden mandar cero brevemente sin abrir otra sesión.
        assert robot.starts == robot.stops == 1
        assert robot.drives[-1] == (0, 0)
    finally:
        nav.close()


def test_collinear_user_points_do_not_insert_zero_velocity_pauses():
    camera = FakeCamera()
    robot = ClosedLoopRobot(camera)
    robot.update(max_velocity_mm_s=1000)
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .001
    nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 12000., 18000.
    nav.SETTLE_SECONDS = .010
    samples = []
    original = robot.drive_velocity_navigation

    def drive(left, right):
        samples.append((nav._waypoint_index, left, right))
        original(left, right)

    robot.drive_velocity_navigation = drive
    try:
        calibrate(nav)
        nav.set_speeds(200, 100)
        state = nav.select_target(880., 480., 1280, 960, "usb-camera")
        state = nav.append_target(1040., 480., 1280, 960, "usb-camera", state["route_revision"])
        nav.append_target(1200., 480., 1280, 960, "usb-camera", state["route_revision"])
        nav.start()
        wait_for(lambda: not nav.state()["active"], timeout=5.)
        assert nav.state()["phase"] == "arrived"
        first_motion = next(i for i, (_, left, right) in enumerate(samples) if left > 0 and right > 0)
        # El índice 2 ya corresponde al destino final: allí sí debe frenar.
        # Excluir únicamente el arranque y ese frenado, no las transiciones
        # entre los dos puntos anteriores.
        intermediate = [(left, right) for index, left, right in samples[first_motion:] if index < 2]
        assert intermediate
        assert all(left > 0 and right > 0 for left, right in intermediate)
        assert samples[-1][1:] == (0, 0)
    finally:
        nav.close()
