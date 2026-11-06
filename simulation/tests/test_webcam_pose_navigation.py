"""Aceptación de maniobras orientadas con cámara y ruedas virtuales.

La planta integra consignas firmadas durante 20 ms por ciclo y publica una
imagen nueva. No abre cámaras, USB ni conexiones del robot. Comprueba rutas y
signos cinemáticos; no constituye una calibración de la dinámica física.
"""
from __future__ import annotations

from contextlib import contextmanager
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import pytest

from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot, wait_for
from simulation.webcam.navigation import Navigation, NavigationError
from simulation.webcam.obstacle_planner import camera_visibility_margin_mm, point_clearances_mm
from simulation.webcam.robot_footprint import ROBOT_FOOTPRINT_MM, ROBOT_RADIUS_MM
from simulation.webcam.straight_control import StraightControl


CAMERA_ID = "recorded-camera"
SCALE = 800.0
FRAME_WIDTH, FRAME_HEIGHT = 1280, 960
FIXTURE = Path(__file__).with_name("fixtures") / "webcam_obstacle_pose_scene.json"


def pixel(point):
    return [640.0 + point[0] * SCALE, 480.0 - point[1] * SCALE]


def obstacle(y=.100):
    return [[(-.180, y), (.180, y), (.180, .250), (-.180, .250)]]


def projected_obstacles(polygons):
    return [[pixel(point) for point in polygon] for polygon in polygons]


def body_polygon(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[x * 1000.0 + c * px - s * py,
                      y * 1000.0 + s * px + c * py]
                     for px, py in ROBOT_FOOTPRINT_MM], dtype=np.float32)


def overlap_area(x, y, yaw, polygons):
    body = body_polygon(x, y, yaw)
    return sum(cv2.intersectConvexConvex(body, np.asarray(polygon, dtype=np.float32) * 1000.0)[0]
               for polygon in polygons)


def penetration_mm(x, y, yaw, polygons):
    """Profundidad independiente por separación de ejes, en mm y sin margen."""
    body = body_polygon(x, y, yaw).astype(np.float64)
    maximum = 0.0
    for polygon in polygons:
        other = np.asarray(polygon, dtype=np.float64) * 1000.0
        axes = []
        for shape in (body, other):
            edges = np.roll(shape, -1, axis=0) - shape
            normals = np.column_stack((-edges[:, 1], edges[:, 0]))
            normals /= np.linalg.norm(normals, axis=1)[:, None]
            axes.extend(normals)
        axes = np.asarray(axes)
        first, second = body @ axes.T, other @ axes.T
        depths = np.minimum(first.max(axis=0) - second.min(axis=0),
                            second.max(axis=0) - first.min(axis=0))
        maximum = max(maximum, float(depths.min()))
    return max(0.0, maximum)


class SignedWheelPlant(FakeRobot):
    """Planta diferencial exacta para mandos constantes y entre ejes de 190 mm."""

    def __init__(self, camera, start, polygons):
        super().__init__()
        self.camera = camera
        self.x, self.y, self.yaw = start
        self.polygons = polygons
        self.trace = []
        self.navigation = None

    def drive_velocity_navigation(self, left, right):
        super().drive_velocity_navigation(left, right)
        previous_pose = (self.x, self.y, self.yaw)
        dt = .020
        speed, angular = (left + right) / 2000.0, (right - left) / 190.0
        delta = angular * dt
        if abs(angular) > 1e-9:
            self.x += speed / angular * (math.sin(self.yaw + delta) - math.sin(self.yaw))
            self.y += speed / angular * (math.cos(self.yaw) - math.cos(self.yaw + delta))
        else:
            self.x += speed * math.cos(self.yaw) * dt
            self.y += speed * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + delta), math.cos(self.yaw + delta))
        index = self.navigation._pose_step_index
        steps = self.navigation._pose_steps
        step = steps[index] if index < len(steps) else {}
        self.trace.append({"position": (self.x, self.y), "yaw": self.yaw,
                           "previous_pose": previous_pose,
                           "left": left, "right": right,
                           "overlap_mm2": overlap_area(self.x, self.y, self.yaw, self.polygons),
                           "penetration_mm": penetration_mm(self.x, self.y, self.yaw, self.polygons),
                           "step_index": index, "kind": step.get("kind"),
                           "step": dict(step),
                           "escaping": step.get("kind") == "escape" or step.get("escaping") is True})
        px, py = pixel((self.x, self.y))
        self.camera.pose(x=px, y=py, heading=self.yaw, scale=SCALE)


@contextmanager
def synthetic_scene(*, start=(0.0, 0.0, math.pi / 2), polygons=None, plant=False):
    polygons = obstacle() if polygons is None else polygons
    camera = FakeCamera()
    camera.update(frame_width=FRAME_WIDTH, frame_height=FRAME_HEIGHT, camera_id=CAMERA_ID,
                  running=True, opening=False)
    center = pixel(start[:2])
    camera.pose(x=center[0], y=center[1], heading=start[2], scale=SCALE)
    robot = SignedWheelPlant(camera, start, polygons) if plant else FakeRobot()
    robot.update(max_velocity_mm_s=1000)
    nav = Navigation(camera, robot)
    if plant:
        robot.navigation = nav
    nav.CONTROL_INTERVAL = .001
    nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 12000.0, 18000.0
    nav.RESUME_STABLE_SECONDS, nav.RESUME_RETRY_SECONDS = .015, .015
    nav.SETTLE_SECONDS, nav.WAYPOINT_SETTLE_SECONDS = .010, .005
    front = pixel((start[0] + .04 * math.cos(start[2]), start[1] + .04 * math.sin(start[2])))
    nav.calibrate(center, front, FRAME_WIDTH, FRAME_HEIGHT, CAMERA_ID)
    nav.set_speeds(200, 400)
    nav.set_obstacles(projected_obstacles(polygons), FRAME_WIDTH, FRAME_HEIGHT, CAMERA_ID,
                      nav.state()["route_revision"])
    try:
        yield camera, robot, nav
    finally:
        nav.close()


def select(nav, point=(-.400, .300)):
    return nav.select_target(*pixel(point), FRAME_WIDTH, FRAME_HEIGHT, CAMERA_ID)


def assert_forward_route(steps):
    """Solo el prefijo de liberación puede retroceder antes del recorrido."""
    normal_drive_seen = False
    for step in steps:
        if step["kind"] == "drive":
            assert step["direction"] == 1, step
            normal_drive_seen = True
        if step["direction"] < 0:
            assert step["kind"] == "escape", step
            assert not normal_drive_seen, step


def test_zero_margin_preview_accepts_oriented_start_rejected_by_a_circle_without_arming():
    with synthetic_scene() as (camera, robot, nav):
        state = select(nav)
        distances = point_clearances_mm(camera.state()["marker_corners"], nav._calibration,
                                        state["center_px"], state["obstacles_px"])
        assert distances["obstacle_mm"] == pytest.approx(100.0, abs=.01)
        assert distances["obstacle_mm"] < ROBOT_RADIUS_MM
        assert state["collision_model"] == "oriented_footprint"
        assert state["planning_status"] == "ready", state["planning_message"]
        assert len(state["robot_footprint_px"]) >= 3
        assert state["clearance_info"]["footprint_gap_mm"] > 10.0
        assert state["inflated_obstacles_px"] == []
        assert state["planned_path_px"][0] == pytest.approx(state["center_px"], abs=.01)
        assert state["planned_path_px"][-1] == pytest.approx(pixel((-.4, .3)), abs=.01)
        assert state["maneuver"]["kind"] in {"drive", "turn", "escape"}
        assert_forward_route(nav._pose_steps)
        assert robot.starts == robot.stops == 0 and robot.drives == []


def test_recorded_projective_camera_accepts_visible_goal_80mm_from_the_edge():
    data = json.loads(FIXTURE.read_text())
    camera, robot = FakeCamera(), FakeRobot()
    camera.update(camera_id=CAMERA_ID, frame_width=data["frame_width"],
                  frame_height=data["frame_height"], running=True, opening=False,
                  marker_corners=data["recorded_pose"]["marker_corners"])
    nav = Navigation(camera, robot)
    try:
        nav.calibrate(data["recorded_pose"]["center_px"], data["recorded_pose"]["front_px"],
                      data["frame_width"], data["frame_height"], CAMERA_ID)
        nav.set_obstacles(data["obstacles_px"], data["frame_width"], data["frame_height"], CAMERA_ID,
                          nav.state()["route_revision"])
        goal = data["visible_pose_at_camera_edge_80_mm"]["center_px"]
        distances = point_clearances_mm(camera.state()["marker_corners"], nav._calibration,
                                        goal, data["obstacles_px"])
        assert distances["camera_mm"] == pytest.approx(80.0, abs=.01)
        assert distances["camera_mm"] < camera_visibility_margin_mm(nav._calibration, extra_margin_mm=0)
        state = nav.select_target(*goal, data["frame_width"], data["frame_height"], CAMERA_ID)
        assert state["planning_status"] == "ready", state["planning_message"]
        assert state["collision_model"] == "oriented_footprint"
        assert state["clearance_info"]["view_required_mm"] == 0.0
        assert state["waypoints_px"] == [goal]
        assert state["planned_path_px"][-1] == pytest.approx(goal, abs=.01)
        assert robot.starts == robot.stops == 0 and robot.drives == []
    finally:
        nav.close()


def test_full_wall_still_has_no_path_and_never_arms():
    wall = [[(-.02, -.599), (.02, -.599), (.02, .599), (-.02, .599)]]
    with synthetic_scene(start=(-.3, 0.0, 0.0), polygons=wall) as (_, robot, nav):
        state = select(nav, (.3, 0.0))
        assert state["collision_model"] == "oriented_footprint"
        assert state["planning_status"] == "blocked"
        assert state["planned_path_px"] == []
        with pytest.raises(NavigationError):
            nav.start()
        assert robot.starts == 0 and robot.drives == []


@pytest.mark.parametrize("obstacle_y", [.100, .080])
def test_signed_wheel_feedback_completes_detour_and_shallow_contact_escape(obstacle_y, monkeypatch):
    # La planta avanza 20 ms por ciclo de 1 ms: filtros y rampas usan el mismo factor 20.
    monkeypatch.setattr(StraightControl, "BEARING_FILTER_SECONDS", .12 / 20)
    monkeypatch.setattr(StraightControl, "DERIVATIVE_FILTER_SECONDS", .15 / 20)
    monkeypatch.setattr(StraightControl, "KD", .12 / 20)
    monkeypatch.setattr(StraightControl, "MAX_SLEW_RAD_S2", math.radians(25) * 20)
    polygons = obstacle(obstacle_y)
    with synthetic_scene(polygons=polygons, plant=True) as (_, robot, nav):
        initial_depth = penetration_mm(robot.x, robot.y, robot.yaw, polygons)
        state = select(nav)
        assert state["planning_status"] == "ready", state["planning_message"]
        assert_forward_route(nav._pose_steps)
        nav.start()
        wait_for(lambda: not nav.state()["active"], timeout=12.0)
        result = nav.state()
        assert result["phase"] == "arrived", (result["phase"], result["planning_message"], result["message"])
        assert result["waypoint_index"] == result["waypoint_count"] == 1
        assert math.hypot(robot.x + .4, robot.y - .3) <= nav.SETTLED_M
        backwards = [row for row in robot.trace if row["left"] + row["right"] < 0]
        if initial_depth > .002:
            assert backwards
        else:
            # Puede liberar un giro bloqueado sin que exista contacto en la
            # orientación inicial: a cero grados, la rueda invadiría la barrera.
            assert penetration_mm(0.0, 0.0, 0.0, polygons) > 0.0
        assert all(row["kind"] == "escape" and row["step"]["direction"] < 0
                   for row in backwards)
        normal_drive_seen = False
        for row in robot.trace:
            if row["kind"] == "drive" and row["left"] + row["right"] > 0:
                normal_drive_seen = True
            if row["left"] + row["right"] < 0:
                assert not normal_drive_seen, row
        assert all(row["step"]["direction"] == 1 and row["left"] + row["right"] >= 0
                   for row in robot.trace if row["kind"] == "drive")
        assert any(row["left"] * row["right"] < 0 for row in robot.trace)
        assert robot.starts == 1 and robot.stops == 1
        previous = initial_depth
        for row in robot.trace:
            depth = row["penetration_mm"]
            if row["escaping"]:
                # El escape admite 50 micrómetros de redondeo por muestra;
                # medir área haría depender esa tolerancia del ancho del borde.
                assert depth <= previous + .05, row
                assert depth <= initial_depth + .05, row
            else:
                # Al terminar el escape, solo queda la precisión geométrica de
                # dos micrómetros; la ruta libre no admite esa tolerancia de contacto.
                assert depth <= .002, json.dumps(row)
            previous = depth
        assert robot.trace[-1]["penetration_mm"] <= .002


def test_destination_behind_robot_turns_around_then_drives_forward():
    far_obstacle = [[(.3, .2), (.4, .2), (.4, .3), (.3, .3)]]
    with synthetic_scene(start=(0.0, 0.0, 0.0), polygons=far_obstacle, plant=True) as (_, robot, nav):
        selected = select(nav, (-.2, 0.0))
        assert selected["planning_status"] == "ready", selected["planning_message"]
        steps = list(nav._pose_steps)
        assert_forward_route(steps)
        first_drive = next(index for index, step in enumerate(steps) if step["kind"] == "drive")
        assert any(step["kind"] == "turn" for step in steps[:first_drive])
        assert math.cos(steps[first_drive]["heading_rad"]) < -.99
        assert all(step["kind"] != "escape" for step in steps)
        nav.start()
        wait_for(lambda: not nav.state()["active"], timeout=6.0)
        result = nav.state()
        assert result["phase"] == "arrived", (result["phase"], result["planning_message"], result["message"])
        assert math.hypot(robot.x + .2, robot.y) <= nav.SETTLED_M
        assert all(row["left"] + row["right"] >= 0 for row in robot.trace)
        assert any(row["left"] * row["right"] < 0 for row in robot.trace)


@pytest.mark.parametrize("stop_kind", ["interface", "D48"])
def test_stop_and_d48_cancel_movement_but_keep_user_route_and_map(stop_kind):
    far_obstacle = [[(.3, .2), (.4, .2), (.4, .3), (.3, .3)]]
    with synthetic_scene(start=(0.0, 0.0, 0.0), polygons=far_obstacle) as (_, robot, nav):
        before = select(nav, (.2, 0.0))
        nav.start()
        wait_for(lambda: any(command != (0, 0) for command in robot.drives))
        if stop_kind == "interface":
            nav.stop()
        else:
            robot.update(stop_reason="button_stop", navigation_active=False, moving=False)
        wait_for(lambda: not nav.state()["active"])
        after = nav.state()
        assert after["waypoints_px"] == before["waypoints_px"]
        assert after["obstacles_px"] == before["obstacles_px"]
        assert after["calibrated"] and not after["pending_mission"]
        assert after["last_left"] == after["last_right"] == 0
        starts = robot.starts
        time.sleep(.02)
        assert robot.starts == starts


def test_vision_pause_stops_immediately_then_replans_the_same_user_destination():
    far_obstacle = [[(.3, .2), (.4, .2), (.4, .3), (.3, .3)]]
    with synthetic_scene(start=(0.0, 0.0, 0.0), polygons=far_obstacle) as (camera, robot, nav):
        before = select(nav, (.2, 0.0))
        nav.start()
        wait_for(lambda: any(command != (0, 0) for command in robot.drives))
        camera.update(visible=False)
        wait_for(lambda: nav.state()["phase"] == "paused_vision")
        paused = nav.state()
        assert robot.drives[-1] == (0, 0)
        assert paused["waypoints_px"] == before["waypoints_px"]
        assert paused["waypoint_index"] == 0
        assert paused["robot_footprint_px"] == []
        camera.update(visible=True, frame_age_ms=0.0)
        for _ in range(nav.RESUME_FRAMES + 2):
            camera.pose(x=640.0, y=480.0, heading=0.0, scale=SCALE)
            time.sleep(.008)
        wait_for(lambda: nav.state()["phase"] in {"advancing", "turning"}, timeout=3.0)
        restored = nav.state()
        assert restored["planning_status"] == "ready"
        assert restored["waypoints_px"] == before["waypoints_px"]
        assert restored["waypoint_index"] == 0
        assert restored["collision_model"] == "oriented_footprint"
        assert restored["robot_footprint_px"]


def test_coincident_oriented_goal_completes_without_a_nonzero_command():
    with synthetic_scene() as (_, robot, nav):
        state = select(nav, (0., 0.))
        assert state['planning_status'] == 'ready'
        nav.start(state['route_revision'])
        wait_for(lambda: not nav.state()['active'], timeout=2.)
        state = nav.state()
        assert state['phase'] == 'arrived'
        assert state['waypoint_index'] == 1
        assert all(left == right == 0 for left, right in robot.drives)
