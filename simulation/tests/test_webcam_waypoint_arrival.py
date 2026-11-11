"""Llegada por cobertura del robot, con cámara y ruedas ficticias.

No se accede a webcam, USB, red ni motores reales. Los destinos se expresan
respecto al centro entre ruedas y el frente calibrado del robot.
"""
from __future__ import annotations

import math

import pytest

from simulation.tests.test_webcam_point_navigation import wait_for
from simulation.tests.test_webcam_pose_navigation import (
    CAMERA_ID, FRAME_HEIGHT, FRAME_WIDTH, SCALE, pixel, select, synthetic_scene,
)
import simulation.webcam.navigation as navigation_module
from simulation.webcam.robot_footprint import point_under_robot


REMOTE_OBSTACLE = [[(.40, .30), (.50, .30), (.50, .40), (.40, .40)]]


@pytest.mark.parametrize("local", [(.080, 0.), (0., .090), (0., -.090), (-.050, 0.),
                                    (.08315, .032), (-.05285, -.026)])
def test_point_under_robot_accepts_body_wheels_and_boundary(local):
    assert point_under_robot(local)


@pytest.mark.parametrize("local", [(.084, 0.), (-.054, 0.), (0., .117),
                                    (.080, .090), (.080, -.090)])
def test_point_under_robot_rejects_empty_bounding_box_corners(local):
    assert not point_under_robot(local)


@pytest.mark.parametrize("local", [(.080, 0.), (0., .090), (-.050, 0.)])
@pytest.mark.parametrize("yaw", [0., math.pi / 2, math.pi])
def test_covered_user_point_arrives_without_center_alignment_or_extra_motion(local, yaw):
    with synthetic_scene(start=(0., 0., yaw), polygons=[]) as (_, robot, nav):
        x = math.cos(yaw) * local[0] - math.sin(yaw) * local[1]
        y = math.sin(yaw) * local[0] + math.cos(yaw) * local[1]
        state = select(nav, (x, y))
        assert state["remaining_mm"] > 40.
        nav.start()
        wait_for(lambda: not nav.state()["active"])
        state = nav.state()
        assert state["phase"] == "arrived", state["message"]
        assert state["waypoint_index"] == state["waypoint_count"] == 1
        assert all(left == right == 0 for left, right in robot.drives)


def test_a_point_in_an_empty_footprint_corner_still_requires_navigation():
    with synthetic_scene(start=(0., 0., 0.), polygons=[]) as (_, robot, nav):
        select(nav, (.080, .090))
        nav.start()
        wait_for(lambda: any(left != 0 or right != 0 for left, right in robot.drives))
        assert nav.state()["active"]
        assert nav.state()["waypoint_index"] == 0


def test_oriented_goal_already_under_robot_uses_a_zero_motion_preview():
    with synthetic_scene(start=(0., 0., 0.), polygons=REMOTE_OBSTACLE) as (_, robot, nav):
        state = select(nav, (0., .090))
        assert state["planning_status"] == "ready", state["planning_message"]
        assert state["maneuver"]["kind"] == "drive"
        assert state["maneuver"]["target_px"] == pytest.approx(state["center_px"], abs=.01)
        nav.start()
        wait_for(lambda: not nav.state()["active"])
        state = nav.state()
        assert state["phase"] == "arrived", state["message"]
        assert state["waypoint_index"] == 1
        assert all(left == right == 0 for left, right in robot.drives)


def test_covered_goal_does_not_need_a_feasible_turn_during_preflight(monkeypatch):
    with synthetic_scene(start=(0., 0., 0.), polygons=REMOTE_OBSTACLE) as (_, robot, nav):
        def unnecessary_plan(*args, **kwargs):
            raise ValueError("No existe espacio para realizar el giro innecesario.")

        monkeypatch.setattr(navigation_module, "plan_pose_path", unnecessary_plan)
        selected = select(nav, (0., .090))
        assert selected["planning_status"] == "ready", selected["planning_message"]
        nav.start()
        wait_for(lambda: not nav.state()["active"])
        state = nav.state()
        assert state["phase"] == "arrived", state["message"]
        assert state["waypoint_index"] == 1
        assert all(left == right == 0 for left, right in robot.drives)


def test_reached_waypoint_stays_reached_while_braking_carries_the_body_past_it(monkeypatch):
    with synthetic_scene(start=(0., 0., 0.), polygons=[]) as (camera, robot, nav):
        nav.SPEED_FALL_MM_S2 = 1000.
        select(nav, (.300, 0.))
        original_drive = robot.drive_velocity_navigation
        previous = [0.]
        enable_coast, coast_happened = [False], [False]

        def drive(left, right):
            original_drive(left, right)
            magnitude = max(abs(left), abs(right))
            if enable_coast[0] and not coast_happened[0] and magnitude < previous[0]:
                coast_happened[0] = True
                center = pixel((.500, 0.))
                camera.pose(x=center[0], y=center[1], heading=0., scale=SCALE)
            previous[0] = magnitude

        monkeypatch.setattr(robot, "drive_velocity_navigation", drive)
        nav.start()
        wait_for(lambda: any(left >= 100 and right >= 100 for left, right in robot.drives))
        enable_coast[0] = True
        center = pixel((.225, 0.))  # El punto queda 75 mm delante, bajo la PCB.
        camera.pose(x=center[0], y=center[1], heading=0., scale=SCALE)
        wait_for(lambda: not nav.state()["active"], timeout=2.)
        state = nav.state()
        assert coast_happened[0]
        assert state["phase"] == "arrived", state["message"]
        assert state["waypoint_index"] == 1
        assert all(left >= 0 and right >= 0 for left, right in robot.drives)


def test_ordered_covered_points_replan_the_next_leg_from_the_actual_stopped_pose(monkeypatch):
    with synthetic_scene(start=(0., 0., 0.), polygons=REMOTE_OBSTACLE) as (camera, robot, nav):
        first = select(nav, (.080, 0.))
        nav.append_target(*pixel((.300, 0.)), FRAME_WIDTH, FRAME_HEIGHT,
                          CAMERA_ID, first["route_revision"])
        original_plan = navigation_module.plan_pose_path
        runtime_starts = []

        def plan(*args, **kwargs):
            if nav._active:
                runtime_starts.append(list(args[2]))
            return original_plan(*args, **kwargs)

        monkeypatch.setattr(navigation_module, "plan_pose_path", plan)
        nav.start()
        wait_for(lambda: nav.state()["waypoint_index"] == 1 and
                 any(left > 0 and right > 0 for left, right in robot.drives), timeout=3.)
        assert runtime_starts
        # No se llegó al centro del primer punto de 80 mm: el segundo tramo
        # debe partir del robot detenido en el origen, no de ese punto ideal.
        assert runtime_starts[0] == pytest.approx(pixel((0., 0.)), abs=.01)
        center = pixel((.225, 0.))
        camera.pose(x=center[0], y=center[1], heading=0., scale=SCALE)
        wait_for(lambda: not nav.state()["active"], timeout=3.)
        state = nav.state()
        assert state["phase"] == "arrived", state["message"]
        assert state["waypoint_index"] == state["waypoint_count"] == 2


def test_next_direct_leg_keeps_its_green_preview_from_the_actual_stopped_pose():
    with synthetic_scene(start=(0., 0., 0.), polygons=[]) as (_, robot, nav):
        selected = select(nav, (.080, 0.))
        goal = pixel((.300, 0.))
        nav.append_target(*goal, FRAME_WIDTH, FRAME_HEIGHT, CAMERA_ID, selected["route_revision"])
        nav.start()
        wait_for(lambda: nav.state()["waypoint_index"] == 1 and
                 any(left > 0 and right > 0 for left, right in robot.drives))
        state = nav.state()
        assert state["active"]
        assert len(state["planned_path_px"]) == 2
        assert state["planned_path_px"][0] == pytest.approx(pixel((0., 0.)), abs=.01)
        assert state["planned_path_px"][-1] == pytest.approx(goal, abs=.01)
