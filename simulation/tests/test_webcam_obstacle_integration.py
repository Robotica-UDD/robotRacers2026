"""Recorridos con obstáculos y robot cinemático; sin cámaras ni transporte real."""
import math
import time

import cv2
import numpy as np
import pytest

import simulation.webcam.navigation as navigation_module
from simulation.webcam.navigation import Navigation, INT32_MAX
from simulation.webcam.navigation_geometry import _homographies, _project, observe_robot
from simulation.webcam.obstacle_planner import point_clearances_mm
from simulation.webcam.pose_obstacle_planner import check_pose
from simulation.webcam.robot_footprint import ROBOT_FOOTPRINT_MM, ROBOT_RADIUS_MM
from simulation.webcam.straight_control import StraightControl
from simulation.tests.test_webcam_point_navigation import (FakeCamera, FakeRobot, ClosedLoopRobot,
                                                          goal_under_body, wait_for, scale_path_clock)
from simulation.tests.test_webcam_obstacle_current_scene import (CORNERS, CENTER, FRONT, OBSTACLES,
                                                               GOAL, calibration)


class WideCamera(FakeCamera):
    """Conserva los ejes de la planta ideal y amplía el campo a600 píxeles/m."""
    def pose(self, x=640., y=480., heading=0., scale=2000.):
        return super().pose(x=472. + (x - 640.) * .3,
                            y=480. + (y - 480.) * .3, heading=heading, scale=600.)


POLYGON = [[628., 444.], [652., 444.], [652., 516.], [628., 516.]]


def prepare(tmp_path=None, closed_loop=False):
    camera = WideCamera()
    robot = ClosedLoopRobot(camera) if closed_loop else FakeRobot()
    robot.update(max_velocity_mm_s=INT32_MAX)
    nav = Navigation(camera, robot, tmp_path / 'navigation-state.json' if tmp_path else None)
    nav.CONTROL_INTERVAL = .002
    nav.SETTLE_SECONDS = .01
    nav.WAYPOINT_SETTLE_SECONDS = .005
    nav.SPEED_RISE_MM_S2 = 6000.
    nav.SPEED_FALL_MM_S2 = 9000.
    nav.TURN_CONFIRM_SECONDS = .025
    nav.calibrate([472.,480.], [532.,480.], 1280,960,'usb-camera')
    nav.set_obstacle_settings(30, nav.state()['route_revision'])
    nav.select_target(808.,480.,1280,960,'usb-camera')
    return camera, robot, nav


def install(nav, polygon=POLYGON):
    return nav.set_obstacles([polygon], 1280,960,'usb-camera',nav.state()['route_revision'])


def test_two_user_destinations_keep_their_order_while_both_legs_avoid_the_obstacle():
    camera, robot, nav = prepare(closed_loop=True)
    passed = []
    try:
        nav.append_target(472.,480.,1280,960,'usb-camera',nav.state()['route_revision'])
        install(nav)
        original_sync = nav._sync_target_locked
        def record_progress():
            original_sync()
            if nav._active:
                passed.append((nav._waypoint_index,robot.x,robot.y,robot.yaw))
        nav._sync_target_locked = record_progress
        nav.start(nav.state()['route_revision'])
        wait_for(lambda:not nav.state()['active'], timeout=15.)
        state=nav.state()
        assert state['phase']=='arrived', (state['phase'],state['planning_status'],state['planning_message'])
        assert state['waypoint_index']==state['waypoint_count']==2
        assert [index for index,_,_,_ in passed] == [1,2]
        assert goal_under_body(*passed[0][1:], (.56, 0.))
        assert goal_under_body(robot.x, robot.y, robot.yaw, (0., 0.))
        assert robot.starts==1 and robot.drives[-1]==(0,0)
        corners=[[460.,468.],[484.,468.],[484.,492.],[460.,492.]]
        clearances=[point_clearances_mm(corners,nav._calibration,[472+x*600,480-y*600],[POLYGON])
                    for x,y,_,_,_ in robot.trace]
        assert min(value['obstacle_mm'] for value in clearances)>ROBOT_RADIUS_MM + 30
        assert min(value['camera_mm'] for value in clearances)>nav._camera_view_margin(nav._calibration,30)
        assert max(abs(y) for _,y,_,_,_ in robot.trace)>(ROBOT_RADIUS_MM + 30) / 1000.
        assert all(left + right >= 0 for _, _, _, left, right in robot.trace)
    finally:
        nav.close()


@pytest.mark.parametrize("speeds", [(100, 32), (200, 400)])
def test_recorded_scene_checks_oriented_footprints_in_plan_runtime_and_arrival(monkeypatch, speeds):
    """La cámara fija grabada alimenta una planta ideal; ningún IO real.

    El inicio queda a unos145 mm del obstáculo. Se comprueba la huella orientada
    durante todo el recorrido, sin imponer un círculo como segundo filtro.
    """
    scale_path_clock(monkeypatch)
    cal = calibration()
    _, projection, _ = _homographies(CORNERS, .04, 1280, 720)
    camera = FakeCamera()
    camera.update(frame_height=720, camera_id="recorded-camera", running=True,
                  opening=False, marker_corners=CORNERS)
    plans, sweeps, planned_steps = [], [], []
    original_plan, original_sweep = navigation_module.plan_pose_path, navigation_module.check_wheel_motion

    def recorded_plan(*args, **kwargs):
        plans.append(1)
        result = original_plan(*args, **kwargs)
        planned_steps.extend(result["steps"])
        return result

    def recorded_sweep(*args, **kwargs):
        result = original_sweep(*args, **kwargs)
        sweeps.append(result)
        return result

    monkeypatch.setattr(navigation_module, "plan_pose_path", recorded_plan)
    monkeypatch.setattr(navigation_module, "check_wheel_motion", recorded_sweep)
    # El paso de la planta es20 ms; el ensayo acelera veinte veces el reloj.
    monkeypatch.setattr(StraightControl, "BEARING_FILTER_SECONDS", .12 / 20)
    monkeypatch.setattr(StraightControl, "DERIVATIVE_FILTER_SECONDS", .15 / 20)
    monkeypatch.setattr(StraightControl, "KD", .12 / 20)
    monkeypatch.setattr(StraightControl, "MAX_SLEW_RAD_S2", math.radians(25) * 20)

    class RecordedSceneRobot(FakeRobot):
        def __init__(self):
            super().__init__()
            self.center = np.array(cal.center_tag_m)
            self.heading = cal.front_angle_rad
            self.trace = []
            self.motion_steps = []

        def drive_velocity_navigation(self, left, right):
            super().drive_velocity_navigation(left, right)
            step_index = nav._pose_step_index
            step = nav._pose_steps[step_index] if step_index < len(nav._pose_steps) else {}
            self.motion_steps.append((left, right, dict(step)))
            dt, speed, angular = .02, (left + right) / 2000., (right - left) / 190.
            delta = angular * dt
            if abs(angular) > 1e-9:
                self.center += speed / angular * np.array([
                    math.sin(self.heading + delta) - math.sin(self.heading),
                    math.cos(self.heading) - math.cos(self.heading + delta)])
            else:
                self.center += speed * dt * np.array([math.cos(self.heading), math.sin(self.heading)])
            self.heading += delta
            angle = self.heading - cal.front_angle_rad
            rotation = np.array([[math.cos(angle), -math.sin(angle)],
                                 [math.sin(angle), math.cos(angle)]])
            origin = self.center - rotation @ np.array(cal.center_tag_m)
            corners = [_project(projection, origin + rotation @ np.array(point), np.zeros(2)).tolist()
                       for point in [(-.02, .02), (.02, .02), (.02, -.02), (-.02, -.02)]]
            center_px = _project(projection, self.center, np.zeros(2)).tolist()
            heading_image = math.radians(observe_robot(corners, cal)["heading_deg"])
            self.trace.append((center_px, check_pose(CORNERS, cal, center_px, heading_image, OBSTACLES)))
            camera.update(marker_corners=corners)

    robot = RecordedSceneRobot()
    robot.update(max_velocity_mm_s=INT32_MAX)
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .001
    nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 12000., 18000.
    nav.WAYPOINT_SETTLE_SECONDS, nav.SETTLE_SECONDS = .005, .01
    nav.TURN_CONFIRM_SECONDS = .25 / 20
    try:
        nav.calibrate(CENTER, FRONT, 1280, 720, "recorded-camera")
        nav.set_speeds(*speeds)
        nav.select_target(*GOAL, 1280, 720, "recorded-camera")
        state = nav.set_obstacles(OBSTACLES, 1280, 720, "recorded-camera", nav.state()["route_revision"])
        assert state["planning_status"] == "ready", state["planning_message"]
        assert state["obstacle_settings"]["robot_radius_mm"] == ROBOT_RADIUS_MM
        assert state["clearance_info"]["obstacle_required_mm"] == 0
        assert state["clearance_info"]["view_required_mm"] == 0
        assert nav._pose_steps
        nav.start(state["route_revision"])
        wait_for(lambda: not nav.state()["active"], timeout=10.)
        state = nav.state()
        assert state["phase"] == "arrived", (state["phase"], state["planning_message"], state["message"])
        assert state["waypoint_index"] == state["waypoint_count"] == 1
        assert robot.starts == 1 and robot.drives[-1] == (0, 0)
        assert plans and sweeps and any(result["clear"] for result in sweeps)
        assert all(step["direction"] == 1 for step in planned_steps if step["kind"] == "drive")
        assert all(left + right >= 0 or step.get("kind") == "escape"
                   for left, right, step in robot.motion_steps)
        normal_drive_seen = False
        for left, right, step in robot.motion_steps:
            if step.get("kind") == "drive" and left + right > 0:
                normal_drive_seen = True
            if left + right < 0:
                assert not normal_drive_seen, step
        assert all(values["clear"] for _, values in robot.trace)
        final = observe_robot(camera.state()["marker_corners"], cal, GOAL)
        assert cv2.pointPolygonTest(np.asarray(ROBOT_FOOTPRINT_MM, dtype=np.float32),
                                   tuple(value * 1000. for value in final["target_local_m"]), False) >= 0.
    finally:
        nav.close()
