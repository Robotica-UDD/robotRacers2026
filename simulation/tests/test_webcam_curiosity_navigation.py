"""Acercamiento fijo a una mano con ruedas ideales, sin acceso al robot real."""
import math
import time

import pytest

from simulation.tests.test_webcam_continuous_tracking import live_frames
from simulation.tests.test_webcam_point_navigation import wait_for
from simulation.tests.test_webcam_pose_navigation import (
    CAMERA_ID, FRAME_WIDTH, FRAME_HEIGHT, pixel, synthetic_scene, penetration_mm,
)
from simulation.webcam.home_behavior import hand_clearance_mm
from simulation.webcam.navigation import NavigationError


SOURCE = {"camera_id": CAMERA_ID, "frame_width": FRAME_WIDTH, "frame_height": FRAME_HEIGHT}
HAND = [1136., 440., 1200., 520.]  # Rectángulo fijo a 620–700 mm del centro inicial.
STAND_OFF_MM = 100.


def approach(nav, camera, box=None, authorize=None):
    return nav.approach_hand(list(HAND) if box is None else box,
                             camera.state()["marker_corners"], SOURCE,
                             STAND_OFF_MM, authorize=authorize)


def original_route(nav):
    first = nav.select_target(*pixel((-.40, .25)), **SOURCE)
    return nav.append_target(*pixel((-.40, -.25)), **SOURCE,
                             expected_route_revision=first["route_revision"])["waypoints_px"]


def final_clearance(camera, nav):
    return hand_clearance_mm(HAND, camera.state()["marker_corners"], nav._calibration)


def arc_subposes(row):
    """Cuatro posiciones de 5 ms sobre el arco real, no solo sus extremos."""
    x, y, yaw = row["previous_pose"]
    speed = (row["left"] + row["right"]) / 2000.
    angular = (row["right"] - row["left"]) / 190.
    for fraction in (.25, .5, .75, 1.):
        elapsed = .020 * fraction
        angle = angular * elapsed
        if abs(angular) > 1e-9:
            px = x + speed / angular * (math.sin(yaw + angle) - math.sin(yaw))
            py = y + speed / angular * (math.cos(yaw) - math.cos(yaw + angle))
        else:
            px, py = x + speed * elapsed * math.cos(yaw), y + speed * elapsed * math.sin(yaw)
        yield px, py, yaw + angle


def use_plant_clock(nav):
    """El control y las ruedas avanzan los mismos 20 ms por muestra.

    La planta se ejecuta acelerada, pero la geometría consume CPU variable.
    Usar su tiempo real multiplicado por 20 alteraría los perfiles de giro.
    """
    original = nav._continuous_command
    logical = [0.]

    def command(frame, calibration, path, follower, speeds, tick, **kwargs):
        logical[0] += .020
        stamp = nav._ramp_stamp
        rise, fall = nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2
        try:
            nav._ramp_stamp = logical[0] - .020
            nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 600., 900.
            if follower._last_now is None:
                follower._last_now = logical[0] - .020
            return original(frame, calibration, path, follower, speeds, logical[0], **kwargs)
        finally:
            nav._ramp_stamp = stamp
            nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = rise, fall

    nav._continuous_command = command


def test_curiosity_keeps_its_point_after_the_hand_disappears_and_stops_with_clearance():
    with synthetic_scene(start=(0., 0., 0.), polygons=[], plant=True) as (camera, robot, nav):
        use_plant_clock(nav)
        nav.set_speeds(240, 90)
        original = original_route(nav)
        nav.arm_home_watch()
        wait_for(lambda: bool(robot.drives))
        session = robot.state()["control_session"]
        box = list(HAND)
        result = approach(nav, camera, box)
        target = list(result["target_px"])
        assert result["purpose"] == "curiosity"
        # No hay nuevas observaciones de manos durante este recorrido. Cambiar
        # la caja de entrada tampoco puede mover el destino que ya fue fijado.
        box[:] = [900., 100., 1000., 200.]
        wait_for(lambda: not nav.state()["active"], timeout=8.)
        state = nav.state()
        assert state["phase"] == "arrived"
        assert "interés" in state["message"].lower()
        expected = ((target[0] - 640.) / 800., (480. - target[1]) / 800.)
        # La curiosidad tiene llegada por centro, no la cobertura amplia de
        # los destinos normales que podía terminar 80–116 mm antes del punto.
        assert math.hypot(robot.x - expected[0], robot.y - expected[1]) <= .040
        # Son distancias del plano ideal, no una medición de profundidad real.
        # El radio que permite llegar en cualquier orientación reserva algo más
        # que los 100 mm mínimos; la planta frontal termina cerca de 176 mm.
        assert STAND_OFF_MM - .5 <= final_clearance(camera, nav) <= 185.
        # La separación solicitada es 100 mm: no debe conservar el extremo
        # anterior calculado para 250 mm ni terminar apenas inicia el avance.
        assert robot.x > .30
        assert state["waypoints_px"] == original and state["waypoint_index"] == 0
        assert not state["active"] and not state["pending_mission"]
        assert robot.starts == 1 and robot.state()["control_session"] == session
        assert state["home_watch_armed"] and state["home_watch_idle"]
        count = len(robot.drives)
        time.sleep(.06)
        assert all(pair == (0, 0) for pair in robot.drives[count:])


def test_curiosity_plans_around_a_static_obstacle_without_entering_the_hand():
    barrier = [[(.100, -.060), (.140, -.060), (.140, .060), (.100, .060)]]
    with synthetic_scene(start=(0., 0., 0.), polygons=barrier, plant=True) as (camera, robot, nav):
        use_plant_clock(nav)
        nav.set_speeds(240, 90)
        nav.arm_home_watch()
        wait_for(lambda: bool(robot.drives))
        result = approach(nav, camera)
        wait_for(lambda: not nav.state()["active"], timeout=15.)
        assert nav.state()["phase"] == "arrived", nav.state()["message"]
        # Tras el rodeo, la llegada lateral queda aproximadamente a 125 mm.
        assert STAND_OFF_MM - .5 <= final_clearance(camera, nav) <= 185.
        assert max(row["penetration_mm"] for row in robot.trace) <= .002
        hand_polygon = [[(.620, -.050), (.700, -.050), (.700, .050), (.620, .050)]]
        assert max(penetration_mm(*pose, barrier) for row in robot.trace
                   for pose in arc_subposes(row)) <= .002
        assert max(penetration_mm(*pose, hand_polygon) for row in robot.trace
                   for pose in arc_subposes(row)) <= .002
        assert max(abs(row["position"][1]) for row in robot.trace) > .10
        assert all(row["left"] + row["right"] >= 0 for row in robot.trace if not row["escaping"])
        target = result["target_px"]
        assert not (HAND[0] <= target[0] <= HAND[2] and HAND[1] <= target[1] <= HAND[3])


def test_fear_preempts_curiosity_without_overwriting_the_original_route():
    with synthetic_scene(start=(0., 0., 0.), polygons=[]) as (camera, robot, nav), live_frames(camera):
        nav.set_speeds(240, 90)
        original = original_route(nav)
        nav.arm_home_watch()
        result = approach(nav, camera)
        wait_for(lambda: nav.state()["active"] and any(sum(pair) > 0 for pair in robot.drives))
        session, worker = robot.state()["control_session"], nav._thread
        assert nav.state()["behavior_purpose"] == "curiosity"
        home = pixel((-.30, 0.))
        nav.return_home(home, SOURCE)
        wait_for(lambda: nav.state()["behavior_purpose"] == "home" and not nav._home_redirect)
        assert nav._target == home and nav._thread is worker
        assert nav._home_saved_route[0] == original
        assert result["target_px"] != home
        camera.pose(x=home[0], y=home[1], scale=800.)
        wait_for(lambda: not nav.state()["active"], timeout=4.)
        state = nav.state()
        assert state["phase"] == "arrived"
        assert "casa" in state["message"].lower()
        assert state["waypoints_px"] == original and state["waypoint_index"] == 0
        assert robot.starts == 1 and robot.state()["control_session"] == session


@pytest.mark.parametrize("stop_kind", ["panel", "D48"])
def test_stopping_curiosity_discards_the_movement_and_never_rearms(stop_kind):
    with synthetic_scene(start=(0., 0., 0.), polygons=[]) as (camera, robot, nav), live_frames(camera):
        original = original_route(nav)
        nav.arm_home_watch()
        approach(nav, camera)
        wait_for(lambda: nav.state()["active"] and any(sum(pair) > 0 for pair in robot.drives))
        if stop_kind == "panel":
            nav.stop()
        else:
            robot.update(stop_reason="button_stop", navigation_active=False, resume_blocked=True)
        wait_for(lambda: not nav.state()["active"])
        state = nav.state()
        assert not state["home_watch_armed"]
        assert state["waypoints_px"] == original and state["waypoint_index"] == 0
        assert state["last_left"] == state["last_right"] == 0
        with pytest.raises(NavigationError):
            approach(nav, camera)
        count = robot.starts
        time.sleep(.06)
        assert robot.starts == count == 1


def test_revoked_curiosity_authorization_never_changes_the_mission():
    with synthetic_scene(start=(0., 0., 0.), polygons=[]) as (camera, robot, nav):
        original = original_route(nav)
        nav.arm_home_watch()
        with pytest.raises(NavigationError):
            approach(nav, camera, authorize=lambda: False)
        assert nav.state()["waypoints_px"] == original
        assert not nav.state()["active"]
        assert all(pair == (0, 0) for pair in robot.drives)
