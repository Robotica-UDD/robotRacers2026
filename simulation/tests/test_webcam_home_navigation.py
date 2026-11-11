"""Casa reutiliza una sesión armada y conserva la prioridad de las paradas.

Los robots y las imágenes son dobles locales. No se abre USB, cámara o conexión
al robot físico, y las pruebas no llaman a ningún servicio externo.
"""
from contextlib import contextmanager
import threading
import time

import pytest

from simulation.tests.test_webcam_point_navigation import (
    FakeCamera, FakeRobot, ClosedLoopRobot, calibrate, scale_path_clock,
    goal_under_body, wait_for,
)
from simulation.tests.test_webcam_continuous_tracking import live_frames
from simulation.webcam.navigation import Navigation, NavigationError


METADATA = {"camera_id": "usb-camera", "frame_width": 1280, "frame_height": 960}


@contextmanager
def rig(monkeypatch, *, plant=False):
    camera = FakeCamera()
    camera.update(running=True, opening=False)
    robot = ClosedLoopRobot(camera) if plant else FakeRobot()
    robot.update(max_velocity_mm_s=1000)
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .001
    nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 12000., 18000.
    nav.SETTLE_SECONDS = .01
    nav.RESUME_STABLE_SECONDS, nav.RESUME_RETRY_SECONDS = .015, .015
    if plant:
        scale_path_clock(monkeypatch)
    calibrate(nav)
    nav.set_speeds(240, 90)
    try:
        yield camera, robot, nav
    finally:
        nav.close()


def route(nav):
    first = nav.select_target(1180., 480., 1280, 960, 'usb-camera')
    second = nav.append_target(1050., 700., 1280, 960, 'usb-camera', first['route_revision'])
    return second['waypoints_px']


def test_explicit_watch_arming_sends_only_zero_velocity_heartbeats(monkeypatch):
    with rig(monkeypatch) as (_, robot, nav):
        nav.arm_home_watch()
        wait_for(lambda: len(robot.drives) >= 2)
        assert robot.starts == 1 and robot.stops == 0
        assert all(pair == (0, 0) for pair in robot.drives)
        state = nav.state()
        assert state['home_watch_armed'] and state['home_watch_idle']
        assert not state['active'] and not state['pending_mission']
        nav.disarm_home_watch()
        assert robot.stops >= 1 and not nav.state()['home_watch_armed']


@pytest.mark.parametrize('reason', ['button_stop', 'driver_fault', 'encoder_stall'])
def test_known_stop_reason_ends_watch_even_if_cached_session_still_matches(monkeypatch, reason):
    with rig(monkeypatch) as (_, robot, nav):
        nav.arm_home_watch()
        wait_for(lambda: bool(robot.drives))
        # La razón de parada debe prevalecer incluso antes de que todos los
        # campos de una caché de estado reflejen que terminó la sesión.
        robot.update(stop_reason=reason, navigation_active=True, resume_blocked=False)
        wait_for(lambda: not nav.state()['home_watch_armed'], timeout=.4)
        with pytest.raises(NavigationError):
            nav.return_home([900., 480.], METADATA)
        assert robot.starts == 1
        assert all(pair == (0, 0) for pair in robot.drives)


def test_d48_invalidates_idle_watch_without_any_automatic_rearm(monkeypatch):
    with rig(monkeypatch) as (_, robot, nav):
        nav.arm_home_watch()
        wait_for(lambda: bool(robot.drives))
        robot.update(stop_reason='button_stop', navigation_active=False, resume_blocked=True)
        wait_for(lambda: not nav.state()['home_watch_armed'])
        with pytest.raises(NavigationError):
            nav.return_home([900., 480.], METADATA)
        time.sleep(.06)
        assert robot.starts == 1 and not nav.state()['active']
        assert all(pair == (0, 0) for pair in robot.drives)


def test_idle_home_return_adopts_watch_session_and_restores_pending_route(monkeypatch):
    with rig(monkeypatch, plant=True) as (_, robot, nav):
        original = route(nav)
        with nav._lock:
            nav._waypoint_index = 1
            nav._sync_target_locked()
        nav.arm_home_watch()
        wait_for(lambda: bool(robot.drives))
        session = robot.state()['control_session']
        home = [1000., 480.]
        nav.return_home(home, METADATA)
        wait_for(lambda: not nav.state()['active'], timeout=6.)
        state = nav.state()
        assert state['phase'] == 'arrived'
        assert state['waypoints_px'] == original and state['waypoint_index'] == 1
        assert state['target_px'] == original[1] and not state['pending_mission']
        assert goal_under_body(robot.x, robot.y, robot.yaw, (.18, 0.))
        assert robot.starts == 1 and robot.state()['control_session'] == session
        assert state['home_watch_armed'] and state['home_watch_idle']
        count = len(robot.drives)
        time.sleep(.07)
        assert all(pair == (0, 0) for pair in robot.drives[count:])


def test_active_return_redirects_the_existing_worker_and_session(monkeypatch):
    with rig(monkeypatch, plant=True) as (_, robot, nav):
        original = route(nav)
        nav.arm_home_watch()
        nav.start()
        wait_for(lambda: any(sum(pair) > 0 for pair in robot.drives))
        session, worker = robot.state()['control_session'], nav._thread
        home = [800., 680.]
        # El recorrido virtual puede cubrir un punto mientras el panel espera
        # el primer mando. Guardar el índice exactamente al solicitar el desvío.
        with nav._lock:
            interrupted_index = nav._waypoint_index
            result = nav.return_home(home, METADATA)
        assert result['returning_home'] and nav._target == home
        assert nav._thread is worker
        wait_for(lambda: not nav.state()['active'], timeout=8.)
        state = nav.state()
        assert state['phase'] == 'arrived'
        assert goal_under_body(robot.x, robot.y, robot.yaw, (.08, -.10))
        assert state['waypoints_px'] == original and state['waypoint_index'] == interrupted_index
        assert robot.starts == 1 and robot.state()['control_session'] == session
        assert not state['active'] and not state['pending_mission']


def test_arming_watch_during_a_mission_never_starts_another_session(monkeypatch):
    with rig(monkeypatch) as (camera, robot, nav), live_frames(camera):
        route(nav)
        nav.start()
        wait_for(lambda: any(sum(pair) > 0 for pair in robot.drives))
        count = len(robot.drives)
        nav.arm_home_watch()
        time.sleep(.07)
        assert robot.starts == 1 and nav.state()['home_watch_armed']
        assert robot.drives[count:] and all(sum(pair) > 0 for pair in robot.drives[count:])


@pytest.mark.parametrize('stop_kind', ['panel', 'D48'])
def test_manual_stop_during_active_home_return_restores_route_and_never_rearms(monkeypatch, stop_kind):
    with rig(monkeypatch) as (camera, robot, nav), live_frames(camera):
        original_route = route(nav)
        nav.start()
        wait_for(lambda: any(sum(pair) > 0 for pair in robot.drives))
        nav.arm_home_watch()
        nav.return_home([900., 700.], METADATA)
        if stop_kind == 'panel':
            nav.stop()
        else:
            robot.update(stop_reason='button_stop', navigation_active=False, resume_blocked=True)
        wait_for(lambda: not nav.state()['active'])
        state = nav.state()
        assert not state['home_watch_armed'] and not state['pending_mission']
        assert state['waypoints_px'] == original_route and state['waypoint_index'] == 0
        assert state['last_left'] == state['last_right'] == 0
        starts = robot.starts
        with pytest.raises(NavigationError):
            nav.return_home([900., 700.], METADATA)
        time.sleep(.06)
        assert robot.starts == starts == 1
        if stop_kind == 'D48':
            assert 'D48' in state['message'] or 'parada manual' in state['message']


def test_revocation_during_watch_http_arming_stops_the_late_session(monkeypatch):
    with rig(monkeypatch) as (_, robot, nav):
        authorized = threading.Event()
        authorized.set()
        original = robot.start_navigation

        def late_ack(**kwargs):
            result = original(**kwargs)
            authorized.clear()
            return result

        monkeypatch.setattr(robot, 'start_navigation', late_ack)
        with pytest.raises(NavigationError):
            nav.arm_home_watch(authorize=authorized.is_set)
        assert robot.starts == 1 and robot.stops >= 1
        assert not robot.state()['navigation_active']
        assert not nav.state()['home_watch_armed'] and not nav.state()['active']
        assert all(pair == (0, 0) for pair in robot.drives)


def test_revocation_during_home_planning_cannot_launch_and_restores_original_route(monkeypatch):
    with rig(monkeypatch) as (_, robot, nav):
        original_route = route(nav)
        nav.arm_home_watch()
        authorized, entered, release = threading.Event(), threading.Event(), threading.Event()
        authorized.set()
        calculate = nav._calculate_plan
        errors = []

        def delayed(*args, **kwargs):
            entered.set()
            assert release.wait(2.)
            return calculate(*args, **kwargs)

        def request():
            try:
                nav.return_home([900., 700.], METADATA, authorize=authorized.is_set)
            except NavigationError as exc:
                errors.append(str(exc))

        monkeypatch.setattr(nav, '_calculate_plan', delayed)
        worker = threading.Thread(target=request)
        worker.start()
        try:
            assert entered.wait(1.)
            authorized.clear()
            nav.disarm_home_watch()
        finally:
            release.set()
            worker.join(2.)
        assert not worker.is_alive() and errors
        state = nav.state()
        assert not state['active'] and not state['home_watch_armed']
        assert state['waypoints_px'] == original_route and state['waypoint_index'] == 0
        assert state['target_px'] == original_route[0]
        assert robot.starts == 1 and all(pair == (0, 0) for pair in robot.drives)
