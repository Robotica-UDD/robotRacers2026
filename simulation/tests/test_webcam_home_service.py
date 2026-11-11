"""API y orquestación de Casa con navegación real y dispositivos falsos."""
from contextlib import contextmanager
import threading
import time

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot, calibrate, wait_for
from simulation.tests.test_webcam_continuous_tracking import live_frames
from simulation.webcam.app import CaptureSession
from simulation.webcam.home_service import HomeService, make_home_router
from simulation.webcam.navigation import Navigation


HEADER = {'X-Requested-With': 'XMLHttpRequest'}
SOURCE = {'frame_width': 1280, 'frame_height': 960, 'camera_id': 'usb-camera'}
POINT = {'x': 1000., 'y': 480., **SOURCE}


class FakeHands:
    def __init__(self, **changes):
        self.value = {'status': 'ready', 'ready': True, 'model': 'YOLO de prueba',
                      'error': None, 'hands': [], 'sequence': None, 'captured_at': None,
                      'frame_width': 1280, 'frame_height': 960}
        self.value.update(changes)
        self.closed = False

    def state(self):
        return dict(self.value)

    def close(self):
        self.closed = True

    def submit(self, *args):
        pass


@contextmanager
def service_rig():
    camera, robot = FakeCamera(), FakeRobot()
    camera.update(running=True, opening=False)
    robot.update(max_velocity_mm_s=1000)
    nav = Navigation(camera, robot)
    nav.CONTROL_INTERVAL = .001
    nav.SPEED_RISE_MM_S2, nav.SPEED_FALL_MM_S2 = 12000., 18000.
    nav.RESUME_STABLE_SECONDS, nav.RESUME_RETRY_SECONDS = .015, .015
    calibrate(nav)
    nav.set_speeds(240, 90)
    detector = FakeHands()
    service = HomeService(camera, nav, robot, detector=detector, load_detector=False)
    app = FastAPI()
    app.include_router(make_home_router(service))
    try:
        with TestClient(app) as client, live_frames(camera):
            yield camera, robot, nav, service, client
    finally:
        service.close()
        nav.close()


def post(client, path, payload):
    return client.post('/api/home/' + path, headers=HEADER, json=payload)


def observed_hand(service, camera, sequence, stamp):
    return service.behavior.observe({'ready': True, 'status': 'ready', 'model': 'YOLO de prueba',
        'hands': [{'bbox_px': [850., 440., 930., 520.], 'confidence': .9}],
        'sequence': sequence, 'captured_at': stamp, **SOURCE}, camera.state())


def test_house_point_only_saves_and_explicit_enable_arms_zero_velocity_watch():
    with service_rig() as (_, robot, nav, service, client):
        saved = post(client, 'point', POINT)
        assert saved.status_code == 200
        assert saved.json()['home_px'] == [1000., 480.]
        assert not saved.json()['enabled']
        assert robot.starts == robot.stops == 0 and robot.drives == []
        assert nav.state()['waypoint_count'] == 0
        response = post(client, 'settings', {'enabled': True})
        assert response.status_code == 200 and response.json()['enabled']
        wait_for(lambda: len(robot.drives) >= 2)
        assert robot.starts == 1 and all(pair == (0, 0) for pair in robot.drives)
        assert not nav.state()['active'] and nav.state()['home_watch_idle']
        response = post(client, 'clear', {})
        assert response.status_code == 200 and response.json()['home_px'] is None
        assert not response.json()['enabled'] and not nav.state()['home_watch_armed']
        assert service.detector.closed is False


@pytest.mark.parametrize('path,payload,status', [
    ('point', {'x': 100., 'y': 200.}, 422),
    ('point', POINT | {'unexpected': 1}, 422),
    ('point', POINT | {'x': True}, 422),
    ('point', POINT | {'frame_width': True}, 422),
    ('point', POINT | {'x': -1}, 422),
    ('point', POINT | {'camera_id': 'other-camera'}, 409),
    ('settings', {}, 422),
    ('settings', {'enabled': 1}, 422),
    ('settings', {'enabled': 'true'}, 422),
    ('settings', {'enabled': None}, 422),
    ('settings', {'distance_mm': True}, 422),
    ('settings', {'distance_mm': 0}, 422),
    ('settings', {'distance_mm': '150'}, 422),
    ('settings', {'distance_mm': 1001}, 422),
    ('settings', {'distance_mm': 100, 'extra': 1}, 422),
    ('clear', {'enabled': False}, 422),
])
def test_invalid_home_api_bodies_never_arm_or_send_motion(path, payload, status):
    with service_rig() as (_, robot, _, _, client):
        response = post(client, path, payload)
        assert response.status_code == status, response.text
        assert robot.starts == 0 and robot.drives == []


def test_home_api_rejects_missing_header_nonfinite_and_oversized_json():
    with service_rig() as (_, robot, _, _, client):
        assert client.post('/api/home/point', json=POINT).status_code == 403
        assert client.post('/api/home/settings', headers=HEADER,
                           content='{"distance_mm": NaN}').status_code == 422
        assert client.post('/api/home/settings', headers=HEADER,
                           content='{"enabled": true, "extra": "' + 'x' * 4096 + '"}').status_code == 413
        assert client.post('/api/home/settings', headers=HEADER, json=[]).status_code == 422
        assert robot.starts == 0 and robot.drives == []


def test_house_and_ready_detector_are_required_before_enabling():
    with service_rig() as (_, robot, _, service, client):
        assert post(client, 'settings', {'enabled': True}).status_code == 409
        assert post(client, 'point', POINT).status_code == 200
        service.detector.value.update(ready=False, status='missing_model')
        response = post(client, 'settings', {'enabled': True})
        assert response.status_code == 409
        assert not service.state()['enabled'] and robot.starts == 0


def test_confirmed_hand_redirects_the_existing_real_navigation_session():
    with service_rig() as (camera, robot, nav, service, client):
        assert post(client, 'point', POINT).status_code == 200
        original = nav.select_target(1150., 700., **SOURCE)['waypoints_px']
        assert post(client, 'settings', {'enabled': True}).status_code == 200
        wait_for(lambda: bool(robot.drives))
        session = robot.state()['control_session']
        nav.start()
        wait_for(lambda: any(sum(pair) > 0 for pair in robot.drives))
        worker = nav._thread
        stamp = [time.monotonic()]
        service.behavior.clock = lambda: stamp[0]
        with service._lock:
            first = observed_hand(service, camera, 1, stamp[0])
        assert first['phase'] == 'near_hand' and robot.starts == 1
        stamp[0] += .2
        with service._lock:
            second = observed_hand(service, camera, 2, stamp[0])
        assert second['returning'] and second['trigger_count'] == 1
        wait_for(lambda: nav._target == [1000., 480.] and not nav._home_redirect)
        assert nav._thread is worker and robot.starts == 1
        assert robot.state()['control_session'] == session
        assert nav._home_saved_route[0] == original
        assert nav.get_speeds() == {'forward_mm_s': 240, 'turn_deg_s': 90}
        assert post(client, 'settings', {'enabled': False}).status_code == 200
        wait_for(lambda: not nav.state()['active'])


@pytest.mark.parametrize('action', ['disable', 'clear'])
def test_cancel_or_clear_does_not_wait_for_the_observers_route_calculation(action):
    with service_rig() as (camera, robot, nav, service, client):
        post(client, 'point', POINT)
        assert post(client, 'settings', {'enabled': True}).status_code == 200
        wait_for(lambda: bool(robot.drives))
        entered, release = threading.Event(), threading.Event()
        original = nav._refresh_plan

        def delayed(*args, **kwargs):
            if not entered.is_set():
                # Reproducir un A* que posee su candado. Los sondeos de estado
                # de stop() deben poder omitir este cálculo sin bloquearse.
                with nav._planning_lock:
                    entered.set()
                    assert release.wait(2.)
            return original(*args, **kwargs)

        nav._refresh_plan = delayed
        stamp = [time.monotonic()]
        service.behavior.clock = lambda: stamp[0]
        observed_hand(service, camera, 1, stamp[0])
        stamp[0] += .2
        errors = []

        def observer():
            try:
                with service._lock:
                    observed_hand(service, camera, 2, stamp[0])
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=observer)
        worker.start()
        try:
            assert entered.wait(1.)
            began = time.monotonic()
            response = (post(client, 'settings', {'enabled': False}) if action == 'disable'
                        else post(client, 'clear', {}))
            assert response.status_code == 200, response.text
            assert time.monotonic() - began < .5
            assert not response.json()['enabled']
            if action == 'clear':
                assert response.json()['home_px'] is None
        finally:
            release.set()
            worker.join(2.)
        assert not worker.is_alive() and not errors
        assert robot.starts == 1 and all(pair == (0, 0) for pair in robot.drives)
        assert not nav.state()['active'] and not nav.state()['home_watch_armed']


def test_hand_observation_matches_the_original_marker_frame_and_timestamp():
    clock = [10.2]
    camera = CaptureSession(capture_factory=lambda *_: None, clock=lambda: clock[0])
    detector = FakeHands(sequence=7, captured_at=10.)
    camera.hand_detector = detector
    corners = [[100., 100.], [140., 100.], [140., 140.], [100., 140.]]
    with camera._lock:
        camera._running = True
        camera._camera_id = 'usb-camera'
        camera._hand_frames[7] = {'visible': True, 'marker_corners': corners, 'captured_at': 10.,
            'sequence': 7, 'opening': False, **SOURCE}
        camera._hand_frames[8] = {'visible': True, 'marker_corners': [[500., 500.]] * 4,
            'captured_at': 10.1, 'sequence': 8, 'opening': False, **SOURCE}
    _, observed = camera.hand_observation()
    assert observed['marker_corners'] == corners
    assert observed['sequence'] == 7 and observed['frame_age_ms'] == pytest.approx(200.)
    detector.value['captured_at'] = 10.1
    assert not camera.hand_observation()[1]['visible']
    detector.value['captured_at'] = 10.
    camera._camera_id = 'different-camera'
    assert not camera.hand_observation()[1]['visible']


def test_camera_restart_never_reuses_a_hand_result_from_the_previous_capture():
    class PausedCapture:
        interruptible = True

        def __init__(self):
            self.entered, self.release_event = threading.Event(), threading.Event()

        def isOpened(self):
            return True

        def set(self, *_):
            return True

        def read(self):
            self.entered.set()
            self.release_event.wait(2.)
            return False, None

        def release(self):
            self.release_event.set()

    capture = PausedCapture()
    camera = CaptureSession(capture_factory=lambda *_: capture)
    camera.hand_detector = FakeHands(sequence=7, captured_at=10.)
    with camera._lock:
        camera._camera_id = 'usb-camera'
        camera._hand_sequence = 7
        camera._hand_frames[7] = {'visible': True, 'captured_at': 10., 'marker_corners': [], **SOURCE}
    try:
        camera.start(0, 'usb-camera')
        assert capture.entered.wait(1.)
        _, frame = camera.hand_observation()
        assert not frame['visible'] and 'captured_at' not in frame
        assert camera._hand_sequence == 7  # No reciclar identificadores al reabrir.
    finally:
        camera.stop()
