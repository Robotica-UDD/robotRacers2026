"""La compensación de luz conserva captura, geometría y permisos de movimiento."""
import numpy as np
import pytest
from fastapi.testclient import TestClient

from simulation.tests.test_webcam import FakeCapture, marker_frame, wait_until
from simulation.webcam.app import CaptureSession, create_app
from simulation.webcam.detector import AprilTagDetector
from simulation.webcam.illumination import IlluminationSettings, enhance_frame


HEADER = {"X-Requested-With": "XMLHttpRequest"}


def test_illumination_can_be_saved_without_opening_camera_or_arming_navigation():
    calls = []
    app = create_app(capture_factory=lambda index: calls.append(index))
    with TestClient(app) as client:
        initial = client.get('/api/illumination').json()
        assert initial['enabled'] is False and initial['hardware_controls'] is False
        result = client.post('/api/illumination', headers=HEADER, json={
            'enabled': True, 'automatic': True, 'expected_revision': initial['revision']})
        assert result.status_code == 200
        assert result.json()['revision'] == initial['revision'] + 1
        assert result.json()['enabled']
        state = client.get('/api/state').json()
        assert state['illumination']['enabled'] and not state['running']
        assert not app.state.navigation.state()['active']
        assert not app.state.robot.state()['moving']
        assert calls == []
        # Una petición retrasada no sobrescribe el ajuste recién aceptado.
        stale = client.post('/api/illumination', headers=HEADER,
                            json={'brightness': 10, 'expected_revision': initial['revision']})
        assert stale.status_code == 409
        assert client.get('/api/illumination').json()['brightness'] == 0


@pytest.mark.parametrize('data', [
    {}, {'unknown': 1}, {'enabled': 1}, {'automatic': None}, {'local_contrast': 'true'},
    {'brightness': True}, {'brightness': 61}, {'brightness': -61}, {'contrast': .49},
    {'contrast': 2.51}, {'gamma': .49}, {'gamma': 2.01}, {'gamma': '1.2'},
    {'gamma': None}, {'gamma': 10**400}, {'gamma': 1, 'expected_revision': True},
    {'gamma': 1, 'expected_revision': None}, {'gamma': 1, 'expected_revision': -1},
])
def test_invalid_illumination_never_opens_camera_or_changes_settings(data):
    calls = []
    app = create_app(capture_factory=lambda index: calls.append(index))
    with TestClient(app) as client:
        before = client.get('/api/illumination').json()
        assert client.post('/api/illumination', headers=HEADER, json=data).status_code == 422
        assert client.get('/api/illumination').json() == before
        assert not calls


def test_illumination_rejects_missing_local_header_nonfinite_json_and_oversize_body():
    app = create_app(capture_factory=lambda _: None)
    with TestClient(app) as client:
        assert client.post('/api/illumination', json={'enabled': True}).status_code == 403
        headers = {**HEADER, 'Content-Type': 'application/json'}
        for body in ('{"gamma":NaN}', '{"gamma":1e999}', '[1,2]', 'null', '{'):
            assert client.post('/api/illumination', headers=headers, content=body).status_code == 422
        assert client.post('/api/illumination', headers=headers,
                           content='{"unknown":"' + 'a'*2100 + '"}').status_code == 413


def test_live_adjustment_preserves_camera_identity_size_sequence_and_original_hand_pixels():
    frame = marker_frame()
    capture = FakeCapture(frame)
    frames = []

    class HandObserver:
        def state(self):
            return {'hands': [], 'sequence': None}

        def submit(self, pixels, sequence, stamp):
            frames.append((pixels.copy(), sequence, stamp))

    session = CaptureSession(capture_factory=lambda _: capture)
    session.hand_detector = HandObserver()
    try:
        session.start(0, 'usb-light-test')
        initial = wait_until(lambda: (s if (s := session.state())['visible'] else None))
        updated = session.configure_illumination({'enabled': True, 'automatic': False,
            'brightness': 20, 'local_contrast': False})
        active = wait_until(lambda: (s if (s := session.state())['illumination']['frame_revision']
                                   == updated['revision'] else None))
        assert active['running'] and active['visible']
        assert active['camera_id'] == initial['camera_id'] == 'usb-light-test'
        assert (active['frame_width'], active['frame_height']) == (480, 360)
        assert active['sequence'] > initial['sequence']
        assert active['center_px'] == pytest.approx(initial['center_px'], abs=1.)
        assert not capture.released
        wait_until(lambda: len(frames) > 2)
        assert all(np.array_equal(pixels, frame) for pixels, _, _ in frames)
        assert all(stamp > 0 for _, _, stamp in frames)
    finally:
        session.stop()


def test_saved_settings_survive_restart_and_unreadable_file_uses_original(tmp_path):
    path = tmp_path / 'illumination.json'
    first = CaptureSession(capture_factory=lambda _: None, illumination_path=path)
    first.configure_illumination({'enabled': True, 'gamma': 1.4, 'brightness': -12})
    reloaded = CaptureSession(capture_factory=lambda _: None, illumination_path=path)
    assert reloaded.illumination_state()['gamma'] == 1.4
    assert reloaded.illumination_state()['brightness'] == -12
    assert reloaded.illumination_state()['enabled']
    assert reloaded.illumination_state()['revision'] == first.illumination_state()['revision']
    assert not reloaded.state()['running']
    path.write_text('{broken', encoding='utf-8')
    original = CaptureSession(capture_factory=lambda _: None, illumination_path=path)
    assert original.illumination_state()['enabled'] is False


def test_failed_persistence_does_not_partially_apply_settings(tmp_path, monkeypatch):
    path = tmp_path / 'illumination.json'
    session = CaptureSession(capture_factory=lambda _: None, illumination_path=path)
    before = session.illumination_state()

    def cannot_write(*args, **kwargs):
        raise PermissionError('Read-only test directory')

    monkeypatch.setattr(type(path), 'write_text', cannot_write)
    with pytest.raises(OSError):
        session.configure_illumination({'enabled': True})
    assert session.illumination_state() == before


def test_lighting_detector_still_checks_identity_and_keeps_coordinates():
    detector = AprilTagDetector()
    settings = IlluminationSettings(enabled=True)
    original = marker_frame()
    adjusted, _ = enhance_frame(original, settings)
    result = detector.process_with_lighting(original, adjusted)
    assert result.visible and result.center_px == pytest.approx([239.5, 179.5], abs=1)
    for identifier in [1, 12, 100]:
        frame = marker_frame(identifier=identifier)
        adjusted, _ = enhance_frame(frame, settings)
        assert not detector.process_with_lighting(frame, adjusted).visible
    blank = np.full_like(original, 200)
    adjusted, _ = enhance_frame(blank, settings)
    assert not detector.process_with_lighting(blank, adjusted).visible
