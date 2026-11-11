"""Detección y trabajador de manos: sin webcam, red ni órdenes al robot."""
import io
import json
import threading
import time

import numpy as np
import pytest

from simulation.tools import download_hand_model
from simulation.webcam.hand_detector import (
    AsyncHandDetector, HandDetector, CONFIG_NAME, WEIGHTS_NAME,
)


def wait_until(predicate, timeout=2.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.002)
    assert predicate(), "El trabajador de inferencia no publicó su estado."


class FakeNet:
    def __init__(self, rows):
        self.rows = np.asarray(rows, dtype=np.float32)
        self.backend = self.target = self.blob = None

    def setPreferableBackend(self, backend): self.backend = backend
    def setPreferableTarget(self, target): self.target = target
    def getUnconnectedOutLayersNames(self): return ('hands',)
    def setInput(self, blob): self.blob = blob
    def forward(self, names):
        assert names == ('hands',)
        return [self.rows]


def detector_with_rows(tmp_path, rows):
    (tmp_path / CONFIG_NAME).write_text('fake')
    (tmp_path / WEIGHTS_NAME).write_bytes(b'fake')
    net = FakeNet(rows)
    detector = HandDetector(tmp_path, net_factory=lambda *_: net)
    return detector, net


def test_missing_model_has_an_explicit_offline_installation_message(tmp_path):
    with pytest.raises(FileNotFoundError, match='download_hand_model'):
        HandDetector(tmp_path)


@pytest.mark.parametrize('options', [dict(confidence=False), dict(confidence=1),
                                    dict(nms_threshold=0), dict(input_size=300)])
def test_invalid_network_settings_are_rejected(tmp_path, options):
    with pytest.raises(ValueError):
        HandDetector(tmp_path, **options)


def test_opencv_class_confidence_boxes_clipping_and_suppression(tmp_path):
    detector, net = detector_with_rows(tmp_path, [
        [.5, .5, .4, .4, .6, .9],
        [.51, .5, .4, .4, .7, .8],  # misma mano, se suprime por solapamiento
        [0., .1, .2, .4, .5, .75],  # recortar el borde de imagen
        [.2, .2, .1, .1, .9, .4],   # confianza insuficiente
        [.2, .2, -.1, .1, .9, .9],  # caja inválida
        [float('nan'), .2, .1, .1, .9, .9],
    ])
    hands = detector.process(np.zeros((100, 200, 3), np.uint8))
    assert len(hands) == 2
    assert hands[0]['bbox_px'] == pytest.approx([60., 30., 140., 70.], abs=1e-4)
    assert hands[0]['confidence'] == pytest.approx(.9)  # no multiplicar otra vez por objectness
    assert hands[1]['bbox_px'] == pytest.approx([0., 0., 20., 30.], abs=1e-4)
    assert net.blob.shape == (1, 3, 416, 416)
    # La salida pública es serializable y no conserva escalares de numpy.
    json.dumps(hands, allow_nan=False)


def test_empty_network_output_returns_no_previous_boxes(tmp_path):
    detector, _ = detector_with_rows(tmp_path, [[.5, .5, .1, .1, .9, .2]])
    assert detector.process(np.zeros((100, 200, 3), np.uint8)) == []


@pytest.mark.parametrize('image', [None, np.zeros((10, 10)), np.zeros((10, 10, 4), np.uint8)])
def test_invalid_frames_are_not_submitted_to_the_network(tmp_path, image):
    detector, _ = detector_with_rows(tmp_path, [])
    with pytest.raises(ValueError):
        detector.process(image)


class RecordingDetector:
    def __init__(self):
        self.entered, self.release = threading.Event(), threading.Event()
        self.calls = []

    def process(self, frame):
        self.calls.append(int(frame[0, 0, 0]))
        if len(self.calls) == 1:
            self.entered.set()
            assert self.release.wait(3.)
        return [{'bbox_px': [1., 2., 3., 4.], 'confidence': .9}]


def test_async_inference_keeps_only_the_latest_pending_frame_and_original_timestamp():
    detector = RecordingDetector()
    async_detector = AsyncHandDetector(lambda: detector, max_fps=60.)
    try:
        wait_until(lambda: async_detector.state()['ready'])
        captured = time.monotonic() - .25
        image = np.full((16, 24, 3), 1, np.uint8)
        assert async_detector.submit(image, 1, captured)
        assert detector.entered.wait(1.)
        image[:] = 99  # la red no comparte el buffer mutable de la cámara
        async_detector.submit(np.full_like(image, 2), 2, captured)
        async_detector.submit(np.full_like(image, 3), 3, captured)
        assert not async_detector.submit(image, 2, captured)
        detector.release.set()
        wait_until(lambda: async_detector.state()['sequence'] == 3)
        state = async_detector.state()
        assert detector.calls == [1, 3]
        assert state['captured_at'] == captured and state['age_ms'] >= 250
        assert state['frame_width'] == 24 and state['frame_height'] == 16
        state['hands'][0]['bbox_px'][0] = 1000
        assert async_detector.state()['hands'][0]['bbox_px'][0] == 1
    finally:
        detector.release.set()
        async_detector.close()


def test_missing_model_does_not_block_frame_capture_or_attempt_downloads():
    def missing():
        raise FileNotFoundError('modelo no instalado')
    detector = AsyncHandDetector(missing)
    try:
        wait_until(lambda: detector.state()['status'] == 'missing_model')
        state = detector.state()
        assert not state['ready'] and not state['hands']
        assert not detector.submit(np.zeros((10, 10, 3), np.uint8), 1, time.monotonic())
    finally:
        detector.close()


def test_inference_failure_clears_the_previous_detection():
    class Detector:
        def __init__(self): self.calls = 0
        def process(self, frame):
            self.calls += 1
            if self.calls > 1:
                raise RuntimeError('fallo simulado')
            return [{'bbox_px': [1, 1, 3, 3], 'confidence': .9}]
    detector = AsyncHandDetector(Detector, max_fps=60.)
    image = np.zeros((8, 8, 3), np.uint8)
    try:
        detector.submit(image, 1, time.monotonic())
        wait_until(lambda: detector.state()['sequence'] == 1)
        assert detector.state()['hands']
        detector.submit(image, 2, time.monotonic())
        wait_until(lambda: detector.state()['status'] == 'error')
        assert detector.state()['hands'] == [] and not detector.state()['ready']
    finally:
        detector.close()


def test_closing_during_inference_cannot_publish_a_late_detection():
    detector = RecordingDetector()
    worker = AsyncHandDetector(lambda: detector)
    try:
        worker.submit(np.zeros((8, 8, 3), np.uint8), 1, time.monotonic())
        assert detector.entered.wait(1.)
        before = time.monotonic()
        worker.close()
        assert time.monotonic() - before < 1.3
        detector.release.set()
        worker._worker.join(1.)
        state = worker.state()
        assert state['status'] == 'closed' and not state['ready'] and state['hands'] == []
        assert not worker.submit(np.zeros((8, 8, 3), np.uint8), 2, time.monotonic())
    finally:
        detector.release.set()
        worker.close()


def test_installer_rejects_unexpected_download_bytes_and_keeps_valid_files(tmp_path, monkeypatch):
    assets = [dict(name='model.weights', url='https://example.invalid/weights',
                   sha256='a' * 64)]
    monkeypatch.setattr(download_hand_model, 'ASSETS', assets)
    with pytest.raises(ValueError, match='SHA-256'):
        download_hand_model.install(tmp_path, opener=lambda *_args, **_kwargs: io.BytesIO(b'incorrecto'))
    assert not (tmp_path / 'model.weights').exists()
    assert list(tmp_path.glob('*.download')) == []


def test_installer_reuses_verified_assets_without_network_access(tmp_path, monkeypatch):
    content = b'pesos locales verificados'
    path = tmp_path / 'model.weights'
    path.write_bytes(content)
    assets = [dict(name=path.name, url='https://example.invalid/weights',
                   sha256=download_hand_model.file_hash(path))]
    monkeypatch.setattr(download_hand_model, 'ASSETS', assets)
    def no_network(*_args, **_kwargs):
        raise AssertionError('La verificación offline no debe acceder a la red.')
    manifest = download_hand_model.install(tmp_path, opener=no_network, check_only=True)
    assert manifest['model'] == 'YOLOv4-tiny Cross-Hands'
    assert not (tmp_path / 'provenance.json').exists()
