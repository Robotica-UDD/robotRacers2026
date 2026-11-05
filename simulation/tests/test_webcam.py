"""Prueba la visión de webcam con marcadores reales y cámaras falsas, sin hardware."""
from __future__ import annotations

import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient

from simulation.webcam.app import CaptureSession, create_app
from simulation.webcam.detector import AprilTagDetector, annotate_frame


def marker_frame(identifier=0, rotation=0):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, identifier, 160)
    gray = np.full((360, 480), 255, np.uint8)
    gray[100:260, 160:320] = marker
    return cv2.cvtColor(np.rot90(gray, rotation).copy(), cv2.COLOR_GRAY2BGR)


def wait_until(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.01)
    raise AssertionError("La captura falsa no llegó al estado esperado.")


class FakeCapture:
    def __init__(self, frame=None, opened=True):
        self.image = marker_frame() if frame is None else frame
        self.opened = opened
        self.released = False
        self.disconnected = threading.Event()
        self.second_read = threading.Event()
        self.gate = None
        self.reads = 0

    def isOpened(self):
        return self.opened

    def set(self, *_):
        return True

    def read(self):
        self.reads += 1
        if self.reads > 1:
            self.second_read.set()
            if self.gate is not None:
                self.gate.wait(3.)
        time.sleep(.005)
        if self.disconnected.is_set() or self.released:
            return False, None
        return True, self.image.copy()

    def release(self):
        self.released = True


@pytest.mark.parametrize("rotation, expected", [(0, 0.), (1, 90.), (2, 180.), (3, -90.)])
def test_webcam_marker_orientation_is_its_right_edge_without_mirroring(rotation, expected):
    frame = marker_frame(rotation=rotation)
    result = AprilTagDetector().process(frame)
    assert result.visible
    center = np.asarray(result.center_px)
    assert center == pytest.approx([(frame.shape[1] - 1) / 2, (frame.shape[0] - 1) / 2], abs=1.)
    angle_error = (result.heading_deg - expected + 180.) % 360. - 180.
    assert angle_error == pytest.approx(0., abs=.5)
    assert np.asarray(result.marker_corners).shape == (4, 2)
    assert result.processing_ms >= 0


@pytest.mark.parametrize("mode", ["blank", "other_identifier", "occluded"])
def test_webcam_lost_marker_clears_position_and_orientation(mode):
    detector = AprilTagDetector()
    assert detector.process(marker_frame()).visible
    if mode == "blank":
        frame = np.full((360, 480, 3), 255, np.uint8)
    elif mode == "other_identifier":
        frame = marker_frame(identifier=1)
    else:
        frame = marker_frame()
        frame[100:260, 160:255] = 255
    result = detector.process(frame)
    assert not result.visible
    assert result.center_px is None
    assert result.heading_deg is None
    assert result.marker_corners is None
    # El fotograma sin detección no conserva ni caja ni flecha anteriores.
    annotated = annotate_frame(frame, result)
    assert annotated.shape == frame.shape


def test_webcam_annotation_does_not_modify_the_detector_input():
    frame = marker_frame()
    original = frame.copy()
    result = AprilTagDetector().process(frame)
    annotated = annotate_frame(frame, result)
    assert np.array_equal(frame, original)
    assert not np.array_equal(annotated, original)


def test_webcam_startup_and_camera_choices_do_not_open_a_camera():
    calls = []
    def factory(index):
        calls.append(index)
        return FakeCapture()
    app = create_app(capture_factory=factory)
    assert calls == []
    with TestClient(app) as client:
        state = client.get("/api/state").json()
        assert not state["running"] and not state["visible"]
        assert state["center_px"] is None and state["heading_deg"] is None
        cameras = client.get("/api/cameras").json()
        assert cameras["enumerated"] is False
        assert cameras["options"][0]["index"] == 0
        assert calls == []
    assert calls == []


def test_webcam_import_does_not_access_camera_hardware():
    program = """
import cv2
def forbidden(*args, **kwargs):
    raise AssertionError('Importar webcam no debe abrir ninguna cámara.')
cv2.VideoCapture = forbidden
import simulation.webcam.app
import simulation.webcam.__main__
"""
    subprocess.run([sys.executable, "-c", program], check=True, capture_output=True, text=True, timeout=10.)


@pytest.mark.parametrize("index", [-1, 10, 1.5, "/tmp/video.mp4", "rtsp://example.invalid"])
def test_webcam_rejects_invalid_camera_indices_without_opening(index):
    calls = []
    app = create_app(capture_factory=lambda index: calls.append(index))
    with TestClient(app) as client:
        response = client.post("/api/start", json={"camera_index": index})
        assert response.status_code == 422
    assert calls == []


def test_webcam_start_stop_and_restart_release_the_capture():
    captures = []
    calls = []
    def factory(index):
        calls.append(index)
        capture = FakeCapture()
        captures.append(capture)
        return capture
    app = create_app(capture_factory=factory)
    with TestClient(app) as client:
        assert client.post("/api/start", json={"camera_index": 2}).status_code == 200
        visible = wait_until(lambda: (s if (s := client.get("/api/state").json())["visible"] else None))
        assert visible["running"] and visible["camera_index"] == 2
        assert visible["frame_width"] == 480 and visible["frame_height"] == 360
        assert visible["center_px"] == pytest.approx([239.5, 179.5], abs=1.)
        assert (visible["heading_deg"] + 180.) % 360. - 180. == pytest.approx(0., abs=.5)
        assert visible["marker_id"] == 0
        assert visible["sequence"] > 0
        token, jpeg = app.state.camera.frame()
        assert token is not None
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None and decoded.shape == (360, 480, 3)
        stopped = client.post("/api/stop").json()
        assert not stopped["running"] and not stopped["visible"]
        assert stopped["center_px"] is None and stopped["heading_deg"] is None
        assert captures[0].released
        assert client.post("/api/start", json={"camera_index": 1}).status_code == 200
        wait_until(lambda: client.get("/api/state").json()["visible"])
    assert captures[-1].released
    assert calls == [2, 1]


def test_webcam_disconnect_retires_the_last_detected_pose_and_releases_device():
    capture = FakeCapture()
    session = CaptureSession(capture_factory=lambda _: capture)
    try:
        session.start(0)
        wait_until(lambda: session.state()["visible"])
        capture.disconnected.set()
        state = wait_until(lambda: (s if not (s := session.state())["running"] else None))
        assert not state["visible"]
        assert state["center_px"] is None and state["heading_deg"] is None
        assert state["marker_corners"] is None
        assert state["message"]
        wait_until(lambda: capture.released)
    finally:
        session.stop()


def test_webcam_failed_open_is_reported_and_releases_device():
    capture = FakeCapture(opened=False)
    session = CaptureSession(capture_factory=lambda _: capture)
    try:
        session.start(0)
        state = wait_until(lambda: (s if not (s := session.state())["running"] else None))
        assert not state["visible"]
        assert state["center_px"] is None and state["heading_deg"] is None
        assert state["message"]
        wait_until(lambda: capture.released)
    finally:
        session.stop()


def test_webcam_stale_frame_hides_pose_while_capture_is_blocked():
    capture = FakeCapture()
    capture.gate = threading.Event()
    now = [100.]
    session = CaptureSession(capture_factory=lambda _: capture, clock=lambda: now[0], stale_after_s=.75)
    try:
        session.start(0)
        wait_until(lambda: session.state()["visible"])
        assert capture.second_read.wait(2.)
        now[0] += 1.
        state = session.state()
        assert not state["visible"]
        assert state["center_px"] is None and state["heading_deg"] is None
        assert state["marker_corners"] is None
    finally:
        capture.gate.set()
        session.stop()


def test_webcam_detector_failure_retires_pose_and_releases_the_camera():
    capture = FakeCapture()
    capture.gate = threading.Event()
    class FailingDetector:
        def __init__(self):
            self.detector = AprilTagDetector()
            self.calls = 0
        def process(self, frame):
            self.calls += 1
            if self.calls > 1:
                raise ValueError("Fallo de procesamiento simulado.")
            return self.detector.process(frame)
    session = CaptureSession(capture_factory=lambda _: capture, detector=FailingDetector())
    try:
        session.start(0)
        wait_until(lambda: session.state()["visible"])
        _, last_detected_image = session.frame()
        assert capture.second_read.wait(2.)
        capture.gate.set()
        state = wait_until(lambda: (s if not (s := session.state())["running"] else None))
        assert not state["visible"]
        assert state["center_px"] is None and state["heading_deg"] is None
        assert state["marker_corners"] is None
        assert state["message"]
        assert session.frame()[1] != last_detected_image
        assert capture.released
    finally:
        capture.gate.set()
        session.stop()


def test_webcam_interface_and_printable_assets_are_local_and_the_png_is_id_zero():
    calls = []
    app = create_app(capture_factory=lambda index: calls.append(index))
    with TestClient(app) as client:
        page = client.get("/")
        assert page.status_code == 200
        assert "text/html" in page.headers["content-type"]
        assert page.headers["cache-control"] == "no-store"
        assert "/assets/apriltag-0-40mm.pdf" in page.text
        pdf = client.get("/assets/apriltag-0-40mm.pdf")
        assert pdf.status_code == 200 and pdf.content.startswith(b"%PDF-")
        assert "application/pdf" in pdf.headers["content-type"]
        png = client.get("/assets/apriltag-0-margen.png")
        assert png.status_code == 200 and "image/png" in png.headers["content-type"]
        image = cv2.imdecode(np.frombuffer(png.content, np.uint8), cv2.IMREAD_COLOR)
        assert image is not None
        detection = AprilTagDetector().process(image)
        assert detection.visible
        assert (detection.heading_deg + 180.) % 360. - 180. == pytest.approx(0., abs=.5)
    assert calls == []
