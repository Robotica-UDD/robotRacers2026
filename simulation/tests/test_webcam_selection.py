"""Selección estable de cámara USB y arranque lento, sin consultar hardware."""
from __future__ import annotations

import threading
import time

import cv2
from fastapi.testclient import TestClient
import numpy as np

from simulation.webcam.app import CaptureSession, create_app


def wait_until(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.005)
    raise AssertionError("La cámara falsa no llegó al estado esperado.")


def camera_catalog(usb_index=1, include_usb=True):
    options = [{"index": 0, "label": "Cámara integrada", "id": "builtin-camera", "transport": "Built-in"}]
    if include_usb:
        options.append({"index": usb_index, "label": "Webcam USB", "id": "usb-camera", "transport": "USB"})
    return {"enumerated": True, "options": options, "default_index": usb_index if include_usb else 0}


class StartupCapture:
    def __init__(self, empty_reads=0, never_ready=False, gate=None):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 160)
        frame = np.full((320, 400), 255, np.uint8)
        frame[80:240, 120:280] = marker
        self.image = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        self.empty_reads = empty_reads
        self.never_ready = never_ready
        self.gate = gate
        self.reads = 0
        self.released = False

    def isOpened(self):
        return True

    def set(self, *_):
        return True

    def read(self):
        self.reads += 1
        time.sleep(.005)
        if self.never_ready or self.reads <= self.empty_reads:
            return False, None
        if self.gate is not None:
            self.gate.wait(3.)
        return True, self.image.copy()

    def release(self):
        self.released = True


def test_usb_selection_resolves_the_identifier_again_when_indices_change():
    current = [camera_catalog(usb_index=1)]
    refreshes = []
    calls = []
    captures = []
    def discover(force=False):
        refreshes.append(force)
        return current[0]
    def capture_factory(index):
        calls.append(index)
        camera = StartupCapture()
        captures.append(camera)
        return camera
    app = create_app(capture_factory=capture_factory, camera_discovery=discover)
    with TestClient(app) as client:
        assert client.get("/api/cameras").json()["options"][1]["index"] == 1
        # El usuario eligió USB1, pero macOS lo reenumera como índice2 antes de Iniciar.
        current[0] = camera_catalog(usb_index=2)
        response = client.post("/api/start", json={"camera_index": 1, "camera_id": "usb-camera"})
        assert response.status_code == 200
        state = wait_until(lambda: (s if (s := client.get("/api/state").json())["visible"] else None))
        assert calls == [2]
        assert state["camera_index"] == 2
        assert state["camera_id"] == "usb-camera"
        assert state["camera_label"] == "Webcam USB"
        assert not state["opening"]
        assert refreshes[-1] is True
    assert captures[0].released


def test_disappeared_usb_identifier_never_falls_back_to_the_builtin_camera():
    current = [camera_catalog()]
    calls = []
    def discover(force=False):
        return current[0]
    app = create_app(capture_factory=lambda index: calls.append(index), camera_discovery=discover)
    with TestClient(app) as client:
        assert client.get("/api/cameras").json()["options"][1]["id"] == "usb-camera"
        current[0] = camera_catalog(include_usb=False)
        response = client.post("/api/start", json={"camera_index": 0, "camera_id": "usb-camera"})
        assert response.status_code == 409
        state = client.get("/api/state").json()
        assert not state["running"] and not state["visible"]
        assert state["center_px"] is None and state["heading_deg"] is None
    assert calls == []


def test_camera_catalog_exposes_names_and_explicit_refresh_forces_discovery():
    refreshes = []
    calls = []
    def discover(force=False):
        refreshes.append(force)
        return camera_catalog()
    app = create_app(capture_factory=lambda index: calls.append(index), camera_discovery=discover)
    refreshes.clear()
    with TestClient(app) as client:
        ordinary = client.get("/api/cameras").json()
        assert ordinary["enumerated"] is True
        assert ordinary["default_index"] == 1
        assert ordinary["options"][1] == {
            "index": 1, "label": "Webcam USB", "id": "usb-camera", "transport": "USB"
        }
        assert client.get("/api/cameras?refresh=true").status_code == 200
    assert refreshes == [False, True]
    assert calls == []


def test_webcam_startup_tolerates_two_empty_reads_then_detects_the_robot():
    gate = threading.Event()
    capture = StartupCapture(empty_reads=2, gate=gate)
    session = CaptureSession(capture_factory=lambda _: capture, startup_timeout_s=1.)
    try:
        session.start(1, camera_id="usb-camera", camera_label="Webcam USB")
        wait_until(lambda: capture.reads >= 3)
        opening = session.state()
        assert opening["running"] and opening["opening"]
        assert not opening["visible"] and not capture.released
        assert opening["camera_id"] == "usb-camera"
        assert opening["camera_label"] == "Webcam USB"
        gate.set()
        state = wait_until(lambda: (s if (s := session.state())["visible"] else None))
        assert not state["opening"]
        assert state["center_px"] is not None and state["heading_deg"] is not None
    finally:
        gate.set()
        session.stop()
    assert capture.released


def test_webcam_startup_timeout_has_no_pose_and_releases_the_device():
    capture = StartupCapture(never_ready=True)
    session = CaptureSession(capture_factory=lambda _: capture, startup_timeout_s=.03)
    try:
        session.start(1, camera_id="usb-camera", camera_label="Webcam USB")
        state = wait_until(lambda: (s if not (s := session.state())["running"] else None))
        assert not state["opening"] and not state["visible"]
        assert state["center_px"] is None and state["heading_deg"] is None
        assert state["marker_corners"] is None
        assert state["message"]
        assert capture.released
    finally:
        session.stop()
