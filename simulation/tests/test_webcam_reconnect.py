"""Reconexión a la misma webcam con capturas falsas, sin consultar hardware."""
from __future__ import annotations

import threading
import time

import numpy as np
from fastapi.testclient import TestClient
import pytest

from simulation.webcam import app as camera_app
from simulation.webcam.app import CaptureSession, create_app
from simulation.webcam.detector import Detection
from simulation.webcam.navigation import Navigation


CAMERA_ID = "usb-camera-original"
OLD_USB_ID = "0x3110000145f02f7"
NEW_USB_ID = "0x3120000145f02f7"
OTHER_USB_ID = "0x3130000145f02f7"
USB_LABEL = "Trust Full HD Webcam (USB)"


def wait_until(predicate, timeout=2.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.005)
    raise AssertionError("La captura simulada no llegó al estado esperado.")


class FixedDetector:
    def process(self, pixels):
        return Detection(True, (320., 240.), 0.,
                         [[280., 200.], [360., 200.], [360., 280.], [280., 280.]])


class FakeCapture:
    interruptible = True

    def __init__(self, *, opened=True):
        self.opened = opened
        self.released = threading.Event()
        self.disconnected = threading.Event()
        self.empty = threading.Event()
        self.empty_reads = 0
        self.reads = 0
        self.frame = np.full((480, 640, 3), 255, np.uint8)

    @property
    def error_message(self):
        return "La webcam USB se desconectó." if self.disconnected.is_set() else ""

    def isOpened(self):
        return self.opened and not self.released.is_set()

    def set(self, *args):
        return True

    def read(self):
        self.reads += 1
        self.released.wait(.005)
        if self.released.is_set() or self.disconnected.is_set():
            return False, None
        if self.empty.is_set():
            self.empty_reads += 1
            return False, None
        return True, self.frame.copy()

    def release(self):
        self.released.set()


def usb_catalog(*identifiers, label=USB_LABEL):
    return {"enumerated": True,
            "options": [{"index": 0, "id": "builtin-camera", "label": "Cámara del Mac",
                         "transport": "Built-in"}]
            + [{"index": index + 1, "id": identifier, "label": label, "transport": "USB"}
               for index, identifier in enumerate(identifiers)]}


def test_native_default_reopens_with_the_original_camera_id_and_keeps_sequence(monkeypatch):
    first, second = FakeCapture(), FakeCapture()
    captures = [first, second]
    opened = []

    def fake_open(index, camera_id=None):
        opened.append((index, camera_id))
        assert len(opened) <= len(captures), "La cámara recuperada no debe reabrirse sola."
        return captures[len(opened) - 1]

    monkeypatch.setattr(camera_app, "open_camera", fake_open)
    monkeypatch.setattr(camera_app, "discover_cameras", lambda force=False:
                        {"enumerated": True, "options": [{"index": 1, "id": CAMERA_ID}]})
    session = CaptureSession(detector=FixedDetector(), reconnect_interval_s=.04)
    try:
        assert session.reconnect_interval_s == .04
        session.start(1, CAMERA_ID, "Webcam USB")
        initial = wait_until(lambda: (s if (s := session.state())["visible"] else None))
        first.disconnected.set()
        waiting = wait_until(lambda: (s if (s := session.state())["opening"] and first.released.is_set() else None))
        assert waiting["running"] and not waiting["visible"]
        assert waiting["camera_id"] == CAMERA_ID
        assert waiting["frame_width"] == initial["frame_width"] == 640
        assert waiting["frame_height"] == initial["frame_height"] == 480
        assert waiting["sequence"] >= initial["sequence"] > 0
        assert waiting["center_px"] is None and waiting["heading_deg"] is None
        assert waiting["marker_corners"] is None
        recovered = wait_until(lambda: (s if len(opened) == 2 and (s := session.state())["visible"] else None))
        assert recovered["running"] and not recovered["opening"]
        assert recovered["sequence"] > waiting["sequence"]
        assert recovered["camera_id"] == CAMERA_ID
        assert opened == [(1, CAMERA_ID), (1, CAMERA_ID)]
    finally:
        session.stop()
    assert second.released.is_set()


def test_short_empty_frame_gap_does_not_teardown_or_reconnect():
    capture = FakeCapture()
    opens = []

    def factory(index):
        opens.append(index)
        return capture

    session = CaptureSession(factory, FixedDetector(), autoreconnect=True,
                             reconnect_interval_s=.02, frame_gap_timeout_s=.4)
    try:
        session.start(1, CAMERA_ID)
        initial = wait_until(lambda: (s if (s := session.state())["visible"] else None))
        capture.empty.set()
        wait_until(lambda: capture.empty_reads >= 3)
        assert session.state()["running"]
        assert not capture.released.is_set()
        assert opens == [1]
        capture.empty.clear()
        recovered = wait_until(lambda: (s if (s := session.state())["sequence"] > initial["sequence"] + 1 else None))
        assert recovered["visible"] and not recovered["opening"]
        assert not capture.released.is_set()
        assert opens == [1]
    finally:
        session.stop()


def test_persistent_empty_frames_reopen_capture_and_retire_the_old_pose():
    first, second = FakeCapture(), FakeCapture()
    captures = [first, second]
    opens = []

    def factory(index):
        opens.append(index)
        return captures[len(opens) - 1]

    session = CaptureSession(factory, FixedDetector(), autoreconnect=True,
                             reconnect_interval_s=.05, frame_gap_timeout_s=.05)
    try:
        session.start(1, CAMERA_ID)
        wait_until(lambda: session.state()["visible"])
        first.empty.set()
        waiting = wait_until(lambda: (s if (s := session.state())["opening"] and first.released.is_set() else None))
        assert waiting["running"] and not waiting["visible"]
        assert waiting["center_px"] is None and waiting["marker_corners"] is None
        wait_until(lambda: len(opens) == 2 and session.state()["visible"])
        assert opens == [1, 1]
    finally:
        session.stop()


def test_missing_selected_id_never_falls_back_and_reenumeration_uses_its_new_index():
    first, second = FakeCapture(), FakeCapture()
    cameras = [{"index": 0, "id": "builtin-camera"}, {"index": 1, "id": CAMERA_ID}]
    discoveries, opens = [], []

    def discover(force=False):
        discoveries.append(force)
        return {"enumerated": True, "options": list(cameras)}

    def factory(index):
        opens.append(index)
        assert index != 0, "Nunca abrir la cámara integrada como alternativa."
        return first if len(opens) == 1 else second

    session = CaptureSession(factory, FixedDetector(), autoreconnect=True,
                             reconnect_interval_s=.03, camera_discovery=discover)
    try:
        session.start(1, CAMERA_ID)
        wait_until(lambda: session.state()["visible"])
        cameras[:] = [{"index": 0, "id": "builtin-camera"}]
        first.disconnected.set()
        wait_until(lambda: len(discoveries) >= 3)
        state = session.state()
        assert state["running"] and state["opening"] and not state["visible"]
        assert state["camera_id"] == CAMERA_ID
        assert opens == [1]
        cameras.append({"index": 2, "id": CAMERA_ID})
        recovered = wait_until(lambda: (s if len(opens) == 2 and (s := session.state())["visible"] else None))
        assert recovered["camera_index"] == 2
        assert recovered["camera_id"] == CAMERA_ID
        assert opens == [1, 2]
        assert all(discoveries)
    finally:
        session.stop()


def test_stop_during_retry_wait_cancels_all_future_reopens():
    capture = FakeCapture(opened=False)
    opens = []

    def factory(index):
        opens.append(index)
        return capture

    session = CaptureSession(factory, FixedDetector(), autoreconnect=True, reconnect_interval_s=1.)
    try:
        session.start(1, CAMERA_ID)
        wait_until(lambda: capture.released.is_set() and session.state()["opening"])
        began = time.monotonic()
        stopped = session.stop()
        assert time.monotonic() - began < .5
        assert not stopped["running"] and not stopped["opening"] and not stopped["visible"]
        assert not session._thread.is_alive()
        time.sleep(.04)
        assert opens == [1]
    finally:
        session.stop()


def test_manual_stop_interrupts_a_reconnection_open_and_does_not_restart():
    class BlockedOpen(FakeCapture):
        def __init__(self):
            super().__init__()
            self.entered = threading.Event()

        def isOpened(self):
            self.entered.set()
            self.released.wait(2)
            return False

    first, blocked = FakeCapture(), BlockedOpen()
    opens = []

    def factory(index):
        opens.append(index)
        return first if len(opens) == 1 else blocked

    session = CaptureSession(factory, FixedDetector(), autoreconnect=True, reconnect_interval_s=.02)
    try:
        session.start(1, CAMERA_ID)
        wait_until(lambda: session.state()["visible"])
        first.disconnected.set()
        assert blocked.entered.wait(1)
        stopped = session.stop()
        assert not stopped["running"] and not stopped["opening"]
        assert not session._thread.is_alive()
        assert blocked.released.is_set()
        time.sleep(.04)
        assert opens == [1, 1]
    finally:
        session.stop()


def test_injected_capture_does_not_autoreconnect_unless_explicitly_enabled():
    capture = FakeCapture(opened=False)
    opens = []
    session = CaptureSession(lambda index: opens.append(index) or capture, FixedDetector(),
                             reconnect_interval_s=.01)
    try:
        session.start(1, CAMERA_ID)
        wait_until(lambda: not session.state()["running"])
        time.sleep(.04)
        assert opens == [1]
    finally:
        session.stop()


def test_index_without_stable_identity_never_autoreconnects_to_another_camera():
    capture = FakeCapture(opened=False)
    opens = []
    session = CaptureSession(lambda index: opens.append(index) or capture, FixedDetector(),
                             autoreconnect=True, reconnect_interval_s=.01)
    try:
        session.start(1, "index:1")
        wait_until(lambda: not session.state()["running"])
        time.sleep(.04)
        assert opens == [1]
    finally:
        session.stop()


def test_reconnection_keeps_navigation_calibration_and_selected_target(tmp_path):
    class NoMotionRobot:
        def stop_navigation(self):
            pass

        def state(self):
            return {"connected": False, "ready": False, "moving": False}

    first, second = FakeCapture(), FakeCapture()
    opens = []

    def factory(index):
        opens.append(index)
        return first if len(opens) == 1 else second

    session = CaptureSession(factory, FixedDetector(), autoreconnect=True, reconnect_interval_s=.08)
    navigation = Navigation(session, NoMotionRobot(), tmp_path / "navigation-state.json")
    try:
        session.start(1, CAMERA_ID)
        wait_until(lambda: session.state()["visible"])
        navigation.calibrate([320., 240.], [360., 240.], 640, 480, CAMERA_ID)
        navigation.select_target(440., 240., 640, 480, CAMERA_ID)
        before = navigation.state()
        saved = (tmp_path / "navigation-state.json").read_bytes()
        first.disconnected.set()
        wait_until(lambda: first.released.is_set() and session.state()["opening"])
        waiting = navigation.state()
        assert waiting["calibrated"]
        assert waiting["target_px"] == before["target_px"] == [440., 240.]
        assert waiting["center_px"] is None
        wait_until(lambda: len(opens) == 2 and session.state()["visible"])
        recovered = navigation.state()
        assert recovered["calibrated"] and recovered["target_px"] == before["target_px"]
        assert recovered["sequence"] > before["sequence"]
        assert (tmp_path / "navigation-state.json").read_bytes() == saved
    finally:
        navigation.close()
        session.stop()


def test_camera_stop_endpoint_preserves_destination_and_calibration_without_canceling_mission():
    catalog = {"enumerated": True, "options": [{"index": 1, "id": CAMERA_ID, "label": "Webcam USB"}]}
    app = create_app(capture_factory=lambda _: FakeCapture(), detector=FixedDetector(),
                     camera_discovery=lambda force=False: catalog)
    with TestClient(app) as client:
        assert client.post("/api/start", json={"camera_index": 1, "camera_id": CAMERA_ID}).status_code == 200
        wait_until(lambda: client.get("/api/state").json()["visible"])
        navigation = app.state.navigation
        navigation.calibrate([320., 240.], [360., 240.], 640, 480, CAMERA_ID)
        navigation.select_target(440., 240., 640, 480, CAMERA_ID)
        cancellations = []
        navigation.cancel_local = cancellations.append
        stopped = client.post("/api/stop")
        assert stopped.status_code == 200
        assert not stopped.json()["running"] and not stopped.json()["visible"]
        assert cancellations == []
        state = navigation.state()
        assert state["calibrated"] and state["target_px"] == [440., 240.]


def test_usb_port_change_reopens_native_id_but_keeps_calibration_identity(monkeypatch, tmp_path):
    class NoMotionRobot:
        def stop_navigation(self):
            pass

        def state(self):
            return {"connected": False, "ready": False, "moving": False}

    first, second = FakeCapture(), FakeCapture()
    catalog = [usb_catalog(OLD_USB_ID)]
    opened = []

    def fake_open(index, camera_id=None):
        opened.append((index, camera_id))
        assert len(opened) <= 2
        return first if len(opened) == 1 else second

    monkeypatch.setattr(camera_app, "open_camera", fake_open)
    monkeypatch.setattr(camera_app, "discover_cameras", lambda force=False: catalog[0])
    session = CaptureSession(detector=FixedDetector(), reconnect_interval_s=.03)
    navigation = Navigation(session, NoMotionRobot(), tmp_path / "navigation-state.json")
    try:
        session.start(1, OLD_USB_ID, USB_LABEL)
        wait_until(lambda: session.state()["visible"])
        navigation.calibrate([320., 240.], [360., 240.], 640, 480, OLD_USB_ID)
        navigation.select_target(440., 240., 640, 480, OLD_USB_ID)
        before = navigation.state()
        saved = (tmp_path / "navigation-state.json").read_bytes()
        catalog[0] = usb_catalog(NEW_USB_ID)
        first.disconnected.set()
        recovered = wait_until(lambda: (s if len(opened) == 2 and (s := session.state())["visible"] else None))
        assert opened == [(1, OLD_USB_ID), (1, NEW_USB_ID)]
        assert recovered["camera_id"] == OLD_USB_ID
        assert recovered["native_camera_id"] == NEW_USB_ID
        assert recovered["camera_label"] == USB_LABEL
        assert recovered["sequence"] > before["sequence"]
        assert recovered["frame_width"] == 640 and recovered["frame_height"] == 480
        assert navigation.state()["calibrated"]
        assert navigation.state()["target_px"] == [440., 240.]
        assert (tmp_path / "navigation-state.json").read_bytes() == saved
    finally:
        navigation.close()
        session.stop()


@pytest.mark.parametrize("replacement", [usb_catalog(NEW_USB_ID, OTHER_USB_ID),
                                          usb_catalog(NEW_USB_ID, label="Otra webcam (USB)"),
                                          usb_catalog("0x3120000123456789"),
                                          {"enumerated": True, "options": [
                                              {"index": 0, "id": NEW_USB_ID,
                                               "label": USB_LABEL, "transport": "Built-in"}]}])
def test_usb_alias_waits_for_unique_same_model_name_and_usb_transport(replacement):
    first = FakeCapture()
    catalog = [usb_catalog(OLD_USB_ID)]
    discoveries, opens = [], []

    def discover(force=False):
        discoveries.append(force)
        return catalog[0]

    def factory(index):
        opens.append(index)
        assert len(opens) == 1, "Una candidata ambigua o distinta no debe abrirse."
        return first

    session = CaptureSession(factory, FixedDetector(), autoreconnect=True,
                             reconnect_interval_s=.02, camera_discovery=discover)
    try:
        session.start(1, OLD_USB_ID, USB_LABEL)
        wait_until(lambda: session.state()["visible"])
        catalog[0] = replacement
        first.disconnected.set()
        wait_until(lambda: len(discoveries) >= 3)
        state = session.state()
        assert state["running"] and state["opening"] and not state["visible"]
        assert state["camera_id"] == state["native_camera_id"] == OLD_USB_ID
        assert opens == [1]
    finally:
        session.stop()


@pytest.mark.parametrize("requested_id", [OLD_USB_ID, NEW_USB_ID])
def test_restart_endpoint_accepts_port_alias_without_resetting_navigation(requested_id):
    catalog = [usb_catalog(OLD_USB_ID)]
    opens = []

    def factory(index):
        opens.append(index)
        return FakeCapture()

    app = create_app(capture_factory=factory, detector=FixedDetector(),
                     camera_discovery=lambda force=False: catalog[0])
    with TestClient(app) as client:
        assert client.post("/api/start", json={"camera_index": 1, "camera_id": OLD_USB_ID}).status_code == 200
        wait_until(lambda: client.get("/api/state").json()["visible"])
        navigation = app.state.navigation
        navigation.calibrate([320., 240.], [360., 240.], 640, 480, OLD_USB_ID)
        navigation.select_target(440., 240., 640, 480, OLD_USB_ID)
        before = app.state.camera.state()
        assert client.post("/api/stop").status_code == 200
        catalog[0] = usb_catalog(NEW_USB_ID)
        response = client.post("/api/start", json={"camera_index": 1, "camera_id": requested_id})
        assert response.status_code == 200
        initial = response.json()
        assert initial["camera_id"] == OLD_USB_ID and initial["native_camera_id"] == NEW_USB_ID
        assert initial["frame_width"] == 640 and initial["frame_height"] == 480
        assert initial["sequence"] >= before["sequence"]
        recovered = wait_until(lambda: (s if (s := app.state.camera.state())["visible"] else None))
        assert recovered["sequence"] > before["sequence"]
        assert navigation.state()["calibrated"]
        assert navigation.state()["target_px"] == [440., 240.]
        assert opens == [1, 1]


def test_start_endpoint_preserves_saved_calibration_id_after_application_restart():
    catalog = usb_catalog(NEW_USB_ID)
    app = create_app(capture_factory=lambda _: FakeCapture(), detector=FixedDetector(),
                     camera_discovery=lambda force=False: catalog)
    app.state.navigation.saved_camera_id = lambda: OLD_USB_ID
    with TestClient(app) as client:
        response = client.post("/api/start", json={"camera_index": 1, "camera_id": NEW_USB_ID})
        assert response.status_code == 200
        assert response.json()["camera_id"] == OLD_USB_ID
        assert response.json()["native_camera_id"] == NEW_USB_ID
        assert app.state.navigation.saved_camera_id() == OLD_USB_ID


@pytest.mark.parametrize("requested_id", [OLD_USB_ID, NEW_USB_ID])
def test_start_endpoint_rejects_ambiguous_port_alias_and_keeps_saved_metadata(requested_id):
    catalog = [usb_catalog(OLD_USB_ID)]
    opens = []
    app = create_app(capture_factory=lambda index: opens.append(index) or FakeCapture(),
                     detector=FixedDetector(), camera_discovery=lambda force=False: catalog[0])
    with TestClient(app) as client:
        assert client.post("/api/start", json={"camera_index": 1, "camera_id": OLD_USB_ID}).status_code == 200
        wait_until(lambda: app.state.camera.state()["visible"])
        navigation = app.state.navigation
        navigation.calibrate([320., 240.], [360., 240.], 640, 480, OLD_USB_ID)
        navigation.select_target(440., 240., 640, 480, OLD_USB_ID)
        assert client.post("/api/stop").status_code == 200
        catalog[0] = usb_catalog(NEW_USB_ID, OTHER_USB_ID)
        response = client.post("/api/start", json={"camera_index": 1, "camera_id": requested_id})
        assert response.status_code == 409
        assert opens == [1]
        assert app.state.camera.state()["camera_id"] == OLD_USB_ID
        assert navigation.state()["calibrated"]
        assert navigation.state()["target_px"] == [440., 240.]
