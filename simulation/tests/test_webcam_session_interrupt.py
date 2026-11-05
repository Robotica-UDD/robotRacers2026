"""El panel puede cancelar una apertura bloqueada sin retener la sesión."""
import threading
import time

from simulation.webcam.app import CaptureSession


def test_stop_interrupts_a_blocked_isolated_camera_open():
    class BlockedCapture:
        interruptible = True

        def __init__(self):
            self.entered = threading.Event()
            self.closed = threading.Event()

        def isOpened(self):
            self.entered.set()
            self.closed.wait(5)
            return False

        def release(self):
            self.closed.set()

    capture = BlockedCapture()
    session = CaptureSession(capture_factory=lambda _: capture)
    try:
        session.start(0)
        assert capture.entered.wait(2)
        began = time.monotonic()
        state = session.stop()
        assert time.monotonic() - began < 1
        assert not state["running"] and not state["opening"]
        assert state["center_px"] is None and state["heading_deg"] is None
        assert capture.closed.is_set()
        assert not session._thread.is_alive()
        assert session._active_capture is None
    finally:
        session.stop()
