"""Procesos de cámara reales con dispositivos falsos; no abre ningún hardware."""
from __future__ import annotations

from dataclasses import dataclass
import multiprocessing as mp
from multiprocessing import shared_memory
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from simulation.webcam.process_capture import ProcessCapture


def devices():
    return [{"index": 0, "id": "built-in", "label": "Integrada"},
            {"index": 2, "id": "usb-current", "label": "USB"}]


@dataclass
class Factory:
    mode: str = "frames"
    called: object = None
    released: object = None
    blocked: object = None
    initial_empty: int = 0

    def __call__(self, index):
        if self.called is not None:
            self.called.set()
        return FakeCamera(index, self)


class FakeCamera:
    def __init__(self, index, factory):
        self.index = index
        self.factory = factory
        self.counter = 0

    def isOpened(self):
        if self.factory.mode == "open_hang":
            self.factory.blocked.set()
            while True:
                time.sleep(.1)
        return self.factory.mode != "failed_open"

    def set(self, *args):
        return True

    def read(self):
        self.counter += 1
        if self.factory.mode == "single_then_block" and self.counter > 1:
            self.factory.blocked.set()
            while True:
                time.sleep(.1)
        time.sleep(.01)
        if self.factory.mode == "empty" or self.counter <= self.factory.initial_empty:
            return False, None
        frame = np.full((24, 32, 3), self.index + 1, np.uint8)
        frame[0, 0, 0] = self.counter % 256
        return True, frame

    def release(self):
        if self.factory.released is not None:
            self.factory.released.set()


def context():
    return mp.get_context("spawn")


def test_process_capture_import_never_opens_a_device_or_starts_a_process():
    program = """
import cv2, multiprocessing
def forbidden(*args, **kwargs):
    raise AssertionError('No se debe abrir cámara ni proceso al importar.')
cv2.VideoCapture = forbidden
multiprocessing.get_context = forbidden
import simulation.webcam.process_capture
"""
    subprocess.run([sys.executable, "-c", program], check=True, capture_output=True, timeout=10.)


def test_process_capture_resolves_current_uid_in_its_own_spawn_process_and_releases():
    called, released = context().Event(), context().Event()
    capture = ProcessCapture(0, "usb-current", capture_factory=Factory(called=called, released=released),
                             devices_provider=devices)
    memory_name = capture._memory.name
    try:
        assert capture.isOpened()
        okay, pixels = capture.read()
        assert okay and pixels.shape == (24, 32, 3)
        # La petición tenía índice0 viejo; UID nuevo resuelve USB al índice2.
        assert pixels[10, 10].tolist() == [3, 3, 3]
        assert capture.frame_capture_stamp is not None
        assert 0 <= time.monotonic() - capture.frame_capture_stamp < 1.
        assert capture.process_pid != __import__('os').getpid()
        assert called.is_set()
        assert not hasattr(capture, "set")
    finally:
        capture.release()
    assert released.is_set()
    assert not capture._process.is_alive()
    capture.release()
    with pytest.raises(FileNotFoundError):
        shared_memory.SharedMemory(name=memory_name)


def test_process_capture_does_not_open_another_device_when_uid_disappears():
    called = context().Event()
    capture = ProcessCapture(0, "usb-removed", capture_factory=Factory(called=called), devices_provider=devices)
    try:
        assert not capture.isOpened()
        assert "ya no está conectada" in capture.error_message
        assert not called.is_set()
        assert capture.read() == (False, None)
    finally:
        capture.release()


def test_process_capture_tolerates_empty_initial_reads_and_returns_new_frames_only():
    capture = ProcessCapture(0, "usb-current", capture_factory=Factory(initial_empty=2), devices_provider=devices)
    try:
        assert capture.isOpened()
        first_ok, first = capture.read()
        first_stamp = capture.frame_capture_stamp
        second_ok, second = capture.read()
        assert first_ok and second_ok
        assert capture.frame_capture_stamp > first_stamp
        assert not np.array_equal(first, second)
    finally:
        capture.release()


def test_process_capture_reports_initial_timeout_and_releases_child_device():
    released = context().Event()
    capture = ProcessCapture(0, "usb-current", startup_timeout_s=.05,
                             capture_factory=Factory(mode="empty", released=released), devices_provider=devices)
    try:
        assert capture.isOpened()
        assert capture.read() == (False, None)
        assert "dejó de entregar imágenes" in capture.error_message
        assert released.wait(1.)
    finally:
        capture.release()


def test_process_capture_can_cancel_an_open_blocked_in_the_driver():
    blocked = context().Event()
    capture = ProcessCapture(0, "usb-current", capture_factory=Factory(mode="open_hang", blocked=blocked),
                             devices_provider=devices)
    result = []
    opener = threading.Thread(target=lambda: result.append(capture.isOpened()))
    opener.start()
    try:
        assert blocked.wait(5.)
        started = time.monotonic()
        capture.release()
        opener.join(1.)
        assert time.monotonic() - started < 2.
        assert result == [False]
        assert not opener.is_alive() and not capture._process.is_alive()
    finally:
        capture.release()
        opener.join(1.)


def test_process_capture_can_cancel_a_blocked_read_without_returning_last_frame():
    blocked = context().Event()
    capture = ProcessCapture(0, "usb-current", capture_factory=Factory(mode="single_then_block", blocked=blocked),
                             devices_provider=devices, read_timeout_s=.1)
    try:
        assert capture.isOpened()
        okay, first = capture.read()
        assert okay and first is not None
        assert blocked.wait(2.)
        # El frame ya consumido nunca se entrega otra vez al agotarse el plazo.
        assert capture.read() == (False, None)
        result = []
        reader = threading.Thread(target=lambda: result.append(capture.read()))
        reader.start()
        capture.release()
        reader.join(1.)
        assert result == [(False, None)]
        assert not capture._process.is_alive()
    finally:
        capture.release()


def test_process_capture_open_timeout_does_not_leave_a_child_running():
    # Conservar el semáforo en el padre hasta que el hijo termine de arrancar.
    blocked = context().Event()
    capture = ProcessCapture(0, "usb-current", capture_factory=Factory(mode="open_hang", blocked=blocked),
                             devices_provider=devices, open_timeout_s=.1)
    try:
        assert not capture.isOpened()
        assert "tardó demasiado" in capture.error_message
        assert not capture._process.is_alive()
    finally:
        capture.release()
