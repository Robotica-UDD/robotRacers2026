"""Captura aislada: cada inicio utiliza un proceso nativo nuevo de macOS.

Los píxeles viajan por memoria compartida con secuencia impar/par. La cola de
un elemento solo avisa de un cambio: nunca acumula fotogramas ni transporta una
imagen grande que pudiera quedar truncada al cancelar un controlador bloqueado.
"""
from __future__ import annotations

import multiprocessing as mp
from multiprocessing import shared_memory
import queue
import struct
import sys
import threading
import time
from typing import Callable

import cv2
import numpy as np


HEADER = struct.Struct("<QIIId")
MAX_FRAME_BYTES = 4096 * 2160 * 3


def _native_capture(index: int):
    return cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION) if sys.platform == "darwin" else cv2.VideoCapture(index)


def _resolve_index(index: int, camera_id: str | None, provider: Callable | None) -> int:
    if provider is None and sys.platform == "darwin":
        from .devices import _macos_devices
        provider = _macos_devices
    if provider is None:
        if camera_id and not camera_id.startswith("index:"):
            raise RuntimeError("No se pudo confirmar la identidad de la cámara seleccionada.")
        return index
    devices = provider()
    if camera_id and not camera_id.startswith("index:"):
        selected = next((item for item in devices if item.get("id") == camera_id), None)
        if selected is None:
            raise RuntimeError("La cámara seleccionada ya no está conectada. Actualiza la lista de cámaras.")
        return int(selected["index"])
    if not any(item["index"] == index for item in devices):
        raise RuntimeError("La cámara seleccionada ya no está conectada. Actualiza la lista de cámaras.")
    return index


def _capture_worker(index, camera_id, camera_label, stop, notices, status, memory_name,
                    capacity, startup_timeout_s, capture_factory=None, devices_provider=None):
    """Único propietario de AVFoundation y del dispositivo físico."""
    capture = None
    memory = None
    try:
        memory = shared_memory.SharedMemory(name=memory_name)
        resolved = _resolve_index(index, camera_id, devices_provider)
        capture = (capture_factory or _native_capture)(resolved)
        if not capture.isOpened():
            raise RuntimeError("No se pudo abrir la cámara seleccionada. Comprueba su conexión y el permiso de Cámara.")
        if hasattr(capture, "set"):
            for prop, value in ((cv2.CAP_PROP_FRAME_WIDTH, 1280), (cv2.CAP_PROP_FRAME_HEIGHT, 720),
                                (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_BUFFERSIZE, 1)):
                try:
                    capture.set(prop, value)
                except Exception:
                    # Un control opcional no debe impedir usar el formato nativo.
                    pass
        status.send({"type": "opened", "index": resolved, "camera_label": camera_label})
        initial_deadline = time.monotonic() + startup_timeout_s
        first_frame = False
        empty_since = None
        sequence = 0
        while not stop.is_set():
            okay, pixels = capture.read()
            stamp = time.monotonic()
            if stop.is_set():
                break
            if not okay or pixels is None or pixels.size == 0:
                empty_since = stamp if empty_since is None else empty_since
                expired = stamp >= initial_deadline if not first_frame else stamp - empty_since >= .8
                if expired:
                    raise RuntimeError("La cámara dejó de entregar imágenes. Comprueba el cable USB y vuelve a iniciar.")
                stop.wait(.03)
                continue
            first_frame = True
            empty_since = None
            if pixels.ndim == 2:
                pixels = cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
            elif pixels.ndim == 3 and pixels.shape[2] == 4:
                pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
            if pixels.ndim != 3 or pixels.shape[2] != 3 or pixels.dtype != np.uint8:
                raise RuntimeError("La cámara entregó un formato de imagen no compatible.")
            height, width, channels = pixels.shape
            length = int(pixels.size)
            if not (0 < width <= 4096 and 0 < height <= 2160 and length <= capacity):
                raise RuntimeError("La resolución de la cámara excede el límite de captura.")
            sequence += 2
            HEADER.pack_into(memory.buf, 0, sequence - 1, width, height, channels, stamp)
            memory.buf[HEADER.size:HEADER.size + length] = pixels.tobytes()
            struct.pack_into("<Q", memory.buf, 0, sequence)
            try:
                notices.put_nowait(sequence)
            except queue.Full:
                # El aviso pendiente basta: el lector copia la versión más nueva
                # de la memoria, no la imagen correspondiente al aviso anterior.
                pass
    except Exception as exc:
        try:
            status.send({"type": "error", "message": str(exc) if isinstance(exc, RuntimeError)
                         else "Falló la captura de la cámara. Detén e inicia de nuevo."})
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if capture is not None:
            try:
                capture.release()
            except Exception:
                pass
        if memory is not None:
            memory.close()
        try:
            status.send({"type": "closed"})
        except (BrokenPipeError, EOFError, OSError):
            pass
        status.close()
        notices.cancel_join_thread()
        notices.close()


class ProcessCapture:
    """Interfaz de captura cancelable; no crea procesos al importar el módulo.

    ``capture_factory`` y ``devices_provider`` permiten pruebas con objetos
    serializables, siempre en un proceso ``spawn`` independiente.
    """
    interruptible = True

    def __init__(self, index: int, camera_id: str | None = None, camera_label: str | None = None,
                 *, open_timeout_s: float = 8., read_timeout_s: float = 1.25,
                 startup_timeout_s: float = 5., capture_factory=None, devices_provider=None,
                 worker_target=None):
        self.error_message = ""
        self.frame_capture_stamp: float | None = None
        self._open_timeout_s = open_timeout_s
        self._read_timeout_s = read_timeout_s
        self._started_at = time.monotonic()
        self._opened = False
        self._terminal = False
        self._closed = threading.Event()
        self._release_lock = threading.Lock()
        self._memory_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._read_lock = threading.Lock()
        self._last_sequence = 0
        context = mp.get_context("spawn")
        self._stop = context.Event()
        self._notices = context.Queue(maxsize=1)
        self._status, sender = context.Pipe(duplex=False)
        self._memory = shared_memory.SharedMemory(create=True, size=HEADER.size + MAX_FRAME_BYTES)
        self._memory.buf[:HEADER.size] = bytes(HEADER.size)
        self._process = context.Process(
            target=worker_target or _capture_worker,
            args=(index, camera_id, camera_label, self._stop, self._notices, sender,
                  self._memory.name, MAX_FRAME_BYTES, startup_timeout_s, capture_factory, devices_provider),
            name="robotracers-camera", daemon=True,
        )
        try:
            self._process.start()
        except BaseException:
            sender.close()
            self.release()
            raise
        sender.close()

    @property
    def process_pid(self):
        return self._process.pid

    def _refresh_status(self):
        with self._status_lock:
            if self._closed.is_set():
                return
            try:
                while self._status.poll():
                    message = self._status.recv()
                    if message.get("type") == "opened":
                        self._opened = True
                    elif message.get("type") == "error":
                        self.error_message = message.get("message") or "Falló la captura de la cámara."
                        self._terminal = True
                    elif message.get("type") == "closed":
                        self._terminal = True
            except (EOFError, OSError):
                self._terminal = True
            if not self._process.is_alive():
                self._terminal = True
            if self._terminal and not self.error_message and not self._stop.is_set():
                self.error_message = "El proceso de cámara terminó. Vuelve a iniciar la captura."

    def isOpened(self):
        deadline = self._started_at + self._open_timeout_s
        while not self._closed.is_set():
            self._refresh_status()
            if self._terminal:
                return False
            if self._opened:
                return True
            if time.monotonic() >= deadline:
                self.error_message = "La cámara tardó demasiado en abrir. Comprueba su conexión y vuelve a iniciar."
                self.release()
                return False
            self._closed.wait(.02)
        return False

    def _new_frame(self):
        with self._memory_lock:
            if self._closed.is_set():
                return None
            try:
                before, width, height, channels, stamp = HEADER.unpack_from(self._memory.buf)
                if before == 0 or before % 2 or before <= self._last_sequence:
                    return None
                length = width * height * channels
                if not (0 < width <= 4096 and 0 < height <= 2160 and channels == 3 and length <= MAX_FRAME_BYTES):
                    return None
                payload = bytes(self._memory.buf[HEADER.size:HEADER.size + length])
                after = struct.unpack_from("<Q", self._memory.buf)[0]
                if before != after or after % 2:
                    return None
                self._last_sequence = before
                self.frame_capture_stamp = stamp
                return np.frombuffer(payload, np.uint8).reshape(height, width, channels)
            except (ValueError, OSError, TypeError, struct.error):
                return None

    def read(self):
        with self._read_lock:
            deadline = time.monotonic() + self._read_timeout_s
            while not self._closed.is_set():
                self._refresh_status()
                if self._terminal:
                    return False, None
                frame = self._new_frame()
                if frame is not None:
                    return True, frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, None
                try:
                    self._notices.get(timeout=min(.04, remaining))
                except queue.Empty:
                    pass
                except (ValueError, OSError, EOFError):
                    return False, None
            return False, None

    def release(self):
        """Cancela incluso un open/read nativo bloqueado; puede repetirse."""
        with self._release_lock:
            if self._closed.is_set():
                return
            self._closed.set()
            self._stop.set()
            if self._process.pid is not None:
                self._process.join(timeout=.4)
                if self._process.is_alive():
                    self._process.terminate()
                    self._process.join(timeout=.5)
                if self._process.is_alive():
                    self._process.kill()
                    self._process.join(timeout=.5)
            with self._memory_lock:
                self._memory.close()
                try:
                    self._memory.unlink()
                except FileNotFoundError:
                    pass
            with self._status_lock:
                self._status.close()
            self._notices.cancel_join_thread()
            self._notices.close()
