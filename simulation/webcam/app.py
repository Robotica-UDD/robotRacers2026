"""Servicio local de webcam y detección. No importa ni conecta controladores."""
from __future__ import annotations

import asyncio
from collections import deque
from contextlib import asynccontextmanager
import logging
from pathlib import Path
import sys
import threading
import time
from typing import Callable

import cv2
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
import numpy as np
from pydantic import BaseModel, Field

from .detector import AprilTagDetector, Detection, annotate_frame
from .devices import discover_cameras, _manual_options
from .robot_link import RobotLink, make_robot_router

LOGGER = logging.getLogger(__name__)
STATIC = Path(__file__).parent / "static"
ASSETS = Path(__file__).parent / "assets"


def open_camera(index: int, camera_id: str | None = None):
    """Una sesión nativa nueva evita conservar dispositivos USB desconectados."""
    if sys.platform == "darwin":
        from .process_capture import ProcessCapture
        return ProcessCapture(index, camera_id=camera_id)
    return cv2.VideoCapture(index)


class CaptureSession:
    """Un único lector de cámara; conserva solo el fotograma más reciente.

    Abrir, leer y liberar pertenecen al mismo hilo. Si el controlador bloquea
    read(), Detener invalida la detección inmediatamente y espera un tiempo
    acotado; no se abre otra cámara mientras el lector anterior siga activo.
    """

    def __init__(self, capture_factory: Callable | None = None, detector=None,
                 clock: Callable = time.monotonic, stale_after_s: float = 0.75,
                 startup_timeout_s: float = 5.0):
        self.capture_factory = capture_factory or open_camera
        self._uses_default_capture = capture_factory is None
        self.detector = detector or AprilTagDetector()
        self.clock = clock
        self.stale_after_s = stale_after_s
        self.startup_timeout_s = startup_timeout_s
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._active_capture = None
        self._stop_event = threading.Event()
        self._running = False
        self._camera_index: int | None = None
        self._camera_id: str | None = None
        self._camera_label: str | None = None
        self._opening = False
        self._message = "Cámara detenida."
        self._sequence = 0
        self._version = 0
        self._width = self._height = 0
        self._fps = 0.0
        self._last_frame_at: float | None = None
        self._detection = Detection(False)
        self._jpeg: bytes | None = None
        self._placeholder_cache: dict[str, bytes] = {}

    def start(self, camera_index: int = 0, camera_id: str | None = None,
              camera_label: str | None = None) -> dict:
        if isinstance(camera_index, bool) or not isinstance(camera_index, int) or not 0 <= camera_index <= 9:
            raise ValueError("El índice de cámara debe estar entre 0 y 9.")
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("La cámara anterior sigue activa. Detén la captura y espera a que se cierre.")
            self._stop_event = threading.Event()
            self._running = True
            self._camera_index = camera_index
            self._camera_id = camera_id
            self._camera_label = camera_label
            self._opening = True
            self._message = "Abriendo cámara…"
            self._last_frame_at = None
            self._jpeg = None
            self._detection = Detection(False)
            self._width = self._height = 0
            self._fps = 0.0
            self._sequence = 0
            self._version += 1
            self._thread = threading.Thread(target=self._capture, args=(camera_index, self._stop_event),
                                            name="robotracers-webcam", daemon=True)
            self._thread.start()
        return self.state()

    def stop(self) -> dict:
        with self._lock:
            self._stop_event.set()
            self._running = False
            self._opening = False
            self._detection = Detection(False)
            self._jpeg = None
            self._last_frame_at = None
            self._fps = 0.0
            self._message = "Cámara detenida."
            self._version += 1
            worker = self._thread
            active_capture = self._active_capture
        # Un controlador aislado se puede interrumpir incluso si abrir o leer
        # queda bloqueado. Las capturas nativas inyectadas siguen en su hilo.
        if active_capture is not None and getattr(active_capture, "interruptible", False):
            active_capture.release()
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2.0)
            if worker.is_alive():
                with self._lock:
                    self._message = "Cerrando cámara; esperando al controlador."
        return self.state()

    def _capture(self, index: int, stop: threading.Event):
        capture = None
        timestamps: deque[float] = deque(maxlen=60)
        try:
            capture = (open_camera(index, camera_id=self._camera_id) if self._uses_default_capture
                       else self.capture_factory(index))
            with self._lock:
                self._active_capture = capture
            if stop.is_set():
                return
            if not capture.isOpened():
                raise RuntimeError(getattr(capture, "error_message", None) or
                                   "No se pudo abrir la cámara. Revisa el índice y el permiso de Cámara en macOS.")
            if hasattr(capture, "set"):
                for prop, value in ((cv2.CAP_PROP_FRAME_WIDTH, 1280), (cv2.CAP_PROP_FRAME_HEIGHT, 720),
                                    (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_BUFFERSIZE, 1)):
                    capture.set(prop, value)
            # Algunas webcams USB necesitan varios intentos tras negociar el
            # formato. Un primer read() vacío no implica una desconexión.
            startup_deadline = time.monotonic() + self.startup_timeout_s
            received_first_frame = False
            while not stop.is_set():
                okay, pixels = capture.read()
                received = self.clock()
                if stop.is_set():
                    break
                if not okay or pixels is None or pixels.size == 0:
                    if getattr(capture, "error_message", None):
                        raise RuntimeError(capture.error_message)
                    if not received_first_frame and time.monotonic() < startup_deadline:
                        with self._lock:
                            if not stop.is_set():
                                self._message = "Preparando la cámara; esperando la primera imagen…"
                        stop.wait(0.05)
                        continue
                    raise RuntimeError("Cámara desconectada o sin imágenes. Detén e inicia la captura para reintentar.")
                received_first_frame = True
                # El proceso de captura marca la recepción antes del traslado;
                # no se presenta un fotograma antiguo como recién capturado.
                capture_stamp = getattr(capture, "frame_capture_stamp", None)
                if isinstance(capture_stamp, (int, float)):
                    received = float(capture_stamp)
                if pixels.ndim == 2:
                    pixels = cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
                elif pixels.shape[2] == 4:
                    pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
                detection = self.detector.process(pixels)
                display = annotate_frame(pixels, detection)
                encoded, jpeg = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not encoded:
                    raise RuntimeError("No se pudo preparar la imagen de cámara.")
                timestamps.append(received)
                elapsed = timestamps[-1] - timestamps[0]
                fps = (len(timestamps) - 1) / elapsed if elapsed > 0 else 0.0
                with self._lock:
                    if stop.is_set():
                        break
                    self._sequence += 1
                    self._opening = False
                    self._version += 1
                    self._detection = detection
                    self._last_frame_at = received
                    self._width, self._height = pixels.shape[1], pixels.shape[0]
                    self._fps = fps
                    self._jpeg = jpeg.tobytes()
                    self._message = "Robot detectado." if detection.visible else "Buscando AprilTag 0…"
        except Exception as exc:
            LOGGER.warning("Captura detenida: %s", exc)
            with self._lock:
                if not stop.is_set():
                    self._message = str(exc) if isinstance(exc, RuntimeError) else "Error de captura. Revisa la cámara y vuelve a iniciar."
        finally:
            # Invalidar antes de liberar: algunos controladores tardan en cerrar.
            with self._lock:
                self._running = False
                self._opening = False
                self._detection = Detection(False)
                self._jpeg = None
                self._fps = 0.0
                self._version += 1
            if capture is not None:
                try:
                    capture.release()
                except Exception:
                    LOGGER.exception("No se pudo liberar la cámara")
            with self._lock:
                if self._active_capture is capture:
                    self._active_capture = None

    def _state_locked(self) -> dict:
        age = None if self._last_frame_at is None else max(0.0, self.clock() - self._last_frame_at)
        stale = age is not None and age > self.stale_after_s
        visible = bool(self._running and not self._stop_event.is_set() and not stale and self._detection.visible)
        message = "Sin fotograma reciente. Esperando cámara…" if self._running and stale else self._message
        return {
            "running": self._running, "visible": visible, "message": message,
            "opening": self._opening,
            "camera_index": self._camera_index, "frame_width": self._width, "frame_height": self._height,
            "camera_id": self._camera_id, "camera_label": self._camera_label,
            "fps": round(self._fps, 2) if self._running and not stale else 0.0,
            "processing_ms": round(self._detection.processing_ms, 2), "sequence": self._sequence,
            "center_px": list(self._detection.center_px) if visible else None,
            "heading_deg": self._detection.heading_deg if visible else None,
            "marker_corners": self._detection.marker_corners if visible else None,
            "marker_id": 0, "frame_age_ms": None if age is None else round(age * 1000, 1),
            "heading_convention": "0° derecha, 90° arriba; ángulo en la imagen sin espejo.",
        }

    def state(self) -> dict:
        with self._lock:
            return self._state_locked()

    def _placeholder(self, message: str) -> bytes:
        # Texto principal en la interfaz; OpenCV usa aquí solo caracteres ASCII.
        label = ("Sin imagen reciente" if message.startswith("Sin fotograma") else
                 "Abriendo camara" if message.startswith("Abriendo") else "Camara detenida")
        with self._lock:
            cached = self._placeholder_cache.get(label)
        if cached is not None:
            return cached
        pixels = np.full((720, 1280, 3), (24, 20, 17), np.uint8)
        cv2.putText(pixels, label, (430, 365), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (185, 190, 191), 1, cv2.LINE_AA)
        _, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 80])
        data = encoded.tobytes()
        with self._lock:
            self._placeholder_cache[label] = data
        return data

    def frame(self) -> tuple[tuple[int, bool], bytes]:
        with self._lock:
            status = self._state_locked()
            stale = self._last_frame_at is not None and self.clock() - self._last_frame_at > self.stale_after_s
            jpeg = self._jpeg if self._running and not stale else None
            token = (self._version, stale)
        return token, jpeg if jpeg is not None else self._placeholder(status["message"])


class StartRequest(BaseModel):
    camera_index: int = Field(default=0, ge=0, le=9, strict=True)
    camera_id: str | None = Field(default=None, min_length=1, max_length=512)


def create_app(capture_factory: Callable | None = None, detector=None,
               clock: Callable = time.monotonic, stale_after_s: float = 0.75,
               camera_discovery: Callable | None = None, robot_link=None) -> FastAPI:
    camera = CaptureSession(capture_factory, detector, clock, stale_after_s)
    # Una cámara inyectada no corresponde al hardware del equipo de pruebas.
    choices = camera_discovery or (discover_cameras if capture_factory is None
                                  else lambda force=False: _manual_options())
    robot = robot_link if robot_link is not None else RobotLink(
        runtime_path=Path(__file__).resolve().parents[1] / ".runtime/robot-link.json"
        if capture_factory is None else None)

    @asynccontextmanager
    async def lifespan(application: FastAPI):
        yield
        await asyncio.to_thread(camera.stop)
        await asyncio.to_thread(robot.close)

    application = FastAPI(title="RobotRacers · Cámara", lifespan=lifespan)
    application.state.camera = camera
    application.state.robot = robot
    application.include_router(make_robot_router(robot))

    @application.middleware("http")
    async def uncached(request: Request, call_next):
        response = await call_next(request)
        response.headers["Cache-Control"] = "no-store"
        return response

    @application.get("/")
    def index():
        return FileResponse(STATIC / "index.html")

    @application.get("/api/state")
    def state():
        return camera.state()

    @application.get("/api/cameras")
    def cameras(refresh: bool = False):
        return choices(force=refresh)

    @application.post("/api/start")
    def start(payload: StartRequest):
        catalog = choices(force=True)
        selected = None
        if payload.camera_id and not payload.camera_id.startswith("index:"):
            if catalog.get("enumerated"):
                selected = next((item for item in catalog["options"]
                                 if item.get("id") == payload.camera_id), None)
            if selected is None:
                raise HTTPException(409, "La cámara seleccionada ya no está disponible. Actualiza la lista y vuelve a seleccionarla.")
        else:
            selected = next((item for item in catalog.get("options", [])
                             if item["index"] == payload.camera_index), None)
        index = selected["index"] if selected is not None else payload.camera_index
        if not 0 <= index <= 9:
            raise HTTPException(422, "El índice de cámara debe estar entre 0 y 9.")
        try:
            return camera.start(index, selected.get("id") if selected else None,
                                selected.get("label") if selected else None)
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc

    @application.post("/api/stop")
    def stop():
        return camera.stop()

    @application.get("/api/stream")
    async def stream(request: Request):
        async def latest_frames():
            previous = None
            while not await request.is_disconnected():
                token, jpeg = camera.frame()
                if token != previous:
                    previous = token
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode()
                           + b"\r\n\r\n" + jpeg + b"\r\n")
                await asyncio.sleep(1 / 30)
        return StreamingResponse(latest_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    application.mount("/assets", StaticFiles(directory=ASSETS, check_dir=False), name="assets")
    return application


app = create_app()
