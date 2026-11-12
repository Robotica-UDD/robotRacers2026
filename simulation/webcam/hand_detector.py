"""MediaPipe Hand Landmarker local, separado de captura y control de ruedas.

El modo VIDEO conserva seguimiento entre imágenes. No se filtra por altura,
profundidad ni tamaño aparente; solo por los umbrales propios del modelo.
"""
from __future__ import annotations

import copy
import math
from numbers import Integral, Real
from pathlib import Path
import threading
import time
from typing import Callable

import cv2
import numpy as np


MODEL_NAME = "MediaPipe Hand Landmarker"
MODEL_DIR = Path(__file__).resolve().parents[1] / ".runtime" / "hand-models"
MODEL_FILE = "hand_landmarker.task"


def _frame(frame):
    if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or
            frame.ndim != 3 or frame.shape[2] != 3 or
            not 1 <= frame.shape[0] <= 8192 or not 1 <= frame.shape[1] <= 8192):
        raise ValueError("La detección de manos necesita una imagen BGR de 8 bits.")
    return frame


class _MediaPipeBackend:
    def __init__(self, model, *, num_hands, confidence, presence_confidence, tracking_confidence):
        import mediapipe as mp
        self.mp = mp
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model),
                                              delegate=mp.tasks.BaseOptions.Delegate.CPU),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=num_hands, min_hand_detection_confidence=confidence,
            min_hand_presence_confidence=presence_confidence,
            min_tracking_confidence=tracking_confidence)
        self.landmarker = mp.tasks.vision.HandLandmarker.create_from_options(options)

    def detect(self, rgb, timestamp_ms):
        image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        return self.landmarker.detect_for_video(image, timestamp_ms)

    def close(self):
        self.landmarker.close()


class HandDetector:
    """Cajas y 21 puntos de manos con seguimiento temporal de MediaPipe.

    MediaPipe no publica el score de presencia por mano; los resultados ya
    aprobaron min_hand_detection/presence_confidence. Por ello confidence=1
    significa aceptación del modelo, nunca una probabilidad calibrada. La
    confianza de izquierda/derecha se mantiene por separado.
    """

    def __init__(self, model_dir=None, *, confidence=.35, presence_confidence=.35,
                 tracking_confidence=.35, num_hands=4, backend_factory=None,
                 clock=time.monotonic):
        for value in (confidence, presence_confidence, tracking_confidence):
            if (isinstance(value, bool) or not isinstance(value, Real) or
                    not math.isfinite(value) or not 0 < value < 1):
                raise ValueError("Los umbrales de detección deben estar entre 0 y 1.")
        if type(num_hands) is not int or not 1 <= num_hands <= 8:
            raise ValueError("El número de manos debe estar entre 1 y 8.")
        directory = MODEL_DIR if model_dir is None else Path(model_dir)
        model = directory / MODEL_FILE
        if not model.is_file():
            raise FileNotFoundError("Falta el modelo MediaPipe de manos. Ejecuta python -m "
                                    "simulation.tools.download_hand_model.")
        factory = backend_factory or _MediaPipeBackend
        self._backend = factory(model, num_hands=num_hands, confidence=float(confidence),
            presence_confidence=float(presence_confidence), tracking_confidence=float(tracking_confidence))
        self._clock, self._timestamp_ms = clock, -1

    def process(self, frame):
        image = _frame(frame)
        height, width = image.shape[:2]
        # VIDEO exige tiempos estrictamente crecientes, incluso al procesar
        # dos imágenes en el mismo milisegundo en una prueba o tras reconexión.
        stamp = max(self._timestamp_ms + 1, int(self._clock() * 1000.))
        self._timestamp_ms = stamp
        result = self._backend.detect(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), stamp)
        hands = []
        for index, landmarks in enumerate(result.hand_landmarks):
            if len(landmarks) != 21:
                continue
            points = np.asarray([(p.x * width, p.y * height) for p in landmarks], dtype=float)
            if not np.all(np.isfinite(points)):
                continue
            low = np.maximum([0., 0.], np.min(points, axis=0))
            high = np.minimum([float(width), float(height)], np.max(points, axis=0))
            if np.any(high <= low):
                continue
            category = (result.handedness[index][0] if index < len(result.handedness)
                        and result.handedness[index] else None)
            palm = np.mean(points[[0, 5, 9, 13, 17]], axis=0)
            hands.append({"bbox_px": [*low.tolist(), *high.tolist()],
                "confidence": 1., "confidence_kind": "model_threshold_acceptance",
                "landmarks_px": points.tolist(), "palm_px": palm.tolist(),
                "handedness": getattr(category, "category_name", None),
                "handedness_score": float(category.score) if category is not None else None})
        return hands

    def close(self):
        self._backend.close()


class AsyncHandDetector:
    """Un trabajador, un fotograma pendiente y ningún bloqueo de la captura.

    ``captured_at`` usa el mismo reloj monotónico que la captura. La antigüedad
    se refiere a la imagen original, no al momento en que terminó la red.
    """

    def __init__(self, detector_factory: Callable | None = None, *, max_fps=20.,
                 clock=time.monotonic):
        if (isinstance(max_fps, bool) or not isinstance(max_fps, Real) or
                not math.isfinite(max_fps) or not 0 < max_fps <= 60):
            raise ValueError("La frecuencia de detección debe ser positiva y de hasta 60 imágenes/s.")
        self._factory = detector_factory or HandDetector
        self._clock, self._interval = clock, 1. / max_fps
        self._condition = threading.Condition()
        self._pending = None
        self._closed = False
        self._last_submitted = -1
        self._state = {"status": "loading", "ready": False, "model": MODEL_NAME,
                       "hands": [], "sequence": None, "captured_at": None,
                       "processing_ms": None, "frame_width": None, "frame_height": None,
                       "error": None}
        self._worker = threading.Thread(target=self._run, name="mediapipe-manos", daemon=True)
        self._worker.start()

    def submit(self, frame, sequence, captured_at):
        image = _frame(frame)
        if isinstance(sequence, bool) or not isinstance(sequence, Integral) or sequence < 0:
            raise ValueError("El fotograma necesita una secuencia entera válida.")
        if (isinstance(captured_at, bool) or not isinstance(captured_at, Real) or
                not math.isfinite(captured_at) or captured_at < 0):
            raise ValueError("El fotograma necesita una marca temporal válida.")
        with self._condition:
            if self._closed or self._state["status"] in {"missing_model", "error"} or sequence <= self._last_submitted:
                return False
            self._last_submitted = int(sequence)
            self._pending = (image.copy(), int(sequence), float(captured_at))
            self._condition.notify()
            return True

    def state(self):
        with self._condition:
            state = copy.deepcopy(self._state)
        stamp = state["captured_at"]
        state["age_ms"] = None if stamp is None else round(max(0., self._clock() - stamp) * 1000., 1)
        return state

    def close(self):
        with self._condition:
            self._closed = True
            self._pending = None
            self._state.update(status="closed", ready=False, hands=[])
            self._condition.notify_all()
        if threading.current_thread() is not self._worker:
            self._worker.join(1.)

    def _error(self, status, message):
        with self._condition:
            if not self._closed:
                self._state.update(status=status, ready=False, hands=[], error=message)

    def _run(self):
        try:
            detector = self._factory()
        except FileNotFoundError as exc:
            self._error("missing_model", str(exc))
            return
        except Exception:
            self._error("error", "No se pudo cargar el modelo local de manos.")
            return
        try:
            self._run_detector(detector)
        finally:
            close = getattr(detector, "close", None)
            if close is not None:
                close()

    def _run_detector(self, detector):
        with self._condition:
            if self._closed:
                return
            self._state.update(status="ready", ready=True)
        next_inference = 0.
        while True:
            with self._condition:
                while True:
                    if self._closed:
                        return
                    delay = next_inference - self._clock()
                    if self._pending is not None and delay <= 0:
                        image, sequence, captured_at = self._pending
                        self._pending = None
                        break
                    self._condition.wait(None if self._pending is None else delay)
            started = self._clock()
            try:
                hands = detector.process(image)
            except Exception:
                self._error("error", "La inferencia de manos falló. Reinicia la detección para reintentar.")
                return
            elapsed = max(0., self._clock() - started)
            next_inference = started + self._interval
            with self._condition:
                if self._closed:
                    return
                self._state.update(status="ready", ready=True, hands=hands, sequence=sequence,
                                   captured_at=captured_at, processing_ms=round(elapsed * 1000., 2),
                                   frame_width=image.shape[1], frame_height=image.shape[0], error=None)
