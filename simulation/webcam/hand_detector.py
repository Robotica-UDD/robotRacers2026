"""Manos con YOLOv4-tiny Cross-Hands, usando OpenCV y archivos locales.

La inferencia vive fuera del hilo de cámara y del controlador del robot. No
descarga recursos ni envía imágenes a otros servicios. El modelo se instala con
``python -m simulation.tools.download_hand_model``.
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


MODEL_NAME = "YOLOv4-tiny Cross-Hands"
MODEL_DIR = Path(__file__).resolve().parents[1] / ".runtime" / "hand-models"
CONFIG_NAME = "cross-hands-yolov4-tiny.cfg"
WEIGHTS_NAME = "cross-hands-yolov4-tiny.weights"


def _frame(frame):
    if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or
            frame.ndim != 3 or frame.shape[2] != 3 or
            not 1 <= frame.shape[0] <= 8192 or not 1 <= frame.shape[1] <= 8192):
        raise ValueError("La detección de manos necesita una imagen BGR de 8 bits.")
    return frame


class HandDetector:
    """Detector de una sola clase (mano), con cajas en píxeles de la imagen.

    La salida Region de OpenCV ya entrega la confianza conjunta por clase;
    como en la implementación del autor, se usa ``row[5]`` directamente.
    https://github.com/cansik/yolo-hand-detection/blob/master/yolo.py
    """

    def __init__(self, model_dir=None, *, input_size=416, confidence=.5,
                 nms_threshold=.3, net_factory=None):
        if type(input_size) is not int or input_size not in (256, 320, 416):
            raise ValueError("El tamaño de entrada debe ser 256, 320 o 416.")
        for value in (confidence, nms_threshold):
            if isinstance(value, bool) or not isinstance(value, Real) or not 0 < value < 1:
                raise ValueError("Los umbrales de detección deben estar entre 0 y 1.")
        directory = MODEL_DIR if model_dir is None else Path(model_dir)
        config, weights = directory / CONFIG_NAME, directory / WEIGHTS_NAME
        if not config.is_file() or not weights.is_file():
            raise FileNotFoundError("Falta el modelo de manos. Ejecuta python -m "
                                    "simulation.tools.download_hand_model.")
        self.input_size, self.confidence, self.nms_threshold = input_size, float(confidence), float(nms_threshold)
        factory = net_factory or cv2.dnn.readNetFromDarknet
        self.net = factory(str(config), str(weights))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.output_names = self.net.getUnconnectedOutLayersNames()

    def process(self, frame):
        image = _frame(frame)
        height, width = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1 / 255.,
                                    (self.input_size, self.input_size), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_names)
        boxes, confidences = [], []
        for output in outputs:
            for row in output:
                if len(row) < 6 or not np.all(np.isfinite(row[:6])):
                    continue
                score = float(row[5])
                if not self.confidence <= score <= 1.:
                    continue
                cx, cy, box_width, box_height = row[:4] * [width, height, width, height]
                if box_width <= 0 or box_height <= 0:
                    continue
                x1, y1 = max(0., float(cx - box_width / 2)), max(0., float(cy - box_height / 2))
                x2, y2 = min(float(width), float(cx + box_width / 2)), min(float(height), float(cy + box_height / 2))
                if x2 <= x1 or y2 <= y1:
                    continue
                boxes.append([x1, y1, x2 - x1, y2 - y1])
                confidences.append(score)
        if not boxes:
            return []
        kept = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence, self.nms_threshold)
        hands = []
        for index in np.asarray(kept).reshape(-1):
            x, y, box_width, box_height = boxes[int(index)]
            hands.append({"bbox_px": [x, y, x + box_width, y + box_height],
                          "confidence": confidences[int(index)]})
        return sorted(hands, key=lambda hand: hand["confidence"], reverse=True)


class AsyncHandDetector:
    """Un trabajador, un fotograma pendiente y ningún bloqueo de la captura.

    ``captured_at`` usa el mismo reloj monotónico que la captura. La antigüedad
    se refiere a la imagen original, no al momento en que terminó la red.
    """

    def __init__(self, detector_factory: Callable | None = None, *, max_fps=10.,
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
        self._worker = threading.Thread(target=self._run, name="yolo-manos", daemon=True)
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
