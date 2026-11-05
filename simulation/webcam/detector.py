"""Detección del marcador físico, sin calibración ni coordenadas métricas."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    visible: bool
    center_px: tuple[float, float] | None = None
    heading_deg: float | None = None
    marker_corners: list[list[float]] | None = None
    processing_ms: float = 0.0
    markers: int = 0


class AprilTagDetector:
    """AprilTag 36h11, ID 0. El borde derecho de la tarjeta indica el frente.

    El ángulo corresponde a la imagen sin espejo: 0° derecha, 90° arriba.
    No representa por sí solo un ángulo del suelo ni el centro del robot.
    """

    marker_id = 0

    def __init__(self):
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def process(self, frame: np.ndarray) -> Detection:
        started = time.monotonic()
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found: dict[int, np.ndarray] = {}
        for scale in (1.0, 1.5, 3.0):
            pixels = gray if scale == 1.0 else cv2.resize(
                gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC
            )
            corners, ids, _ = self.detector.detectMarkers(pixels)
            if ids is not None:
                for identifier, corner in zip(ids.flatten(), corners):
                    found.setdefault(int(identifier), np.asarray(corner, dtype=float).reshape(4, 2) / scale)
            if self.marker_id in found:
                break
        duration = (time.monotonic() - started) * 1000
        if self.marker_id not in found:
            return Detection(False, processing_ms=duration, markers=len(found))
        corners = found[self.marker_id]
        # OpenCV devuelve las esquinas en el orden del marcador canónico:
        # superior izquierda, superior derecha, inferior derecha, inferior izquierda.
        # La flecha impresa queda a la derecha, independiente de su rotación en cámara.
        center = corners.mean(axis=0)
        direction = (corners[1] + corners[2] - corners[0] - corners[3]) / 2
        if float(np.linalg.norm(direction)) < 1.0:
            return Detection(False, processing_ms=duration, markers=len(found))
        heading = math.degrees(math.atan2(-float(direction[1]), float(direction[0]))) % 360.0
        return Detection(True, (float(center[0]), float(center[1])), heading,
                         corners.tolist(), duration, len(found))


def annotate_frame(frame: np.ndarray, detection: Detection) -> np.ndarray:
    """Copia visual: nunca modifica el fotograma usado por el detector."""
    result = frame.copy()
    if not detection.visible or detection.marker_corners is None or detection.center_px is None:
        return result
    corners = np.asarray(detection.marker_corners, dtype=float)
    points = np.rint(corners).astype(np.int32)
    cv2.polylines(result, [points], True, (110, 231, 52), 2, cv2.LINE_AA)
    center = np.asarray(detection.center_px)
    direction = (corners[1] + corners[2] - corners[0] - corners[3]) / 2
    direction /= np.linalg.norm(direction)
    length = max(35.0, float(np.linalg.norm(corners[1] - corners[0])) * 0.85)
    start = tuple(np.rint(center).astype(int))
    end = tuple(np.rint(center + length * direction).astype(int))
    cv2.drawMarker(result, start, (255, 255, 255), cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
    cv2.arrowedLine(result, start, end, (255, 210, 85), 3, cv2.LINE_AA, tipLength=0.22)
    label = (max(4, min(result.shape[1] - 85, end[0] + 8)),
             max(22, min(result.shape[0] - 8, end[1] - 8)))
    cv2.putText(result, "FRENTE", label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(result, "FRENTE", label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 210, 85), 1, cv2.LINE_AA)
    return result
