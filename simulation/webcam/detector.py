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
        # Una impresión recortada con poco margen puede unir su contorno a una
        # sombra o al soporte. Se prueban otras aproximaciones de cuadriláteros;
        # se mantienen diccionario, identificación y corrección de bits.
        contour_parameters = cv2.aruco.DetectorParameters()
        contour_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        contour_parameters.polygonalApproxAccuracyRate = 0.08
        self.contour_detector = cv2.aruco.ArucoDetector(dictionary, contour_parameters)
        intermediate_parameters = cv2.aruco.DetectorParameters()
        intermediate_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        intermediate_parameters.polygonalApproxAccuracyRate = 0.06
        self.intermediate_detector = cv2.aruco.ArucoDetector(dictionary, intermediate_parameters)

    def process(self, frame: np.ndarray) -> Detection:
        started = time.monotonic()
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found: dict[int, np.ndarray] = {}
        attempts = ((self.detector, 1.0), (self.contour_detector, 1.0),
                    (self.intermediate_detector, 1.0),
                    (self.detector, 1.5), (self.detector, 3.0))
        for detector, scale in attempts:
            pixels = gray if scale == 1.0 else cv2.resize(
                gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC
            )
            corners, ids, _ = detector.detectMarkers(pixels)
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


def annotate_frame(frame: np.ndarray, detection: Detection, robot_pose: dict | None = None,
                   *, navigation_overlay: dict | None = None, hands=None) -> np.ndarray:
    """Copia visual: nunca modifica el fotograma usado por el detector."""
    result = frame.copy()
    for hand in hands or []:
        first, last = np.rint(np.asarray(hand["bbox_px"], dtype=float)).astype(int).reshape(2, 2)
        cv2.rectangle(result, tuple(first), tuple(last), (75, 165, 255), 2, cv2.LINE_AA)
        cv2.putText(result, "MANO", (int(first[0]), max(14, int(first[1]) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .42, (75, 165, 255), 1, cv2.LINE_AA)
        landmarks = np.asarray(hand.get("landmarks_px", []), dtype=float)
        if landmarks.shape == (21, 2) and np.all(np.isfinite(landmarks)):
            points = np.rint(landmarks).astype(int)
            for chain in ((0, 1, 2, 3, 4), (0, 5, 6, 7, 8), (5, 9, 10, 11, 12),
                          (9, 13, 14, 15, 16), (13, 17, 18, 19, 20), (0, 17)):
                cv2.polylines(result, [points[list(chain)].astype(np.int32)], False,
                              (115, 225, 180), 1, cv2.LINE_AA)
            for point in points:
                cv2.circle(result, tuple(point), 2, (75, 165, 255), -1, cv2.LINE_AA)
    if navigation_overlay:
        color = (205, 229, 108)
        path = navigation_overlay.get("path_px", [])
        if len(path) > 1:
            points = np.rint(np.asarray(path, dtype=float)).astype(np.int32)
            cv2.polylines(result, [points], False, color, 2, cv2.LINE_AA)
        hull = navigation_overlay.get("footprint_px", [])
        if len(hull) >= 3:
            points = np.rint(np.asarray(hull, dtype=float)).astype(np.int32)
            cv2.polylines(result, [points], True, color, 1, cv2.LINE_AA)
        target = navigation_overlay.get("steering_target_px")
        if robot_pose is not None and target is not None:
            start = tuple(np.rint(robot_pose["center_px"]).astype(int))
            end = tuple(np.rint(target).astype(int))
            cv2.line(result, start, end, color, 2, cv2.LINE_AA)
    if not detection.visible or detection.marker_corners is None or detection.center_px is None:
        return result
    corners = np.asarray(detection.marker_corners, dtype=float)
    points = np.rint(corners).astype(np.int32)
    cv2.polylines(result, [points], True, (110, 231, 52), 2, cv2.LINE_AA)
    if robot_pose is None and navigation_overlay is not None:
        # Existe calibración, pero este fotograma no permite proyectar su
        # centro. No sustituirlo por una cruz o flecha en el AprilTag trasero.
        return result
    center = np.asarray(detection.center_px)
    direction = (corners[1] + corners[2] - corners[0] - corners[3]) / 2
    direction /= np.linalg.norm(direction)
    length = max(35.0, float(np.linalg.norm(corners[1] - corners[0])) * 0.85)
    if robot_pose is not None:
        center = np.asarray(robot_pose["center_px"], dtype=float)
        direction = np.asarray(robot_pose["front_px"], dtype=float) - center
        norm = float(np.linalg.norm(direction))
        if norm > 0:
            direction /= norm
        cv2.circle(result, tuple(np.rint(center).astype(int)), 5, (255, 210, 85), -1, cv2.LINE_AA)
    # Una sola cruz de referencia: el centro calibrado cuando existe; el
    # centro del marcador solo se usa antes de calibrar el robot.
    cv2.drawMarker(result, tuple(np.rint(center).astype(int)), (255, 255, 255),
                   cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
    if robot_pose is not None:
        label_center = tuple(np.rint(center + np.array([9., 16.])).astype(int))
        cv2.putText(result, "CENTRO", label_center, cv2.FONT_HERSHEY_SIMPLEX, .35,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(result, "CENTRO", label_center, cv2.FONT_HERSHEY_SIMPLEX, .35,
                    (255, 255, 255), 1, cv2.LINE_AA)
    start = tuple(np.rint(center).astype(int))
    end = tuple(np.rint(center + length * direction).astype(int))
    cv2.arrowedLine(result, start, end, (255, 210, 85), 3, cv2.LINE_AA, tipLength=0.22)
    label = (max(4, min(result.shape[1] - 85, end[0] + 8)),
             max(22, min(result.shape[0] - 8, end[1] - 8)))
    cv2.putText(result, "FRENTE", label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(result, "FRENTE", label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 210, 85), 1, cv2.LINE_AA)
    return result
