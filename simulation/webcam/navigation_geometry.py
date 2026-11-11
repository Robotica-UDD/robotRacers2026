"""Geometría planar del robot a partir del AprilTag y dos clics de calibración.

La homografía utiliza el plano horizontal del marcador, no el plano del suelo.
Con marcadores pequeños se ajusta un plano afín con las cuatro esquinas:
su ruido subpíxel no permite calibrar la perspectiva de todo el suelo. Todas las
mediciones y comprobaciones de ruta comparten esta misma transformación.
El eje X canónico del tag apunta a su borde derecho y el eje Y a su borde
superior. Dos clics fijan el centro entre ruedas y el frente real en ese marco.
El consumidor debe comprobar que cámara y resolución coincidan con Calibration.

https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Integral, Real

import cv2
import numpy as np


SMALL_MARKER_EDGE_PX = 48.


def _number(value, label: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{label} debe ser un número finito.")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise ValueError(f"{label} debe ser un número finito.") from None
    if not math.isfinite(result):
        raise ValueError(f"{label} debe ser un número finito.")
    return result


def _point(value, label: str) -> np.ndarray:
    try:
        if len(value) != 2:
            raise ValueError()
        return np.array([_number(value[0], label), _number(value[1], label)], dtype=np.float64)
    except (TypeError, IndexError, KeyError):
        raise ValueError(f"{label} debe tener dos coordenadas finitas.") from None


def _size(value) -> float:
    size = _number(value, "El lado del marcador")
    if not .005 <= size <= .25:
        raise ValueError("El lado del marcador debe estar entre 5 y 250 mm.")
    return size


def _dimensions(width, height) -> tuple[int, int]:
    if any(isinstance(item, (bool, np.bool_)) or not isinstance(item, Integral)
           or not 1 <= int(item) <= 8192 for item in (width, height)):
        raise ValueError("La resolución de la cámara no es válida.")
    return int(width), int(height)


def _camera_id(value) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 512:
        raise ValueError("La identidad de la cámara no es válida.")
    return value


def _inside(point: np.ndarray, width: int, height: int, label: str) -> None:
    if not (0 <= point[0] < width and 0 <= point[1] < height):
        raise ValueError(f"{label} debe quedar dentro de la imagen.")


@dataclass(frozen=True)
class Calibration:
    center_tag_m: tuple[float, float]
    front_angle_rad: float
    camera_id: str
    width: int
    height: int
    marker_size_m: float = .04

    def __post_init__(self):
        center = _point(self.center_tag_m, "El centro del robot")
        if float(np.linalg.norm(center)) > .2 + 1e-10:
            raise ValueError("El centro seleccionado está demasiado lejos del marcador.")
        angle = _number(self.front_angle_rad, "La dirección del frente")
        width, height = _dimensions(self.width, self.height)
        object.__setattr__(self, "center_tag_m", (float(center[0]), float(center[1])))
        object.__setattr__(self, "front_angle_rad", (angle + math.pi) % (2 * math.pi) - math.pi)
        object.__setattr__(self, "camera_id", _camera_id(self.camera_id))
        object.__setattr__(self, "width", width)
        object.__setattr__(self, "height", height)
        object.__setattr__(self, "marker_size_m", _size(self.marker_size_m))

    def to_dict(self) -> dict:
        return {"center_tag_m": list(self.center_tag_m),
                "front_angle_rad": self.front_angle_rad, "camera_id": self.camera_id,
                "width": self.width, "height": self.height, "marker_size_m": self.marker_size_m}

    @classmethod
    def from_dict(cls, value: dict) -> "Calibration":
        expected = {"center_tag_m", "front_angle_rad", "camera_id", "width", "height", "marker_size_m"}
        if not isinstance(value, dict) or set(value) != expected:
            raise ValueError("El archivo de calibración no tiene el formato esperado.")
        return cls(**value)


def _homographies(corners, marker_size_m: float, width: int, height: int):
    try:
        points = np.asarray(corners)
    except (ValueError, TypeError):
        raise ValueError("Las esquinas del marcador no son válidas.") from None
    if points.shape != (4, 2) or points.dtype.kind not in "iuf":
        raise ValueError("El marcador necesita cuatro esquinas con coordenadas numéricas.")
    points = points.astype(np.float64)
    if not np.all(np.isfinite(points)):
        raise ValueError("Las esquinas del marcador deben ser finitas.")
    for point in points:
        _inside(point, width, height, "El marcador")
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    cross = edges[:, 0] * np.roll(edges, -1, axis=0)[:, 1] - edges[:, 1] * np.roll(edges, -1, axis=0)[:, 0]
    area = .5 * abs(float(np.sum(points[:, 0] * np.roll(points[:, 1], -1)
                                - points[:, 1] * np.roll(points[:, 0], -1))))
    if (np.any(lengths < 4.) or area < 16.
            or not (np.all(cross > 1e-6) or np.all(cross < -1e-6))):
        raise ValueError("El contorno del marcador es demasiado pequeño, degenerado o no convexo.")
    half = marker_size_m / 2
    # OpenCV: superior izquierda, superior derecha, inferior derecha, inferior izquierda.
    plane = np.float32([[-half, half], [half, half], [half, -half], [-half, -half]])
    try:
        # Un marcador pequeño no calibra la perspectiva de todo el suelo con
        # precisión subpíxel. Ajustar las cuatro esquinas por mínimos cuadrados
        # evita que una diferencia de décimas de píxel coloque un horizonte
        # artificial dentro de la imagen o agrande el mapa decenas de metros.
        design = np.column_stack((plane, np.ones(4)))
        coefficients = np.linalg.lstsq(design, points, rcond=None)[0]
        affine_inverse = np.eye(3)
        affine_inverse[:2] = coefficients.T
        affine_forward = np.linalg.inv(affine_inverse)
        small_marker = float(np.mean(lengths)) <= SMALL_MARKER_EDGE_PX
        if small_marker:
            return affine_forward, affine_inverse, points.mean(axis=0)

        image_to_tag = cv2.getPerspectiveTransform(points.astype(np.float32), plane)
        tag_to_image = np.linalg.inv(image_to_tag)
    except (cv2.error, np.linalg.LinAlgError):
        raise ValueError("No se pudo calcular el plano del marcador.") from None
    if not np.all(np.isfinite(image_to_tag)) or not np.all(np.isfinite(tag_to_image)):
        raise ValueError("El plano del marcador no es válido.")
    return image_to_tag, tag_to_image, points.mean(axis=0)


def _project(matrix: np.ndarray, point: np.ndarray, reference: np.ndarray) -> np.ndarray:
    transformed = matrix @ np.array([point[0], point[1], 1.])
    reference_w = float(matrix[2] @ np.array([reference[0], reference[1], 1.]))
    if (not np.all(np.isfinite(transformed)) or abs(float(transformed[2])) < 1e-12
            or reference_w * float(transformed[2]) <= 0):
        raise ValueError("El punto no tiene una proyección válida en el plano del marcador.")
    result = transformed[:2] / transformed[2]
    if not np.all(np.isfinite(result)):
        raise ValueError("El punto no tiene una proyección finita.")
    return result


def calibrate_robot(corners, center_px, front_px, *, camera_id: str, width: int,
                    height: int, marker_size_m: float = .04) -> Calibration:
    """Obtiene centro y frente reales sin asumir dónde se pegó el marcador."""
    width, height = _dimensions(width, height)
    camera_id = _camera_id(camera_id)
    size = _size(marker_size_m)
    center = _point(center_px, "El centro del robot")
    front = _point(front_px, "El punto delantero")
    _inside(center, width, height, "El centro del robot")
    _inside(front, width, height, "El punto delantero")
    image_to_tag, _, reference = _homographies(corners, size, width, height)
    center_tag = _project(image_to_tag, center, reference)
    front_tag = _project(image_to_tag, front, reference)
    direction = front_tag - center_tag
    distance = float(np.linalg.norm(direction))
    if distance < .01 - 1e-8 or distance > .25 + 1e-8:
        raise ValueError("Marca el frente entre 10 y 250 mm del centro, dentro del plano del marcador.")
    return Calibration(tuple(center_tag), math.atan2(float(direction[1]), float(direction[0])),
                       camera_id, width, height, size)


def observe_robot(corners, calibration: Calibration, target_px=None) -> dict:
    """Proyecta el centro y devuelve el destino relativo: X delante, Y izquierda.

    ``distance_m`` es distancia en el plano del marcador. El origen no es el
    promedio de las esquinas: usa la proyección del centro entre ruedas calibrado.
    """
    if not isinstance(calibration, Calibration):
        raise ValueError("Se necesita una calibración válida del robot.")
    width, height = calibration.width, calibration.height
    image_to_tag, tag_to_image, reference = _homographies(
        corners, calibration.marker_size_m, width, height)
    center_tag = np.asarray(calibration.center_tag_m, dtype=np.float64)
    theta = calibration.front_angle_rad
    direction = np.array([math.cos(theta), math.sin(theta)])
    center = _project(tag_to_image, center_tag, np.zeros(2))
    front = _project(tag_to_image, center_tag + .02 * direction, np.zeros(2))
    _inside(center, width, height, "El centro del robot")
    _inside(front, width, height, "El frente del robot")
    delta_px = front - center
    if float(np.linalg.norm(delta_px)) < 1e-6:
        raise ValueError("No se puede resolver la dirección del robot en la imagen.")
    result = {"center_px": center.tolist(), "front_px": front.tolist(),
              "heading_deg": math.degrees(math.atan2(-float(delta_px[1]), float(delta_px[0]))) % 360.,
              "distance_m": None, "bearing_rad": None, "target_local_m": None}
    if target_px is not None:
        target = _point(target_px, "El destino")
        _inside(target, width, height, "El destino")
        delta_tag = _project(image_to_tag, target, reference) - center_tag
        c, s = math.cos(theta), math.sin(theta)
        local = np.array([[c, s], [-s, c]]) @ delta_tag
        distance = float(np.linalg.norm(local))
        result.update(distance_m=distance, bearing_rad=math.atan2(float(local[1]), float(local[0])),
                      target_local_m=local.tolist())
    return result
