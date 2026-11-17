"""Huella compartida del ensamblaje RevQ, en milímetros.

El origen es el centro del eje de las ruedas; X apunta al frente e Y a la
izquierda. Es el mismo centro que se selecciona en la calibración de la webcam,
no el centro del AprilTag ni el centro de masa.

Procedencia: proyección del cuerpo RevQ, desplazado +13,15 mm en X, y ruedas
actualizadas a las medidas comunicadas por el usuario el 17-11-2026: diámetro
33 mm, ancho 7 mm y ancho exterior total 170 mm. La separación entre centros
es 170 - 7 = 163 mm. Cada rueda barre X entre ±16,5 mm y se centra en Y =
±81,5 mm. Se conserva el portador simulado de 52 × 52 mm centrado en
(-26,85, 0) mm; el planificador añade la tarjeta física en su posición calibrada.

La huella evita combinar los extremos independientes de la caja delimitadora:
la esquina (83,15, 85) mm está vacía. El radio se deriva de la huella y se
redondea hacia arriba a 0,1 mm; no incorpora un margen adicional de separación.

Es una reconstrucción dimensional, no un escaneado del robot ensamblado.
Batería, cables, patín y soporte del marcador son aproximados. No se añaden los
tacos laterales del modelo de rueda anterior. La posición real del marcador
depende de su calibración y no
debe sustituirse por la posición simulada al proyectar el robot en la imagen.
Este módulo no lee archivos de ejecución ni altera la calibración.
"""
from __future__ import annotations

import math
from numbers import Real


WHEEL_DIAMETER_MM = 33.0
WHEEL_WIDTH_MM = 7.0
TRACK_WIDTH_MM = 163.0
HALF_TRACK_MM = TRACK_WIDTH_MM / 2.0
_WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2.0
_WHEEL_OUTER_Y_MM = HALF_TRACK_MM + WHEEL_WIDTH_MM / 2.0

# Orden antihorario: unión convexa del cuerpo, portador simulado y barrido de
# las ruedas. Las posiciones del cuerpo respecto al eje no cambian de montaje.
ROBOT_FOOTPRINT_MM: tuple[tuple[float, float], ...] = (
    (-52.85, -26.0),
    (-_WHEEL_RADIUS_MM, -_WHEEL_OUTER_Y_MM),
    (_WHEEL_RADIUS_MM, -_WHEEL_OUTER_Y_MM),
    (70.15, -54.5),
    (83.15, -32.0),
    (83.15, 32.0),
    (70.15, 54.5),
    (_WHEEL_RADIUS_MM, _WHEEL_OUTER_Y_MM),
    (-_WHEEL_RADIUS_MM, _WHEEL_OUTER_Y_MM),
    (-52.85, 26.0),
)

ROBOT_RADIUS_MM: float = math.ceil(
    max(math.hypot(x, y) for x, y in ROBOT_FOOTPRINT_MM) * 10.0
) / 10.0


def robot_radius_mm() -> float:
    """Radio del barrido completo del robot sobre su eje, sin margen extra."""
    return ROBOT_RADIUS_MM


def point_under_robot(local_point_m) -> bool:
    """Un destino queda alcanzado al entrar en el contorno del robot.

    Coordenadas relativas al eje de las ruedas, X hacia delante e Y hacia la
    izquierda, como ``observe_robot()["target_local_m"]``. Incluye el borde;
    no añade el círculo de barrido ni el soporte desplazado del marcador.
    """
    try:
        if len(local_point_m) != 2 or any(isinstance(v, bool) or not isinstance(v, Real)
                                         or not math.isfinite(v) for v in local_point_m):
            return False
        x, y = (float(value) * 1000. for value in local_point_m)
    except (TypeError, ValueError, OverflowError):
        return False
    signs = []
    for first, last in zip(ROBOT_FOOTPRINT_MM, ROBOT_FOOTPRINT_MM[1:] + ROBOT_FOOTPRINT_MM[:1]):
        signs.append((last[0] - first[0]) * (y - first[1])
                     - (last[1] - first[1]) * (x - first[0]))
    return all(value >= -1e-6 for value in signs) or all(value <= 1e-6 for value in signs)
