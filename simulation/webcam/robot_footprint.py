"""Huella compartida del ensamblaje RevQ, en milímetros.

El origen es el centro del eje de las ruedas; X apunta al frente e Y a la
izquierda. Es el mismo centro que se selecciona en la calibración de la webcam,
no el centro del AprilTag ni el centro de masa.

Procedencia: proyección convexa de ``assets/robot/body.obj`` y ``wheel.obj``;
sus vértices coinciden con los archivos ``.inc.wbt``. ``tools/build_world.py``
desplaza el cuerpo +13,15 mm en X y coloca las ruedas en (0, ±95, 11,5) mm.
Las ruedas ya tienen eje local Y; la rotación X de -pi/2 pertenece únicamente
al cilindro de colisión de Webots. Se incluye su barrido completo alrededor de
Y: cada vértice (x, y, z) ocupa X entre ±hypot(x, z). También se incluye el
portador simulado de 52 × 52 mm, centrado en (-26,85, 0) mm.

La huella evita combinar los extremos independientes de la caja delimitadora:
la esquina (83,15, 115,9) mm está vacía. El radio se deriva de la huella y se
redondea hacia arriba a 0,1 mm; no incorpora un margen adicional de separación.

Es una reconstrucción dimensional, no una medición del robot ensamblado. La
separación de ruedas se infirió de los 230 mm exteriores nominales; los tacos
modelados añaden 0,9 mm por lado. Batería, cables, patín y soporte del marcador
son aproximados. La posición real del marcador depende de su calibración y no
debe sustituirse por la posición simulada al proyectar el robot en la imagen.
Este módulo no lee archivos de ejecución ni altera la calibración.
"""
from __future__ import annotations

import math
from numbers import Real


# En el extremo lateral, el taco tiene (x, z) = (4,29282, 2,5) mm.
# Al girar la rueda, su máxima proyección longitudinal es hypot(x, z).
_WHEEL_SIDE_SWEEP_X_MM = math.hypot(4.29282, 2.5)

# Orden antihorario. La precisión de las coordenadas procede de las mallas
# serializadas; 11,500001 cubre el extremo radial a Y = ±115 mm.
ROBOT_FOOTPRINT_MM: tuple[tuple[float, float], ...] = (
    (-52.85, -26.0),
    (-11.500001, -115.0),
    (-_WHEEL_SIDE_SWEEP_X_MM, -115.9),
    (_WHEEL_SIDE_SWEEP_X_MM, -115.9),
    (11.500001, -115.0),
    (70.15, -54.5),
    (83.15, -32.0),
    (83.15, 32.0),
    (70.15, 54.5),
    (11.500001, 115.0),
    (_WHEEL_SIDE_SWEEP_X_MM, 115.9),
    (-_WHEEL_SIDE_SWEEP_X_MM, 115.9),
    (-11.500001, 115.0),
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
