"""Punto fijo de acercamiento, separado de la caja de la mano en el plano."""
from __future__ import annotations

import math
from numbers import Real
import time

import cv2
import numpy as np

from .navigation_geometry import _homographies, _project, observe_robot
from .obstacle_planner import _grid, _grid_anchor
from .pose_obstacle_planner import _Scene
from .robot_footprint import ROBOT_RADIUS_MM, ROBOT_FOOTPRINT_MM


def _reachable_component(scene, center_px, deadline):
    """Descarta regiones desconectadas antes de seleccionar un destino.

    Es solo un rechazo preliminar. Cada candidato conservado todavía necesita
    el plan continuo de Navigation con la huella orientada antes del movimiento.
    Si el centro o un candidato no tienen ancla por la discretización, decide
    ese plan exacto, no la rejilla.
    """
    try:
        blocked, minimum, step, _ = _grid(scene.plane, 0.,
            min(deadline, time.monotonic() + .12), camera_clearance=0.)
        # En campos muy extensos la rejilla adaptativa puede cerrar pasos
        # físicos por cuantización. Conservar entonces el buscador de poses.
        if step > .01:
            return None
        first = _grid_anchor(scene.plane.world(center_px), blocked, minimum,
                             step, scene.plane, 0., camera_clearance=0.)
        _, labels = cv2.connectedComponents((~blocked).astype(np.uint8), connectivity=4)
        labels = labels.reshape(-1)
        component = labels[first]
    except ValueError:
        return None

    def reachable(point_px):
        try:
            index = _grid_anchor(scene.plane.world(point_px), blocked, minimum,
                                 step, scene.plane, 0., camera_clearance=0.)
        except ValueError:
            return True
        return labels[index] == component

    return reachable


def choose_interest_point(corners, calibration, box, obstacles, stand_off_mm, *, arrival_m=.025):
    """Elige un extremo; no autoriza ni calcula la trayectoria de movimiento.

    La navegación valida después el recorrido completo con la huella real.
    La distancia al robot no limita la selección: se exploran separaciones
    crecientes hasta la posición inicial, conservando la preferencia por la
    más cercana a la mano que tenga un extremo geométricamente admisible.
    """
    if (isinstance(stand_off_mm, bool) or not isinstance(stand_off_mm, Real)
            or not math.isfinite(stand_off_mm) or not 20 <= stand_off_mm <= 2000):
        raise ValueError("La separación de acercamiento no es válida.")
    if (not isinstance(box, (list, tuple)) or len(box) != 4 or
            any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v) for v in box)):
        raise ValueError("La mano no tiene una caja válida.")
    x1, y1, x2, y2 = box
    if not (0 <= x1 < x2 <= calibration.width and 0 <= y1 < y2 <= calibration.height):
        raise ValueError("La mano debe quedar dentro de la imagen.")
    forward, backward, reference = _homographies(corners, calibration.marker_size_m,
                                                 calibration.width, calibration.height)
    hand = cv2.convexHull(np.asarray([_project(forward, p, reference) for p in
        ((x1, y1), (x2, y1), (x2, y2), (x1, y2))], dtype=np.float32)).reshape(-1, 2)
    center = np.asarray(calibration.center_tag_m)
    midpoint = np.mean(hand, axis=0)

    def gap(point):
        return -float(cv2.pointPolygonTest(hand, tuple(map(float, point)), True))

    initial_gap = gap(center)
    # Reservar la huella en cualquier orientación y los 25 mm de llegada del
    # centro. Cubrir un punto con una rueda no termina esta aproximación.
    required = (float(stand_off_mm) + ROBOT_RADIUS_MM) / 1000. + arrival_m
    if initial_gap <= required + .030:
        raise ValueError("La mano ya está a la distancia de observación.")
    away = center - midpoint
    away /= np.linalg.norm(away)
    deadline = time.monotonic() + .8
    scene = _Scene(corners, calibration, obstacles, ROBOT_FOOTPRINT_MM)
    observed = observe_robot(corners, calibration)
    heading = math.radians(observed["heading_deg"])
    reachable = _reachable_component(scene, observed["center_px"], deadline) if obstacles else None
    desired, oriented_fallback = required, None
    # No limitar la búsqueda a 350 mm adicionales: una pared puede exigir
    # observar la mano desde más lejos aunque su región sea alcanzable.
    # La evaluación es perezosa y conserva un presupuesto de cálculo, sin
    # construir listas cuyo tamaño dependa de la escala del plano visible.
    step = .05
    while desired < initial_gap - .030:
        for degrees in (0., -15., 15., -30., 30., -45., 45., -60., 60., -90., 90.):
            if time.monotonic() >= deadline:
                if oriented_fallback is not None:
                    return oriented_fallback
                raise ValueError("No encontré un acercamiento accesible manteniendo la separación.")
            angle = math.radians(degrees)
            c, s = math.cos(angle), math.sin(angle)
            direction = np.asarray([c * away[0] - s * away[1], s * away[0] + c * away[1]])
            low, high = 0., float(np.linalg.norm(center - midpoint)) + desired + 1.
            # La distancia al polígono convexo crece al salir desde su centro.
            # Resolverla evita asumir que la caja sigue cuadrada en perspectiva.
            for _ in range(30):
                along = (low + high) / 2.
                if gap(midpoint + direction * along) < desired:
                    low = along
                else:
                    high = along
            candidate = midpoint + direction * high
            image = _project(backward, candidate, [0., 0.])
            if not (0 <= image[0] < calibration.width and 0 <= image[1] < calibration.height):
                continue
            if reachable is not None and not reachable(image.tolist()):
                continue
            for turn in (0., math.pi / 2., -math.pi / 2., math.pi,
                         math.pi / 4., -math.pi / 4., 3 * math.pi / 4., -3 * math.pi / 4.):
                depths, marker_visible, center_inside = scene.evaluate(scene.pose(image, heading + turn))
                if marker_visible and center_inside and np.max(depths, initial=0.) <= .000002:
                    result = {"target_px": image.tolist(), "stand_off_mm": float(stand_off_mm)}
                    if oriented_fallback is None:
                        oriented_fallback = result
                    # Preferir un extremo donde pueda orientar el cuerpo al
                    # llegar, en lugar de un hueco que solo admite entrar de
                    # espaldas. El círculo es una preferencia suficiente,
                    # nunca un veto para pasillos que sí admitan la huella.
                    point = scene.plane.world(image.tolist())
                    if scene.plane.segment_clear(point, point, scene.radius, camera_clearance=0.):
                        return result
                    break
        desired += step
    if oriented_fallback is not None:
        return oriented_fallback
    raise ValueError("No encontré un punto libre para acercarme manteniendo la separación.")
