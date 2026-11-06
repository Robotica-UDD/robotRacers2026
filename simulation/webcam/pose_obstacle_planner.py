"""Rutas de poses con la huella orientada, avance de frente y giro local.

La colisión utiliza el polígono del ensamblaje, nunca su círculo envolvente.
La cámara limita el centro y las cuatro esquinas reales del marcador; no añade
un radio de visibilidad ni separación extra. La métrica sigue siendo la del
plano del AprilTag, pendiente de calibración física del suelo.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import time

import cv2
import numpy as np

from .navigation_geometry import Calibration, observe_robot, _project
from .obstacle_planner import (_PlaneMap, _finite, _inside_polygon,
                                _segment_distance, _grid, _grid_anchor, _astar, _simplify)
from .robot_footprint import ROBOT_FOOTPRINT_MM

POSITION_CELL_M = .0125
DRIVE_PRIMITIVE_M = .025
ANGLE_PRIMITIVE_RAD = math.radians(15.)
MAX_ESCAPE_TRAVEL_M = .20
MAX_REVERSE_ESCAPE_M = .080
MAX_OUTPUT_SEGMENT_M = .45
_DISTANCE_EPS_M = 1e-7


def _wrap(value):
    return (value + math.pi) % (2. * math.pi) - math.pi


def _matrix_yaw(yaw):
    # El cuerpo usa X delante / Y izquierda. El mapa de cámara usa Y abajo.
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s], [s, -c]])


def _footprint(values):
    try:
        array = np.asarray(values)
    except (ValueError, TypeError):
        raise ValueError('La huella del robot no es válida.') from None
    if (array.ndim != 2 or array.shape[1] != 2 or not 3 <= len(array) <= 64
            or array.dtype.kind not in 'iuf' or not np.all(np.isfinite(array))):
        raise ValueError('La huella necesita entre 3 y 64 puntos finitos en milímetros.')
    array = array.astype(float) / 1000.
    if float(np.max(np.linalg.norm(array, axis=1))) > .5:
        raise ValueError('La huella del robot está fuera de escala.')
    hull = cv2.convexHull(array.astype(np.float32)).reshape(-1, 2).astype(float)
    if len(hull) < 3 or abs(float(cv2.contourArea(hull.astype(np.float32)))) < 1e-6:
        raise ValueError('La huella del robot es degenerada.')
    return hull


def _axes(polygon):
    edges = np.roll(polygon, -1, axis=0) - polygon
    lengths = np.linalg.norm(edges, axis=1)
    normals = np.column_stack((-edges[:, 1], edges[:, 0])) / lengths[:, None]
    return normals


def _triangle_contains(point, a, b, c):
    cross = lambda x, y, z: float((y[0] - x[0]) * (z[1] - x[1]) - (y[1] - x[1]) * (z[0] - x[0]))
    signs = [cross(a, b, point), cross(b, c, point), cross(c, a, point)]
    return min(signs) >= -1e-10 or max(signs) <= 1e-10


def _convex_parts(polygon):
    edges = np.roll(polygon, -1, axis=0) - polygon
    cross = edges[:, 0] * np.roll(edges, -1, axis=0)[:, 1] - edges[:, 1] * np.roll(edges, -1, axis=0)[:, 0]
    if np.all(cross > 0) or np.all(cross < 0):
        return [polygon]
    area = float(np.sum(polygon[:, 0] * np.roll(polygon[:, 1], -1)
                        - polygon[:, 1] * np.roll(polygon[:, 0], -1)))
    orientation = 1 if area > 0 else -1
    for i in range(4):
        a, b, c = polygon[i - 1], polygon[i], polygon[(i + 1) % 4]
        if orientation * float((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])) <= 0:
            continue
        other = polygon[(i + 2) % 4]
        if not _triangle_contains(other, a, b, c):
            return [np.array([a, b, c]), np.array([a, c, other])]
    raise ValueError('No se pudo resolver la geometría del obstáculo.')


def _penetration(a, b, padding=0.):
    """Profundidad SAT entre polígonos convexos; cero también permite contacto."""
    if (np.any(np.max(a, axis=0) + padding < np.min(b, axis=0))
            or np.any(np.max(b, axis=0) + padding < np.min(a, axis=0))):
        return 0.
    axes = np.vstack((_axes(a), _axes(b)))
    first, second = a @ axes.T, b @ axes.T
    low_a, high_a = np.min(first, axis=0), np.max(first, axis=0)
    low_b, high_b = np.min(second, axis=0), np.max(second, axis=0)
    overlaps = np.minimum(high_a - low_b, high_b - low_a) + padding
    if np.min(overlaps) <= 0:
        return 0.
    return float(np.min(overlaps))


@dataclass(frozen=True)
class _Pose:
    x: float
    y: float
    yaw: float

    @property
    def point(self):
        return np.array([self.x, self.y])


class _Scene:
    def __init__(self, corners, calibration, obstacles, footprint_mm, marker_visible=True):
        self.calibration = calibration
        self.plane = _PlaneMap.build(corners, calibration, obstacles)
        self.footprint = _footprint(footprint_mm)
        self.radius = float(np.max(np.linalg.norm(self.footprint, axis=1)))
        self.parts = [[part for part in _convex_parts(polygon)] for polygon in self.plane.obstacles]
        self.require_marker = marker_visible
        if type(marker_visible) is not bool:
            raise ValueError('La comprobación del marcador debe ser verdadera o falsa.')
        half = calibration.marker_size_m / 2.
        marker = np.array([[-half, half], [half, half], [half, -half], [-half, -half]])
        theta = calibration.front_angle_rad
        rotation = np.array([[math.cos(theta), math.sin(theta)],
                             [-math.sin(theta), math.cos(theta)]])
        self.marker_body = (marker - np.asarray(calibration.center_tag_m)) @ rotation.T
        carrier = np.array([[-.026, .026], [.026, .026], [.026, -.026], [-.026, -.026]])
        carrier_body = (carrier - np.asarray(calibration.center_tag_m)) @ rotation.T
        self.footprint = cv2.convexHull(np.vstack((self.footprint, carrier_body)).astype(np.float32)).reshape(-1, 2).astype(float)
        self.radius = float(np.max(np.linalg.norm(self.footprint, axis=1)))
        frame = self.plane.frame
        normals = _axes(frame)
        center = np.mean(frame, axis=0)
        offsets = np.einsum('ij,ij->i', normals, frame)
        flip = normals @ center < offsets
        normals[flip] *= -1.; offsets[flip] *= -1.
        self.camera_normals, self.camera_offsets = normals, offsets
        self.cache = {}
        self.deadline = None

    def proximity_cost(self, pose):
        if not self.parts:
            return 0.
        polygon = self.polygon(pose)
        low, high = np.min(polygon, axis=0), np.max(polygon, axis=0)
        gap = math.inf
        for obstacle in self.plane.obstacles:
            delta = np.maximum(np.maximum(np.min(obstacle, axis=0) - high,
                                          low - np.max(obstacle, axis=0)), 0.)
            gap = min(gap, float(np.linalg.norm(delta)))
        # Preferencia suave; jamás decide que una pose esté ocupada.
        return .4 * math.exp(-gap / .04)

    def image_point(self, point):
        return _project(self.plane.plane_to_image, np.asarray(point), np.zeros(2)).tolist()

    def yaw_from_image(self, center_px, heading):
        heading = _finite(heading, 'El rumbo')
        image = np.asarray(center_px, dtype=float)
        first = _project(self.plane.image_to_plane, image, self.plane.reference)
        second = _project(self.plane.image_to_plane,
                          image + np.array([math.cos(heading), -math.sin(heading)]) * 5.,
                          self.plane.reference)
        delta = second - first
        if np.linalg.norm(delta) < 1e-9:
            raise ValueError('No se puede resolver el rumbo en esta imagen.')
        return math.atan2(float(delta[1]), float(delta[0]))

    def pose(self, center_px, heading):
        point = self.plane.world(center_px)
        return _Pose(float(point[0]), float(point[1]), self.yaw_from_image(center_px, heading))

    def image_heading(self, pose):
        center = np.asarray(self.image_point(pose.point))
        front = np.asarray(self.image_point(pose.point + .02 * np.array([math.cos(pose.yaw), math.sin(pose.yaw)])))
        delta = front - center
        return math.atan2(-float(delta[1]), float(delta[0]))

    def polygon(self, pose):
        return self.footprint @ _matrix_yaw(pose.yaw).T + pose.point

    def in_camera(self, points):
        return bool(np.all(np.asarray(points) @ self.camera_normals.T >= self.camera_offsets - _DISTANCE_EPS_M))

    def evaluate(self, pose):
        key = (round(pose.x, 9), round(pose.y, 9), round(_wrap(pose.yaw), 9))
        if key in self.cache:
            return self.cache[key]
        polygon = self.polygon(pose)
        depths = np.array([max((_penetration(polygon, part) for part in parts), default=0.)
                           for parts in self.parts], dtype=float)
        marker = self.marker_body @ _matrix_yaw(pose.yaw).T + pose.point
        visible = self.in_camera(marker)
        center_visible = self.in_camera([pose.point])
        result = (depths, visible, center_visible)
        self.cache[key] = result
        return result

    def swept(self, start, end, *, allow_overlap=False, max_penetration=None, tolerance=0., curve_error=0.):
        initial, _, _ = self.evaluate(start)
        allowed = np.maximum(initial, tolerance)
        ceiling = float(np.max(initial, initial=0.)) if max_penetration is None else max_penetration
        initial_max = float(np.max(initial, initial=0.))
        if not allow_overlap and initial_max > tolerance + _DISTANCE_EPS_M:
            return False, initial_max
        if allow_overlap and initial_max > max(ceiling, tolerance) + _DISTANCE_EPS_M:
            return False, initial_max
        distance = math.hypot(end.x - start.x, end.y - start.y)
        angle = _wrap(end.yaw - start.yaw)
        intervals = max(1, math.ceil(distance / .01), math.ceil(abs(angle) / math.radians(3.)))
        previous = start
        previous_depths = initial
        maximum = initial_max
        for i in range(1, intervals + 1):
            if self.deadline is not None and time.monotonic() > self.deadline:
                return False, maximum
            t = i / intervals
            pose = _Pose(start.x + (end.x - start.x) * t,
                         start.y + (end.y - start.y) * t, start.yaw + angle * t)
            depths, marker, center = self.evaluate(pose)
            maximum = max(maximum, float(np.max(depths, initial=0.)))
            if not center or self.require_marker and not marker:
                return False, maximum
            # La envolvente convexa de ambos extremos cubre una traslación.
            # Al girar, la sagita añade solo el error de aproximación del arco.
            hull = cv2.convexHull(np.vstack((self.polygon(previous), self.polygon(pose))).astype(np.float32)).reshape(-1, 2).astype(float)
            arc_error = self.radius * (1. - math.cos(abs(angle) / intervals / 2.)) + curve_error
            swept_depths = np.array([max((_penetration(hull, part, arc_error) for part in parts), default=0.)
                                     for parts in self.parts], dtype=float)
            maximum = max(maximum, float(np.max(swept_depths, initial=0.)))
            if allow_overlap:
                # Ningún obstáculo nuevo puede penetrar la huella. El contacto
                # inicial puede disminuir; solo tolera 0,05 mm de redondeo.
                if (np.any(depths > previous_depths + .00005)
                        or np.any(swept_depths > allowed + .00005)
                        or maximum > max(ceiling, tolerance) + .00005):
                    return False, maximum
            elif np.any(swept_depths > tolerance + _DISTANCE_EPS_M):
                return False, maximum
            previous, previous_depths = pose, depths
        return True, maximum


def _options(contact_tolerance_mm, max_penetration_mm):
    tolerance = _finite(contact_tolerance_mm, 'La tolerancia de contacto')
    if not 0 <= tolerance <= 5:
        raise ValueError('La tolerancia de contacto debe estar entre 0 y 5 mm.')
    maximum = None
    if max_penetration_mm is not None:
        maximum = _finite(max_penetration_mm, 'El contacto máximo')
        if not 0 <= maximum <= 1000:
            raise ValueError('El contacto máximo debe estar entre 0 y 1000 mm.')
        maximum /= 1000.
    return tolerance / 1000., maximum


def _pose_report(scene, pose, tolerance):
    depths, marker, center = scene.evaluate(pose)
    penetration = float(np.max(depths, initial=0.))
    footprint = scene.polygon(pose)
    if penetration > 0:
        clearance = -penetration * 1000.
    elif not scene.plane.obstacles:
        clearance = None
    else:
        clearance = min(_segment_distance(footprint[i], footprint[(i + 1) % len(footprint)],
                                          obstacle[j], obstacle[(j + 1) % len(obstacle)])
                        for obstacle in scene.plane.obstacles for i in range(len(footprint))
                        for j in range(len(obstacle))) * 1000.
    clear = penetration <= tolerance + _DISTANCE_EPS_M and center and (marker or not scene.require_marker)
    return {'clear': bool(clear), 'penetration_mm': penetration * 1000.,
            'penetrations_mm': (depths * 1000.).tolist(), 'marker_visible': bool(marker),
            'center_visible': bool(center), 'clearance_mm': clearance,
            'footprint_px': [scene.image_point(point) for point in footprint],
            'reason': '' if clear else 'El marcador queda fuera de la imagen.' if not marker and scene.require_marker
            else 'El centro queda fuera de la imagen.' if not center else 'La huella del robot intersecta un obstáculo.'}


def check_pose(corners, calibration, center_px, heading_rad, obstacles_px, *,
               footprint_mm=ROBOT_FOOTPRINT_MM, contact_tolerance_mm=0., require_marker_visible=True):
    """Rumbo en imagen: cero derecha, positivo hacia arriba. No añade margen."""
    tolerance, _ = _options(contact_tolerance_mm, None)
    scene = _Scene(corners, calibration, obstacles_px, footprint_mm, require_marker_visible)
    return _pose_report(scene, scene.pose(center_px, heading_rad), tolerance)


def check_swept_motion(corners, calibration, start_px, start_heading_rad, end_px, end_heading_rad,
                       obstacles_px, *, footprint_mm=ROBOT_FOOTPRINT_MM,
                       allow_start_overlap=False, max_penetration_mm=None,
                       contact_tolerance_mm=0., require_marker_visible=True):
    if type(allow_start_overlap) is not bool:
        raise ValueError('La salida de un contacto debe ser verdadera o falsa.')
    tolerance, maximum = _options(contact_tolerance_mm, max_penetration_mm)
    scene = _Scene(corners, calibration, obstacles_px, footprint_mm, require_marker_visible)
    start, end = scene.pose(start_px, start_heading_rad), scene.pose(end_px, end_heading_rad)
    initial = scene.evaluate(start)
    if not initial[2] or scene.require_marker and not initial[1]:
        return {'clear': False, 'penetration_mm': float(np.max(initial[0], initial=0.)) * 1000.,
                'marker_visible': bool(initial[1]), 'reason': 'El marcador o el centro quedan fuera de la imagen.'}
    clear, penetration = scene.swept(start, end, allow_overlap=allow_start_overlap,
                                    max_penetration=maximum, tolerance=tolerance)
    return {'clear': bool(clear), 'penetration_mm': penetration * 1000.,
            'marker_visible': bool(scene.evaluate(end)[1]),
            'reason': '' if clear else 'El movimiento barrido invade un obstáculo o sale de la imagen.'}


def heading_error_rad(corners, calibration, heading_rad):
    """Error de giro físico positivo a la izquierda para un rumbo en imagen."""
    pose = observe_robot(corners, calibration)
    scene = _Scene(corners, calibration, [], ROBOT_FOOTPRINT_MM)
    current = scene.yaw_from_image(pose['center_px'], math.radians(pose['heading_deg']))
    desired = scene.yaw_from_image(pose['center_px'], heading_rad)
    return -_wrap(desired - current)


def check_wheel_motion(corners, calibration, left_mm_s, right_mm_s, duration_s=.12, *,
                       obstacles_px, footprint_mm=ROBOT_FOOTPRINT_MM,
                       allow_start_overlap=False, max_penetration_mm=None,
                       contact_tolerance_mm=0., require_marker_visible=True):
    """Predice el arco diferencial de las consignas, sin enviar movimiento.

    Integra con pasos de hasta20ms y verifica la huella barrida. Usa las órdenes
    solicitadas, no promete odometría real ni compensa inercia desconocida.
    """
    left = _finite(left_mm_s, 'La velocidad izquierda')
    right = _finite(right_mm_s, 'La velocidad derecha')
    seconds = _finite(duration_s, 'El horizonte de movimiento')
    if not 0 < seconds <= .5:
        raise ValueError('El horizonte de movimiento debe ser mayor que cero y de hasta0,5segundos.')
    if type(allow_start_overlap) is not bool:
        raise ValueError('La salida de un contacto debe ser verdadera o falsa.')
    tolerance, maximum = _options(contact_tolerance_mm, max_penetration_mm)
    scene = _Scene(corners, calibration, obstacles_px, footprint_mm, require_marker_visible)
    observed = observe_robot(corners, calibration)
    pose = scene.pose(observed['center_px'], math.radians(observed['heading_deg']))
    initial_depths, marker, center = scene.evaluate(pose)
    peak = float(np.max(initial_depths, initial=0.))
    ceiling = peak if maximum is None else maximum
    velocity = (left + right) / 2000.
    omega = (left - right) / 190.
    count = max(1, math.ceil(seconds / .02), math.ceil(abs(omega) * seconds / math.radians(5.)),
                math.ceil(abs(velocity) * seconds / .01))
    if count > 500:
        return {'clear': False, 'penetration_mm': peak * 1000., 'marker_visible': bool(marker),
                'predicted_center_px': observed['center_px'],
                'reason': 'La maniobra predicha supera la resolución de cálculo disponible.'}
    if not center or scene.require_marker and not marker:
        return {'clear': False, 'penetration_mm': peak * 1000., 'marker_visible': bool(marker),
                'predicted_center_px': observed['center_px'], 'reason': 'El marcador queda fuera de la imagen.'}
    dt = seconds / count
    for _ in range(count):
        angle = omega * dt
        if abs(omega) < 1e-12:
            point = pose.point + velocity * dt * np.array([math.cos(pose.yaw), math.sin(pose.yaw)])
            curve_error = 0.
        else:
            point = pose.point + velocity / omega * np.array([
                math.sin(pose.yaw + angle) - math.sin(pose.yaw),
                math.cos(pose.yaw) - math.cos(pose.yaw + angle)])
            curve_error = abs(velocity / omega) * (1. - math.cos(abs(angle) / 2.))
        following = _Pose(float(point[0]), float(point[1]), pose.yaw + angle)
        valid, penetration = scene.swept(pose, following, allow_overlap=allow_start_overlap,
                                         max_penetration=ceiling, tolerance=tolerance,
                                         curve_error=curve_error)
        peak = max(peak, penetration)
        depths, marker, _ = scene.evaluate(following)
        if allow_start_overlap and np.any(depths > np.maximum(initial_depths, tolerance) + .00005):
            valid = False
        if not valid:
            return {'clear': False, 'penetration_mm': peak * 1000., 'marker_visible': bool(marker),
                    'predicted_center_px': scene.image_point(following.point),
                    'reason': 'El arco aplicado invade un obstáculo o pierde el marcador.'}
        pose = following
    return {'clear': True, 'penetration_mm': peak * 1000., 'marker_visible': bool(marker),
            'predicted_center_px': scene.image_point(pose.point), 'reason': ''}


@dataclass
class _Node:
    pose: _Pose
    cost: float
    parent: int | None
    motion: str
    direction: int
    escaping: bool
    escape_travel: float = 0.
    reverse_travel: float = 0.


def _actions_cost(scene, start, moves):
    cost = 0.
    previous = start
    for pose, kind, direction, _ in moves:
        distance = float(np.linalg.norm(pose.point - previous.point))
        if kind == 'turn':
            cost += .025 * abs(_wrap(pose.yaw - previous.yaw))
        else:
            middle = _Pose((pose.x + previous.x) / 2., (pose.y + previous.y) / 2., pose.yaw)
            cost += distance * ((1.08 if direction < 0 else 1.)
                                + (scene.proximity_cost(previous) + scene.proximity_cost(middle)
                                   + scene.proximity_cost(pose)) / 3.)
        previous = pose
    return cost


def _connector(scene, start, goal, tolerance, deadline, all_options=False):
    delta = goal - start.point
    distance = float(np.linalg.norm(delta))
    if distance < .0005:
        return [(0., [])] if all_options else []
    bearing = math.atan2(float(delta[1]), float(delta[0]))
    candidates = []
    # Política de la aplicación: toda traslación ordinaria es hacia delante.
    # Para un objetivo detrás se valida primero el giro, sin cambiar la línea.
    for direction in (1,):
        if time.monotonic() > deadline:
            break
        yaw = _wrap(bearing + (math.pi if direction < 0 else 0.))
        turned = _Pose(start.x, start.y, yaw)
        final = _Pose(float(goal[0]), float(goal[1]), yaw)
        clear, _ = scene.swept(start, turned, tolerance=tolerance)
        if not clear:
            continue
        clear, _ = scene.swept(turned, final, tolerance=tolerance)
        if clear:
            moves = []
            if abs(_wrap(yaw - start.yaw)) > math.radians(.2):
                moves.append((turned, 'turn', 1, False))
            moves.append((final, 'drive', direction, False))
            candidates.append((_actions_cost(scene, start, moves), moves))
    candidates.sort(key=lambda value: value[0])
    if all_options:
        return candidates
    return candidates[0][1] if candidates else None


def _template_routes(scene, start, goal, tolerance, deadline):
    """Candidatos geométricos rápidos; cada giro y traslado se valida exactamente."""
    if not scene.plane.obstacles:
        return None
    boxes = [(np.min(poly, axis=0), np.max(poly, axis=0)) for poly in scene.plane.obstacles]
    if len(boxes) > 1:
        boxes.append((np.min([box[0] for box in boxes], axis=0), np.max([box[1] for box in boxes], axis=0)))
    proposals = []
    for pad in (scene.radius + .005, scene.radius + .04, scene.radius * .7, scene.radius * .4, .025):
        for low, high in boxes:
            points = [low + [-pad, -pad], [high[0] + pad, low[1] - pad],
                      high + [pad, pad], [low[0] - pad, high[1] + pad]]
            for i in range(4):
                for direction in (1, -1):
                    chain = [np.asarray(points[i]), np.asarray(points[(i + direction) % 4])]
                    estimate = float(np.linalg.norm(chain[0] - start.point)
                                     + np.linalg.norm(chain[1] - chain[0]) + np.linalg.norm(goal - chain[1]))
                    # Se prueban antes las rutas con sitio para girar; las capas
                    # estrechas siguen disponibles y no son una restricción.
                    proposals.append((0 if pad > scene.radius else 1, estimate, chain))
    proposals.sort(key=lambda value: value[:2])
    best = None
    for _, _, chain in proposals:
        if time.monotonic() > deadline:
            break
        beam = [(0., start, [])]
        for target in [*chain, goal]:
            next_beam = []
            for cost, pose, actions in beam:
                for extra, moves in _connector(scene, pose, target, tolerance, deadline, all_options=True):
                    last = moves[-1][0] if moves else pose
                    next_beam.append((cost + extra, last, actions + moves))
            if not next_beam:
                beam = []
                break
            next_beam.sort(key=lambda value: value[0])
            beam = next_beam[:2]
        if beam and (best is None or beam[0][0] < best[0]):
            best = (beam[0][0], beam[0][2])
    return best[1] if best else None


def _needs_initial_reverse(scene, start, goal, tolerance, deadline):
    bearing = math.atan2(float(goal[1] - start.y), float(goal[0] - start.x))
    desired = _Pose(start.x, start.y, bearing)
    required_turn, _ = scene.swept(start, desired, tolerance=tolerance)
    if not required_turn:
        return True
    quarters = []
    for sign in (1, -1):
        if time.monotonic() > deadline:
            return False
        free, _ = scene.swept(start, _Pose(start.x, start.y, start.yaw + sign * math.pi / 2.),
                              tolerance=tolerance)
        quarters.append(free)
    return not any(quarters)


def _guided_routes(scene, start, goal, tolerance, escaping, initial_penetration, deadline):
    """Usa el A* planar como generador; solo la huella barrida acepta maniobras.

    Un círculo guía jamás veta una salida. Se prueban radios distintos y un
    prefijo firmado para liberar una esquina antes de girar. Si ninguna guía
    sirve, la búsqueda de poses conserva su oportunidad de encontrar un paso
    orientado más estrecho.
    """
    if not scene.plane.obstacles:
        return None
    prefixes = [(start, [], False)] if not escaping else []
    allow_reverse = escaping or _needs_initial_reverse(scene, start, goal, tolerance, deadline)
    for direction in (1, -1):
        if direction < 0 and not allow_reverse:
            continue
        for length in (.005, .010, .020, .025, .050, .075, MAX_REVERSE_ESCAPE_M):
            if time.monotonic() > deadline:
                return None
            point = start.point + direction * length * np.array([math.cos(start.yaw), math.sin(start.yaw)])
            pose = _Pose(float(point[0]), float(point[1]), start.yaw)
            valid, _ = scene.swept(start, pose, allow_overlap=escaping,
                                   max_penetration=initial_penetration if escaping else None,
                                   tolerance=tolerance)
            if valid and float(np.max(scene.evaluate(pose)[0], initial=0.)) <= tolerance + _DISTANCE_EPS_M:
                recovery = escaping or direction < 0
                prefixes.append((pose, [(pose, 'escape' if recovery else 'drive', direction, recovery)], direction < 0))
    # El radio grande suele dejar espacio para girar; los menores son solo
    # propuestas adicionales, sin sustituir jamás la colisión orientada.
    # Agotar primero las propuestas sin retroceso, incluso las guías estrechas.
    # La vuelta inicial hacia atrás solo libera contacto o falta de giro.
    for reverse_phase in (False, True):
        for radius in (math.ceil(scene.radius * 10000.) / 10000., scene.radius * .6, 0.):
            for pose, prefix, is_reverse in prefixes:
                if is_reverse != reverse_phase:
                    continue
                if time.monotonic() > deadline:
                    return None
                if (not scene.plane.segment_clear(pose.point, pose.point, radius, 0.)
                        or not scene.plane.segment_clear(goal, goal, radius, 0.)):
                    continue
                try:
                    blocked, minimum, cell, _ = _grid(scene.plane, radius, deadline, 0.)
                    first = _grid_anchor(pose.point, blocked, minimum, cell, scene.plane, radius, 0.)
                    last = _grid_anchor(goal, blocked, minimum, cell, scene.plane, radius, 0.)
                    indices = _astar(blocked, first, last, cell, deadline)
                    columns = blocked.shape[1]
                    points = [pose.point] + [minimum + np.array([node % columns, node // columns]) * cell
                                            for node in indices] + [goal]
                    guide = _simplify(points, scene.plane, radius, deadline, 0.)
                except ValueError:
                    continue
                beam = [(0., pose, prefix)]
                for waypoint in guide[1:]:
                    following = []
                    for cost, current, moves in beam:
                        for extra, tail in _connector(scene, current, waypoint, tolerance, deadline, all_options=True):
                            following.append((cost + extra, tail[-1][0] if tail else current, moves + tail))
                    if not following:
                        beam = []
                        break
                    following.sort(key=lambda value: value[0])
                    beam = following[:2]
                if beam:
                    return min(beam, key=lambda value: _actions_cost(scene, start, value[2]))[2]
    return None

def _output(scene, start, actions, initial_penetration, planning_started):
    merged = []
    before = start
    for pose, kind, direction, escaping in actions:
        distance = float(np.linalg.norm(pose.point - before.point))
        segment_limit = .15 if escaping or kind == 'escape' else MAX_OUTPUT_SEGMENT_M
        if merged and kind == merged[-1][1] and direction == merged[-1][2] and escaping == merged[-1][3]:
            last_pose = merged[-1][0]
            previous = start if len(merged) == 1 else merged[-2][0]
            if kind in {'drive', 'escape'} and abs(_wrap(pose.yaw - last_pose.yaw)) < 1e-6:
                total = float(np.linalg.norm(pose.point - previous.point))
                if total <= segment_limit + 1e-7:
                    merged[-1] = (pose, kind, direction, escaping)
                    before = pose
                    continue
            if kind == 'turn' and np.linalg.norm(pose.point - last_pose.point) < 1e-7:
                a = _wrap(last_pose.yaw - previous.yaw)
                b = _wrap(pose.yaw - last_pose.yaw)
                if a * b >= 0 and abs(a + b) <= math.pi / 2 + 1e-7:
                    merged[-1] = (pose, kind, direction, escaping)
                    before = pose
                    continue
        # Los conectores pueden ser largos: dividirlos para el controlador local.
        angle = _wrap(pose.yaw - before.yaw)
        pieces = (max(1, math.ceil(distance / segment_limit)) if kind != 'turn'
                  else max(1, math.ceil(abs(angle) / (math.pi / 2.))))
        for i in range(1, pieces + 1):
            ratio = i / pieces
            next_pose = _Pose(before.x + (pose.x - before.x) * ratio,
                              before.y + (pose.y - before.y) * ratio,
                              before.yaw + angle * ratio if kind == 'turn' else pose.yaw)
            merged.append((next_pose, kind, direction, escaping))
        before = pose
    steps = []
    path = [scene.image_point(start.point)]
    length = 0.
    previous = start
    for pose, kind, direction, escaping in merged:
        image = scene.image_point(pose.point)
        steps.append({'target_px': image, 'heading_rad': scene.image_heading(pose),
                      'direction': direction, 'kind': kind, 'escaping': escaping})
        distance = float(np.linalg.norm(pose.point - previous.point))
        length += distance
        if distance > 1e-7:
            path.append(image)
        previous = pose
    if len(path) == 1:
        path.append(path[0][:])
    return {'steps': steps, 'path_px': path,
            'footprint_px': [scene.image_point(point) for point in scene.polygon(start)],
            'initial_penetration_mm': initial_penetration * 1000., 'path_length_m': length,
            'planning_ms': (time.monotonic() - planning_started) * 1000.}


def plan_pose_path(corners, calibration, start_px, goal_px, obstacles_px, *, start_heading_rad=None,
                   footprint_mm=ROBOT_FOOTPRINT_MM, max_seconds=3., allow_start_overlap=True,
                   contact_tolerance_mm=0., require_marker_visible=True):
    """Búsqueda híbrida de poses con avance ordinario siempre hacia delante.

    ``heading_rad`` de cada paso está en la imagen, positivo hacia arriba.
    ``escaping`` indica un paso que libera un contacto inicial sin aumentarlo.
    No permite atravesar otro obstáculo ni añadir penetración durante el escape.
    El retroceso se limita al prefijo de liberación inicial, con hasta80mm.
    """
    seconds = _finite(max_seconds, 'El tiempo de cálculo')
    if not .05 <= seconds <= 5.:
        raise ValueError('El tiempo de cálculo debe estar entre 0,05 y 5 segundos.')
    if type(allow_start_overlap) is not bool:
        raise ValueError('La salida de un contacto debe ser verdadera o falsa.')
    started = time.monotonic(); deadline = started + seconds
    tolerance, _ = _options(contact_tolerance_mm, None)
    scene = _Scene(corners, calibration, obstacles_px, footprint_mm, require_marker_visible)
    scene.deadline = deadline
    if start_heading_rad is None:
        start_heading_rad = math.radians(observe_robot(corners, calibration)['heading_deg'])
    start = scene.pose(start_px, start_heading_rad)
    goal = scene.plane.world(goal_px)
    depths, marker, center = scene.evaluate(start)
    initial_penetration = float(np.max(depths, initial=0.))
    if not center or require_marker_visible and not marker:
        raise ValueError('El marcador o el centro quedan fuera de la imagen al iniciar.')
    if initial_penetration > tolerance and not allow_start_overlap:
        raise ValueError('La huella del robot intersecta un obstáculo al iniciar.')
    if initial_penetration > 2. * scene.radius:
        raise ValueError('La superposición inicial es demasiado profunda para una maniobra de salida.')
    if any(_inside_polygon(goal, polygon) for polygon in scene.plane.obstacles):
        raise ValueError('El destino está dentro de un obstáculo.')
    escaping = initial_penetration > tolerance + _DISTANCE_EPS_M
    incumbent = None
    if not escaping:
        direct = _connector(scene, start, goal, tolerance, deadline)
        if direct is not None:
            if _actions_cost(scene, start, direct) < float(np.linalg.norm(goal - start.point)) * 1.12 + .03:
                return _output(scene, start, direct, initial_penetration, started)
            incumbent = direct
    guide_deadline = min(deadline - .3, started + seconds * .45)
    guided = _guided_routes(scene, start, goal, tolerance, escaping, initial_penetration, guide_deadline)
    if guided is not None:
        if incumbent is None or _actions_cost(scene, start, guided) < _actions_cost(scene, start, incumbent):
            incumbent = guided
        return _output(scene, start, incumbent, initial_penetration, started)
    if not escaping:
        # Una pequeña maniobra previa resuelve puntos pegados a una esquina sin
        # obligar a mover primero 25 mm ni a atravesar el objeto.
        for direction in (1,):
            for travel in (.005, .010, .020):
                point = start.point + direction * travel * np.array([math.cos(start.yaw), math.sin(start.yaw)])
                pose = _Pose(float(point[0]), float(point[1]), start.yaw)
                valid, _ = scene.swept(start, pose, tolerance=tolerance)
                if valid:
                    tail = _connector(scene, pose, goal, tolerance, deadline)
                    if tail is not None:
                        moves = [(pose, 'drive', direction, False), *tail]
                        if incumbent is None or _actions_cost(scene, start, moves) < _actions_cost(scene, start, incumbent):
                            incumbent = moves
        template_deadline = min(deadline - .3, started + seconds * .45)
        template = _template_routes(scene, start, goal, tolerance, template_deadline)
        if template is not None and (incumbent is None or _actions_cost(scene, start, template) < _actions_cost(scene, start, incumbent)):
            incumbent = template
        if incumbent is not None:
            return _output(scene, start, incumbent, initial_penetration, started)
    nodes = [_Node(start, 0., None, '', 1, escaping)]
    serial = 0
    def key(pose, escape):
        fine = escape or math.hypot(pose.x - start.x, pose.y - start.y) < .05
        cell = .005 if fine else POSITION_CELL_M
        return (round(pose.x / cell), round(pose.y / cell),
                round(_wrap(pose.yaw) / ANGLE_PRIMITIVE_RAD), escape, fine)
    best = {key(start, escaping): 0.}
    heap = [(1.2 * float(np.linalg.norm(goal - start.point)), serial, 0)]
    best_distance = math.inf
    expanded = 0
    while heap:
        if time.monotonic() > deadline:
            raise ValueError('No se pudo completar la ruta de maniobras en el tiempo disponible.')
        _, _, index = heapq.heappop(heap)
        node = nodes[index]
        if node.cost > best.get(key(node.pose, node.escaping), math.inf) + 1e-9:
            continue
        distance = float(np.linalg.norm(goal - node.pose.point))
        expanded += 1
        if not node.escaping and (distance < best_distance - .012 or expanded % 20 == 0):
            best_distance = min(best_distance, distance)
            tail = _connector(scene, node.pose, goal, tolerance, deadline)
            if tail is not None:
                actions = []
                current = index
                while nodes[current].parent is not None:
                    child = nodes[current]
                    actions.append((child.pose, child.motion, child.direction, child.escaping
                                    or nodes[child.parent].escaping))
                    current = child.parent
                actions.reverse(); actions.extend(tail)
                return _output(scene, start, actions, initial_penetration, started)
        primitives = []
        for direction in ((1, -1) if node.escaping else (1,)):
            lengths = ((.005, DRIVE_PRIMITIVE_M) if node.escaping
                       or math.hypot(node.pose.x - start.x, node.pose.y - start.y) < .05
                       else (DRIVE_PRIMITIVE_M,))
            for length in lengths:
                point = node.pose.point + direction * length * np.array([math.cos(node.pose.yaw), math.sin(node.pose.yaw)])
                primitives.append((_Pose(float(point[0]), float(point[1]), node.pose.yaw),
                                   'escape' if node.escaping else 'drive', direction, length))
        for sign in (1, -1):
            primitives.append((_Pose(node.pose.x, node.pose.y, _wrap(node.pose.yaw + sign * ANGLE_PRIMITIVE_RAD)),
                               'turn', 1, scene.radius * ANGLE_PRIMITIVE_RAD))
        for pose, motion, direction, travel in primitives:
            if time.monotonic() > deadline:
                break
            escape_travel = node.escape_travel + travel if node.escaping else 0.
            reverse_travel = node.reverse_travel + (travel if direction < 0 and motion != 'turn' else 0.)
            if node.escaping and escape_travel > MAX_ESCAPE_TRAVEL_M:
                continue
            if reverse_travel > MAX_REVERSE_ESCAPE_M + 1e-9:
                continue
            valid, _ = scene.swept(node.pose, pose, allow_overlap=node.escaping,
                                   max_penetration=initial_penetration if node.escaping else None,
                                   tolerance=tolerance)
            if not valid:
                continue
            new_depth = float(np.max(scene.evaluate(pose)[0], initial=0.))
            still_escaping = node.escaping and new_depth > tolerance + _DISTANCE_EPS_M
            movement_cost = .025 * ANGLE_PRIMITIVE_RAD if motion == 'turn' else travel * (1.08 if direction < 0 else 1.)
            switch_cost = .01 if motion != 'turn' and direction != node.direction else 0.
            cost = node.cost + movement_cost + switch_cost + (new_depth * 2. if still_escaping else 0.)
            state_key = key(pose, still_escaping)
            if cost + 1e-9 >= best.get(state_key, math.inf):
                continue
            best[state_key] = cost
            nodes.append(_Node(pose, cost, index, motion, direction, still_escaping, escape_travel, reverse_travel))
            serial += 1
            heuristic = float(np.linalg.norm(goal - pose.point))
            heapq.heappush(heap, (cost + 1.2 * heuristic, serial, len(nodes) - 1))
    raise ValueError('No hay una secuencia libre de avance, retroceso y giro para llegar al punto.')
