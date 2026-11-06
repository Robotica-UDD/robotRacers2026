"""Planificación planar con obstáculos dibujados en una cámara fija.

El AprilTag aporta una escala en su plano, no una calibración del suelo. El
robot se representa por un círculo conservador. Se infla por separado el margen
de seguridad y una reserva de seguimiento; cada segmento final se comprueba con
geometría continua, además de la búsqueda A* sobre una cuadrícula conservadora.
No abre dispositivos ni envía órdenes.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from numbers import Real
import time

import cv2
import numpy as np

from .navigation_geometry import Calibration, _homographies, _project

MAX_OBSTACLES = 32
MAX_GRID_NODES = 300_000
MIN_GRID_M = .005
MAX_GRID_M = .010
MAX_PLANNING_SECONDS = 3.
_EPS = 1e-9
_DISTANCE_TOLERANCE_M = 2e-6


def _finite(value, label):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{label} debe ser un número finito.")
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise ValueError(f"{label} debe ser un número finito.") from None
    if not math.isfinite(result):
        raise ValueError(f"{label} debe ser un número finito.")
    return result


def _point(value, width, height, label):
    if (not isinstance(value, (list, tuple, np.ndarray))
            or isinstance(value, np.ndarray) and value.ndim != 1 or len(value) != 2):
        raise ValueError(f"{label} necesita dos coordenadas.")
    result = [_finite(value[0], label), _finite(value[1], label)]
    if not (0 <= result[0] < width and 0 <= result[1] < height):
        raise ValueError(f"{label} debe quedar dentro de la imagen.")
    return result


def _cross(a, b, c):
    return float((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))


def _point_segment_distance(point, a, b):
    delta = b - a
    squared = float(delta @ delta)
    if squared <= _EPS * _EPS:
        return float(np.linalg.norm(point - a))
    t = max(0., min(1., float((point - a) @ delta) / squared))
    return float(np.linalg.norm(point - a - t * delta))


def _segments_intersect(a, b, c, d):
    values = (_cross(a, b, c), _cross(a, b, d), _cross(c, d, a), _cross(c, d, b))
    if ((values[0] > _EPS and values[1] < -_EPS or values[0] < -_EPS and values[1] > _EPS)
            and (values[2] > _EPS and values[3] < -_EPS or values[2] < -_EPS and values[3] > _EPS)):
        return True
    return any(abs(value) <= _EPS and _point_segment_distance(point, x, y) <= _EPS
               for value, point, x, y in ((values[0], c, a, b), (values[1], d, a, b),
                                           (values[2], a, c, d), (values[3], b, c, d)))


def _segment_distance(a, b, c, d):
    if _segments_intersect(a, b, c, d):
        return 0.
    return min(_point_segment_distance(a, c, d), _point_segment_distance(b, c, d),
               _point_segment_distance(c, a, b), _point_segment_distance(d, a, b))


def _inside_polygon(point, polygon):
    inside = False
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        if _point_segment_distance(point, a, b) <= _EPS:
            return True
        if (a[1] > point[1]) != (b[1] > point[1]):
            x = (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]
            if point[0] < x:
                inside = not inside
    return inside


def validate_obstacles(polygons_px, width, height):
    """Normaliza hasta 32 cuadriláteros simples, incluidos los cóncavos."""
    if (type(width) is not int or type(height) is not int
            or not 1 <= width <= 8192 or not 1 <= height <= 8192):
        raise ValueError("La resolución de la cámara no es válida.")
    if not isinstance(polygons_px, (list, tuple)) or len(polygons_px) > MAX_OBSTACLES:
        raise ValueError("Puedes dibujar hasta 32 obstáculos de cuatro lados.")
    result = []
    for value in polygons_px:
        if (not isinstance(value, (list, tuple, np.ndarray))
                or isinstance(value, np.ndarray) and value.ndim != 2 or len(value) != 4):
            raise ValueError("Cada obstáculo debe tener exactamente cuatro vértices.")
        points = [_point(point, width, height, "El vértice") for point in value]
        polygon = np.asarray(points, dtype=float)
        edges = np.roll(polygon, -1, axis=0) - polygon
        area = .5 * abs(float(np.sum(polygon[:, 0] * np.roll(polygon[:, 1], -1)
                                    - polygon[:, 1] * np.roll(polygon[:, 0], -1))))
        if (area < 4. or np.any(np.linalg.norm(edges, axis=1) < 2.)
                or any(abs(_cross(polygon[i - 1], polygon[i], polygon[(i + 1) % 4])) < 1e-6
                       for i in range(4))):
            raise ValueError("El obstáculo es demasiado pequeño o tiene lados degenerados.")
        if (_segments_intersect(polygon[0], polygon[1], polygon[2], polygon[3])
                or _segments_intersect(polygon[1], polygon[2], polygon[3], polygon[0])):
            raise ValueError("Los lados del obstáculo no pueden cruzarse.")
        result.append(points)
    return result


@dataclass
class _PlaneMap:
    image_to_plane: np.ndarray
    plane_to_image: np.ndarray
    reference: np.ndarray
    frame: np.ndarray
    obstacles: list[np.ndarray]
    width: int
    height: int

    @classmethod
    def build(cls, corners, calibration, obstacles):
        if not isinstance(calibration, Calibration):
            raise ValueError("Calibra el robot antes de calcular el camino.")
        values = validate_obstacles(obstacles, calibration.width, calibration.height)
        raw_forward, _, reference = _homographies(corners, calibration.marker_size_m,
                                                  calibration.width, calibration.height)
        # El marco del AprilTag gira y se traslada con el robot. Una cuadrícula
        # fijada a ese marco puede resolver un empate A* por el lado opuesto al
        # girar el marcador, aunque la cámara y los obstáculos no cambien.
        # Fijar origen y ejes al borde superior de la imagen conserva las
        # distancias métricas mediante una transformación rígida/reflexión.
        image_frame = ((0., 0.), (calibration.width - 1., 0.),
                       (calibration.width - 1., calibration.height - 1.),
                       (0., calibration.height - 1.))
        raw_frame = np.array([_project(raw_forward, np.asarray(point), reference)
                              for point in image_frame])
        origin = raw_frame[0]
        x_axis = raw_frame[1] - origin
        edge_length = float(np.linalg.norm(x_axis))
        if not math.isfinite(edge_length) or edge_length < 1e-8:
            raise ValueError("El plano visible no permite fijar los ejes de la cámara.")
        x_axis /= edge_length
        y_axis = np.array([-x_axis[1], x_axis[0]])
        if float(y_axis @ (raw_frame[3] - origin)) < 0:
            y_axis *= -1.
        rigid = np.array([[x_axis[0], x_axis[1], -float(x_axis @ origin)],
                          [y_axis[0], y_axis[1], -float(y_axis @ origin)],
                          [0., 0., 1.]])
        forward = rigid @ raw_forward
        backward = np.linalg.inv(forward)
        def project(point):
            # La precisión geométrica del visor es muy inferior a un micrómetro;
            # estabilizar ese ruido numérico evita cambiar celdas por redondeo.
            return np.round(_project(forward, np.asarray(point, dtype=float), reference), 6)
        frame = np.array([project(point) for point in image_frame])
        span = np.ptp(frame, axis=0)
        if np.any(span <= 1e-6) or np.any(span > 10.) or not np.all(np.isfinite(span)):
            raise ValueError("El plano visible es demasiado grande o está muy inclinado. Ajusta la cámara cenital.")
        turns = [_cross(frame[i - 1], frame[i], frame[(i + 1) % 4]) for i in range(4)]
        if not (all(value > _EPS for value in turns) or all(value < -_EPS for value in turns)):
            raise ValueError("El plano visible no permite un recorrido seguro. Ajusta la cámara cenital.")
        return cls(forward, backward, reference, frame,
                   [np.array([project(point) for point in polygon]) for polygon in values],
                   calibration.width, calibration.height)

    def world(self, point):
        value = _point(point, self.width, self.height, "El punto")
        return np.round(_project(self.image_to_plane, np.asarray(value), self.reference), 6)

    def image(self, point):
        value = _project(self.plane_to_image, np.asarray(point), np.zeros(2))
        # Pequeñas diferencias numéricas en el contorno no salen del visor.
        return [float(np.clip(value[0], 0., self.width - 1.)),
                float(np.clip(value[1], 0., self.height - 1.))]

    def point_clearances(self, point):
        """Distancias independientes, sin restar los márgenes del consumidor."""
        signed_area = sum(self.frame[i, 0] * self.frame[(i + 1) % 4, 1]
                          - self.frame[i, 1] * self.frame[(i + 1) % 4, 0] for i in range(4))
        direction = 1. if signed_area > 0 else -1.
        camera = min(direction * _cross(a, self.frame[(i + 1) % 4], point)
                     / float(np.linalg.norm(self.frame[(i + 1) % 4] - a))
                     for i, a in enumerate(self.frame))
        obstacle = math.inf
        for polygon in self.obstacles:
            if _inside_polygon(point, polygon):
                obstacle = 0.
                break
            obstacle = min(obstacle, *(_point_segment_distance(point, polygon[i], polygon[(i + 1) % len(polygon)])
                                       for i in range(len(polygon))))
        return max(0., camera), obstacle

    def segment_clear(self, a, b, clearance, camera_clearance=None):
        # La imagen proyectada es convexa: comprobar sus semiplanos en ambos
        # extremos también comprueba todo el segmento intermedio.
        signed_area = sum(self.frame[i, 0] * self.frame[(i + 1) % 4, 1]
                          - self.frame[i, 1] * self.frame[(i + 1) % 4, 0] for i in range(4))
        direction = 1. if signed_area > 0 else -1.
        view_margin = clearance if camera_clearance is None else camera_clearance
        for i, c in enumerate(self.frame):
            d = self.frame[(i + 1) % 4]
            length = float(np.linalg.norm(d - c))
            if any(direction * _cross(c, d, point) / length <= view_margin + _DISTANCE_TOLERANCE_M
                   for point in (a, b)):
                return False
        for polygon in self.obstacles:
            if _inside_polygon(a, polygon) or _inside_polygon(b, polygon):
                return False
            if any(_segment_distance(a, b, polygon[i], polygon[(i + 1) % len(polygon)])
                   <= clearance + _DISTANCE_TOLERANCE_M for i in range(len(polygon))):
                return False
        return True


def _clearance(value, label):
    value = _finite(value, label)
    if not 0 <= value <= 1000.:
        raise ValueError(f"{label} debe estar entre 0 y 1000 mm.")
    return value / 1000.


def camera_visibility_margin_mm(calibration, tracking_margin_mm=0, extra_margin_mm=20):
    """Radio que mantiene el marcador visible al girar alrededor del eje real.

    Incluye el desplazamiento del AprilTag respecto al centro, su semidiagonal
    y, por defecto, 20 mm extra para la imagen. La planificación puede añadir su reserva de
    seguimiento; no es el radio físico usado alrededor de obstáculos.
    """
    if not isinstance(calibration, Calibration):
        raise ValueError("Calibra el robot antes de calcular el margen visible.")
    reserve = _clearance(tracking_margin_mm, "La reserva de seguimiento") * 1000.
    extra = _clearance(extra_margin_mm, "El margen extra de visibilidad") * 1000.
    offset = math.hypot(*calibration.center_tag_m) * 1000.
    marker_radius = calibration.marker_size_m * math.sqrt(2.) / 2. * 1000.
    return offset + marker_radius + extra + reserve


def segment_is_clear(corners, calibration, start_px, end_px, obstacles_px, clearance_mm=180,
                     camera_clearance_mm=None):
    """Comprueba un segmento o un centro (inicio=fin) sin búsqueda ni IO.

    El margen de cámara es independiente del margen de obstáculos. None mantiene
    el comportamiento anterior: aplicar el mismo margen a ambos. La tangencia
    exacta se considera ocupada para no depender de redondeos de la proyección.
    """
    clearance = _clearance(clearance_mm, "La distancia de separación")
    view = clearance if camera_clearance_mm is None else _clearance(camera_clearance_mm, "El margen visible")
    plane = _PlaneMap.build(corners, calibration, obstacles_px)
    return plane.segment_clear(plane.world(start_px), plane.world(end_px), clearance, view)


def point_clearances_mm(corners, calibration, point_px, obstacles_px):
    """Devuelve obstacle_mm y camera_mm, sin restar robot ni márgenes.

    Sin obstáculos obstacle_mm es infinito; camera_mm siempre corresponde al
    borde visible. Fuera de la imagen la distancia visible es cero.
    """
    plane = _PlaneMap.build(corners, calibration, obstacles_px)
    if (not isinstance(point_px, (list, tuple, np.ndarray))
            or isinstance(point_px, np.ndarray) and point_px.ndim != 1 or len(point_px) != 2):
        raise ValueError("El punto necesita dos coordenadas.")
    image_point = [_finite(item, "El punto") for item in point_px]
    if not (0 <= image_point[0] < plane.width and 0 <= image_point[1] < plane.height):
        return {"obstacle_mm": 0. if plane.obstacles else math.inf, "camera_mm": 0.}
    point = plane.world(image_point)
    camera, obstacle = plane.point_clearances(point)
    return {"obstacle_mm": obstacle * 1000., "camera_mm": camera * 1000.}


def point_clearance_mm(corners, calibration, point_px, obstacles_px):
    """Compatibilidad: menor distancia al obstáculo o al borde visible."""
    values = point_clearances_mm(corners, calibration, point_px, obstacles_px)
    return min(values["obstacle_mm"], values["camera_mm"])


def _check_deadline(deadline):
    if time.monotonic() > deadline:
        raise ValueError("El mapa requiere demasiado cálculo. Reduce los obstáculos o el área visible.")


def _grid(plane, clearance, deadline, camera_clearance=None):
    minimum = np.min(plane.frame, axis=0)
    maximum = np.max(plane.frame, axis=0)
    span = maximum - minimum
    step = MIN_GRID_M
    while True:
        columns, rows = (int(math.ceil(float(value) / step)) + 1 for value in span)
        if columns * rows <= MAX_GRID_NODES:
            break
        if step >= MAX_GRID_M:
            raise ValueError("El área visible es demasiado grande para planificar. Acerca la cámara.")
        step = min(MAX_GRID_M, step * 1.1)
    x, y = np.meshgrid(minimum[0] + np.arange(columns) * step,
                       minimum[1] + np.arange(rows) * step)
    obstacle_mask = np.zeros((rows, columns), dtype=bool)
    padded_obstacles = np.zeros_like(obstacle_mask)
    pad = step / math.sqrt(2.) + 1e-7
    for polygon in plane.obstacles:
        _check_deadline(deadline)
        inside = np.zeros_like(obstacle_mask)
        squared_distance = np.full_like(x, math.inf)
        for i, a in enumerate(polygon):
            b = polygon[(i + 1) % len(polygon)]
            dx, dy = b - a
            denominator = dx * dx + dy * dy
            t = np.clip(((x - a[0]) * dx + (y - a[1]) * dy) / denominator, 0., 1.)
            squared_distance = np.minimum(squared_distance, (x - a[0] - t * dx) ** 2 + (y - a[1] - t * dy) ** 2)
            if abs(dy) > _EPS:
                inside ^= ((a[1] > y) != (b[1] > y)) & (x < (b[0] - a[0]) * (y - a[1]) / dy + a[0])
        obstacle_mask |= inside | (squared_distance <= (clearance + _EPS) ** 2)
        padded_obstacles |= inside | (squared_distance <= (clearance + pad) ** 2)
    blocked = padded_obstacles.copy()
    area = sum(plane.frame[i, 0] * plane.frame[(i + 1) % 4, 1]
               - plane.frame[i, 1] * plane.frame[(i + 1) % 4, 0] for i in range(4))
    direction = 1. if area > 0 else -1.
    view_margin = clearance if camera_clearance is None else camera_clearance
    for i, a in enumerate(plane.frame):
        b = plane.frame[(i + 1) % 4]
        dx, dy = b - a
        signed = direction * (dx * (y - a[1]) - dy * (x - a[0])) / math.hypot(dx, dy)
        blocked |= signed <= view_margin + pad
    contours, _ = cv2.findContours(obstacle_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    outlines = []
    for contour in contours:
        simplified = cv2.approxPolyDP(contour, .7, True).reshape(-1, 2)
        if len(simplified) >= 3:
            outlines.append([plane.image(minimum + point * step) for point in simplified])
    return blocked, minimum, step, outlines


def _grid_anchor(point, blocked, minimum, step, plane, clearance, camera_clearance=None):
    rows, columns = blocked.shape
    column, row = np.rint((point - minimum) / step).astype(int)
    candidates = []
    for r in range(max(0, row - 4), min(rows, row + 5)):
        for c in range(max(0, column - 4), min(columns, column + 5)):
            if not blocked[r, c]:
                world = minimum + np.array([c, r]) * step
                candidates.append((float(np.linalg.norm(world - point)), r, c, world))
    for _, row, column, world in sorted(candidates, key=lambda item: item[:3]):
        if plane.segment_clear(point, world, clearance, camera_clearance):
            return row * columns + column
    raise ValueError("El punto está demasiado cerca del margen de un obstáculo o del borde visible.")


def _astar(blocked, start, goal, step, deadline):
    rows, columns = blocked.shape
    count = rows * columns
    costs = np.full(count, math.inf)
    parents = np.full(count, -1, dtype=np.int32)
    closed = np.zeros(count, dtype=bool)
    goal_row, goal_col = divmod(goal, columns)
    costs[start] = 0.
    def heuristic(node):
        row, col = divmod(node, columns)
        return math.hypot(col - goal_col, row - goal_row) * step
    heap = [(heuristic(start), heuristic(start), start)]
    neighbors = ((-1, 0), (0, -1), (0, 1), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1))
    expanded = 0
    while heap:
        if expanded % 512 == 0:
            _check_deadline(deadline)
        _, _, node = heapq.heappop(heap)
        if closed[node]:
            continue
        if node == goal:
            path = [node]
            while path[-1] != start:
                path.append(int(parents[path[-1]]))
            return list(reversed(path))
        closed[node] = True
        expanded += 1
        row, col = divmod(node, columns)
        for dr, dc in neighbors:
            r, c = row + dr, col + dc
            if not (0 <= r < rows and 0 <= c < columns) or blocked[r, c]:
                continue
            if dr and dc and (blocked[row, c] or blocked[r, col]):
                continue
            adjacent = r * columns + c
            if closed[adjacent]:
                continue
            proposed = costs[node] + step * (math.sqrt(2.) if dr and dc else 1.)
            if proposed + 1e-12 < costs[adjacent]:
                costs[adjacent], parents[adjacent] = proposed, node
                h = heuristic(adjacent)
                heapq.heappush(heap, (proposed + h, h, adjacent))
    raise ValueError("No hay un camino libre con el margen de seguridad actual.")


def _simplify(points, plane, clearance, deadline, camera_clearance=None):
    result = [points[0]]
    i = 0
    while i < len(points) - 1:
        _check_deadline(deadline)
        j = len(points) - 1
        while j > i + 1 and not plane.segment_clear(points[i], points[j], clearance, camera_clearance):
            j -= 1
            if j % 32 == 0:
                _check_deadline(deadline)
        if not plane.segment_clear(points[i], points[j], clearance, camera_clearance):
            raise ValueError("El camino calculado no conserva la distancia de seguridad.")
        result.append(points[j])
        i = j
    return result


def plan_path(corners, calibration, start_px, goal_px, obstacles_px, robot_radius_mm=150,
              safety_margin_mm=30, tracking_margin_mm=40, max_segment_m=.45,
              camera_clearance_mm=None):
    """Calcula un camino de centros y reserva espacio para el robot completo.

    No modifica los polígonos ni el objetivo. Un fallo lanza ValueError; nunca
    sustituye un camino bloqueado por un avance directo.
    """
    clearance = sum(_clearance(value, label) for value, label in
                    ((robot_radius_mm, "El radio del robot"), (safety_margin_mm, "El margen de seguridad"),
                     (tracking_margin_mm, "La reserva de seguimiento")))
    view = clearance if camera_clearance_mm is None else _clearance(camera_clearance_mm, "El margen visible")
    segment_length = _finite(max_segment_m, "La longitud de tramo")
    if not .005 <= segment_length <= 1.:
        raise ValueError("La longitud de tramo debe estar entre 5 mm y 1 m.")
    plane = _PlaneMap.build(corners, calibration, obstacles_px)
    deadline = time.monotonic() + MAX_PLANNING_SECONDS
    start, goal = plane.world(start_px), plane.world(goal_px)
    for label, point in (("El robot", start), ("El destino", goal)):
        camera_distance, obstacle_distance = plane.point_clearances(point)
        if obstacle_distance <= clearance + _DISTANCE_TOLERANCE_M:
            raise ValueError(f"{label} está dentro del margen de un obstáculo: distancia estimada "
                             f"{obstacle_distance * 1000.:.0f} mm; se necesitan {clearance * 1000.:.0f} mm.")
        if camera_distance <= view + _DISTANCE_TOLERANCE_M:
            raise ValueError(f"{label} está demasiado cerca del borde de la imagen: distancia estimada "
                             f"{camera_distance * 1000.:.0f} mm; se necesitan {view * 1000.:.0f} mm.")
    blocked, minimum, step, outlines = _grid(plane, clearance, deadline, view)
    if plane.segment_clear(start, goal, clearance, view):
        path = [start, goal]
    else:
        first = _grid_anchor(start, blocked, minimum, step, plane, clearance, view)
        last = _grid_anchor(goal, blocked, minimum, step, plane, clearance, view)
        indices = _astar(blocked, first, last, step, deadline)
        columns = blocked.shape[1]
        points = [start] + [minimum + np.array([node % columns, node // columns]) * step for node in indices] + [goal]
        path = _simplify(points, plane, clearance, deadline, view)
    subdivided = [path[0]]
    length = 0.
    for a, b in zip(path, path[1:]):
        distance = float(np.linalg.norm(b - a))
        length += distance
        parts = max(1, math.ceil(distance / segment_length))
        for index in range(1, parts + 1):
            subdivided.append(a + (b - a) * (index / parts))
    if len(subdivided) > 4096:
        raise ValueError("El camino requiere demasiados tramos. Reduce el área visible.")
    if any(not plane.segment_clear(a, b, clearance, view) for a, b in zip(subdivided, subdivided[1:])):
        raise ValueError("El camino calculado no conserva la distancia de seguridad.")
    return {"path_px": [plane.image(point) for point in subdivided],
            "inflated_obstacles_px": outlines, "path_length_m": length,
            "clearance_mm": clearance * 1000., "camera_clearance_mm": view * 1000.,
            "grid_step_mm": step * 1000.}
