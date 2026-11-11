"""Recorridos rectilíneos con pocos giros y preferencia por espacio libre.

Este módulo solo propone maniobras. La escena compartida comprueba cada giro
y cada traslación con la huella completa. Las separaciones son costes suaves:
un pasillo estrecho sigue siendo utilizable si el robot cabe realmente.
"""
from __future__ import annotations

import heapq
import math
import time

import numpy as np


PREFERRED_OBSTACLE_GAP_M = .070
PREFERRED_CAMERA_GAP_M = .100
TURN_COST = .35
DISTANCE_COST = .25
MAX_LANES_PER_AXIS = 18
MAX_EXPANDED_STATES = 900


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _floor_axes(scene, reference=None):
    """El borde largo de una pared fija los ejes del suelo, no la cámara."""
    edges = []
    for polygon in scene.plane.obstacles:
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            edge = b - a
            length = float(edge @ edge)
            edges.append((length, edge, a, b))
    longest = max(edges, key=lambda v: v[0]) if edges else None
    if longest is not None and reference is not None:
        candidates = [item for item in edges if item[0] >= longest[0] * .6]
        def distance(item):
            _, edge, a, _ = item
            fraction = np.clip(float((reference - a) @ edge) / item[0], 0., 1.)
            return float(np.linalg.norm(reference - a - fraction * edge))
        longest = min(candidates, key=distance)
    angle = 0. if longest is None else math.atan2(float(longest[1][1]), float(longest[1][0]))
    # La misma familia perpendicular de ejes, con la dirección horizontal
    # orientada de izquierda a derecha y sin depender del orden del polígono.
    angle = (angle + math.pi / 4.) % (math.pi / 2.) - math.pi / 4.
    c, s = math.cos(angle), math.sin(angle)
    return angle, np.array([[c, -s], [s, c]])


def _lanes(frame, obstacles, start, goal, radius, axis):
    low, high = float(np.min(frame[:, axis])), float(np.max(frame[:, axis]))
    mandatory = [float(start[axis]), float(goal[axis])]
    candidates = [(low + high) / 2., low + radius + .07, high - radius - .07]
    intervals = [(low, low), (high, high)]
    for polygon in obstacles:
        a, b = float(np.min(polygon[:, axis])), float(np.max(polygon[:, axis]))
        intervals.append((a, b))
        for pad in (radius + PREFERRED_OBSTACLE_GAP_M, radius + .005):
            candidates.extend((a - pad, b + pad))
    intervals.sort()
    merged = []
    for a, b in intervals:
        if merged and a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    for first, last in zip(merged, merged[1:]):
        if last[0] > first[1]:
            candidates.append((first[1] + last[0]) / 2.)
    candidates = [v for v in candidates if low + 1e-6 < v < high - 1e-6]
    candidates.sort(key=lambda v: (min(abs(v - x) for x in mandatory), abs(v - (low + high) / 2.), v))
    selected = mandatory[:]
    for value in candidates:
        if len(selected) >= MAX_LANES_PER_AXIS:
            break
        if all(abs(value - old) > .012 for old in selected):
            selected.append(value)
    return sorted(set(selected))


def _risk(scene, pose, obstacle_boxes):
    polygon = scene.polygon(pose)
    low, high = np.min(polygon, axis=0), np.max(polygon, axis=0)
    lows, highs = obstacle_boxes
    gap = math.inf
    if len(lows):
        delta = np.maximum(np.maximum(lows - high, low - highs), 0.)
        gap = float(np.min(np.linalg.norm(delta, axis=1)))
    camera_gap = float(np.min(polygon @ scene.camera_normals.T - scene.camera_offsets))
    return (3. * math.exp(-gap / PREFERRED_OBSTACLE_GAP_M)
            + 2. * math.exp(-max(0., camera_gap) / PREFERRED_CAMERA_GAP_M))


def orthogonal_route(scene, start, goal, tolerance, deadline):
    """Devuelve acciones de frente con giros ortogonales, o ``None``.

    ``scene`` y ``start`` pertenecen al planificador de poses. No se importa ese
    módulo para evitar una dependencia circular. El plazo se comparte con el
    llamador y no se envían órdenes al robot.
    """
    previous_deadline = scene.deadline
    scene.deadline = deadline if previous_deadline is None else min(deadline, previous_deadline)
    try:
        return _orthogonal_route(scene, start, goal, tolerance, deadline)
    finally:
        scene.deadline = previous_deadline


def _orthogonal_route(scene, start, goal, tolerance, deadline):
    if time.monotonic() >= deadline:
        return None
    pose_type = type(start)
    angle, rotation = _floor_axes(scene, (start.point + np.asarray(goal)) / 2.)
    first = start.point @ rotation
    last = np.asarray(goal) @ rotation
    if float(np.linalg.norm(last - first)) < .0005:
        return []
    frame = scene.plane.frame @ rotation
    obstacles = [polygon @ rotation for polygon in scene.plane.obstacles]
    xs = _lanes(frame, obstacles, first, last, scene.radius, 0)
    ys = _lanes(frame, obstacles, first, last, scene.radius, 1)
    obstacle_boxes = (np.asarray([np.min(p, axis=0) for p in scene.plane.obstacles]),
                      np.asarray([np.max(p, axis=0) for p in scene.plane.obstacles]))
    risk_cache, edge_cache = {}, {}

    def pose(point, yaw):
        world = np.asarray(point) @ rotation.T
        return pose_type(float(world[0]), float(world[1]), _wrap(yaw + angle))

    def risk(value):
        key = (round(value.x, 6), round(value.y, 6), round(value.yaw, 6))
        if key not in risk_cache:
            risk_cache[key] = _risk(scene, value, obstacle_boxes)
        return risk_cache[key]

    def translation_risk(before, following):
        distance = float(np.linalg.norm(following.point - before.point))
        samples = max(1, min(32, int(math.ceil(distance / .05))))
        return sum(risk(pose_type(before.x + (following.x - before.x) * (i + .5) / samples,
                                  before.y + (following.y - before.y) * (i + .5) / samples,
                                  following.yaw)) for i in range(samples)) / samples

    def turn_options(value, desired):
        delta = _wrap(desired - value.yaw)
        if abs(delta) < 1e-7:
            return [[]]
        signs = (1., -1.) if abs(abs(delta) - math.pi) < 1e-6 else (1. if delta > 0 else -1.,)
        results = []
        for sign in signs:
            pieces = max(1, int(math.ceil(abs(delta) / (math.pi / 2.) - 1e-8)))
            actions, before = [], value
            for i in range(1, pieces + 1):
                following = pose_type(value.x, value.y, _wrap(value.yaw + sign * abs(delta) * i / pieces))
                if time.monotonic() >= deadline or not scene.swept(before, following, tolerance=tolerance)[0]:
                    actions = None
                    break
                actions.append((following, 'turn', 1, False))
                before = following
            if actions is not None:
                results.append(actions)
        return results

    def path_actions(points):
        actions, before = [], start
        for point in points[1:]:
            world = np.asarray(point) @ rotation.T
            delta = world - before.point
            if np.linalg.norm(delta) < 1e-7:
                continue
            yaw = math.atan2(float(delta[1]), float(delta[0]))
            options = turn_options(before, yaw)
            if not options:
                return None
            turns = min(options, key=lambda v: sum(risk(item[0]) for item in v))
            actions.extend(turns)
            turned = turns[-1][0] if turns else before
            following = pose_type(float(world[0]), float(world[1]), yaw)
            if time.monotonic() >= deadline or not scene.swept(turned, following, tolerance=tolerance)[0]:
                return None
            actions.append((following, 'drive', 1, False))
            before = following
        return actions

    def estimate(points):
        total, heading = 0., _wrap(start.yaw - angle)
        for a, b in zip(points, points[1:]):
            if time.monotonic() >= deadline:
                return math.inf
            distance = float(np.linalg.norm(b - a))
            if distance < 1e-7:
                continue
            yaw = math.atan2(float(b[1] - a[1]), float(b[0] - a[0]))
            change = abs(_wrap(yaw - heading))
            total += TURN_COST * change / (math.pi / 2.)
            total += distance * (DISTANCE_COST + translation_risk(pose(a, yaw), pose(b, yaw)))
            heading = yaw
        return total

    # Probar primero recorridos sencillos de una, dos o tres rectas. Ordenarlos
    # por espacio y giros evita aceptar el primer atajo pegado a una pared.
    proposals = []
    delta = last - first
    bearing = math.atan2(float(delta[1]), float(delta[0]))
    if abs(_wrap(bearing - round(bearing / (math.pi / 2.)) * math.pi / 2.)) <= math.radians(5.):
        proposals.append([first, last])
    proposals.extend(([first, np.array([last[0], first[1]]), last],
                      [first, np.array([first[0], last[1]]), last]))
    proposals.extend([first, np.array([x, first[1]]), np.array([x, last[1]]), last] for x in xs)
    proposals.extend([first, np.array([first[0], y]), np.array([last[0], y]), last] for y in ys)
    ranked = sorted((estimate(points), i, points) for i, points in enumerate(proposals))
    # Las propuestas ya comparan separación y giros; la primera válida es la
    # mejor de esta familia. La búsqueda general queda para rodeos complejos.
    for _, _, points in ranked:
        if time.monotonic() >= deadline:
            return None
        actions = path_actions(points)
        if actions is not None:
            return actions

    # A* sobre carriles y cuatro orientaciones. Cada arista se verifica con la
    # misma huella barrida que emplea el control; no hay diagonales ni reversa.
    source = (xs.index(float(first[0])), ys.index(float(first[1])))
    destination = (xs.index(float(last[0])), ys.index(float(last[1])))
    costs, parents, heap = {}, {}, []
    serial = 0
    for q in range(4):
        yaw = _wrap(angle + q * math.pi / 2.)
        options = turn_options(start, yaw)
        if not options:
            continue
        initial = min(options, key=lambda v: len(v))
        cost = TURN_COST * abs(_wrap(yaw - start.yaw)) / (math.pi / 2.)
        key = (*source, q)
        costs[key], parents[key] = cost, (None, initial)
        serial += 1
        heuristic = DISTANCE_COST * (abs(last[0] - first[0]) + abs(last[1] - first[1]))
        heapq.heappush(heap, (cost + heuristic, cost, serial, key))
    expanded = 0
    while heap and expanded < MAX_EXPANDED_STATES and time.monotonic() < deadline:
        _, cost, _, key = heapq.heappop(heap)
        if cost > costs.get(key, math.inf) + 1e-10:
            continue
        i, j, q = key
        if (i, j) == destination:
            chunks = []
            while key is not None:
                parent, moves = parents[key]
                chunks.append(moves)
                key = parent
            return [move for chunk in reversed(chunks) for move in chunk]
        expanded += 1
        current = pose([xs[i], ys[j]], q * math.pi / 2.)
        next_states = [(i, j, (q + turn) % 4, 'turn') for turn in (-1, 1)]
        dx, dy = ((1, 0), (0, 1), (-1, 0), (0, -1))[q]
        if 0 <= i + dx < len(xs) and 0 <= j + dy < len(ys):
            next_states.append((i + dx, j + dy, q, 'drive'))
        for ni, nj, nq, kind in next_states:
            following = pose([xs[ni], ys[nj]], nq * math.pi / 2.)
            edge = (key, (ni, nj, nq))
            if edge not in edge_cache:
                edge_cache[edge] = scene.swept(current, following, tolerance=tolerance)[0]
            if not edge_cache[edge]:
                continue
            distance = float(np.linalg.norm(following.point - current.point))
            extra = (TURN_COST + .04 * max(risk(current), risk(following)) if kind == 'turn'
                     else distance * (DISTANCE_COST + translation_risk(current, following)))
            total, next_key = cost + extra, (ni, nj, nq)
            if total >= costs.get(next_key, math.inf) - 1e-10:
                continue
            costs[next_key] = total
            parents[next_key] = (key, [(following, kind, 1, False)])
            heuristic = DISTANCE_COST * (abs(xs[ni] - last[0]) + abs(ys[nj] - last[1]))
            serial += 1
            heapq.heappush(heap, (total + heuristic, total, serial, next_key))
    return None
