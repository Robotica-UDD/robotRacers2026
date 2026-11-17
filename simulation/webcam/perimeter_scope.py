"""Pertenencia a recintos cerrados en píxeles, sin rejilla ni escala métrica.

Los segmentos de un perímetro válido forman anillos de vértices de grado dos.
Las paredes abiertas siguen siendo colisiones de Navigation, pero no definen
por sí solas un recinto cerrado. La paridad se compara por anillo: dos recintos
disjuntos no se confunden por tener ambos un número impar de contornos.
"""
from __future__ import annotations

import math
from numbers import Real


def _number(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("Las coordenadas y la tolerancia deben ser números finitos.")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise ValueError("Las coordenadas y la tolerancia deben ser números finitos.") from None
    if not math.isfinite(result):
        raise ValueError("Las coordenadas y la tolerancia deben ser números finitos.")
    return result


def _point(value):
    try:
        if isinstance(value, (str, bytes, dict)) or len(value) != 2:
            raise ValueError()
        return (_number(value[0]), _number(value[1]))
    except (TypeError, IndexError, KeyError):
        raise ValueError("Cada punto del perímetro necesita dos coordenadas finitas.") from None


def _closed_loops(obstacles, tolerance):
    if not isinstance(obstacles, (list, tuple)) or len(obstacles) > 512:
        raise ValueError("El perímetro debe contener hasta 512 paredes o contornos.")
    vertices, buckets, edges = [], {}, set()

    def vertex(point):
        if tolerance == 0.:
            key = point
            if key not in buckets:
                buckets[key] = len(vertices)
                vertices.append(point)
            return buckets[key]
        try:
            key = (math.floor(point[0] / tolerance), math.floor(point[1] / tolerance))
        except (ValueError, OverflowError):
            raise ValueError("Las coordenadas del perímetro no se pueden normalizar.") from None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for index in buckets.get((key[0] + dx, key[1] + dy), ()):
                    if math.dist(point, vertices[index]) <= tolerance:
                        return index
        index = len(vertices)
        vertices.append(point)
        buckets.setdefault(key, []).append(index)
        return index

    for wall in obstacles:
        if not isinstance(wall, (list, tuple)) or len(wall) not in (2, 4):
            raise ValueError("Cada pared necesita dos extremos o cuatro vértices.")
        points = [_point(value) for value in wall]
        pairs = [(points[0], points[1])] if len(points) == 2 else list(zip(points, points[1:] + points[:1]))
        for first, last in pairs:
            a, b = vertex(first), vertex(last)
            if a == b:
                raise ValueError("Los extremos de una pared no pueden coincidir.")
            edges.add(tuple(sorted((a, b))))
    neighbors = [set() for _ in vertices]
    for a, b in edges:
        neighbors[a].add(b)
        neighbors[b].add(a)
    loops, visited = [], set()
    for first in range(len(vertices)):
        if first in visited:
            continue
        pending, component = [first], set()
        while pending:
            node = pending.pop()
            if node in component:
                continue
            component.add(node)
            pending.extend(neighbors[node] - component)
        visited.update(component)
        if len(component) < 3 or any(len(neighbors[node]) != 2 for node in component):
            continue
        previous, current, loop = None, first, []
        while True:
            loop.append(vertices[current])
            following = next(node for node in neighbors[current] if node != previous)
            previous, current = current, following
            if current == first:
                break
        area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(loop, loop[1:] + loop[:1]))
        if abs(area) > tolerance * tolerance:
            loops.append(loop)
    return loops


def _membership(point, loop, tolerance):
    inside = False
    x, y = point
    for first, last in zip(loop, loop[1:] + loop[:1]):
        dx, dy = last[0] - first[0], last[1] - first[1]
        length2 = dx * dx + dy * dy
        along = max(0., min(1., ((x - first[0]) * dx + (y - first[1]) * dy) / length2))
        if math.hypot(x - first[0] - along * dx, y - first[1] - along * dy) <= tolerance:
            return None  # El borde pertenece al cierre de cualquiera de sus dos lados.
        if (first[1] > y) != (last[1] > y):
            crossing_x = first[0] + (y - first[1]) * dx / dy
            if x < crossing_x:
                inside = not inside
    return inside


def same_perimeter_region(sample_px, center_px, obstacles_px, *, tolerance_px=1e-5):
    """Indica si ambos puntos pertenecen a la misma región de anillos cerrados.

    Acepta contornos cóncavos, anidados, disjuntos y extremos con ruido menor
    que ``tolerance_px``. No exige que toda la caja de una mano quede dentro.
    Sin anillos cerrados devuelve True; el consumidor comprueba la imagen.
    No abre dispositivos, no proyecta metros ni autoriza movimiento.
    """
    sample, center = _point(sample_px), _point(center_px)
    tolerance = _number(tolerance_px)
    if tolerance < 0.:
        raise ValueError("La tolerancia del perímetro no puede ser negativa.")
    for loop in _closed_loops(obstacles_px, tolerance):
        sample_inside = _membership(sample, loop, tolerance)
        center_inside = _membership(center, loop, tolerance)
        if sample_inside is not None and center_inside is not None and sample_inside != center_inside:
            return False
    return True
