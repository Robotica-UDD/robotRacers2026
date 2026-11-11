"""Seguimiento continuo de una polilínea con predicción de movimiento local.

No abre dispositivos ni envía órdenes. El llamador entrega la misma ruta en
coordenadas del robot en cada imagen: X hacia delante, Y hacia la izquierda.
La ruta incluye su vértice inicial fijo; no se sustituye por la posición actual.
Las salidas son velocidades de rueda en mm/s, no valores de PWM.
"""
from __future__ import annotations

import math
from numbers import Real


_INT32_MAX = 2_147_483_647


def _number(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        result = float(value)
    except (ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _clamp(value, low, high):
    return max(low, min(high, value))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PathFollower:
    """Persigue un punto adelantado sin detenerse en los vértices intermedios.

    La curvatura limita la velocidad por aceleración lateral. La velocidad
    angular se filtra y su variación está acotada. El llamador conserva su rampa
    de ruedas y valida también la orden que esa rampa aplicará realmente.
    Cuando la primera propuesta no cabe, se prueban hasta doce movimientos de
    menor alcance o giros locales mediante is_clear(left, right, horizon).
    Un giro no necesita un período de confirmación parado antes de avanzar.
    """

    HALF_TRACK_MM = 95.
    LOOKAHEAD_MIN_M = .070
    LOOKAHEAD_MAX_M = .180
    LOOKAHEAD_SECONDS = .20
    LATERAL_ACCELERATION_M_S2 = .8
    BRAKING_MM_S2 = 450.
    ANGULAR_ACCELERATION_RAD_S2 = math.radians(240.)
    BEARING_FILTER_SECONDS = .07
    BEARING_DEADBAND_RAD = math.radians(.75)
    PIVOT_ENTRY_RAD = math.radians(80.)
    PIVOT_EXIT_RAD = math.radians(38.)
    MIN_FORWARD_MM_S = 25.
    MIN_PIVOT_RAD_S = math.radians(12.)
    HORIZON_SECONDS = .20
    MAX_CANDIDATES = 12

    def __init__(self):
        self.reset()

    def reset(self):
        self._last_now = None
        self._segment = 0
        self._path_size = None
        self._bearing = None
        self._angular = 0.
        self._linear = 0.
        self._pivoting = False
        self._pivot_sign = 1.

    @staticmethod
    def _result(left=0, right=0, *, phase="advancing", done=False, blocked=False,
                message="", target=None, lookahead=0., remaining=0., index=0, checks=0,
                goal_remaining=None):
        left = round(_clamp(left, -_INT32_MAX, _INT32_MAX))
        right = round(_clamp(right, -_INT32_MAX, _INT32_MAX))
        return {"left": left, "right": right, "phase": phase, "done": done,
                "blocked": blocked, "replan": blocked, "message": message,
                "target_local_m": None if target is None else list(target),
                "lookahead_m": lookahead, "remaining_m": remaining,
                "goal_remaining_m": remaining if goal_remaining is None else goal_remaining,
                "path_index": index, "candidate_count": checks,
                "linear_mm_s": (left + right) / 2.,
                "angular_deg_s": math.degrees((right - left) / 190.)}

    def _invalid(self, message):
        self.reset()
        return self._result(phase="invalid", blocked=True, message=message)

    @staticmethod
    def _projection(a, b, point=(0., 0.)):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return a, math.hypot(point[0] - a[0], point[1] - a[1]), 0.
        ux, uy = dx / length, dy / length
        along = _clamp((point[0] - a[0]) * ux + (point[1] - a[1]) * uy, 0., length)
        projected = (a[0] + along * ux, a[1] + along * uy)
        return projected, math.hypot(point[0] - projected[0], point[1] - projected[1]), along

    def _target(self, points, lookahead, start_at_index=0):
        if len(points) == 1:
            return points[0], math.hypot(*points[0]), 0
        self._segment = max(start_at_index, min(self._segment, len(points) - 2))
        # Solo el segmento actual y el siguiente: no saltar una vuelta de la
        # misión porque un tramo futuro pase otra vez cerca de la posición.
        options = []
        for index in range(self._segment, len(points) - 1):
            if math.hypot(points[index + 1][0] - points[index][0],
                          points[index + 1][1] - points[index][1]) <= 1e-8:
                continue
            projected, distance, _ = self._projection(points[index], points[index + 1])
            options.append((distance, index, projected))
            if len(options) == 2:
                break
        if not options:
            return points[-1], math.hypot(*points[-1]), len(points) - 1
        _, index, projected = min(options)
        self._segment = max(self._segment, index)
        remaining = math.hypot(points[index + 1][0] - projected[0], points[index + 1][1] - projected[1])
        remaining += sum(math.hypot(b[0] - a[0], b[1] - a[1])
                         for a, b in zip(points[index + 1:], points[index + 2:]))
        before, travel = projected, lookahead
        for endpoint in points[index + 1:]:
            distance = math.hypot(endpoint[0] - before[0], endpoint[1] - before[1])
            if distance >= travel and distance > 1e-9:
                ratio = travel / distance
                return (before[0] + (endpoint[0] - before[0]) * ratio,
                        before[1] + (endpoint[1] - before[1]) * ratio), remaining, index + 1
            travel -= distance
            before = endpoint
        return points[-1], remaining, len(points) - 1

    @staticmethod
    def _prediction(linear_mm_s, angular_rad_s, horizon):
        speed = linear_mm_s / 1000.
        angle = angular_rad_s * horizon
        if abs(angular_rad_s) < 1e-8:
            return speed * horizon, 0., angle
        return (speed / angular_rad_s * math.sin(angle),
                speed / angular_rad_s * (1. - math.cos(angle)), angle)

    def update(self, path_local_m, now, speeds, *, is_clear=None, goal_covered=False,
               sequence=None, previous_wheels=None, stop_at_index=None, start_at_index=0):
        """Devuelve una orden continua usando una nueva proyección de la ruta.

        El índice devuelto señala el próximo vértice y nunca retrocede mientras
        el llamador mantenga la misma ruta. Llamar reset al sustituirla. Pasar
        goal_covered únicamente al terminar la misión, no en un punto intermedio.
        previous_wheels permite usar la última consigna aplicada por la rampa.
        stop_at_index limita el punto adelantado al destino de usuario pendiente
        hasta que el llamador confirme su cobertura. Aumentar ese índice no
        reinicia el filtro ni impone una parada. remaining_m sigue midiendo toda
        la ruta; goal_remaining_m mide el recorrido hasta ese límite.
        start_at_index es el vértice del último destino ya cubierto. Impide
        volver a perseguir su tramo de entrada cuando la ruta regresa por la
        misma línea; actualizarlo tampoco reinicia los filtros.
        """
        stamp = _number(now)
        if stamp is None or stamp < 0 or self._last_now is not None and stamp < self._last_now:
            return self._invalid("Se necesita una medición temporal válida.")
        if type(goal_covered) is not bool or not isinstance(speeds, dict):
            return self._invalid("No se pudo interpretar el estado del recorrido.")
        forward, turn = (_number(speeds.get(key)) for key in ("forward_mm_s", "turn_deg_s"))
        if forward is None or turn is None or forward <= 0 or turn <= 0:
            return self._invalid("Se necesita una velocidad válida para continuar.")
        if not isinstance(path_local_m, (list, tuple)) or not 1 <= len(path_local_m) <= 4096:
            return self._invalid("Se necesita una ruta válida para continuar.")
        points = []
        for point in path_local_m:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                return self._invalid("La ruta contiene una coordenada inválida.")
            values = [_number(value) for value in point]
            if any(value is None for value in values):
                return self._invalid("La ruta contiene una coordenada inválida.")
            points.append(tuple(values))
        if (stop_at_index is not None and
                (type(stop_at_index) is not int or not 0 <= stop_at_index < len(points))):
            return self._invalid("El destino pendiente no pertenece a la ruta.")
        limit_index = len(points) - 1 if stop_at_index is None else stop_at_index
        if type(start_at_index) is not int or not 0 <= start_at_index <= limit_index:
            return self._invalid("El inicio del tramo pendiente no pertenece a la ruta.")
        active_points = points[:limit_index + 1]
        if self._path_size is not None and self._path_size != len(points):
            self._segment = 0
        self._path_size = len(points)
        dt = .05 if self._last_now is None else _clamp(stamp - self._last_now, 0., .10)
        self._last_now = stamp
        if goal_covered:
            self._angular = self._linear = 0.
            return self._result(phase="arrived", done=True, message="Recorrido completado.")
        previous_linear, previous_angular = self._linear, self._angular
        if previous_wheels is not None:
            try:
                previous = [_number(value) for value in previous_wheels]
                if len(previous) != 2 or any(value is None for value in previous):
                    return self._invalid("La consigna anterior no es válida.")
                previous_linear = (previous[0] + previous[1]) / 2.
                previous_angular = (previous[1] - previous[0]) / 190.
            except TypeError:
                return self._invalid("La consigna anterior no es válida.")
        lookahead = _clamp(self.LOOKAHEAD_MIN_M + max(0., previous_linear) / 1000. * self.LOOKAHEAD_SECONDS,
                           self.LOOKAHEAD_MIN_M, self.LOOKAHEAD_MAX_M)
        if start_at_index == limit_index:
            target, goal_remaining, index = points[limit_index], math.hypot(*points[limit_index]), limit_index
        else:
            target, goal_remaining, index = self._target(active_points, lookahead, start_at_index)
        remaining = goal_remaining + sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                         for a, b in zip(points[limit_index:], points[limit_index + 1:]))
        if not math.isfinite(remaining) or not all(math.isfinite(value) for value in target):
            return self._invalid("No se pudo resolver la geometría del recorrido.")
        target_distance = math.hypot(*target)
        if goal_remaining < .008 and math.hypot(*active_points[-1]) < .008:
            if limit_index < len(points) - 1:
                return self._result(phase="advancing", target=target, index=index,
                                    remaining=remaining, goal_remaining=goal_remaining,
                                    lookahead=lookahead, message="Punto pendiente alcanzado.")
            self._angular = self._linear = 0.
            return self._result(phase="arrived", done=True, target=target, index=index,
                                remaining=remaining, goal_remaining=goal_remaining,
                                lookahead=lookahead)
        bearing = math.atan2(target[1], target[0])
        if self._bearing is None:
            self._bearing = bearing
        else:
            alpha = -math.expm1(-dt / self.BEARING_FILTER_SECONDS)
            self._bearing = _wrap(self._bearing + alpha * _wrap(bearing - self._bearing))
        error = self._bearing
        if self._pivoting:
            self._pivoting = abs(error) > self.PIVOT_EXIT_RAD
        elif abs(error) > self.PIVOT_ENTRY_RAD:
            self._pivoting = True
            self._pivot_sign = 1. if error >= 0 else -1.
        if self._pivoting and abs(error) < math.radians(120.):
            self._pivot_sign = 1. if error >= 0 else -1.
        max_angular = math.radians(turn)
        if self._pivoting:
            rate = min(max_angular, max(self.MIN_PIVOT_RAD_S, abs(error) * 2.),
                       math.sqrt(2. * self.ANGULAR_ACCELERATION_RAD_S2 * abs(error)))
            desired_linear = 0.
            # Cerca de ±180°, décimas de píxel pueden cambiar el signo del
            # ángulo sin cambiar el lado elegido para completar la media vuelta.
            desired_angular = self._pivot_sign * rate
        else:
            corrected = math.copysign(max(0., abs(error) - self.BEARING_DEADBAND_RAD), error)
            curvature = 2. * math.sin(corrected) / max(target_distance, .025)
            curve_speed = (math.sqrt(self.LATERAL_ACCELERATION_M_S2 / abs(curvature)) * 1000.
                           if abs(curvature) > 1e-8 else forward)
            turn_speed = (max_angular / abs(curvature) * 1000.
                          if abs(curvature) > 1e-8 else forward)
            braking_speed = math.sqrt(2. * self.BRAKING_MM_S2 * max(0., goal_remaining * 1000.))
            desired_linear = min(forward, curve_speed, turn_speed,
                                 max(self.MIN_FORWARD_MM_S, braking_speed))
            desired_linear *= max(.25, math.cos(error))
            desired_angular = _clamp(desired_linear / 1000. * curvature, -max_angular, max_angular)
        # El robot es diferencial: una rueda puede invertir su sentido mientras
        # el centro sigue avanzando. Limitar primero ambas ruedas con el mismo
        # factor conserva el radio solicitado, incluso por debajo de95mm.
        desired_limit = (forward if desired_linear > 0 else
                         max(forward, self.HALF_TRACK_MM * max_angular))
        wheel_peak = max(abs(desired_linear - self.HALF_TRACK_MM * desired_angular),
                         abs(desired_linear + self.HALF_TRACK_MM * desired_angular), 1.)
        desired_scale = min(1., desired_limit / wheel_peak)
        desired_linear *= desired_scale
        desired_angular *= desired_scale
        angular_change = self.ANGULAR_ACCELERATION_RAD_S2 * dt
        angular = _clamp(desired_angular, previous_angular - angular_change, previous_angular + angular_change)
        if desired_linear > 0 and angular * desired_angular < 0:
            # Frenar el giro anterior antes de avanzar hacia el lado opuesto.
            # Mantener toda la velocidad lineal durante esa inversión crea un
            # arco que se aleja del punto adelantado y puede cerrar una vuelta.
            desired_linear = 0.
        elif (desired_linear > 0 and abs(desired_angular) > 1e-8
                and abs(angular) < abs(desired_angular)):
            # Al aumentar la velocidad angular, no avanzar a una velocidad que
            # convierta el giro previsto en un arco mucho más abierto.
            desired_linear *= abs(angular / desired_angular)

        def wheels(linear, angular):
            left, right = linear - self.HALF_TRACK_MM * angular, linear + self.HALF_TRACK_MM * angular
            limit = forward if linear > 0 else max(forward, self.HALF_TRACK_MM * max_angular)
            scale = min(1., limit / max(abs(left), abs(right), 1.))
            return round(left * scale), round(right * scale)

        checks = 0
        tested = set()

        def feasible(linear, angular):
            nonlocal checks
            pair = wheels(linear, angular)
            if pair in tested or checks >= self.MAX_CANDIDATES or pair == (0, 0):
                return None
            tested.add(pair)
            checks += 1
            if is_clear is None:
                return pair
            try:
                return pair if is_clear(pair[0], pair[1], self.HORIZON_SECONDS) else None
            except Exception:
                return None

        chosen = feasible(desired_linear, angular)
        selected_half_turn_sign = None
        if chosen is None:
            alternatives = []
            for fraction in (.65, .35, .20):
                alternatives.append((desired_linear * fraction, angular * fraction, None))
            if desired_linear > 0:
                for rate in (0., angular * .5, angular * 1.5):
                    alternatives.append((desired_linear * .40,
                                         _clamp(rate, -max_angular, max_angular), None))
            pivot_rate = min(max_angular, max(self.MIN_PIVOT_RAD_S, min(abs(error) * 2., math.radians(60.))))
            sign = self._pivot_sign if self._pivoting else (1. if error >= 0 else -1.)
            # Un giro contrario al punto adelantado puede ser libre durante
            # unos grados, pero oscilar contra el mismo borde indefinidamente.
            # Esa salida requiere una maniobra global de recuperación, no un
            # pivote local que aleje el rumbo de la trayectoria.
            alternatives.extend(((0., sign * pivot_rate, None), (0., sign * pivot_rate * .5, None)))
            if abs(error) >= math.radians(150.):
                # Para un destino casi a180°, ambos sentidos acercan el rumbo.
                # El lado libre puede ser el contrario al signo de atan2.
                alternatives.extend(((0., -sign * pivot_rate, -sign),
                                     (0., -sign * pivot_rate * .5, -sign)))
            scored = []
            for linear, rate, half_turn_sign in alternatives:
                pair = feasible(linear, rate)
                if pair is None:
                    continue
                actual_linear = sum(pair) / 2.
                actual_angular = (pair[1] - pair[0]) / 190.
                x, y, angle = self._prediction(actual_linear, actual_angular, self.HORIZON_SECONDS)
                distance = math.hypot(target[0] - x, target[1] - y)
                cross_track = min(self._projection(a, b, (x, y))[1]
                                  for a, b in zip(active_points[start_at_index:],
                                                  active_points[start_at_index + 1:])) if start_at_index < limit_index else distance
                score = 4. * distance + 2. * cross_track + .08 * abs(_wrap(error - angle))
                score += .025 * abs(actual_angular - previous_angular)
                score += .12 if actual_linear <= 0 else 0.
                scored.append((score, pair, half_turn_sign))
            if scored:
                _, chosen, selected_half_turn_sign = min(scored, key=lambda value: value[0])
        if chosen is None:
            self._angular = self._linear = 0.
            return self._result(phase="blocked", blocked=True,
                message="Buscando una dirección con espacio para continuar.", target=target,
                lookahead=lookahead, remaining=remaining, goal_remaining=goal_remaining,
                index=index, checks=checks)
        self._linear = sum(chosen) / 2.
        self._angular = (chosen[1] - chosen[0]) / 190.
        if self._pivoting and selected_half_turn_sign is not None:
            # Solo cambiar de hemisferio cuando la predicción haya elegido
            # expresamente el pivote alternativo. La velocidad residual de un
            # giro anterior no debe invertir por sí sola el lado solicitado.
            self._pivot_sign = selected_half_turn_sign
        phase = "advancing" if self._linear > 0 else "turning"
        return self._result(*chosen, phase=phase,
            message="Siguiendo el recorrido…" if phase == "advancing" else "Cambiando de dirección…",
            target=target, lookahead=lookahead, remaining=remaining, goal_remaining=goal_remaining,
            index=index, checks=checks)
