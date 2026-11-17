"""Seguimiento continuo de una polilínea con predicción de movimiento local.

No abre dispositivos ni envía órdenes. El llamador entrega la misma ruta en
coordenadas del robot en cada imagen: X hacia delante, Y hacia la izquierda.
La ruta incluye su vértice inicial fijo; no se sustituye por la posición actual.
Las salidas son velocidades de rueda en mm/s, no valores de PWM.
"""
from __future__ import annotations

import math
from numbers import Real

from .robot_footprint import TRACK_WIDTH_MM


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

    En rectas ya alineadas, un PD de corrección menor rechaza el ruido de
    orientación sin recortar la máxima de avance. Se mezcla progresivamente
    con la persecución normal al salir del corredor, acercarse a un codo o
    quedar una rotación importante de la consigna anterior. No limita los
    pivotes ni sustituye la comprobación de colisiones.
    """

    HALF_TRACK_MM = TRACK_WIDTH_MM / 2.
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
    STRAIGHT_FILTER_SECONDS = .12
    STRAIGHT_DERIVATIVE_SECONDS = .15
    STRAIGHT_GAIN = .5
    STRAIGHT_DERIVATIVE_GAIN = .12
    STRAIGHT_DEADBAND_RAD = math.radians(1.5)
    STRAIGHT_MAX_RATE_RAD_S = math.radians(10.)
    STRAIGHT_MAX_SLEW_RAD_S2 = math.radians(25.)

    def __init__(self):
        self.reset()

    def reset(self):
        self._last_now = None
        self._segment = 0
        self._path_size = None
        self._path_heading = None
        self._bearing = None
        self._angular = 0.
        self._linear = 0.
        self._pivoting = False
        self._pivot_sign = 1.
        self._straight_bearing = None
        self._straight_stamp = None
        self._straight_sequence = None
        self._straight_derivative = 0.
        self._straight_rate = 0.

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
                "angular_deg_s": math.degrees((right - left) / TRACK_WIDTH_MM)}

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

    @staticmethod
    def _smoothstep(value):
        value = _clamp(value, 0., 1.)
        return value * value * (3. - 2. * value)

    def _straight_weight(self, points, lookahead, error):
        """Suavizar realimentación pequeña, sin recortar curvas planificadas."""
        alignment = 1. - self._smoothstep((abs(error) - math.radians(12.)) / math.radians(20.))
        if alignment <= 0 or len(points) < 2:
            return alignment
        index = min(self._segment, len(points) - 2)
        a, b = points[index:index + 2]
        projected, cross_track, _ = self._projection(a, b)
        if math.hypot(b[0] - a[0], b[1] - a[1]) <= 1e-8:
            return 0.
        corridor = 1. - self._smoothstep((cross_track - .015) / .020)
        if corridor <= 0:
            return 0.
        heading = math.atan2(b[1] - a[1], b[0] - a[0])
        distance = math.hypot(b[0] - projected[0], b[1] - projected[1])
        corner_weight = 1.
        for c, d in zip(points[index + 1:], points[index + 2:]):
            length = math.hypot(d[0] - c[0], d[1] - c[1])
            if length <= 1e-8:
                continue
            next_heading = math.atan2(d[1] - c[1], d[0] - c[0])
            if abs(_wrap(next_heading - heading)) > math.radians(3.):
                # La transición termina antes de que el punto adelantado
                # cruce el codo: el giro intencional conserva su controlador.
                corner_weight = self._smoothstep((distance - lookahead) / max(.04, lookahead * .5))
                break
            distance += length
        return alignment * corridor * corner_weight

    def _straight_correction(self, bearing, stamp, sequence, linear_mm_s,
                             target_distance, max_angular):
        """PD de corrección sobre mediciones: el ruido de yaw no es una orden."""
        cap = min(max_angular, self.STRAIGHT_MAX_RATE_RAD_S,
                  linear_mm_s * .15 / self.HALF_TRACK_MM)
        if self._straight_bearing is None:
            self._straight_bearing = bearing
            self._straight_stamp, self._straight_sequence = stamp, sequence
            self._straight_derivative = self._straight_rate = 0.
            return 0.
        if sequence is not None and sequence == self._straight_sequence:
            return _clamp(self._straight_rate, -cap, cap)
        seconds = stamp - self._straight_stamp
        if seconds <= 0:
            return _clamp(self._straight_rate, -cap, cap)
        if seconds > .5:
            self._straight_bearing = bearing
            self._straight_derivative = self._straight_rate = 0.
            self._straight_stamp, self._straight_sequence = stamp, sequence
            return 0.
        alpha = -math.expm1(-seconds / self.STRAIGHT_FILTER_SECONDS)
        increment = alpha * _wrap(bearing - self._straight_bearing)
        self._straight_bearing = _wrap(self._straight_bearing + increment)
        derivative_alpha = -math.expm1(-seconds / self.STRAIGHT_DERIVATIVE_SECONDS)
        self._straight_derivative += derivative_alpha * (increment / seconds - self._straight_derivative)
        self._straight_stamp, self._straight_sequence = stamp, sequence
        corrected = math.copysign(max(0., abs(self._straight_bearing) - self.STRAIGHT_DEADBAND_RAD),
                                  self._straight_bearing)
        gain = self.STRAIGHT_GAIN * 2. * linear_mm_s / 1000. / max(target_distance, .025)
        proportional = gain * corrected
        derivative = _clamp(self.STRAIGHT_DERIVATIVE_GAIN * self._straight_derivative,
                            -abs(proportional) * .6, abs(proportional) * .6)
        desired = _clamp(proportional + derivative, -cap, cap)
        self._straight_rate += _clamp(desired - self._straight_rate,
                                     -self.STRAIGHT_MAX_SLEW_RAD_S2 * seconds,
                                     self.STRAIGHT_MAX_SLEW_RAD_S2 * seconds)
        self._straight_rate = _clamp(self._straight_rate, -cap, cap)
        return self._straight_rate

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
            self._path_heading = None
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
                previous_angular = (previous[1] - previous[0]) / TRACK_WIDTH_MM
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
        path_heading = next((math.atan2(b[1] - a[1], b[0] - a[0])
                             for a, b in zip(points, points[1:])
                             if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-8), None)
        frame_rotation = (0. if path_heading is None or self._path_heading is None
                          else _wrap(self._path_heading - path_heading))
        self._path_heading = path_heading
        if self._bearing is None:
            self._bearing = bearing
        else:
            # El error se expresa en el marco móvil del robot. Antes de
            # filtrar una nueva imagen hay que retirar el giro observado:
            # filtrar directamente ambos ángulos retrasa también la propia
            # rotación y mantiene la corrección cuando el rumbo ya la alcanzó.
            # Una arista fija de la ruta proporciona esa rotación entre
            # imágenes, sin asumir que las ruedas ejecutaron la consigna.
            predicted_bearing = _wrap(self._bearing - frame_rotation)
            alpha = -math.expm1(-dt / self.BEARING_FILTER_SECONDS)
            self._bearing = _wrap(predicted_bearing + alpha * _wrap(bearing - predicted_bearing))
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
        straight_weight = (0. if self._pivoting else
                           self._straight_weight(active_points, lookahead, error))
        # No confundir el final de un giro rápido con ruido de una recta:
        # mientras quede rotación importante conserva el transporte observado
        # y el frenado angular original, sin añadir la fase de este PD suave.
        straight_weight *= 1. - self._smoothstep(
            (abs(previous_angular) - self.STRAIGHT_MAX_RATE_RAD_S) / math.radians(20.))
        if straight_weight > 0:
            correction = self._straight_correction(bearing, stamp, sequence,
                                                   desired_linear, target_distance, max_angular)
            desired_angular = (1. - straight_weight) * desired_angular + straight_weight * correction
        else:
            self._straight_bearing = self._straight_stamp = self._straight_sequence = None
            self._straight_derivative = self._straight_rate = 0.
        # El robot es diferencial: una rueda puede invertir su sentido mientras
        # el centro sigue avanzando. Limitar primero ambas ruedas con el mismo
        # factor conserva el radio solicitado, incluso por debajo de la semitrocha.
        desired_limit = (forward if desired_linear > 0 else
                         max(forward, self.HALF_TRACK_MM * max_angular))
        wheel_peak = max(abs(desired_linear - self.HALF_TRACK_MM * desired_angular),
                         abs(desired_linear + self.HALF_TRACK_MM * desired_angular), 1.)
        desired_scale = min(1., desired_limit / wheel_peak)
        desired_linear *= desired_scale
        desired_angular *= desired_scale
        angular_change = self.ANGULAR_ACCELERATION_RAD_S2 * dt
        angular = _clamp(desired_angular, previous_angular - angular_change, previous_angular + angular_change)
        gentle_motion = (straight_weight >= .8 and
                         max(abs(previous_angular), abs(desired_angular)) <= self.STRAIGHT_MAX_RATE_RAD_S * 1.5)
        # Una pequeña corrección de recta puede cambiar de signo sin pedir
        # una detención. La regla de frenado se conserva para giros amplios.
        if desired_linear > 0 and angular * desired_angular < 0 and not gentle_motion:
            # Frenar el giro anterior antes de avanzar hacia el lado opuesto.
            # Mantener toda la velocidad lineal durante esa inversión crea un
            # arco que se aleja del punto adelantado y puede cerrar una vuelta.
            desired_linear = 0.
        elif (not gentle_motion and desired_linear > 0 and abs(desired_angular) > 1e-8
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
                actual_angular = (pair[1] - pair[0]) / TRACK_WIDTH_MM
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
        self._angular = (chosen[1] - chosen[0]) / TRACK_WIDTH_MM
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
