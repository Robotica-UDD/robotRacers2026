"""Seguimiento de maniobras planificadas, sin transporte ni acceso al robot.

Las salidas son velocidades firmadas de rueda en mm/s. El consumidor verifica
el barrido de la huella y aplica su rampa; este módulo nunca improvisa un giro
en el sitio para corregir un tramo de avance o retroceso.
"""
from __future__ import annotations

import math
from numbers import Integral, Real

from .straight_control import StraightControl


_INT32_MAX = 2_147_483_647


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        result = float(value)
    except (ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _velocity(value):
    return round(max(-_INT32_MAX, min(_INT32_MAX, value)))


class PoseFollower:
    """Ejecuta un paso fijo del plan; ``reset`` separa tramos y recuperaciones.

    ``heading_error_rad`` es el error angular físico calculado por la geometría
    del planificador. Nunca se resta directamente un ángulo de píxeles a otro.
    El estado ``done`` solicita cero; el consumidor debe esperar además a que
    ambas ruedas se hayan detenido antes de activar el siguiente paso.
    """

    ARRIVAL_M = .008
    TURN_TOLERANCE_RAD = math.radians(3.)
    TURN_HOLD_SECONDS = .10
    MAX_DRIVE_ERROR_RAD = math.radians(18.)
    HALF_TRACK_MM = 95.
    BRAKING_MM_S2 = 450.
    ESCAPE_SPEED_MM_S = 80.
    ESCAPE_TURN_DEG_S = 30.

    def __init__(self):
        self._straight = StraightControl()
        self.reset()

    def reset(self):
        self._straight.reset()
        self._step_key = None
        self._turn_settled_since = None
        self._last_now = None
        self._sequence = 0

    @staticmethod
    def _result(left=0, right=0, *, done=False, phase="adjusting", message="", replan=False):
        return {"left": _velocity(left), "right": _velocity(right), "done": done,
                "phase": phase, "message": message, "replan": replan}

    def _invalid(self, message):
        self.reset()
        return self._result(message=message, replan=True)

    def update(self, step, observation, heading_error_rad, now, speeds):
        stamp = _finite(now)
        if stamp is None or stamp < 0 or (self._last_now is not None and stamp < self._last_now):
            return self._invalid("Se necesita una medición temporal válida para continuar.")
        if not isinstance(step, dict) or not isinstance(observation, dict) or not isinstance(speeds, dict):
            return self._invalid("No se pudo interpretar la maniobra actual.")
        kind, direction = step.get("kind"), step.get("direction", 1)
        if kind not in {"turn", "drive", "escape"}:
            return self._invalid("Se necesita recalcular la maniobra.")
        if type(direction) is not int or direction not in {-1, 1}:
            return self._invalid("La dirección de la maniobra no es válida.")
        point = step.get("target_px")
        if not isinstance(point, (list, tuple)) or len(point) != 2 or any(_finite(value) is None for value in point):
            return self._invalid("El destino de la maniobra no es válido.")
        heading = _finite(step.get("heading_rad"))
        if heading is None:
            return self._invalid("La orientación de la maniobra no es válida.")
        forward, turn = (_finite(speeds.get(key)) for key in ("forward_mm_s", "turn_deg_s"))
        if forward is None or turn is None or forward <= 0 or turn <= 0:
            return self._invalid("Se necesita una velocidad válida para continuar.")
        escaping = step.get("escape") is True or step.get("escaping") is True
        key = (kind, tuple(point), heading, direction, escaping)
        if key != self._step_key:
            self._straight.reset()
            self._turn_settled_since = None
            self._step_key = key
        self._last_now = stamp

        if kind == "turn":
            error = _finite(heading_error_rad)
            if error is None:
                return self._invalid("No se pudo medir la orientación actual.")
            error = _wrap(error)
            if abs(error) <= self.TURN_TOLERANCE_RAD:
                if self._turn_settled_since is None:
                    self._turn_settled_since = stamp
                done = stamp - self._turn_settled_since >= self.TURN_HOLD_SECONDS - 1e-12
                return self._result(done=done, phase="settling", message="Confirmando la orientación…")
            self._turn_settled_since = None
            if escaping:
                turn = min(turn, self.ESCAPE_TURN_DEG_S)
            angle = abs(math.degrees(error))
            braking_rate = math.sqrt(2. * 240. * max(0., angle - 8.))
            rate = max(min(12., turn), min(turn, angle * turn / 40., braking_rate))
            magnitude = min(self.HALF_TRACK_MM * math.radians(turn),
                            max(20., self.HALF_TRACK_MM * math.radians(rate)))
            sign = 1 if error > 0 else -1
            return self._result(-sign * magnitude, sign * magnitude,
                                phase="turning", message="Orientando el robot…")

        distance, bearing = (_finite(observation.get(key)) for key in ("distance_m", "bearing_rad"))
        if distance is None or bearing is None or distance < 0:
            return self._invalid("No se pudo medir la posición actual.")
        if distance <= self.ARRIVAL_M:
            self._straight.reset()
            return self._result(done=True, phase="settling", message="Deteniendo el robot en la maniobra…")
        error = _wrap(bearing - (math.pi if direction < 0 else 0.))
        if abs(error) > self.MAX_DRIVE_ERROR_RAD and distance > .015:
            self._straight.reset()
            return self._result(replan=True, message="Recalculando el recorrido desde la orientación actual…")
        limit = min(_INT32_MAX, forward, self.ESCAPE_SPEED_MM_S if kind == "escape" else _INT32_MAX)
        braking = math.sqrt(2. * self.BRAKING_MM_S2 * max(0., distance * 1000. - self.ARRIVAL_M * 1000.))
        speed = min(limit, max(min(20., limit), braking))
        sequence = observation.get("sequence")
        if isinstance(sequence, bool) or not isinstance(sequence, Integral) or sequence < 0:
            self._sequence += 1
            sequence = self._sequence
        angular = self._straight.update(error, stamp, sequence, speed, distance)
        if kind == "escape":
            angular = max(-math.radians(self.ESCAPE_TURN_DEG_S), min(math.radians(self.ESCAPE_TURN_DEG_S), angular))
        correction = self.HALF_TRACK_MM * angular
        left, right = speed - correction, speed + correction
        scale = min(1., limit / max(abs(left), abs(right)))
        left, right = left * scale, right * scale
        if direction < 0:
            # v negativo conserva el signo de omega: negar ambas ruedas lo invertiría.
            left, right = -right, -left
        return self._result(left, right, phase="advancing",
                            message="Retrocediendo por el recorrido…" if direction < 0 else "Avanzando por el recorrido…")
