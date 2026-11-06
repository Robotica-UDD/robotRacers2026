"""Corrección suave del rumbo durante el avance, independiente del hardware.

El error positivo indica que el objetivo queda a la izquierda. La salida es una
velocidad angular en rad/s; para una separación de ruedas de 190 mm corresponde
a una corrección diferencial de ``95 * salida`` mm/s por rueda. Este controlador
no calcula giros en el sitio ni determina la llegada al objetivo.
"""
from __future__ import annotations

import math
from numbers import Integral, Real


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        value = float(value)
    except (ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def _limit(value, magnitude):
    return max(-magnitude, min(magnitude, value))


class StraightControl:
    """Control proporcional y derivativo filtrado para un único tramo de avance.

    Llamar ``reset()`` al cambiar de objetivo, abandonar el avance o recuperar
    una pérdida larga. El primer fotograma establece la referencia y devuelve
    cero. Un fotograma repetido conserva la última salida sin integrar tiempo.
    Los datos inválidos o una discontinuidad temporal reinician el control y
    devuelven cero. No contiene integrador ni realiza IO.

    La derivada se suma al término proporcional: si disminuye un error positivo,
    su contribución es negativa y amortigua la corrección. Se acota respecto al
    término proporcional para evitar inversiones causadas por ruido.
    """
    KP = .8
    KD = .12
    DEADBAND_RAD = math.radians(1.5)
    MAX_RATE_RAD_S = math.radians(8.)
    MAX_SLEW_RAD_S2 = math.radians(25.)
    BEARING_FILTER_SECONDS = .12
    DERIVATIVE_FILTER_SECONDS = .15
    MAX_SAMPLE_GAP_SECONDS = .5
    HALF_TRACK_MM = 95.
    MAX_WHEEL_CORRECTION_FRACTION = .25
    MAX_DERIVATIVE_FRACTION = .6

    def __init__(self):
        self.reset()

    def reset(self):
        self._stamp = None
        self._sequence = None
        self._bearing = 0.
        self._derivative = 0.
        self._rate = 0.

    def _cap(self, speed_mm_s, distance_m):
        # Menor curvatura al aproximarse; nunca invierte la rueda interior.
        approach = min(1., max(.4, distance_m / .1))
        geometric = speed_mm_s * self.MAX_WHEEL_CORRECTION_FRACTION / self.HALF_TRACK_MM
        return min(self.MAX_RATE_RAD_S * approach, geometric)

    def update(self, bearing_rad, stamp_s, sequence, speed_mm_s, distance_m):
        """Devuelve velocidad angular a partir de un fotograma identificado.

        ``stamp_s`` es el instante de captura, no el momento de consultar una
        imagen antigua. Se requieren secuencias enteras no negativas. Una caída
        repentina del límite por velocidad/distancia puede recortar la salida
        inmediatamente: conservar avance positivo tiene prioridad sobre la rampa.
        """
        bearing, stamp, speed, distance = (_finite(value) for value in
                                         (bearing_rad, stamp_s, speed_mm_s, distance_m))
        if (any(value is None for value in (bearing, stamp, speed, distance))
                or isinstance(sequence, bool) or not isinstance(sequence, Integral)
                or sequence < 0 or stamp < 0 or speed <= 0 or distance < 0):
            self.reset()
            return 0.
        sequence = int(sequence)
        cap = self._cap(speed, distance)
        if sequence == self._sequence:
            self._rate = _limit(self._rate, cap)
            return self._rate

        bearing = math.remainder(bearing, 2. * math.pi)
        if self._stamp is None:
            self._stamp, self._sequence, self._bearing = stamp, sequence, bearing
            return 0.
        seconds = stamp - self._stamp
        if (sequence < self._sequence or seconds <= 0
                or seconds > self.MAX_SAMPLE_GAP_SECONDS):
            self.reset()
            return 0.

        alpha = -math.expm1(-seconds / self.BEARING_FILTER_SECONDS)
        delta = math.remainder(bearing - self._bearing, 2. * math.pi)
        increment = alpha * delta
        self._bearing = math.remainder(self._bearing + increment, 2. * math.pi)
        derivative_alpha = -math.expm1(-seconds / self.DERIVATIVE_FILTER_SECONDS)
        self._derivative += derivative_alpha * (increment / seconds - self._derivative)
        self._stamp, self._sequence = stamp, sequence

        error = max(0., abs(self._bearing) - self.DEADBAND_RAD)
        proportional = self.KP * math.copysign(error, self._bearing)
        if error == 0:
            desired = 0.
        else:
            derivative = _limit(self.KD * self._derivative,
                                abs(proportional) * self.MAX_DERIVATIVE_FRACTION)
            desired = _limit(proportional + derivative, cap)
        self._rate += _limit(desired - self._rate, self.MAX_SLEW_RAD_S2 * seconds)
        self._rate = _limit(self._rate, cap)
        return self._rate
