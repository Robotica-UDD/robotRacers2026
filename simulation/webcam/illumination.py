"""Corrección fotométrica de webcam, sin desplazar píxeles ni acceder a cámaras.

La compensación digital mejora contraste/exposición disponibles; no reconstruye
detalle perdido por saturación. El diagnóstico siempre describe la captura
original para que aclarar la imagen no oculte una subexposición de la cámara.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

import cv2
import numpy as np


@dataclass(frozen=True)
class IlluminationSettings:
    enabled: bool = False
    automatic: bool = True
    brightness: float = 0.0
    contrast: float = 1.0
    gamma: float = 1.0
    local_contrast: bool = True

    def __post_init__(self) -> None:
        for name in ("enabled", "automatic", "local_contrast"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} debe ser booleano")
        for name, minimum, maximum in (("brightness", -60.0, 60.0),
                                       ("contrast", 0.5, 2.5),
                                       ("gamma", 0.5, 2.0)):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, Real)
                    or not minimum <= value <= maximum
                    or not math.isfinite(float(value))):
                raise ValueError(f"{name} debe estar entre {minimum} y {maximum}")


def _sample(channel: np.ndarray) -> np.ndarray:
    """Muestra acotada y determinista: no se redimensiona la imagen de salida."""
    height, width = channel.shape[:2]
    return channel[::max(1, math.ceil(height / 120)),
                   ::max(1, math.ceil(width / 160))]


def _statistics(luminance_sample: np.ndarray) -> dict[str, float]:
    pixels = luminance_sample.reshape(-1)
    p10, p25, median, p75, p90 = np.percentile(pixels, [10, 25, 50, 75, 90])
    return {
        "mean": float(np.mean(pixels)),
        "median": float(median),
        "p10": float(p10),
        "p25": float(p25),
        "p75": float(p75),
        "p90": float(p90),
        "overexposed_pct": float(np.count_nonzero(pixels >= 250) * 100.0 / pixels.size),
        "underexposed_pct": float(np.count_nonzero(pixels <= 12) * 100.0 / pixels.size),
    }


def _automatic_gamma(stats: dict[str, float]) -> float:
    # Cuartiles y mediana reducen la influencia de reflejos pequeños. No se
    # asume que el suelo sea gris ni se ajusta balance de blancos. El límite
    # inferior del nivel evita amplificar arbitrariamente el ruido negro.
    level = (0.6 * stats["median"] + 0.2 * stats["p25"] + 0.2 * stats["p75"]) / 255.0
    level = min(247.0 / 255.0, max(8.0 / 255.0, level))
    ideal = math.log(level) / math.log(0.46)
    return float(np.clip(ideal ** 0.6, 0.82, 1.55))


def enhance_frame(frame: np.ndarray, settings: IlluminationSettings
                  ) -> tuple[np.ndarray, dict[str, float]]:
    """Devuelve BGR uint8 del mismo tamaño y diagnósticos de la captura.

    ``gamma > 1`` aclara: la curva usa ``(Y / 255) ** (1 / gamma)``.
    Brillo es un offset de luminancia; contraste gira alrededor de 127,5.
    No hay recorte, espejo, desenfoque, deformación, interpolación geométrica
    ni creación de detalles. La entrada nunca se modifica. Deshabilitado
    devuelve exactamente el mismo objeto para mantener intacta la captura.
    """
    if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8
            or frame.ndim != 3 or frame.shape[2] != 3
            or frame.shape[0] == 0 or frame.shape[1] == 0):
        raise ValueError("Se requiere una imagen BGR uint8 de tres canales, no vacía")
    if not isinstance(settings, IlluminationSettings):
        raise TypeError("settings debe ser IlluminationSettings")

    if not settings.enabled:
        # El muestreo se hace antes de convertir, sin pagar una conversión
        # completa de cada captura cuando la herramienta está apagada.
        stats = _statistics(cv2.cvtColor(_sample(frame), cv2.COLOR_BGR2YCrCb)[:, :, 0])
        return frame, {**stats, "gamma_used": 1.0, "automatic_gamma": 1.0,
                       "output_mean": stats["mean"]}

    channels = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
    luminance = channels[:, :, 0]
    stats = _statistics(_sample(luminance))
    automatic_gamma = _automatic_gamma(stats) if settings.automatic else 1.0
    gamma = float(settings.gamma) * automatic_gamma
    if (not settings.local_contrast and gamma == 1.0
            and settings.contrast == 1.0 and settings.brightness == 0.0):
        return frame.copy(), {**stats, "gamma_used": gamma,
                              "automatic_gamma": automatic_gamma,
                              "output_mean": stats["mean"]}

    if settings.local_contrast:
        # CLAHE limitado y mezclado conserva una apariencia moderada. Opera
        # solo sobre Y, nunca sobre coordenadas ni canales cromáticos.
        local = cv2.createCLAHE(clipLimit=1.6, tileGridSize=(8, 8)).apply(luminance)
        luminance = cv2.addWeighted(luminance, 0.65, local, 0.35, 0)

    values = np.arange(256, dtype=np.float32) / 255.0
    values = np.power(values, 1.0 / gamma) * 255.0
    values = (values - 127.5) * float(settings.contrast) + 127.5 + float(settings.brightness)
    lookup = np.clip(np.rint(values), 0, 255).astype(np.uint8)
    enhanced = cv2.LUT(luminance, lookup)
    channels[:, :, 0] = enhanced
    result = cv2.cvtColor(channels, cv2.COLOR_YCrCb2BGR)
    return result, {**stats, "gamma_used": gamma, "automatic_gamma": automatic_gamma,
                    "output_mean": float(np.mean(_sample(enhanced)))}
