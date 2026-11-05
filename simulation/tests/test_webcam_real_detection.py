"""Variaciones de impresión, soporte e iluminación, sin fotos de los usuarios."""
from __future__ import annotations

import cv2
import numpy as np
import pytest

from simulation.webcam.detector import AprilTagDetector


def mounted_marker(identifier=0, rotation=0, blur=0.0, contrast=1.0):
    """Tarjeta con margen fino y una zona oscura del soporte junto al borde.

    La zona exterior altera el contorno, como una sombra o cinta oscura junto
    a una impresión recortada. Los módulos de datos del marcador no se tocan.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    gray = np.full((480, 640), 160, np.uint8)
    gray[129:331, 219:421] = 245
    gray[130:330, 220:420] = cv2.aruco.generateImageMarker(dictionary, identifier, 200)
    support = np.array([[250, 130], [300, 105], [350, 130]], np.int32)
    cv2.fillConvexPoly(gray, support, 0)
    gray = np.rot90(gray, rotation).copy()
    if contrast != 1.0:
        gray = np.clip(70 + contrast * gray.astype(np.float32), 0, 255).astype(np.uint8)
    if blur:
        gray = cv2.GaussianBlur(gray, (0, 0), blur)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


@pytest.mark.parametrize("rotation", range(4))
@pytest.mark.parametrize("blur,contrast", [(0.0, 1.0), (0.6, 0.7), (1.2, 0.5)])
def test_webcam_detects_realistic_contour_interference_without_changing_identity(rotation, blur, contrast):
    frame = mounted_marker(rotation=rotation, blur=blur, contrast=contrast)
    result = AprilTagDetector().process(frame)
    assert result.visible
    assert result.center_px is not None and result.marker_corners is not None
    expected = 90.0 * rotation
    error = (result.heading_deg - expected + 180.0) % 360.0 - 180.0
    assert error == pytest.approx(0.0, abs=3.0)
    assert np.asarray(result.center_px) == pytest.approx(np.asarray(frame.shape[1::-1]) / 2, abs=15.0)


@pytest.mark.parametrize("rotation", range(4))
def test_webcam_printed_marker_orientation_survives_oblique_camera_perspective(rotation):
    frame = mounted_marker(rotation=rotation, blur=0.6)
    height, width = frame.shape[:2]
    source = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
    target = np.float32([[.04 * width, .02 * height], [.97 * width, .10 * height],
                         [.99 * width, .91 * height], [.01 * width, .99 * height]])
    transform = cv2.getPerspectiveTransform(source, target)
    oblique = cv2.warpPerspective(frame, transform, (width, height), borderValue=(160, 160, 160))
    result = AprilTagDetector().process(oblique)
    assert result.visible
    angle = np.deg2rad(rotation * 90.0)
    center = np.array([width / 2, height / 2], dtype=np.float32)
    forward = center + np.float32([np.cos(angle), -np.sin(angle)]) * 40
    projected = cv2.perspectiveTransform(np.float32([[center, forward]]), transform)[0]
    direction = projected[1] - projected[0]
    expected = np.degrees(np.arctan2(-direction[1], direction[0]))
    error = (result.heading_deg - expected + 180.0) % 360.0 - 180.0
    assert error == pytest.approx(0.0, abs=4.0)


@pytest.mark.parametrize("identifier", [1, 12, 100, 586])
def test_webcam_contour_fallback_does_not_accept_another_apriltag(identifier):
    result = AprilTagDetector().process(mounted_marker(identifier=identifier))
    assert not result.visible
    assert result.center_px is None and result.heading_deg is None


def test_webcam_contour_fallback_does_not_make_a_dark_square_into_a_marker():
    frame = mounted_marker()
    frame[130:330, 220:420] = 0
    result = AprilTagDetector().process(frame)
    assert not result.visible
    assert result.marker_corners is None


def test_webcam_contour_fallback_still_retires_pose_after_occlusion():
    detector = AprilTagDetector()
    frame = mounted_marker()
    assert detector.process(frame).visible
    frame[130:330, 220:345] = 160
    result = detector.process(frame)
    assert not result.visible
    assert result.center_px is None and result.heading_deg is None


@pytest.mark.parametrize("side_pixels", [48, 80, 160])
@pytest.mark.parametrize("rotation", range(4))
@pytest.mark.parametrize("illumination", ["uniform", "gradient"])
def test_webcam_correct_white_margin_preserves_identity_at_different_apparent_sizes(
        side_pixels, rotation, illumination):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    frame = np.full((480, 640), 160, np.uint8)
    x, y = (frame.shape[1] - side_pixels) // 2, (frame.shape[0] - side_pixels) // 2
    # 5 mm blancos por lado para un patrón negro de 40 mm: margen = lado / 8.
    margin = side_pixels // 8
    frame[y - margin:y + side_pixels + margin,
          x - margin:x + side_pixels + margin] = 255
    frame[y:y + side_pixels, x:x + side_pixels] = cv2.aruco.generateImageMarker(
        dictionary, 0, side_pixels)
    if illumination == "gradient":
        lighting = np.linspace(.85, 1.05, frame.shape[1], dtype=np.float32)[None, :]
        frame = np.clip((30.0 + .72 * frame.astype(np.float32)) * lighting, 0, 255).astype(np.uint8)
        frame = cv2.GaussianBlur(frame, (0, 0), .7)
    frame = np.rot90(frame, rotation).copy()
    result = AprilTagDetector().process(frame)
    assert result.visible
    assert result.markers == 1
    assert np.asarray(result.center_px) == pytest.approx(
        (np.asarray(frame.shape[1::-1]) - 1) / 2, abs=1.5)
    error = (result.heading_deg - rotation * 90.0 + 180.0) % 360.0 - 180.0
    assert error == pytest.approx(0.0, abs=3.0)

    # Confirma que las esquinas devueltas pertenecen al identificador 0 y
    # conservan su orden canónico; no basta con aceptar cualquier cuadrado.
    transform = cv2.getPerspectiveTransform(
        np.float32(result.marker_corners), np.float32([[0, 0], [79, 0], [79, 79], [0, 79]]))
    canonical = cv2.warpPerspective(frame, transform, (80, 80))
    _, binary = cv2.threshold(canonical, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    bits = np.array([[np.mean(binary[row * 10 + 3:row * 10 + 7,
                                     column * 10 + 3:column * 10 + 7]) > 127
                      for column in range(1, 7)] for row in range(1, 7)], dtype=np.uint8)
    assert dictionary.getDistanceToId(bits, 0, False) == 0
