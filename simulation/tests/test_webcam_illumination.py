"""Fotometría sin cámaras: geometría, exposición, controles e identidad AprilTag."""
from dataclasses import FrozenInstanceError

import cv2
import numpy as np
import pytest

from simulation.webcam.detector import AprilTagDetector
from simulation.webcam.illumination import IlluminationSettings, enhance_frame


def gray_frame(gray):
    return cv2.cvtColor(np.asarray(gray, dtype=np.uint8), cv2.COLOR_GRAY2BGR)


def tag_frame(identifier=0):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    gray = np.full((240, 320), 145, np.uint8)
    gray[54:186, 94:226] = 255
    gray[66:174, 106:214] = cv2.aruco.generateImageMarker(dictionary, identifier, 108)
    # Sombra y exposición reducida, sin deformar o desenfocar el patrón.
    lighting = np.linspace(0.45, 0.8, gray.shape[1])[None, :]
    return gray_frame(np.clip(gray * lighting + 10, 0, 255))


def test_settings_are_immutable_and_default_does_not_enable_processing():
    settings = IlluminationSettings()
    assert not settings.enabled
    with pytest.raises(FrozenInstanceError):
        settings.enabled = True


@pytest.mark.parametrize("field,value", [
    ("enabled", 1), ("enabled", "false"), ("automatic", None),
    ("local_contrast", np.bool_(True)), ("brightness", True),
    ("contrast", False), ("gamma", "1"), ("brightness", -61),
    ("brightness", 61), ("contrast", .49), ("contrast", 2.51),
    ("gamma", .49), ("gamma", 2.01), ("gamma", float("nan")),
    ("contrast", float("inf")), ("brightness", -float("inf")),
])
def test_settings_reject_invalid_values(field, value):
    with pytest.raises(ValueError):
        IlluminationSettings(**{field: value})


@pytest.mark.parametrize("frame", [np.zeros((2, 3), np.uint8),
                                  np.zeros((2, 3, 4), np.uint8),
                                  np.zeros((2, 3, 3), float),
                                  np.zeros((0, 3, 3), np.uint8), None])
def test_invalid_frames_are_explicitly_rejected(frame):
    with pytest.raises(ValueError):
        enhance_frame(frame, IlluminationSettings())


def test_disabled_is_the_original_object_and_reports_original_exposure():
    frame = gray_frame([[0, 10, 100], [240, 250, 255]])
    original = frame.copy()
    output, diagnostics = enhance_frame(frame, IlluminationSettings())
    assert output is frame
    assert np.array_equal(frame, original)
    assert diagnostics["underexposed_pct"] == pytest.approx(100 / 3)
    assert diagnostics["overexposed_pct"] == pytest.approx(100 / 3)
    assert diagnostics["gamma_used"] == 1
    assert all(np.isfinite(value) for value in diagnostics.values())


def test_enabled_manual_neutral_retains_exact_pixel_values_without_aliasing():
    frame = np.random.default_rng(3).integers(0, 256, (73, 129, 3), dtype=np.uint8)
    original = frame.copy()
    output, _ = enhance_frame(frame, IlluminationSettings(
        enabled=True, automatic=False, local_contrast=False))
    assert np.array_equal(output, original)
    assert not np.shares_memory(frame, output)
    assert np.array_equal(frame, original)


def test_manual_adjustment_preserves_pixel_coordinates_and_accepts_readonly_input():
    frame = np.zeros((47, 83, 3), np.uint8)
    frame[13, 61] = 120
    frame.flags.writeable = False
    output, _ = enhance_frame(frame, IlluminationSettings(
        enabled=True, automatic=False, local_contrast=False, brightness=20))
    assert output.shape == frame.shape and output.dtype == np.uint8
    assert np.all(output[13, 61] == 140)
    assert np.unravel_index(np.argmax(output[:, :, 0]), frame.shape[:2]) == (13, 61)
    assert np.all(output[0, 0] == 20)
    assert np.all(frame[0, 0] == 0)
    assert not np.shares_memory(output, frame)


def test_global_adjustment_does_not_mirror_rotate_or_crop_geometry():
    frame = tag_frame()
    settings = IlluminationSettings(enabled=True, automatic=False,
                                    local_contrast=False, gamma=1.4, brightness=5)
    normal, _ = enhance_frame(frame, settings)
    rotated, _ = enhance_frame(np.rot90(frame).copy(), settings)
    assert np.array_equal(rotated, np.rot90(normal))


def test_gamma_larger_than_one_brightens_and_smaller_darkens():
    frame = np.full((41, 53, 3), 80, np.uint8)
    light, light_stats = enhance_frame(frame, IlluminationSettings(
        enabled=True, automatic=False, local_contrast=False, gamma=1.5))
    dark, dark_stats = enhance_frame(frame, IlluminationSettings(
        enabled=True, automatic=False, local_contrast=False, gamma=.7))
    assert light.mean() > frame.mean() > dark.mean()
    assert light_stats["gamma_used"] == 1.5
    assert dark_stats["gamma_used"] == .7
    assert light_stats["mean"] == dark_stats["mean"] == 80


def test_automatic_lift_is_bounded_and_not_driven_by_small_white_reflections():
    frame = np.full((120, 160, 3), 35, np.uint8)
    reflection = frame.copy()
    reflection[:10] = 255
    output, stats = enhance_frame(frame, IlluminationSettings(enabled=True))
    _, reflected_stats = enhance_frame(reflection, IlluminationSettings(enabled=True))
    assert 35 < output.mean() < 110
    assert 1 < stats["gamma_used"] <= 1.55
    assert reflected_stats["gamma_used"] == pytest.approx(stats["gamma_used"], abs=.05)
    assert reflected_stats["overexposed_pct"] > 0
    assert stats["mean"] == 35


def test_manual_contrast_expands_available_washed_detail():
    gray = np.tile(np.arange(145, 186, dtype=np.uint8), (64, 1))
    frame = gray_frame(gray)
    output, _ = enhance_frame(frame, IlluminationSettings(
        enabled=True, automatic=False, local_contrast=True, contrast=1.5))
    assert np.std(cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)) > 1.2 * np.std(gray)


def test_moderate_luminance_change_preserves_chroma_when_not_clipped():
    frame = np.array([[[75, 95, 115], [105, 85, 75], [65, 100, 80]]], np.uint8)
    output, _ = enhance_frame(frame, IlluminationSettings(
        enabled=True, automatic=False, local_contrast=False, brightness=10))
    before = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
    after = cv2.cvtColor(output, cv2.COLOR_BGR2YCrCb)
    assert np.max(np.abs(before[:, :, 1:].astype(int) - after[:, :, 1:].astype(int))) <= 1


def test_saturated_white_does_not_gain_invented_detail_or_hide_capture_clipping():
    frame = np.full((120, 160, 3), 80, np.uint8)
    frame[25:95, 40:120] = 255
    original = frame.copy()
    output, stats = enhance_frame(frame, IlluminationSettings(
        enabled=True, brightness=-20, contrast=1.1, gamma=1.3))
    assert np.array_equal(frame, original)
    assert np.std(output[25:95, 40:120]) == 0
    assert stats["overexposed_pct"] > 0
    assert stats["output_mean"] != stats["mean"]


@pytest.mark.parametrize("identifier", [0, 1, 12])
@pytest.mark.parametrize("rotation", [0, 1, 2, 3])
def test_processed_tag_keeps_identity_corners_and_heading(identifier, rotation):
    frame = np.rot90(tag_frame(identifier), rotation).copy()
    original = frame.copy()
    output, stats = enhance_frame(frame, IlluminationSettings(enabled=True))
    detection = AprilTagDetector().process(output)
    assert np.array_equal(frame, original)
    assert output.shape == frame.shape
    assert all(np.isfinite(value) for value in stats.values())
    assert detection.visible is (identifier == 0)
    if identifier == 0:
        assert detection.center_px == pytest.approx(
            (np.asarray(frame.shape[1::-1]) - 1) / 2, abs=1)
        error = (detection.heading_deg - rotation * 90 + 180) % 360 - 180
        assert error == pytest.approx(0, abs=2)


def test_local_contrast_does_not_turn_plain_square_into_apriltag():
    gray = np.full((240, 320), 170, np.uint8)
    gray[60:180, 100:220] = 30
    output, _ = enhance_frame(gray_frame(gray), IlluminationSettings(enabled=True))
    detection = AprilTagDetector().process(output)
    assert not detection.visible
    assert detection.marker_corners is None


def test_underexposed_tag_becomes_readable_without_reconstructing_or_moving_its_pattern():
    """Poca señal, sombra gradual y ruido de lectura; los bits aún existen."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, 0, 80)
    reference = np.full((240, 320), 145, np.uint8)
    reference[70:170, 110:210] = 255
    reference[80:160, 120:200] = marker

    luminance = np.full((240, 320), 24 * .8, np.float32)
    luminance[70:170, 110:210] = 24
    luminance[80:160, 120:200] = 12 + 12 * marker.astype(np.float32) / 255
    luminance *= np.linspace(.7, 1., 320, dtype=np.float32)[None, :]
    luminance += np.random.default_rng(17).normal(0, .5, luminance.shape).astype(np.float32)
    # No hay luces recortadas: la mejora utiliza diferencias ya registradas.
    assert 0 < luminance.min() < luminance.max() < 255
    original = gray_frame(np.rint(luminance).astype(np.uint8))
    snapshot = original.copy()
    detector = AprilTagDetector()
    assert not detector.process(original).visible
    expected = detector.process(gray_frame(reference))
    assert expected.visible

    adjusted, diagnostics = enhance_frame(original, IlluminationSettings(enabled=True))
    detected = detector.process(adjusted)
    assert detected.visible
    assert np.array_equal(original, snapshot)
    assert adjusted.shape == original.shape
    assert diagnostics["overexposed_pct"] == 0
    assert np.asarray(detected.marker_corners) == pytest.approx(
        np.asarray(expected.marker_corners), abs=1.5)
    assert detected.center_px == pytest.approx(expected.center_px, abs=1.5)
    heading_error = (detected.heading_deg - expected.heading_deg + 180) % 360 - 180
    assert heading_error == pytest.approx(0, abs=2)
