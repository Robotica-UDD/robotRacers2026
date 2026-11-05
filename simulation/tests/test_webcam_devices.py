"""Descubrimiento sin cámaras físicas, sesiones ni permisos."""
import importlib

import pytest

from simulation.webcam import devices


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    monkeypatch.setattr(devices, "_cache_at", None)
    monkeypatch.setattr(devices, "_cache_value", None)
    monkeypatch.setattr(devices, "PLATFORM", "darwin")
    monkeypatch.setattr(devices, "_macos_devices", lambda: [
        {"index": 0, "label": "Integrada", "id": "builtin", "transport": "bltn"},
        {"index": 1, "label": "Webcam (USB)", "id": "usb-camera", "transport": "usb"},
    ])


def test_discovery_preserves_native_indices_and_prefers_usb():
    result = devices.discover_cameras()
    assert result["enumerated"] is True
    assert [(item["index"], item["id"]) for item in result["options"]] == [
        (0, "builtin"), (1, "usb-camera")]
    assert result["default_index"] == 1


def test_discovery_cache_is_short_and_force_refreshes(monkeypatch):
    calls = []
    native = devices._macos_devices
    monkeypatch.setattr(devices, "_macos_devices", lambda: calls.append(True) or native())
    now = [100.0]
    monkeypatch.setattr(devices.time, "monotonic", lambda: now[0])
    devices.discover_cameras()
    now[0] = 101.0
    devices.discover_cameras()
    assert len(calls) == 1
    now[0] = 104.0
    devices.discover_cameras()
    assert len(calls) == 2
    devices.discover_cameras(force=True)
    assert len(calls) == 3


def test_callers_cannot_change_shared_cached_options():
    first = devices.discover_cameras()
    first["options"][0]["label"] = "alterado"
    assert devices.discover_cameras()["options"][0]["label"] == "Integrada"


def test_missing_native_backend_keeps_honest_manual_fallback(monkeypatch):
    def unavailable():
        raise OSError("No disponible")
    monkeypatch.setattr(devices, "_macos_devices", unavailable)
    result = devices.discover_cameras()
    assert result["enumerated"] is False
    assert result["message"]
    assert [item["index"] for item in result["options"]] == list(range(10))
    assert all(item["label"].startswith("Cámara ") for item in result["options"])


def test_non_macos_does_not_touch_native_runtime(monkeypatch):
    monkeypatch.setattr(devices, "PLATFORM", "linux")
    def prohibited():
        pytest.fail("No debe consultar AVFoundation fuera de macOS")
    monkeypatch.setattr(devices, "_macos_devices", prohibited)
    assert devices.discover_cameras()["enumerated"] is False


def test_no_devices_returns_empty_enumerated_list(monkeypatch):
    monkeypatch.setattr(devices, "_macos_devices", lambda: [])
    result = devices.discover_cameras()
    assert result["enumerated"] is True
    assert result["options"] == []
    assert "No se detectaron" in result["message"]


def test_import_does_not_load_framework_or_enumerate(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Importar no debe cargar frameworks ni tocar cámaras")
    with monkeypatch.context() as guard:
        guard.setattr(devices.ctypes, "CDLL", forbidden)
        importlib.reload(devices)
