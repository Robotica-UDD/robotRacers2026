"""Descubrimiento sin cámaras físicas, sesiones ni permisos."""
import importlib
import json
import subprocess
import sys
from types import SimpleNamespace

import pytest

from simulation.webcam import devices

fresh_query = devices._fresh_macos_devices


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    monkeypatch.setattr(devices, "_cache_at", None)
    monkeypatch.setattr(devices, "_cache_value", None)
    monkeypatch.setattr(devices, "PLATFORM", "darwin")
    monkeypatch.setattr(devices, "_fresh_macos_devices", lambda: [
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
    native = devices._fresh_macos_devices
    monkeypatch.setattr(devices, "_fresh_macos_devices", lambda: calls.append(True) or native())
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
    monkeypatch.setattr(devices, "_fresh_macos_devices", unavailable)
    result = devices.discover_cameras()
    assert result["enumerated"] is False
    assert result["message"]
    assert [item["index"] for item in result["options"]] == list(range(10))
    assert all(item["label"].startswith("Cámara ") for item in result["options"])


def test_non_macos_does_not_touch_native_runtime(monkeypatch):
    monkeypatch.setattr(devices, "PLATFORM", "linux")
    def prohibited():
        pytest.fail("No debe consultar AVFoundation fuera de macOS")
    monkeypatch.setattr(devices, "_fresh_macos_devices", prohibited)
    assert devices.discover_cameras()["enumerated"] is False


def test_no_devices_returns_empty_enumerated_list(monkeypatch):
    monkeypatch.setattr(devices, "_fresh_macos_devices", lambda: [])
    result = devices.discover_cameras()
    assert result["enumerated"] is True
    assert result["options"] == []
    assert "No se detectaron" in result["message"]


def test_import_does_not_load_framework_or_enumerate(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Importar no debe cargar frameworks ni tocar cámaras")
    with monkeypatch.context() as guard:
        guard.setattr(devices.ctypes, "CDLL", forbidden)
        guard.setattr(devices.subprocess, "run", forbidden)
        importlib.reload(devices)


def test_fresh_discovery_uses_a_bounded_python_process_without_a_shell(monkeypatch):
    expected = [{"index": 0, "label": "Webcam USB", "id": "usb-new-port", "transport": "usb"}]
    calls = []
    def run(arguments, **options):
        calls.append((arguments, options))
        return SimpleNamespace(stdout=json.dumps(expected))
    monkeypatch.setattr(devices.subprocess, "run", run)
    assert fresh_query() == expected
    assert len(calls) == 1
    arguments, options = calls[0]
    assert arguments == [sys.executable, "-m", "simulation.webcam.devices", "--list-native"]
    assert options == {"capture_output": True, "text": True, "encoding": "utf-8",
                       "timeout": 8, "check": True, "shell": False}


def test_force_refresh_replaces_the_old_usb_identity_and_indices(monkeypatch):
    snapshots = iter([
        [{"index": 0, "label": "Integrada", "id": "builtin", "transport": "bltn"},
         {"index": 1, "label": "USB", "id": "old-port", "transport": "usb"}],
        [{"index": 0, "label": "USB", "id": "new-port", "transport": "usb"},
         {"index": 1, "label": "Integrada", "id": "builtin", "transport": "bltn"}],
    ])
    monkeypatch.setattr(devices, "_fresh_macos_devices", lambda: next(snapshots))
    first = devices.discover_cameras()
    assert first["default_index"] == 1
    assert devices.discover_cameras() == first
    refreshed = devices.discover_cameras(force=True)
    assert refreshed["default_index"] == 0
    assert refreshed["options"][0]["id"] == "new-port"
    assert all(item["id"] != "old-port" for item in refreshed["options"])


@pytest.mark.parametrize("problem", [
    subprocess.TimeoutExpired(["python"], 8),
    subprocess.CalledProcessError(1, ["python"], stderr="No disponible"),
    OSError("No se pudo ejecutar el auxiliar"),
])
def test_auxiliary_failure_keeps_an_honest_manual_fallback(monkeypatch, problem):
    def run(*args, **kwargs):
        raise problem
    monkeypatch.setattr(devices.subprocess, "run", run)
    monkeypatch.setattr(devices, "_fresh_macos_devices", fresh_query)
    state = devices.discover_cameras(force=True)
    assert state["enumerated"] is False
    assert state["message"]
    assert [item["index"] for item in state["options"]] == list(range(10))


@pytest.mark.parametrize("payload", [
    "not JSON", "{}", "null", '["camera"]',
    '[{"index":true,"label":"USB","id":"x","transport":"usb"}]',
    '[{"index":1,"label":"USB","id":"x","transport":"usb"}]',
    '[{"index":0,"label":"USB","id":"","transport":"usb"}]',
    '[{"index":0,"label":123,"id":"x","transport":"usb"}]',
    '[{"index":0,"label":"USB","id":"x"}]',
    json.dumps([{"index": 0, "label": "USB", "id": "x" * 513, "transport": "usb"}]),
    json.dumps([{"index": 0, "label": "USB\u0000", "id": "x", "transport": "usb"}]),
    " " * (devices.NATIVE_OUTPUT_LIMIT + 1),
])
def test_native_response_rejects_malformed_or_unbounded_payloads(payload):
    with pytest.raises((ValueError, TypeError)):
        devices._parse_native_devices(payload)


def test_native_response_limit_counts_utf8_bytes():
    with pytest.raises(ValueError):
        devices._parse_native_devices("é" * (devices.NATIVE_OUTPUT_LIMIT // 2 + 1))


def test_native_response_rejects_too_many_devices():
    entries = [{"index": i, "label": "USB", "id": str(i), "transport": "usb"}
               for i in range(65)]
    with pytest.raises(ValueError):
        devices._parse_native_devices(json.dumps(entries))


def test_native_command_prints_direct_enumeration_without_recursing(monkeypatch, capsys):
    expected = [{"index": 0, "label": "Cámara USB", "id": "current-port", "transport": "usb"}]
    monkeypatch.setattr(devices.sys, "argv", ["devices", "--list-native"])
    monkeypatch.setattr(devices, "_macos_devices", lambda: expected)
    def forbidden(*args, **kwargs):
        pytest.fail("El auxiliar debe enumerar directamente, sin crear otro proceso.")
    monkeypatch.setattr(devices, "discover_cameras", forbidden)
    monkeypatch.setattr(devices.subprocess, "run", forbidden)
    devices.main()
    assert json.loads(capsys.readouterr().out) == expected
