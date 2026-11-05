"""El actualizador se prueba con puertos y procesos simulados, sin USB real."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from simulation.webcam import upload_robot as uploader


def board(path="/dev/cu.robot"):
    return SimpleNamespace(vid=0x303A, pid=0x1001, device=path)


@pytest.mark.parametrize("ports", [[], [board("a"), board("b")]])
def test_missing_or_ambiguous_robot_never_runs_a_command(ports):
    with pytest.raises(uploader.UploadError):
        uploader.update_robot(ports_provider=lambda: ports,
                              cli_finder=lambda: pytest.fail("No debe buscar el compilador"),
                              runner=lambda *a, **k: pytest.fail("No debe cargar firmware"))


def test_check_does_not_open_usb_compile_or_upload(capsys):
    uploader.update_robot(check_only=True, ports_provider=lambda: [board()],
                          cli_finder=lambda: Path("/fake/arduino-cli"),
                          stopped_checker=lambda *a: pytest.fail("No debe abrir USB"),
                          runner=lambda *a, **k: pytest.fail("No debe compilar"))
    assert "No se abrió el puerto" in capsys.readouterr().out


class SerialDouble:
    def __init__(self, status):
        self.status = status
        self.closed = False
    def reset_input_buffer(self): pass
    def write(self, data):
        assert data == b'{"command":"status"}\n'
    def readline(self, size):
        return (json.dumps(self.status) + "\n").encode()
    def close(self): self.closed = True


@pytest.mark.parametrize("motion", [{"moving": True}, {"navigation_active": True}])
def test_moving_robot_blocks_update_and_closes_usb(motion, capsys):
    serial = SerialDouble({"device": "RobotRacers", "version": 1, "moving": False,
                           "navigation_active": False, "token": "NO_PUBLICAR", **motion})
    with pytest.raises(uploader.UploadError, match="Detén el recorrido"):
        uploader.update_robot(ports_provider=lambda: [board()],
                              cli_finder=lambda: Path("/fake/arduino-cli"),
                              stopped_checker=lambda port: uploader.verify_stopped(
                                  port, serial_factory=lambda *a, **k: serial),
                              runner=lambda *a, **k: pytest.fail("No debe compilar ni cargar"))
    assert serial.closed
    assert "NO_PUBLICAR" not in capsys.readouterr().out


def test_compile_then_recheck_and_upload_without_erase_flags(tmp_path, monkeypatch):
    monkeypatch.setattr(uploader, "BUILD", tmp_path / "build")
    events = []
    uploader.update_robot(ports_provider=lambda: [board()],
                          cli_finder=lambda: Path("/fake/arduino-cli"),
                          stopped_checker=lambda port: events.append(("idle", port)),
                          runner=lambda args, **kwargs: events.append((args[1], args)))
    assert [event[0] for event in events] == ["idle", "compile", "idle", "upload"]
    assert uploader.FQBN in events[1][1]
    assert "--input-dir" in events[3][1]
    assert not any("erase" in str(argument).lower() for _, args in events if isinstance(args, list) for argument in args)


def test_port_change_after_compile_blocks_upload(tmp_path, monkeypatch):
    monkeypatch.setattr(uploader, "BUILD", tmp_path / "build")
    calls = iter([[board("first")], [board("second")]])
    commands = []
    with pytest.raises(uploader.UploadError, match="puerto USB cambió"):
        uploader.update_robot(ports_provider=lambda: next(calls),
                              cli_finder=lambda: Path("/fake/arduino-cli"),
                              stopped_checker=lambda port: None,
                              runner=lambda args, **kwargs: commands.append(args[1]))
    assert commands == ["compile"]
