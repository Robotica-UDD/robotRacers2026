"""Contrato de navegación del robot, con HTTP/UDP simulados y sin dispositivos.

Las consignas firmadas usan PWM de ocho bits, limitado a ±100 en esta prueba.
Armar una sesión no mueve ruedas; cada datagrama lleva un plazo local de 300 ms.
"""
from copy import deepcopy
from dataclasses import dataclass
import json
import threading
from urllib.parse import urlsplit
import uuid

import pytest

from simulation.webcam.robot_link import RobotLink, RobotLinkError


IP = "10.0.0.2"
MAC = "AA:BB:CC:DD:EE:01"
TOKEN = "0123456789abcdef0123456789abcdef"


def firmware_status(**changes):
    return {"device": "RobotRacers", "version": 1, "mac": MAC,
            "boot_id": "aabbccddeeff0011", "ip": IP, "connected": True,
            "ready": True, "moving": False, "last_id": "", "stop_reason": "boot",
            "navigation_active": False, "control_seq": 0, "control_mode": "idle",
            "control_session": "", **changes}


class FakeFirmwareHttp:
    """Identidad y ACK del firmware; ninguna solicitud sale del proceso."""

    def __init__(self):
        self.status = firmware_status()
        self.calls = []
        self.bad_navigation_ack = False
        self.fail_navigation_start = False

    def request(self, method, url, *, payload=None, headers=None, timeout=None):
        path = urlsplit(url).path
        self.calls.append({"method": method, "url": url, "path": path,
                           "payload": deepcopy(payload), "headers": dict(headers or {}),
                           "timeout": timeout})
        assert urlsplit(url).hostname == IP
        if method == "GET" and path == "/status":
            return dict(self.status)
        assert method == "POST"
        assert headers == {"X-Robot-Token": TOKEN}
        if path == "/navigation/start":
            assert set(payload) == {"session"}
            session = payload["session"]
            assert str(uuid.UUID(session)) == session
            if self.status["navigation_active"] or self.status["moving"]:
                raise RobotLinkError("Robot simulado ocupado.", 409)
            # La orden puede llegar aunque el ACK se pierda: nunca reintentarlo.
            self.status.update(navigation_active=True, moving=False,
                               control_session=session, control_mode="navigation",
                               control_seq=0)
            if self.fail_navigation_start:
                raise RobotLinkError("No se recibió el ACK simulado.")
            return {"accepted": True,
                    "session": str(uuid.uuid4()) if self.bad_navigation_ack else session}
        if path == "/stop":
            self.status.update(navigation_active=False, moving=False,
                               control_session="", control_mode="idle", stop_reason="remote")
            return {"stopped": True}
        if path == "/pulse":
            if self.status["navigation_active"]:
                raise RobotLinkError("Robot simulado ocupado.", 409)
            self.status.update(moving=True, last_id=payload["id"], control_mode="pulse")
            return {"accepted": True, "id": payload["id"], "moving": True}
        raise AssertionError(f"Solicitud no contemplada: {method} {path}")

    def actions(self, path=None):
        return [call for call in self.calls if call["method"] == "POST"
                and (path is None or call["path"] == path)]


class FakeDatagramSocket:
    def __init__(self, owner):
        self.owner = owner
        self.closed = False
        self.blocking = None

    def setblocking(self, value):
        self.blocking = value

    def close(self):
        self.closed = True

    def sendto(self, data, destination):
        assert not self.closed
        assert isinstance(data, bytes)
        assert len(data) <= 512
        packet = json.loads(data)
        self.owner.packets.append({"packet": packet, "destination": destination})
        if self.owner.fail_send:
            raise OSError("UDP simulado no disponible")
        state = self.owner.http.status
        if packet["session"] == state["control_session"] and state["navigation_active"]:
            state["control_seq"] = packet["seq"]
            if packet["command"] == "stop":
                state.update(navigation_active=False, moving=False, control_session="",
                             control_mode="idle", stop_reason="remote")
            else:
                state["moving"] = packet["left"] != 0 or packet["right"] != 0
        return len(data)


class FakeDatagramFactory:
    def __init__(self, http):
        self.http = http
        self.packets = []
        self.sockets = []
        self.fail_send = False

    def __call__(self, *args, **kwargs):
        instance = FakeDatagramSocket(self)
        self.sockets.append(instance)
        return instance

    def commands(self, name):
        return [entry["packet"] for entry in self.packets
                if entry["packet"]["command"] == name]


@dataclass
class Harness:
    link: RobotLink
    http: FakeFirmwareHttp
    udp: FakeDatagramFactory


@pytest.fixture
def harness(tmp_path):
    runtime = tmp_path / "robot-link.json"
    runtime.write_text(json.dumps({"ip": IP, "mac": MAC, "token": TOKEN}))
    runtime.chmod(0o600)
    http = FakeFirmwareHttp()
    udp = FakeDatagramFactory(http)

    def forbidden_serial(*args, **kwargs):
        raise AssertionError("Esta prueba no debe abrir USB.")

    link = RobotLink(runtime, serial_factory=forbidden_serial, http_client=http,
                     ports_provider=lambda: {"ports": []}, udp_socket_factory=udp,
                     refresh_ttl_s=1000, http_timeout_s=.1)
    yield Harness(link, http, udp)
    link.close()


def assert_conflict(action):
    with pytest.raises(RobotLinkError) as error:
        action()
    assert error.value.status_code == 409


def test_stop_still_reaches_robot_if_panel_callback_raises(harness):
    harness.link.start_navigation()
    def failed_callback(reason):
        raise RuntimeError("Fallo simulado del panel")
    harness.link.set_navigation_cancel_callback(failed_callback)
    harness.link.stop()
    assert len(harness.udp.commands("stop")) == 1
    assert len(harness.http.actions("/stop")) == 1
    assert_conflict(lambda: harness.link.drive_navigation(100, 100))


def test_abandon_stops_udp_without_waiting_for_http_and_allows_fresh_session(harness):
    old_session = harness.link.start_navigation()["session"]
    harness.link.drive_navigation(80, 80)
    before = len(harness.http.calls)
    harness.link.abandon_navigation()
    assert len(harness.http.calls) == before
    assert len(harness.udp.commands("stop")) == 1
    assert_conflict(lambda: harness.link.drive_navigation(100, 100))
    assert harness.link.start_navigation()["session"] != old_session


def test_constructing_saved_link_and_closing_never_sends_commands(harness):
    assert harness.http.calls == []
    assert harness.udp.packets == []
    assert harness.udp.sockets == []
    harness.link.close()
    assert harness.http.calls == []
    assert harness.udp.packets == []


def test_arming_session_checks_identity_and_does_not_move_wheels(harness):
    result = harness.link.start_navigation()
    assert set(result) == {"session"}
    assert str(uuid.UUID(result["session"])) == result["session"]
    assert [call["method"] for call in harness.http.calls] == ["GET", "POST"]
    start = harness.http.actions("/navigation/start")
    assert len(start) == 1
    assert start[0]["payload"] == result
    assert start[0]["headers"] == {"X-Robot-Token": TOKEN}
    assert harness.udp.packets == []
    assert harness.udp.sockets == []
    assert harness.http.status["navigation_active"]
    assert not harness.http.status["moving"]
    assert harness.link._snapshot()["navigation_active"]
    assert TOKEN not in json.dumps(harness.link._snapshot())


def test_signed_drive_packets_preserve_values_and_use_increasing_sequences(harness):
    session = harness.link.start_navigation()["session"]
    http_calls = len(harness.http.calls)
    speeds = [(-100, 100), (-73, 84), (0, 0), (100, -100)]
    for left, right in speeds:
        harness.link.drive_navigation(left, right)
    assert len(harness.udp.packets) == len(speeds)
    packets = harness.udp.commands("drive")
    for packet, (left, right) in zip(packets, speeds, strict=True):
        assert set(packet) == {"token", "session", "seq", "command", "left", "right", "lease_ms"}
        assert packet == {"token": TOKEN, "session": session, "seq": packet["seq"],
                          "command": "drive", "left": left, "right": right, "lease_ms": 300}
        assert type(packet["seq"]) is int and 0 <= packet["seq"] <= 0xffffffff
    assert all(((new["seq"] - old["seq"]) & 0xffffffff) == 1
               for old, new in zip(packets, packets[1:]))
    assert all(entry["destination"] == (IP, 4210) for entry in harness.udp.packets)
    # No HTTP ni confirmaciones bloqueantes dentro del circuito de consignas.
    assert len(harness.http.calls) == http_calls
    assert len(harness.udp.sockets) == 1
    assert harness.udp.sockets[0].blocking is False


@pytest.mark.parametrize("left,right", [(-101, 0), (101, 0), (0, -101), (0, 101),
                                       (True, 0), (0, False), (1.0, 0), (0, -1.0),
                                       ("100", 0), (0, None)])
def test_drive_rejects_non_integer_or_out_of_range_values_without_sending(harness, left, right):
    harness.link.start_navigation()
    with pytest.raises(RobotLinkError) as error:
        harness.link.drive_navigation(left, right)
    assert error.value.status_code == 422
    assert harness.udp.packets == []


def test_drive_requires_an_explicit_active_session(harness):
    assert_conflict(lambda: harness.link.drive_navigation(100, 100))
    assert harness.udp.packets == []
    assert harness.http.actions() == []


def test_stop_invalidates_session_and_next_start_uses_a_new_uuid(harness):
    first_session = harness.link.start_navigation()["session"]
    harness.link.drive_navigation(50, 50)
    drive = harness.udp.commands("drive")[-1]
    harness.link.stop_navigation()
    stops = harness.udp.commands("stop")
    assert len(stops) == 1
    assert stops[0] == {"token": TOKEN, "session": first_session,
                        "seq": (drive["seq"] + 1) & 0xffffffff, "command": "stop",
                        "left": 0, "right": 0, "lease_ms": 300}
    assert len(harness.http.actions("/stop")) == 1
    assert not harness.link._snapshot()["navigation_active"]
    assert not harness.http.status["navigation_active"]
    sent = len(harness.udp.packets)
    assert_conflict(lambda: harness.link.drive_navigation(50, 50))
    assert len(harness.udp.packets) == sent
    second_session = harness.link.start_navigation()["session"]
    assert second_session != first_session
    harness.link.drive_navigation(-50, 50)
    assert harness.udp.commands("drive")[-1]["session"] == second_session


def test_zero_drive_keeps_armed_session_busy_for_pulse(harness):
    harness.link.start_navigation()
    harness.link.drive_navigation(0, 0)
    harness.link._refresh()
    assert harness.http.status["navigation_active"]
    assert not harness.http.status["moving"]
    assert_conflict(harness.link.pulse)
    assert harness.http.actions("/pulse") == []
    assert len(harness.udp.commands("drive")) == 1


def test_already_armed_session_cannot_be_restarted(harness):
    harness.link.start_navigation()
    assert_conflict(harness.link.start_navigation)
    assert len(harness.http.actions("/navigation/start")) == 1
    assert harness.udp.packets == []


def test_remote_armed_session_blocks_pulse_and_navigation_even_without_local_session(harness):
    harness.http.status.update(navigation_active=True, control_mode="navigation",
                               control_session=str(uuid.uuid4()), moving=False)
    assert_conflict(harness.link.pulse)
    assert_conflict(harness.link.start_navigation)
    assert harness.http.actions() == []
    assert harness.udp.packets == []


@pytest.mark.parametrize("changes", [{"mac": "AA:BB:CC:DD:EE:02"},
                                     {"device": "Otro dispositivo"},
                                     {"version": 2}, {"version": True},
                                     {"ip": "10.0.0.3"}])
def test_wrong_identity_prevents_navigation_post_and_all_drive_packets(harness, changes):
    harness.http.status.update(changes)
    with pytest.raises(RobotLinkError):
        harness.link.start_navigation()
    assert harness.http.actions() == []
    assert harness.udp.packets == []
    assert_conflict(lambda: harness.link.drive_navigation(100, 100))


@pytest.mark.parametrize("failure", ["bad_navigation_ack", "fail_navigation_start"])
def test_uncertain_start_is_not_retried_and_never_arms_local_motion(harness, failure):
    setattr(harness.http, failure, True)
    with pytest.raises(RobotLinkError):
        harness.link.start_navigation()
    assert len(harness.http.actions("/navigation/start")) == 1
    assert harness.udp.packets == []
    assert_conflict(lambda: harness.link.drive_navigation(100, 100))
    assert harness.udp.packets == []


def test_udp_send_error_is_not_retried(harness):
    harness.link.start_navigation()
    harness.udp.fail_send = True
    with pytest.raises(RobotLinkError):
        harness.link.drive_navigation(100, -100)
    assert len(harness.udp.commands("drive")) == 1
    assert harness.http.actions("/pulse") == []


def test_udp_stop_error_still_invalidates_session_and_attempts_http_stop(harness):
    harness.link.start_navigation()
    harness.udp.fail_send = True
    harness.link.stop_navigation()
    assert len(harness.udp.commands("stop")) == 1
    assert len(harness.http.actions("/stop")) == 1
    assert not harness.link._snapshot()["navigation_active"]
    assert_conflict(lambda: harness.link.drive_navigation(100, 100))
    assert harness.udp.commands("drive") == []


def test_stop_callback_can_inspect_state_without_deadlocking(harness):
    observations = []
    harness.link.set_navigation_cancel_callback(
        lambda reason: observations.append((reason, harness.link._snapshot())))
    harness.link.start_navigation()
    errors = []

    def stop():
        try:
            harness.link.stop()
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=stop, daemon=True)
    worker.start()
    worker.join(timeout=1)
    assert not worker.is_alive(), "El callback de parada no debe ejecutarse bajo el cerrojo de estado."
    assert errors == []
    assert len(observations) == 1
    assert isinstance(observations[0][0], str) and observations[0][0]
    assert len(harness.udp.commands("stop")) == 1
    assert_conflict(lambda: harness.link.drive_navigation(100, 100))


def test_explicit_stop_navigation_does_not_recursively_notify_navigation_callback(harness):
    calls = []
    harness.link.set_navigation_cancel_callback(calls.append)
    harness.link.start_navigation()
    harness.link.stop_navigation()
    assert calls == []
    harness.link.start_navigation()
    harness.link.stop(notify_navigation=False)
    assert calls == []


def test_closing_active_navigation_sends_stop_and_never_new_drive(harness):
    harness.link.start_navigation()
    harness.link.drive_navigation(0, 0)
    drive_count = len(harness.udp.commands("drive"))
    harness.link.close()
    assert len(harness.udp.commands("drive")) == drive_count
    assert len(harness.udp.commands("stop")) == 1
    assert all(sock.closed for sock in harness.udp.sockets)
    assert_conflict(lambda: harness.link.drive_navigation(100, 100))
