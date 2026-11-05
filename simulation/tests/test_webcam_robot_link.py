"""Comunicación del panel con un robot simulado, sin USB, Wi-Fi ni motores."""
from collections import deque
import json
from pathlib import Path
import threading
import time

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from simulation.webcam.robot_link import (
    HttpRobotClient, RobotLink, RobotLinkError, make_robot_router,
)


MAC = "AA:BB:CC:DD:EE:01"
TOKEN = "0123456789abcdef0123456789abcdef"
PORT = "/dev/cu.usbmodem-test"
HEADERS = {"X-Requested-With": "XMLHttpRequest"}


def status(**changes):
    return {"device": "RobotRacers", "version": 1, "mac": MAC, "boot_id": "test-boot",
            "ip": "192.168.1.80", "connected": True, "ready": True,
            "moving": False, "last_id": "", "stop_reason": "boot", **changes}


def wait_until(predicate, timeout=2.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.005)
    raise AssertionError("El robot simulado no llegó al estado esperado.")


class FakeHttp:
    def __init__(self):
        self.status = status()
        self.calls = []
        self.fail_pulse = False
        self.bad_ack = False
        self.pulse_gate = None
        self.pulse_entered = threading.Event()

    def request(self, method, url, *, payload=None, headers=None, timeout=None):
        self.calls.append((method, url, payload, headers, timeout))
        if method == "GET":
            return dict(self.status)
        if url.endswith("/pulse"):
            self.pulse_entered.set()
            self.status.update(moving=True, last_id=payload["id"])
            if self.pulse_gate is not None:
                self.pulse_gate.wait(2)
            if self.fail_pulse:
                raise TimeoutError("información interna que no debe devolverse")
            return {"accepted": True, "id": "incorrecto" if self.bad_ack else payload["id"], "moving": True}
        if url.endswith("/stop"):
            self.status.update(moving=False, stop_reason="remote")
            return {"stopped": True}
        raise AssertionError(url)

    def actions(self):
        return [call for call in self.calls if call[0] == "POST"]


class FakeSerial:
    def __init__(self, initial=None, use_wifi=True):
        self.current = status(token=TOKEN) if initial is None else dict(initial)
        self.use_wifi = use_wifi
        self.writes = []
        self.lines = deque()
        self.closed = False

    def write(self, data):
        command = json.loads(data)
        self.writes.append(command)
        if command["command"] == "wifi" and self.use_wifi:
            self.current.update(connected=True, ip="192.168.1.80")
        self.lines.append((json.dumps(self.current) + "\n").encode())

    def readline(self, size):
        if self.lines:
            return self.lines.popleft()
        time.sleep(.002)
        return b""

    def close(self):
        self.closed = True


def build_link(tmp_path=None, *, serial=None, http=None, **kwargs):
    serial = serial or FakeSerial()
    http = http or FakeHttp()
    opened = []
    def factory(port):
        opened.append(port)
        return serial
    configuration = {"provision_timeout_s": .2, "refresh_ttl_s": .01, **kwargs}
    link = RobotLink(runtime_path=None if tmp_path is None else tmp_path / "robot-link.json",
                     serial_factory=factory, http_client=http,
                     ports_provider=lambda: {"ports": [{"path": PORT, "label": "Robot USB"}]},
                     **configuration)
    return link, serial, http, opened


def connect_ready(link, **kwargs):
    link.connect(PORT, **kwargs)
    wait_until(lambda: link._snapshot()["connected"])
    wait_until(lambda: link._worker is None or not link._worker.is_alive())
    return link._snapshot()


def test_constructor_ports_and_router_do_not_open_usb_or_send_network_commands():
    link, serial, http, opened = build_link()
    app = FastAPI(); app.include_router(make_robot_router(link))
    try:
        with TestClient(app) as client:
            assert client.get("/api/robot/state").json()["phase"] == "disconnected"
            assert client.get("/api/robot/ports").json()["ports"][0]["path"] == PORT
        assert opened == [] and serial.writes == [] and http.calls == []
    finally:
        link.close()


def test_provision_only_sends_credentials_after_robot_identity_and_persists_only_pair(tmp_path):
    serial = FakeSerial(initial=status(connected=False, ip="0.0.0.0", token=TOKEN))
    link, serial, http, opened = build_link(tmp_path, serial=serial)
    try:
        connect_ready(link, ssid="Red de prueba", password="clave-secreta")
        assert serial.writes[0] == {"command": "status"}
        assert serial.writes[1] == {"command": "wifi", "ssid": "Red de prueba", "password": "clave-secreta"}
        assert opened == [PORT] and serial.closed
        value = json.loads(link.runtime_path.read_text())
        assert value == {"ip": "192.168.1.80", "mac": MAC, "token": TOKEN}
        assert link.runtime_path.stat().st_mode & 0o777 == 0o600
        public = json.dumps(link.state())
        assert "clave-secreta" not in public and "Red de prueba" not in public and TOKEN not in public
        assert http.actions() == []
    finally:
        link.close()


def test_empty_ssid_uses_saved_robot_network_without_sending_credentials():
    link, serial, http, _ = build_link()
    try:
        assert connect_ready(link)["ready"]
        assert all(command == {"command": "status"} for command in serial.writes)
        assert http.actions() == []
    finally:
        link.close()


@pytest.mark.parametrize("initial", [status(device="Otro equipo", token=TOKEN),
                                    {"message": "REV3.11 D44 inicia; D48 detiene."}])
def test_unknown_usb_firmware_never_receives_wifi_credentials(initial):
    serial = FakeSerial(initial=initial)
    link, serial, _, _ = build_link(serial=serial)
    try:
        link.connect(PORT, "MiRed", "no-registrar-clave")
        state = wait_until(lambda: (s if (s := link._snapshot())["phase"] == "error" else None))
        assert all(command == {"command": "status"} for command in serial.writes)
        assert "no-registrar-clave" not in json.dumps(state)
        assert serial.closed
    finally:
        link.close()


@pytest.mark.parametrize("ssid,password", [("x" * 33, "password123"), ("é" * 17, "password123"),
                                            ("network\n", "password123"), ("network", "short"),
                                            ("", "password123"), ("network", "x" * 65),
                                            ("network", "a" * 64)])
def test_invalid_wifi_configuration_is_rejected_without_echo_or_io(ssid, password):
    link, serial, http, opened = build_link()
    try:
        with pytest.raises(RobotLinkError) as error:
            link.connect(PORT, ssid, password)
        assert error.value.status_code == 422
        assert password not in str(error.value)
        assert opened == [] and serial.writes == [] and http.calls == []
    finally:
        link.close()


def test_unlisted_serial_port_is_not_opened():
    link, serial, http, opened = build_link()
    try:
        with pytest.raises(RobotLinkError):
            link.connect("/tmp/arbitrary-file")
        assert opened == [] and serial.writes == [] and http.calls == []
    finally:
        link.close()


@pytest.mark.parametrize("ip", ["127.0.0.1", "8.8.8.8", "169.254.1.1", "::1", "192.168.1.80/24", "localhost"])
def test_robot_ip_must_be_private_ipv4_before_network_io(ip):
    serial = FakeSerial(initial=status(ip=ip, token=TOKEN))
    link, _, http, _ = build_link(serial=serial)
    try:
        link.connect(PORT)
        wait_until(lambda: link._snapshot()["phase"] == "error")
        assert http.calls == []
    finally:
        link.close()


@pytest.mark.parametrize("change", [{"mac": "AA:BB:CC:DD:EE:02"}, {"device": "Other"}, {"version": 2}])
def test_identity_is_confirmed_before_every_motor_action(change):
    link, _, http, _ = build_link()
    try:
        connect_ready(link)
        http.status.update(change)
        with pytest.raises(RobotLinkError):
            link.pulse()
        assert http.actions() == []
        assert not link._snapshot()["ready"]
    finally:
        link.close()


def test_pulse_has_exact_pwm_duration_unique_id_and_only_one_post_per_press():
    link, _, http, _ = build_link()
    try:
        connect_ready(link)
        response = link.pulse()
        actions = http.actions()
        assert len(actions) == 1
        method, url, payload, headers, timeout = actions[0]
        assert method == "POST" and url == "http://192.168.1.80/pulse"
        assert payload == {"id": response["id"], "left": 100, "right": 100, "duration_ms": 1000}
        assert headers == {"X-Robot-Token": TOKEN} and timeout == 1.5
        assert response["accepted"] and response["moving"]
        with pytest.raises(RobotLinkError) as busy:
            link.pulse()
        assert busy.value.status_code == 409
        assert len(http.actions()) == 1
        http.status.update(moving=False, stop_reason="timeout")
        link._refresh()
        second = link.pulse()
        assert second["id"] != response["id"] and len(http.actions()) == 2
    finally:
        link.close()


@pytest.mark.parametrize("failure", ["timeout", "wrong_ack"])
def test_uncertain_pulse_is_never_retried(failure):
    link, _, http, _ = build_link()
    try:
        connect_ready(link)
        http.fail_pulse = failure == "timeout"
        http.bad_ack = failure == "wrong_ack"
        with pytest.raises(RobotLinkError):
            link.pulse()
        with pytest.raises(RobotLinkError):
            link.pulse()
        assert len(http.actions()) == 1
        assert not link._snapshot()["ready"]
    finally:
        link.close()


def test_simultaneous_press_does_not_send_a_second_pulse():
    link, _, http, _ = build_link()
    http.pulse_gate = threading.Event()
    try:
        connect_ready(link)
        errors = []
        def pulse():
            try: link.pulse()
            except Exception as exc: errors.append(exc)
        worker = threading.Thread(target=pulse)
        worker.start()
        assert http.pulse_entered.wait(1)
        with pytest.raises(RobotLinkError) as busy:
            link.pulse()
        assert busy.value.status_code == 409 and len(http.actions()) == 1
        http.pulse_gate.set(); worker.join(1)
        assert not worker.is_alive() and errors == []
    finally:
        http.pulse_gate.set()
        link.close()


def test_not_ready_robot_is_linked_but_cannot_move_and_stop_requires_identity():
    http = FakeHttp(); http.status["ready"] = False
    link, _, http, _ = build_link(http=http)
    try:
        assert connect_ready(link)["phase"] == "connected"
        with pytest.raises(RobotLinkError): link.pulse()
        assert http.actions() == []
        assert link.stop()["stop_reason"] == "remote"
        assert http.actions()[0][1].endswith("/stop")
        http.status["mac"] = "AA:BB:CC:DD:EE:03"
        with pytest.raises(RobotLinkError): link.stop()
        assert len(http.actions()) == 1
    finally:
        link.close()


def test_disconnected_status_cannot_keep_the_robot_ready_or_send_a_pulse():
    link, _, http, _ = build_link()
    try:
        connect_ready(link)
        http.status.update(connected=False, ready=True)
        with pytest.raises(RobotLinkError):
            link.pulse()
        assert not link._snapshot()["connected"] and not link._snapshot()["ready"]
        assert http.actions() == []
        link._apply_status(status(connected=False, ready=True), 1.)
        assert not link._snapshot()["connected"] and not link._snapshot()["ready"]
    finally:
        link.close()


def test_state_refresh_is_throttled_and_no_automatic_motor_action_is_created():
    link, _, http, _ = build_link(refresh_ttl_s=1.)
    try:
        connect_ready(link)
        count = len(http.calls)
        for _ in range(20): link.state()
        assert len(http.calls) == count and http.actions() == []
    finally:
        link.close()


def test_none_runtime_path_does_not_load_or_create_pairing_files(tmp_path):
    link, _, http, _ = build_link()
    try:
        assert link.runtime_path is None
        connect_ready(link)
        assert http.actions() == []
    finally:
        link.close()


def test_close_cancels_provision_and_never_sends_motor_commands():
    serial = FakeSerial(initial=status(connected=False, ip="0.0.0.0", token=TOKEN))
    link, serial, http, _ = build_link(serial=serial)
    link.connect(PORT)
    wait_until(lambda: len(serial.writes) > 0)
    link.close()
    assert serial.closed and not link._worker.is_alive()
    assert http.actions() == [] and not link._snapshot()["connected"]


def test_router_requires_local_intent_header_and_never_echoes_bad_wifi_secrets():
    link, serial, http, opened = build_link()
    app = FastAPI(); app.include_router(make_robot_router(link))
    try:
        with TestClient(app) as client:
            for endpoint in ("connect", "pulse", "stop"):
                assert client.post(f"/api/robot/{endpoint}", json={}).status_code == 403
            secret = "private-value-" * 9
            response = client.post("/api/robot/connect", headers=HEADERS,
                                   json={"port": PORT, "ssid": "network", "password": secret})
            assert response.status_code == 422 and secret not in response.text
            assert client.post("/api/robot/pulse", headers=HEADERS, json={"left": 255}).status_code == 422
            assert opened == [] and serial.writes == [] and http.actions() == []
    finally:
        link.close()


def test_default_http_transport_disables_redirects_and_limits_response(monkeypatch):
    from simulation.webcam import robot_link as module
    handlers = []
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self, size): return b"x" * size
    class Opener:
        def open(self, request, timeout): return Response()
    def build(*values):
        handlers.extend(values)
        return Opener()
    monkeypatch.setattr(module, "build_opener", build)
    client = HttpRobotClient()
    assert handlers[0].proxies == {}
    assert handlers[1].redirect_request(None, None, None, None, None, "http://other") is None
    with pytest.raises(RobotLinkError):
        client.request("GET", "http://192.168.1.80/status")
