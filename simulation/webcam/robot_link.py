"""Enlace local del robot: pulso HTTP y navegación mediante mandos UDP finitos.

La provisión inicial utiliza USB; las acciones posteriores utilizan HTTP en la
red local. Importar el módulo o construir RobotLink no abre dispositivos ni
envía órdenes. Las órdenes de movimiento no se reintentan automáticamente.
"""
from __future__ import annotations

import asyncio
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import socket
import threading
import time
from typing import Callable
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener
import uuid

from fastapi import APIRouter, HTTPException, Request as WebRequest


MAX_JSON_BYTES = 8192
RUNTIME = Path(__file__).resolve().parents[1] / ".runtime" / "robot-link.json"
PRIVATE_NETWORKS = tuple(ipaddress.IPv4Network(value) for value in
                         ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16"))


def _numeric_pair(value, *, integers=False):
    if not isinstance(value, list) or len(value) != 2:
        return None
    if any(isinstance(item, bool) or not isinstance(item, (int, float))
           or abs(item) > 1e12 or not math.isfinite(item)
           or (integers and type(item) is not int) for item in value):
        return None
    return list(value)


def _velocity_limit(status):
    value = status.get("max_velocity_mm_s", 100)
    return value if type(value) is int and 1 <= value <= 300 else 0


class RobotLinkError(RuntimeError):
    def __init__(self, message: str, status_code: int = 503):
        super().__init__(message)
        self.status_code = status_code


def _private_ip(value) -> str:
    if not isinstance(value, str):
        raise RobotLinkError("El robot no proporciona una dirección IPv4 privada válida.")
    try:
        address = ipaddress.IPv4Address(value)
    except (ValueError, TypeError, ipaddress.AddressValueError):
        raise RobotLinkError("El robot no proporciona una dirección IPv4 privada válida.") from None
    if not any(address in network for network in PRIVATE_NETWORKS):
        raise RobotLinkError("El robot debe estar en una dirección IPv4 privada de la red local.")
    return str(address)


def _mac(value) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5}", value) is None:
        raise RobotLinkError("No se pudo confirmar la identidad del robot.")
    return value.upper()


def _validate_status(value, expected_mac: str | None = None) -> dict:
    if not isinstance(value, dict) or value.get("device") != "RobotRacers" or type(value.get("version")) is not int or value["version"] != 1:
        raise RobotLinkError("El dispositivo no utiliza el firmware de prueba RobotRacers.")
    mac = _mac(value.get("mac"))
    if expected_mac is not None and mac != expected_mac:
        raise RobotLinkError("La dirección corresponde a otro dispositivo; vuelve a enlazar el robot.")
    for key in ("connected", "ready", "moving"):
        if type(value.get(key)) is not bool:
            raise RobotLinkError("El estado recibido del robot no es válido.")
    return {**value, "mac": mac}


def _wifi_values(ssid: str, password: str) -> None:
    if any(ord(char) < 32 or ord(char) == 127 for char in ssid + password):
        raise RobotLinkError("El nombre de red o la clave contienen caracteres no válidos.", 422)
    if len(ssid.encode("utf-8")) > 32:
        raise RobotLinkError("El nombre de red no puede superar 32 bytes.", 422)
    if not ssid:
        if password:
            raise RobotLinkError("Indica el nombre de red para configurar una clave nueva.", 422)
        return
    length = len(password.encode("utf-8"))
    if length and not 8 <= length <= 63:
        raise RobotLinkError("La clave Wi-Fi debe tener entre 8 y 63 bytes, o quedar vacía si la red es abierta.", 422)


def serial_ports() -> dict:
    try:
        from serial.tools import list_ports
        values = [{"path": item.device, "label": item.description or item.device}
                  for item in list_ports.comports()]
        values.sort(key=lambda item: ("usb" not in item["path"].lower(), item["path"]))
        return {"ports": values, "message": "" if values else "Conecta el robot por USB para enlazarlo."}
    except ImportError:
        return {"ports": [], "message": "Falta instalar la dependencia de comunicación USB."}


def _open_serial(port: str):
    import serial
    return serial.Serial(port, 115200, timeout=.15, write_timeout=1)


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class HttpRobotClient:
    """HTTP local, sin proxies, sin reintentos y con respuestas de tamaño acotado."""
    def __init__(self):
        self._opener = build_opener(ProxyHandler({}), _NoRedirect())

    def request(self, method: str, url: str, *, payload=None, headers=None, timeout=1.5):
        request_headers = {"Accept": "application/json", **(headers or {})}
        data = None
        if payload is not None:
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            request_headers["Content-Type"] = "application/json"
        request = Request(url, data=data, headers=request_headers, method=method)
        try:
            with self._opener.open(request, timeout=timeout) as response:
                data = response.read(MAX_JSON_BYTES + 1)
            if len(data) > MAX_JSON_BYTES:
                raise RobotLinkError("La respuesta del robot supera el tamaño permitido.")
            value = json.loads(data)
            if not isinstance(value, dict):
                raise ValueError()
            return value
        except HTTPError as exc:
            exc.close()
            if exc.code == 409:
                raise RobotLinkError("El robot está ocupado; espera a que termine.", 409) from None
            if exc.code == 423:
                raise RobotLinkError("El robot recibió una parada. Inicia un recorrido nuevo para continuar.", 423) from None
            if exc.code in (401, 403):
                raise RobotLinkError("El robot rechazó la identificación; vuelve a enlazarlo por USB.") from None
            raise RobotLinkError("El robot rechazó la solicitud.") from None
        except (TimeoutError, socket.timeout):
            raise RobotLinkError("El robot no respondió a tiempo.") from None
        except (URLError, OSError):
            raise RobotLinkError("No se pudo conectar con el robot por Wi-Fi.") from None
        except (ValueError, UnicodeError):
            raise RobotLinkError("La respuesta del robot no es válida.") from None


class RobotLink:
    def __init__(self, runtime_path: Path | None = RUNTIME, *, serial_factory: Callable | None = None,
                 http_client=None, ports_provider: Callable | None = None,
                 provision_timeout_s: float = 25., refresh_ttl_s: float = 1.,
                 http_timeout_s: float = 1.5, udp_socket_factory: Callable | None = None):
        self.runtime_path = Path(runtime_path) if runtime_path is not None else None
        self.serial_factory = serial_factory or _open_serial
        self.http = http_client or HttpRobotClient()
        self.ports_provider = ports_provider or serial_ports
        self.provision_timeout_s = provision_timeout_s
        self.refresh_ttl_s = refresh_ttl_s
        self.http_timeout_s = http_timeout_s
        self._lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._udp_lock = threading.Lock()
        self._udp_socket_factory = udp_socket_factory or (lambda: socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        self._udp_socket = None
        self._navigation_session = None
        self._navigation_sequence = 0
        self._navigation_cancel_callback = None
        self._closed = threading.Event()
        self._worker: threading.Thread | None = None
        self._serial = None
        self._pair: dict | None = None
        self._last_refresh = 0.
        self._pending_motion = False
        self._state = {"phase": "disconnected", "message": "Robot sin enlazar.",
                       "connected": False, "ready": False, "moving": False, "ip": None,
                       "port": None, "round_trip_ms": None, "last_id": None, "stop_reason": None,
                       "navigation_active": False, "control_session": None, "control_seq": 0,
                       "boot_id": None, "resume_blocked": False, "velocity_control": False,
                       "encoder_ready": False, "encoder_counts": None, "speed_mm_s": None,
                       "applied_pwm": None, "startup": None, "max_velocity_mm_s": 100}
        self._load_pair()

    def _load_pair(self):
        if self.runtime_path is None:
            return
        try:
            if self.runtime_path.stat().st_size > MAX_JSON_BYTES:
                return
            value = json.loads(self.runtime_path.read_text())
            pair = self._valid_pair(value)
        except (OSError, ValueError, RobotLinkError):
            return
        with self._lock:
            self._pair = pair
            self._state.update(ip=pair["ip"], message="Comprobando el enlace guardado.")

    @staticmethod
    def _valid_pair(value):
        if not isinstance(value, dict) or not isinstance(value.get("token"), str) or re.fullmatch(r"[0-9a-fA-F]{32}", value["token"]) is None:
            raise RobotLinkError("El robot no proporciona una identificación de enlace válida.")
        return {"ip": _private_ip(value.get("ip")), "mac": _mac(value.get("mac")), "token": value["token"]}

    def _persist_pair(self, pair):
        if self.runtime_path is None:
            return
        self.runtime_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.runtime_path.with_name(self.runtime_path.name + ".tmp")
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        try:
            os.fchmod(fd, 0o600)
            with os.fdopen(fd, "w") as stream:
                json.dump(pair, stream, separators=(",", ":"))
            os.replace(temporary, self.runtime_path)
        except BaseException:
            try:
                os.close(fd)
            except OSError:
                pass
            raise

    def ports(self):
        try:
            result = self.ports_provider()
            return result if isinstance(result, dict) else {"ports": result}
        except Exception:
            return {"ports": [], "message": "No se pudo consultar los puertos USB."}

    def _snapshot(self):
        with self._lock:
            return dict(self._state)

    def state(self):
        with self._lock:
            should_refresh = (self._pair is not None and not self._closed.is_set()
                              and self._state["phase"] not in ("provisioning", "connecting")
                              and time.monotonic() - self._last_refresh >= self.refresh_ttl_s
                              and not (self._worker and self._worker.is_alive()))
            if should_refresh:
                self._last_refresh = time.monotonic()
                self._worker = threading.Thread(target=self._refresh, name="robot-status", daemon=True)
                self._worker.start()
            return dict(self._state)

    def _request(self, method, pair, path, payload=None, authenticated=False):
        headers = {"X-Robot-Token": pair["token"]} if authenticated else {}
        started = time.monotonic()
        value = self.http.request(method, f"http://{pair['ip']}{path}", payload=payload,
                                  headers=headers, timeout=self.http_timeout_s)
        return value, (time.monotonic() - started) * 1000

    def _read_status(self, pair):
        value, elapsed = self._request("GET", pair, "/status")
        status = _validate_status(value, pair["mac"])
        if not status["connected"]:
            raise RobotLinkError("El robot indica que perdió la conexión Wi-Fi.")
        if status.get("ip"):
            if _private_ip(status["ip"]) != pair["ip"]:
                raise RobotLinkError("La dirección del robot cambió; vuelve a enlazarlo por USB.")
        return status, elapsed

    def _apply_status(self, status, elapsed):
        connected = status["connected"]
        moving = status["moving"]
        ready = connected and status["ready"]
        with self._lock:
            if self._closed.is_set():
                return
            if connected and not moving:
                self._pending_motion = False
            last_id = status.get("last_id")
            if not isinstance(last_id, str) or len(last_id) > 64:
                last_id = None
            reason = status.get("stop_reason")
            if not isinstance(reason, str) or re.fullmatch(r"[a-zA-Z_\- ]{0,48}", reason) is None:
                reason = None
            self._state.update(phase="error" if not connected else "moving" if moving else "ready" if ready else "connected",
                               message="El robot perdió la conexión Wi-Fi." if not connected else
                               "Controlando la velocidad con encoders." if moving and status.get("navigation_active") and status.get("velocity_control") else
                               "Ejecutando prueba de 1 segundo." if moving else
                               "Robot listo; encoders disponibles." if ready and status.get("encoder_ready") else
                               "Robot listo para la prueba." if ready else "Robot enlazado; comprueba alimentación y estado de los drivers.",
                               connected=connected, ready=ready, moving=moving,
                               round_trip_ms=round(elapsed, 1), last_id=last_id, stop_reason=reason,
                               navigation_active=status.get("navigation_active") is True,
                               control_session=status.get("control_session"),
                               control_seq=status.get("control_seq", 0), boot_id=status.get("boot_id"),
                               resume_blocked=status.get("resume_blocked") is True,
                               velocity_control=status.get("velocity_control") is True,
                               encoder_ready=status.get("encoder_ready") is True,
                               max_velocity_mm_s=_velocity_limit(status),
                               encoder_counts=_numeric_pair(status.get("encoder_counts", status.get("count")), integers=True),
                               speed_mm_s=_numeric_pair(status.get("speed_mm_s")),
                               applied_pwm=_numeric_pair(status.get("applied_pwm"), integers=True),
                               startup=(list(status["startup"]) if isinstance(status.get("startup"), list)
                                        and len(status["startup"]) == 2
                                        and all(type(item) is bool for item in status["startup"]) else None))
            self._last_refresh = time.monotonic()

    def _error(self, message):
        with self._lock:
            if not self._closed.is_set():
                self._state.update(phase="error", message=message, connected=False,
                                   ready=False, moving=False)
                self._last_refresh = time.monotonic()

    def _refresh(self):
        if not self._io_lock.acquire(blocking=False):
            return
        try:
            with self._lock:
                pair = dict(self._pair) if self._pair else None
            if pair and not self._closed.is_set():
                status, elapsed = self._read_status(pair)
                self._apply_status(status, elapsed)
        except RobotLinkError as exc:
            self._error(str(exc))
        except Exception:
            self._error("No se pudo comprobar el estado del robot.")
        finally:
            self._io_lock.release()

    def connect(self, port: str, ssid: str = "", password: str = ""):
        if not isinstance(port, str) or not isinstance(ssid, str) or not isinstance(password, str):
            raise RobotLinkError("La configuración del enlace no es válida.", 422)
        _wifi_values(ssid, password)
        if port not in {item.get("path") for item in self.ports().get("ports", []) if isinstance(item, dict)}:
            raise RobotLinkError("Selecciona un puerto USB disponible del robot.", 422)
        with self._lock:
            if self._closed.is_set():
                raise RobotLinkError("El servicio del robot está cerrado.", 409)
            if self._navigation_session or self._state["moving"] or self._pending_motion or (self._worker and self._worker.is_alive()):
                raise RobotLinkError("Espera a que termine la operación actual del robot.", 409)
            self._pair = None
            self._state.update(phase="provisioning", message="Comprobando el robot por USB…",
                               connected=False, ready=False, moving=False, ip=None, port=port,
                               last_id=None, stop_reason=None, round_trip_ms=None)
            self._worker = threading.Thread(target=self._provision, args=(port, ssid, password),
                                            name="robot-provision", daemon=True)
            self._worker.start()
            return dict(self._state)

    def _provision(self, port, ssid, password):
        serial = None
        deadline = time.monotonic() + self.provision_timeout_s
        try:
            with self._io_lock:
                serial = self.serial_factory(port)
                with self._lock:
                    self._serial = serial
                configured = False
                buffer = b""
                next_query = 0.
                while not self._closed.is_set() and time.monotonic() < deadline:
                    now = time.monotonic()
                    if now >= next_query:
                        serial.write(b'{"command":"status"}\n')
                        next_query = now + .5
                    raw = serial.readline(MAX_JSON_BYTES + 1)
                    if not raw:
                        self._closed.wait(.01)
                        continue
                    if len(raw) > MAX_JSON_BYTES:
                        buffer = b""
                        continue
                    buffer += raw
                    if len(buffer) > MAX_JSON_BYTES:
                        buffer = b""
                        continue
                    if not buffer.endswith(b"\n"):
                        continue
                    line, buffer = buffer, b""
                    try:
                        status = _validate_status(json.loads(line))
                    except (ValueError, UnicodeError, RobotLinkError):
                        continue
                    # No se envían credenciales a un puerto que no se haya
                    # identificado primero como el firmware esperado.
                    if ssid and not configured:
                        command = {"command": "wifi", "ssid": ssid, "password": password}
                        serial.write((json.dumps(command, separators=(",", ":")) + "\n").encode("utf-8"))
                        configured = True
                        with self._lock:
                            self._state.update(phase="connecting", message="Conectando el robot a la red Wi-Fi…")
                        continue
                    if not status["connected"]:
                        continue
                    pair = self._valid_pair(status)
                    with self._lock:
                        self._state.update(phase="connecting", ip=pair["ip"], message="Comprobando el enlace Wi-Fi…")
                    try:
                        remote, elapsed = self._read_status(pair)
                    except RobotLinkError:
                        continue
                    if self._closed.is_set():
                        return
                    self._persist_pair(pair)
                    with self._lock:
                        self._pair = pair
                    self._apply_status(remote, elapsed)
                    return
                if not self._closed.is_set():
                    raise RobotLinkError("No se completó el enlace. Comprueba el firmware, la red de 2,4 GHz y la alimentación.")
        except RobotLinkError as exc:
            self._error(str(exc))
        except Exception:
            self._error("No se pudo enlazar el robot por USB. Comprueba el puerto y cierra el monitor serie.")
        finally:
            # Las credenciales solo existen durante esta llamada; no se guardan
            # en archivos ni se incluyen en errores o registros.
            ssid = password = ""
            if serial is not None:
                try:
                    serial.close()
                except Exception:
                    pass
            with self._lock:
                if self._serial is serial:
                    self._serial = None

    def pulse(self):
        if not self._io_lock.acquire(blocking=False):
            raise RobotLinkError("El robot está ocupado; espera un momento.", 409)
        identifier = None
        try:
            with self._lock:
                if self._closed.is_set() or not self._pair:
                    raise RobotLinkError("Enlaza el robot antes de iniciar la prueba.", 409)
                if self._navigation_session or self._state["moving"] or self._pending_motion:
                    raise RobotLinkError("El robot ya está ejecutando la prueba.", 409)
                pair = dict(self._pair)
            remote, elapsed = self._read_status(pair)
            self._apply_status(remote, elapsed)
            if remote["moving"] or remote.get("navigation_active") is True:
                raise RobotLinkError("El robot ya está ejecutando la prueba.", 409)
            if not remote["ready"]:
                raise RobotLinkError("El robot no está listo. Comprueba alimentación y drivers.", 409)
            identifier = str(uuid.uuid4())
            with self._lock:
                if self._closed.is_set():
                    raise RobotLinkError("El servicio del robot está cerrado.", 409)
                self._pending_motion = True
                self._state.update(phase="moving", moving=True, last_id=identifier,
                                   message="Enviando prueba de 1 segundo…")
            ack, _ = self._request("POST", pair, "/pulse", {"id": identifier, "left": 100,
                                  "right": 100, "duration_ms": 1000}, authenticated=True)
            if ack.get("accepted") is not True or ack.get("id") != identifier or type(ack.get("moving")) is not bool:
                raise RobotLinkError("No se pudo confirmar la orden. No se reenviará automáticamente.")
            with self._lock:
                self._pending_motion = ack["moving"]
                self._state.update(phase="moving" if ack["moving"] else "ready", moving=ack["moving"],
                                   message="Prueba enviada: 100 / 100 durante 1000 ms.")
                self._last_refresh = time.monotonic() if ack["moving"] else 0.
            return {**self._snapshot(), "accepted": True, "id": identifier}
        except RobotLinkError as exc:
            if exc.status_code != 409:
                self._error(str(exc) if identifier is None else
                            "No se confirmó la orden; no se reenviará. El robot tiene parada local de un segundo.")
            raise
        except Exception:
            self._error("No se confirmó la orden; no se reenviará. El robot tiene parada local de un segundo.")
            raise RobotLinkError("No se pudo confirmar la prueba; no se reenviará automáticamente.") from None
        finally:
            self._io_lock.release()

    def set_navigation_cancel_callback(self, callback):
        self._navigation_cancel_callback = callback

    def start_navigation(self, *, resume=False, require_velocity=False, required_velocity_mm_s=0):
        """Arma una sesión nueva sin aplicar velocidad ni reintentar órdenes."""
        if type(required_velocity_mm_s) is not int or not 0 <= required_velocity_mm_s <= 300:
            raise RobotLinkError("El límite de velocidad requerido no es válido.", 422)
        if not self._io_lock.acquire(timeout=self.http_timeout_s + .2):
            raise RobotLinkError("El enlace está ocupado; vuelve a intentarlo.", 409)
        try:
            with self._lock:
                if self._closed.is_set() or not self._pair:
                    raise RobotLinkError("Enlaza el robot antes de iniciar el recorrido.", 409)
                if self._navigation_session or self._pending_motion or self._state["moving"]:
                    raise RobotLinkError("El robot ya tiene una operación activa.", 409)
                pair = dict(self._pair)
            remote, elapsed = self._read_status(pair)
            self._apply_status(remote, elapsed)
            if (require_velocity or required_velocity_mm_s) and (remote.get("velocity_control") is not True or remote.get("encoder_ready") is not True):
                raise RobotLinkError("Falta cargar el firmware de velocidad con encoders o los encoders no están listos.", 409)
            if _velocity_limit(remote) < required_velocity_mm_s:
                raise RobotLinkError("La velocidad seleccionada supera el límite del firmware. Actualiza la ESP32 por USB.", 409)
            if resume and (remote.get("resume_blocked") is True or remote.get("stop_reason") in
                           {"button_stop", "driver_fault", "encoder_stall", "encoder_fault", "encoder_error", "remote", "remote_stop", "manual_stop"}):
                raise RobotLinkError("El robot recibió una parada. El recorrido requiere un nuevo inicio explícito.", 423)
            if not remote["ready"] or remote["moving"] or remote.get("navigation_active"):
                raise RobotLinkError("El robot no está disponible para iniciar el recorrido.", 409)
            session = str(uuid.uuid4())
            payload = {"session": session, **({"resume": True} if resume else {})}
            ack, _ = self._request("POST", pair, "/navigation/start", payload, authenticated=True)
            if ack.get("accepted") is not True or ack.get("session") != session:
                raise RobotLinkError("No se pudo confirmar el inicio; no se reenviará automáticamente.")
            with self._lock:
                if self._closed.is_set():
                    raise RobotLinkError("El servicio del robot está cerrado.", 409)
                self._navigation_session = session
                self._navigation_sequence = 0
                self._state.update(navigation_active=True, control_session=session, control_seq=0,
                                   moving=False, stop_reason="navigating", resume_blocked=False,
                                   message="Recorrido iniciado.")
                self._last_refresh = time.monotonic()
            return {"session": session}
        finally:
            self._io_lock.release()

    def drive_navigation(self, left, right):
        return self._drive_command("drive", left, right)

    def drive_velocity_navigation(self, left_mm_s, right_mm_s):
        """Consignas de velocidad de rueda en mm/s; el ESP32 regula el PWM."""
        return self._drive_command("velocity", left_mm_s, right_mm_s)

    def _drive_command(self, command, left, right):
        maximum = 300 if command == "velocity" else 100
        if any(type(value) is not int or not -maximum <= value <= maximum for value in (left, right)):
            raise RobotLinkError(f"Las velocidades deben ser enteros entre {-maximum} y {maximum}.", 422)
        with self._udp_lock:
            with self._lock:
                if self._closed.is_set() or not self._navigation_session or not self._pair:
                    raise RobotLinkError("El recorrido no tiene una sesión activa.", 409)
                if command == "velocity" and (not self._state["velocity_control"] or not self._state["encoder_ready"]):
                    raise RobotLinkError("El control por encoders no está disponible; el robot debe detenerse.", 409)
                if command == "velocity" and max(abs(left), abs(right)) > self._state["max_velocity_mm_s"]:
                    raise RobotLinkError("La consigna supera el límite anunciado por el firmware.", 409)
                pair = dict(self._pair)
                session = self._navigation_session
                self._navigation_sequence = (self._navigation_sequence + 1) & 0xffffffff
                sequence = self._navigation_sequence
            packet = {"token": pair["token"], "session": session, "seq": sequence,
                      "command": command, "left": left, "right": right, "lease_ms": 300}
            self._send_navigation_packet(pair, packet)

    def _send_navigation_packet(self, pair, packet):
        try:
            if self._udp_socket is None:
                self._udp_socket = self._udp_socket_factory()
                # UDP solo transporta el último mando; nunca esperar una cola de red.
                self._udp_socket.setblocking(False)
            self._udp_socket.sendto(json.dumps(packet, separators=(",", ":")).encode(), (pair["ip"], 4210))
        except OSError:
            raise RobotLinkError("No se pudo enviar la velocidad al robot.") from None

    def _stop_udp_navigation(self):
        with self._udp_lock:
            with self._lock:
                session = self._navigation_session
                pair = dict(self._pair) if self._pair else None
                self._navigation_session = None
                self._navigation_sequence = (self._navigation_sequence + 1) & 0xffffffff
                sequence = self._navigation_sequence
                self._state.update(navigation_active=False, control_session=None)
            if session and pair:
                self._send_navigation_packet(pair, {"token": pair["token"], "session": session,
                    "seq": sequence, "command": "stop", "left": 0, "right": 0, "lease_ms": 300})

    def stop_navigation(self):
        return self.stop(notify_navigation=False)

    def abandon_navigation(self):
        """Invalida el mando local sin esperar HTTP; el destino lo conserva el controlador."""
        try:
            self._stop_udp_navigation()
        except RobotLinkError:
            pass

    def stop(self, notify_navigation=True):
        if notify_navigation and self._navigation_cancel_callback:
            try:
                self._navigation_cancel_callback("Parada solicitada.")
            except Exception:
                pass  # La cancelación del panel nunca debe impedir la parada física.
        try:
            self._stop_udp_navigation()
        except RobotLinkError:
            pass  # La parada HTTP sigue intentándose; queda además el plazo local de 300 ms.
        with self._io_lock:
            with self._lock:
                if not self._pair or self._closed.is_set():
                    raise RobotLinkError("El robot no está enlazado.", 409)
                pair = dict(self._pair)
            try:
                remote, elapsed = self._read_status(pair)
                self._apply_status(remote, elapsed)
                self._request("POST", pair, "/stop", {}, authenticated=True)
                with self._lock:
                    self._pending_motion = False
                    self._state.update(phase="ready" if remote["ready"] else "connected", moving=False,
                                       navigation_active=False, control_session=None,
                                       message="Parada enviada al robot.", stop_reason="remote")
                    self._last_refresh = 0.
                return self._snapshot()
            except RobotLinkError as exc:
                self._error(str(exc))
                raise
            except Exception:
                self._error("No se pudo confirmar la parada del robot.")
                raise RobotLinkError("No se pudo confirmar la parada del robot.") from None

    def close(self):
        """Libera USB y lectores; cerrar el servicio nunca ordena movimiento."""
        try:
            self._stop_udp_navigation()
        except RobotLinkError:
            pass
        self._closed.set()
        with self._udp_lock:
            if self._udp_socket is not None:
                self._udp_socket.close()
                self._udp_socket = None
        with self._lock:
            serial = self._serial
            worker = self._worker
            self._state.update(phase="disconnected", connected=False, ready=False, moving=False,
                               message="Servicio del robot cerrado.")
        if serial is not None:
            try:
                serial.close()
            except Exception:
                pass
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2.)


def make_robot_router(link: RobotLink) -> APIRouter:
    router = APIRouter(prefix="/api/robot")

    def perform(action, *args):
        try:
            return action(*args)
        except RobotLinkError as exc:
            raise HTTPException(exc.status_code, str(exc)) from None

    async def action_body(request: WebRequest, allowed: set[str]) -> dict:
        if request.headers.get("X-Requested-With") != "XMLHttpRequest":
            raise HTTPException(403, "La solicitud debe realizarse desde el panel local.")
        data = bytearray()
        async for part in request.stream():
            data.extend(part)
            if len(data) > MAX_JSON_BYTES:
                raise HTTPException(413, "La solicitud supera el tamaño permitido.")
        try:
            value = json.loads(data) if data else {}
        except (ValueError, UnicodeError):
            raise HTTPException(422, "La solicitud no es válida.") from None
        if not isinstance(value, dict) or not set(value).issubset(allowed):
            raise HTTPException(422, "La solicitud contiene campos no permitidos.")
        return value

    @router.get("/state")
    def state():
        return link.state()

    @router.get("/ports")
    def ports():
        return link.ports()

    @router.post("/connect")
    async def connect(request: WebRequest):
        payload = await action_body(request, {"port", "ssid", "password"})
        return await asyncio.to_thread(perform, link.connect, payload.get("port"),
                                       payload.get("ssid", ""), payload.get("password", ""))

    @router.post("/pulse")
    async def pulse(request: WebRequest):
        await action_body(request, set())
        return await asyncio.to_thread(perform, link.pulse)

    @router.post("/stop")
    async def stop(request: WebRequest):
        await action_body(request, set())
        return await asyncio.to_thread(perform, link.stop)

    return router
