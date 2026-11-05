"""Navegación local a un punto con la cámara cenital y una sesión de mando finita.

La selección y la calibración nunca envían órdenes. El inicio es explícito;
una misión activa espera sin moverse si pierde visión o enlace y continúa al
recuperarlos. D48, una parada manual o un fallo de driver cancelan la misión.
Las distancias se estiman en el plano horizontal del marcador de 40 mm.
"""
from __future__ import annotations

import asyncio
import json
import math
from numbers import Real
from pathlib import Path
import threading
import time

from fastapi import APIRouter, HTTPException, Request

from .navigation_geometry import Calibration, calibrate_robot, observe_robot

INT32_MAX = 2_147_483_647
MAX_TURN_DEG_S = math.floor(INT32_MAX * 180. / (95. * math.pi))


class NavigationError(RuntimeError):
    def __init__(self, message: str, status_code: int = 409):
        super().__init__(message)
        self.status_code = status_code


def _finite(value, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise NavigationError(f"{label} debe ser un número finito.", 422)
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise NavigationError(f"{label} debe ser un número finito.", 422) from None
    if not math.isfinite(result):
        raise NavigationError(f"{label} debe ser un número finito.", 422)
    return result


def _velocity_mm_s(value: float) -> int:
    return round(max(-INT32_MAX, min(INT32_MAX, value)))


class Navigation:
    CONTROL_INTERVAL = .05
    FRESH_SECONDS = .20
    VISION_GRACE_SECONDS = .15
    BRIEF_VISION_GAP_SECONDS = .10
    RESUME_STABLE_SECONDS = .15
    RESUME_FRAMES = 3
    RESUME_RETRY_SECONDS = 1.
    SPEED_RISE_MM_S2 = 600.
    SPEED_FALL_MM_S2 = 900.
    RETRY_PAUSE_SECONDS = .5
    SETTLE_SECONDS = .40
    WAYPOINT_SETTLE_SECONDS = .15
    MAX_DURATION_SECONDS = 30.
    NO_PROGRESS_SECONDS = 5.
    STATUS_STARTUP_SECONDS = 1.25  # RobotLink actualiza el estado remoto cada segundo.
    MAX_DISTANCE_M = .60
    ARRIVAL_M = .025
    SETTLED_M = .04
    SPEED_LIMITS = {"forward_min_mm_s": 50, "forward_max_mm_s": INT32_MAX,
                    "turn_min_deg_s": 12, "turn_max_deg_s": MAX_TURN_DEG_S}
    SPEED_SCALE_DEFAULTS = {"forward_mm_s": 300, "turn_deg_s": 100}

    def __init__(self, camera, robot, runtime_path: Path | None = None):
        self.camera = camera
        self.robot = robot
        self._runtime_path = Path(runtime_path) if runtime_path is not None else None
        self._speeds_path = self._runtime_path.with_name("navigation-speeds.json") if self._runtime_path else None
        self._lock = threading.RLock()
        self._persistence_lock = threading.Lock()
        self._calibration: Calibration | None = None
        self._target: list[float] | None = None
        self._waypoints: list[list[float]] = []
        self._waypoint_index = 0
        self._route_generation = 0
        self._last_arrival_mm = None
        self._phase = "uncalibrated"
        self._message = "Marca el centro entre ruedas y después el frente del robot."
        self._active = False
        self._closed = False
        self._thread: threading.Thread | None = None
        self._cancel = threading.Event()
        self._cancel_reason = "Parada solicitada."
        self._session: str | None = None
        self._pending_mission = False
        self._arm_thread: threading.Thread | None = None
        self._arm_result: tuple | None = None
        self._last_left = self._last_right = 0
        self._ramp_values = [0., 0.]
        self._ramp_stamp: float | None = None
        self._velocity_sample = None
        self._measured_speed = self._measured_turn = None
        self._started_at: float | None = None
        self._elapsed_ms = 0
        self._vision_hold = False
        self._vision_gap_ms = 0.
        self._vision_pauses = self._brief_vision_gaps = self._mode_switches = 0
        self._speeds = {"forward_mm_s": 100, "turn_deg_s": 32}
        self._speed_revision = 0
        self._load_calibration()
        self._load_speeds()

    @staticmethod
    def _validated_speeds(forward_mm_s, turn_deg_s):
        if type(forward_mm_s) is not int or not 50 <= forward_mm_s <= INT32_MAX:
            raise NavigationError(f"La velocidad de avance debe ser un entero entre 50 y {INT32_MAX} mm/s.", 422)
        if type(turn_deg_s) is not int or not 12 <= turn_deg_s <= MAX_TURN_DEG_S:
            raise NavigationError(f"La velocidad de giro debe ser un entero entre 12 y {MAX_TURN_DEG_S} grados/s.", 422)
        return {"forward_mm_s": forward_mm_s, "turn_deg_s": turn_deg_s}

    @staticmethod
    def _required_velocity(speeds):
        values = Navigation._validated_speeds(speeds["forward_mm_s"], speeds["turn_deg_s"])
        required = max(values["forward_mm_s"], math.ceil(95. * math.radians(values["turn_deg_s"])))
        if required > INT32_MAX:
            raise NavigationError("La velocidad por rueda excede la representación de 32 bits.", 422)
        return required

    @staticmethod
    def _remote_velocity_limit(remote):
        value = remote.get("max_velocity_mm_s", 100)
        return min(INT32_MAX, value) if type(value) is int and value > 0 else 100

    def get_speeds(self):
        with self._lock:
            return dict(self._speeds)

    def _load_speeds(self):
        if self._speeds_path is None:
            return
        try:
            if self._speeds_path.stat().st_size > 4096:
                return
            value = json.loads(self._speeds_path.read_text(encoding="utf-8"))
            fields = {"forward_mm_s", "turn_deg_s"}
            if isinstance(value, dict) and set(value) in (fields, fields | {"speed_revision"}):
                self._speeds = self._validated_speeds(value["forward_mm_s"], value["turn_deg_s"])
                revision = value.get("speed_revision", 0)
                self._speed_revision = max(0, revision) if type(revision) is int else 0
        except (OSError, ValueError, TypeError, NavigationError):
            pass

    def _save_speeds(self):
        if self._speeds_path is None:
            return
        with self._persistence_lock:
            with self._lock:
                value = {**self._speeds, "speed_revision": self._speed_revision}
            try:
                self._speeds_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self._speeds_path.with_name(self._speeds_path.name + ".tmp")
                temporary.write_text(json.dumps(value, allow_nan=False), encoding="utf-8")
                temporary.chmod(0o600)
                temporary.replace(self._speeds_path)
            except OSError:
                pass

    def set_speeds(self, forward_mm_s, turn_deg_s):
        speeds = self._validated_speeds(forward_mm_s, turn_deg_s)
        remote = self.robot.state()
        if remote.get("connected") and self._required_velocity(speeds) > self._remote_velocity_limit(remote):
            raise NavigationError("Actualiza el firmware para usar estas velocidades; el robot conectado admite un máximo menor.")
        with self._lock:
            if self._closed:
                raise NavigationError("El servicio de navegación está cerrado.")
            self._speeds = speeds
            self._speed_revision += 1
        self._save_speeds()
        return self.state()

    def _load_calibration(self):
        if self._runtime_path is None:
            return
        try:
            path = self._runtime_path
            if not path.exists() and path.name == "navigation-state.json":
                path = path.with_name("navigation-calibration.json")
            if path.stat().st_size > 1_048_576:
                return
            value = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(value, dict) and value.get("version") in (1, 2):
                version = value["version"]
                expected = ({"version", "calibration", "target_px", "pending_mission"} if version == 1
                            else {"version", "calibration", "waypoints_px", "waypoint_index", "pending_mission"})
                if set(value) not in (expected, expected | {"route_revision"}):
                    return
                revision = value.get("route_revision", 0)
                if type(revision) is not int or revision < 0:
                    return
                calibration = (Calibration.from_dict(value["calibration"])
                               if value["calibration"] is not None else None)
                points = ([value["target_px"]] if value["target_px"] is not None else []) if version == 1 else value["waypoints_px"]
                index = 0 if version == 1 else value["waypoint_index"]
                if (not isinstance(points, list) or type(index) is not int or not 0 <= index <= len(points)
                        or type(value["pending_mission"]) is not bool):
                    return
                route = []
                for point in points:
                    if calibration is None or not isinstance(point, list) or len(point) != 2:
                        return
                    point = [_finite(point[0], "El destino"), _finite(point[1], "El destino")]
                    if not (0 <= point[0] < calibration.width and 0 <= point[1] < calibration.height):
                        return
                    route.append(point)
                self._calibration, self._waypoints, self._waypoint_index = calibration, route, index
                self._route_generation = revision
                self._sync_target_locked()
                self._pending_mission = value["pending_mission"] and self._target is not None
            else:
                self._calibration = Calibration.from_dict(value)  # Archivo anterior: solo calibración.
            self._phase = ("arrived" if self._waypoints and self._target is None else
                           "selected" if self._target is not None else "idle"
                           if self._calibration is not None else "uncalibrated")
            self._message = ("Recorrido guardado. Pulsa Iniciar recorrido para continuar tras reiniciar el programa."
                             if self._pending_mission else
                             "Calibración guardada. Comprueba la cámara y el frente antes de iniciar.")
        except (OSError, ValueError, TypeError, NavigationError):
            pass

    def _save_calibration(self):
        if self._runtime_path is None:
            return
        with self._persistence_lock:
            with self._lock:
                calibration = self._calibration
                value = {"version": 2, "calibration": calibration.to_dict() if calibration else None,
                         "waypoints_px": [list(point) for point in self._waypoints],
                         "waypoint_index": self._waypoint_index, "pending_mission": self._pending_mission,
                         "route_revision": self._route_generation}
            try:
                self._runtime_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self._runtime_path.with_name(self._runtime_path.name + ".tmp")
                temporary.write_text(json.dumps(value, allow_nan=False), encoding="utf-8")
                temporary.chmod(0o600)
                temporary.replace(self._runtime_path)
            except OSError:
                # Una sesión puede utilizar la calibración en memoria sin persistencia.
                pass

    def _camera_state(self) -> dict:
        try:
            value = self.camera.state()
            return value if isinstance(value, dict) else {}
        except Exception:
            return {}

    def _sync_target_locked(self):
        self._target = (list(self._waypoints[self._waypoint_index])
                        if self._waypoint_index < len(self._waypoints) else None)

    @staticmethod
    def _same_source(frame: dict, calibration: Calibration) -> bool:
        return (frame.get("camera_id") == calibration.camera_id
                and frame.get("frame_width") == calibration.width
                and frame.get("frame_height") == calibration.height)

    def saved_camera_id(self) -> str | None:
        """Identidad lógica guardada; consultar no valida ni modifica la fuente."""
        with self._lock:
            return self._calibration.camera_id if self._calibration is not None else None

    def _check_source(self, frame: dict):
        """Un cambio efectivo de fuente exige repetir los dos clics."""
        with self._lock:
            calibration = self._calibration
            if calibration is None:
                return
            identity = frame.get("camera_id")
            width, height = frame.get("frame_width"), frame.get("frame_height")
            changed = ((isinstance(identity, str) and identity and identity != calibration.camera_id)
                       or (type(width) is int and width > 0 and width != calibration.width)
                       or (type(height) is int and height > 0 and height != calibration.height))
            if not changed:
                return
            self._calibration = None
            self._route_generation += 1
            self._waypoints = []
            self._waypoint_index = 0
            self._last_arrival_mm = None
            self._target = None
            self._pending_mission = False
            self._message = "Cambió la cámara o la resolución. Repite la calibración."
            if self._active:
                self._cancel_reason = self._message
                self._cancel.set()
                self._phase = "stopping"
            else:
                self._phase = "uncalibrated"
        self._save_calibration()

    def _vision(self, frame: dict) -> bool:
        age = frame.get("frame_age_ms")
        return (frame.get("visible") is True and isinstance(age, Real)
                and not isinstance(age, bool) and math.isfinite(age)
                and 0 <= age <= self.FRESH_SECONDS * 1000
                and frame.get("marker_corners") is not None)

    @staticmethod
    def _request_source(frame: dict, width, height, camera_id):
        if (type(width) is not int or type(height) is not int
                or not 1 <= width <= 8192 or not 1 <= height <= 8192
                or not isinstance(camera_id, str) or not camera_id or len(camera_id) > 512):
            raise NavigationError("La cámara o la resolución de la solicitud no es válida.", 422)
        if (frame.get("camera_id") != camera_id or frame.get("frame_width") != width
                or frame.get("frame_height") != height):
            raise NavigationError("La imagen cambió. Actualiza el panel y repite la selección.")

    def _editable(self):
        if self._closed:
            raise NavigationError("El servicio de navegación está cerrado.")
        if (self._active or (self._thread is not None and self._thread.is_alive())
                or (self._arm_thread is not None and self._arm_thread.is_alive())):
            raise NavigationError("Detén el robot antes de modificar el destino o la calibración.")

    def calibrate(self, center_px, front_px, frame_width, frame_height, camera_id):
        with self._lock:
            generation = self._route_generation
        frame = self._camera_state()
        self._check_source(frame)
        self._request_source(frame, frame_width, frame_height, camera_id)
        if not self._vision(frame):
            raise NavigationError("Se necesita el marcador visible en una imagen reciente.")
        try:
            calibration = calibrate_robot(frame["marker_corners"], center_px, front_px,
                                          camera_id=camera_id, width=frame_width, height=frame_height)
        except ValueError as exc:
            raise NavigationError(str(exc), 422) from None
        with self._lock:
            self._editable()
            if generation != self._route_generation:
                raise NavigationError("La ruta cambió durante la calibración. Repite los dos clics.")
            self._calibration = calibration
            self._route_generation += 1
            self._waypoints = []
            self._waypoint_index = 0
            self._last_arrival_mm = None
            self._target = None
            self._pending_mission = False
            self._phase = "idle"
            self._message = "Centro y frente calibrados. Selecciona un punto cercano."
        self._save_calibration()
        return self.state()

    def _select_target(self, x, y, frame_width, frame_height, camera_id, *, append, expected_route_revision=None):
        if expected_route_revision is not None and (type(expected_route_revision) is not int
                                                    or expected_route_revision < 0):
            raise NavigationError("La revisión de la ruta debe ser un entero no negativo.", 422)
        with self._lock:
            generation = self._route_generation
        target = [_finite(x, "La coordenada X"), _finite(y, "La coordenada Y")]
        frame = self._camera_state()
        self._check_source(frame)
        self._request_source(frame, frame_width, frame_height, camera_id)
        with self._lock:
            self._editable()
            if generation != self._route_generation:
                raise NavigationError("La ruta cambió durante la selección. Vuelve a añadir el punto.")
            if expected_route_revision is not None and expected_route_revision != self._route_generation:
                raise NavigationError("La ruta cambió. Actualiza el panel antes de añadir el punto.")
            calibration = self._calibration
            if calibration is None or not self._same_source(frame, calibration):
                raise NavigationError("Calibra el centro y el frente antes de seleccionar un destino.")
            observation = self._observe(frame, calibration, target, required=True)
            corners = frame["marker_corners"]
            edge = sum(math.hypot(corners[(i + 1) % 4][0] - corners[i][0],
                                  corners[(i + 1) % 4][1] - corners[i][1]) for i in range(4)) / 4
            margin = max(20., edge * .6)
            if not (margin <= target[0] < frame_width - margin
                    and margin <= target[1] < frame_height - margin):
                raise NavigationError("El destino debe quedar más alejado del borde de la imagen.", 422)
            if observation["distance_m"] > self.MAX_DISTANCE_M:
                raise NavigationError("Selecciona un destino a no más de 600 mm del robot.", 422)
            if append:
                self._waypoints.append(target)
                self._route_generation += 1
            else:
                self._route_generation += 1
                self._waypoints = [target]
                self._waypoint_index = 0
                self._last_arrival_mm = None
            self._sync_target_locked()
            self._pending_mission = False
            self._phase = "selected"
            self._message = "Ruta preparada. Inicia el recorrido cuando la zona esté despejada."
        self._save_calibration()
        return self.state()

    def select_target(self, x, y, frame_width, frame_height, camera_id):
        return self._select_target(x, y, frame_width, frame_height, camera_id, append=False)

    def append_target(self, x, y, frame_width, frame_height, camera_id, expected_route_revision):
        if type(expected_route_revision) is not int or expected_route_revision < 0:
            raise NavigationError("La revisión de la ruta debe ser un entero no negativo.", 422)
        return self._select_target(x, y, frame_width, frame_height, camera_id, append=True,
                                   expected_route_revision=expected_route_revision)

    def clear_targets(self):
        with self._lock:
            self._route_generation += 1
            self._waypoints = []
            self._waypoint_index = 0
            self._last_arrival_mm = None
            self._target = None
            self._pending_mission = False
            if self._active:
                self._cancel_reason = "Ruta borrada. Robot detenido."
                self._cancel.set()
                self._phase = "stopping"
            else:
                self._phase = "idle" if self._calibration is not None else "uncalibrated"
            self._message = "Ruta borrada. Robot detenido."
        self._safe_stop()
        self._save_calibration()
        return self.state()

    # Alias útil para clientes Python; ambos métodos solamente seleccionan.
    set_target = select_target

    def _observe(self, frame, calibration, target=None, *, required=False):
        if calibration is None or not self._same_source(frame, calibration) or not self._vision(frame):
            if required:
                raise NavigationError("Se perdió el marcador o la imagen está atrasada. No se iniciará el movimiento.")
            return None
        try:
            return observe_robot(frame["marker_corners"], calibration, target)
        except (ValueError, TypeError, KeyError):
            if required:
                raise NavigationError("No se pudo medir la posición actual del robot.") from None
            return None

    def state(self):
        frame = self._camera_state()
        self._check_source(frame)
        with self._lock:
            calibration = self._calibration
            target = None if self._target is None else list(self._target)
            observation = self._observe(frame, calibration, target)
            return {"phase": self._phase, "active": self._active,
                    "calibrated": calibration is not None and self._same_source(frame, calibration),
                    "message": self._message, "target_px": target,
                    "waypoints_px": [list(point) for point in self._waypoints],
                    "waypoint_index": self._waypoint_index, "waypoint_count": len(self._waypoints),
                    "route_revision": self._route_generation,
                    "last_arrival_mm": self._last_arrival_mm,
                    "pending_mission": self._pending_mission,
                    "vision_grace_active": self._vision_hold, "vision_gap_ms": self._vision_gap_ms,
                    "vision_pauses": self._vision_pauses, "brief_vision_gaps": self._brief_vision_gaps,
                    "mode_switches": self._mode_switches,
                    "speeds": dict(self._speeds), "speed_limits": dict(self.SPEED_LIMITS),
                    "speed_scale_defaults": dict(self.SPEED_SCALE_DEFAULTS),
                    "speed_revision": self._speed_revision,
                    "center_px": observation["center_px"] if observation else None,
                    "front_px": observation["front_px"] if observation else None,
                    "bearing_deg": (round(math.degrees(observation["bearing_rad"]), 2)
                                    if observation and observation["bearing_rad"] is not None else None),
                    "last_left": self._last_left, "last_right": self._last_right,
                    "left_target_mm_s": self._last_left, "right_target_mm_s": self._last_right,
                    "measured_speed_mps": self._measured_speed if observation else None,
                    "measured_turn_deg_s": self._measured_turn if observation else None,
                    "elapsed_ms": (round((time.monotonic() - self._started_at) * 1000)
                                   if self._active and self._started_at is not None else self._elapsed_ms),
                    "sequence": frame.get("sequence") if type(frame.get("sequence")) is int else None,
                    "remaining_mm": (round(observation["distance_m"] * 1000, 1)
                                     if observation and observation["distance_m"] is not None
                                     else self._last_arrival_mm if self._phase == "arrived" else None)}

    def overlay_pose(self, detection):
        # El proveedor se ejecuta en el hilo de captura: solamente lectura y
        # geometría. Invalidar/persistir corresponde a state() y al control.
        frame = self._camera_state()
        with self._lock:
            calibration = self._calibration
        if calibration is None or not self._same_source(frame, calibration):
            return None
        visible = (detection.get("visible") if isinstance(detection, dict)
                   else getattr(detection, "visible", False))
        corners = (detection.get("marker_corners") if isinstance(detection, dict)
                   else getattr(detection, "marker_corners", None))
        if visible is not True:
            return None
        try:
            return observe_robot(corners, calibration)
        except (ValueError, TypeError):
            return None

    def start(self, expected_route_revision=None):
        if expected_route_revision is not None and (type(expected_route_revision) is not int
                                                    or expected_route_revision < 0):
            raise NavigationError("La revisión de la ruta debe ser un entero no negativo.", 422)
        frame = self._camera_state()
        self._check_source(frame)
        with self._lock:
            self._editable()
            if expected_route_revision is not None and expected_route_revision != self._route_generation:
                raise NavigationError("La ruta cambió. Actualiza el panel antes de iniciar.")
            calibration = self._calibration
            if calibration is None or not self._waypoints:
                raise NavigationError("Calibra el robot y selecciona un destino antes de iniciar.")
            index = self._waypoint_index if self._waypoint_index < len(self._waypoints) else 0
            target = list(self._waypoints[index])
            observation = self._observe(frame, calibration, target, required=True)
            if observation["distance_m"] > self.MAX_DISTANCE_M:
                raise NavigationError("El destino quedó a más de 600 mm. Vuelve a seleccionarlo.")
            remote = self.robot.state()
            if not remote.get("connected") or not remote.get("ready"):
                raise NavigationError("Conecta el robot y comprueba que esté listo antes de iniciar.")
            if remote.get("velocity_control") is not True or remote.get("encoder_ready") is not True:
                raise NavigationError("Se requiere el firmware de control de velocidad y encoders disponibles.")
            required_velocity = self._required_velocity(self._speeds)
            if required_velocity > self._remote_velocity_limit(remote):
                raise NavigationError("Actualiza el firmware antes de iniciar con las velocidades configuradas.")
            if remote.get("moving") or remote.get("navigation_active"):
                raise NavigationError("El robot ya tiene un movimiento en curso.")
            self._waypoint_index = index
            self._sync_target_locked()
            self._last_arrival_mm = None
            self._active = True
            self._phase = "arming"
            self._message = "Preparando el recorrido…"
            self._cancel = threading.Event()
            self._arm_result = None
            self._cancel_reason = "Parada solicitada."
            self._started_at = time.monotonic()
            self._elapsed_ms = 0
            self._vision_hold = False
            self._vision_gap_ms = 0.
            self._vision_pauses = self._brief_vision_gaps = self._mode_switches = 0
            self._last_left = self._last_right = 0
            self._ramp_values = [0., 0.]
            self._ramp_stamp = time.monotonic()
            self._velocity_sample = None
            self._measured_speed = self._measured_turn = None
            boot_id = remote.get("boot_id")
        launched = False
        try:
            ack = self.robot.start_navigation(require_velocity=True, required_velocity_mm_s=required_velocity)
            session = ack.get("session") if isinstance(ack, dict) else None
            if not isinstance(session, str) or not session or len(session) > 128:
                raise NavigationError("El robot no confirmó la sesión. No se reenviará automáticamente.", 502)
            with self._lock:
                if self._cancel.is_set() or self._closed:
                    raise NavigationError(self._cancel_reason)
                self._session = session
                self._pending_mission = True
                self._thread = threading.Thread(target=self._run,
                    args=(calibration, list(target), session, boot_id), daemon=True,
                    name="webcam-point-navigation")
                self._thread.start()
                launched = True
            self._save_calibration()
            return self.state()
        except NavigationError:
            raise
        except Exception as exc:
            # RobotLinkError tiene un estado HTTP propio y un mensaje apto para el panel.
            message = str(exc) if hasattr(exc, "status_code") else "No se pudo iniciar el recorrido."
            raise NavigationError(message, getattr(exc, "status_code", 502)) from None
        finally:
            if not launched:
                self._safe_stop()
                with self._lock:
                    self._active = False
                    self._elapsed_ms = round((time.monotonic() - self._started_at) * 1000)
                    self._phase = "stopped" if self._cancel.is_set() else "error"
                    self._message = (self._cancel_reason if self._cancel.is_set()
                                     else "No se confirmó el inicio. Vuelve a iniciar explícitamente.")

    def cancel_local(self, reason: str = "Parada solicitada."):
        """Cancela el trabajador sin IO ni esperas; apto para el callback de RobotLink."""
        with self._lock:
            if not self._closed:
                self._route_generation += 1
            if self._active:
                self._cancel_reason = reason
                self._cancel.set()
                if not self._closed:
                    self._pending_mission = False
                self._phase = "stopping"
                self._message = reason

    def _safe_stop(self):
        try:
            self.robot.stop_navigation()
        except Exception:
            # El firmware mantiene además una caducidad local de la orden de 300 ms.
            pass
        finally:
            with self._lock:
                self._last_left = self._last_right = 0
                self._ramp_values, self._ramp_stamp = [0., 0.], None
                self._vision_hold = False
                self._vision_gap_ms = 0.

    def _drive(self, left, right):
        self.robot.drive_velocity_navigation(left, right)
        with self._lock:
            self._last_left, self._last_right = left, right

    def _zero_drive(self):
        """La pérdida de seguimiento y las paradas omiten la rampa."""
        with self._lock:
            self._ramp_values, self._ramp_stamp = [0., 0.], None
        self._drive(0, 0)

    @staticmethod
    def _ramp_value(current, target, seconds, rise=600., fall=900.):
        # Una inversión llega primero a cero; el signo nuevo empieza en otro ciclo.
        effective = 0. if current * target < 0 else float(target)
        rate = fall if abs(effective) < abs(current) else rise
        step = max(0., seconds) * rate
        return current + max(-step, min(step, effective - current))

    def _ramp_drive(self, left, right):
        now = time.monotonic()
        with self._lock:
            seconds = (self.CONTROL_INTERVAL if self._ramp_stamp is None
                       else max(0., min(.1, now - self._ramp_stamp)))
            self._ramp_stamp = now
            self._ramp_values = [self._ramp_value(current, desired, seconds,
                                self.SPEED_RISE_MM_S2, self.SPEED_FALL_MM_S2)
                                 for current, desired in zip(self._ramp_values, (left, right))]
            outputs = [_velocity_mm_s(value) for value in self._ramp_values]
        self._drive(*outputs)

    def _update_motion_estimate(self, frame, observation, now):
        """Velocidades visuales filtradas; no son lecturas de encoder."""
        if observation is None:
            with self._lock:
                self._velocity_sample = None
                self._measured_speed = self._measured_turn = None
            return
        stamp = now - frame["frame_age_ms"] / 1000
        sample = (frame.get("sequence"), stamp, observation["center_px"], observation["heading_deg"])
        with self._lock:
            previous = self._velocity_sample
            if previous is not None and previous[0] == sample[0]:
                return
            self._velocity_sample = sample
        if previous is None or not .0005 < stamp - previous[1] <= .5:
            return
        seconds = stamp - previous[1]
        try:
            relative = observe_robot(frame["marker_corners"], self._calibration, previous[2])
            speed = -relative["target_local_m"][0] / seconds
            angular = ((observation["heading_deg"] - previous[3] + 180) % 360 - 180) / seconds
        except (ValueError, TypeError, KeyError):
            return
        alpha = 1. - math.exp(-seconds / .2)
        with self._lock:
            self._measured_speed = (speed if self._measured_speed is None
                                    else self._measured_speed + alpha * (speed - self._measured_speed))
            self._measured_turn = (angular if self._measured_turn is None
                                   else self._measured_turn + alpha * (angular - self._measured_turn))

    def _abandon(self):
        """Descarta la sesión sin esperar HTTP; no descarta el destino."""
        try:
            self.robot.abandon_navigation()
        except Exception:
            pass
        with self._lock:
            self._session = None
            self._last_left = self._last_right = 0
            self._ramp_values, self._ramp_stamp = [0., 0.], None
            self._vision_hold = False
            self._vision_gap_ms = 0.

    def _launch_rearm(self, required_velocity):
        cancel = self._cancel

        def arm():
            try:
                ack = self.robot.start_navigation(resume=True, require_velocity=True,
                                                  required_velocity_mm_s=required_velocity)
                session = ack.get("session") if isinstance(ack, dict) else None
                if not isinstance(session, str) or not session or len(session) > 128:
                    raise NavigationError("No se confirmó la nueva sesión.")
                if cancel.is_set() or self._closed:
                    self._abandon()
                result = (session, None)
            except Exception as exc:
                result = (None, exc)
            with self._lock:
                self._arm_result = result

        with self._lock:
            self._arm_result = None
            self._arm_thread = threading.Thread(target=arm, name="webcam-navigation-reconnect", daemon=True)
            self._arm_thread.start()

    def stop(self):
        self.cancel_local()
        self._safe_stop()
        with self._lock:
            self._pending_mission = False
            if not self._active:
                self._phase = "stopped"
                self._message = "Robot detenido. El siguiente recorrido requiere iniciar de nuevo."
        self._save_calibration()
        return self.state()

    def close(self):
        with self._lock:
            self._closed = True
            worker = self._thread
            arming = self._arm_thread
        self.cancel_local("Servicio de navegación cerrado.")
        self._safe_stop()
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2.)
        if arming is not None and arming is not threading.current_thread():
            arming.join(timeout=2.)

    def _set_phase(self, phase, message):
        with self._lock:
            if not self._cancel.is_set():
                if self._phase != phase:
                    self._mode_switches += 1
                    if phase == "paused_vision":
                        self._vision_pauses += 1
                self._phase, self._message = phase, message

    @staticmethod
    def _wheel_commands(mode, bearing, distance, mode_elapsed, *, forward_mm_s=100, turn_deg_s=32):
        """Consignas físicas por rueda en mm/s; el firmware regula con encoders."""
        if mode == "turning":
            angle = abs(math.degrees(bearing))
            braking_rate = math.sqrt(2. * 240. * max(0., angle - 8.))
            target_rate = max(12., min(turn_deg_s, angle * turn_deg_s / 40., braking_rate))
            magnitude = max(20., min(95. * math.radians(turn_deg_s), 95. * math.radians(target_rate)))
            sign = 1 if bearing > 0 else -1
            return _velocity_mm_s(-sign * magnitude), _velocity_mm_s(sign * magnitude)
        base_speed = 50. + (forward_mm_s - 50.) * min(1., max(0., distance) / .15)
        braking_speed = math.sqrt(2. * 450. * max(0., distance * 1000. - 25.))
        speed = max(50., min(base_speed, braking_speed))
        angular_rate = max(-math.radians(12), min(math.radians(12), bearing * 1.5))
        correction = 95. * angular_rate
        left, right = speed - correction, speed + correction
        scale = min(1., forward_mm_s / max(abs(left), abs(right)))
        return _velocity_mm_s(left * scale), _velocity_mm_s(right * scale)

    def _run(self, calibration, target, session, boot_id):
        armed_at = last_tick = time.monotonic()
        active_seconds = no_progress_seconds = 0.
        was_controlling = False
        mode = None
        mode_started = 0.
        best_distance = best_angle = math.inf
        session_seen = False
        waiting_vision = False
        stable_since = None
        stable_frames = 0
        last_sequence = None
        next_arm = 0.
        retry_until = None
        settle_started = None
        last_valid_capture = None
        last_valid_sequence = None
        final_phase, final_message = "error", "Recorrido detenido."
        try:
            while not self._cancel.is_set():
                tick = time.monotonic()
                if was_controlling:
                    active_seconds += tick - last_tick
                    no_progress_seconds += tick - last_tick
                last_tick = tick
                was_controlling = False
                frame = self._camera_state()
                self._check_source(frame)
                if self._cancel.is_set():
                    break
                observation = self._observe(frame, calibration, target)
                # No se conduce con la pose anterior cuando falla una detección.
                if observation is not None and frame["frame_age_ms"] > self.VISION_GRACE_SECONDS * 1000:
                    observation = None
                self._update_motion_estimate(frame, observation, tick)
                if observation is None:
                    stable_since, stable_frames, last_sequence = None, 0, None
                else:
                    sequence = frame.get("sequence")
                    if last_valid_capture is None or sequence != last_valid_sequence:
                        last_valid_capture = tick - frame["frame_age_ms"] / 1000
                        last_valid_sequence = sequence
                    with self._lock:
                        self._vision_hold = False
                        self._vision_gap_ms = 0.
                    if sequence != last_sequence:
                        if stable_since is None:
                            stable_since = tick
                        stable_frames += 1
                        last_sequence = sequence
                stable = (observation is not None and stable_frames >= self.RESUME_FRAMES
                          and stable_since is not None and tick - stable_since >= self.RESUME_STABLE_SECONDS)
                remote = self.robot.state()  # Snapshot; el refresco HTTP se hace fuera de este hilo.
                speeds = self.get_speeds()
                required_velocity = self._required_velocity(speeds)
                if remote.get("connected") and (remote.get("velocity_control") is not True
                                                or remote.get("encoder_ready") is not True):
                    raise NavigationError("El control de velocidad con encoders dejó de estar disponible.")
                reason = remote.get("stop_reason")
                if reason in {"encoder_stall", "encoder_fault", "encoder_error"}:
                    raise NavigationError("No hubo respuesta del encoder durante 5 segundos. Revisa la rueda y el cableado."
                                          if reason == "encoder_stall" else
                                          "No se pudo leer un encoder. Revisa la conexión antes de iniciar de nuevo.")
                if remote.get("resume_blocked") is True:
                    raise NavigationError("El robot mantiene una parada manual o un fallo. Recorrido cancelado.")
                if reason in {"button_stop", "driver_fault", "remote", "remote_stop", "manual_stop"}:
                    messages = {"button_stop": "Se pulsó D48. Recorrido cancelado.",
                                "driver_fault": "El driver de un motor notificó un fallo. Recorrido cancelado."}
                    raise NavigationError(messages.get(reason, "El robot recibió una parada manual. Recorrido cancelado."))

                if remote.get("connected") and required_velocity > self._remote_velocity_limit(remote):
                    if session is not None or remote.get("navigation_active"):
                        self._abandon()
                    session, session_seen = None, False
                    waiting_vision = True
                    self._set_phase("paused_connection", "Actualiza el firmware para continuar con estas velocidades.")
                    self._cancel.wait(self.CONTROL_INTERVAL)
                    continue

                with self._lock:
                    arm_result = self._arm_result
                    self._arm_result = None
                    arming = bool(self._arm_thread and self._arm_thread.is_alive())
                if arm_result is not None:
                    new_session, error = arm_result
                    if error is None:
                        session, armed_at, session_seen = new_session, tick, False
                        boot_id = remote.get("boot_id")
                        mode = None
                        with self._lock:
                            self._session = session
                        remote = self.robot.state()
                        reason = remote.get("stop_reason")
                        if remote.get("resume_blocked") is True:
                            raise NavigationError("El robot mantiene una parada manual o un fallo. Recorrido cancelado.")
                    else:
                        if getattr(error, "status_code", None) == 423:
                            raise NavigationError("El robot recibió una parada manual o un fallo. Recorrido cancelado.")
                        session = None
                        next_arm = tick + self.RESUME_RETRY_SECONDS

                current_boot = remote.get("boot_id")
                rebooted = boot_id is not None and current_boot is not None and current_boot != boot_id
                if current_boot is not None:
                    boot_id = current_boot
                connected = remote.get("connected") is True and remote.get("ready") is True
                active_session = remote.get("navigation_active") is True
                remote_session = remote.get("control_session")
                invalid_session = False
                if session is not None:
                    if active_session and remote_session == session:
                        session_seen = True
                    elif session_seen or tick - armed_at > self.STATUS_STARTUP_SECONDS:
                        invalid_session = True
                    if invalid_session and reason in {"remote", "remote_stop", "navigation_stop", "manual_stop"}:
                        raise NavigationError("El robot recibió una parada manual. Recorrido cancelado.")
                    if (not connected or rebooted or invalid_session or reason == "control_timeout"):
                        self._abandon()
                        session, session_seen = None, False
                        waiting_vision = True
                        mode = None
                        # Reconfirmar varios fotogramas antes de restaurar un enlace.
                        stable_since, stable_frames, last_sequence = None, 0, None
                        stable = False
                        next_arm = tick

                if session is None:
                    self._set_phase("paused_connection", "Esperando conexión y seguimiento. El destino se conserva.")
                    if (connected and stable and not arming and tick >= next_arm
                            and not remote.get("moving") and not remote.get("navigation_active")):
                        self._launch_rearm(required_velocity)  # HTTP en otro trabajador, sin bloquear el control.
                        next_arm = tick + self.RESUME_RETRY_SECONDS
                    self._cancel.wait(self.CONTROL_INTERVAL)
                    continue

                age_ms = frame.get("frame_age_ms")
                can_hold = (observation is None and not waiting_vision
                            and last_valid_capture is not None
                            and 0 <= tick - last_valid_capture <= self.BRIEF_VISION_GAP_SECONDS
                            and self._same_source(frame, calibration)
                            and frame.get("running") is True and frame.get("opening") is False
                            and isinstance(age_ms, Real) and not isinstance(age_ms, bool)
                            and math.isfinite(age_ms) and 0 <= age_ms <= self.VISION_GRACE_SECONDS * 1000
                            and connected and active_session and remote_session == session)
                if can_hold:
                    with self._lock:
                        if not self._vision_hold:
                            self._brief_vision_gaps += 1
                        self._vision_hold = True
                        self._vision_gap_ms = round((tick - last_valid_capture) * 1000, 1)
                        held = (self._last_left, self._last_right)
                        self._ramp_stamp = tick
                    # Solo conserva el mando: no acelera, recalcula ni confirma llegada.
                    try:
                        self._drive(*held)
                        was_controlling = True
                    except Exception:
                        self._abandon()
                        session, session_seen = None, False
                        waiting_vision = True
                    remaining_gap = last_valid_capture + self.BRIEF_VISION_GAP_SECONDS - time.monotonic()
                    self._cancel.wait(min(self.CONTROL_INTERVAL, max(0., remaining_gap)))
                    continue

                if observation is None:
                    waiting_vision = True
                    with self._lock:
                        self._vision_hold = False
                        self._vision_gap_ms = 0.

                if observation is None or (waiting_vision and not stable):
                    self._set_phase("paused_vision", "Esperando una imagen estable. El destino se conserva.")
                    try:
                        self._zero_drive()  # Renueva la sesión sin mover el robot.
                    except Exception:
                        self._abandon()
                        session, session_seen = None, False
                    self._cancel.wait(self.CONTROL_INTERVAL)
                    continue
                if waiting_vision:
                    waiting_vision = False
                    mode = None
                    retry_until = None
                    no_progress_seconds = 0.
                    best_distance, best_angle = math.inf, math.inf
                distance, bearing = observation["distance_m"], observation["bearing_rad"]
                angle = abs(bearing)
                if retry_until is not None:
                    self._set_phase("retrying", "Pausa breve para volver a estimar el recorrido.")
                    self._zero_drive()
                    if tick >= retry_until:
                        retry_until = None
                        mode = None
                        no_progress_seconds = 0.
                        best_distance = best_angle = math.inf
                    self._cancel.wait(self.CONTROL_INTERVAL)
                    continue
                if mode == "adjusting":
                    left = right = 0
                    if self._last_left == 0 and self._last_right == 0:
                        mode = None  # Recalcular dirección una vez frenadas ambas ruedas.
                elif mode == "settling":
                    left = right = 0
                    if self._last_left == 0 and self._last_right == 0:
                        if settle_started is None:
                            settle_started = active_seconds
                    else:
                        settle_started = None
                    with self._lock:
                        settle_duration = (self.WAYPOINT_SETTLE_SECONDS if
                                           self._waypoint_index + 1 < len(self._waypoints) else self.SETTLE_SECONDS)
                    if settle_started is not None and active_seconds - settle_started >= settle_duration:
                        if distance <= self.SETTLED_M:
                            with self._lock:
                                if self._cancel.is_set():
                                    break
                                self._last_arrival_mm = round(distance * 1000, 1)
                                self._waypoint_index += 1
                                self._sync_target_locked()
                                next_target = None if self._target is None else list(self._target)
                                if next_target is None:
                                    self._pending_mission = False
                            self._save_calibration()
                            if next_target is None:
                                final_phase, final_message = "arrived", "Ruta completada. Robot detenido."
                                break
                            target = next_target
                            mode = None
                            retry_until = settle_started = None
                            no_progress_seconds = 0.
                            best_distance = best_angle = math.inf
                            self._cancel.wait(self.CONTROL_INTERVAL)
                            continue
                        mode = None
                elif distance <= self.ARRIVAL_M:
                    mode, mode_started = "settling", active_seconds
                    settle_started = None
                    self._set_phase(mode, "Deteniendo el robot en el destino…")
                    left = right = 0
                else:
                    desired = mode
                    if mode is None:
                        desired = "turning" if angle > math.radians(8) else "advancing"
                    elif mode == "turning" and angle <= math.radians(8):
                        desired = "advancing"
                    elif mode == "advancing" and angle > math.radians(25):
                        desired = "turning"
                    if desired != mode and mode is not None:
                        mode = "adjusting"
                        self._set_phase(mode, "Frenando suavemente antes de cambiar de movimiento…")
                    elif desired != mode:
                        mode, mode_started = desired, active_seconds
                        best_angle = angle
                        self._set_phase(mode, "Orientando hacia el destino…" if mode == "turning"
                                        else "Avanzando hacia el destino…")
                    if distance < best_distance - .003:
                        best_distance, no_progress_seconds = distance, 0.
                    if mode == "turning" and angle < best_angle - math.radians(3):
                        best_angle, no_progress_seconds = angle, 0.
                    if no_progress_seconds >= self.NO_PROGRESS_SECONDS:
                        retry_until = tick + self.RETRY_PAUSE_SECONDS
                        self._set_phase("retrying", "Pausa breve para volver a estimar el recorrido.")
                        self._zero_drive()
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    if mode == "adjusting":
                        left = right = 0
                    else:
                        left, right = self._wheel_commands(mode, bearing, distance, active_seconds - mode_started,
                                                           **speeds)
                try:
                    self._ramp_drive(left, right)
                    was_controlling = True
                except Exception:
                    self._abandon()
                    session, session_seen = None, False
                    waiting_vision = True
                    stable_since, stable_frames, last_sequence = None, 0, None
                    mode = None
                self._cancel.wait(max(0., self.CONTROL_INTERVAL - (time.monotonic() - tick)))
            if self._cancel.is_set():
                final_phase, final_message = "stopped", self._cancel_reason
        except NavigationError as exc:
            final_message = str(exc)
        except Exception:
            final_message = "No se pudo mantener el control del recorrido. Robot detenido."
        finally:
            self._safe_stop()
            with self._lock:
                self._session = None
                self._active = False
                if self._cancel.is_set():
                    final_phase, final_message = "stopped", self._cancel_reason
                elif final_phase != "arrived" and not self._closed:
                    # Una parada física/fallo también invalida órdenes Go antiguas.
                    self._route_generation += 1
                if not self._closed:
                    self._pending_mission = False
                self._elapsed_ms = round((time.monotonic() - self._started_at) * 1000)
                self._phase, self._message = final_phase, final_message
            self._save_calibration()


def make_navigation_router(navigation: Navigation) -> APIRouter:
    router = APIRouter(prefix="/api/navigation")

    async def action_body(request: Request, fields: set[str] | tuple[set[str], ...]) -> dict:
        if request.headers.get("X-Requested-With") != "XMLHttpRequest":
            raise HTTPException(403, "La solicitud debe realizarse desde el panel local.")
        data = bytearray()
        async for part in request.stream():
            data.extend(part)
            if len(data) > 8192:
                raise HTTPException(413, "La solicitud supera el tamaño permitido.")
        try:
            def reject_constant(_):
                raise ValueError()
            value = json.loads(data, parse_constant=reject_constant) if data else {}
        except (ValueError, UnicodeError):
            raise HTTPException(422, "La solicitud no es válida.") from None
        shapes = fields if isinstance(fields, tuple) else (fields,)
        if not isinstance(value, dict) or not any(set(value) == shape for shape in shapes):
            raise HTTPException(422, "La solicitud contiene campos incorrectos.")
        return value

    def perform(action, *args):
        try:
            return action(*args)
        except NavigationError as exc:
            raise HTTPException(exc.status_code, str(exc)) from None

    @router.get("/state")
    def state():
        return navigation.state()

    @router.post("/calibrate")
    async def calibrate(request: Request):
        data = await action_body(request, {"center_px", "front_px", "frame_width", "frame_height", "camera_id"})
        return await asyncio.to_thread(perform, navigation.calibrate, data["center_px"], data["front_px"],
                                       data["frame_width"], data["frame_height"], data["camera_id"])

    @router.post("/target")
    async def target(request: Request):
        data = await action_body(request, {"x", "y", "frame_width", "frame_height", "camera_id"})
        return await asyncio.to_thread(perform, navigation.select_target, data["x"], data["y"],
                                       data["frame_width"], data["frame_height"], data["camera_id"])

    @router.post("/targets")
    async def targets(request: Request):
        data = await action_body(request, {"x", "y", "frame_width", "frame_height", "camera_id", "expected_route_revision"})
        return await asyncio.to_thread(perform, navigation.append_target, data["x"], data["y"],
                                       data["frame_width"], data["frame_height"], data["camera_id"],
                                       data["expected_route_revision"])

    @router.post("/clear-targets")
    async def clear_targets(request: Request):
        await action_body(request, set())
        return await asyncio.to_thread(perform, navigation.clear_targets)

    @router.post("/speeds")
    async def speeds(request: Request):
        data = await action_body(request, {"forward_mm_s", "turn_deg_s"})
        return await asyncio.to_thread(perform, navigation.set_speeds,
                                       data["forward_mm_s"], data["turn_deg_s"])

    @router.post("/start")
    async def start(request: Request):
        data = await action_body(request, (set(), {"expected_route_revision"}))
        revision = data.get("expected_route_revision")
        if data and (type(revision) is not int or revision < 0):
            raise HTTPException(422, "La revisión de la ruta debe ser un entero no negativo.")
        return await asyncio.to_thread(perform, navigation.start, revision)

    @router.post("/stop")
    async def stop(request: Request):
        await action_body(request, set())
        return await asyncio.to_thread(perform, navigation.stop)

    return router
