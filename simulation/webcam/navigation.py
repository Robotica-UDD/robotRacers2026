"""Navegación local a un punto con la cámara cenital y una sesión de mando finita.

La selección y la calibración nunca envían órdenes. El inicio es explícito;
una misión activa tolera una pérdida breve de visión conservando el mando;
si se prolonga o pierde el enlace, espera y continúa al recuperarlos.
D48, una parada manual o un fallo de driver cancelan la misión.
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
from .robot_footprint import ROBOT_RADIUS_MM
from .pose_follower import PoseFollower
from .pose_obstacle_planner import (plan_pose_path, check_pose, check_swept_motion,
                                    heading_error_rad, check_wheel_motion)
from .straight_control import StraightControl
from .obstacle_planner import (plan_path, segment_is_clear, point_clearances_mm,
                               camera_visibility_margin_mm, validate_obstacles)

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
    CONTINUATION_MAX_SECONDS = .40
    CONTINUATION_MAX_TRAVEL_MM = 60.
    CONTINUATION_MAX_SWEEP_DEG = 15.
    TURN_CONFIRM_SECONDS = .25
    TURN_CONFIRM_FRAMES = 3
    RESUME_STABLE_SECONDS = .15
    RESUME_FRAMES = 3
    RESUME_RETRY_SECONDS = 1.
    PLAN_RETRY_SECONDS = 1.
    SPEED_RISE_MM_S2 = 600.
    SPEED_FALL_MM_S2 = 900.
    RETRY_PAUSE_SECONDS = .5
    SETTLE_SECONDS = .40
    WAYPOINT_SETTLE_SECONDS = .15
    MAX_DURATION_SECONDS = 30.
    NO_PROGRESS_SECONDS = 5.
    STATUS_STARTUP_SECONDS = 1.25  # RobotLink actualiza el estado remoto cada segundo.
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
        self._obstacles_path = self._runtime_path.with_name("navigation-obstacles.json") if self._runtime_path else None
        self._lock = threading.RLock()
        self._persistence_lock = threading.Lock()
        self._planning_lock = threading.Lock()
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
        self._obstacles = []
        self._obstacle_load_error = None
        self._obstacle_source = None
        self._safety_margin_mm = 0
        self._planned_path = []
        self._inflated_obstacles = []
        self._planned_legs = []
        self._avoidance_points = []
        self._avoidance_index = 0
        self._steering_target = None
        self._planning_status = "empty"
        self._planning_message = "Añade puntos al recorrido."
        self._plan_key = None
        self._plan_attempted_at = None
        self._plan_center = None
        self._plan_heading = None
        self._pose_legs = []
        self._pose_steps = []
        self._pose_step_index = 0
        self._clearance_info = None
        self._load_calibration()
        self._load_speeds()
        self._load_obstacles()

    def _obstacle_settings(self):
        return {"robot_radius_mm": ROBOT_RADIUS_MM, "safety_margin_mm": self._safety_margin_mm,
                "tracking_margin_mm": 0 if self._safety_margin_mm == 0 else 40}

    def _uses_oriented_footprint(self):
        return bool(self._obstacles) and self._safety_margin_mm == 0

    @staticmethod
    def _camera_view_margin(calibration, safety_margin, *, planning=False):
        if safety_margin == 0:
            return 0.  # La pose comprueba el contorno real del marcador, no un círculo.
        return camera_visibility_margin_mm(calibration,
            tracking_margin_mm=40 if planning and safety_margin > 0 else 0,
            extra_margin_mm=0 if safety_margin == 0 else 20)

    def _load_obstacles(self):
        if self._obstacles_path is None or not self._obstacles_path.exists():
            return
        try:
            if self._obstacles_path.stat().st_size > 65536:
                raise ValueError()
            value = json.loads(self._obstacles_path.read_text(encoding="utf-8"))
            if not isinstance(value, dict) or set(value) != {"version", "source", "polygons_px", "safety_margin_mm"}:
                raise ValueError()
            if type(value["version"]) is not int or value["version"] != 1 or type(value["safety_margin_mm"]) is not int or not 0 <= value["safety_margin_mm"] <= 200:
                raise ValueError()
            source = value["source"]
            if source is None:
                if value["polygons_px"] != []:
                    raise ValueError()
                polygons = []
            else:
                if not isinstance(source, dict) or set(source) != {"camera_id", "frame_width", "frame_height"}:
                    raise ValueError()
                self._request_source(source, source["frame_width"], source["frame_height"], source["camera_id"])
                polygons = validate_obstacles(value["polygons_px"], source["frame_width"], source["frame_height"])
            self._obstacles, self._obstacle_source = polygons, source
            self._safety_margin_mm = value["safety_margin_mm"]
        except (OSError, ValueError, TypeError, KeyError, NavigationError):
            self._obstacle_load_error = "No se pudo leer el mapa guardado. Bórralo o dibújalo de nuevo."

    def _save_obstacles(self):
        if self._obstacles_path is None:
            return
        with self._persistence_lock:
            with self._lock:
                if self._obstacle_load_error:
                    return  # Solo borrar o dibujar explícitamente libera un mapa rechazado.
                value = {"version": 1, "source": self._obstacle_source,
                         "polygons_px": self._obstacles, "safety_margin_mm": self._safety_margin_mm}
                encoded = json.dumps(value, allow_nan=False)
            try:
                self._obstacles_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self._obstacles_path.with_name(self._obstacles_path.name + ".tmp")
                temporary.write_text(encoded, encoding="utf-8")
                temporary.chmod(0o600)
                temporary.replace(self._obstacles_path)
            except OSError:
                pass

    @staticmethod
    def _source_changed(frame, source):
        if source is None:
            return False
        identity, width, height = (frame.get(field) for field in ("camera_id", "frame_width", "frame_height"))
        return ((isinstance(identity, str) and identity and identity != source["camera_id"])
                or (type(width) is int and width > 0 and width != source["frame_width"])
                or (type(height) is int and height > 0 and height != source["frame_height"]))

    def _invalidate_plan_locked(self):
        self._planned_path = []
        self._inflated_obstacles = []
        self._planned_legs = []
        self._avoidance_points = []
        self._avoidance_index = 0
        self._steering_target = None
        self._planning_status = "waiting" if self._waypoints else "empty"
        self._planning_message = "Esperando posición para calcular el recorrido." if self._waypoints else "Añade puntos al recorrido."
        self._plan_key = self._plan_center = self._plan_heading = None
        self._plan_attempted_at = None
        self._pose_legs, self._pose_steps, self._pose_step_index = [], [], 0
        self._clearance_info = None

    def _clearance_diagnostics(self, frame, calibration, obstacles, safety_margin):
        if calibration is None or not self._same_source(frame, calibration) or not self._vision(frame):
            return None
        try:
            center = observe_robot(frame["marker_corners"], calibration)["center_px"]
            values = point_clearances_mm(frame["marker_corners"], calibration, center, obstacles)
            pose = (check_pose(frame["marker_corners"], calibration, center,
                               math.radians(observe_robot(frame["marker_corners"], calibration)["heading_deg"]),
                               obstacles) if safety_margin == 0 and obstacles else None)
            return {"footprint_gap_mm": pose["clearance_mm"] if pose else None,
                    "obstacle_mm": values["obstacle_mm"] if math.isfinite(values["obstacle_mm"]) else None,
                    "view_mm": values["camera_mm"] if math.isfinite(values["camera_mm"]) else None,
                    "view_required_mm": self._camera_view_margin(calibration, safety_margin),
                    "obstacle_required_mm": 0. if pose else ROBOT_RADIUS_MM + safety_margin}
        except (ValueError, KeyError, TypeError):
            return None

    def _check_route_revision(self, expected):
        if type(expected) is not int or expected < 0:
            raise NavigationError("La revisión de la ruta debe ser un entero no negativo.", 422)
        if expected != self._route_generation:
            raise NavigationError("La ruta o el mapa cambió. Actualiza el panel antes de editar.")

    def set_obstacles(self, polygons_px, frame_width, frame_height, camera_id, expected_route_revision):
        frame = self._camera_state()
        self._check_source(frame)
        self._request_source(frame, frame_width, frame_height, camera_id)
        try:
            polygons = validate_obstacles(polygons_px, frame_width, frame_height)
        except ValueError as exc:
            raise NavigationError(str(exc), 422) from None
        with self._lock:
            self._editable()
            self._check_route_revision(expected_route_revision)
            self._obstacles = polygons
            self._obstacle_load_error = None
            self._obstacle_source = {"camera_id": camera_id, "frame_width": frame_width, "frame_height": frame_height}
            self._route_generation += 1
            self._invalidate_plan_locked()
        self._save_obstacles()
        self._save_calibration()
        return self.state()

    def set_obstacle_settings(self, safety_margin_mm, expected_route_revision):
        if type(safety_margin_mm) is not int or not 0 <= safety_margin_mm <= 200:
            raise NavigationError("El margen de seguridad debe ser un entero entre 0 y 200 mm.", 422)
        with self._lock:
            self._editable()
            self._check_route_revision(expected_route_revision)
            if self._obstacle_load_error:
                raise NavigationError(self._obstacle_load_error)
            self._safety_margin_mm = safety_margin_mm
            self._route_generation += 1
            self._invalidate_plan_locked()
        self._save_obstacles()
        self._save_calibration()
        return self.state()

    def clear_obstacles(self):
        with self._lock:
            self._route_generation += 1
            self._obstacles, self._obstacle_source = [], None
            self._obstacle_load_error = None
            self._invalidate_plan_locked()
            self._pending_mission = False
            if self._active:
                self._cancel_reason = "Obstáculos borrados. Robot detenido."
                self._cancel.set()
                self._phase = "stopping"
            self._message = "Obstáculos borrados. Inicia de nuevo para continuar."
        self._safe_stop()
        self._save_obstacles()
        self._save_calibration()
        return self.state()

    def _calculate_plan(self, frame, calibration, points, index, obstacles, settings, generation):
        if self._obstacle_load_error:
            return {"status": "blocked", "message": self._obstacle_load_error, "path": [],
                    "inflated": [], "legs": [], "center": None}
        if not points:
            return {"status": "empty", "message": "Añade puntos al recorrido.", "path": [],
                    "inflated": [], "legs": [], "center": None}
        if calibration is None or not self._same_source(frame, calibration) or not self._vision(frame):
            return {"status": "waiting", "message": "Se necesita el marcador visible para calcular el recorrido.",
                    "path": [], "inflated": [], "legs": [], "center": None}
        try:
            center = observe_robot(frame["marker_corners"], calibration)["center_px"]
        except (ValueError, TypeError, KeyError):
            return {"status": "waiting", "message": "No se pudo medir la posición para planificar.",
                    "path": [], "inflated": [], "legs": [], "center": None}
        goals = points[index:]
        if not obstacles:
            return {"status": "ready" if goals else "empty", "message": "Recorrido preparado." if goals else "Añade puntos al recorrido.",
                    "path": [center] + goals if goals else [], "inflated": [], "legs": [], "center": center}
        if settings["safety_margin_mm"] == 0:
            path, legs, pose_legs = [center], [], []
            heading = math.radians(observe_robot(frame["marker_corners"], calibration)["heading_deg"])
            first_heading = heading
            start = center
            try:
                for number, goal in enumerate(goals, index + 1):
                    with self._lock:
                        if generation != self._route_generation or self._cancel.is_set() and self._active:
                            return {"status": "waiting", "message": "El recorrido cambió durante el cálculo.",
                                    "path": [], "inflated": [], "legs": [], "center": center}
                    result = plan_pose_path(frame["marker_corners"], calibration, start, goal, obstacles,
                                            start_heading_rad=heading, allow_start_overlap=True)
                    path.extend(result["path_px"][1:])
                    # A coincident, free destination is a zero-speed step. It must
                    # count as arrival rather than endlessly asking for another plan.
                    steps = result["steps"] or [{"target_px": list(goal), "heading_rad": heading,
                                                "direction": 1, "kind": "drive"}]
                    if any(step["kind"] == "drive" and step["direction"] != 1 for step in steps):
                        raise ValueError("La ruta necesita recalcularse para avanzar de frente.")
                    pose_legs.append(steps)
                    legs.append([list(step["target_px"]) for step in steps] or [list(goal)])
                    start = goal
                    if steps:
                        heading = steps[-1]["heading_rad"]
            except ValueError as exc:
                return {"status": "blocked", "message": f"Punto {number}: {exc}",
                        "path": [], "inflated": [], "legs": [], "center": center}
            return {"status": "ready", "message": "Recorrido con maniobras preparado.",
                    "path": path, "inflated": [], "legs": legs, "pose_legs": pose_legs,
                    "center": center, "heading": first_heading}
        path, legs, inflated, start = [center], [], [], center
        number = index + 1
        try:
            planning_clearance = (settings["robot_radius_mm"] + settings["safety_margin_mm"]
                                  + settings["tracking_margin_mm"])
            view_clearance = self._camera_view_margin(calibration, settings["safety_margin_mm"], planning=True)
            # Validar todos los destinos antes de buscar rutas a los anteriores.
            for number, goal in enumerate(goals, index + 1):
                distances = point_clearances_mm(frame["marker_corners"], calibration, goal, obstacles)
                if distances["obstacle_mm"] <= planning_clearance:
                    return {"status": "blocked", "message": f"Punto {number}: destino a {distances['obstacle_mm']:.0f} mm de un obstáculo; la envolvente del robot requiere {planning_clearance:.0f} mm.",
                            "path": [], "inflated": [], "legs": [], "center": center}
                if distances["camera_mm"] <= view_clearance:
                    return {"status": "blocked", "message": f"Punto {number}: el destino está cerca del borde de la imagen ({distances['camera_mm']:.0f} mm); el marcador requiere {view_clearance:.0f} mm visibles.",
                            "path": [], "inflated": [], "legs": [], "center": center}
            for number, goal in enumerate(goals or [center], index + 1):
                with self._lock:
                    if generation != self._route_generation or (self._active and self._cancel.is_set()):
                        return {"status": "waiting", "message": "El recorrido cambió durante el cálculo.",
                                "path": [], "inflated": [], "legs": [], "center": center}
                result = plan_path(frame["marker_corners"], calibration, start, goal, obstacles,
                                   **settings, max_segment_m=.45,
                                   camera_clearance_mm=view_clearance)
                inflated = result["inflated_obstacles_px"]
                leg = [list(point) for point in result["path_px"][1:]] or [list(goal)]
                if goals:
                    path.extend(leg)
                    legs.append(leg)
                start = goal
        except ValueError as exc:
            return {"status": "blocked" if goals else "empty", "message": f"Punto {number}: {exc}",
                    "path": [], "inflated": inflated, "legs": [], "center": center}
        return {"status": "ready" if goals else "empty", "message": "Recorrido preparado." if goals else "Añade puntos al recorrido.",
                "path": path if goals else [], "inflated": inflated, "legs": legs, "center": center}

    def _refresh_plan(self, frame=None, *, force=False, runtime=False):
        frame = self._camera_state() if frame is None else frame
        if not self._planning_lock.acquire(blocking=force):
            return None
        try:
            with self._lock:
                if self._active and not runtime:
                    return None
                calibration = self._calibration
                points = [list(point) for point in self._waypoints]
                index = self._waypoint_index if self._waypoint_index < len(points) else (0 if not self._active else len(points))
                generation = self._route_generation
                obstacles = [[list(point) for point in polygon] for polygon in self._obstacles]
                settings = self._obstacle_settings()
                key = (generation, index, calibration)
                cached_key, cached_center = self._plan_key, self._plan_center
                cached_status, attempted_at = self._planning_status, self._plan_attempted_at
            reuse_result = (cached_status == "ready" or attempted_at is not None
                            and time.monotonic() - attempted_at < self.PLAN_RETRY_SECONDS)
            if (not force and reuse_result and cached_key == key
                    and cached_center is not None and self._vision(frame)):
                try:
                    cached_observation = observe_robot(frame["marker_corners"], calibration, cached_center)
                    same_heading = (settings["safety_margin_mm"] != 0 or not obstacles or
                                    self._plan_heading is not None and abs(heading_error_rad(
                                        frame["marker_corners"], calibration, self._plan_heading)) < math.radians(3))
                    if cached_observation["distance_m"] < .01 and same_heading:
                        return None
                except (ValueError, TypeError):
                    pass
            if runtime:
                with self._lock:
                    self._planning_status, self._planning_message = "waiting", "Calculando recorrido…"
            result = self._calculate_plan(frame, calibration, points, index, obstacles, settings, generation)
            with self._lock:
                if (generation != self._route_generation or calibration != self._calibration
                        or (self._active and not runtime)):
                    return None
                self._planning_status, self._planning_message = result["status"], result["message"]
                self._planned_path, self._inflated_obstacles = result["path"], result["inflated"]
                self._planned_legs = result["legs"]
                self._plan_attempted_at = time.monotonic()
                self._plan_key, self._plan_center = key, result["center"]
                self._plan_heading = result.get("heading")
                if self._plan_heading is None and calibration is not None and self._vision(frame):
                    try:
                        self._plan_heading = math.radians(observe_robot(frame["marker_corners"], calibration)["heading_deg"])
                    except (ValueError, TypeError, KeyError):
                        pass
                self._pose_legs = result.get("pose_legs", [])
                self._pose_steps = self._pose_legs[0] if self._pose_legs else []
                self._pose_step_index = 0
                if obstacles and result["legs"]:
                    self._avoidance_points, self._avoidance_index = result["legs"][0], 0
                    self._steering_target = list(self._avoidance_points[0])
                else:
                    self._avoidance_points, self._avoidance_index = [], 0
                    self._steering_target = list(points[index]) if index < len(points) else None
            return result
        finally:
            self._planning_lock.release()

    def _runtime_plan(self, frame, calibration):
        # El robot permanece sin consigna de movimiento durante A*, incluso si
        # el cálculo dura más que la sesión UDP y esta debe armarse otra vez.
        try:
            self._zero_drive()
        except Exception:
            pass
        result = self._refresh_plan(frame, force=True, runtime=True)
        fresh = self._camera_state()
        if result is None or result["status"] != "ready":
            return None, result["center"] if result else None
        if not self._vision(fresh) or fresh["frame_age_ms"] > self.VISION_GRACE_SECONDS * 1000:
            with self._lock:
                self._planning_status, self._planning_message = "waiting", "Esperando una imagen reciente después del cálculo."
                self._planned_path = []
            return None, None
        try:
            moved = self._observe(fresh, calibration, result["center"], required=True)
        except NavigationError:
            return None, None
        if (moved["distance_m"] > .01 or result.get("heading") is not None and
                abs(heading_error_rad(fresh["marker_corners"], calibration, result["heading"])) > math.radians(3)):
            return None, None
        with self._lock:
            return (list(self._steering_target) if self._steering_target else None), result["center"]

    @staticmethod
    def _obstacle_speed_limit(clearance_mm, physical_clearance_mm, *, reserve_mm=15.):
        extra = max(0., clearance_mm - physical_clearance_mm - reserve_mm)
        return max(0., -45. + math.sqrt(45. ** 2 + 2. * 450. * extra))

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
        """Un cambio efectivo de fuente invalida geometría y mapa; no una apertura vacía."""
        with self._lock:
            calibration = self._calibration
            calibration_source = ({"camera_id": calibration.camera_id, "frame_width": calibration.width,
                                   "frame_height": calibration.height} if calibration else None)
            changed_calibration = self._source_changed(frame, calibration_source)
            changed_map = self._source_changed(frame, self._obstacle_source)
            if not changed_calibration and not changed_map:
                return
            self._route_generation += 1
            if changed_calibration:
                self._calibration = None
                self._waypoints = []
                self._waypoint_index = 0
                self._last_arrival_mm = None
                self._target = None
            if changed_map:
                self._obstacles, self._obstacle_source = [], None
                self._obstacle_load_error = "El mapa guardado corresponde a otra cámara o resolución. Bórralo o dibújalo de nuevo."
            self._invalidate_plan_locked()
            self._pending_mission = False
            self._message = "Cambió la cámara o la resolución. Revisa la calibración y el mapa."
            if self._active:
                self._cancel_reason = self._message
                self._cancel.set()
                self._phase = "stopping"
            else:
                self._phase = "uncalibrated" if self._calibration is None else "selected"
        self._save_calibration()
        # Conservar el archivo incompatible hasta que el usuario confirme borrarlo
        # evita que un reinicio convierta el mapa rechazado en espacio libre.

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
            self._invalidate_plan_locked()
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
            self._observe(frame, calibration, target, required=True)
            corners = frame["marker_corners"]
            edge = sum(math.hypot(corners[(i + 1) % 4][0] - corners[i][0],
                                  corners[(i + 1) % 4][1] - corners[i][1]) for i in range(4)) / 4
            margin = 0. if self._safety_margin_mm == 0 else max(20., edge * .6)
            if not (margin <= target[0] < frame_width - margin
                    and margin <= target[1] < frame_height - margin):
                raise NavigationError("El destino debe quedar más alejado del borde de la imagen.", 422)
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
            self._invalidate_plan_locked()
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
            self._invalidate_plan_locked()
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
        self._refresh_plan(frame)
        with self._lock:
            diag_calibration, diag_margin = self._calibration, self._safety_margin_mm
            diag_obstacles, diag_revision = self._obstacles, self._route_generation
        info = self._clearance_diagnostics(frame, diag_calibration, diag_obstacles, diag_margin)
        with self._lock:
            if diag_revision == self._route_generation:
                self._clearance_info = info
            calibration = self._calibration
            target = None if self._target is None else list(self._target)
            observation = self._observe(frame, calibration, target)
            map_visible = (self._obstacle_source is not None and all(
                frame.get(field) == self._obstacle_source.get(field)
                for field in ("camera_id", "frame_width", "frame_height")))
            planning_visible = not self._obstacles or (map_visible and self._vision(frame))
            footprint = []
            if observation and self._uses_oriented_footprint():
                try:
                    footprint = check_pose(frame["marker_corners"], calibration, observation["center_px"],
                        math.radians(observation["heading_deg"]), self._obstacles)["footprint_px"]
                except (ValueError, TypeError, KeyError):
                    pass
            return {"phase": self._phase, "active": self._active,
                    "calibrated": calibration is not None and self._same_source(frame, calibration),
                    "message": self._message, "target_px": target,
                    "waypoints_px": [list(point) for point in self._waypoints],
                    "waypoint_index": self._waypoint_index, "waypoint_count": len(self._waypoints),
                    "route_revision": self._route_generation,
                    "last_arrival_mm": self._last_arrival_mm,
                    "obstacles_px": self._obstacles if map_visible else [],
                    "obstacle_settings": self._obstacle_settings(),
                    "clearance_info": self._clearance_info,
                    "collision_model": "oriented_footprint" if self._uses_oriented_footprint() else "circle",
                    "robot_footprint_px": footprint,
                    "maneuver": (self._pose_steps[self._pose_step_index] if self._pose_steps and
                                 self._pose_step_index < len(self._pose_steps) else None),
                    "planned_path_px": self._planned_path if planning_visible else [],
                    "inflated_obstacles_px": self._inflated_obstacles if planning_visible else [],
                    "planning_status": self._planning_status if planning_visible else "waiting",
                    "planning_message": self._planning_message if planning_visible else "Esperando posición para planificar.",
                    "steering_target_px": self._steering_target if planning_visible else None,
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
            planning_revision = self._route_generation
            if expected_route_revision is not None and expected_route_revision != planning_revision:
                raise NavigationError("La ruta cambió. Actualiza el panel antes de iniciar.")
        planned = self._refresh_plan(frame, force=True)
        fresh = self._camera_state()
        with self._lock:
            self._editable()
            if planning_revision != self._route_generation:
                raise NavigationError("La ruta o el mapa cambió durante el cálculo. Vuelve a iniciar.")
            if expected_route_revision is not None and expected_route_revision != self._route_generation:
                raise NavigationError("La ruta cambió. Actualiza el panel antes de iniciar.")
            calibration = self._calibration
            if calibration is None or not self._waypoints:
                raise NavigationError("Calibra el robot y selecciona un destino antes de iniciar.")
            index = self._waypoint_index if self._waypoint_index < len(self._waypoints) else 0
            target = list(self._waypoints[index])
            self._observe(frame, calibration, target, required=True)
            if self._obstacle_load_error:
                raise NavigationError(self._obstacle_load_error)
            if self._obstacles:
                if planned is None or planned["status"] != "ready":
                    raise NavigationError(self._planning_message or "No existe un camino libre para esta ruta.")
                current = self._observe(fresh, calibration, planned["center"], required=True)
                if (current["distance_m"] > .01 or planned.get("heading") is not None and
                        abs(heading_error_rad(fresh["marker_corners"], calibration, planned["heading"])) > math.radians(3)):
                    raise NavigationError("El robot se movió durante el cálculo. Vuelve a iniciar.")
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
            self._steering_target = (list(self._avoidance_points[0]) if self._obstacles and self._avoidance_points
                                     else list(target))
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
        with self._lock:
            if (left or right) and (self._cancel.is_set() or self._closed):
                return
            self.robot.drive_velocity_navigation(left, right)
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
    def _advance_speed(distance, forward_mm_s):
        base = 50. + (forward_mm_s - 50.) * min(1., max(0., distance) / .15)
        braking = math.sqrt(2. * 450. * max(0., distance * 1000. - 25.))
        return max(50., min(base, braking))

    def _continuation_window(self, mode, commands, distance, *, recent_speed=0.):
        """Presupuesto por consignas, no odometría ni distancia física garantizada."""
        left, right = commands
        if mode in {"settling", "adjusting", "retrying"}:
            return 0.
        wheel_speed = max(abs(left), abs(right), abs(recent_speed))
        window = self.BRIEF_VISION_GAP_SECONDS
        if mode == "advancing" and left > 0 and right > 0 and distance is not None:
            window = self.CONTINUATION_MAX_SECONDS
            braking_distance = wheel_speed * wheel_speed / (2. * 450.)
            remaining = max(0., distance * 1000. - self.ARRIVAL_M * 1000. - 10. - braking_distance)
            if wheel_speed > 0:
                window = min(window, self.CONTINUATION_MAX_TRAVEL_MM / wheel_speed,
                             remaining / wheel_speed)
        angular_deg_s = abs(math.degrees((right - left) / 190.))
        if angular_deg_s > 0:
            window = min(window, self.CONTINUATION_MAX_SWEEP_DEG / angular_deg_s)
        if wheel_speed > 0:
            window = min(window, self.CONTINUATION_MAX_TRAVEL_MM / wheel_speed)
        return max(0., window)

    @staticmethod
    def _wheel_commands(mode, bearing, distance, mode_elapsed, *, forward_mm_s=100, turn_deg_s=32,
                        steering_rate=None):
        """Consignas físicas por rueda en mm/s; el firmware regula con encoders."""
        if mode == "turning":
            angle = abs(math.degrees(bearing))
            braking_rate = math.sqrt(2. * 240. * max(0., angle - 8.))
            target_rate = max(12., min(turn_deg_s, angle * turn_deg_s / 40., braking_rate))
            magnitude = max(20., min(95. * math.radians(turn_deg_s), 95. * math.radians(target_rate)))
            sign = 1 if bearing > 0 else -1
            return _velocity_mm_s(-sign * magnitude), _velocity_mm_s(sign * magnitude)
        speed = Navigation._advance_speed(distance, forward_mm_s)
        angular_rate = (max(-math.radians(8), min(math.radians(8), bearing * .8))
                        if steering_rate is None else steering_rate)
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
        last_valid_distance = None
        gap_window = None
        recent_motion = []
        camera_sequence, camera_capture = None, None
        large_angle_since, large_angle_frames, large_angle_sequence = None, 0, None
        straight = StraightControl()
        pose_follower = PoseFollower()
        pose_finishing = False
        pose_replan_at = 0.
        with self._lock:
            avoiding = bool(self._obstacles)
            if avoiding and self._steering_target is not None:
                target = list(self._steering_target)
        needs_plan = False
        blocked_center = None
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
                age = frame.get("frame_age_ms")
                frame_sequence = frame.get("sequence")
                if (type(frame_sequence) is int and frame_sequence != camera_sequence
                        and isinstance(age, Real) and not isinstance(age, bool)
                        and math.isfinite(age) and age >= 0):
                    camera_sequence, camera_capture = frame_sequence, tick - age / 1000.
                observation = self._observe(frame, calibration, target)
                # No se conduce con la pose anterior cuando falla una detección.
                if observation is not None and frame["frame_age_ms"] > self.VISION_GRACE_SECONDS * 1000:
                    observation = None
                self._update_motion_estimate(frame, observation, tick)
                if observation is None:
                    stable_since, stable_frames, last_sequence = None, 0, None
                    large_angle_since, large_angle_frames, large_angle_sequence = None, 0, None
                else:
                    sequence = frame.get("sequence")
                    if last_valid_capture is None or sequence != last_valid_sequence:
                        last_valid_capture = tick - frame["frame_age_ms"] / 1000
                        last_valid_sequence = sequence
                        last_valid_distance = observation["distance_m"]
                        gap_window = None
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
                        straight.reset()
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
                        straight.reset()
                        # Reconfirmar varios fotogramas antes de restaurar un enlace.
                        stable_since, stable_frames, last_sequence = None, 0, None
                        stable = False
                        next_arm = tick
                        needs_plan = avoiding
                        blocked_center = None

                if session is None:
                    straight.reset()
                    self._set_phase("paused_connection", "Esperando conexión y seguimiento. El destino se conserva.")
                    if (connected and stable and not arming and tick >= next_arm
                            and not remote.get("moving") and not remote.get("navigation_active")):
                        if avoiding and needs_plan:
                            if (not self._uses_oriented_footprint() and blocked_center is not None
                                    and observation is not None and observe_robot(
                                    frame["marker_corners"], calibration, blocked_center)["distance_m"] < .01):
                                self._cancel.wait(self.CONTROL_INTERVAL)
                                continue
                            target, blocked_center = self._runtime_plan(frame, calibration)
                            if target is None:
                                self._cancel.wait(self.CONTROL_INTERVAL)
                                continue
                            needs_plan = False
                            if self._uses_oriented_footprint():
                                pose_follower.reset()
                                pose_finishing = False
                                best_distance = best_angle = math.inf
                        self._launch_rearm(required_velocity)  # HTTP en otro trabajador, sin bloquear el control.
                        next_arm = tick + self.RESUME_RETRY_SECONDS
                    self._cancel.wait(self.CONTROL_INTERVAL)
                    continue

                age_ms = frame.get("frame_age_ms")
                recent_motion = [(stamp, speed) for stamp, speed in recent_motion if tick - stamp <= .25]
                with self._lock:
                    held = (self._last_left, self._last_right)
                hold_seconds = self._continuation_window(mode, held, last_valid_distance,
                    recent_speed=max((speed for _, speed in recent_motion), default=0.))
                if observation is None:
                    gap_window = hold_seconds if gap_window is None else min(gap_window, hold_seconds)
                    hold_seconds = gap_window
                can_hold = (not avoiding and observation is None and not waiting_vision
                            and last_valid_capture is not None
                            and 0 <= tick - last_valid_capture < hold_seconds
                            and camera_capture is not None and 0 <= tick - camera_capture <= self.VISION_GRACE_SECONDS
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
                    remaining_gap = last_valid_capture + hold_seconds - time.monotonic()
                    self._cancel.wait(min(self.CONTROL_INTERVAL, max(0., remaining_gap)))
                    continue

                if observation is None:
                    waiting_vision = True
                    with self._lock:
                        self._vision_hold = False
                        self._vision_gap_ms = 0.

                if observation is None or (waiting_vision and not stable):
                    straight.reset()
                    large_angle_since, large_angle_frames, large_angle_sequence = None, 0, None
                    self._set_phase("paused_vision", "Esperando una imagen estable. El destino se conserva.")
                    if avoiding:
                        needs_plan, blocked_center = True, None
                        with self._lock:
                            self._planning_status = "waiting"
                            self._planning_message = "Esperando seguimiento estable para recalcular."
                            self._planned_path = []
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
                if avoiding and self._uses_oriented_footprint():
                    if needs_plan or not self._pose_steps:
                        self._zero_drive()
                        if tick >= pose_replan_at:
                            next_target, blocked_center = self._runtime_plan(frame, calibration)
                            pose_replan_at = time.monotonic() + .5
                            if next_target is not None:
                                target, needs_plan = next_target, False
                                pose_follower.reset()
                                pose_finishing = False
                                no_progress_seconds = 0.
                                best_distance = best_angle = math.inf
                            else:
                                needs_plan = True
                                self._set_phase("paused_obstacle", self._planning_message)
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    with self._lock:
                        step = self._pose_steps[self._pose_step_index]
                        target = list(step["target_px"])
                        self._steering_target = target
                        obstacles = self._obstacles
                    observation = self._observe(frame, calibration, target)
                    if observation is None:
                        self._zero_drive()
                        waiting_vision, needs_plan = True, True
                        pose_finishing = False
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    if step["kind"] == "drive" and step["direction"] != 1:
                        self._zero_drive()
                        needs_plan, pose_finishing = True, False
                        pose_follower.reset()
                        self._set_phase("planning", "Recalculando el recorrido para avanzar de frente…")
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    current_heading = math.radians(observation["heading_deg"])
                    escaping = step["kind"] == "escape" or step.get("escaping", False)
                    # Validate the remaining oriented motion, including every intermediate
                    # footprint. An escape may only reduce existing contact, never cross it.
                    try:
                        swept = check_swept_motion(frame["marker_corners"], calibration,
                            observation["center_px"], current_heading, target, step["heading_rad"],
                            obstacles, allow_start_overlap=escaping)
                    except (ValueError, KeyError, TypeError):
                        swept = {"clear": False}
                    if not swept["clear"]:
                        self._zero_drive()
                        needs_plan, blocked_center, pose_finishing = True, None, False
                        pose_follower.reset()
                        self._set_phase("planning", "Buscando una maniobra para alejarse del obstáculo…")
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    error = heading_error_rad(frame["marker_corners"], calibration, step["heading_rad"])
                    observation["sequence"] = frame_sequence
                    if step["kind"] == "turn":
                        if abs(error) < best_angle - math.radians(2):
                            best_angle, no_progress_seconds = abs(error), 0.
                    elif observation["distance_m"] < best_distance - .003:
                        best_distance, no_progress_seconds = observation["distance_m"], 0.
                    output = pose_follower.update(step, observation, error, tick, speeds)
                    if output["replan"]:
                        self._zero_drive()
                        needs_plan, blocked_center, pose_finishing = True, None, False
                        pose_follower.reset()
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    pose_finishing = pose_finishing or output["done"]
                    left, right = (0, 0) if pose_finishing else (output["left"], output["right"])
                    self._set_phase(output["phase"], output["message"])
                    # The actual wheel correction traces an arc, not the straight
                    # interpolation used to describe a planned translation.
                    # Check the next ramp output and the command it approaches.
                    with self._lock:
                        ramp_dt = (self.CONTROL_INTERVAL if self._ramp_stamp is None else
                                   max(0., min(.1, tick - self._ramp_stamp)))
                        ramp_values = tuple(self._ramp_values)
                    horizon = max(.12, min(.25, self.CONTROL_INTERVAL + frame["frame_age_ms"] / 1000. + .05))
                    def command_is_clear(command):
                        preview = [self._ramp_value(value, desired, ramp_dt,
                            self.SPEED_RISE_MM_S2, self.SPEED_FALL_MM_S2)
                            for value, desired in zip(ramp_values, command)]
                        try:
                            return all(check_wheel_motion(frame["marker_corners"], calibration,
                                a, b, duration_s=horizon, obstacles_px=obstacles,
                                allow_start_overlap=escaping)["clear"]
                                for a, b in (preview, command))
                        except (ValueError, TypeError, KeyError):
                            return False
                    command_clear = command_is_clear((left, right))
                    if not command_clear and not pose_finishing and step["kind"] != "turn":
                        # A small alignment correction can be free in place even
                        # when the curved advancing correction would clip a corner.
                        # Test it, and slower alternatives, against the actual hull.
                        drive_error = math.atan2(math.sin(observation["bearing_rad"] -
                            (math.pi if step["direction"] < 0 else 0.)),
                            math.cos(observation["bearing_rad"] -
                            (math.pi if step["direction"] < 0 else 0.)))
                        candidates = []
                        if abs(drive_error) > math.radians(.5):
                            sign = 1 if drive_error > 0 else -1
                            candidates.append((-sign * 20, sign * 20))
                        candidates.extend(((left * .5, right * .5), (left * .25, right * .25)))
                        for alternative in candidates:
                            if command_is_clear(alternative):
                                left, right = alternative
                                command_clear = True
                                break
                    if not command_clear:
                        self._zero_drive()
                        needs_plan, pose_finishing = True, False
                        pose_follower.reset()
                        self._set_phase("planning", "Buscando una maniobra con espacio para corregir…")
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    if pose_finishing and self._last_left == self._last_right == 0:
                        # Check the stopped pose as well: braking can move the robot
                        # beyond a short step or carry a turn past its endpoint.
                        if (observation["distance_m"] > .020 or step["kind"] == "turn" and
                                abs(error) > math.radians(6)):
                            self._zero_drive()
                            needs_plan, pose_finishing = True, False
                            pose_follower.reset()
                            self._cancel.wait(self.CONTROL_INTERVAL)
                            continue
                        # Never count a rotation as arrival at a user's destination.
                        with self._lock:
                            next_step = self._pose_step_index + 1
                            if next_step < len(self._pose_steps):
                                self._pose_step_index = next_step
                                target = list(self._pose_steps[next_step]["target_px"])
                                self._steering_target = target
                            else:
                                remaining = observe_robot(frame["marker_corners"], calibration,
                                                          self._waypoints[self._waypoint_index])["distance_m"]
                                if remaining > self.SETTLED_M:
                                    needs_plan = True
                                else:
                                    self._last_arrival_mm = round(remaining * 1000, 1)
                                    self._waypoint_index += 1
                                    self._sync_target_locked()
                                    if self._target is None:
                                        self._pending_mission = False
                                        final_phase, final_message = "arrived", "Ruta completada. Robot detenido."
                                    else:
                                        target = list(self._target)
                                        needs_plan = True
                        self._save_calibration()
                        if self._target is None:
                            break
                        pose_follower.reset()
                        pose_finishing = False
                        no_progress_seconds = 0.
                        best_distance = best_angle = math.inf
                    if no_progress_seconds >= self.NO_PROGRESS_SECONDS:
                        self._zero_drive()
                        needs_plan, blocked_center = True, None
                        pose_finishing = False
                        pose_follower.reset()
                        no_progress_seconds = 0.
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    try:
                        # Apply exactly the ramp sample whose arc was checked;
                        # geometry work must not silently increase its elapsed time.
                        applied = [self._ramp_value(value, desired, ramp_dt,
                            self.SPEED_RISE_MM_S2, self.SPEED_FALL_MM_S2)
                            for value, desired in zip(ramp_values, (left, right))]
                        if step["kind"] == "drive" and sum(applied) < 0:
                            self._zero_drive()
                            needs_plan, pose_finishing = True, False
                            pose_follower.reset()
                            self._cancel.wait(self.CONTROL_INTERVAL)
                            continue
                        with self._lock:
                            self._ramp_values, self._ramp_stamp = applied, time.monotonic()
                        self._drive(*[_velocity_mm_s(value) for value in applied])
                        was_controlling = True
                    except Exception:
                        self._abandon()
                        session, session_seen = None, False
                        waiting_vision, needs_plan = True, True
                        pose_finishing = False
                    self._cancel.wait(max(0., self.CONTROL_INTERVAL - (time.monotonic() - tick)))
                    continue
                if avoiding:
                    with self._lock:
                        obstacles = self._obstacles
                        safety_margin = self._safety_margin_mm
                        physical_clearance = ROBOT_RADIUS_MM + safety_margin
                    view_clearance = self._camera_view_margin(calibration, safety_margin)
                    center = observation["center_px"]
                    if needs_plan:
                        moved = (blocked_center is None or observe_robot(
                            frame["marker_corners"], calibration, blocked_center)["distance_m"] >= .01)
                        if moved:
                            next_target, blocked_center = self._runtime_plan(frame, calibration)
                            if next_target is not None:
                                target = next_target
                                needs_plan = False
                                mode = None
                                straight.reset()
                        if needs_plan:
                            self._set_phase("paused_obstacle", self._planning_message)
                            self._zero_drive()
                            self._cancel.wait(self.CONTROL_INTERVAL)
                            continue
                        frame = self._camera_state()
                        observation = self._observe(frame, calibration, target)
                        if observation is None:
                            waiting_vision = True
                            self._zero_drive()
                            continue
                        center = observation["center_px"]
                    try:
                        clear = (segment_is_clear(frame["marker_corners"], calibration, center, center,
                                                  obstacles, clearance_mm=physical_clearance,
                                                  camera_clearance_mm=view_clearance)
                                 and segment_is_clear(frame["marker_corners"], calibration, center, target,
                                                      obstacles, clearance_mm=physical_clearance,
                                                      camera_clearance_mm=view_clearance))
                        distances = point_clearances_mm(frame["marker_corners"], calibration, center, obstacles)
                    except ValueError:
                        clear, distances = False, {"obstacle_mm": 0., "camera_mm": 0.}
                    if not clear:
                        self._zero_drive()
                        needs_plan, blocked_center = True, None
                        straight.reset()
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    reserve = 0. if safety_margin == 0 else 15.
                    obstacle_speed = self._obstacle_speed_limit(distances["obstacle_mm"], physical_clearance, reserve_mm=reserve)
                    view_speed = self._obstacle_speed_limit(distances["camera_mm"], view_clearance, reserve_mm=reserve)
                    safe_speed = min(obstacle_speed, view_speed)
                    about_to_stop = (observation["distance_m"] <= .015)
                    will_advance = (mode == "advancing" or (mode is None and abs(observation["bearing_rad"]) <= math.radians(3)))
                    minimum_speed = 1. if safety_margin == 0 else 50.
                    if safe_speed < minimum_speed and will_advance and not about_to_stop:
                        self._zero_drive()
                        message = ("El marcador está demasiado cerca del borde visible para avanzar y frenar."
                                   if view_speed < obstacle_speed else
                                   "Falta separación del obstáculo para avanzar y frenar con seguridad.")
                        self._set_phase("paused_obstacle", message)
                        with self._lock:
                            self._planning_status = "blocked"
                            self._planning_message = message
                        needs_plan, blocked_center = True, center
                        self._cancel.wait(self.CONTROL_INTERVAL)
                        continue
                    if safe_speed >= minimum_speed:
                        speeds = {**speeds, "forward_mm_s": min(speeds["forward_mm_s"], math.floor(safe_speed))}
                    with self._lock:
                        self._planned_path = [list(center)] + self._avoidance_points[self._avoidance_index:] + [
                            list(point) for leg in self._planned_legs[1:] for point in leg]
                    is_subpoint = self._avoidance_index + 1 < len(self._avoidance_points)
                else:
                    is_subpoint = False
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
                        settle_duration = (self.WAYPOINT_SETTLE_SECONDS if is_subpoint or
                                           self._waypoint_index + 1 < len(self._waypoints) else self.SETTLE_SECONDS)
                    if settle_started is not None and active_seconds - settle_started >= settle_duration:
                        if distance <= (.025 if is_subpoint else self.SETTLED_M):
                            if is_subpoint:
                                with self._lock:
                                    if self._cancel.is_set():
                                        break
                                    self._avoidance_index += 1
                                    target = list(self._avoidance_points[self._avoidance_index])
                                    self._steering_target = list(target)
                                mode = None
                                straight.reset()
                                large_angle_since, large_angle_frames, large_angle_sequence = None, 0, None
                                retry_until = settle_started = None
                                no_progress_seconds = 0.
                                best_distance = best_angle = math.inf
                                self._cancel.wait(self.CONTROL_INTERVAL)
                                continue
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
                            if avoiding:
                                needs_plan, blocked_center = True, None
                            mode = None
                            straight.reset()
                            large_angle_since, large_angle_frames, large_angle_sequence = None, 0, None
                            retry_until = settle_started = None
                            no_progress_seconds = 0.
                            best_distance = best_angle = math.inf
                            self._cancel.wait(self.CONTROL_INTERVAL)
                            continue
                        mode = None
                elif distance <= (.015 if is_subpoint else self.ARRIVAL_M):
                    mode, mode_started = "settling", active_seconds
                    settle_started = None
                    self._set_phase(mode, "Deteniendo el robot en el destino…")
                    left = right = 0
                else:
                    desired = mode
                    if mode is None:
                        desired = "turning" if angle > math.radians(3 if avoiding else 8) else "advancing"
                    elif mode == "turning" and angle <= math.radians(3 if avoiding else 8):
                        desired = "advancing"
                    elif mode == "advancing":
                        if angle > math.radians(15 if avoiding else 25):
                            if large_angle_since is None:
                                large_angle_since = tick
                            if frame_sequence != large_angle_sequence:
                                large_angle_frames += 1
                                large_angle_sequence = frame_sequence
                            if (tick - large_angle_since >= self.TURN_CONFIRM_SECONDS
                                    and large_angle_frames >= self.TURN_CONFIRM_FRAMES):
                                desired = "turning"
                        else:
                            large_angle_since, large_angle_frames, large_angle_sequence = None, 0, None
                    if desired != mode and mode is not None:
                        straight.reset()
                        large_angle_since, large_angle_frames, large_angle_sequence = None, 0, None
                        mode = "adjusting"
                        self._set_phase(mode, "Frenando suavemente antes de cambiar de movimiento…")
                    elif desired != mode:
                        straight.reset()
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
                        steering = (straight.update(bearing, tick - frame["frame_age_ms"] / 1000.,
                                     frame_sequence, min(speeds["forward_mm_s"], self._advance_speed(distance, speeds["forward_mm_s"])), distance)
                                    if mode == "advancing" else None)
                        left, right = self._wheel_commands(mode, bearing, distance, active_seconds - mode_started,
                                                           **speeds, steering_rate=steering)
                try:
                    self._ramp_drive(left, right)
                    with self._lock:
                        recent_motion.append((tick, max(abs(self._last_left), abs(self._last_right))))
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

    @router.post("/obstacles")
    async def obstacles(request: Request):
        data = await action_body(request, {"polygons_px", "frame_width", "frame_height", "camera_id", "expected_route_revision"})
        return await asyncio.to_thread(perform, navigation.set_obstacles, data["polygons_px"],
                                       data["frame_width"], data["frame_height"], data["camera_id"],
                                       data["expected_route_revision"])

    @router.post("/clear-obstacles")
    async def clear_obstacles(request: Request):
        await action_body(request, set())
        return await asyncio.to_thread(perform, navigation.clear_obstacles)

    @router.post("/obstacle-settings")
    async def obstacle_settings(request: Request):
        data = await action_body(request, {"safety_margin_mm", "expected_route_revision"})
        return await asyncio.to_thread(perform, navigation.set_obstacle_settings,
                                       data["safety_margin_mm"], data["expected_route_revision"])

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
