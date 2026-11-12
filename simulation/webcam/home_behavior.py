"""Reacción a manos: refugio cercano y curiosidad por una presencia lejana.

El detector MediaPipe y la navegación conservan trabajadores separados. Este módulo
consume sus estados recientes: no abre cámaras, conexiones ni modelos y nunca
inicia movimiento al cargar ajustes o marcar Casa. La proximidad se estima en
el plano del marcador, no mediante una medición de profundidad de la mano.
"""
from __future__ import annotations

import json
import math
from numbers import Real
from pathlib import Path
import threading
import time

import cv2
import numpy as np

from .navigation_geometry import Calibration, _homographies, _project, observe_robot
from .robot_footprint import ROBOT_FOOTPRINT_MM, point_under_robot


class HomeBehaviorError(RuntimeError):
    def __init__(self, message: str, status_code: int = 409):
        super().__init__(message)
        self.status_code = status_code


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise HomeBehaviorError(f"{label} debe ser un número finito.", 422)
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise HomeBehaviorError(f"{label} debe ser un número finito.", 422) from None
    if not math.isfinite(result):
        raise HomeBehaviorError(f"{label} debe ser un número finito.", 422)
    return result


def _source(width, height, camera_id):
    if (type(width) is not int or type(height) is not int or not 1 <= width <= 8192
            or not 1 <= height <= 8192 or not isinstance(camera_id, str)
            or not camera_id.strip() or len(camera_id) > 512):
        raise HomeBehaviorError("La cámara de Casa no es válida.", 422)
    return {"frame_width": width, "frame_height": height, "camera_id": camera_id}


def _point(x, y, source):
    point = [_finite(x, "La coordenada horizontal"), _finite(y, "La coordenada vertical")]
    if not (0 <= point[0] < source["frame_width"] and 0 <= point[1] < source["frame_height"]):
        raise HomeBehaviorError("Casa debe quedar dentro de la imagen.", 422)
    return point


def _segment_distance(point, first, last):
    edge = last - first
    length2 = float(edge @ edge)
    ratio = 0. if length2 <= 1e-15 else max(0., min(1., float((point - first) @ edge) / length2))
    return float(np.linalg.norm(point - (first + ratio * edge)))


def hand_clearance_mm(box, corners, calibration):
    """Distancia estimada entre el rectángulo de una mano y la huella del robot."""
    if not isinstance(calibration, Calibration) or not isinstance(box, (list, tuple)) or len(box) != 4:
        raise ValueError("La mano no tiene una caja válida.")
    x1, y1, x2, y2 = (_finite(value, "La caja de la mano") for value in box)
    if not (0 <= x1 < x2 <= calibration.width and 0 <= y1 < y2 <= calibration.height):
        raise ValueError("La mano debe quedar dentro de la imagen.")
    forward, _, reference = _homographies(corners, calibration.marker_size_m,
                                         calibration.width, calibration.height)
    c, s = math.cos(calibration.front_angle_rad), math.sin(calibration.front_angle_rad)
    rotation = np.asarray([[c, s], [-s, c]])
    polygon = np.asarray([rotation @ (_project(forward, point, reference) -
        np.asarray(calibration.center_tag_m)) * 1000.
        for point in ((x1, y1), (x2, y1), (x2, y2), (x1, y2))])
    hand = cv2.convexHull(polygon.astype(np.float32)).reshape(-1, 2)
    body = np.asarray(ROBOT_FOOTPRINT_MM, dtype=np.float32)
    area, _ = cv2.intersectConvexConvex(body, hand)
    if area > 0 or any(cv2.pointPolygonTest(body, tuple(point), False) >= 0 for point in hand):
        return 0.
    if any(cv2.pointPolygonTest(hand, tuple(point), False) >= 0 for point in body):
        return 0.
    return min(_segment_distance(point, first, last)
        for points, polygon in ((body, hand), (hand, body))
        for point in points for first, last in zip(polygon, np.roll(polygon, -1, axis=0)))


class HomeBehavior:
    """Máquina de estados explícita; ``observe`` pertenece al trabajador de reacción."""

    DEFAULT_DISTANCE_MM = 150.
    CONFIRM_FRAMES = 2
    CONFIRM_SECONDS = .15
    MAX_DETECTION_AGE_SECONDS = .35
    MAX_FRAME_AGE_MS = 200.
    RELEASE_MARGIN_MM = 50.
    RELEASE_SECONDS = 1.
    RETRIGGER_SECONDS = 1.
    CURIOSITY_SECONDS = 2.
    CURIOSITY_GAP_SECONDS = 1.5
    CURIOSITY_MIN_DISTANCE_MM = 250.
    CURIOSITY_STAND_OFF_MM = 100.
    HAND_ASSOCIATION_DISTANCE_MM = 160.
    ATTRACTOR_POSITION_TOLERANCE_MM = 120.
    FAULT_REASONS = {"driver_fault", "encoder_stall", "encoder_fault", "encoder_error"}
    MANUAL_REASONS = {"button_stop", "remote", "remote_stop", "manual_stop"}

    def __init__(self, camera, navigation, robot=None, runtime_path=None, clock=time.monotonic):
        self.camera, self.navigation = camera, navigation
        self.robot = robot if robot is not None else getattr(navigation, "robot", None)
        self.clock = clock
        self.runtime_path = Path(runtime_path) if runtime_path is not None else None
        self._lock = threading.RLock()
        self._observe_lock = threading.Lock()
        self._persistence_lock = threading.Lock()
        self._home, self._source = None, None
        self._distance = self.DEFAULT_DISTANCE_MM
        self._enabled = self._returning = self._closed = False
        self._phase, self._message = "disabled", "Marca Casa y activa la reacción a manos."
        self._nearest = None
        self._near = False
        self._generation = self._trigger_count = 0
        self._last_sequence = self._near_since = self._far_since = None
        self._near_frames = 0
        self._latched = False
        self._last_trigger = -math.inf
        self._curious = self._interest_reached = self._interest_latched = False
        self._interest_target = None
        self._interest_box = self._interest_since = self._interest_last_seen = None
        self._interest_position = self._attractor_anchor = None
        self._interest_was_visible = False
        self._interest_absent_since = None
        self._far_hand_seconds = 0.
        self._curiosity_count = 0
        self._stop_signature = None
        self._detector = {"status": "loading", "ready": False, "error": None, "model": None}
        self._load()

    def _assert_open(self):
        if self._closed:
            raise HomeBehaviorError("La reacción a manos está cerrada.")

    def _reset_observations(self, preserve_interest=False):
        self._near_since = self._far_since = None
        self._near_frames = 0
        self._nearest = None
        self._near = False
        if preserve_interest:
            self._pause_interest_confirmation(self.clock())
        else:
            self._last_sequence = None
            self._reset_interest_confirmation()
            self._interest_absent_since = None

    def _reset_interest_confirmation(self):
        self._interest_box = self._interest_since = self._interest_last_seen = None
        self._interest_position = None
        self._interest_was_visible = False
        self._far_hand_seconds = 0.

    @staticmethod
    def _hand_position(hand, box):
        """La palma varía menos que la caja al mover los dedos o cambiar altura."""
        point = hand.get("palm_px")
        if isinstance(point, (list, tuple)) and len(point) == 2:
            try:
                x, y = (_finite(value, "La palma") for value in point)
                if box[0] <= x <= box[2] and box[1] <= y <= box[3]:
                    return [x, y]
            except HomeBehaviorError:
                pass
        return [(box[0] + box[2]) / 2., (box[1] + box[3]) / 2.]

    @staticmethod
    def _position_distance_mm(first, second, corners, calibration):
        forward, _, reference = _homographies(corners, calibration.marker_size_m,
                                             calibration.width, calibration.height)
        return float(np.linalg.norm(_project(forward, first, reference) -
                                    _project(forward, second, reference))) * 1000.

    def _pause_interest_confirmation(self, captured):
        self._interest_was_visible = False
        if (self._interest_last_seen is not None and
                captured - self._interest_last_seen > self.CURIOSITY_GAP_SECONDS + 1e-9):
            self._reset_interest_confirmation()

    def _fear_candidates(self, hands, corners, calibration):
        # Acercarse a la mano elegida no equivale a que esa mano se acerque al
        # robot. Se conserva su posición original, incluso después de llegar.
        # Solo una mano puede recibir esta excepción; otra mano cercana o una
        # mano que invade la huella sigue teniendo prioridad de refugio.
        if not ((self._curious or self._interest_reached) and self._attractor_anchor is not None and hands):
            return hands
        nearest = min(range(len(hands)), key=lambda index: self._position_distance_mm(
            self._attractor_anchor, hands[index][2], corners, calibration))
        distance, _, position = hands[nearest]
        same_position = self._position_distance_mm(self._attractor_anchor, position, corners, calibration)
        if distance > 0 and same_position <= self.ATTRACTOR_POSITION_TOLERANCE_MM:
            return [hand for index, hand in enumerate(hands) if index != nearest]
        return hands

    def _interest_presence(self, hands, captured, corners, calibration):
        # Solo una ausencia confirmada de toda mano libera una reacción ya
        # realizada. Llegar al punto no reinicia otra aproximación al mismo gesto.
        if not hands:
            if self._interest_absent_since is None:
                self._interest_absent_since = captured
            if captured - self._interest_absent_since >= self.RELEASE_SECONDS:
                self._interest_latched = False
        else:
            self._interest_absent_since = None
        if not hands:
            self._pause_interest_confirmation(captured)
        elif (self._interest_last_seen is not None and
                captured - self._interest_last_seen > self.CURIOSITY_GAP_SECONDS + 1e-9):
            self._reset_interest_confirmation()
        if self._curious or self._returning or self._interest_latched:
            return None
        minimum = max(self.CURIOSITY_MIN_DISTANCE_MM, self._distance + 100.)
        distant = [hand for hand in hands if hand[0] >= minimum]
        if not distant:
            if hands:
                self._reset_interest_confirmation()
            return None
        chosen = min(distant, key=lambda item: item[0])
        if self._interest_position is not None:
            matches = [item for item in distant if self._position_distance_mm(
                self._interest_position, item[2], corners, calibration) <= self.HAND_ASSOCIATION_DISTANCE_MM]
            if matches:
                chosen = min(matches, key=lambda item: self._position_distance_mm(
                    self._interest_position, item[2], corners, calibration))
            else:
                self._reset_interest_confirmation()
        if self._interest_since is None:
            self._interest_since = captured
        if self._interest_was_visible and self._interest_last_seen is not None:
            elapsed = captured - self._interest_last_seen
            # Un resultado fresco no demuestra que la mano permaneció visible
            # durante un hueco sin resultados. Esos intervalos no suman espera.
            if 0 <= elapsed <= self.MAX_DETECTION_AGE_SECONDS:
                self._far_hand_seconds += elapsed
        self._interest_box = list(chosen[1])
        self._interest_position = list(chosen[2])
        self._interest_last_seen = captured
        self._interest_was_visible = True
        if self._far_hand_seconds + 1e-9 >= self.CURIOSITY_SECONDS:
            return list(chosen[1])
        return None

    def _calibration(self):
        lock = getattr(self.navigation, "_lock", None)
        if lock is not None:
            with lock:
                return getattr(self.navigation, "_calibration", None)
        return getattr(self.navigation, "_calibration", None)

    def _navigation_state(self):
        # No recalcular la vista geométrica desde el trabajador de manos.
        lock = getattr(self.navigation, "_lock", None)
        if lock is not None and hasattr(self.navigation, "_phase"):
            with lock:
                return {"phase": self.navigation._phase, "active": self.navigation._active,
                        "route_revision": self.navigation._route_generation}
        return self.navigation.state()

    def _robot_state(self):
        return self.robot.state() if self.robot is not None else {}

    @staticmethod
    def _stop_key(remote):
        return tuple(remote.get(key) for key in ("stop_reason", "resume_blocked", "last_id", "boot_id"))

    @staticmethod
    def _matches(frame, source):
        return source is not None and all(frame.get(key) == value for key, value in source.items())

    @staticmethod
    def _changed_source(frame, source):
        return source is not None and any(frame.get(key) not in (None, "", 0) and
            frame.get(key) != value for key, value in source.items())

    def state(self):
        with self._lock:
            return {"enabled": self._enabled, "phase": self._phase, "message": self._message,
                "home_px": list(self._home) if self._home is not None else None,
                "source": dict(self._source) if self._source is not None else None,
                "distance_mm": self._distance, "nearest_hand_mm": self._nearest,
                "near_hand": self._near, "returning": self._returning,
                "curious": self._curious,
                "interest_target_px": list(self._interest_target) if self._interest_target is not None else None,
                "far_hand_seconds": round(self._far_hand_seconds, 2),
                "curiosity_wait_seconds": self.CURIOSITY_SECONDS,
                "curiosity_distance_mm": max(self.CURIOSITY_MIN_DISTANCE_MM, self._distance + 100.),
                "interest_stand_off_mm": self.CURIOSITY_STAND_OFF_MM,
                "curiosity_count": self._curiosity_count,
                "trigger_count": self._trigger_count, "detector": dict(self._detector)}

    def set_home(self, x, y, frame_width, frame_height, camera_id):
        source = _source(frame_width, frame_height, camera_id)
        point = _point(x, y, source)
        frame = self.camera.state()
        calibration = self._calibration()
        if not self._matches(frame, source) or calibration is None:
            raise HomeBehaviorError("Calibra la cámara actual antes de marcar Casa.")
        if (calibration.camera_id != camera_id or calibration.width != frame_width
                or calibration.height != frame_height):
            raise HomeBehaviorError("Casa y la calibración deben usar la misma cámara.")
        if self._navigation_state().get("active"):
            raise HomeBehaviorError("Detén el recorrido antes de mover Casa.")
        with self._lock:
            self._assert_open()
            self._generation += 1
            self._home, self._source = point, source
            self._enabled = self._returning = self._latched = False
            self._curious = self._interest_reached = self._interest_latched = False
            self._interest_target = None
            self._attractor_anchor = None
            self._phase, self._message = "disabled", "Casa guardada. Activa la reacción para vigilar las manos."
            self._reset_observations()
        self._save()
        return self.state()

    def clear_home(self):
        self.set_enabled(False)
        with self._lock:
            self._home = self._source = None
            self._message = "Casa borrada."
        self._save()
        return self.state()

    def set_distance(self, distance_mm):
        distance = _finite(distance_mm, "La distancia de reacción")
        if not 20 <= distance <= 1000:
            raise HomeBehaviorError("La distancia de reacción debe estar entre 20 y 1000 mm.", 422)
        with self._lock:
            self._assert_open()
            self._distance = distance
            self._reset_observations()
        self._save()
        return self.state()

    def set_enabled(self, enabled):
        if type(enabled) is not bool:
            raise HomeBehaviorError("La activación debe ser verdadera o falsa.", 422)
        remote = self._robot_state() if enabled else {}
        if enabled:
            frame, calibration = self.camera.state(), self._calibration()
            with self._lock:
                if self._home is None:
                    raise HomeBehaviorError("Marca el punto Casa antes de activar la reacción.")
                if calibration is None or not self._matches(frame, self._source):
                    raise HomeBehaviorError("Casa necesita la misma cámara y su calibración.")
            if remote.get("stop_reason") in self.FAULT_REASONS | {"button_stop"}:
                raise HomeBehaviorError("El robot conserva una parada o un fallo. Revísalo antes de activar la reacción.")
        with self._lock:
            self._assert_open()
            if enabled and self._enabled:
                return self.state()
            stop_own_return = (self._returning or self._curious) and not enabled
            self._generation += 1
            self._enabled = enabled
            self._returning = self._latched = False
            self._curious = self._interest_reached = self._interest_latched = False
            self._interest_target = None
            self._attractor_anchor = None
            self._stop_signature = self._stop_key(remote) if enabled else None
            self._reset_observations()
            self._phase = "watching" if enabled else "disabled"
            self._message = "Vigilando las manos cerca del robot." if enabled else "Reacción a manos desactivada."
        if stop_own_return:
            self.navigation.stop()
        return self.state()

    def cancel(self, reason="Parada solicitada."):
        """Invalida autorizaciones pendientes sin IO; apto para una parada local."""
        with self._lock:
            self._generation += 1
            self._enabled = self._returning = False
            self._curious = self._interest_reached = self._interest_latched = False
            self._interest_target = None
            self._attractor_anchor = None
            self._phase, self._message = "disabled", str(reason)
            self._reset_observations()

    def close(self):
        self.cancel("Reacción a manos cerrada.")
        with self._lock:
            self._closed = True
            self._phase = "closed"

    def _at_home(self, frame, calibration, home):
        try:
            return point_under_robot(observe_robot(frame["marker_corners"], calibration, home)["target_local_m"])
        except (ValueError, TypeError, KeyError):
            return False

    def observe(self, hand_state, frame=None):
        """Consume un resultado reciente; nunca reutiliza una detección vieja."""
        if not self._observe_lock.acquire(blocking=False):
            return self.state()
        try:
            return self._observe(hand_state, self.camera.state() if frame is None else frame)
        finally:
            self._observe_lock.release()

    def _observe(self, hand_state, frame):
        now = self.clock()
        remote = self._robot_state()
        navigation = self._navigation_state()
        # Leer Navigation antes de tomar el candado de reacción: el armado de
        # Navigation consulta authorize(), que toma este último candado.
        calibration = self._calibration()
        with self._lock:
            if self._closed:
                return self.state()
            if isinstance(hand_state, dict):
                self._detector = {key: hand_state.get(key) for key in ("status", "ready", "error", "model")}
            if not self._enabled:
                return self.state()
            key = self._stop_key(remote)
            reason = remote.get("stop_reason")
            if reason in self.FAULT_REASONS | {"button_stop"} or (
                    reason in self.MANUAL_REASONS and key != self._stop_signature):
                self.cancel("Parada manual o fallo del robot. Activa la reacción de nuevo para continuar.")
                return self.state()
            self._stop_signature = key
            if self._returning:
                if navigation.get("phase") == "arrived":
                    self._returning = False
                    self._phase, self._message = "at_home", "En Casa."
                elif navigation.get("phase") in {"error", "stopped", "stopping"}:
                    self.cancel("Regreso cancelado. Activa la reacción de nuevo para continuar.")
                    return self.state()
                else:
                    self._phase, self._message = "returning", "Me asusté. Volviendo a Casa…"
                    return self.state()
            if self._changed_source(frame, self._source):
                self.cancel("La cámara cambió. Marca Casa de nuevo en esta vista.")
                return self.state()
            if self._curious:
                if navigation.get("phase") == "arrived":
                    self._curious = False
                    self._interest_reached = True
                    self._phase, self._message = "interest_reached", "Llegué al punto de interés."
                elif navigation.get("phase") in {"error", "stopped", "stopping"}:
                    self.cancel("Acercamiento cancelado. Activa la reacción de nuevo para continuar.")
                    return self.state()
            elif self._interest_reached and navigation.get("active"):
                # Un recorrido normal iniciado por el usuario vuelve a ser la
                # misión actual; no conservar una etiqueta de llegada anterior.
                self._interest_reached = False
                self._interest_target = None
                self._attractor_anchor = None
            age = frame.get("frame_age_ms")
            if (not self._matches(frame, self._source) or calibration is None
                    or frame.get("visible") is not True or frame.get("running") is False
                    or frame.get("opening") is True or not isinstance(age, Real)
                    or isinstance(age, bool) or not math.isfinite(age) or not 0 <= age <= self.MAX_FRAME_AGE_MS):
                self._phase, self._message = (("curious", "Voy al punto de interés que elegí.") if self._curious else
                    ("waiting_vision", "Esperando ver el centro del robot. Casa sigue guardada."))
                self._reset_observations(preserve_interest=True)
                return self.state()
            if self.robot is not None and (remote.get("connected") is not True or remote.get("ready") is not True):
                self._phase, self._message = (("curious", "Voy al punto de interés que elegí.") if self._curious else
                    ("waiting_connection", "Esperando conexión con el robot."))
                self._reset_observations(preserve_interest=True)
                return self.state()
            captured = hand_state.get("captured_at") if isinstance(hand_state, dict) else None
            sequence = hand_state.get("sequence") if isinstance(hand_state, dict) else None
            if (not isinstance(hand_state, dict) or hand_state.get("ready") is not True
                    or not isinstance(captured, Real) or isinstance(captured, bool)
                    or not math.isfinite(captured) or not 0 <= now - captured <= self.MAX_DETECTION_AGE_SECONDS
                    or type(sequence) is not int or sequence < 0
                    or any(hand_state.get(key) != frame.get(key) for key in ("frame_width", "frame_height"))
                    or hand_state.get("camera_id", frame.get("camera_id")) != frame.get("camera_id")):
                self._phase, self._message = (("curious", "Voy al punto de interés que elegí.") if self._curious else
                    ("waiting_detector", "Esperando una detección reciente de manos."))
                self._reset_observations(preserve_interest=True)
                return self.state()
            if sequence == self._last_sequence:
                return self.state()
            self._last_sequence = sequence
            detected = []
            hands = hand_state.get("hands", [])
            for hand in hands if isinstance(hands, list) else []:
                if not isinstance(hand, dict):
                    continue
                try:
                    confidence = _finite(hand.get("confidence"), "La confianza")
                    if confidence >= .5:
                        box = hand.get("bbox_px")
                        detected.append((hand_clearance_mm(box, frame["marker_corners"], calibration),
                                         list(box), self._hand_position(hand, box)))
                except (ValueError, TypeError, KeyError, HomeBehaviorError, cv2.error):
                    continue
            self._nearest = round(min(hand[0] for hand in detected), 1) if detected else None
            threats = self._fear_candidates(detected, frame["marker_corners"], calibration)
            nearest_threat = min((hand[0] for hand in threats), default=math.inf)
            self._near = nearest_threat <= self._distance
            at_home = self._at_home(frame, calibration, self._home)
            if not self._near:
                self._near_since, self._near_frames = None, 0
                if nearest_threat >= self._distance + self.RELEASE_MARGIN_MM:
                    self._far_since = now if self._far_since is None else self._far_since
                    if now - self._far_since >= self.RELEASE_SECONDS:
                        self._latched = False
                else:
                    self._far_since = None
                interest = self._interest_presence(detected, captured, frame["marker_corners"], calibration)
                if self._curious:
                    self._phase, self._message = "curious", "Voy al punto de interés que elegí."
                    return self.state()
                if interest is None:
                    if self._interest_since is not None and not self._interest_latched:
                        self._phase, self._message = "observing_hand", "Observo una mano lejana…"
                    elif self._interest_reached:
                        self._phase, self._message = "interest_reached", "Llegué al punto de interés."
                    else:
                        self._phase = "at_home" if at_home else "watching"
                        self._message = "En Casa." if at_home else "Vigilando las manos cerca del robot."
                    return self.state()
                action = "curiosity"
                self._curious = self._interest_latched = True
                self._interest_reached = False
                self._interest_target = None
                self._attractor_anchor = list(self._interest_position)
                self._phase, self._message = "curious", "Me da curiosidad. Voy a acercarme."
                corners = [list(point) for point in frame["marker_corners"]]
                stand_off = self.CURIOSITY_STAND_OFF_MM
            else:
                self._reset_interest_confirmation()
                self._interest_absent_since = None
                self._far_since = None
                self._near_since = captured if self._near_since is None else self._near_since
                self._near_frames += 1
                if at_home and not self._curious:
                    self._phase, self._message = "at_home", "Estoy en Casa."
                    self._latched = True
                    return self.state()
                if (self._near_frames < self.CONFIRM_FRAMES or captured - self._near_since < self.CONFIRM_SECONDS
                        or now - self._last_trigger < self.RETRIGGER_SECONDS):
                    if not self._curious:
                        self._phase, self._message = "near_hand", "Hay una mano cerca del robot."
                    return self.state()
                # El miedo tiene prioridad sobre la aproximación por curiosidad.
                if self._latched and self._phase != "at_home" and not self._curious:
                    return self.state()
                action = "home"
                self._curious = self._interest_reached = False
                self._interest_target = None
                self._attractor_anchor = None
                self._returning = self._latched = True
                self._last_trigger = now
                self._phase, self._message = "returning", "Me asusté. Volviendo a Casa…"
            self._generation += 1
            generation = self._generation
            home, source = list(self._home), dict(self._source)

        def authorized():
            with self._lock:
                return self._enabled and not self._closed and self._generation == generation

        try:
            result = (self.navigation.return_home(home, source, authorize=authorized) if action == "home" else
                self.navigation.approach_hand(interest, corners, source, stand_off_mm=stand_off,
                                              authorize=authorized))
        except Exception as exc:
            with self._lock:
                if generation == self._generation and self._enabled:
                    if action == "home":
                        self._enabled = self._returning = False
                        self._phase = "error"
                        self._message = str(exc) if isinstance(exc, RuntimeError) else "No se pudo calcular el regreso a Casa."
                    else:
                        # Una mano puede estar junto a un borde o dentro del
                        # obstáculo dibujado. No perder por ello la reacción de
                        # refugio ni recalcular el mismo episodio cada imagen.
                        self._curious = False
                        self._attractor_anchor = None
                        self._phase = "watching"
                        self._message = "No encontré espacio para acercarme. Sigo atento a las manos."
        else:
            with self._lock:
                if generation == self._generation and self._enabled:
                    if action == "home":
                        self._trigger_count += 1
                    else:
                        target = result.get("target_px") if isinstance(result, dict) else None
                        self._interest_target = list(target) if isinstance(target, (list, tuple)) and len(target) == 2 else None
                        self._curiosity_count += 1
        return self.state()

    def _save(self):
        if self.runtime_path is None:
            return
        with self._persistence_lock:
            with self._lock:
                data = {"version": 1, "home_px": self._home, "source": self._source,
                        "distance_mm": self._distance}
            self.runtime_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.runtime_path.with_name(self.runtime_path.name + ".tmp")
            temporary.write_text(json.dumps(data, allow_nan=False), encoding="utf-8")
            temporary.chmod(0o600)
            temporary.replace(self.runtime_path)

    def _load(self):
        if self.runtime_path is None or not self.runtime_path.exists():
            return
        try:
            if self.runtime_path.stat().st_size > 16384:
                raise ValueError()
            data = json.loads(self.runtime_path.read_text())
            if (not isinstance(data, dict) or set(data) != {"version", "home_px", "source", "distance_mm"}
                    or type(data["version"]) is not int or data["version"] != 1):
                raise ValueError()
            distance = _finite(data["distance_mm"], "La distancia")
            if not 20 <= distance <= 1000:
                raise ValueError()
            if data["home_px"] is None:
                if data["source"] is not None:
                    raise ValueError()
            else:
                source = data["source"]
                if not isinstance(source, dict) or set(source) != {"frame_width", "frame_height", "camera_id"}:
                    raise ValueError()
                source = _source(source["frame_width"], source["frame_height"], source["camera_id"])
                point = data["home_px"]
                if not isinstance(point, list) or len(point) != 2:
                    raise ValueError()
                self._home, self._source = _point(*point, source), source
            self._distance = distance
            self._message = "Casa guardada. Activa la reacción para vigilar las manos."
        except (OSError, ValueError, TypeError, KeyError, HomeBehaviorError):
            self._home = self._source = None
            self._phase, self._message = "error", "No se pudo leer Casa. Vuelve a marcarla."
