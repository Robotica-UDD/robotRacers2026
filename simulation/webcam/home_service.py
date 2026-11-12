"""Orquestación local de MediaPipe y regreso a Casa; sin inferencia en la captura."""
from __future__ import annotations

import asyncio
import json
import threading

from fastapi import APIRouter, HTTPException, Request

from .hand_detector import AsyncHandDetector
from .home_behavior import HomeBehavior, HomeBehaviorError
from .navigation import NavigationError


class HomeService:
    def __init__(self, camera, navigation, robot, *, runtime_path=None,
                 detector=None, load_detector=True):
        self.camera, self.navigation = camera, navigation
        self.behavior = HomeBehavior(camera, navigation, robot, runtime_path=runtime_path)
        self.detector, self.load_detector = detector, load_detector
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._worker = None
        navigation._home_cancel_callback = self.behavior.cancel

    def start(self):
        if self.detector is None and self.load_detector:
            self.detector = AsyncHandDetector()
        if self.detector is not None:
            self.camera.hand_detector = self.detector
            self._worker = threading.Thread(target=self._run, daemon=True,
                                             name="webcam-home-reaction")
            self._worker.start()

    def _run(self):
        while not self._stop.wait(.05):
            if not self._lock.acquire(blocking=False):
                continue
            try:
                if self.behavior.state()["enabled"]:
                    hands, frame = self.camera.hand_observation()
                    self.behavior.observe(hands, frame)
                if not self.behavior.state()["enabled"]:
                    self.navigation.disarm_home_watch()
            except Exception:
                # Un fallo del observador no debe dejar una autorización viva.
                self.behavior.cancel("La reacción a manos se detuvo. Actívala de nuevo para reintentar.")
                self.navigation.disarm_home_watch()
            finally:
                self._lock.release()

    def state(self):
        result = self.behavior.state()
        result["detector"] = (self.detector.state() if self.detector is not None else
            {"status": "missing_model", "ready": False, "error": "El detector de manos no está iniciado."})
        return result

    def settings(self, *, enabled=None, distance_mm=None):
        if enabled is not None and type(enabled) is not bool:
            raise HomeBehaviorError("La activación debe ser verdadera o falsa.", 422)
        if enabled is False:
            # No esperar al candado del observador: puede estar calculando A*.
            # Invalidar primero su autorización impide un arranque tardío.
            self.behavior.set_enabled(False)
            self.navigation.disarm_home_watch()
            if distance_mm is not None:
                self.behavior.set_distance(distance_mm)
            return self.state()
        with self._lock:
            if distance_mm is not None:
                self.behavior.set_distance(distance_mm)
            if enabled is not None:
                if enabled:
                    if self.detector is None or self.detector.state().get("ready") is not True:
                        raise HomeBehaviorError("Espera a que el detector MediaPipe esté listo.")
                    self.behavior.set_enabled(True)
                    try:
                        self.navigation.arm_home_watch(authorize=lambda: self.behavior.state()["enabled"])
                    except Exception:
                        self.behavior.cancel("No se pudo activar la vigilancia. Comprueba el robot.")
                        raise
                else:
                    self.behavior.set_enabled(False)
                    self.navigation.disarm_home_watch()
            return self.state()

    def point(self, **values):
        with self._lock:
            self.behavior.set_home(values["x"], values["y"], values["frame_width"],
                                   values["frame_height"], values["camera_id"])
            return self.state()

    def clear(self):
        self.behavior.clear_home()
        self.navigation.disarm_home_watch()
        return self.state()

    def close(self):
        self._stop.set()
        self.behavior.close()
        self.navigation._home_cancel_callback = None
        self.navigation.disarm_home_watch()
        if self._worker is not None:
            self._worker.join(timeout=2.)
        self.camera.hand_detector = None
        if self.detector is not None:
            self.detector.close()


def make_home_router(service):
    router = APIRouter(prefix="/api/home")

    async def body(request, shapes):
        if request.headers.get("X-Requested-With") != "XMLHttpRequest":
            raise HTTPException(403, "La solicitud debe realizarse desde el panel local.")
        data = bytearray()
        async for chunk in request.stream():
            data.extend(chunk)
            if len(data) > 4096:
                raise HTTPException(413, "La solicitud supera el tamaño permitido.")
        try:
            def reject_constant(_):
                raise ValueError()
            value = json.loads(data, parse_constant=reject_constant)
        except (ValueError, UnicodeError):
            raise HTTPException(422, "La solicitud no es válida.") from None
        if not isinstance(value, dict) or set(value) not in shapes:
            raise HTTPException(422, "La solicitud contiene campos incorrectos.")
        return value

    def perform(action, **values):
        try:
            return action(**values)
        except (HomeBehaviorError, NavigationError) as exc:
            raise HTTPException(exc.status_code, str(exc)) from None
        except RuntimeError as exc:
            raise HTTPException(getattr(exc, "status_code", 409), str(exc)) from None

    @router.get("/state")
    def state():
        return service.state()

    @router.post("/point")
    async def point(request: Request):
        data = await body(request, [{"x", "y", "frame_width", "frame_height", "camera_id"}])
        return await asyncio.to_thread(perform, service.point, **data)

    @router.post("/settings")
    async def settings(request: Request):
        data = await body(request, [{"enabled"}, {"distance_mm"}, {"enabled", "distance_mm"}])
        if "enabled" in data and type(data["enabled"]) is not bool:
            raise HTTPException(422, "La activación debe ser verdadera o falsa.")
        return await asyncio.to_thread(perform, service.settings, **data)

    @router.post("/clear")
    async def clear(request: Request):
        await body(request, [set()])
        return await asyncio.to_thread(perform, service.clear)

    return router
