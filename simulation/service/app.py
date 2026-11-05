"""API local del producto y distribución de su interfaz."""
from __future__ import annotations
import asyncio
from contextlib import asynccontextmanager
import os
import math
import time
import json
from pathlib import Path
from fastapi import FastAPI,HTTPException,WebSocket,WebSocketDisconnect,Body
from fastapi.responses import FileResponse,Response,HTMLResponse
from .config import SIMULATION_DIR
from .runtime import RobotService


def create_app(service:RobotService|None=None,background:bool=True) -> FastAPI:
    service=service or RobotService()
    @asynccontextmanager
    async def lifespan(app):
        if background:await service.start()
        try:yield
        finally:
            if background:await service.close()
    app=FastAPI(title="RobotRacers",version="0.1.0",lifespan=lifespan)
    app.state.service=service
    app.state.client_metrics={}

    @app.middleware('http')
    async def local_resources_only(request,call_next):
        response=await call_next(request)
        # All running product resources are local, including W3D and textures.
        # wasm/eval is required by the pinned upstream Wren renderer.
        response.headers['Content-Security-Policy']=(
            "default-src 'self'; script-src 'self' 'unsafe-inline' 'unsafe-eval' 'wasm-unsafe-eval' blob:; "
            "style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; font-src 'self' data:; "
            "connect-src 'self' ws://127.0.0.1:8000 ws://127.0.0.1:1234 http://127.0.0.1:1234; worker-src 'self' blob:; "
            "object-src 'none'; base-uri 'self'; frame-ancestors 'self'")
        return response

    def perform(action):
        try:action()
        except (ValueError,KeyError,TypeError) as exc:raise HTTPException(409 if "Detén" in str(exc) or "simulador" in str(exc) or "Espera" in str(exc) or "escena" in str(exc) else 400,str(exc))
        return service.state()

    @app.get("/api/config")
    async def configuration():return service.configuration()
    @app.get("/api/state")
    async def state():return service.state()
    @app.get("/api/health")
    async def health():
        return {"service":"ready","connected":service.connected(),"camera":service.reader.path.exists(),
            "runtime_directory":str(service.directory),"scene_pending":service.scene_pending,
            "resume_warning":service.resume_warning,
            "vision_errors":service.vision_errors,"invalid_packets":service.invalid_packets,
            "client_metrics":app.state.client_metrics}
    @app.post("/api/client-metrics",include_in_schema=False)
    async def client_metrics(body:dict=Body(...)):
        fps=body.get("render_fps",0)
        frames=body.get("total_frames",0)
        if not isinstance(fps,(int,float)) or not math.isfinite(fps) or not 0<=fps<=240 or not isinstance(frames,int) or frames<0:
            raise HTTPException(400,"Métricas no válidas.")
        app.state.client_metrics={"render_fps":float(fps),"total_frames":frames,
            "visible":bool(body.get("visible",False)),"received_stamp":time.monotonic()}
        return {"received":True}
    @app.post("/api/mission/start")
    async def start():return perform(service.mission_start)
    @app.post("/api/mission/pause")
    async def pause():return perform(service.mission_pause)
    @app.post("/api/mission/stop")
    async def stop():return perform(service.mission_stop)
    @app.post("/api/mission/reset")
    async def reset():return perform(service.mission_reset)
    @app.post("/api/settings")
    async def settings(body:dict=Body(...)):return perform(lambda:service.change_settings(body))
    @app.put("/api/goals")
    async def goals(body:dict=Body(...)):
        if "goals" not in body or not isinstance(body["goals"],list):raise HTTPException(400,"Se requiere una lista de objetivos.")
        return perform(lambda:service.change_goals(body["goals"]))
    @app.get("/api/camera.jpg")
    async def camera():
        packet=await asyncio.to_thread(service.camera_packet)
        if packet is None:raise HTTPException(503,"La cámara todavía no tiene imágenes.")
        image,tracking=packet
        return Response(image,media_type="image/jpeg",headers={"Cache-Control":"no-store",
            "X-RobotRacers-Tracking":json.dumps(tracking,ensure_ascii=True,allow_nan=False,separators=(",",":"))})
    @app.websocket("/api/ws")
    async def websocket(socket:WebSocket):
        await socket.accept()
        try:
            while True:
                await socket.send_json(service.state())
                await asyncio.sleep(.05)
        except (WebSocketDisconnect,RuntimeError):pass

    @app.get("/simulation-assets/{filename:path}",include_in_schema=False)
    async def simulation_asset(filename:str):
        base=(SIMULATION_DIR/"assets").resolve()
        try:
            target=(base/filename).resolve()
        except (OSError,ValueError):
            raise HTTPException(404,"Recurso no disponible.")
        allowed_extensions={".png",".jpg",".jpeg",".webp",".hdr"}
        if not target.is_relative_to(base) or not target.is_file() or target.suffix.lower() not in allowed_extensions:
            raise HTTPException(404,"Recurso no disponible.")
        return FileResponse(target,headers={"Cache-Control":"no-cache"})

    @app.get("/webots/{filename:path}",include_in_schema=False)
    async def webots_resource(filename:str):
        home=Path(os.getenv("WEBOTS_HOME","/Applications/Webots.app/Contents"))
        if filename.startswith("wwi/") and (service.directory/"wwi").is_dir():
            base=service.directory/"wwi";relative=filename.removeprefix("wwi/")
        else:base=home/"resources"/"web";relative=filename
        target=(base/relative).resolve()
        if not target.is_relative_to(base.resolve()) or not target.is_file():raise HTTPException(404,"Recurso no disponible.")
        return FileResponse(target)

    @app.get("/{filename:path}",include_in_schema=False)
    async def interface(filename:str):
        if filename.startswith(("api/","webots/")):raise HTTPException(404,"Ruta no disponible.")
        base=Path(os.getenv("ROBOTRACERS_UI_DIR",SIMULATION_DIR/"ui"/"dist")).resolve()
        target=(base/filename).resolve()
        if not target.is_relative_to(base):raise HTTPException(404)
        if target.is_file():return FileResponse(target)
        if not filename or "." not in Path(filename).name:
            if (base/"index.html").is_file():return FileResponse(base/"index.html")
        if not filename:
            return HTMLResponse('<!doctype html><html lang="es"><meta charset="utf-8"><title>RobotRacers</title><body><h1>RobotRacers</h1><p>La aplicación no está disponible.</p></body></html>',status_code=503)
        raise HTTPException(404,"Archivo no disponible.")
    return app

app=create_app()
