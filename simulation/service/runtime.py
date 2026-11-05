"""Coordinación de cámaras, telemetría y consignas; nunca integra pose verdadera."""
from __future__ import annotations
import asyncio
from collections import deque
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import cv2
from .config import load_configuration,runtime_directory
from .estimator import PoseEstimator,wrap
from .geometry import Arena,Point
from .navigation import Navigation,Command
from .metrics import AlignedEvaluation
from .vision import CameraReader,CameraVision

class DatagramReceiver(asyncio.DatagramProtocol):
    def __init__(self,service):self.service=service
    def datagram_received(self,data,addr):
        try:
            if len(data)>32768:return
            message=json.loads(data)
            if isinstance(message,dict):self.service.receive(message,time.monotonic())
        except (ValueError,TypeError,KeyError,OverflowError):self.service.invalid_packets+=1

def finite(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError("Valor numérico no válido.")
    return float(value)

class RobotService:
    def __init__(self,runtime:Path|None=None):
        self.robot_config,self.arena_config=load_configuration()
        self.directory=runtime or runtime_directory()
        r,a=self.robot_config,self.arena_config
        self.settings={"speed":a["speed"],"scale":a["scale"],"mass_kg":r["mass_kg"],
            "friction":r["friction"],"skid_friction":r["skid_friction"],"fan_force_n":r["fan_force_n"]}
        self.arena=Arena(a,self.settings["scale"])
        self.goals=[{"id":key,**self.arena.nodes[key].json()} for key in a["mission"]]
        self.estimator=PoseEstimator(r);self.navigation=Navigation(r["track_width_m"])
        self.reader=CameraReader(self.directory/"camera.frame")
        self.vision=CameraVision(self.directory/"camera_calibration.json")
        self.transport=None;self.tasks=[];self.status="disconnected";self.message="Esperando simulador."
        self.telemetry={};self.telemetry_time=None;self.telemetry_seq=None;self.sim_time=0.
        self.evaluation={};self.evaluation_time=None;self.evaluation_metrics=AlignedEvaluation()
        self.last_vision=None;self.last_frame=None;self.frame_times=deque(maxlen=100)
        self.command_seq=0;self.sent_command=Command();self.invalid_packets=0
        self.vision_errors=0;self.accepted_frames=0;self.accepted_vision=0
        self.vision_timeout=max(.10,min(1.,float(os.getenv("ROBOTRACERS_VISION_GRACE_S",".50"))))
        self.scene_seq=0
        try:self.scene_seq=max(0,int(json.loads((self.directory/"scene_command.json").read_text()).get("seq",0)))
        except (OSError,ValueError,TypeError,json.JSONDecodeError):pass
        self.scene_pending=None;self.scene_requested=None;self.min_camera_stamp=0.
        self.jpeg=None;self.jpeg_stamp=0.;self.jpeg_lock=threading.Lock();self.last_loop_stamp=None;self.loop_duration_ms=0.
        self.telemetry_latency_ms=None
        self.command_address=(os.getenv("ROBOTRACERS_DRIVER_HOST","127.0.0.1"),int(os.getenv("ROBOTRACERS_COMMAND_PORT","9202")))

    async def start(self):
        self.directory.mkdir(parents=True,exist_ok=True)
        loop=asyncio.get_running_loop()
        self.transport,_=await loop.create_datagram_endpoint(lambda:DatagramReceiver(self),
            local_addr=("127.0.0.1",int(os.getenv("ROBOTRACERS_TELEMETRY_PORT","9201"))))
        self.tasks=[asyncio.create_task(self.control_loop()),asyncio.create_task(self.camera_loop())]

    async def close(self):
        self.send(Command(mode="stop"))
        for task in self.tasks:task.cancel()
        await asyncio.gather(*self.tasks,return_exceptions=True)
        self.reader.close()
        if self.transport:self.transport.close();self.transport=None

    def receive(self,message:dict,now:float):
        kind=message.get("type")
        if kind=="telemetry":
            seq=message["seq"]
            if not isinstance(seq,int):raise ValueError("Secuencia incorrecta")
            stamp=finite(message["stamp"]);sim_time=finite(message["sim_time"])
            if now-stamp>.25 or stamp>now+.1 or sim_time<0:return
            if self.telemetry_seq is not None and seq<=self.telemetry_seq:
                if self.telemetry_time is not None and now-self.telemetry_time<1:return
                self.estimator.reset();self.evaluation_metrics.clear();self.status="idle"
            for key in ("left_count","right_count"):
                if not isinstance(message[key],int):raise ValueError("Cuenta incorrecta")
            for key in ("left_velocity","right_velocity","battery_voltage"):finite(message[key])
            for key,maximum in (("reflectance",2),("line_reflectance",16)):
                values=message.get(key,[])
                if not isinstance(values,list) or len(values)>maximum:raise ValueError("Sensores incorrectos")
                for value in values:
                    if not 0<=finite(value)<=1:raise ValueError("Reflectancia fuera de rango")
            self.telemetry=message.copy();self.telemetry_time=now;self.telemetry_seq=seq;self.sim_time=sim_time
            self.telemetry_latency_ms=max(0.,(now-stamp)*1000)
            self.estimator.predict(message["left_count"],message["right_count"],stamp)
            self.record_estimate(now)
            if self.status=="disconnected":self.status="idle";self.message="Listo."
        elif kind=="evaluation":
            # Frontera explícita: estas coordenadas jamás llegan al estimador o al planificador.
            self.evaluation={key:finite(message[key]) for key in ("x","y","yaw","sim_time")}
            self.evaluation["contacts"]=max(0,int(message.get("contacts",0)));self.evaluation_time=now
            self.evaluation_metrics.evaluate(self.evaluation["sim_time"],self.evaluation["x"],self.evaluation["y"],self.evaluation["yaw"],now)
        elif kind=="scene_ack" and self.scene_pending is not None and message.get("seq")==self.scene_pending:
            self.scene_pending=None;self.scene_requested=None
            self.estimator.reset();self.evaluation_metrics.clear();self.last_vision=None;self.min_camera_stamp=now
            self.telemetry_seq=None;self.navigation.clear()
            self.status="idle" if self.connected(now) else "disconnected"
            self.message="Escena actualizada; esperando localización."

    def record_estimate(self,received):
        if self.estimator.initialized:
            self.evaluation_metrics.estimate(self.sim_time,*self.estimator.x,received)

    def connected(self,now:float|None=None) -> bool:
        now=time.monotonic() if now is None else now
        return self.telemetry_time is not None and now-self.telemetry_time<.30 and now-self.telemetry.get("stamp",0.)<.30

    def visual_ready(self,now:float) -> bool:
        stamp=self.estimator.last_vision_stamp
        return self.estimator.initialized and stamp is not None and now-stamp<self.vision_timeout

    def stationary(self) -> bool:
        return self.status!="running" and max(abs(self.telemetry.get("left_velocity",0.)),abs(self.telemetry.get("right_velocity",0.)))<.025

    def send(self,command:Command):
        self.command_seq+=1;self.sent_command=command
        payload={"seq":self.command_seq,"stamp":time.monotonic(),"v":command.v,"w":command.w,"mode":command.mode}
        if self.estimator.last_vision_stamp is not None:payload["vision_stamp"]=self.estimator.last_vision_stamp
        if self.transport:self.transport.sendto(json.dumps(payload,allow_nan=False).encode(),self.command_address)

    async def control_loop(self):
        while True:
            started=time.monotonic()
            try:self.tick(started)
            except Exception:
                self.status="stopped";self.message="Control detenido por un error interno.";self.send(Command(mode="stop"))
            self.loop_duration_ms=(time.monotonic()-started)*1000
            await asyncio.sleep(max(.001,.01-(time.monotonic()-started)))

    def tick(self,now:float):
        if not self.connected(now):
            if self.status=="running":self.navigation.pause()
            self.status="disconnected";self.message="Simulador desconectado."
            self.send(Command(mode="stop"));return
        if self.scene_pending is not None:
            self.send(Command(mode="stop"))
            if self.scene_requested and now-self.scene_requested>20:self.message="Esperando confirmación de la escena."
            return
        if self.status=="running":
            if not self.visual_ready(now):
                self.status="paused";self.message="Cámara sin localización reciente. Reanuda al recuperar la señal."
                self.navigation.pause();self.send(Command(mode="pause"));return
            try:
                command=self.navigation.tick(self.estimator.pose(now),self.sim_time,self.settings["speed"],
                    self.telemetry.get("left_velocity",0.),self.telemetry.get("right_velocity",0.))
            except RuntimeError as exc:
                self.status="stopped";self.message=str(exc);self.send(Command(mode="stop"));return
            if self.navigation.finished:self.status="completed";self.message="Recorrido completado."
            self.send(command)
        else:self.send(Command(mode="pause" if self.status=="paused" else "stop" if self.status=="stopped" else "idle"))

    async def camera_loop(self):
        while True:
            try:
                frame=self.reader.read()
                if frame is not None:
                    self.last_frame=frame;self.accepted_frames+=1;self.frame_times.append(frame.capture_stamp)
                    if self.scene_pending is None and frame.capture_stamp>=self.min_camera_stamp:
                        generation=self.scene_seq
                        outcome=await asyncio.to_thread(self.vision.process,frame)
                        # Un reset puede confirmarse mientras OpenCV trabaja en otro hilo.
                        # Nunca aplicar entonces el fotograma de la escena anterior.
                        if self.scene_pending is None and generation==self.scene_seq and frame.capture_stamp>=self.min_camera_stamp:
                            self.last_vision=outcome
                            if outcome.visible and self.estimator.correct(outcome.measurement,frame.capture_stamp,time.monotonic()):
                                self.accepted_vision+=1
                                self.record_estimate(time.monotonic())
            except asyncio.CancelledError:raise
            except Exception:self.vision_errors+=1
            await asyncio.sleep(.005)

    def mission_start(self):
        now=time.monotonic()
        if not self.connected(now):raise ValueError("Conecta el simulador antes de iniciar.")
        if self.scene_pending is not None:raise ValueError("La escena todavía se está actualizando.")
        if not self.visual_ready(now):raise ValueError("Espera a que la cámara localice el robot.")
        if self.status=="running":return
        if not self.stationary():raise ValueError("Espera a que el robot se detenga antes de iniciar.")
        if self.status=="paused" and self.navigation.waypoints and not self.navigation.finished:
            self.navigation.resume(self.sim_time,self.estimator.pose(now))
        else:self.navigation.plan(self.arena,self.estimator.pose(now),self.goals,self.sim_time)
        self.status="running";self.message="Recorrido en curso."

    def mission_pause(self):
        if self.status=="running":self.status="paused";self.message="Recorrido en pausa.";self.navigation.pause()
        self.send(Command(mode="pause"))

    def mission_stop(self):
        self.status="stopped";self.message="Robot detenido.";self.navigation.pause();self.send(Command(mode="stop"))

    def scene_command(self,action:str):
        self.directory.mkdir(parents=True,exist_ok=True)
        self.scene_seq+=1
        start=self.arena.nodes["A"]
        payload={"seq":self.scene_seq,"action":action,"scale":self.settings["scale"],
            "robot_mass_kg":self.settings["mass_kg"],"friction":self.settings["friction"],
            "skid_friction":self.settings["skid_friction"],"fan_force_n":self.settings["fan_force_n"],
            "start":{**start.json(),"yaw":self.arena_config["initial_yaw"]}}
        with tempfile.NamedTemporaryFile("w",dir=self.directory,prefix="scene-",suffix=".tmp",delete=False) as temporary:
            json.dump(payload,temporary,allow_nan=False);temporary.flush();os.fsync(temporary.fileno());name=temporary.name
        os.replace(name,self.directory/"scene_command.json")
        self.scene_pending=self.scene_seq;self.scene_requested=time.monotonic()
        self.estimator.reset();self.evaluation_metrics.clear();self.navigation.clear();self.last_vision=None
        self.min_camera_stamp=self.scene_requested
        self.status="idle";self.message="Actualizando escena."
        self.send(Command(mode="stop"))

    def mission_reset(self):
        self.mission_stop();self.scene_command("reset")

    def change_settings(self,changes:dict):
        bounds={"speed":(self.arena_config["speed_min"],self.arena_config["speed_max"]),
            "scale":(self.arena_config["scale_min"],self.arena_config["scale_max"]),"mass_kg":(.05,.6),
            "friction":(.05,2.),"skid_friction":(0.,1.),"fan_force_n":(0.,3.)}
        if not changes or set(changes)-set(bounds):raise ValueError("Ajuste no reconocido.")
        validated={}
        for key,value in changes.items():
            value=finite(value);minimum,maximum=bounds[key]
            if not minimum<=value<=maximum:raise ValueError(f"{key}: valor fuera del intervalo permitido.")
            validated[key]=value
        structural=bool(set(validated)-{"speed"})
        if structural and not self.stationary():raise ValueError("Detén el robot para modificar la escena.")
        previous_scale=self.settings["scale"];self.settings.update(validated)
        if structural:
            scale=self.settings["scale"];self.arena=Arena(self.arena_config,scale)
            self.goals=[{"id":goal["id"],**self.arena.projection(Point(goal["x"]*scale/previous_scale,goal["y"]*scale/previous_scale))[0].json()} for goal in self.goals]
            self.scene_command("configure")

    def change_goals(self,goals:list[dict]):
        if not self.stationary():raise ValueError("Detén el robot para editar los objetivos.")
        if not 1<=len(goals)<=8:raise ValueError("Usa entre uno y ocho objetivos.")
        result=[];identifiers=set()
        for goal in goals:
            identifier=str(goal.get("id",""))
            if not identifier or len(identifier)>16 or identifier in identifiers:raise ValueError("Los objetivos deben tener nombres únicos.")
            identifiers.add(identifier)
            point=self.arena.projection(Point(finite(goal["x"]),finite(goal["y"])))[0]
            result.append({"id":identifier,**point.json()})
        self.goals=result;self.navigation.clear();self.message="Objetivos actualizados."

    def state(self,now:float|None=None) -> dict:
        now=time.monotonic() if now is None else now
        pose=self.estimator.pose(now);t=self.telemetry
        vision_age=None if self.estimator.last_vision_stamp is None else max(0.,1000*(now-self.estimator.last_vision_stamp))
        fps=0.
        if len(self.frame_times)>1 and self.frame_times[-1]>self.frame_times[0]:fps=(len(self.frame_times)-1)/(self.frame_times[-1]-self.frame_times[0])
        metrics={"camera_fps":fps,"vision_age_ms":vision_age,"telemetry_age_ms":None if self.telemetry_time is None else max(0.,(now-self.telemetry_time)*1000),
            "telemetry_latency_ms":self.telemetry_latency_ms,"control_processing_ms":self.loop_duration_ms,
            "vision_processing_ms":None if not self.last_vision else self.last_vision.processing_ms,
            "vision_detection_scale":None if not self.last_vision else self.last_vision.detection_scale,
            "frames_processed":self.accepted_frames,"vision_updates":self.accepted_vision,
            "vision_rejected":self.estimator.rejected_vision,"invalid_packets":self.invalid_packets,
            "contacts":self.evaluation.get("contacts",0),"position_error_m":None,"heading_error_deg":None,
            "command_applied_latency_ms":t.get("command_latency_ms")}
        metrics.update(self.evaluation_metrics.result(now))
        return {"status":self.status,"message":self.message,"sim_time":self.sim_time,"pose":pose,
            "robot":{"speed":(t.get("left_velocity",0.)+t.get("right_velocity",0.))/2,
                "left_speed":t.get("left_velocity",0.),"right_speed":t.get("right_velocity",0.),
                "battery_voltage":t.get("battery_voltage",self.robot_config["battery_voltage"]),
                "left_count":t.get("left_count",0),"right_count":t.get("right_count",0),
                "reflectance":t.get("reflectance",[]),"line_reflectance":t.get("line_reflectance",[]),
                "command_latency_ms":t.get("command_latency_ms")},
            "mission":self.navigation.public(self.goals,pose),"settings":self.settings.copy(),
            "vision":{"visible":self.visual_ready(now),"latency_ms":None if not self.last_vision else self.last_vision.latency_ms,
                "calibrated":bool(self.last_vision and self.last_vision.calibrated),
                "message":"Esperando cámara." if not self.last_vision else self.last_vision.message},
            "metrics":metrics,"connected":self.connected(now),"webots_url":"ws://127.0.0.1:1234",
            "scene_pending":self.scene_pending is not None}

    def configuration(self) -> dict:
        return {"robot":self.robot_config,"arena":self.arena.public(),"defaults":{
            "speed":self.arena_config["speed"],"scale":self.arena_config["scale"],"mass_kg":self.robot_config["mass_kg"],
            "friction":self.robot_config["friction"],"skid_friction":self.robot_config["skid_friction"],"fan_force_n":0.},
            "webots":{"url":"ws://127.0.0.1:1234","module":"/webots/wwi/WebotsView.js"}}

    def camera_jpeg(self) -> bytes|None:
        now=time.monotonic()
        if self.last_frame is None:return None
        with self.jpeg_lock:
            now=time.monotonic()
            if self.jpeg is None or now-self.jpeg_stamp>=.1:
                okay,encoded=cv2.imencode(".jpg",self.last_frame.bgra,[cv2.IMWRITE_JPEG_QUALITY,82])
                if okay:self.jpeg=encoded.tobytes();self.jpeg_stamp=now
            return self.jpeg
