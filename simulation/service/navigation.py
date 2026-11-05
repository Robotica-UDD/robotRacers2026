"""Rectas geométricas y pivotes; cada llegada se verifica después de frenar."""
from __future__ import annotations
from dataclasses import dataclass
import math
from .geometry import Arena,Point
from .estimator import wrap

@dataclass
class Waypoint:
    point:Point
    goal_id:str|None=None

@dataclass
class Command:
    v:float=0.
    w:float=0.
    mode:str="idle"

def approach(value:float,target:float,maximum:float) -> float:
    return value+max(-maximum,min(maximum,target-value))

class Navigation:
    STOP_DISTANCE=.010
    ACCEPT_DISTANCE=.018  # Deja margen para el error de medida dentro del requisito de 20 mm.
    SETTLE_TIME=1.10
    def __init__(self,track_width:float):
        self.track=track_width;self.clear()
    def clear(self):
        self.waypoints=[];self.index=0;self.stage="idle";self.finished=False
        self.active_goal=None;self.reached_goals=[];self.goal_measurements=[];self.command=Command()
        self.last_time=None;self.phase_started=0.;self.turn_yaw=0.;self.turn_sign=1.
        self.turn_amount=0.;self.turn_progress=0.;self.turn_progress_mark=0.;self.start_position=Point(0,0)
        self.previous_waypoint=Point(0,0);self.planned_length=0.;self.traversed_length=0.
        self.last_progress_time=0.;self.best_distance=float("inf")
        self.line_start=Point(0,0);self.line_heading=0.;self.drive_sign=1.
        self.settle_reason=None;self.quiet_since=None;self.correction_attempts=0;self.correcting=False

    def plan(self,arena:Arena,pose:dict,goals:list[dict],sim_time:float):
        self.clear();position=Point(pose["x"],pose["y"])
        self.previous_waypoint=position;self.start_position=position
        if position.distance(arena.projection(position)[0])>arena.config["corridor_width_m"]*arena.scale/2:
            raise ValueError("El robot está fuera de los corredores.")
        previous=position
        for goal in goals:
            endpoint=Point(goal["x"],goal["y"])
            route=arena.route(previous,endpoint)
            for point in route:
                if not self.waypoints or self.waypoints[-1].point.distance(point)>1e-8:self.waypoints.append(Waypoint(point))
            if self.waypoints[-1].goal_id is not None:self.waypoints.append(Waypoint(endpoint))
            self.waypoints[-1].goal_id=goal["id"];previous=endpoint
        current=position
        for point in self.waypoints:self.planned_length+=current.distance(point.point);current=point.point
        self.last_time=sim_time;self._prepare(pose,sim_time)

    def _accept_current(self,pose,now):
        target=self.waypoints[self.index]
        self.traversed_length+=self.previous_waypoint.distance(target.point)
        if target.goal_id:
            self.reached_goals.append(target.goal_id)
            self.goal_measurements.append({"id":target.goal_id,"x":pose["x"],"y":pose["y"],
                "error_m":Point(pose["x"],pose["y"]).distance(target.point),"sim_time":now})
        self.previous_waypoint=target.point;self.index+=1;self.correction_attempts=0

    def _prepare(self,pose:dict,now:float,correction:bool=False):
        position=Point(pose["x"],pose["y"])
        # Se llama al iniciar detenido o tras la comprobación de quietud.
        while self.index<len(self.waypoints) and position.distance(self.waypoints[self.index].point)<=self.STOP_DISTANCE:
            self._accept_current(pose,now)
        if self.index>=len(self.waypoints):
            self.finished=True;self.stage="complete";self.command=Command();self.active_goal=None;return
        self.active_goal=next((point.goal_id for point in self.waypoints[self.index:] if point.goal_id),None)
        target=self.waypoints[self.index].point
        # La línea se fija desde la posición realmente asentada, no desde la esquina nominal.
        self.line_start=position;self.line_heading=math.atan2(target.y-position.y,target.x-position.x)
        self.drive_sign=1.;self.correcting=correction
        body_heading=self.line_heading
        if correction and position.distance(target)<=.20 and abs(wrap(body_heading-pose["yaw"]))>math.pi/2:
            self.drive_sign=-1.;body_heading=wrap(body_heading+math.pi)
        error=wrap(body_heading-pose["yaw"])
        self.turn_yaw=body_heading;self.turn_amount=abs(error);self.turn_sign=1. if error>=0 else -1.
        self.turn_progress=0.;self.turn_progress_mark=0.;self.phase_started=now;self.last_progress_time=now
        self.best_distance=position.distance(target);self.quiet_since=None
        self.stage="turn" if abs(error)>.045 else "drive"

    def _brake(self,now,reason):
        self.stage="settle";self.phase_started=now;self.quiet_since=None;self.settle_reason=reason
        self.command=Command();return self.command

    def pause(self):
        self.command=Command(mode="pause");self.last_time=None

    def resume(self,sim_time:float,pose:dict|None=None):
        self.last_time=sim_time;self.last_progress_time=sim_time
        if self.stage=="settle":
            self.phase_started=sim_time;self.quiet_since=None
        elif pose is not None:
            if self.index<len(self.waypoints) and Point(pose["x"],pose["y"]).distance(self.waypoints[self.index].point)<=self.STOP_DISTANCE:
                self._brake(sim_time,"arrival")
            else:self._prepare(pose,sim_time,self.correcting)

    def tick(self,pose:dict,sim_time:float,speed_limit:float,left_speed:float,right_speed:float) -> Command:
        if self.finished or not self.waypoints:return Command()
        dt=.01 if self.last_time is None else max(0.,min(.05,sim_time-self.last_time));self.last_time=sim_time
        position=Point(pose["x"],pose["y"])
        if self.stage=="settle":
            if max(abs(left_speed),abs(right_speed))<.018:
                if self.quiet_since is None:self.quiet_since=sim_time
            else:self.quiet_since=None
            quiet=self.quiet_since is not None and sim_time-self.quiet_since>=.15
            if sim_time-self.phase_started>=self.SETTLE_TIME and quiet:
                if self.settle_reason=="turn":
                    target=self.waypoints[self.index].point
                    self.line_start=position;self.line_heading=math.atan2(target.y-position.y,target.x-position.x)
                    self.stage="drive";self.best_distance=position.distance(target);self.last_progress_time=sim_time
                else:
                    target=self.waypoints[self.index].point
                    if position.distance(target)<=self.ACCEPT_DISTANCE:
                        self._accept_current(pose,sim_time);self._prepare(pose,sim_time)
                    else:
                        self.correction_attempts+=1
                        if self.correction_attempts>3:raise RuntimeError("No se alcanzó el punto dentro del margen de 20 mm.")
                        self._prepare(pose,sim_time,correction=True)
            elif sim_time-self.phase_started>5:raise RuntimeError("El robot no se detuvo después del frenado.")
            self.command=Command();return self.command
        if self.stage=="turn":
            error=wrap(self.turn_yaw-pose["yaw"])
            if abs(error)<=.015 or self.turn_sign*error<0:return self._brake(sim_time,"turn")
            new_progress=max(self.turn_progress,self.turn_amount-abs(error))
            if new_progress>self.turn_progress_mark+.008:
                self.last_progress_time=sim_time;self.turn_progress_mark=new_progress
            self.turn_progress=new_progress
            if sim_time-self.last_progress_time>5 or sim_time-self.phase_started>30:
                raise RuntimeError("Giro detenido: no hay progreso suficiente.")
            equivalent_degrees=90*self.turn_progress/max(.001,self.turn_amount)
            u=max(0.,min(1.,(equivalent_degrees-40.)/50.))
            wheel_speed=.100+(.035-.100)*(u*u*(3-2*u))
            target_w=self.turn_sign*2*wheel_speed/self.track
            self.command=Command(0.,approach(self.command.w,target_w,2*.150/self.track*dt),"run")
            return self.command
        target=self.waypoints[self.index].point;distance=position.distance(target)
        c,s=math.cos(self.line_heading),math.sin(self.line_heading)
        along=(target.x-position.x)*c+(target.y-position.y)*s
        cross=(position.y-self.line_start.y)*c-(position.x-self.line_start.x)*s
        # Cruzar el plano final ordena frenar, pero NO acredita haber llegado.
        if distance<=self.STOP_DISTANCE or along<=self.STOP_DISTANCE:return self._brake(sim_time,"arrival")
        if distance<self.best_distance-.004:self.best_distance=distance;self.last_progress_time=sim_time
        if sim_time-self.last_progress_time>5:raise RuntimeError("Sin avance hacia el siguiente punto durante 5 segundos.")
        # La referencia de rumbo no gira 180 grados cuando se sobrepasa el objetivo.
        correction=max(-.25,min(.25,math.atan2(1.2*cross,max(.10,abs(self.command.v)))))
        heading=self.line_heading-correction+(math.pi if self.drive_sign<0 else 0.)
        error=wrap(heading-pose["yaw"])
        limit=min(speed_limit,.06 if self.correcting else speed_limit)
        desired=min(limit,1.4*along,math.sqrt(2*.60*max(0.,along-.006)))
        desired*=self.drive_sign*max(0.,math.cos(error))
        v=approach(self.command.v,desired,(.50 if abs(desired)>abs(self.command.v) else .80)*dt)
        self.command=Command(v,max(-.65,min(.65,3*error)),"run");return self.command

    def public(self,goals:list[dict],pose:dict) -> dict:
        completed=self.traversed_length
        if self.stage=="drive" and self.index<len(self.waypoints):
            target=self.waypoints[self.index].point;length=self.previous_waypoint.distance(target)
            completed+=max(0.,min(length,length-Point(pose["x"],pose["y"]).distance(target)))
        progress=1. if self.finished else min(.999,completed/self.planned_length) if self.planned_length else 0.
        return {"goals":goals,"active_goal":self.active_goal,"waypoints":[w.point.json() for w in self.waypoints],
                "progress":progress,"phase":self.stage,"reached_goals":self.reached_goals,"goal_measurements":self.goal_measurements}
