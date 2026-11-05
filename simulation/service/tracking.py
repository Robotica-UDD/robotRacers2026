"""Geometría de presentación de cámara, separada de las imágenes de control."""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import TYPE_CHECKING
import cv2
import numpy as np

if TYPE_CHECKING:
    from .vision import Frame,VisionResult


@dataclass(frozen=True)
class CameraProjection:
    """Copia de la calibración óptica estimada al procesar un fotograma."""
    matrix:np.ndarray
    distortion:np.ndarray
    rotation:np.ndarray
    translation:np.ndarray

    def pixels(self,world) -> list[list[float]]|None:
        points=np.asarray(world,dtype=np.float64).reshape(-1,3)
        if not np.isfinite(points).all():return None
        camera=(self.rotation@points.T).T+self.translation
        if np.any(camera[:,2]<=0):return None
        rotation,_=cv2.Rodrigues(self.rotation)
        image,_=cv2.projectPoints(points,rotation,self.translation,self.matrix,self.distortion)
        image=image.reshape(-1,2)
        return image.tolist() if np.isfinite(image).all() else None


def camera_tracking(frame:Frame,outcome:VisionResult|None,robot:dict,goals:list[dict],
                    active_goal:str|None,now:float,max_age:float=.5) -> dict:
    """Proyecta exclusivamente observaciones y calibración de esta misma imagen.

    Los objetivos son intenciones editables, no observaciones del robot. No se
    consulta el estimador de pose ni la evaluación verdadera del simulador.
    """
    height,width=frame.bgra.shape[:2]
    metadata={"seq":frame.sequence,"sim_time":frame.sim_time,"capture_stamp":frame.capture_stamp,
        "age_ms":max(0.,(now-frame.capture_stamp)*1000),"width":width,"height":height,
        "tracking":"lost","robot_center":None,"robot_box":None,"marker_corners":None,
        "target":None,"goals":[]}
    projection=None if outcome is None else outcome.projection
    if projection is None:return metadata
    for goal in goals:
        pixels=projection.pixels([[goal["x"],goal["y"],0.]])
        if pixels is not None:
            item={"id":goal["id"],"pixel":pixels[0]}
            metadata["goals"].append(item)
            if goal["id"]==active_goal:metadata["target"]=item.copy()
    if (not outcome.visible or outcome.measurement is None or outcome.marker_corners is None
            or now-frame.capture_stamp>max_age or frame.capture_stamp>now+.1):return metadata
    x,y,yaw=outcome.measurement
    # Envolvente visual del CAD RevQ: incluye ruedas, barra frontal y soporte trasero.
    # Se puede sustituir al actualizar las medidas del modelo, sin tocar el control.
    half_width=(robot["track_width_m"]+robot["wheel_width_m"])/2
    bounds=robot.get("visual_bounds_m",{"x":[-.05285,.08315],"y":[-half_width,half_width],"z":[0.,.0353]})
    local=np.asarray([[a,b,c] for a in bounds["x"] for b in bounds["y"] for c in bounds["z"]],float)
    c,s=math.cos(yaw),math.sin(yaw)
    world=local@np.array([[c,s,0.],[-s,c,0.],[0.,0.,1.]])+[x,y,0.]
    pixels=projection.pixels(world)
    center=projection.pixels([[x,y,robot.get("marker_height_m",.023)]])
    if pixels is None or center is None:return metadata
    pixels=np.asarray(pixels)
    minimum=np.maximum([0.,0.],pixels.min(axis=0));maximum=np.minimum([width,height],pixels.max(axis=0))
    if np.any(maximum<=minimum):return metadata
    metadata.update(tracking="visible",robot_center=center[0],
        robot_box={"x":float(minimum[0]),"y":float(minimum[1]),
            "width":float(maximum[0]-minimum[0]),"height":float(maximum[1]-minimum[1])},
        marker_corners=outcome.marker_corners)
    return metadata
