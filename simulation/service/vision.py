"""Cámara compartida y localización óptica a partir de AprilTags renderizados."""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
import mmap
from pathlib import Path
import struct
import time
import cv2
import numpy as np
from .tracking import CameraProjection

HEADER=struct.Struct("<8sIIIIQdd16x")

@dataclass
class Frame:
    sequence:int
    sim_time:float
    capture_stamp:float
    bgra:np.ndarray

class CameraReader:
    def __init__(self,path:Path):
        self.path=path;self.mapping=None;self.file=None;self.signature=None;self.sequence=None
    def close(self):
        if self.mapping is not None:self.mapping.close()
        if self.file is not None:self.file.close()
        self.mapping=self.file=None;self.signature=None
    def read(self) -> Frame|None:
        try:
            stat=self.path.stat();signature=(stat.st_dev,stat.st_ino,stat.st_size)
            if signature!=self.signature:
                self.close()
                if stat.st_size<HEADER.size:return None
                self.file=self.path.open("rb");self.mapping=mmap.mmap(self.file.fileno(),0,access=mmap.ACCESS_READ)
                self.signature=signature;self.sequence=None
            data=self.mapping
            magic,width,height,channels,_,sequence,sim_time,capture=HEADER.unpack(data[:HEADER.size])
            if magic!=b"RRFRAME1" or sequence%2 or sequence==self.sequence:return None
            if not (16<=width<=4096 and 16<=height<=4096 and channels==4):return None
            length=width*height*channels
            if len(data)<HEADER.size+length:return None
            pixels=bytes(data[HEADER.size:HEADER.size+length])
            after=struct.unpack_from("<Q",data,24)[0]
            if sequence!=after or after%2:return None
            self.sequence=sequence
            return Frame(sequence,sim_time,capture,np.frombuffer(pixels,np.uint8).reshape(height,width,4))
        except (FileNotFoundError,ValueError,OSError,struct.error):
            self.close();return None

@dataclass
class VisionResult:
    visible:bool
    measurement:tuple[float,float,float]|None
    capture_stamp:float
    latency_ms:float
    processing_ms:float
    markers:int
    calibrated:bool
    reprojection_error_px:float|None
    message:str=""
    detection_scale:float=1.
    marker_corners:list[list[float]]|None=None
    projection:CameraProjection|None=None

class CameraVision:
    def __init__(self,calibration_path:Path):
        self.path=calibration_path;self.modified=None;self.calibration=None
        parameters=cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),parameters)
        self.R=None;self.t=None;self.reprojection_error=None;self.detection_scale=1.

    def _load(self):
        stat=self.path.stat()
        if self.modified!=stat.st_mtime_ns:
            self.calibration=json.loads(self.path.read_text());self.modified=stat.st_mtime_ns
            self.R=self.t=None;self.reprojection_error=None
        return self.calibration

    def detect(self,gray,reference_ids):
        detections={};self.detection_scale=1.
        need_floor=self.R is None
        for scale in (1.,1.5,3.):
            pixels=gray if scale==1. else cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
            corners,ids,_=self.detector.detectMarkers(pixels)
            self.detection_scale=scale
            if ids is not None:
                for identifier,corner in zip(ids.flatten(),corners):
                    detections.setdefault(int(identifier),np.asarray(corner,dtype=np.float64).reshape(4,2)/scale)
            # En régimen las referencias calibradas no requieren detección ampliada continua.
            if 0 in detections and (not need_floor or set(reference_ids).issubset(detections)):break
        return detections

    def process(self,frame:Frame) -> VisionResult:
        started=time.monotonic()
        marker_corners=None;projection=None
        def result(measurement=None,message="",markers=0):
            ended=time.monotonic()
            return VisionResult(measurement is not None,measurement,frame.capture_stamp,
                max(0.,(ended-frame.capture_stamp)*1000),(ended-started)*1000,markers,
                self.R is not None,self.reprojection_error,message,self.detection_scale,marker_corners,projection)
        try:
            config=self._load()
            if frame.bgra.shape[:2]!=(config["height"],config["width"]):
                return result(message="La resolución de cámara cambió; esperando calibración.")
            K=np.asarray(config["camera_matrix"],dtype=np.float64)
            distortion=np.asarray(config.get("dist_coeffs",[0,0,0,0,0]),dtype=np.float64)
            if self.R is not None:projection=CameraProjection(K.copy(),distortion.copy(),self.R.copy(),self.t.copy())
            gray=cv2.cvtColor(frame.bgra,cv2.COLOR_BGRA2GRAY)
            detections=self.detect(gray,[int(tag["id"]) for tag in config["reference_tags"]])
            if not detections:return result(message="Marcador del robot fuera de vista.")
            floor_points=[];image_points=[]
            for tag in config["reference_tags"]:
                if int(tag["id"]) in detections:
                    floor_points.extend(tag["corners_world"]);image_points.extend(detections[int(tag["id"])])
            if len(floor_points)>=8:
                world=np.asarray(floor_points,np.float64);pixels=np.asarray(image_points,np.float64)
                okay,rotation,translation=cv2.solvePnP(world,pixels,K,distortion,flags=cv2.SOLVEPNP_ITERATIVE)
                if okay:
                    projected,_=cv2.projectPoints(world,rotation,translation,K,distortion)
                    error=float(np.sqrt(np.mean((projected.reshape(-1,2)-pixels)**2)))
                    R=cv2.Rodrigues(rotation)[0];camera=-R.T@translation.reshape(3)
                    if error<3. and camera[2]>0:
                        self.R=R;self.t=translation.reshape(3);self.reprojection_error=error
            if self.R is None:return result(message="Esperando referencias del suelo.",markers=len(detections))
            projection=CameraProjection(K.copy(),distortion.copy(),self.R.copy(),self.t.copy())
            if 0 not in detections:return result(message="Marcador del robot fuera de vista.",markers=len(detections))
            robot_pixels=detections[0]
            if min(np.linalg.norm(robot_pixels-np.roll(robot_pixels,-1,axis=0),axis=1))<7:
                return result(message="Marcador demasiado pequeño.",markers=len(detections))
            normalized=cv2.undistortPoints(robot_pixels.reshape(-1,1,2),K,distortion).reshape(-1,2)
            camera=-self.R.T@self.t
            world_corners=[]
            height=float(config["marker_height_m"])
            for x,y in normalized:
                ray=self.R.T@np.array([x,y,1.])
                if abs(ray[2])<1e-9:return result(message="Cámara sin intersección con el robot.")
                distance=(height-camera[2])/ray[2]
                if distance<=0:return result(message="Marcador detrás de la cámara.")
                world_corners.append(camera+distance*ray)
            world_corners=np.asarray(world_corners)
            measured_size=float(np.mean(np.linalg.norm(world_corners-np.roll(world_corners,-1,axis=0),axis=1)))
            expected=float(config.get("marker_size_m",.04))
            if not .65*expected<measured_size<1.35*expected:
                return result(message="Geometría del marcador incoherente.",markers=len(detections))
            forward=(world_corners[0]+world_corners[1]-world_corners[2]-world_corners[3])/2
            yaw=math.atan2(forward[1],forward[0]);center=np.mean(world_corners,axis=0)
            dx,dy=config.get("marker_offset_m",[0.,0.])
            x=center[0]-(math.cos(yaw)*dx-math.sin(yaw)*dy)
            y=center[1]-(math.sin(yaw)*dx+math.cos(yaw)*dy)
            marker_corners=robot_pixels.tolist()
            return result((float(x),float(y),yaw),markers=len(detections))
        except (OSError,ValueError,KeyError,cv2.error,json.JSONDecodeError) as exc:
            return result(message="Esperando calibración de cámara.")
