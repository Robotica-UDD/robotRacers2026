"""Productor de imágenes OpenCV compatible con el mismo lector de la simulación."""
from __future__ import annotations
import argparse
import json
import mmap
import os
from pathlib import Path
import tempfile
import time
import cv2
import numpy as np
from simulation.service.vision import HEADER

class FrameWriter:
    """Publica imágenes completas; una resolución nueva reemplaza el archivo atómicamente."""
    def __init__(self,path:Path):
        self.path=Path(path);self.mapping=None;self.file=None;self.size=None;self.sequence=0
    def close(self):
        if self.mapping is not None:self.mapping.close()
        if self.file is not None:self.file.close()
        self.mapping=self.file=None;self.size=None
    def _allocate(self,required:int):
        self.close();self.path.parent.mkdir(parents=True,exist_ok=True)
        temporary=tempfile.NamedTemporaryFile("w+b",dir=self.path.parent,prefix="camera-",suffix=".tmp",delete=False)
        temporary.truncate(required);temporary.flush()
        mapped=mmap.mmap(temporary.fileno(),required,access=mmap.ACCESS_WRITE)
        os.replace(temporary.name,self.path)
        self.file=temporary;self.mapping=mapped;self.size=required
    def publish(self,bgra:np.ndarray,stamp:float,elapsed:float):
        if bgra.dtype!=np.uint8 or bgra.ndim!=3 or bgra.shape[2]!=4:
            raise ValueError("La imagen debe ser BGRA de ocho bits.")
        height,width=bgra.shape[:2];pixels=np.ascontiguousarray(bgra).tobytes()
        required=HEADER.size+len(pixels)
        if self.mapping is None or self.size!=required:self._allocate(required)
        self.sequence+=2
        self.mapping[:HEADER.size]=HEADER.pack(b"RRFRAME1",width,height,4,0,self.sequence-1,elapsed,stamp)
        self.mapping[HEADER.size:]=pixels
        self.mapping[:HEADER.size]=HEADER.pack(b"RRFRAME1",width,height,4,0,self.sequence,elapsed,stamp)

class RealCamera:
    def __init__(self,source,calibration:dict,capture_factory=None,clock=time.monotonic):
        self.source=source;self.calibration=calibration;self.clock=clock
        self.factory=capture_factory or cv2.VideoCapture;self.capture=None;self.started=None
        K=np.asarray(calibration["camera_matrix"])
        if K.shape!=(3,3) or not np.all(np.isfinite(K)):raise ValueError("Matriz intrínseca no válida.")
        if calibration["width"]<=0 or calibration["height"]<=0:raise ValueError("Resolución no válida.")
    def open(self):
        if self.capture is not None:return self
        capture=self.factory(self.source)
        if not capture.isOpened():capture.release();raise RuntimeError("No se pudo abrir la cámara seleccionada.")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH,self.calibration["width"])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT,self.calibration["height"])
        capture.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.capture=capture;self.started=self.clock();return self
    def read(self):
        if self.capture is None:raise RuntimeError("La cámara todavía no se abrió.")
        okay,bgr=self.capture.read();stamp=self.clock()
        if not okay or bgr is None:raise RuntimeError("La cámara no entregó una imagen.")
        if bgr.shape[:2]!=(self.calibration["height"],self.calibration["width"]):
            raise ValueError("La resolución recibida no coincide con la calibración.")
        if bgr.ndim==2:bgra=cv2.cvtColor(bgr,cv2.COLOR_GRAY2BGRA)
        elif bgr.shape[2]==3:bgra=cv2.cvtColor(bgr,cv2.COLOR_BGR2BGRA)
        elif bgr.shape[2]==4:bgra=bgr
        else:raise ValueError("Formato de imagen no compatible.")
        return bgra,stamp,stamp-self.started
    def close(self):
        if self.capture is not None:self.capture.release();self.capture=None
    def __enter__(self):return self.open()
    def __exit__(self,*_):self.close()


def write_calibration(path:Path,config:dict):
    config={**config,"source":"real_camera","timestamp_kind":"host_receipt"}
    path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile("w",dir=path.parent,prefix="calibration-",suffix=".tmp",delete=False) as temporary:
        json.dump(config,temporary,allow_nan=False);temporary.flush();os.fsync(temporary.fileno());filename=temporary.name
    os.replace(filename,path)


def main():
    parser=argparse.ArgumentParser(description="Cámara real para RobotRacers. No controla motores.")
    parser.add_argument("--device",required=True,help="Índice de cámara, archivo de vídeo o URL local autorizada.")
    parser.add_argument("--calibration",required=True,type=Path)
    parser.add_argument("--runtime",required=True,type=Path,help="Directorio dedicado al modo hardware.")
    arguments=parser.parse_args()
    source=int(arguments.device) if arguments.device.isdecimal() else arguments.device
    calibration=json.loads(arguments.calibration.read_text())
    writer=FrameWriter(arguments.runtime/"camera.frame")
    write_calibration(arguments.runtime/"camera_calibration.json",calibration)
    try:
        with RealCamera(source,calibration) as camera:
            while True:
                frame,stamp,elapsed=camera.read();writer.publish(frame,stamp,elapsed)
    except KeyboardInterrupt:pass
    finally:writer.close()

if __name__=="__main__":main()
