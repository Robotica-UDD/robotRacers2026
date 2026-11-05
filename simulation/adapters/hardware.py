"""Puente explícito JSON Lines/USB para un futuro firmware compatible."""
from __future__ import annotations
import argparse
import importlib
import json
import math
from pathlib import Path
import socket
import time
from typing import Protocol
from simulation.service.config import load_configuration

class LineTransport(Protocol):
    def readline(self) -> bytes: ...
    def write(self,data:bytes) -> int: ...
    def close(self) -> None: ...

class SerialLineTransport:
    """La construcción no abre el puerto. open() requiere una acción explícita."""
    def __init__(self,port:str,baudrate:int=921600,timeout:float=.01):
        self.port=port;self.baudrate=baudrate;self.timeout=timeout;self.serial=None
    def open(self):
        serial=importlib.import_module("serial")
        connection=serial.Serial(port=None,baudrate=self.baudrate,timeout=self.timeout,write_timeout=.1)
        # DTR/RTS no deben provocar un RESET del ESP32 al abrir.
        connection.dtr=False;connection.rts=False;connection.port=self.port;connection.open()
        self.serial=connection;return self
    def readline(self):
        if self.serial is None:raise RuntimeError("El puerto no se abrió.")
        return self.serial.readline(16384)
    def write(self,data):
        if self.serial is None:raise RuntimeError("El puerto no se abrió.")
        return self.serial.write(data)
    def close(self):
        if self.serial is not None:self.serial.close();self.serial=None

class Esp32Bridge:
    def __init__(self,transport:LineTransport,robot:dict|None=None,allow_motion:bool=False,clock=time.monotonic):
        self.transport=transport;self.robot=robot or load_configuration()[0];self.clock=clock
        self.allow_motion=allow_motion;self.needs_rearm=True;self.last_command=None
        self.command_seq=None;self.wire_seq=0;self.telemetry_seq=0;self.started=clock()
        self.previous_raw=None;self.previous_micros=None;self.unwrapped_micros=0
        self.offsets=[0,0];self.previous_counts=None;self.velocity=[0.,0.]
        self.distance=2*math.pi*self.robot["wheel_radius_m"]/self.robot["encoder_counts_per_revolution"]
        self.last_sent_stop=None;self.buffer=bytearray()
    def _write(self,payload):
        self.wire_seq+=1
        payload={**payload,"source_seq":payload.get("seq",0),"seq":self.wire_seq}
        return self.transport.write(json.dumps(payload,allow_nan=False,separators=(",",":")).encode()+b"\n")
    def encoder(self,message:dict,received:float|None=None) -> dict:
        received=self.clock() if received is None else received
        if message.get("type")!="encoder":raise ValueError("Se esperaba una muestra de encoders.")
        raw=[message["left_count"],message["right_count"]];micros=message["device_micros"]
        if any(isinstance(value,bool) or not isinstance(value,int) for value in raw+[micros]):raise ValueError("Contadores no válidos.")
        if not 0<=micros<2**32:raise ValueError("device_micros debe ser uint32.")
        reset=False;delta_us=None
        normalized=[-raw[0],raw[1]]
        if self.previous_micros is not None:
            delta=micros-self.previous_micros
            if delta<-(2**31):delta+=2**32
            elif delta<0:
                reset=True;self.needs_rearm=True;self.command_seq=None
                if self.previous_counts is not None:self.offsets=[self.previous_counts[i]-normalized[i] for i in range(2)]
                delta=0
            delta_us=delta;self.unwrapped_micros+=delta
        else:self.unwrapped_micros=micros
        counts=[normalized[i]+self.offsets[i] for i in range(2)]
        if self.previous_counts is not None and delta_us is not None and 0<delta_us<=1_000_000:
            self.velocity=[(counts[i]-self.previous_counts[i])*self.distance/(delta_us*1e-6) for i in range(2)]
        else:self.velocity=[0.,0.]
        self.previous_counts=counts;self.previous_micros=micros;self.previous_raw=raw
        self.telemetry_seq+=1
        battery=float(message.get("battery_voltage",self.robot["battery_voltage"]))
        if not math.isfinite(battery):raise ValueError("Voltaje no válido.")
        return {"type":"telemetry","seq":self.telemetry_seq,"sim_time":received-self.started,"stamp":received,
            "left_count":counts[0],"right_count":counts[1],"left_velocity":self.velocity[0],"right_velocity":self.velocity[1],
            "left_pwm":float(message.get("left_pwm",0.)),"right_pwm":float(message.get("right_pwm",0.)),
            "battery_voltage":battery,"reflectance":message.get("reflectance",[]),"device_micros":micros,
            "device_elapsed_s":self.unwrapped_micros*1e-6,"device_reset":reset,"source":"hardware"}
    def command(self,message:dict,received:float|None=None) -> bool:
        now=self.clock() if received is None else received
        try:
            stamp=float(message["stamp"]);seq=int(message["seq"]);v=float(message["v"]);w=float(message["w"]);mode=message["mode"]
        except (KeyError,TypeError,ValueError):return False
        if not all(math.isfinite(value) for value in (stamp,v,w)):return False
        if now-stamp>.25 or stamp>now+.1 or (self.command_seq is not None and seq<=self.command_seq):return False
        if mode not in {"run","idle","pause","stop"}:return False
        self.command_seq=seq;self.last_command=now
        if mode!="run":self.needs_rearm=False
        if not self.allow_motion or self.needs_rearm:mode="stop"
        if mode!="run":v=w=0.
        v=max(-1.5,min(1.5,v));w=max(-12.,min(12.,w));track=self.robot["track_width_m"]
        self._write({"type":"wheel_target","seq":seq,"stamp":now,"mode":mode,"ttl_ms":250,
            "left_mps":v-w*track/2,"right_mps":v+w*track/2})
        return True
    def watchdog(self,now:float|None=None):
        now=self.clock() if now is None else now
        if self.last_command is None or now-self.last_command>.25:
            if self.last_sent_stop is None or now-self.last_sent_stop>=.10:
                self._write({"type":"wheel_target","seq":self.command_seq or 0,"stamp":now,"mode":"stop",
                    "ttl_ms":250,"left_mps":0.,"right_mps":0.})
                self.last_sent_stop=now
    def read(self) -> list[dict]:
        data=self.transport.readline()
        if data:self.buffer.extend(data)
        if len(self.buffer)>32768:self.buffer.clear();raise ValueError("Línea de telemetría demasiado larga.")
        result=[]
        while b"\n" in self.buffer:
            line,_,rest=self.buffer.partition(b"\n");self.buffer=bytearray(rest)
            if not line.strip():continue
            try:
                message=json.loads(line)
                if isinstance(message,dict) and message.get("type")=="encoder":result.append(self.encoder(message))
            except (ValueError,KeyError,TypeError):continue
        return result
    def close(self):
        try:self._write({"type":"wheel_target","seq":self.command_seq or 0,"stamp":self.clock(),"mode":"stop",
                "ttl_ms":250,"left_mps":0.,"right_mps":0.})
        finally:self.transport.close()


def main():
    parser=argparse.ArgumentParser(description="Puente USB/UDP. Requiere firmware JSON compatible, no REV3.11 sin adaptar.")
    parser.add_argument("--port",required=True);parser.add_argument("--baud",type=int,default=921600)
    parser.add_argument("--command-port",type=int,default=9212);parser.add_argument("--telemetry-port",type=int,default=9201)
    parser.add_argument("--enable-motion",action="store_true",help="Permite consignas después de un mensaje idle/stop; sin esta opción solo se recibe telemetría.")
    arguments=parser.parse_args()
    connection=SerialLineTransport(arguments.port,arguments.baud).open()
    bridge=Esp32Bridge(connection,allow_motion=arguments.enable_motion)
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind(("127.0.0.1",arguments.command_port));udp.setblocking(False)
    try:
        while True:
            latest=None
            while True:
                try:latest=udp.recv(4096)
                except BlockingIOError:break
            if latest is not None:
                try:bridge.command(json.loads(latest))
                except (ValueError,TypeError):pass
            try:
                for sample in bridge.read():udp.sendto(json.dumps(sample,allow_nan=False).encode(),("127.0.0.1",arguments.telemetry_port))
            except (ValueError,KeyError,TypeError):pass
            bridge.watchdog()
    except KeyboardInterrupt:pass
    finally:bridge.close();udp.close()

if __name__=="__main__":main()
