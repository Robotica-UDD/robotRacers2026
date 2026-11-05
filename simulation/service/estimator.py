"""Filtro de Kalman extendido: solamente encoders y medidas visuales."""
from __future__ import annotations
from collections import deque
import math
import numpy as np

def wrap(angle: float) -> float:
    return (angle+math.pi)%(2*math.pi)-math.pi

class PoseEstimator:
    def __init__(self, robot: dict):
        self.radius=robot["wheel_radius_m"];self.track=robot["track_width_m"]
        self.count_distance=2*math.pi*self.radius/robot["encoder_counts_per_revolution"]
        self.reset()

    def reset(self):
        self.x=np.zeros(3,dtype=float);self.P=np.diag([.05,.05,.3])**2
        self.initialized=False;self.last_counts=None;self.last_stamp=None
        self.last_vision_stamp=None;self.last_pose_stamp=None
        self.history=deque(maxlen=300);self.rejected_vision=0

    def predict(self,left_count: int,right_count: int,stamp: float):
        counts=(left_count,right_count)
        previous=self.last_counts;self.last_counts=counts
        previous_stamp=self.last_stamp;self.last_stamp=stamp
        if previous is None or not self.initialized:return
        if previous_stamp is not None and stamp<=previous_stamp:return
        dl=(counts[0]-previous[0])*self.count_distance
        dr=(counts[1]-previous[1])*self.count_distance
        # Un salto de contador por reinicio no se convierte en movimiento físico.
        if abs(dl)>1 or abs(dr)>1:
            self.initialized=False;self.history.clear();return
        distance=(dl+dr)/2;dyaw=(dr-dl)/self.track
        middle=self.x[2]+dyaw/2;c,s=math.cos(middle),math.sin(middle)
        F=np.eye(3);F[0,2]=-distance*s;F[1,2]=distance*c
        self.x += [distance*c,distance*s,dyaw];self.x[2]=wrap(self.x[2])
        quantization=self.count_distance*.20
        q_distance=quantization**2+abs(distance)*.00015
        q_angle=(quantization/self.track)**2+abs(dyaw)*.0004
        Q=np.diag([q_distance,q_distance,q_angle])
        self.P=F@self.P@F.T+Q
        self.last_pose_stamp=stamp
        self.history.append((stamp,self.x.copy()))

    def correct(self,measurement,stamp:float,now:float) -> bool:
        z=np.asarray(measurement,dtype=float)
        if z.shape!=(3,) or not np.all(np.isfinite(z)):return False
        if now-stamp>.25 or stamp>now+.1:return False
        if self.last_vision_stamp is not None and stamp<=self.last_vision_stamp:return False
        if not self.initialized:
            self.x=z.copy();self.x[2]=wrap(self.x[2]);self.P=np.diag([.004,.004,.018])**2
            self.initialized=True;self.history.clear()
        else:
            # Compensar el transporte de la imagen con el desplazamiento de los encoders.
            candidates=[entry for entry in self.history if entry[0]<=stamp]
            if candidates:
                displacement=self.x-candidates[-1][1];displacement[2]=wrap(displacement[2])
                z=z+displacement;z[2]=wrap(z[2])
            residual=z-self.x;residual[2]=wrap(residual[2])
            R=np.diag([.004,.004,.018])**2
            S=self.P+R
            if float(residual@np.linalg.solve(S,residual))>25:
                self.rejected_vision+=1;return False
            K=np.linalg.solve(S,self.P.T).T
            correction=K@residual;self.x+=correction;self.x[2]=wrap(self.x[2])
            I=np.eye(3);self.P=(I-K)@self.P@(I-K).T+K@R@K.T
            for _,historical in self.history:
                historical+=correction;historical[2]=wrap(historical[2])
        self.last_vision_stamp=stamp;self.last_pose_stamp=now
        return True

    def pose(self,now:float) -> dict:
        age=None if self.last_pose_stamp is None else max(0.,1000*(now-self.last_pose_stamp))
        source="sin localización"
        if self.initialized:
            source="cámara + encoders" if self.last_vision_stamp and now-self.last_vision_stamp<.08 else "encoders"
        return {"x":float(self.x[0]),"y":float(self.x[1]),"yaw":float(self.x[2]),"source":source,"age_ms":age}
