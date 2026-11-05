"""Evaluación temporal separada: nunca suministra estados al control del robot."""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
import math
from .estimator import wrap

@dataclass(frozen=True)
class Sample:
    sim_time:float
    x:float
    y:float
    yaw:float
    received:float

class AlignedEvaluation:
    """Compara ambas trayectorias en un instante común sin extrapolar ninguna."""
    def __init__(self):self.clear()
    def clear(self):
        self.estimated=deque(maxlen=1000);self.truth=deque(maxlen=1000)
    def _append(self,collection,sim_time,x,y,yaw,received):
        sample=Sample(float(sim_time),float(x),float(y),float(yaw),float(received))
        if not all(math.isfinite(value) for value in vars(sample).values()):return
        if collection and sample.sim_time<collection[-1].sim_time-1e-8:
            self.clear()
            collection=self.estimated if collection is self.estimated else self.truth
        if collection and abs(sample.sim_time-collection[-1].sim_time)<1e-8:collection[-1]=sample
        else:collection.append(sample)
    def estimate(self,sim_time,x,y,yaw,received):
        if self.estimated and sim_time<self.estimated[-1].sim_time-1e-8:self.clear()
        self._append(self.estimated,sim_time,x,y,yaw,received)
    def evaluate(self,sim_time,x,y,yaw,received):
        if self.truth and sim_time<self.truth[-1].sim_time-1e-8:self.clear()
        self._append(self.truth,sim_time,x,y,yaw,received)
    @staticmethod
    def _interpolate(samples,stamp):
        for sample in reversed(samples):
            if abs(sample.sim_time-stamp)<1e-8:return sample
        for first,second in zip(samples,list(samples)[1:]):
            if first.sim_time<=stamp<=second.sim_time:
                u=(stamp-first.sim_time)/(second.sim_time-first.sim_time)
                return Sample(stamp,first.x+(second.x-first.x)*u,first.y+(second.y-first.y)*u,
                    wrap(first.yaw+wrap(second.yaw-first.yaw)*u),max(first.received,second.received))
        return None
    def result(self,now):
        empty={"position_error_m":None,"heading_error_deg":None,"evaluation_aligned":False,
            "evaluation_comparison_time":None,"evaluation_alignment_lag_ms":None}
        if not self.estimated or not self.truth:return empty
        if now-self.estimated[-1].received>.30 or now-self.truth[-1].received>.30:return empty
        first=max(self.estimated[0].sim_time,self.truth[0].sim_time)
        stamp=min(self.estimated[-1].sim_time,self.truth[-1].sim_time)
        if stamp<first-1e-8:return empty
        lag=max(self.estimated[-1].sim_time,self.truth[-1].sim_time)-stamp
        if lag>.10:return empty
        estimate=self._interpolate(self.estimated,stamp);truth=self._interpolate(self.truth,stamp)
        if estimate is None or truth is None:return empty
        return {"position_error_m":math.hypot(estimate.x-truth.x,estimate.y-truth.y),
            "heading_error_deg":math.degrees(wrap(estimate.yaw-truth.yaw)),"evaluation_aligned":True,
            "evaluation_comparison_time":stamp,"evaluation_alignment_lag_ms":max(0.,lag*1000)}
