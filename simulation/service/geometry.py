"""Geometría transitable y planificación A* sobre los corredores reales."""
from __future__ import annotations
from dataclasses import dataclass
import heapq
import math

@dataclass(frozen=True)
class Point:
    x: float
    y: float
    def distance(self, other: "Point") -> float:
        return math.hypot(self.x-other.x, self.y-other.y)
    def json(self) -> dict:
        return {"x": self.x, "y": self.y}

class Arena:
    def __init__(self, config: dict, scale: float = 1.):
        self.config = config
        self.scale = scale
        self.nodes = {key: Point(x*scale, y*scale) for key,(x,y) in config["nodes"].items()}
        self.edges = [tuple(edge) for edge in config["edges"]]

    def projection(self, point: Point) -> tuple[Point, int, float]:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            raise ValueError("La posición debe ser finita.")
        best = None
        for index,(a,b) in enumerate(self.edges):
            start,end = self.nodes[a],self.nodes[b]
            dx,dy = end.x-start.x,end.y-start.y
            t = min(1.,max(0.,((point.x-start.x)*dx+(point.y-start.y)*dy)/(dx*dx+dy*dy)))
            projected = Point(start.x+t*dx,start.y+t*dy)
            candidate = (point.distance(projected),index,t,projected)
            if best is None or candidate[:3]<best[:3]: best=candidate
        return best[3],best[1],best[2]

    def route(self, start: Point, goal: Point) -> list[Point]:
        projected_start,si,st=self.projection(start)
        projected_goal,gi,gt=self.projection(goal)
        vertices=dict(self.nodes); vertices["@start"]=projected_start;vertices["@goal"]=projected_goal
        adjacency={key:[] for key in vertices}
        def connect(a,b):
            weight=vertices[a].distance(vertices[b])
            adjacency[a].append((b,weight));adjacency[b].append((a,weight))
        for index,(a,b) in enumerate(self.edges):
            splits=[(0.,a),(1.,b)]
            if index==si:splits.append((st,"@start"))
            if index==gi:splits.append((gt,"@goal"))
            splits.sort()
            for (_,left),(_,right) in zip(splits,splits[1:]):connect(left,right)
        queue=[(projected_start.distance(projected_goal),0.,"@start")]
        costs={"@start":0.};parents={}
        while queue:
            _,cost,current=heapq.heappop(queue)
            if cost>costs.get(current,float("inf"))+1e-9:continue
            if current=="@goal":break
            for following,weight in adjacency[current]:
                candidate=cost+weight
                if candidate+1e-9<costs.get(following,float("inf")):
                    costs[following]=candidate;parents[following]=current
                    heapq.heappush(queue,(candidate+vertices[following].distance(projected_goal),candidate,following))
        if "@goal" not in costs:raise ValueError("No existe un camino entre los objetivos.")
        result=[];current="@goal"
        while True:
            result.append(vertices[current])
            if current=="@start":break
            current=parents[current]
        result.reverse()
        return [point for index,point in enumerate(result) if index==0 or point.distance(result[index-1])>1e-8]

    def public(self) -> dict:
        return {**self.config,"scale":self.scale,"nodes":{k:[p.x,p.y] for k,p in self.nodes.items()},
                "segments":[{"start":self.nodes[a].json(),"end":self.nodes[b].json()} for a,b in self.edges]}
