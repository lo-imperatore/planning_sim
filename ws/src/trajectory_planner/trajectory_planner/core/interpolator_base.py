from abc import ABC, abstractmethod
from typing import List
from .types import Waypoint, TrajSample

class InterpolatorBase(ABC):
    def __init__(self):
        self.waypoints: List[Waypoint] = []

    def set_waypoints(self, waypoints: List[Waypoint]) -> None:
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")
        self.waypoints = sorted(waypoints, key=lambda w: w.t)
        for i in range(len(self.waypoints)-1):
            if self.waypoints[i+1].t <= self.waypoints[i].t:
                raise ValueError("Waypoint times must be strictly increasing")

    @abstractmethod
    def sample(self, t: float) -> TrajSample:
        ...
