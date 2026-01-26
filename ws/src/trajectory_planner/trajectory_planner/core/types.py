from dataclasses import dataclass
from typing import Optional
import numpy as np

@dataclass
class Waypoint:
    t: float
    p: np.ndarray          # shape (D,)
    yaw: float = 0.0       # optional

@dataclass
class TrajSample:
    t: float
    p: np.ndarray          # position (D,)
    v: np.ndarray          # velocity (D,)
    a: np.ndarray          # acceleration (D,)
    yaw: float = 0.0
    yaw_rate: float = 0.0
