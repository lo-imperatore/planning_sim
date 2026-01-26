import numpy as np
from .interpolator_base import InterpolatorBase
from .types import TrajSample
from .utils import find_segment, clamp, unwrap_angle

class LinearTimeInterpolator(InterpolatorBase):
    def sample(self, t: float) -> TrajSample:
        wps = self.waypoints
        times = np.array([w.t for w in wps], dtype=float)

        i = find_segment(times, t)
        w0, w1 = wps[i], wps[i+1]
        dt = max(1e-9, (w1.t - w0.t))
        a = clamp((t - w0.t)/dt, 0.0, 1.0)

        p = w0.p + a*(w1.p - w0.p)
        v = (w1.p - w0.p) / dt
        acc = np.zeros_like(v)

        dyaw = unwrap_angle(w0.yaw, w1.yaw)
        yaw = w0.yaw + a*dyaw
        yaw_rate = dyaw / dt

        return TrajSample(t=t, p=p, v=v, a=acc, yaw=yaw, yaw_rate=yaw_rate)
