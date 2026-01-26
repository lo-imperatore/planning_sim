import numpy as np
from .types import TrajSample
from .utils import clamp

class TimeScaledPath:
    """
    Takes a pre-sampled path (positions along s), builds a time law respecting v_max, a_max.
    Then allows sampling by time -> (p, v, a) approximately.
    """
    def __init__(self, p_samples: np.ndarray, ds: float, v_max: float, a_max: float):
        """
        p_samples: (N, D) positions along the path in order
        ds: approximate arc-length step between samples
        """
        if len(p_samples) < 2:
            raise ValueError("Need >=2 path samples")
        self.p = p_samples
        self.ds = float(ds)
        self.v_max = float(v_max)
        self.a_max = float(a_max)

        self.N, self.D = self.p.shape
        self.s = np.linspace(0.0, self.ds*(self.N-1), self.N)

        self.v = self._compute_speed_profile()
        self.t = self._integrate_time()

    def _compute_speed_profile(self):
        N = self.N
        ds = self.ds
        v = np.ones((N,), dtype=float) * self.v_max
        amax = max(1e-9, self.a_max)

        # Forward pass (accel limit)
        v[0] = min(v[0], self.v_max)
        for i in range(N-1):
            v[i+1] = min(v[i+1], np.sqrt(max(0.0, v[i]**2 + 2*amax*ds)))

        # Backward pass (decel limit)
        for i in reversed(range(N-1)):
            v[i] = min(v[i], np.sqrt(max(0.0, v[i+1]**2 + 2*amax*ds)))

        # Ensure endpoints can be zero if desired (optional):
        # v[0] = 0.0; v[-1] = 0.0
        return v

    def _integrate_time(self):
        N = self.N
        ds = self.ds
        t = np.zeros((N,), dtype=float)

        for i in range(N-1):
            v0 = max(1e-6, self.v[i])
            v1 = max(1e-6, self.v[i+1])
            dt = 2*ds/(v0+v1)  # trapezoidal on v
            t[i+1] = t[i] + dt
        return t

    def duration(self):
        return float(self.t[-1])

    def sample(self, t_query: float) -> TrajSample:
        # Find indices around t_query
        tq = clamp(t_query, 0.0, self.t[-1])
        j = int(np.searchsorted(self.t, tq))
        if j <= 0:
            j = 1
        if j >= self.N:
            j = self.N - 1

        t0, t1 = self.t[j-1], self.t[j]
        a = 0.0 if (t1 - t0) < 1e-9 else (tq - t0)/(t1 - t0)

        p = (1-a)*self.p[j-1] + a*self.p[j]

        # approximate tangent dp/ds using neighbors
        if 1 <= j < self.N-1:
            dp_ds = (self.p[j+1] - self.p[j-1]) / (2*self.ds)
        else:
            dp_ds = (self.p[j] - self.p[j-1]) / max(1e-9, self.ds)

        # scalar speed and accel approximations
        v_s = (1-a)*self.v[j-1] + a*self.v[j]

        # accel in s-dot: finite diff of speed over time
        # a_s ~ dv/dt
        dv = (self.v[j] - self.v[j-1])
        dt = max(1e-9, (t1 - t0))
        a_s = dv / dt

        v = dp_ds * v_s
        acc = dp_ds * a_s  # approx (ignores curvature term)

        return TrajSample(t=tq, p=p, v=v, a=acc, yaw=0.0, yaw_rate=0.0)
