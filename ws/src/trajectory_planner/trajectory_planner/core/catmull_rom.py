import numpy as np
from .interpolator_base import InterpolatorBase
from .types import TrajSample
from .utils import find_segment, clamp, unwrap_angle

class CatmullRomInterpolator(InterpolatorBase):
    """
    C¹ cubic Hermite with Catmull–Rom-like tangent estimation.
    """

    def _estimate_velocities(self):
        wps = self.waypoints
        n = len(wps)
        D = wps[0].p.shape[0]
        v = np.zeros((n, D), dtype=float)

        t = np.array([w.t for w in wps], dtype=float)
        p = np.stack([w.p for w in wps], axis=0)

        # Endpoints: one-sided differences
        v[0] = (p[1] - p[0]) / max(1e-9, (t[1] - t[0]))
        v[-1] = (p[-1] - p[-2]) / max(1e-9, (t[-1] - t[-2]))

        # Interior: weighted central difference for nonuniform time
        for i in range(1, n-1):
            dt_prev = max(1e-9, t[i] - t[i-1])
            dt_next = max(1e-9, t[i+1] - t[i])
            v_prev = (p[i] - p[i-1]) / dt_prev
            v_next = (p[i+1] - p[i]) / dt_next
            # simple average of neighbor segment velocities
            v[i] = 0.5*(v_prev + v_next)

        return v

    def sample(self, t_query: float) -> TrajSample:
        wps = self.waypoints
        times = np.array([w.t for w in wps], dtype=float)
        i = find_segment(times, t_query)

        w0, w1 = wps[i], wps[i+1]
        dt = max(1e-9, (w1.t - w0.t))
        u = clamp((t_query - w0.t)/dt, 0.0, 1.0)  # normalized segment time

        v_wp = self._estimate_velocities()
        m0 = v_wp[i]     # vel at w0
        m1 = v_wp[i+1]   # vel at w1

        # Hermite basis (position)
        h00 =  2*u**3 - 3*u**2 + 1
        h10 =      u**3 - 2*u**2 + u
        h01 = -2*u**3 + 3*u**2
        h11 =      u**3 -   u**2

        p = h00*w0.p + h10*(dt*m0) + h01*w1.p + h11*(dt*m1)

        # Derivatives wrt u
        dh00 =  6*u**2 - 6*u
        dh10 =  3*u**2 - 4*u + 1
        dh01 = -6*u**2 + 6*u
        dh11 =  3*u**2 - 2*u

        dp_du = dh00*w0.p + dh10*(dt*m0) + dh01*w1.p + dh11*(dt*m1)
        v = dp_du / dt

        # Second derivative wrt u
        d2h00 = 12*u - 6
        d2h10 =  6*u - 4
        d2h01 = -12*u + 6
        d2h11 =  6*u - 2

        d2p_du2 = d2h00*w0.p + d2h10*(dt*m0) + d2h01*w1.p + d2h11*(dt*m1)
        a = d2p_du2 / (dt**2)

        dyaw = unwrap_angle(w0.yaw, w1.yaw)
        yaw = w0.yaw + u*dyaw
        yaw_rate = dyaw / dt

        return TrajSample(t=t_query, p=p, v=v, a=a, yaw=yaw, yaw_rate=yaw_rate)
