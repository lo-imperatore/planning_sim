import numpy as np
from .interpolator_base import InterpolatorBase
from .types import TrajSample
from .utils import find_segment, clamp, unwrap_angle

class QuinticInterpolator(InterpolatorBase):
    def __init__(self):
        super().__init__()
        self._coeffs = None  # list of (D,6) lowest-first coeffs per segment

    def set_waypoints(self, waypoints):
        super().set_waypoints(waypoints)
        self._build()

    def _estimate_va(self):
        wps = self.waypoints
        n = len(wps)
        D = wps[0].p.shape[0]
        t = np.array([w.t for w in wps], dtype=float)
        p = np.stack([w.p for w in wps], axis=0)

        v = np.zeros((n, D), dtype=float)
        a = np.zeros((n, D), dtype=float)

        # velocity: same as catmull-rom estimate
        v[0] = (p[1] - p[0]) / max(1e-9, t[1]-t[0])
        v[-1] = (p[-1] - p[-2]) / max(1e-9, t[-1]-t[-2])
        for i in range(1, n-1):
            dt_prev = max(1e-9, t[i]-t[i-1])
            dt_next = max(1e-9, t[i+1]-t[i])
            v_prev = (p[i] - p[i-1]) / dt_prev
            v_next = (p[i+1] - p[i]) / dt_next
            v[i] = 0.5*(v_prev + v_next)

        # acceleration: difference of segment velocities
        a[0] = (v[1] - v[0]) / max(1e-9, t[1]-t[0])
        a[-1] = (v[-1] - v[-2]) / max(1e-9, t[-1]-t[-2])
        for i in range(1, n-1):
            dt = max(1e-9, t[i+1]-t[i-1])
            a[i] = (v[i+1] - v[i-1]) / dt

        return v, a

    def _build(self):
        wps = self.waypoints
        nseg = len(wps) - 1
        D = wps[0].p.shape[0]
        v_wp, a_wp = self._estimate_va()

        self._coeffs = []
        for i in range(nseg):
            w0, w1 = wps[i], wps[i+1]
            T = max(1e-9, w1.t - w0.t)

            p0 = w0.p
            p1 = w1.p
            v0 = v_wp[i]
            v1 = v_wp[i+1]
            a0 = a_wp[i]
            a1 = a_wp[i+1]

            # Quintic p(t)=c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5
            # with t in [0,T]
            c0 = p0
            c1 = v0
            c2 = 0.5*a0

            # Solve for c3,c4,c5 per dimension
            # Constraints at T:
            # p(T)=p1
            # p'(T)=v1
            # p''(T)=a1
            TT = T
            A = np.array([
                [TT**3,    TT**4,     TT**5],
                [3*TT**2,  4*TT**3,   5*TT**4],
                [6*TT,     12*TT**2,  20*TT**3],
            ], dtype=float)

            b_pos = p1 - (c0 + c1*TT + c2*(TT**2))
            b_vel = v1 - (c1 + 2*c2*TT)
            b_acc = a1 - (2*c2)

            # Stack RHS (3, D) and solve for each dim
            B = np.stack([b_pos, b_vel, b_acc], axis=0)  # (3, D)
            X = np.linalg.solve(A, B)  # (3, D)
            c3 = X[0]
            c4 = X[1]
            c5 = X[2]

            C = np.stack([c0, c1, c2, c3, c4, c5], axis=1)  # (D, 6) lowest-first
            self._coeffs.append((w0.t, T, C))

    def sample(self, t_query: float) -> TrajSample:
        wps = self.waypoints
        times = np.array([w.t for w in wps], dtype=float)
        i = find_segment(times, t_query)
        t0, T, C = self._coeffs[i]

        tau = clamp(t_query - t0, 0.0, T)

        # Evaluate poly and derivatives
        # p = sum c_k tau^k
        powers = np.array([tau**k for k in range(6)], dtype=float)        # (6,)
        dp = np.array([k*tau**(k-1) if k >= 1 else 0.0 for k in range(6)], dtype=float)
        ddp = np.array([k*(k-1)*tau**(k-2) if k >= 2 else 0.0 for k in range(6)], dtype=float)

        p = C @ powers
        v = C @ dp
        a = C @ ddp

        # yaw linear between segment endpoints (simple & stable)
        w0, w1 = wps[i], wps[i+1]
        u = clamp((t_query - w0.t)/max(1e-9, w1.t-w0.t), 0.0, 1.0)
        dyaw = unwrap_angle(w0.yaw, w1.yaw)
        yaw = w0.yaw + u*dyaw
        yaw_rate = dyaw / max(1e-9, w1.t-w0.t)

        return TrajSample(t=t_query, p=p, v=v, a=a, yaw=yaw, yaw_rate=yaw_rate)
