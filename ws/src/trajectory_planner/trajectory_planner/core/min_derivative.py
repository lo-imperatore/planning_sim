import numpy as np
from .interpolator_base import InterpolatorBase
from .types import TrajSample
from .utils import find_segment, clamp, unwrap_angle

def _segment_Q(T: float, deg: int, r: int) -> np.ndarray:
    """
    Cost matrix for one segment for minimizing integral of squared r-th derivative.
    Polynomial basis: p(t)=sum_{k=0..deg} c_k t^k (lowest-first)
    Q_{ij} = ∫ d^r/dt^r (t^i) * d^r/dt^r (t^j) dt from 0..T
    """
    Q = np.zeros((deg+1, deg+1), dtype=float)
    for i in range(deg+1):
        for j in range(deg+1):
            if i < r or j < r:
                continue
            ci = 1.0
            cj = 1.0
            for k in range(r):
                ci *= (i-k)
                cj *= (j-k)
            power = (i-r) + (j-r)
            Q[i, j] = ci * cj * (T**(power+1)) / (power+1)
    return Q

def _A_row(t: float, deg: int, d: int) -> np.ndarray:
    """
    Row mapping coeffs -> derivative d evaluated at time t.
    p^(d)(t) = sum c_k * k*(k-1)*...*(k-d+1) * t^(k-d)
    """
    row = np.zeros((deg+1,), dtype=float)
    for k in range(deg+1):
        if k < d:
            continue
        coef = 1.0
        for j in range(d):
            coef *= (k-j)
        row[k] = coef * (t**(k-d))
    return row

class MinimumDerivativeInterpolator(InterpolatorBase):
    """
    Global minimum-derivative trajectory with KKT solve.
    deg=5, r=3 -> min-jerk
    deg=7, r=4 -> min-snap
    """
    def __init__(self, deg: int, r: int):
        super().__init__()
        self.deg = deg
        self.r = r
        self._segments = []  # list of (t0, T, Cdim) where Cdim=(D, deg+1)

    def set_waypoints(self, waypoints):
        super().set_waypoints(waypoints)
        self._build()

    def _build(self):
        wps = self.waypoints
        n = len(wps)
        nseg = n - 1
        D = wps[0].p.shape[0]

        times = np.array([w.t for w in wps], dtype=float)
        Tseg = np.array([times[i+1]-times[i] for i in range(nseg)], dtype=float)

        # Build block-diagonal Q for all segments
        K = (self.deg + 1) * nseg
        Q = np.zeros((K, K), dtype=float)
        for s in range(nseg):
            Qs = _segment_Q(Tseg[s], self.deg, self.r)
            i0 = s*(self.deg+1)
            Q[i0:i0+self.deg+1, i0:i0+self.deg+1] = Qs

        # Constraints:
        # 1) Position at each waypoint (segment start/end)
        # 2) Continuity of derivatives 1..(r-1) at internal knots
        # 3) Boundary derivatives (1..r-1) at start and end set to 0 (simple default)
        #
        # You can later expose boundary vel/acc params.

        rows = []
        rhs_list = []  # will be (M, D)

        # Helper: constraint on segment s at local time t (0..Tseg[s])
        def add_constraint(seg_idx, local_t, deriv_order, value_vec):
            row = np.zeros((K,), dtype=float)
            base = seg_idx*(self.deg+1)
            row[base:base+self.deg+1] = _A_row(local_t, self.deg, deriv_order)
            rows.append(row)
            rhs_list.append(value_vec)

        P = np.stack([w.p for w in wps], axis=0)  # (n, D)

        # Position constraints at all segment endpoints
        for s in range(nseg):
            add_constraint(s, 0.0, 0, P[s])
            add_constraint(s, Tseg[s], 0, P[s+1])

        # Continuity constraints at internal waypoints for derivatives 1..(r-1)
        for s in range(nseg-1):
            for d in range(1, self.r):
                # p_s^(d)(T) - p_{s+1}^(d)(0) = 0
                row = np.zeros((K,), dtype=float)
                base_s = s*(self.deg+1)
                base_n = (s+1)*(self.deg+1)
                row[base_s:base_s+self.deg+1] = _A_row(Tseg[s], self.deg, d)
                row[base_n:base_n+self.deg+1] = -_A_row(0.0, self.deg, d)
                rows.append(row)
                rhs_list.append(np.zeros((D,), dtype=float))

        # Boundary derivatives set to 0 (start and end), orders 1..(r-1)
        for d in range(1, self.r):
            add_constraint(0, 0.0, d, np.zeros((D,), dtype=float))
            add_constraint(nseg-1, Tseg[-1], d, np.zeros((D,), dtype=float))

        A = np.stack(rows, axis=0)  # (M, K)
        B = np.stack(rhs_list, axis=0)  # (M, D)
        M = A.shape[0]

        # KKT solve: [Q A^T; A 0] [x; lambda] = [0; b]
        # Solve per-dimension (D is small)
        KKT = np.zeros((K+M, K+M), dtype=float)
        KKT[:K, :K] = Q + 1e-12*np.eye(K)  # tiny regularization
        KKT[:K, K:] = A.T
        KKT[K:, :K] = A
        rhs = np.zeros((K+M, D), dtype=float)
        rhs[K:, :] = B

        sol = np.linalg.solve(KKT, rhs)  # (K+M, D)
        x = sol[:K, :]  # polynomial coeffs (K, D)

        # Store per segment coefficients as (D, deg+1) lowest-first
        self._segments = []
        for s in range(nseg):
            base = s*(self.deg+1)
            Cs = x[base:base+self.deg+1, :].T  # (D, deg+1)
            self._segments.append((times[s], Tseg[s], Cs))

    def sample(self, t_query: float) -> TrajSample:
        wps = self.waypoints
        times = np.array([w.t for w in wps], dtype=float)
        i = find_segment(times, t_query)

        t0, T, C = self._segments[i]
        tau = clamp(t_query - t0, 0.0, T)

        # Evaluate p, v, a
        deg = self.deg
        powers = np.array([tau**k for k in range(deg+1)], dtype=float)
        dp = np.array([k*tau**(k-1) if k >= 1 else 0.0 for k in range(deg+1)], dtype=float)
        ddp = np.array([k*(k-1)*tau**(k-2) if k >= 2 else 0.0 for k in range(deg+1)], dtype=float)

        p = C @ powers
        v = C @ dp
        a = C @ ddp

        # yaw: still simple linear between nearest waypoint pair
        w0, w1 = wps[i], wps[i+1]
        u = clamp((t_query - w0.t)/max(1e-9, w1.t-w0.t), 0.0, 1.0)
        dyaw = unwrap_angle(w0.yaw, w1.yaw)
        yaw = w0.yaw + u*dyaw
        yaw_rate = dyaw / max(1e-9, w1.t-w0.t)

        return TrajSample(t=t_query, p=p, v=v, a=a, yaw=yaw, yaw_rate=yaw_rate)

class MinimumJerkInterpolator(MinimumDerivativeInterpolator):
    def __init__(self):
        super().__init__(deg=5, r=3)

class MinimumSnapInterpolator(MinimumDerivativeInterpolator):
    def __init__(self):
        super().__init__(deg=7, r=4)
