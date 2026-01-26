import numpy as np
import math
from typing import Tuple

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def unwrap_angle(prev: float, nxt: float) -> float:
    """Return delta angle nxt-prev wrapped to [-pi, pi]."""
    d = nxt - prev
    while d > math.pi:
        d -= 2*math.pi
    while d < -math.pi:
        d += 2*math.pi
    return d

def find_segment(times: np.ndarray, t: float) -> int:
    """
    Return i such that times[i] <= t <= times[i+1], assuming times increasing.
    """
    if t <= times[0]:
        return 0
    if t >= times[-1]:
        return len(times) - 2
    # binary search
    return int(np.searchsorted(times, t) - 1)

def poly_derivative_coeffs(c: np.ndarray, order: int) -> np.ndarray:
    """
    c: coefficients highest-first or lowest-first? We'll use lowest-first: p(t)=sum c[k] t^k
    Return coefficients of d^order p / dt^order in lowest-first.
    """
    out = c.copy()
    for _ in range(order):
        if len(out) <= 1:
            return np.zeros((1,), dtype=float)
        out = np.array([k*out[k] for k in range(1, len(out))], dtype=float)
    return out

def poly_eval(c: np.ndarray, t: float) -> float:
    # lowest-first Horner
    y = 0.0
    for k in reversed(range(len(c))):
        y = y*t + c[k]
    return y

def poly_eval_vec(C: np.ndarray, t: float) -> np.ndarray:
    """
    C: (D, K) coefficients per dimension, lowest-first
    """
    return np.array([poly_eval(C[d], t) for d in range(C.shape[0])], dtype=float)
