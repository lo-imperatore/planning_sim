from .linear import LinearTimeInterpolator
from .catmull_rom import CatmullRomInterpolator
from .quintic import QuinticInterpolator
from .min_derivative import MinimumJerkInterpolator, MinimumSnapInterpolator

def make_interpolator(name: str):
    name = (name or "").lower()
    if name in ("linear", "lerp"):
        return LinearTimeInterpolator()
    if name in ("catmull_rom", "catmullrom", "cubic", "hermite"):
        return CatmullRomInterpolator()
    if name in ("quintic", "c2"):
        return QuinticInterpolator()
    if name in ("min_jerk", "minimum_jerk", "jerk"):
        return MinimumJerkInterpolator()
    if name in ("min_snap", "minimum_snap", "snap"):
        return MinimumSnapInterpolator()
    raise ValueError(f"Unknown interpolator: {name}")
