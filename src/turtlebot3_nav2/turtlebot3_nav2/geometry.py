"""Geometry helpers used by the navigation controller."""

import math
from typing import Tuple


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert a yaw angle in radians to an ``(x, y, z, w)`` quaternion."""
    half_yaw = yaw / 2.0
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
