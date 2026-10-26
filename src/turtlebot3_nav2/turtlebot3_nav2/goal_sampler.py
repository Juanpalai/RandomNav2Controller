"""Helpers for sampling random navigation goals."""

import math
import random
from typing import Optional, Tuple


def sample_goal(
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    yaw_min: float = -math.pi / 2.0,
    yaw_max: float = math.pi / 2.0,
    rng: Optional[random.Random] = None,
) -> Tuple[float, float, float]:
    """Return a random ``(x, y, yaw)`` goal inside the given bounds."""
    if min_x > max_x:
        raise ValueError('min_x must be less than or equal to max_x')
    if min_y > max_y:
        raise ValueError('min_y must be less than or equal to max_y')
    if yaw_min > yaw_max:
        raise ValueError('yaw_min must be less than or equal to yaw_max')

    generator = rng if rng is not None else random
    x = generator.uniform(min_x, max_x)
    y = generator.uniform(min_y, max_y)
    yaw = generator.uniform(yaw_min, yaw_max)
    return (x, y, yaw)
