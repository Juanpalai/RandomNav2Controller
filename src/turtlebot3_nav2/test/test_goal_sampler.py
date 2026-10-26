"""Tests for random goal sampling."""

import math
import random

import pytest

from turtlebot3_nav2.goal_sampler import sample_goal


def test_sample_goal_stays_within_bounds():
    """Check that sampled goals stay inside the configured area."""
    rng = random.Random(42)
    for _ in range(100):
        x, y, yaw = sample_goal(-1.5, 1.5, -2.0, 2.0, rng=rng)
        assert -1.5 <= x <= 1.5
        assert -2.0 <= y <= 2.0
        assert -math.pi / 2.0 <= yaw <= math.pi / 2.0


def test_sample_goal_rejects_inverted_x_bounds():
    """Reject invalid X bounds with a clear error."""
    with pytest.raises(ValueError, match='min_x'):
        sample_goal(1.0, -1.0, -1.0, 1.0)


def test_sample_goal_rejects_inverted_y_bounds():
    """Reject invalid Y bounds with a clear error."""
    with pytest.raises(ValueError, match='min_y'):
        sample_goal(-1.0, 1.0, 1.0, -1.0)
