"""Tests for the geometry helpers."""

import math

from turtlebot3_nav2.geometry import yaw_to_quaternion


def test_yaw_to_quaternion_returns_unit_quaternion():
    """Verify that quaternions produced from yaw angles are normalized."""
    for yaw in (-math.pi, -1.0, 0.0, 0.5, math.pi):
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        assert math.isclose(qx, 0.0)
        assert math.isclose(qy, 0.0)
        assert math.isclose(qx ** 2 + qy ** 2 + qz ** 2 + qw ** 2, 1.0)


def test_yaw_to_quaternion_encodes_yaw():
    """Verify that the quaternion rotates the +X axis by the requested yaw."""
    yaw = 0.75
    _, _, qz, qw = yaw_to_quaternion(yaw)
    rotated_x = 1.0 - 2.0 * qz ** 2
    rotated_y = 2.0 * qz * qw
    assert math.isclose(math.atan2(rotated_y, rotated_x), yaw)
