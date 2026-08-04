import math

import pytest

from vehicle_approach.goal_calculation import (
    compute_goal_pose,
    compute_standoff_xy,
    is_approach_complete,
    should_resend_goal,
)


def test_standoff_xy_stops_before_vehicle():
    goal = compute_standoff_xy(2.0, 0.0, 0.0, 0.0, 0.7)
    assert goal == pytest.approx((1.3, 0.0))


def test_goal_faces_vehicle_and_uses_standoff():
    pose = compute_goal_pose(2.0, 0.0, 0.0, 0.0, 0.7)
    assert pose is not None
    assert pose.pose.position.x == pytest.approx(1.3)
    assert pose.pose.orientation.z == pytest.approx(0.0)
    assert pose.pose.orientation.w == pytest.approx(1.0)


def test_no_goal_when_already_within_standoff():
    assert compute_goal_pose(0.5, 0.0, 0.0, 0.0, 0.7) is None


def test_should_resend_goal():
    assert should_resend_goal((1.0, 0.0), (0.0, 0.0), 0.5)
    assert not should_resend_goal((0.1, 0.0), (0.0, 0.0), 0.5)


def test_approach_complete():
    assert is_approach_complete(0.0, 0.0, 0.5, 0.0, 0.7)
