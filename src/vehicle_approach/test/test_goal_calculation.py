import math

from vehicle_approach.goal_calculation import (
    compute_goal_pose,
    is_approach_complete,
    should_resend_goal,
)


def test_compute_goal_pose_position_is_vehicle_position():
    pose = compute_goal_pose(vehicle_x=3.0, vehicle_y=4.0, robot_x=0.0, robot_y=0.0)
    assert pose.header.frame_id == 'map'
    assert pose.pose.position.x == 3.0
    assert pose.pose.position.y == 4.0


def test_compute_goal_pose_yaw_faces_vehicle():
    pose = compute_goal_pose(vehicle_x=1.0, vehicle_y=1.0, robot_x=0.0, robot_y=0.0)
    expected_yaw = math.atan2(1.0, 1.0)
    assert math.isclose(pose.pose.orientation.z, math.sin(expected_yaw / 2.0), abs_tol=1e-9)
    assert math.isclose(pose.pose.orientation.w, math.cos(expected_yaw / 2.0), abs_tol=1e-9)


def test_should_resend_goal_below_threshold_is_false():
    assert should_resend_goal((1.0, 1.0), (1.02, 1.0), threshold_m=0.1) is False


def test_should_resend_goal_above_threshold_is_true():
    assert should_resend_goal((1.0, 1.0), (1.2, 1.0), threshold_m=0.1) is True


def test_is_approach_complete_within_threshold():
    assert is_approach_complete(0.0, 0.0, 0.3, 0.0, threshold_m=0.5) is True


def test_is_approach_complete_outside_threshold():
    assert is_approach_complete(0.0, 0.0, 1.0, 0.0, threshold_m=0.5) is False
