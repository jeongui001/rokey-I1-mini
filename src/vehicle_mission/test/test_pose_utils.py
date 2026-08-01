import math

from vehicle_mission.pose_utils import waypoint_to_pose_stamped


def test_zero_yaw_identity_orientation():
    pose = waypoint_to_pose_stamped(1.0, 2.0, 0.0)
    assert pose.header.frame_id == 'map'
    assert pose.pose.position.x == 1.0
    assert pose.pose.position.y == 2.0
    assert math.isclose(pose.pose.orientation.z, 0.0, abs_tol=1e-9)
    assert math.isclose(pose.pose.orientation.w, 1.0, abs_tol=1e-9)


def test_half_pi_yaw_orientation():
    pose = waypoint_to_pose_stamped(0.0, 0.0, math.pi / 2)
    assert math.isclose(pose.pose.orientation.z, math.sin(math.pi / 4), abs_tol=1e-9)
    assert math.isclose(pose.pose.orientation.w, math.cos(math.pi / 4), abs_tol=1e-9)


def test_pi_yaw_orientation():
    pose = waypoint_to_pose_stamped(0.0, 0.0, math.pi)
    assert math.isclose(pose.pose.orientation.z, 1.0, abs_tol=1e-9)
    assert math.isclose(pose.pose.orientation.w, 0.0, abs_tol=1e-9)
