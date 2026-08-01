import math

from geometry_msgs.msg import PoseStamped


def waypoint_to_pose_stamped(
    x: float, y: float, yaw: float, frame_id: str = 'map'
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose
