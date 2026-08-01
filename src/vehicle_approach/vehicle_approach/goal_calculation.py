import math

from geometry_msgs.msg import PoseStamped


def compute_goal_pose(
    vehicle_x: float, vehicle_y: float, robot_x: float, robot_y: float
) -> PoseStamped:
    yaw = math.atan2(vehicle_y - robot_y, vehicle_x - robot_x)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = vehicle_x
    pose.pose.position.y = vehicle_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def should_resend_goal(
    new_position: tuple[float, float],
    last_sent_position: tuple[float, float],
    threshold_m: float,
) -> bool:
    dx = new_position[0] - last_sent_position[0]
    dy = new_position[1] - last_sent_position[1]
    return math.hypot(dx, dy) >= threshold_m


def is_approach_complete(
    robot_x: float, robot_y: float, vehicle_x: float, vehicle_y: float, threshold_m: float
) -> bool:
    return math.hypot(vehicle_x - robot_x, vehicle_y - robot_y) <= threshold_m
