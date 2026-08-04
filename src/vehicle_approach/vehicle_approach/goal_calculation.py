import math

from geometry_msgs.msg import PoseStamped


def compute_standoff_xy(
    vehicle_x: float,
    vehicle_y: float,
    robot_x: float,
    robot_y: float,
    stand_off_distance_m: float,
) -> tuple[float, float] | None:
    dx = vehicle_x - robot_x
    dy = vehicle_y - robot_y
    distance = math.hypot(dx, dy)
    if distance <= stand_off_distance_m or distance <= 1e-9:
        return None
    return (
        vehicle_x - stand_off_distance_m * dx / distance,
        vehicle_y - stand_off_distance_m * dy / distance,
    )


def compute_goal_pose(
    vehicle_x: float,
    vehicle_y: float,
    robot_x: float,
    robot_y: float,
    stand_off_distance_m: float = 0.0,
) -> PoseStamped | None:
    goal_xy = compute_standoff_xy(
        vehicle_x,
        vehicle_y,
        robot_x,
        robot_y,
        stand_off_distance_m,
    )
    if goal_xy is None:
        return None

    goal_x, goal_y = goal_xy
    yaw = math.atan2(vehicle_y - robot_y, vehicle_x - robot_x)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = goal_x
    pose.pose.position.y = goal_y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def should_resend_goal(
    new_position: tuple[float, float],
    last_sent_position: tuple[float, float],
    threshold_m: float,
) -> bool:
    return math.hypot(
        new_position[0] - last_sent_position[0],
        new_position[1] - last_sent_position[1],
    ) >= threshold_m


def is_approach_complete(
    robot_x: float,
    robot_y: float,
    vehicle_x: float,
    vehicle_y: float,
    threshold_m: float,
) -> bool:
    return math.hypot(vehicle_x - robot_x, vehicle_y - robot_y) <= threshold_m
