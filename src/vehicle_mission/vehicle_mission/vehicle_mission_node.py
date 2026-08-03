import rclpy
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import Bool

from vehicle_mission.logging_setup import setup_package_logger
from vehicle_mission.nav_result import handle_nav_result
from vehicle_mission.pose_utils import waypoint_to_pose_stamped

logger = setup_package_logger('vehicle_mission')

TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class VehicleMissionNode(Node):
    def __init__(self, action_client=None):
        super().__init__('vehicle_mission_node')

        self.declare_parameter('waypoint_x', 0.0)
        self.declare_parameter('waypoint_y', 0.0)
        self.declare_parameter('waypoint_yaw', 0.0)

        self._last_webcam_pose: PointStamped | None = None
        self.create_subscription(
            PointStamped,
            '/webcam/vehicle_initial_pose',
            self._on_webcam_pose,
            TRANSIENT_LOCAL_QOS,
        )

        self._enable_publisher = self.create_publisher(
            Bool, '/vehicle_approach/enable', TRANSIENT_LOCAL_QOS
        )

        self._action_client = action_client or ActionClient(
            self, NavigateToPose, '/robot11/navigate_to_pose'
        )

    def _on_webcam_pose(self, msg: PointStamped) -> None:
        # 로깅/모니터링용 보관만 한다 — 이동 목표 계산에는 절대 사용하지 않는다 (스펙 §1.3)
        self._last_webcam_pose = msg
        logger.info(
            f'webcam initial pose (logging only): x={msg.point.x:.3f}, y={msg.point.y:.3f}'
        )

    def send_waypoint_goal(self) -> None:
        x = self.get_parameter('waypoint_x').value
        y = self.get_parameter('waypoint_y').value
        yaw = self.get_parameter('waypoint_yaw').value
        pose = waypoint_to_pose_stamped(x, y, yaw)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        logger.info(
            f'sending waypoint goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )
        logger.info('waiting for Nav2 navigate_to_pose action server...')
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.error('waypoint goal rejected by Nav2')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        if handle_nav_result(result.status):
            msg = Bool()
            msg.data = True
            self._enable_publisher.publish(msg)
            logger.info('waypoint reached, vehicle_approach enabled')
        else:
            logger.warning(
                f'nav goal finished with status={result.status}, approach not enabled'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VehicleMissionNode()
    try:
        node.send_waypoint_goal()
        rclpy.spin(node)
    finally:
        node._action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()
