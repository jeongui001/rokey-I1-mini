import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from vehicle_approach.goal_calculation import compute_goal_pose, should_resend_goal


TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class VehicleApproachNode(Node):
    def __init__(self, action_client=None):
        super().__init__('vehicle_approach_node')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('nav_action_name', '/robot11/navigate_to_pose')
        self.declare_parameter('depth_topic', '/robot11/oakd/stereo/image_raw')
        self.declare_parameter('goal_resend_threshold_m', 0.1)
        self.declare_parameter('stop_depth_m', 0.7)

        self._enabled = False
        self._latest_vehicle_pose: PointStamped | None = None
        self._last_sent_goal: tuple[float, float] | None = None
        self._current_goal_handle = None

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._action_client = action_client or ActionClient(
            self, NavigateToPose, self.get_parameter('nav_action_name').value
        )

        self.create_subscription(
            Bool,
            '/vehicle_approach/enable',
            self._on_enable,
            TRANSIENT_LOCAL_QOS,
        )
        self.create_subscription(
            PointStamped,
            '/webcam/vehicle_initial_pose',
            self._on_webcam_pose,
            TRANSIENT_LOCAL_QOS,
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self._on_depth,
            10,
        )

    def _on_enable(self, msg: Bool) -> None:
        self._enabled = msg.data
        if not self._enabled:
            self._cancel_current_goal()

    def _on_webcam_pose(self, msg: PointStamped) -> None:
        self._latest_vehicle_pose = msg
        self.get_logger().info(
            f'received webcam pose: x={msg.point.x:.3f}, y={msg.point.y:.3f}'
        )

    def _on_depth(self, depth_msg: Image) -> None:
        if not self._enabled:
            return
        if self._latest_vehicle_pose is None:
            return

        depth_image = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # 화면 중앙 하단 영역만 충돌 방지 ROI로 사용한다.
        h, w = depth_image.shape
        roi = depth_image[h // 3 : h, w // 3 : 2 * w // 3]
        valid_depth = roi[roi > 0]

        if valid_depth.size == 0:
            self.get_logger().warn('no valid depth in forward ROI')
            return

        min_depth_m = float(valid_depth.min()) / 1000.0
        stop_depth_m = self.get_parameter('stop_depth_m').value

        if min_depth_m <= stop_depth_m:
            self.get_logger().info(
                f'stopping: min_depth={min_depth_m:.3f}m <= {stop_depth_m:.3f}m'
            )
            self._cancel_current_goal()
            return

        self._send_goal_to_latest_vehicle_pose()

    def _lookup_robot_position(self) -> tuple[float, float]:
        base_frame = self.get_parameter('base_frame').value
        transform = self._tf_buffer.lookup_transform('map', base_frame, rclpy.time.Time())
        return (transform.transform.translation.x, transform.transform.translation.y)

    def _send_goal_to_latest_vehicle_pose(self) -> None:
        if self._latest_vehicle_pose is None:
            return

        target_x = self._latest_vehicle_pose.point.x
        target_y = self._latest_vehicle_pose.point.y
        target = (target_x, target_y)

        resend_threshold_m = self.get_parameter('goal_resend_threshold_m').value
        if self._last_sent_goal is not None and not should_resend_goal(
            target, self._last_sent_goal, resend_threshold_m
        ):
            return

        if not self._action_client.server_is_ready():
            self.get_logger().warn('Nav2 action server not ready')
            return

        robot_x, robot_y = self._lookup_robot_position()
        pose = compute_goal_pose(target_x, target_y, robot_x, robot_y)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f'sending external-camera goal: x={target_x:.3f}, y={target_y:.3f}'
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._last_sent_goal = target

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('goal rejected by Nav2')
            return
        self._current_goal_handle = goal_handle

    def _cancel_current_goal(self) -> None:
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
            self.get_logger().info('goal cancelled')

    def destroy_node(self):
        self._cancel_current_goal()
        self._action_client.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleApproachNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
