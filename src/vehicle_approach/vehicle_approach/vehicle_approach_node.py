import rclpy
import tf2_geometry_msgs  # noqa: F401
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from vehicle_approach.detection import Detection
from vehicle_approach.moving_average import MovingAverageFilter
from vehicle_approach.pipeline import VehicleApproachPipeline

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - 배포 환경에서만 필요
    YOLO = None


ENABLE_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class VehicleApproachNode(Node):
    def __init__(self, detector=None, action_client=None):
        super().__init__('vehicle_approach_node')

        self.declare_parameter('yolo_weights_path', '')
        self.declare_parameter('vehicle_class_id', 0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('moving_average_window', 5)
        self.declare_parameter('goal_resend_threshold_m', 0.1)
        self.declare_parameter('approach_completion_threshold_m', 0.5)
        self.declare_parameter('rgb_topic', '/oakd/rgb/image_raw')
        self.declare_parameter('depth_topic', '/oakd/stereo/image_raw')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/camera_info')

        self._enabled = False
        self.create_subscription(Bool, '/vehicle_approach/enable', self._on_enable, ENABLE_QOS)

        self._detection_center_publisher = self.create_publisher(
            Point, '/vehicle_approach/detection_center', 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pipeline = VehicleApproachPipeline(
            confidence_threshold=self.get_parameter('confidence_threshold').value,
            tf_buffer=self._tf_buffer,
            camera_frame=self.get_parameter('camera_frame').value,
            moving_average=MovingAverageFilter(
                window_size=self.get_parameter('moving_average_window').value
            ),
            resend_threshold_m=self.get_parameter('goal_resend_threshold_m').value,
            completion_threshold_m=self.get_parameter('approach_completion_threshold_m').value,
        )

        self._vehicle_class_id = self.get_parameter('vehicle_class_id').value
        if detector is None:
            detector = YOLO(self.get_parameter('yolo_weights_path').value)
        self._detector = detector
        self._bridge = CvBridge()

        self._action_client = action_client or ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self._current_goal_handle = None

        rgb_sub = Subscriber(self, Image, self.get_parameter('rgb_topic').value)
        depth_sub = Subscriber(self, Image, self.get_parameter('depth_topic').value)
        info_sub = Subscriber(self, CameraInfo, self.get_parameter('camera_info_topic').value)
        self._sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.15
        )
        self._sync.registerCallback(self._on_synchronized)

    def _on_enable(self, msg: Bool) -> None:
        if msg.data and not self._enabled:
            self.get_logger().info('vehicle_approach enabled')
        elif not msg.data and self._enabled:
            self._pipeline.moving_average.reset()
        self._enabled = msg.data

    def _on_synchronized(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        # 오케스트레이션만 담당한다 -- 뎁스보정/역투영+TF/이동평균/goal계산은
        # VehicleApproachPipeline과 그 내부 함수들이 각각 수행한다 (스펙 §5.2.1)
        if not self._enabled:
            return

        detections = self._run_detector(rgb_msg)
        depth_image = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        fx, fy, cx, cy = info_msg.k[0], info_msg.k[4], info_msg.k[2], info_msg.k[5]
        robot_x, robot_y = self._lookup_robot_position()

        result = self._pipeline.process_frame(
            detections, depth_image, fx, fy, cx, cy,
            stamp=rgb_msg.header.stamp,
            robot_x=robot_x, robot_y=robot_y,
        )

        if result.detection_center is not None:
            self._detection_center_publisher.publish(result.detection_center)

        if result.completed:
            self.get_logger().info('vehicle approach completed')
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle = None
                self.get_logger().info('goal cancelled')
            return

        if result.goal_pose is not None:
            self._send_goal(result.goal_pose)

    def _run_detector(self, rgb_msg: Image) -> list[Detection]:
        frame = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        results = self._detector(frame, verbose=False)[0]
        detections = []
        for box in results.boxes:
            if int(box.cls[0]) != self._vehicle_class_id:
                continue
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            detections.append(
                Detection(x1=x1, y1=y1, x2=x2, y2=y2, confidence=float(box.conf[0]))
            )
        return detections

    def _lookup_robot_position(self) -> tuple[float, float]:
        base_frame = self.get_parameter('base_frame').value
        transform = self._tf_buffer.lookup_transform('map', base_frame, rclpy.time.Time())
        return (transform.transform.translation.x, transform.transform.translation.y)

    def _send_goal(self, goal_pose) -> None:
        if not self._action_client.server_is_ready():
            self.get_logger().warn('navigate_to_pose server not ready, skipping goal send')
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.get_logger().info(
            f'sending goal: x={goal_pose.pose.position.x:.3f}, y={goal_pose.pose.position.y:.3f}'
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('goal rejected by Nav2')
            return
        self._current_goal_handle = goal_handle


def main(args=None):
    rclpy.init(args=args)
    node = VehicleApproachNode()
    try:
        rclpy.spin(node)
    finally:
        node._action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()
