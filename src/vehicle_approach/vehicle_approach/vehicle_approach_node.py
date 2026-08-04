import math
import os

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformException, TransformListener

from vehicle_approach.detection import Detection, select_best_detection
from vehicle_approach.goal_calculation import compute_goal_pose
from vehicle_approach.logging_setup import setup_package_logger
from vehicle_approach.moving_average import MovingAverageFilter
from vehicle_approach.pipeline import VehicleApproachPipeline

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover
    YOLO = None

logger = setup_package_logger('vehicle_approach')


ENABLE_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class VehicleApproachNode(Node):
    """OAK-D 정밀 추종 + 외부 웹캠 가이드 + 웹캠 유실 안전 정지.

    우선순위:
    1. 외부 웹캠에서 RC카가 사라지면 모든 이동 goal 취소
    2. 웹캠과 OAK-D에서 모두 보이면 OAK-D RGB-D 정밀 추종
    3. 웹캠에는 보이지만 OAK-D에서 안 보이면 웹캠 map 좌표로 재탐색 이동
    """

    def __init__(self, detector=None, action_client=None):
        super().__init__('vehicle_approach_node')

        self.declare_parameter('debug_view', True)
        self.declare_parameter('yolo_weights_path', '')
        self.declare_parameter('vehicle_class_id', 0)
        self.declare_parameter('confidence_threshold', 0.75)
        self.declare_parameter(
            'camera_frame',
            'oakd_rgb_camera_optical_frame',
        )
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'nav_action_name',
            '/robot11/navigate_to_pose',
        )

        self.declare_parameter('moving_average_window', 3)
        self.declare_parameter('goal_resend_threshold_m', 0.20)
        self.declare_parameter('goal_update_rate_hz', 2.0)
        self.declare_parameter('desired_follow_distance_m', 0.70)
        self.declare_parameter('distance_deadband_m', 0.08)
        self.declare_parameter('target_timeout_s', 0.80)
        self.declare_parameter('bbox_inner_ratio', 0.60)
        self.declare_parameter('target_min_depth_m', 0.20)
        self.declare_parameter('target_max_depth_m', 4.00)
        self.declare_parameter('apply_depth_correction', True)

        self.declare_parameter(
            'rgb_topic',
            '/robot11/oakd/rgb/image_raw',
        )
        self.declare_parameter(
            'depth_topic',
            '/robot11/oakd/stereo/image_raw',
        )
        self.declare_parameter('use_compressed_depth', True)
        self.declare_parameter(
            'depth_compressed_topic',
            '/robot11/oakd/stereo/image_raw/compressedDepth',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/robot11/oakd/rgb/camera_info',
        )
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop_s', 0.15)

        # 외부 웹캠 지속 감시 및 OAK-D 재탐색 가이드
        self.declare_parameter(
            'webcam_visible_topic',
            '/webcam/vehicle_detected',
        )
        self.declare_parameter(
            'webcam_map_pose_topic',
            '/webcam/vehicle_map_pose',
        )
        self.declare_parameter('webcam_state_timeout_s', 1.0)
        self.declare_parameter('webcam_pose_timeout_s', 0.8)
        self.declare_parameter(
            'oakd_loss_before_webcam_guide_s',
            0.35,
        )
        self.declare_parameter('webcam_guide_standoff_m', 0.90)
        self.declare_parameter(
            'webcam_guide_goal_threshold_m',
            0.20,
        )
        self.declare_parameter('webcam_guide_rate_hz', 1.0)

        self.declare_parameter('publish_obstacle_cloud', True)
        self.declare_parameter(
            'obstacle_cloud_topic',
            '/vehicle_approach/obstacle_points',
        )
        self.declare_parameter('obstacle_cloud_rate_hz', 5.0)
        self.declare_parameter('obstacle_cloud_stride', 10)
        self.declare_parameter('obstacle_min_depth_m', 0.15)
        self.declare_parameter('obstacle_max_depth_m', 3.0)

        self._enabled = False

        self._webcam_visible = False
        self._last_webcam_state_time = None
        self._latest_webcam_pose = None
        self._last_webcam_pose_time = None

        self._last_target_seen_time = None
        self._webcam_guide_active = False
        self._last_webcam_guide_target_xy = None
        self._last_webcam_guide_send_time = None

        self._last_goal_send_time = None
        self._last_cloud_publish_time = None
        self._current_goal_handle = None
        self._goal_send_pending = False
        self._cancel_requested = False
        self._active_goal_source = None

        self.create_subscription(
            Bool,
            '/vehicle_approach/enable',
            self._on_enable,
            ENABLE_QOS,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('webcam_visible_topic').value),
            self._on_webcam_visible,
            ENABLE_QOS,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('webcam_map_pose_topic').value),
            self._on_webcam_map_pose,
            ENABLE_QOS,
        )

        self._detection_center_publisher = self.create_publisher(
            Point,
            '/vehicle_approach/detection_center',
            10,
        )
        self._obstacle_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('obstacle_cloud_topic').value),
            qos_profile_sensor_data,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._bridge = CvBridge()

        self._pipeline = VehicleApproachPipeline(
            confidence_threshold=float(
                self.get_parameter('confidence_threshold').value
            ),
            tf_buffer=self._tf_buffer,
            camera_frame=str(self.get_parameter('camera_frame').value),
            moving_average=MovingAverageFilter(
                window_size=int(
                    self.get_parameter('moving_average_window').value
                )
            ),
            resend_threshold_m=float(
                self.get_parameter('goal_resend_threshold_m').value
            ),
            follow_distance_m=float(
                self.get_parameter('desired_follow_distance_m').value
            ),
            distance_deadband_m=float(
                self.get_parameter('distance_deadband_m').value
            ),
            bbox_inner_ratio=float(
                self.get_parameter('bbox_inner_ratio').value
            ),
            target_min_depth_m=float(
                self.get_parameter('target_min_depth_m').value
            ),
            target_max_depth_m=float(
                self.get_parameter('target_max_depth_m').value
            ),
            apply_depth_correction=bool(
                self.get_parameter('apply_depth_correction').value
            ),
        )

        self._vehicle_class_id = int(
            self.get_parameter('vehicle_class_id').value
        )
        if detector is None:
            if YOLO is None:
                raise ImportError(
                    'ultralytics가 필요합니다: pip install ultralytics'
                )
            detector = YOLO(
                os.path.expanduser(
                    str(self.get_parameter('yolo_weights_path').value)
                )
            )
        self._detector = detector

        self._action_client = action_client or ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('nav_action_name').value),
        )

        rgb_sub = Subscriber(
            self,
            Image,
            str(self.get_parameter('rgb_topic').value),
            qos_profile=qos_profile_sensor_data,
        )
        self._use_compressed_depth = bool(
            self.get_parameter('use_compressed_depth').value
        )
        if self._use_compressed_depth:
            depth_message_type = CompressedImage
            depth_topic = str(
                self.get_parameter('depth_compressed_topic').value
            )
        else:
            depth_message_type = Image
            depth_topic = str(self.get_parameter('depth_topic').value)

        depth_sub = Subscriber(
            self,
            depth_message_type,
            depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        info_sub = Subscriber(
            self,
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            qos_profile=qos_profile_sensor_data,
        )
        self._sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop_s').value),
        )
        self._sync.registerCallback(self._on_synchronized)
        self._watchdog = self.create_timer(0.2, self._target_watchdog)

        logger.info(
            'OAK-D follower ready with continuous webcam guard and guide; '
            f'depth_topic={depth_topic} '
            f'({"compressedDepth" if self._use_compressed_depth else "raw"})'
        )

    # ------------------------------------------------------------------
    # Enable / webcam state
    # ------------------------------------------------------------------
    def _on_enable(self, msg: Bool) -> None:
        if msg.data == self._enabled:
            return

        self._enabled = bool(msg.data)
        self._last_target_seen_time = None
        self._webcam_guide_active = False
        self._last_webcam_guide_target_xy = None
        self._pipeline.reset_tracking()

        if self._enabled:
            logger.info('vehicle approach enabled')
            if not self._webcam_is_fresh_and_visible():
                logger.warning(
                    'approach enabled but RC car is not visible in webcam; '
                    'robot remains stopped'
                )
            elif self._webcam_pose_is_fresh():
                # OAK-D RGB-D 동기화가 아직 시작되지 않았더라도
                # 외부 웹캠 좌표로 즉시 첫 가이드 goal을 시도한다.
                self._send_webcam_guide_if_available()
            else:
                logger.warning(
                    'approach enabled; waiting for fresh webcam map pose'
                )
        else:
            self._stop_all_motion('vehicle approach disabled')

    def _on_webcam_visible(self, msg: Bool) -> None:
        previous = self._webcam_visible
        self._webcam_visible = bool(msg.data)
        self._last_webcam_state_time = self.get_clock().now()

        if self._webcam_visible and not previous:
            logger.info('external webcam RC car visible: motion allowed')
        elif not self._webcam_visible and previous:
            logger.warning(
                'external webcam RC car disappeared: stopping robot'
            )
            if self._enabled:
                self._stop_all_motion('external webcam lost RC car')

    def _on_webcam_map_pose(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != 'map':
            logger.warning(
                f'ignoring webcam guide pose in frame={msg.header.frame_id}'
            )
            return
        self._latest_webcam_pose = msg
        self._last_webcam_pose_time = self.get_clock().now()

    def _webcam_is_fresh_and_visible(self) -> bool:
        if not self._webcam_visible or self._last_webcam_state_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self._last_webcam_state_time
        ).nanoseconds / 1e9
        return elapsed <= float(
            self.get_parameter('webcam_state_timeout_s').value
        )

    def _webcam_pose_is_fresh(self) -> bool:
        if (
            self._latest_webcam_pose is None
            or self._last_webcam_pose_time is None
        ):
            return False
        elapsed = (
            self.get_clock().now() - self._last_webcam_pose_time
        ).nanoseconds / 1e9
        return elapsed <= float(
            self.get_parameter('webcam_pose_timeout_s').value
        )

    # ------------------------------------------------------------------
    # OAK-D synchronized processing
    # ------------------------------------------------------------------
    def _on_synchronized(
        self,
        rgb_msg: Image,
        depth_msg,
        info_msg: CameraInfo,
    ) -> None:
        try:
            depth_m = self._depth_to_meters(depth_msg)
        except Exception as ex:
            logger.error(f'depth conversion failed: {ex}')
            return

        # 장애물 PointCloud는 추종 활성화 전에도 계속 발행한다.
        self._publish_obstacle_cloud(depth_m, depth_msg, info_msg)
        if not self._enabled:
            return

        # 외부 웹캠이 RC카를 놓치면 OAK-D 검출 여부와 무관하게 정지한다.
        if not self._webcam_is_fresh_and_visible():
            self._stop_all_motion('webcam state false or stale')
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding='bgr8',
            )
            original_detections = self._run_detector(frame)
        except Exception as ex:
            logger.error(f'YOLO/RGB processing failed: {ex}')
            return

        scaled_detections = self._scale_detections(
            original_detections,
            frame.shape[:2],
            depth_m.shape,
        )
        intrinsics = self._scaled_intrinsics(info_msg, depth_m.shape)
        if intrinsics is None:
            logger.error('invalid camera intrinsics')
            return
        fx, fy, cx, cy = intrinsics

        configured_frame = str(self.get_parameter('camera_frame').value)
        self._pipeline.camera_frame = (
            configured_frame or depth_msg.header.frame_id
        )

        try:
            robot_x, robot_y = self._lookup_robot_position()
            result = self._pipeline.process_frame(
                scaled_detections,
                depth_m,
                fx,
                fy,
                cx,
                cy,
                stamp=depth_msg.header.stamp,
                robot_x=robot_x,
                robot_y=robot_y,
            )
        except TransformException as ex:
            logger.warning(f'tf lookup failed: {ex}')
            return

        mode = 'OAK-D LOST'
        if result.target_visible:
            self._last_target_seen_time = self.get_clock().now()
            self._webcam_guide_active = False
            self._last_webcam_guide_target_xy = None
            mode = 'OAK-D FOLLOW'

            if result.detection_center is not None:
                self._detection_center_publisher.publish(
                    result.detection_center
                )

            if result.completed:
                self._cancel_current_goal(
                    reason='desired distance reached',
                    reset_pipeline=False,
                )
                mode = 'STOP / HOLD'
            elif result.goal_pose is not None:
                sent = self._send_goal_if_due(
                    result.goal_pose,
                    source='oakd',
                    rate_hz=float(
                        self.get_parameter('goal_update_rate_hz').value
                    ),
                )
                # Pipeline은 goal 후보를 만들 때 내부 last-goal을 먼저 갱신한다.
                # 실제 전송이 지연되면 다음 프레임에서 재시도할 수 있게 되돌린다.
                if not sent:
                    self._pipeline._last_sent_goal = None
        else:
            # OAK-D가 잠깐 놓친 경우 즉시 흔들리지 않도록 짧게 기다렸다가
            # 외부 웹캠 map 좌표 가이드로 전환한다.
            if self._oakd_target_lost_long_enough():
                if self._send_webcam_guide_if_available():
                    mode = 'WEBCAM GUIDE'
                else:
                    self._stop_all_motion(
                        'OAK-D target lost and webcam guide unavailable'
                    )
                    mode = 'STOP / NO GUIDE'

        if bool(self.get_parameter('debug_view').value):
            self._show_debug(
                frame,
                original_detections,
                result,
                mode,
            )

    def _oakd_target_lost_long_enough(self) -> bool:
        if self._last_target_seen_time is None:
            return True
        elapsed = (
            self.get_clock().now() - self._last_target_seen_time
        ).nanoseconds / 1e9
        return elapsed >= float(
            self.get_parameter(
                'oakd_loss_before_webcam_guide_s'
            ).value
        )

    # ------------------------------------------------------------------
    # Webcam fallback guidance
    # ------------------------------------------------------------------
    def _send_webcam_guide_if_available(self) -> bool:
        if not self._webcam_is_fresh_and_visible():
            return False
        if not self._webcam_pose_is_fresh():
            return False

        try:
            robot_x, robot_y = self._lookup_robot_position()
        except TransformException as ex:
            logger.warning(f'guide robot TF unavailable: {ex}')
            return False

        target_x = float(self._latest_webcam_pose.point.x)
        target_y = float(self._latest_webcam_pose.point.y)
        target_xy = (target_x, target_y)

        threshold = float(
            self.get_parameter('webcam_guide_goal_threshold_m').value
        )
        if self._last_webcam_guide_target_xy is not None:
            moved = math.hypot(
                target_x - self._last_webcam_guide_target_xy[0],
                target_y - self._last_webcam_guide_target_xy[1],
            )
            # 같은 가이드 목표라도 현재 goal이 사라진 경우에는 다시 보낸다.
            if moved < threshold and (
                self._current_goal_handle is not None
                or self._goal_send_pending
            ):
                return True

        stand_off = float(
            self.get_parameter('webcam_guide_standoff_m').value
        )
        goal_pose = compute_goal_pose(
            target_x,
            target_y,
            robot_x,
            robot_y,
            stand_off_distance_m=stand_off,
        )

        # 이미 stand-off 안쪽인데 OAK-D가 RC카를 못 본다면 제자리에서
        # 웹캠 좌표 방향을 바라보게 하여 OAK-D 재탐지를 유도한다.
        if goal_pose is None:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = robot_x
            goal_pose.pose.position.y = robot_y
            yaw = math.atan2(target_y - robot_y, target_x - robot_x)
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._webcam_guide_active:
            self._pipeline.reset_tracking()
            self._webcam_guide_active = True
            logger.warning(
                'OAK-D cannot see RC car; switching to webcam map guidance'
            )

        sent = self._send_goal_if_due(
            goal_pose,
            source='webcam',
            rate_hz=float(
                self.get_parameter('webcam_guide_rate_hz').value
            ),
        )
        if sent:
            self._last_webcam_guide_target_xy = target_xy
            self._last_webcam_guide_send_time = self.get_clock().now()
        return True

    # ------------------------------------------------------------------
    # Image / depth helpers
    # ------------------------------------------------------------------
    def _depth_to_meters(self, msg) -> np.ndarray:
        if isinstance(msg, CompressedImage):
            return self._compressed_depth_to_meters(msg)

        image = np.asarray(
            self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough',
            )
        )
        encoding = msg.encoding.upper()
        if encoding in ('16UC1', 'MONO16') or image.dtype == np.uint16:
            return image.astype(np.float32) * 0.001
        if encoding == '32FC1' or image.dtype in (
            np.float32,
            np.float64,
        ):
            return image.astype(np.float32)
        raise ValueError(
            f'unsupported depth encoding={msg.encoding}, '
            f'dtype={image.dtype}'
        )

    @staticmethod
    def _compressed_depth_to_meters(msg: CompressedImage) -> np.ndarray:
        """ROS compressedDepth의 16UC1 PNG를 meter 배열로 변환한다.

        compressedDepth 메시지 앞에는 transport 헤더가 붙을 수 있으므로
        PNG signature 위치를 찾아 그 이후만 OpenCV로 디코딩한다.
        현재 OAK-D 토픽에서 확인된 형식은 16UC1; compressedDepth이다.
        """
        payload = bytes(msg.data)
        png_signature = b'\x89PNG\r\n\x1a\n'
        png_offset = payload.find(png_signature)
        if png_offset < 0:
            raise ValueError(
                'compressedDepth payload does not contain a PNG signature'
            )

        encoded = np.frombuffer(payload[png_offset:], dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        if image is None:
            raise ValueError('failed to decode compressedDepth PNG')

        fmt = str(msg.format).upper()
        if '16UC1' in fmt or image.dtype == np.uint16:
            return image.astype(np.float32) * 0.001

        raise ValueError(
            f'unsupported compressedDepth format={msg.format}, '
            f'dtype={image.dtype}; expected 16UC1'
        )

    def _run_detector(self, frame) -> list[Detection]:
        if hasattr(self._detector, 'predict'):
            results = self._detector.predict(
                source=frame,
                verbose=False,
                conf=float(
                    self.get_parameter('confidence_threshold').value
                ),
                classes=[self._vehicle_class_id],
            )[0]
        else:
            results = self._detector(frame, verbose=False)[0]

        detections = []
        if results.boxes is None:
            return detections
        for box in results.boxes:
            if int(box.cls[0]) != self._vehicle_class_id:
                continue
            x1, y1, x2, y2 = [float(value) for value in box.xyxy[0]]
            detections.append(
                Detection(x1, y1, x2, y2, float(box.conf[0]))
            )
        return detections

    @staticmethod
    def _scale_detections(detections, rgb_shape, depth_shape):
        rgb_h, rgb_w = rgb_shape
        depth_h, depth_w = depth_shape
        sx = depth_w / float(rgb_w)
        sy = depth_h / float(rgb_h)
        return [
            Detection(
                detection.x1 * sx,
                detection.y1 * sy,
                detection.x2 * sx,
                detection.y2 * sy,
                detection.confidence,
            )
            for detection in detections
        ]

    @staticmethod
    def _scaled_intrinsics(info: CameraInfo, image_shape):
        height, width = image_shape
        source_w = int(info.width) if info.width > 0 else width
        source_h = int(info.height) if info.height > 0 else height
        sx = width / float(source_w)
        sy = height / float(source_h)
        fx = float(info.k[0]) * sx
        fy = float(info.k[4]) * sy
        cx = float(info.k[2]) * sx
        cy = float(info.k[5]) * sy
        return None if fx <= 0.0 or fy <= 0.0 else (fx, fy, cx, cy)

    def _lookup_robot_position(self) -> tuple[float, float]:
        transform = self._tf_buffer.lookup_transform(
            'map',
            str(self.get_parameter('base_frame').value),
            Time(),
            timeout=Duration(seconds=0.2),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    # ------------------------------------------------------------------
    # Nav2 goal management
    # ------------------------------------------------------------------
    def _send_goal_if_due(
        self,
        goal_pose: PoseStamped,
        source: str,
        rate_hz: float,
    ) -> bool:
        now = self.get_clock().now()
        rate_hz = max(0.1, float(rate_hz))
        if self._last_goal_send_time is not None:
            elapsed = (
                now - self._last_goal_send_time
            ).nanoseconds / 1e9
            if elapsed < 1.0 / rate_hz:
                return False

        if self._goal_send_pending or not self._action_client.server_is_ready():
            return False
        if not self._webcam_is_fresh_and_visible():
            return False

        # 새 목표가 오면 이전 목표를 취소한 뒤 재전송한다.
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.pose.header.stamp = now.to_msg()

        self._goal_send_pending = True
        self._cancel_requested = False
        self._last_goal_send_time = now
        self._active_goal_source = source

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        logger.info(
            f'{source} goal: '
            f'x={goal_pose.pose.position.x:.3f}, '
            f'y={goal_pose.pose.position.y:.3f}'
        )
        return True

    def _on_goal_response(self, future) -> None:
        self._goal_send_pending = False
        try:
            handle = future.result()
        except Exception as ex:
            logger.warning(f'goal response error: {ex}')
            self._pipeline._last_sent_goal = None
            return

        if not handle.accepted:
            logger.warning('follow/guide goal rejected')
            self._pipeline._last_sent_goal = None
            return

        if self._cancel_requested or not self._webcam_is_fresh_and_visible():
            handle.cancel_goal_async()
            logger.warning('goal accepted after stop request; cancelling')
            return

        self._current_goal_handle = handle
        if hasattr(handle, 'get_result_async'):
            handle.get_result_async().add_done_callback(
                lambda done, accepted_handle=handle: self._on_goal_result(
                    done,
                    accepted_handle,
                )
            )

    def _on_goal_result(self, future, handle) -> None:
        try:
            result = future.result()
            logger.info(
                f'{self._active_goal_source} goal result '
                f'status={result.status}'
            )
        except Exception as ex:
            logger.warning(f'goal result error: {ex}')
        if self._current_goal_handle is handle:
            self._current_goal_handle = None

    def _cancel_current_goal(
        self,
        reason: str = '',
        reset_pipeline: bool = False,
    ) -> None:
        self._cancel_requested = True
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
            logger.info(
                f'goal cancelled{": " + reason if reason else ""}'
            )
        if reset_pipeline:
            self._pipeline.reset_tracking()
        self._active_goal_source = None

    def _stop_all_motion(self, reason: str) -> None:
        had_motion = (
            self._current_goal_handle is not None
            or self._goal_send_pending
            or self._active_goal_source is not None
        )
        self._cancel_current_goal(reason=reason, reset_pipeline=True)
        self._last_target_seen_time = None
        self._webcam_guide_active = False
        self._last_webcam_guide_target_xy = None
        if had_motion:
            logger.warning(f'robot stopped: {reason}')

    def _target_watchdog(self) -> None:
        if not self._enabled:
            return

        if not self._webcam_is_fresh_and_visible():
            self._stop_all_motion('webcam RC car lost or heartbeat stale')
            return

        # OAK-D가 유실됐지만 웹캠 좌표가 살아 있으면 재탐색 가이드.
        if self._oakd_target_lost_long_enough():
            if self._send_webcam_guide_if_available():
                return
            self._stop_all_motion(
                'OAK-D target lost and webcam map pose stale'
            )

    # ------------------------------------------------------------------
    # Depth -> obstacle PointCloud2
    # ------------------------------------------------------------------
    def _publish_obstacle_cloud(
        self,
        depth_m: np.ndarray,
        depth_msg,
        info_msg: CameraInfo,
    ) -> None:
        if not bool(self.get_parameter('publish_obstacle_cloud').value):
            return

        now = self.get_clock().now()
        rate_hz = max(
            0.1,
            float(self.get_parameter('obstacle_cloud_rate_hz').value),
        )
        if self._last_cloud_publish_time is not None:
            elapsed = (
                now - self._last_cloud_publish_time
            ).nanoseconds / 1e9
            if elapsed < 1.0 / rate_hz:
                return

        intrinsics = self._scaled_intrinsics(info_msg, depth_m.shape)
        if intrinsics is None:
            return
        fx, fy, cx, cy = intrinsics
        stride = max(
            1,
            int(self.get_parameter('obstacle_cloud_stride').value),
        )
        minimum = float(
            self.get_parameter('obstacle_min_depth_m').value
        )
        maximum = float(
            self.get_parameter('obstacle_max_depth_m').value
        )

        height, width = depth_m.shape
        cols = np.arange(0, width, stride, dtype=np.int32)
        rows = np.arange(0, height, stride, dtype=np.int32)
        uu, vv = np.meshgrid(cols, rows)
        zz = depth_m[vv, uu]
        mask = (
            np.isfinite(zz)
            & (zz >= minimum)
            & (zz <= maximum)
        )
        if not np.any(mask):
            return

        z = zz[mask].astype(np.float32)
        x = (uu[mask].astype(np.float32) - cx) * z / fx
        y = (vv[mask].astype(np.float32) - cy) * z / fy
        points = np.column_stack((x, y, z))

        header = Header()
        header.stamp = depth_msg.header.stamp
        header.frame_id = depth_msg.header.frame_id or str(
            self.get_parameter('camera_frame').value
        )
        cloud = point_cloud2.create_cloud_xyz32(
            header,
            points.tolist(),
        )
        self._obstacle_cloud_publisher.publish(cloud)
        self._last_cloud_publish_time = now

    # ------------------------------------------------------------------
    # Debug / cleanup
    # ------------------------------------------------------------------
    def _show_debug(self, frame, detections, result, mode: str) -> None:
        debug = frame.copy()
        best = select_best_detection(
            detections,
            self._pipeline.confidence_threshold,
        )
        for detection in detections:
            x1, y1, x2, y2 = map(
                int,
                [
                    detection.x1,
                    detection.y1,
                    detection.x2,
                    detection.y2,
                ],
            )
            color = (0, 255, 0) if detection is best else (0, 0, 255)
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                debug,
                f'{detection.confidence:.2f}',
                (x1, max(y1 - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

        distance = (
            f' {result.target_distance_m:.2f}m'
            if result.target_distance_m is not None
            else ''
        )
        cv2.putText(
            debug,
            mode + distance,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
        )
        cv2.imshow('vehicle_approach_debug', debug)
        cv2.waitKey(1)

    def destroy_node(self):
        self._cancel_current_goal(
            reason='node shutdown',
            reset_pipeline=True,
        )
        cv2.destroyAllWindows()
        if hasattr(self._action_client, 'destroy'):
            self._action_client.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleApproachNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
