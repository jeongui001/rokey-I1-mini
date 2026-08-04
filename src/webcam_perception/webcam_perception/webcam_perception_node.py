import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool

from webcam_perception.detection import Detection, select_best_detection
from webcam_perception.geometry import (
    bbox_bottom_center,
    bbox_center,
    point_in_roi,
)
from webcam_perception.homography import (
    build_homography_matrix,
    pixel_to_map,
)
from webcam_perception.logging_setup import setup_package_logger
from webcam_perception.pipeline import VehicleStopPipeline
from webcam_perception.stop_detector import StopDetector

logger = setup_package_logger('webcam_perception')

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover
    YOLO = None


TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _pairs(flat: list) -> list[tuple[float, float]]:
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]


class WebcamPerceptionNode(Node):
    """외부 웹캠 RC카 검출, 지속 감시, map 가이드 좌표 발행 노드.

    발행 토픽:
    - /webcam/vehicle_detected: RC카의 현재 웹캠 가시 상태(True/False)
    - /webcam/vehicle_map_pose: 현재 RC카의 호모그래피 map 좌표
    - /webcam/vehicle_initial_pose: 기존 정지 판정 기반 좌표(호환용)
    """

    def __init__(self, capture=None, detector=None):
        super().__init__('webcam_perception_node')

        self.declare_parameter('debug_view', False)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('yolo_weights_path', '')
        self.declare_parameter('vehicle_class_id', 0)
        self.declare_parameter('capture_period_s', 0.1)
        self.declare_parameter('roi', [0.0, 0.0, 640.0, 480.0])
        self.declare_parameter('confidence_threshold', 0.5)

        # 검출/유실 디바운스. 10 Hz 기준 3프레임=약 0.3초, 5프레임=약 0.5초.
        self.declare_parameter('detect_consecutive_frames', 3)
        self.declare_parameter('lost_consecutive_frames', 5)
        # 과거 파라미터 호환용. 지속 감시 모드에서는 false를 사용한다.
        self.declare_parameter('trigger_consecutive_frames', 3)
        self.declare_parameter('trigger_latch', False)

        self.declare_parameter(
            'vehicle_visible_topic',
            '/webcam/vehicle_detected',
        )
        self.declare_parameter(
            'live_map_pose_topic',
            '/webcam/vehicle_map_pose',
        )

        # 기존 정지 판정 기반 초기 좌표 출력 호환용 파라미터
        self.declare_parameter('publish_legacy_initial_pose', False)
        self.declare_parameter('stop_duration_s', 2.0)
        self.declare_parameter('stop_pixel_threshold', 5.0)
        self.declare_parameter(
            'homography_pixel_points',
            [0.0, 0.0, 640.0, 0.0, 0.0, 480.0, 640.0, 480.0],
        )
        self.declare_parameter(
            'homography_map_points',
            [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        )

        roi_param = list(self.get_parameter('roi').value)
        self._roi = tuple(float(value) for value in roi_param)
        self._vehicle_class_id = int(
            self.get_parameter('vehicle_class_id').value
        )
        self._confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )

        pixel_points = _pairs(
            list(self.get_parameter('homography_pixel_points').value)
        )
        map_points = _pairs(
            list(self.get_parameter('homography_map_points').value)
        )
        self._homography_matrix = build_homography_matrix(
            pixel_points,
            map_points,
        )
        if self._homography_matrix is None:
            raise ValueError('호모그래피 계산 실패: 대응점을 확인하세요')

        self._pipeline = VehicleStopPipeline(
            roi=self._roi,
            confidence_threshold=self._confidence_threshold,
            homography_matrix=self._homography_matrix,
            stop_detector=StopDetector(
                duration_s=float(
                    self.get_parameter('stop_duration_s').value
                ),
                pixel_threshold=float(
                    self.get_parameter('stop_pixel_threshold').value
                ),
            ),
        )

        # 기존 정지 차량 위치 토픽(호환용)
        self._publisher = self.create_publisher(
            PointStamped,
            '/webcam/vehicle_initial_pose',
            TRANSIENT_LOCAL_QOS,
        )
        # 새 지속 가이드 좌표 토픽
        self._live_pose_publisher = self.create_publisher(
            PointStamped,
            str(self.get_parameter('live_map_pose_topic').value),
            TRANSIENT_LOCAL_QOS,
        )
        # 새 지속 가시 상태 토픽
        self._trigger_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('vehicle_visible_topic').value),
            TRANSIENT_LOCAL_QOS,
        )

        self._visible_count = 0
        self._missing_count = 0
        self._visible_state = False
        # 기존 테스트/외부 코드 호환용 필드명
        self._trigger_count = 0
        self._triggered = False
        self._publish_visibility(False)

        if capture is None:
            capture = cv2.VideoCapture(
                int(self.get_parameter('camera_index').value)
            )
        self._capture = capture
        if (
            hasattr(self._capture, 'isOpened')
            and not self._capture.isOpened()
        ):
            raise RuntimeError(
                '외부 웹캠을 열 수 없습니다. camera_index를 확인하세요.'
            )

        if detector is None:
            if YOLO is None:
                raise ImportError(
                    'ultralytics가 필요합니다: pip install ultralytics'
                )
            weights = os.path.expanduser(
                str(self.get_parameter('yolo_weights_path').value)
            )
            detector = YOLO(weights)
        self._detector = detector

        self._timer = self.create_timer(
            float(self.get_parameter('capture_period_s').value),
            self._on_timer,
        )
        logger.info(
            'webcam continuous monitor ready: '
            '/webcam/vehicle_detected + /webcam/vehicle_map_pose'
        )

    def _on_timer(self) -> None:
        ok, frame = self._capture.read()
        if not ok:
            logger.error('외부 웹캠 프레임 캡처 실패')
            # 카메라 입력 자체가 끊겨도 안전하게 False heartbeat를 만든다.
            self._update_visibility(False)
            self._publish_visibility(self._visible_state)
            return

        try:
            detections = self._run_detector(frame)
        except Exception as ex:
            logger.error(f'웹캠 YOLO 추론 실패: {ex}')
            self._update_visibility(False)
            self._publish_visibility(self._visible_state)
            return

        in_roi = [
            detection
            for detection in detections
            if point_in_roi(
                bbox_center(
                    detection.x1,
                    detection.y1,
                    detection.x2,
                    detection.y2,
                ),
                self._roi,
            )
        ]
        best = select_best_detection(
            in_roi,
            self._confidence_threshold,
        )

        self._update_visibility(best is not None)
        # 매 프레임 heartbeat를 발행해 웹캠 노드 정지 여부도 감시 가능하게 한다.
        self._publish_visibility(self._visible_state)

        # RC카가 현재 보이면 정지 여부와 무관하게 최신 map 가이드 좌표를 발행한다.
        if best is not None and self._visible_state:
            self._publish_live_map_pose(best)

        # 과거 정지 판정 기반 one-shot 토픽은 기본적으로 끈다.
        # 추종에는 아래 legacy 토픽을 사용하지 않으며, 켜더라도
        # /webcam/vehicle_detected와 /webcam/vehicle_map_pose는 계속 발행된다.
        if bool(self.get_parameter('publish_legacy_initial_pose').value):
            now_sec = self.get_clock().now().nanoseconds / 1e9
            map_result = self._pipeline.process_detections(
                detections,
                now_sec,
            )
            if map_result is not None:
                map_x, map_y = map_result
                msg = PointStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x = float(map_x)
                msg.point.y = float(map_y)
                msg.point.z = 0.0
                self._publisher.publish(msg)

        if bool(self.get_parameter('debug_view').value):
            self._show_debug(frame, detections, best)

    def _update_visibility(self, raw_visible: bool) -> None:
        if raw_visible:
            self._visible_count += 1
            self._missing_count = 0
        else:
            self._visible_count = 0
            self._missing_count += 1

        detect_required = max(
            1,
            int(
                self.get_parameter('detect_consecutive_frames').value
                or self.get_parameter('trigger_consecutive_frames').value
            ),
        )
        lost_required = max(
            1,
            int(self.get_parameter('lost_consecutive_frames').value),
        )

        new_state = self._visible_state
        if not self._visible_state and self._visible_count >= detect_required:
            new_state = True
        elif self._visible_state and self._missing_count >= lost_required:
            new_state = False

        self._trigger_count = self._visible_count
        if new_state != self._visible_state:
            self._visible_state = new_state
            self._triggered = new_state
            logger.info(f'webcam vehicle visible={new_state}')

    # 기존 테스트/호출 호환용 이름
    def _update_trigger(self, visible: bool) -> None:
        self._update_visibility(visible)
        self._publish_visibility(self._visible_state)

    def _publish_visibility(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self._trigger_publisher.publish(msg)

    # 기존 테스트/호출 호환용 이름
    def _publish_trigger(self, value: bool) -> None:
        self._publish_visibility(value)

    def _publish_live_map_pose(self, detection: Detection) -> None:
        pixel = bbox_bottom_center(
            detection.x1,
            detection.y1,
            detection.x2,
            detection.y2,
        )
        try:
            map_x, map_y = pixel_to_map(
                self._homography_matrix,
                pixel,
            )
        except Exception as ex:
            logger.warning(f'live homography projection failed: {ex}')
            return

        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(map_x)
        msg.point.y = float(map_y)
        msg.point.z = 0.0
        self._live_pose_publisher.publish(msg)

    def _run_detector(self, frame: np.ndarray) -> list[Detection]:
        if hasattr(self._detector, 'predict'):
            results = self._detector.predict(
                source=frame,
                verbose=False,
                conf=self._confidence_threshold,
                classes=[self._vehicle_class_id],
            )[0]
        else:
            # 기존 단위 테스트용 callable detector 호환
            results = self._detector(frame, verbose=False)[0]

        detections = []
        if results.boxes is None:
            return detections

        for box in results.boxes:
            if int(box.cls[0]) != self._vehicle_class_id:
                continue
            x1, y1, x2, y2 = [float(value) for value in box.xyxy[0]]
            detections.append(
                Detection(
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    confidence=float(box.conf[0]),
                )
            )
        return detections

    def _show_debug(self, frame, detections, best) -> None:
        debug = frame.copy()
        rx1, ry1, rx2, ry2 = [int(value) for value in self._roi]
        cv2.rectangle(debug, (rx1, ry1), (rx2, ry2), (255, 255, 0), 2)

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

        status = (
            'VISIBLE / GUIDE ACTIVE'
            if self._visible_state
            else (
                f'DETECTING {self._visible_count}/'
                f'{int(self.get_parameter("detect_consecutive_frames").value)}'
                if best is not None
                else (
                    f'LOST {self._missing_count}/'
                    f'{int(self.get_parameter("lost_consecutive_frames").value)}'
                    if self._triggered
                    else 'WAITING'
                )
            )
        )
        cv2.putText(
            debug,
            status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
        )
        cv2.imshow('webcam_perception_debug', debug)
        cv2.waitKey(1)

    def destroy_node(self):
        if (
            self._capture is not None
            and hasattr(self._capture, 'release')
        ):
            self._capture.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
