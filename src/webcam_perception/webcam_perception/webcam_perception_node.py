import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile

from webcam_perception.detection import Detection
from webcam_perception.homography import build_homography_matrix
from webcam_perception.pipeline import VehicleStopPipeline
from webcam_perception.stop_detector import StopDetector

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - 배포 환경에서만 필요
    YOLO = None


VEHICLE_POSE_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _pairs(flat: list) -> list[tuple[float, float]]:
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]


class WebcamPerceptionNode(Node):
    def __init__(self, capture=None, detector=None):
        super().__init__('webcam_perception_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('yolo_weights_path', '')
        self.declare_parameter('vehicle_class_id', 0)
        self.declare_parameter('capture_period_s', 0.1)
        self.declare_parameter('roi', [0.0, 0.0, 640.0, 480.0])
        self.declare_parameter('stop_duration_s', 2.0)
        self.declare_parameter('stop_pixel_threshold', 5.0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter(
            'homography_pixel_points', [0.0, 0.0, 640.0, 0.0, 0.0, 480.0, 640.0, 480.0]
        )
        self.declare_parameter(
            'homography_map_points', [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0]
        )

        roi_param = list(self.get_parameter('roi').value)
        roi = (roi_param[0], roi_param[1], roi_param[2], roi_param[3])
        self._vehicle_class_id = self.get_parameter('vehicle_class_id').value

        pixel_points = _pairs(list(self.get_parameter('homography_pixel_points').value))
        map_points = _pairs(list(self.get_parameter('homography_map_points').value))
        homography_matrix = build_homography_matrix(pixel_points, map_points)
        if homography_matrix is None:
            raise ValueError('호모그래피 계산 실패 — 캘리브레이션 대응점을 확인하세요')

        stop_detector = StopDetector(
            duration_s=self.get_parameter('stop_duration_s').value,
            pixel_threshold=self.get_parameter('stop_pixel_threshold').value,
        )
        self._pipeline = VehicleStopPipeline(
            roi=roi,
            confidence_threshold=self.get_parameter('confidence_threshold').value,
            homography_matrix=homography_matrix,
            stop_detector=stop_detector,
        )

        self._publisher = self.create_publisher(
            PointStamped, '/webcam/vehicle_initial_pose', VEHICLE_POSE_QOS
        )

        if capture is None:
            capture = cv2.VideoCapture(self.get_parameter('camera_index').value)
        self._capture = capture

        if detector is None:
            if YOLO is None:
                raise ImportError('ultralytics가 필요합니다 — pip install ultralytics')
            detector = YOLO(os.path.expanduser(self.get_parameter('yolo_weights_path').value))
        self._detector = detector

        period = self.get_parameter('capture_period_s').value
        self._timer = self.create_timer(period, self._on_timer)

    def _on_timer(self) -> None:
        ok, frame = self._capture.read()
        if not ok:
            return

        detections = self._run_detector(frame)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        result = self._pipeline.process_detections(detections, now_sec)
        if result is None:
            return

        map_x, map_y = result
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = map_x
        msg.point.y = map_y
        msg.point.z = 0.0
        self._publisher.publish(msg)

    def _run_detector(self, frame: np.ndarray) -> list[Detection]:
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


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
