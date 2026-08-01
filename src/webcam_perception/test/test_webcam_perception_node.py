import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped

from webcam_perception.webcam_perception_node import WebcamPerceptionNode


class _FakeBox:
    def __init__(self, x1, y1, x2, y2, conf, cls_id):
        self.xyxy = [[x1, y1, x2, y2]]
        self.conf = [conf]
        self.cls = [cls_id]


class _FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


class _FakeDetector:
    def __init__(self, boxes):
        self._boxes = boxes

    def __call__(self, frame, verbose=False):
        return [_FakeResult(self._boxes)]


class _FakeCapture:
    def __init__(self, frame):
        self._frame = frame

    def read(self):
        return True, self._frame


def test_stopped_vehicle_publishes_map_pose():
    rclpy.init()
    try:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        boxes = [_FakeBox(100.0, 100.0, 140.0, 180.0, 0.9, 0)]
        node = WebcamPerceptionNode(capture=_FakeCapture(frame), detector=_FakeDetector(boxes))
        # 정지 판정 지속시간을 0으로 낮춰 단일 프레임으로도 즉시 정지 판정이 나오게 함
        # (StopDetector 자체의 시간 로직은 Task 2에서 이미 검증했으므로 여기서는
        #  노드 배선: capture -> detector -> pipeline -> publish 만 확인한다)
        node._pipeline.stop_detector.duration_s = 0.0

        listener = rclpy.create_node('test_listener')
        received: list[PointStamped] = []
        listener.create_subscription(
            PointStamped,
            '/webcam/vehicle_initial_pose',
            lambda msg: received.append(msg),
            node._publisher.qos_profile,
        )

        node._on_timer()
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(listener, timeout_sec=0.05)
            if received:
                break

        assert len(received) == 1
        assert received[0].header.frame_id == 'map'
    finally:
        rclpy.shutdown()
