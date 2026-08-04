import numpy as np
import rclpy
from std_msgs.msg import Bool

from webcam_perception.webcam_perception_node import WebcamPerceptionNode


class _Box:
    def __init__(self):
        self.xyxy = [[250.0, 170.0, 300.0, 240.0]]
        self.conf = [0.9]
        self.cls = [0]


class _Result:
    def __init__(self, visible=True):
        self.boxes = [_Box()] if visible else []


class _Detector:
    def __init__(self):
        self.visible = True

    def __call__(self, frame, verbose=False):
        return [_Result(self.visible)]


class _Capture:
    def read(self):
        return True, np.zeros((480, 640, 3), dtype=np.uint8)

    def release(self):
        pass


def test_visible_then_missing_publishes_false():
    rclpy.init()
    listener = None
    node = None
    try:
        detector = _Detector()
        node = WebcamPerceptionNode(
            capture=_Capture(),
            detector=detector,
        )
        node.set_parameters([
            rclpy.parameter.Parameter(
                'detect_consecutive_frames',
                value=3,
            ),
            rclpy.parameter.Parameter(
                'lost_consecutive_frames',
                value=2,
            ),
        ])

        received = []
        listener = rclpy.create_node('trigger_listener')
        listener.create_subscription(
            Bool,
            '/webcam/vehicle_detected',
            lambda msg: received.append(msg.data),
            node._trigger_publisher.qos_profile,
        )

        node._on_timer()
        node._on_timer()
        node._on_timer()

        detector.visible = False
        node._on_timer()
        node._on_timer()

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.01)
            rclpy.spin_once(listener, timeout_sec=0.05)

        assert True in received
        true_index = received.index(True)
        assert False in received[true_index + 1:]
    finally:
        if listener is not None:
            listener.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
