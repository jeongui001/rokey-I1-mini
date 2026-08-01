import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo

from vehicle_approach.vehicle_approach_node import VehicleApproachNode


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


class _FakeGoalHandle:
    def __init__(self):
        self.accepted = True
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        future = rclpy.task.Future()
        future.set_result(None)
        return future


class _FakeActionClient:
    def __init__(self):
        self.sent_goals = []
        self.goal_handles = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        goal_handle = _FakeGoalHandle()
        self.goal_handles.append(goal_handle)
        future = rclpy.task.Future()
        future.set_result(goal_handle)
        return future


def _seed_tf(node: VehicleApproachNode) -> None:
    base_to_map = TransformStamped()
    base_to_map.header.frame_id = 'map'
    base_to_map.child_frame_id = 'base_link'
    base_to_map.transform.translation.x = 2.0
    base_to_map.transform.rotation.w = 1.0
    node._tf_buffer.set_transform_static(base_to_map, 'test')

    camera_to_base = TransformStamped()
    camera_to_base.header.frame_id = 'base_link'
    camera_to_base.child_frame_id = 'camera_frame'
    camera_to_base.transform.rotation.y = 0.70710678
    camera_to_base.transform.rotation.w = 0.70710678
    node._tf_buffer.set_transform_static(camera_to_base, 'test')


def _make_synced_messages():
    bridge = CvBridge()
    rgb = bridge.cv2_to_imgmsg(np.zeros((480, 640, 3), dtype=np.uint8), encoding='bgr8')
    depth = bridge.cv2_to_imgmsg(
        np.full((480, 640), 2000, dtype=np.uint16), encoding='passthrough'
    )
    info = CameraInfo()
    info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
    return rgb, depth, info


def test_enabled_pipeline_publishes_detection_center_and_sends_goal():
    rclpy.init()
    try:
        boxes = [_FakeBox(300.0, 220.0, 340.0, 260.0, 0.9, 0)]
        fake_client = _FakeActionClient()
        node = VehicleApproachNode(detector=_FakeDetector(boxes), action_client=fake_client)
        node._enabled = True
        _seed_tf(node)

        listener = rclpy.create_node('test_listener')
        received: list[Point] = []
        listener.create_subscription(
            Point, '/vehicle_approach/detection_center', lambda msg: received.append(msg), 10
        )

        rgb, depth, info = _make_synced_messages()
        node._on_synchronized(rgb, depth, info)

        for _ in range(10):
            rclpy.spin_once(listener, timeout_sec=0.05)
            if received:
                break

        assert len(received) == 1
        assert len(fake_client.sent_goals) == 1
    finally:
        rclpy.shutdown()


def test_completion_cancels_goal_exactly_once():
    # approach_completion_threshold_m을 낮은 실측 보정거리(raw 0.6m -> 0.608m)보다 살짝 높게 오버라이드하여
    # 1프레임(멀리, 보정거리 1.721m)에서는 미완료, 2프레임(가까이, 보정거리 0.608m)에서만 완료되도록 한다.
    # moving_average_window=1로 오버라이드해 프레임 간 이동평균 희석을 없앤다.
    rclpy.init(
        args=[
            '--ros-args',
            '-p', 'approach_completion_threshold_m:=0.65',
            '-p', 'moving_average_window:=1',
        ]
    )
    try:
        boxes = [_FakeBox(300.0, 220.0, 340.0, 260.0, 0.9, 0)]
        fake_client = _FakeActionClient()
        node = VehicleApproachNode(detector=_FakeDetector(boxes), action_client=fake_client)
        node._enabled = True
        _seed_tf(node)

        bridge = CvBridge()
        rgb = bridge.cv2_to_imgmsg(np.zeros((480, 640, 3), dtype=np.uint8), encoding='bgr8')
        info = CameraInfo()
        info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]

        far_depth = bridge.cv2_to_imgmsg(
            np.full((480, 640), 2000, dtype=np.uint16), encoding='passthrough'
        )
        node._on_synchronized(rgb, far_depth, info)

        assert len(fake_client.sent_goals) == 1
        assert node._current_goal_handle is not None
        assert node._current_goal_handle.cancel_calls == 0

        close_depth = bridge.cv2_to_imgmsg(
            np.full((480, 640), 600, dtype=np.uint16), encoding='passthrough'
        )
        node._on_synchronized(rgb, close_depth, info)

        assert node._current_goal_handle is None
        total_cancel_calls = sum(gh.cancel_calls for gh in fake_client.goal_handles)
        assert total_cancel_calls == 1
        assert len(fake_client.sent_goals) == 1  # 완료 프레임에서는 goal을 재전송하지 않는다
    finally:
        rclpy.shutdown()


def test_disabled_pipeline_does_nothing():
    rclpy.init()
    try:
        boxes = [_FakeBox(300.0, 220.0, 340.0, 260.0, 0.9, 0)]
        fake_client = _FakeActionClient()
        node = VehicleApproachNode(detector=_FakeDetector(boxes), action_client=fake_client)
        # node._enabled 기본값 False -- enable 토픽 수신 전까지 대기 (스펙 §3.2)
        _seed_tf(node)

        rgb, depth, info = _make_synced_messages()
        node._on_synchronized(rgb, depth, info)

        assert fake_client.sent_goals == []
    finally:
        rclpy.shutdown()
