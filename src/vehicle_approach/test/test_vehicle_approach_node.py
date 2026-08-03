import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped

from vehicle_approach.vehicle_approach_node import VehicleApproachNode


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


def _make_depth_msg(depth_mm: int):
    bridge = CvBridge()
    return bridge.cv2_to_imgmsg(
        np.full((480, 640), depth_mm, dtype=np.uint16), encoding='passthrough'
    )


def _make_webcam_pose(x: float, y: float) -> PointStamped:
    msg = PointStamped()
    msg.header.frame_id = 'map'
    msg.point.x = x
    msg.point.y = y
    return msg


def test_enabled_with_far_depth_sends_goal_to_webcam_pose():
    rclpy.init()
    try:
        fake_client = _FakeActionClient()
        node = VehicleApproachNode(action_client=fake_client)
        node._enabled = True
        _seed_tf(node)
        node._on_webcam_pose(_make_webcam_pose(3.0, 1.0))

        node._on_depth(_make_depth_msg(2000))

        assert len(fake_client.sent_goals) == 1
        goal_pose = fake_client.sent_goals[0].pose
        assert goal_pose.pose.position.x == 3.0
        assert goal_pose.pose.position.y == 1.0
    finally:
        rclpy.shutdown()


def test_close_depth_cancels_goal_instead_of_sending():
    rclpy.init()
    try:
        fake_client = _FakeActionClient()
        node = VehicleApproachNode(action_client=fake_client)
        node._enabled = True
        _seed_tf(node)
        node._on_webcam_pose(_make_webcam_pose(3.0, 1.0))

        node._on_depth(_make_depth_msg(2000))
        assert len(fake_client.sent_goals) == 1
        assert node._current_goal_handle is not None

        node._on_depth(_make_depth_msg(600))

        assert node._current_goal_handle is None
        assert fake_client.goal_handles[0].cancel_calls == 1
        assert len(fake_client.sent_goals) == 1  # 정지 프레임에서는 goal을 재전송하지 않는다
    finally:
        rclpy.shutdown()


def test_disabled_pipeline_does_nothing():
    rclpy.init()
    try:
        fake_client = _FakeActionClient()
        node = VehicleApproachNode(action_client=fake_client)
        # node._enabled 기본값 False -- enable 토픽 수신 전까지 대기
        _seed_tf(node)
        node._on_webcam_pose(_make_webcam_pose(3.0, 1.0))

        node._on_depth(_make_depth_msg(2000))

        assert fake_client.sent_goals == []
    finally:
        rclpy.shutdown()


def test_no_webcam_pose_yet_does_nothing():
    rclpy.init()
    try:
        fake_client = _FakeActionClient()
        node = VehicleApproachNode(action_client=fake_client)
        node._enabled = True
        _seed_tf(node)

        node._on_depth(_make_depth_msg(2000))

        assert fake_client.sent_goals == []
    finally:
        rclpy.shutdown()
