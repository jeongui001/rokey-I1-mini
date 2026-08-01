from types import SimpleNamespace

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

from vehicle_mission.vehicle_mission_node import TRANSIENT_LOCAL_QOS, VehicleMissionNode


class _FakeGoalHandle:
    def __init__(self, accepted: bool, status: int):
        self.accepted = accepted
        self._status = status

    def get_result_async(self):
        future = rclpy.task.Future()
        future.set_result(SimpleNamespace(status=self._status))
        return future


class _FakeActionClient:
    def __init__(self, accepted=True, status=GoalStatus.STATUS_SUCCEEDED):
        self.sent_goals = []
        self._accepted = accepted
        self._status = status

    def wait_for_server(self):
        return True

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        future = rclpy.task.Future()
        future.set_result(_FakeGoalHandle(self._accepted, self._status))
        return future


def test_webcam_pose_is_stored_for_logging_only():
    rclpy.init()
    try:
        node = VehicleMissionNode(action_client=_FakeActionClient())
        publisher_node = rclpy.create_node('test_publisher')
        publisher = publisher_node.create_publisher(
            PointStamped,
            '/webcam/vehicle_initial_pose',
            TRANSIENT_LOCAL_QOS,
        )
        msg = PointStamped()
        msg.point.x = 3.0
        msg.point.y = 4.0
        publisher.publish(msg)

        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(publisher_node, timeout_sec=0.05)
            if node._last_webcam_pose is not None:
                break

        assert node._last_webcam_pose is not None
        assert node._last_webcam_pose.point.x == 3.0
    finally:
        node.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown()


def test_successful_nav_result_publishes_enable_true():
    rclpy.init()
    try:
        fake_client = _FakeActionClient(accepted=True, status=GoalStatus.STATUS_SUCCEEDED)
        node = VehicleMissionNode(action_client=fake_client)
        node.set_parameters([
            rclpy.parameter.Parameter('waypoint_x', value=2.0),
            rclpy.parameter.Parameter('waypoint_y', value=5.0),
            rclpy.parameter.Parameter('waypoint_yaw', value=0.0),
        ])

        listener = rclpy.create_node('test_listener')
        received: list[Bool] = []
        listener.create_subscription(
            Bool,
            '/vehicle_approach/enable',
            lambda msg: received.append(msg),
            node._enable_publisher.qos_profile,
        )

        # 스펙 §1.3: 웹캠 초기 위치는 목표 계산에 절대 쓰이지 않는다.
        # 웨이포인트 파라미터와 다른 좌표를 목표 전송 "전"에 주입해 회귀를 검증한다.
        publisher_node = rclpy.create_node('test_webcam_publisher')
        webcam_publisher = publisher_node.create_publisher(
            PointStamped,
            '/webcam/vehicle_initial_pose',
            TRANSIENT_LOCAL_QOS,
        )
        webcam_msg = PointStamped()
        webcam_msg.point.x = 99.0
        webcam_msg.point.y = 88.0
        webcam_publisher.publish(webcam_msg)

        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(publisher_node, timeout_sec=0.05)
            if node._last_webcam_pose is not None:
                break
        assert node._last_webcam_pose is not None

        node.send_waypoint_goal()

        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(listener, timeout_sec=0.05)
            if received:
                break

        assert len(fake_client.sent_goals) == 1
        sent_pose = fake_client.sent_goals[0].pose
        assert sent_pose.header.frame_id == 'map'
        assert sent_pose.pose.position.x == 2.0
        assert sent_pose.pose.position.y == 5.0
        assert len(received) == 1
        assert received[0].data is True
    finally:
        node.destroy_node()
        listener.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown()


def test_aborted_nav_result_does_not_publish_enable():
    rclpy.init()
    try:
        fake_client = _FakeActionClient(accepted=True, status=GoalStatus.STATUS_ABORTED)
        node = VehicleMissionNode(action_client=fake_client)

        listener = rclpy.create_node('test_listener')
        received: list[Bool] = []
        listener.create_subscription(
            Bool,
            '/vehicle_approach/enable',
            lambda msg: received.append(msg),
            node._enable_publisher.qos_profile,
        )

        node.send_waypoint_goal()

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(listener, timeout_sec=0.05)

        assert received == []
    finally:
        node.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()


def test_rejected_goal_does_not_publish_enable():
    rclpy.init()
    try:
        fake_client = _FakeActionClient(accepted=False)
        node = VehicleMissionNode(action_client=fake_client)

        listener = rclpy.create_node('test_listener')
        received: list[Bool] = []
        listener.create_subscription(
            Bool,
            '/vehicle_approach/enable',
            lambda msg: received.append(msg),
            node._enable_publisher.qos_profile,
        )

        node.send_waypoint_goal()

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(listener, timeout_sec=0.05)

        assert received == []
    finally:
        node.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
