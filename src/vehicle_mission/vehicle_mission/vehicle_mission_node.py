import math
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Undock
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from vehicle_mission.logging_setup import setup_package_logger
from vehicle_mission.pose_utils import waypoint_to_pose_stamped

logger = setup_package_logger('vehicle_mission')


TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class MissionState(Enum):
    WAIT_LOCALIZATION = auto()
    WAIT_WEBCAM_TRIGGER = auto()
    UNDOCKING = auto()
    WAIT_AFTER_UNDOCK = auto()
    NAVIGATING_TO_WAYPOINT = auto()
    APPROACH_ENABLED = auto()
    ERROR = auto()


class VehicleMissionNode(Node):
    """자동 초기 위치 → 웹캠 감시 → 언도킹 → 첫 웨이포인트.

    웹캠 RC카가 첫 웨이포인트 이동 중 사라지면 waypoint goal을 취소하고,
    웹캠에 다시 나타날 때까지 정지 상태로 기다린다.
    """

    def __init__(
        self,
        action_client=None,
        undock_action_client=None,
    ):
        super().__init__('vehicle_mission_node')

        self.declare_parameter('waypoint_x', 0.0)
        self.declare_parameter('waypoint_y', 0.0)
        self.declare_parameter('waypoint_yaw', 0.0)

        self.declare_parameter('auto_set_initial_pose', True)
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('initial_pose_covariance_xy', 0.25)
        self.declare_parameter(
            'initial_pose_covariance_yaw',
            0.06853891945200942,
        )
        self.declare_parameter('initial_pose_retry_s', 2.0)

        self.declare_parameter('post_undock_wait_s', 1.5)
        self.declare_parameter('action_retry_s', 2.0)
        self.declare_parameter('max_undock_attempts', 3)
        self.declare_parameter('max_waypoint_attempts', 3)
        self.declare_parameter('waypoint_position_only_fallback', True)
        self.declare_parameter('waypoint_position_tolerance_m', 0.35)
        self.declare_parameter('webcam_state_timeout_s', 1.0)

        self.declare_parameter(
            'webcam_trigger_topic',
            '/webcam/vehicle_detected',
        )
        self.declare_parameter(
            'approach_enable_topic',
            '/vehicle_approach/enable',
        )
        self.declare_parameter(
            'initial_pose_topic',
            '/robot11/initialpose',
        )
        self.declare_parameter(
            'amcl_pose_topic',
            '/robot11/amcl_pose',
        )
        self.declare_parameter(
            'undock_action_name',
            '/robot11/undock',
        )
        self.declare_parameter(
            'nav_action_name',
            '/robot11/navigate_to_pose',
        )
        self.declare_parameter(
            'bt_navigator_state_service',
            '/robot11/bt_navigator/get_state',
        )
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self._state = MissionState.WAIT_LOCALIZATION
        self._webcam_visible = False
        self._webcam_triggered_once = False
        self._last_webcam_state_time = None

        self._amcl_pose_received = False
        self._last_initial_pose_publish_time = None
        self._retry_not_before = None
        self._post_undock_until = None
        self._undock_attempts = 0
        self._waypoint_attempts = 0
        self._last_webcam_pose = None  # 기존 인터페이스 호환/로그용

        self._current_waypoint_goal_handle = None
        self._waypoint_send_pending = False
        self._waypoint_cancelled_for_webcam = False

        # Action server discovery alone does not mean Nav2 can accept goals.
        # bt_navigator remains discoverable while inactive and rejects goals.
        self._nav2_active = False
        self._nav2_state_request_pending = False
        self._last_nav2_state_id = None

        self._initial_pose_topic = str(
            self.get_parameter('initial_pose_topic').value
        )

        self.create_subscription(
            Bool,
            str(self.get_parameter('webcam_trigger_topic').value),
            self._on_webcam_trigger,
            TRANSIENT_LOCAL_QOS,
        )
        self.create_subscription(
            PointStamped,
            '/webcam/vehicle_initial_pose',
            self._on_legacy_webcam_pose,
            TRANSIENT_LOCAL_QOS,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('amcl_pose_topic').value),
            self._on_amcl_pose,
            10,
        )

        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self._initial_pose_topic,
            TRANSIENT_LOCAL_QOS,
        )
        self._enable_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('approach_enable_topic').value),
            TRANSIENT_LOCAL_QOS,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._action_client = action_client or ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('nav_action_name').value),
        )
        self._undock_client = undock_action_client or ActionClient(
            self,
            Undock,
            str(self.get_parameter('undock_action_name').value),
        )
        self._bt_state_client = self.create_client(
            GetState,
            str(self.get_parameter('bt_navigator_state_service').value),
        )

        self._timer = self.create_timer(0.2, self._coordinator_tick)
        logger.info(
            'mission ready: initial pose -> continuous webcam guard -> '
            'undock -> first waypoint -> OAK-D follow'
        )

    def _set_state(self, state: MissionState) -> None:
        if self._state == state:
            return
        logger.info(f'mission state: {self._state.name} -> {state.name}')
        self._state = state

    def _request_nav2_state(self) -> None:
        if self._nav2_state_request_pending:
            return
        if not self._bt_state_client.service_is_ready():
            return

        self._nav2_state_request_pending = True
        future = self._bt_state_client.call_async(GetState.Request())
        future.add_done_callback(self._on_nav2_state)

    def _on_nav2_state(self, future) -> None:
        self._nav2_state_request_pending = False
        try:
            response = future.result()
        except Exception as ex:
            self._nav2_active = False
            logger.warning(f'failed to read bt_navigator lifecycle state: {ex}')
            return

        state_id = int(response.current_state.id)
        was_active = self._nav2_active
        self._nav2_active = state_id == State.PRIMARY_STATE_ACTIVE

        if state_id != self._last_nav2_state_id:
            self._last_nav2_state_id = state_id
            logger.info(
                'bt_navigator lifecycle state: '
                f'{response.current_state.label} [{state_id}]'
            )
        if self._nav2_active and not was_active:
            logger.info('Nav2 ready: bt_navigator is active')

    def _coordinator_tick(self) -> None:
        self._request_nav2_state()

        if self._state == MissionState.WAIT_LOCALIZATION:
            self._handle_localization()
        elif self._state == MissionState.WAIT_WEBCAM_TRIGGER:
            self._handle_webcam_wait()
        elif self._state == MissionState.WAIT_AFTER_UNDOCK:
            self._handle_post_undock()
        elif self._state == MissionState.NAVIGATING_TO_WAYPOINT:
            if not self._webcam_is_fresh_and_visible():
                self._cancel_waypoint_for_webcam_loss()

    def _handle_localization(self) -> None:
        if bool(self.get_parameter('auto_set_initial_pose').value):
            if self._last_initial_pose_publish_time is None:
                if self.count_subscribers(self._initial_pose_topic) == 0:
                    return
                self._publish_initial_pose()
                return

            if not self._localization_ready():
                elapsed = (
                    self.get_clock().now()
                    - self._last_initial_pose_publish_time
                ).nanoseconds / 1e9
                if elapsed >= float(
                    self.get_parameter('initial_pose_retry_s').value
                ):
                    self._publish_initial_pose()
                return
        elif not self._localization_ready():
            return

        logger.info('localization ready; waiting docked for RC car')
        self._set_state(MissionState.WAIT_WEBCAM_TRIGGER)

    def _publish_initial_pose(self) -> None:
        x = float(self.get_parameter('initial_pose_x').value)
        y = float(self.get_parameter('initial_pose_y').value)
        yaw = float(self.get_parameter('initial_pose_yaw').value)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = str(self.get_parameter('map_frame').value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = float(
            self.get_parameter('initial_pose_covariance_xy').value
        )
        msg.pose.covariance[7] = msg.pose.covariance[0]
        msg.pose.covariance[35] = float(
            self.get_parameter('initial_pose_covariance_yaw').value
        )

        self._amcl_pose_received = False
        self._last_initial_pose_publish_time = self.get_clock().now()
        self._initial_pose_publisher.publish(msg)
        logger.info(
            f'automatic initial pose: '
            f'x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )

    def _on_amcl_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        self._amcl_pose_received = True

    def _localization_ready(self) -> bool:
        return self._amcl_pose_received and self._tf_buffer.can_transform(
            str(self.get_parameter('map_frame').value),
            str(self.get_parameter('base_frame').value),
            Time(),
        )

    def _on_webcam_trigger(self, msg: Bool) -> None:
        previous = self._webcam_visible
        self._webcam_visible = bool(msg.data)
        self._last_webcam_state_time = self.get_clock().now()

        if self._webcam_visible:
            self._webcam_triggered_once = True
            if not previous:
                logger.info('external webcam RC car visible')
        elif previous:
            logger.warning('external webcam RC car lost')
            if self._state == MissionState.NAVIGATING_TO_WAYPOINT:
                self._cancel_waypoint_for_webcam_loss()

    def _webcam_is_fresh_and_visible(self) -> bool:
        if not self._webcam_visible or self._last_webcam_state_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self._last_webcam_state_time
        ).nanoseconds / 1e9
        return elapsed <= float(
            self.get_parameter('webcam_state_timeout_s').value
        )

    def _on_legacy_webcam_pose(self, msg: PointStamped) -> None:
        self._last_webcam_pose = msg

    def _handle_webcam_wait(self) -> None:
        # Do not undock until Nav2 can actually accept NavigateToPose goals.
        if not self._nav2_active:
            return
        if not self._webcam_triggered_once:
            return
        if not self._webcam_is_fresh_and_visible():
            return
        if not self._retry_elapsed():
            return
        if not self._undock_client.server_is_ready():
            return
        self._send_undock_goal()

    def _send_undock_goal(self) -> None:
        maximum = int(self.get_parameter('max_undock_attempts').value)
        if self._undock_attempts >= maximum:
            logger.error('undock retries exhausted')
            self._set_state(MissionState.ERROR)
            return

        self._undock_attempts += 1
        self._set_state(MissionState.UNDOCKING)
        logger.info(
            f'sending undock goal ({self._undock_attempts}/{maximum})'
        )
        future = self._undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self._on_undock_goal_response)

    def _on_undock_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as ex:
            logger.error(f'undock response error: {ex}')
            self._retry_undock()
            return
        if not handle.accepted:
            logger.warning('undock goal rejected')
            self._retry_undock()
            return
        handle.get_result_async().add_done_callback(self._on_undock_result)

    def _on_undock_result(self, future) -> None:
        try:
            result = future.result()
        except Exception as ex:
            logger.error(f'undock result error: {ex}')
            self._retry_undock()
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            logger.warning(f'undock failed: status={result.status}')
            self._retry_undock()
            return

        logger.info('undock succeeded')
        self._post_undock_until = self.get_clock().now() + Duration(
            seconds=float(self.get_parameter('post_undock_wait_s').value)
        )
        self._set_state(MissionState.WAIT_AFTER_UNDOCK)

    def _retry_undock(self) -> None:
        self._schedule_retry()
        self._set_state(MissionState.WAIT_WEBCAM_TRIGGER)

    def _handle_post_undock(self) -> None:
        if (
            self._post_undock_until
            and self.get_clock().now() < self._post_undock_until
        ):
            return
        # 언도킹 후 RC카가 웹캠에서 사라졌다면 움직이지 않고 기다린다.
        if not self._webcam_is_fresh_and_visible():
            return
        if not self._localization_ready() or not self._retry_elapsed():
            return
        if not self._nav2_active:
            return
        if not self._action_client.server_is_ready():
            return
        self.send_waypoint_goal()

    def send_waypoint_goal(self) -> None:
        if self._waypoint_send_pending:
            return

        maximum = int(self.get_parameter('max_waypoint_attempts').value)
        if self._waypoint_attempts >= maximum:
            logger.error('waypoint retries exhausted')
            self._set_state(MissionState.ERROR)
            return

        self._waypoint_attempts += 1
        x = float(self.get_parameter('waypoint_x').value)
        y = float(self.get_parameter('waypoint_y').value)
        yaw = float(self.get_parameter('waypoint_yaw').value)
        pose = waypoint_to_pose_stamped(x, y, yaw)
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._set_state(MissionState.NAVIGATING_TO_WAYPOINT)
        self._waypoint_send_pending = True
        self._waypoint_cancelled_for_webcam = False
        logger.info(
            f'sending first waypoint: '
            f'x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        self._waypoint_send_pending = False
        try:
            handle = future.result()
        except Exception as ex:
            logger.error(f'waypoint response error: {ex}')
            self._retry_waypoint()
            return
        if not handle.accepted:
            logger.warning('first waypoint rejected')
            self._retry_waypoint()
            return

        self._current_waypoint_goal_handle = handle
        if self._waypoint_cancelled_for_webcam or not self._webcam_is_fresh_and_visible():
            handle.cancel_goal_async()
            logger.warning('waypoint accepted after webcam loss; cancelling')

        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        self._current_waypoint_goal_handle = None
        try:
            result = future.result()
        except Exception as ex:
            logger.error(f'waypoint result error: {ex}')
            self._retry_waypoint()
            return

        if self._waypoint_cancelled_for_webcam:
            # 안전 정지는 실패 횟수로 세지 않는다.
            self._waypoint_attempts = max(0, self._waypoint_attempts - 1)
            self._waypoint_cancelled_for_webcam = False
            self._set_state(MissionState.WAIT_AFTER_UNDOCK)
            logger.info('waiting for RC car to reappear before waypoint retry')
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            logger.warning(f'first waypoint failed: status={result.status}')
            if (
                bool(
                    self.get_parameter(
                        'waypoint_position_only_fallback'
                    ).value
                )
                and self._robot_is_within_waypoint_tolerance()
            ):
                logger.warning(
                    'Nav2 did not return SUCCEEDED, but the robot is already '
                    'inside the waypoint position tolerance; enabling follow'
                )
                self._enable_approach()
                return
            self._retry_waypoint()
            return

        self._enable_approach()

    def _robot_is_within_waypoint_tolerance(self) -> bool:
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                Time(),
                timeout=Duration(seconds=0.3),
            )
        except Exception as ex:
            logger.warning(f'waypoint tolerance TF lookup failed: {ex}')
            return False

        robot_x = float(transform.transform.translation.x)
        robot_y = float(transform.transform.translation.y)
        waypoint_x = float(self.get_parameter('waypoint_x').value)
        waypoint_y = float(self.get_parameter('waypoint_y').value)
        distance = math.hypot(robot_x - waypoint_x, robot_y - waypoint_y)
        tolerance = float(
            self.get_parameter('waypoint_position_tolerance_m').value
        )
        logger.info(
            f'waypoint position check: distance={distance:.3f}m, '
            f'tolerance={tolerance:.3f}m'
        )
        return distance <= tolerance

    def _enable_approach(self) -> None:
        enable = Bool()
        enable.data = True
        self._enable_publisher.publish(enable)
        self._set_state(MissionState.APPROACH_ENABLED)
        logger.info('first waypoint reached; OAK-D/webcam follower enabled')

    def _cancel_waypoint_for_webcam_loss(self) -> None:
        if self._state != MissionState.NAVIGATING_TO_WAYPOINT:
            return
        if self._waypoint_cancelled_for_webcam:
            return

        self._waypoint_cancelled_for_webcam = True
        if self._current_waypoint_goal_handle is not None:
            self._current_waypoint_goal_handle.cancel_goal_async()
            logger.warning('webcam lost RC car: cancelling first waypoint')
        elif not self._waypoint_send_pending:
            self._waypoint_attempts = max(0, self._waypoint_attempts - 1)
            self._waypoint_cancelled_for_webcam = False
            self._set_state(MissionState.WAIT_AFTER_UNDOCK)

    def _retry_waypoint(self) -> None:
        self._schedule_retry()
        self._set_state(MissionState.WAIT_AFTER_UNDOCK)

    def _schedule_retry(self) -> None:
        self._retry_not_before = self.get_clock().now() + Duration(
            seconds=float(self.get_parameter('action_retry_s').value)
        )

    def _retry_elapsed(self) -> bool:
        if self._retry_not_before is None:
            return True
        if self.get_clock().now() < self._retry_not_before:
            return False
        self._retry_not_before = None
        return True

    def destroy_node(self):
        if self._current_waypoint_goal_handle is not None:
            self._current_waypoint_goal_handle.cancel_goal_async()
        if hasattr(self._undock_client, 'destroy'):
            self._undock_client.destroy()
        if hasattr(self._action_client, 'destroy'):
            self._action_client.destroy()
        if hasattr(self._bt_state_client, 'destroy'):
            self._bt_state_client.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
