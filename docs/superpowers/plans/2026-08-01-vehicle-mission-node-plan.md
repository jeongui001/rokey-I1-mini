# vehicle_mission_node Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 맵 상 고정된 대기 지점으로 로봇을 이동시키고, 도착하면 `vehicle_approach_node`를 활성화하는 `vehicle_mission_node`를 구현한다.

**Architecture:** 웨이포인트→`PoseStamped` 변환과 Nav2 액션 결과 판정을 순수 함수로 분리해 pytest로 검증한다. `VehicleMissionNode`(rclpy)는 Nav2 `NavigateToPose` 액션 클라이언트 배선과 토픽 구독/발행만 담당하는 얇은 래퍼이며, 액션 클라이언트를 생성자 인자로 주입 가능하게 해 실제 Nav2 서버 없이도 노드 로직(목표 전송 → 결과 콜백 → enable 발행)을 테스트한다.

**Tech Stack:** ROS2 rclpy (Humble 가정), `nav2_msgs/action/NavigateToPose`, `action_msgs/msg/GoalStatus`, `pytest`.

## Global Constraints

- 대상 스펙: `docs/superpowers/specs/2026-08-01-vehicle-approach-design.md` §3.1(vehicle_mission_node), §3.2(상태 전이), §4(인터페이스).
- 패키지는 `src/vehicle_mission/`에 ament_python으로 둔다. `webcam_perception` 패키지와 코드 의존성이 없다 — 표준 메시지로만 런타임 통신하므로 이 플랜만 단독으로 빌드·테스트 가능하다.
- `vehicle_mission_node`는 상태를 **대기 지점 이동 중 → 정밀 접근 활성화됨** 두 가지만 가진다(스펙 §3.2). 활성화 이후 이 노드가 다시 할 일은 없다 — 재활성화·실패 복귀 로직은 구현하지 않는다(범위 밖).
- `/webcam/vehicle_initial_pose` 구독은 **로깅용 보관만** 한다. 이 값을 대기 지점 계산이나 이동 목표에 사용하는 코드를 절대 추가하지 않는다(스펙 §1.3 — 노트 대비 변경 사항의 핵심).
- 대기 지점은 맵 상 고정 좌표(파라미터)이며 웹캠 위치와 무관하다(스펙 §1.3, §3).
- `/webcam/vehicle_initial_pose` 구독 QoS와 `/vehicle_approach/enable` 발행 QoS는 모두 `transient_local`(스펙 §4) — 발행자·구독자 양쪽 다 동일하게 설정해야 QoS mismatch로 인한 조용한 유실을 피한다.
- `/vehicle_approach/enable`은 대기 지점 도착(Nav2 액션 결과 수신) 후 `true`를 발행한다(스펙 §4).
- 스펙 §7의 TBD 값(대기 지점의 구체 map 좌표값)은 ROS2 파라미터로 노출하고 `config/params.yaml`에 임시 기본값 + `# TBD` 주석으로 표시한다.
- 이 노드의 실물 Nav2 스택 연동(액션 서버 실제 응답, 로봇 실제 이동)은 스펙 §1.2/§8에서 테스트/검증 절차가 범위 밖으로 명시되어 있으므로, 자동화 테스트는 액션 클라이언트를 주입 가능한 이중체(fake)로 대체해 노드 내부 로직만 검증한다. 실제 Nav2 대상 수동 검증 절차는 Task 3 마지막에 별도로 안내한다.

---

### Task 1: 패키지 스캐폴드 + pose_utils (웨이포인트 → PoseStamped)

**Files:**
- Create: `src/vehicle_mission/package.xml`
- Create: `src/vehicle_mission/setup.py`
- Create: `src/vehicle_mission/setup.cfg`
- Create: `src/vehicle_mission/resource/vehicle_mission`
- Create: `src/vehicle_mission/vehicle_mission/__init__.py`
- Create: `src/vehicle_mission/vehicle_mission/pose_utils.py`
- Test: `src/vehicle_mission/test/test_pose_utils.py`

**Interfaces:**
- Produces: `pose_utils.waypoint_to_pose_stamped(x: float, y: float, yaw: float, frame_id: str = 'map') -> geometry_msgs.msg.PoseStamped`

- [ ] **Step 1: 패키지 스캐폴드 생성**

`src/vehicle_mission/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vehicle_mission</name>
  <version>0.0.1</version>
  <description>대기 지점 이동 후 vehicle_approach_node를 활성화하는 미션 상태 전이 노드</description>
  <maintainer email="hwangjeongui01@gmail.com">hwangjeongui</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>action_msgs</depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`src/vehicle_mission/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'vehicle_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwangjeongui',
    maintainer_email='hwangjeongui01@gmail.com',
    description='대기 지점 이동 후 vehicle_approach_node를 활성화하는 미션 상태 전이 노드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_mission_node = vehicle_mission.vehicle_mission_node:main',
        ],
    },
)
```

`src/vehicle_mission/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/vehicle_mission
[install]
install_scripts=$base/lib/vehicle_mission
```

`src/vehicle_mission/resource/vehicle_mission`: 빈 파일.

`src/vehicle_mission/vehicle_mission/__init__.py`: 빈 파일.

- [ ] **Step 2: 실패하는 테스트 작성**

`src/vehicle_mission/test/test_pose_utils.py`:
```python
import math

from vehicle_mission.pose_utils import waypoint_to_pose_stamped


def test_zero_yaw_identity_orientation():
    pose = waypoint_to_pose_stamped(1.0, 2.0, 0.0)
    assert pose.header.frame_id == 'map'
    assert pose.pose.position.x == 1.0
    assert pose.pose.position.y == 2.0
    assert math.isclose(pose.pose.orientation.z, 0.0, abs_tol=1e-9)
    assert math.isclose(pose.pose.orientation.w, 1.0, abs_tol=1e-9)


def test_half_pi_yaw_orientation():
    pose = waypoint_to_pose_stamped(0.0, 0.0, math.pi / 2)
    assert math.isclose(pose.pose.orientation.z, math.sin(math.pi / 4), abs_tol=1e-9)
    assert math.isclose(pose.pose.orientation.w, math.cos(math.pi / 4), abs_tol=1e-9)


def test_pi_yaw_orientation():
    pose = waypoint_to_pose_stamped(0.0, 0.0, math.pi)
    assert math.isclose(pose.pose.orientation.z, 1.0, abs_tol=1e-9)
    assert math.isclose(pose.pose.orientation.w, 0.0, abs_tol=1e-9)
```

- [ ] **Step 3: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_mission python3 -m pytest src/vehicle_mission/test/test_pose_utils.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_mission.pose_utils'`

- [ ] **Step 4: pose_utils.py 구현**

`src/vehicle_mission/vehicle_mission/pose_utils.py`:
```python
import math

from geometry_msgs.msg import PoseStamped


def waypoint_to_pose_stamped(
    x: float, y: float, yaw: float, frame_id: str = 'map'
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `PYTHONPATH=src/vehicle_mission python3 -m pytest src/vehicle_mission/test/test_pose_utils.py -v`
Expected: PASS (3 passed)

- [ ] **Step 6: 커밋**

```bash
git add src/vehicle_mission/package.xml src/vehicle_mission/setup.py src/vehicle_mission/setup.cfg \
  src/vehicle_mission/resource/vehicle_mission src/vehicle_mission/vehicle_mission/__init__.py \
  src/vehicle_mission/vehicle_mission/pose_utils.py src/vehicle_mission/test/test_pose_utils.py
git commit -m "feat(vehicle_mission): add package scaffold and waypoint pose conversion"
```

---

### Task 2: Nav2 액션 결과 판정(nav_result)

**Files:**
- Create: `src/vehicle_mission/vehicle_mission/nav_result.py`
- Test: `src/vehicle_mission/test/test_nav_result.py`

**Interfaces:**
- Consumes: `action_msgs.msg.GoalStatus` 상수
- Produces: `nav_result.handle_nav_result(status: int) -> bool`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/vehicle_mission/test/test_nav_result.py`:
```python
from action_msgs.msg import GoalStatus

from vehicle_mission.nav_result import handle_nav_result


def test_succeeded_status_returns_true():
    assert handle_nav_result(GoalStatus.STATUS_SUCCEEDED) is True


def test_aborted_status_returns_false():
    assert handle_nav_result(GoalStatus.STATUS_ABORTED) is False


def test_canceled_status_returns_false():
    assert handle_nav_result(GoalStatus.STATUS_CANCELED) is False
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_mission python3 -m pytest src/vehicle_mission/test/test_nav_result.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_mission.nav_result'`

- [ ] **Step 3: nav_result.py 구현**

`src/vehicle_mission/vehicle_mission/nav_result.py`:
```python
from action_msgs.msg import GoalStatus


def handle_nav_result(status: int) -> bool:
    return status == GoalStatus.STATUS_SUCCEEDED
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `PYTHONPATH=src/vehicle_mission python3 -m pytest src/vehicle_mission/test/test_nav_result.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/vehicle_mission/vehicle_mission/nav_result.py src/vehicle_mission/test/test_nav_result.py
git commit -m "feat(vehicle_mission): add nav2 goal status judgment"
```

---

### Task 3: VehicleMissionNode (rclpy 래퍼) + 파라미터 파일

**Files:**
- Create: `src/vehicle_mission/vehicle_mission/vehicle_mission_node.py`
- Create: `src/vehicle_mission/config/params.yaml`
- Test: `src/vehicle_mission/test/test_vehicle_mission_node.py`

**Interfaces:**
- Consumes: `pose_utils.waypoint_to_pose_stamped`, `nav_result.handle_nav_result`
- Produces: `vehicle_mission_node.VehicleMissionNode(action_client=None)` (rclpy.Node), 메서드 `send_waypoint_goal() -> None`, 진입점 `vehicle_mission_node.main()`

- [ ] **Step 1: 실패하는 노드 테스트 작성**

`src/vehicle_mission/test/test_vehicle_mission_node.py`:
```python
from types import SimpleNamespace

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

from vehicle_mission.vehicle_mission_node import VehicleMissionNode


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
            node._enable_publisher.qos_profile,
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
        rclpy.shutdown()


def test_successful_nav_result_publishes_enable_true():
    rclpy.init()
    try:
        fake_client = _FakeActionClient(accepted=True, status=GoalStatus.STATUS_SUCCEEDED)
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

        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(listener, timeout_sec=0.05)
            if received:
                break

        assert len(fake_client.sent_goals) == 1
        assert len(received) == 1
        assert received[0].data is True
    finally:
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
        rclpy.shutdown()
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_mission python3 -m pytest src/vehicle_mission/test/test_vehicle_mission_node.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_mission.vehicle_mission_node'`

- [ ] **Step 3: vehicle_mission_node.py 구현**

`src/vehicle_mission/vehicle_mission/vehicle_mission_node.py`:
```python
import rclpy
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import Bool

from vehicle_mission.nav_result import handle_nav_result
from vehicle_mission.pose_utils import waypoint_to_pose_stamped

TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class VehicleMissionNode(Node):
    def __init__(self, action_client=None):
        super().__init__('vehicle_mission_node')

        self.declare_parameter('waypoint_x', 0.0)
        self.declare_parameter('waypoint_y', 0.0)
        self.declare_parameter('waypoint_yaw', 0.0)

        self._last_webcam_pose: PointStamped | None = None
        self.create_subscription(
            PointStamped,
            '/webcam/vehicle_initial_pose',
            self._on_webcam_pose,
            TRANSIENT_LOCAL_QOS,
        )

        self._enable_publisher = self.create_publisher(
            Bool, '/vehicle_approach/enable', TRANSIENT_LOCAL_QOS
        )

        self._action_client = action_client or ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    def _on_webcam_pose(self, msg: PointStamped) -> None:
        # 로깅/모니터링용 보관만 한다 — 이동 목표 계산에는 절대 사용하지 않는다 (스펙 §1.3)
        self._last_webcam_pose = msg
        self.get_logger().info(
            f'webcam initial pose (logging only): x={msg.point.x:.3f}, y={msg.point.y:.3f}'
        )

    def send_waypoint_goal(self) -> None:
        pose = waypoint_to_pose_stamped(
            self.get_parameter('waypoint_x').value,
            self.get_parameter('waypoint_y').value,
            self.get_parameter('waypoint_yaw').value,
        )
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('waypoint goal rejected by Nav2')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        if handle_nav_result(result.status):
            msg = Bool()
            msg.data = True
            self._enable_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleMissionNode()
    node.send_waypoint_goal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: config/params.yaml 작성**

`src/vehicle_mission/config/params.yaml`:
```yaml
vehicle_mission_node:
  ros__parameters:
    # 아래 값들은 스펙 §7 TBD 항목 — 실측 후 확정 필요한 대기 지점 좌표의 임시 기본값이다.
    waypoint_x: 0.0    # TBD: 맵 상 대기 지점 x(m)
    waypoint_y: 0.0    # TBD: 맵 상 대기 지점 y(m)
    waypoint_yaw: 0.0  # TBD: 대기 지점에서의 로봇 헤딩(rad)
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `PYTHONPATH=src/vehicle_mission python3 -m pytest src/vehicle_mission/test/test_vehicle_mission_node.py -v`
Expected: PASS (3 passed)

- [ ] **Step 6: colcon 빌드/테스트로 패키지 전체 검증**

Run:
```bash
cd <workspace_root>
colcon build --packages-select vehicle_mission --symlink-install
colcon test --packages-select vehicle_mission
colcon test-result --verbose
```
Expected: 빌드 성공, 이전 3개 태스크의 모든 테스트(pose_utils 3, nav_result 3, node 3 = 총 9개) PASS.

- [ ] **Step 7: 커밋**

```bash
git add src/vehicle_mission/vehicle_mission/vehicle_mission_node.py \
  src/vehicle_mission/config/params.yaml \
  src/vehicle_mission/test/test_vehicle_mission_node.py \
  src/vehicle_mission/setup.py
git commit -m "feat(vehicle_mission): add rclpy node wrapper and params file"
```

- [ ] **Step 8 (선택, 하드웨어/Nav2 스택 필요 — 수동 검증):**

실제 Nav2 스택이 기동된 환경에서:
```bash
ros2 launch nav2_bringup navigation_launch.py   # 또는 로봇 측 기존 launch 구성
ros2 run vehicle_mission vehicle_mission_node --ros-args --params-file src/vehicle_mission/config/params.yaml
ros2 topic echo /vehicle_approach/enable
```
로봇이 파라미터로 지정한 대기 지점까지 실제로 이동하고, 도착 후 `/vehicle_approach/enable`에 `true`가 1회 발행되는지 육안으로 확인한다. 이 절차는 스펙 §1.2/§8에 따라 자동화 테스트 범위 밖이다.

---

## Self-Review 메모 (플랜 작성자용, 실행 시 삭제 가능)

- **스펙 커버리지**: §3.1(vehicle_mission_node 역할) → Task 3, §3.2(상태 전이: 대기 지점 이동 중 → 정밀 접근 활성화됨, 재활성화 없음) → Task 3(단방향 콜백 체인으로 구현, 재시도 로직 없음), §4(webcam 구독은 로깅용/QoS, mission→Nav2 액션, mission→approach enable/QoS) → Task 1·3. 모두 커버.
- `/webcam/vehicle_initial_pose` 값을 이동 목표 계산에 쓰지 않는다는 §1.3 제약은 `_on_webcam_pose`가 저장만 하고 `send_waypoint_goal`은 파라미터만 참조하는 구조로 코드 수준에서 강제했다.
