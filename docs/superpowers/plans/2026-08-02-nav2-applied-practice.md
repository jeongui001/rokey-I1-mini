# nav2_applied_practice Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `rokey-I1-mini`의 실제 차량 접근 프로젝트(`src/vehicle_mission`, `src/vehicle_approach`)가 이미 쓰고 있는 Nav2 API 패턴(`NavigateToPose` 액션 클라이언트, `tf2_ros.Buffer` 좌표 변환, goal 재전송/완료취소 판정)을 TurtleBot3 + Gazebo 시뮬레이션 위에서 그대로 재현하는 ROS2 패키지 `nav2_applied_practice`를 만든다.

**Architecture:** `~/nav2_study_ws/src/nav2_applied_practice`에 3개의 실행 가능한 노드(`navigate_to_pose_client_node`, `tf_lookup_node`, `goal_resend_demo_node`)와 그 밑을 받치는 순수 함수 모듈(`pose_utils.py`, `goal_calculation.py`)을 만든다. 순수 함수는 `rokey-I1-mini`의 실제 파일(`vehicle_mission/pose_utils.py`, `vehicle_approach/goal_calculation.py`)과 동일한 시그니처로 재현하고 pytest로 검증한다. 노드는 실제 파일(`vehicle_mission_node.py`, `vehicle_approach_node.py`+`pipeline.py`)의 핵심 로직만 추려 재현하고, `nav2_fundamentals`의 lesson01 스택(TB3 Gazebo + nav2 bringup) 위에서 수동 실행/관찰로 검증한다.

**Tech Stack:** ROS2 Humble, ament_python, Python 3.10, `rclpy`, `nav2_msgs`, `tf2_ros`/`tf2_geometry_msgs`, pytest

## Global Constraints

- 워크스페이스 경로: `~/nav2_study_ws` — `nav2_fundamentals`와 같은 워크스페이스, 같은 `src/` 아래 (Task 1은 워크스페이스 자체를 새로 만들지 않고 기존 것을 재사용한다)
- 순수 함수(`pose_utils.py`, `goal_calculation.py`)만 pytest 단위테스트를 작성한다. 노드 자체는 시뮬레이션 실행으로 수동 검증한다 (스펙 §6)
- 각 노드 파일의 재현 대상 원본 파일을 문서(`docs/0N_*.md`)에 명시적으로 적는다
- `should_resend_goal`/`is_approach_complete`/`compute_goal_pose`는 `rokey-I1-mini/src/vehicle_approach/vehicle_approach/goal_calculation.py`와 동일한 시그니처로 재현한다: `compute_goal_pose(vehicle_x, vehicle_y, robot_x, robot_y) -> PoseStamped`, `should_resend_goal(new_position, last_sent_position, threshold_m) -> bool`, `is_approach_complete(robot_x, robot_y, vehicle_x, vehicle_y, threshold_m) -> bool`
- `waypoint_to_pose_stamped(x, y, yaw, frame_id='map') -> PoseStamped`는 `rokey-I1-mini/src/vehicle_mission/vehicle_mission/pose_utils.py`와 동일한 시그니처로 재현한다
- `goal_resend_demo_node`의 "결정 시점에 last_sent_goal을 먼저 갱신하고, 그 후 `server_is_ready()`(non-blocking) 체크가 실패하면 실제 전송을 건너뛴다"는 갭은 의도적으로 재현하며 고치지 않는다 (`rokey-I1-mini/docs/NEXT_STEPS.md` §2에 기록된 실제 프로젝트의 보류 이슈와 동일)
- `nav2_fundamentals`의 waypoint 검증값(`(-1.5, 1.15)` 등, 번들 맵의 실측 빈 공간)을 이 패키지의 예시 goal 좌표로도 그대로 재사용해 좌표가 실제로 유효함을 보장한다

---

### Task 1: 패키지 스캐폴딩

**Files:**
- Create: `~/nav2_study_ws/src/nav2_applied_practice/package.xml`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/setup.py`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/setup.cfg`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/resource/nav2_applied_practice`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice/__init__.py`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/test/__init__.py`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/config/params.yaml`

**Interfaces:**
- Produces: `colcon build --packages-select nav2_applied_practice`로 빌드되는 ament_python 패키지. 이후 태스크들은 `nav2_applied_practice/` 아래 모듈과 `test/` 아래 테스트를 추가한다.

- [ ] **Step 1: 디렉터리 생성**

```bash
mkdir -p ~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice
mkdir -p ~/nav2_study_ws/src/nav2_applied_practice/test
mkdir -p ~/nav2_study_ws/src/nav2_applied_practice/config
mkdir -p ~/nav2_study_ws/src/nav2_applied_practice/docs
mkdir -p ~/nav2_study_ws/src/nav2_applied_practice/resource
touch ~/nav2_study_ws/src/nav2_applied_practice/resource/nav2_applied_practice
touch ~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice/__init__.py
touch ~/nav2_study_ws/src/nav2_applied_practice/test/__init__.py
```

- [ ] **Step 2: `package.xml` 작성**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>nav2_applied_practice</name>
  <version>0.0.1</version>
  <description>rokey-I1-mini 차량 접근 프로젝트의 Nav2 API 패턴(액션 클라이언트, TF 변환, goal 재전송/취소)을 TB3 시뮬레이션에서 재현하는 학습 패키지</description>
  <maintainer email="hwangjeongui01@gmail.com">hwangjeongui</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: `setup.py` 작성**

```python
from glob import glob

from setuptools import find_packages, setup

package_name = 'nav2_applied_practice'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwangjeongui',
    maintainer_email='hwangjeongui01@gmail.com',
    description='rokey-I1-mini 차량 접근 프로젝트의 Nav2 API 패턴을 TB3 시뮬레이션에서 재현하는 학습 패키지',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'navigate_to_pose_client_node = '
            'nav2_applied_practice.navigate_to_pose_client_node:main',
            'tf_lookup_node = nav2_applied_practice.tf_lookup_node:main',
            'goal_resend_demo_node = nav2_applied_practice.goal_resend_demo_node:main',
        ],
    },
)
```

- [ ] **Step 4: `setup.cfg` 작성**

```ini
[develop]
script_dir=$base/lib/nav2_applied_practice
[install]
install_scripts=$base/lib/nav2_applied_practice
```

- [ ] **Step 5: `config/params.yaml` 뼈대 작성**

```yaml
# Run with:
#   ros2 run nav2_applied_practice navigate_to_pose_client_node --ros-args --params-file config/params.yaml
#   ros2 run nav2_applied_practice goal_resend_demo_node --ros-args --params-file config/params.yaml
navigate_to_pose_client_node:
  ros__parameters:
    # nav2_fundamentals가 번들 맵에서 검증한 빈 공간 좌표 중 하나를 예시 goal로 사용한다.
    goal_x: -1.5
    goal_y: 1.15
    goal_yaw: 0.0

goal_resend_demo_node:
  ros__parameters:
    goal_resend_threshold_m: 0.3
    approach_completion_threshold_m: 0.5
```

- [ ] **Step 6: 빌드로 스캐폴딩 검증**

```bash
cd ~/nav2_study_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select nav2_applied_practice
```

Expected: `Summary: 1 package finished`, 에러 없음.

- [ ] **Step 7: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_applied_practice
git commit -m "chore: scaffold nav2_applied_practice ament_python package"
```

---

### Task 2: `pose_utils.py` — waypoint → PoseStamped 변환 (재현 + TDD)

**Files:**
- Create: `~/nav2_study_ws/src/nav2_applied_practice/test/test_pose_utils.py`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice/pose_utils.py`

**Interfaces:**
- Produces: `waypoint_to_pose_stamped(x: float, y: float, yaw: float, frame_id: str = 'map') -> PoseStamped` — Task 3(`navigate_to_pose_client_node`)이 사용

재현 대상: `rokey-I1-mini/src/vehicle_mission/vehicle_mission/pose_utils.py` (동일 시그니처, 동일 구현)

- [ ] **Step 1: 실패하는 테스트 작성**

`~/nav2_study_ws/src/nav2_applied_practice/test/test_pose_utils.py`:

```python
import math

from nav2_applied_practice.pose_utils import waypoint_to_pose_stamped


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


def test_custom_frame_id():
    pose = waypoint_to_pose_stamped(0.0, 0.0, 0.0, frame_id='odom')
    assert pose.header.frame_id == 'odom'
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd ~/nav2_study_ws
colcon test --packages-select nav2_applied_practice --pytest-args -v 2>&1 | grep -A5 "test_pose_utils"
```

Expected: `ModuleNotFoundError` 또는 `ImportError` — `pose_utils` 모듈이 아직 없음.

- [ ] **Step 3: 최소 구현 작성**

`~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice/pose_utils.py`:

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

- [ ] **Step 4: 테스트 통과 확인**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_applied_practice
colcon test --packages-select nav2_applied_practice
colcon test-result --verbose
```

Expected: `test_pose_utils.py`의 4개 테스트 모두 PASS.

- [ ] **Step 5: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_applied_practice/nav2_applied_practice/pose_utils.py \
        src/nav2_applied_practice/test/test_pose_utils.py
git commit -m "feat(nav2_applied_practice): add pose_utils with pytest coverage"
```

---

### Task 3: `navigate_to_pose_client_node` — NavigateToPose 액션 클라이언트 최소 예제

**Files:**
- Create: `~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice/navigate_to_pose_client_node.py`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/docs/01_action_client.md`

**Interfaces:**
- Consumes: Task 2의 `waypoint_to_pose_stamped`
- Produces: 콘솔 스크립트 `navigate_to_pose_client_node` (Task 1의 `setup.py`에 이미 등록됨)

재현 대상: `rokey-I1-mini/src/vehicle_mission/vehicle_mission/vehicle_mission_node.py` — 이 프로젝트만의 부가 로직(웹캠 pose 구독, `/vehicle_approach/enable` 발행)은 제외하고, `ActionClient` → `wait_for_server()` → `send_goal_async()` → 거부/결과 처리 흐름만 추린다.

- [ ] **Step 1: `navigate_to_pose_client_node.py` 작성**

```python
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_applied_practice.pose_utils import waypoint_to_pose_stamped


class NavigateToPoseClientNode(Node):
    def __init__(self, action_client=None):
        super().__init__('navigate_to_pose_client_node')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)

        self._action_client = action_client or ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    def send_goal(self) -> None:
        x = self.get_parameter('goal_x').value
        y = self.get_parameter('goal_y').value
        yaw = self.get_parameter('goal_yaw').value
        pose = waypoint_to_pose_stamped(x, y, yaw)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f'sending goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
        self.get_logger().info('waiting for Nav2 navigate_to_pose action server...')
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal rejected by Nav2')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        self.get_logger().info(f'nav goal finished with status={result.status}')


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseClientNode()
    try:
        node.send_goal()
        rclpy.spin(node)
    finally:
        node._action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: `docs/01_action_client.md` 작성**

```markdown
# 01 — NavigateToPose 액션 클라이언트 최소 예제

## 재현 대상

`rokey-I1-mini/src/vehicle_mission/vehicle_mission/vehicle_mission_node.py`. 웹캠 초기 위치 구독, `/vehicle_approach/enable` 발행 같은 이 프로젝트만의 부가 로직은 뺐고, Nav2 액션 클라이언트의 핵심 흐름만 남겼다: 파라미터로 목표 pose 선언 → `ActionClient` 생성 → `wait_for_server()` → `send_goal_async()` → 거부/결과 콜백.

## 사전조건

`nav2_fundamentals`의 lesson01 스택이 실행 중이어야 한다:

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 launch nav2_fundamentals lesson01_bringup.launch.py
```

RViz에서 2D Pose Estimate로 초기 위치를 잡아둔다(그래야 Nav2가 activate 상태가 됨).

## 실행 명령

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 run nav2_applied_practice navigate_to_pose_client_node --ros-args \
  --params-file $(ros2 pkg prefix nav2_applied_practice)/share/nav2_applied_practice/config/params.yaml
```

## 관찰 포인트

1. 터미널 로그에 `sending goal: x=-1.500, y=1.150, yaw=0.000`이 찍히고, 로봇이 실제로 그 좌표로 이동하는 것을 RViz에서 확인한다.
2. `wait_for_server()`는 액션 서버가 뜰 때까지 **블로킹**한다 — Nav2 스택을 아직 안 띄우고 이 노드를 먼저 실행하면 로그가 `waiting for Nav2 navigate_to_pose action server...`에서 멈춘 채 대기하는 것을 확인해본다(먼저 실행해도 안전하다는 뜻이기도 하다).
3. 목적지 도착 후 터미널에 `nav goal finished with status=...`가 찍히는지 확인한다.

## 이해 확인 질문

- `wait_for_server()`(블로킹)와 이후 03(`goal_resend_demo_node`)에서 쓰는 `server_is_ready()`(논블로킹, 즉시 True/False 반환)는 왜 결과가 다를까? 각각 언제 쓰는 게 적절할까?
```

- [ ] **Step 3: 실행 검증**

nav2_fundamentals의 lesson01 스택이 이미 빌드되어 있다고 가정:

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_applied_practice
source install/setup.bash
ros2 launch nav2_fundamentals lesson01_bringup.launch.py
```

별도 터미널에서 2D Pose Estimate로 초기 위치를 잡은 뒤:

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 run nav2_applied_practice navigate_to_pose_client_node --ros-args \
  --params-file ~/nav2_study_ws/src/nav2_applied_practice/config/params.yaml
```

Expected: 로그에 goal 전송 메시지 출력, 로봇이 `(-1.5, 1.15)`로 이동, 도착 후 결과 상태 로그 출력.

- [ ] **Step 4: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_applied_practice/nav2_applied_practice/navigate_to_pose_client_node.py \
        src/nav2_applied_practice/docs/01_action_client.md
git commit -m "feat(nav2_applied_practice): add navigate_to_pose_client_node and doc"
```

---

### Task 4: `goal_calculation.py` — 재전송/완료 판정 순수 함수 (재현 + TDD)

**Files:**
- Create: `~/nav2_study_ws/src/nav2_applied_practice/test/test_goal_calculation.py`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice/goal_calculation.py`

**Interfaces:**
- Produces: `compute_goal_pose(vehicle_x, vehicle_y, robot_x, robot_y) -> PoseStamped`, `should_resend_goal(new_position, last_sent_position, threshold_m) -> bool`, `is_approach_complete(robot_x, robot_y, vehicle_x, vehicle_y, threshold_m) -> bool` — Task 6(`goal_resend_demo_node`)이 사용

재현 대상: `rokey-I1-mini/src/vehicle_approach/vehicle_approach/goal_calculation.py` (동일 시그니처, 동일 구현)

- [ ] **Step 1: 실패하는 테스트 작성**

`~/nav2_study_ws/src/nav2_applied_practice/test/test_goal_calculation.py`:

```python
import math

from nav2_applied_practice.goal_calculation import (
    compute_goal_pose,
    is_approach_complete,
    should_resend_goal,
)


def test_compute_goal_pose_position_is_vehicle_position():
    pose = compute_goal_pose(vehicle_x=3.0, vehicle_y=4.0, robot_x=0.0, robot_y=0.0)
    assert pose.header.frame_id == 'map'
    assert pose.pose.position.x == 3.0
    assert pose.pose.position.y == 4.0


def test_compute_goal_pose_yaw_faces_vehicle():
    pose = compute_goal_pose(vehicle_x=1.0, vehicle_y=1.0, robot_x=0.0, robot_y=0.0)
    expected_yaw = math.atan2(1.0, 1.0)
    assert math.isclose(pose.pose.orientation.z, math.sin(expected_yaw / 2.0), abs_tol=1e-9)
    assert math.isclose(pose.pose.orientation.w, math.cos(expected_yaw / 2.0), abs_tol=1e-9)


def test_should_resend_goal_below_threshold_is_false():
    assert should_resend_goal((1.0, 1.0), (1.02, 1.0), threshold_m=0.1) is False


def test_should_resend_goal_above_threshold_is_true():
    assert should_resend_goal((1.0, 1.0), (1.2, 1.0), threshold_m=0.1) is True


def test_should_resend_goal_exactly_at_threshold_is_true():
    assert should_resend_goal((1.1, 1.0), (1.0, 1.0), threshold_m=0.1) is True


def test_is_approach_complete_within_threshold():
    assert is_approach_complete(0.0, 0.0, 0.3, 0.0, threshold_m=0.5) is True


def test_is_approach_complete_outside_threshold():
    assert is_approach_complete(0.0, 0.0, 1.0, 0.0, threshold_m=0.5) is False


def test_is_approach_complete_exactly_at_threshold_is_true():
    assert is_approach_complete(0.0, 0.0, 0.5, 0.0, threshold_m=0.5) is True
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd ~/nav2_study_ws
colcon test --packages-select nav2_applied_practice --pytest-args -v 2>&1 | grep -A5 "test_goal_calculation"
```

Expected: `ModuleNotFoundError` — `goal_calculation` 모듈이 아직 없음.

- [ ] **Step 3: 최소 구현 작성**

`~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice/goal_calculation.py`:

```python
import math

from geometry_msgs.msg import PoseStamped


def compute_goal_pose(
    vehicle_x: float, vehicle_y: float, robot_x: float, robot_y: float
) -> PoseStamped:
    yaw = math.atan2(vehicle_y - robot_y, vehicle_x - robot_x)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = vehicle_x
    pose.pose.position.y = vehicle_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def should_resend_goal(
    new_position: tuple[float, float],
    last_sent_position: tuple[float, float],
    threshold_m: float,
) -> bool:
    dx = new_position[0] - last_sent_position[0]
    dy = new_position[1] - last_sent_position[1]
    return math.hypot(dx, dy) >= threshold_m


def is_approach_complete(
    robot_x: float, robot_y: float, vehicle_x: float, vehicle_y: float, threshold_m: float
) -> bool:
    return math.hypot(vehicle_x - robot_x, vehicle_y - robot_y) <= threshold_m
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_applied_practice
colcon test --packages-select nav2_applied_practice
colcon test-result --verbose
```

Expected: `test_goal_calculation.py`의 6개 테스트 모두 PASS.

- [ ] **Step 5: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_applied_practice/nav2_applied_practice/goal_calculation.py \
        src/nav2_applied_practice/test/test_goal_calculation.py
git commit -m "feat(nav2_applied_practice): add goal_calculation with pytest coverage"
```

---

### Task 5: `tf_lookup_node` — TF 기반 좌표 변환 예제

**Files:**
- Create: `~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice/tf_lookup_node.py`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/docs/02_tf_transform.md`

**Interfaces:**
- Consumes: 없음 (독립 노드, `tf2_ros`/`tf2_geometry_msgs`만 사용)
- Produces: 콘솔 스크립트 `tf_lookup_node` (Task 1의 `setup.py`에 이미 등록됨)

재현 대상: `rokey-I1-mini/src/vehicle_approach/vehicle_approach/pipeline.py`의 `tf_buffer.transform(point_camera, 'map')` 패턴과 `vehicle_approach_node.py`의 `_lookup_robot_position`(`tf_buffer.lookup_transform('map', base_frame, ...)`) 패턴을 하나로 합쳐 보여준다.

- [ ] **Step 1: `tf_lookup_node.py` 작성**

> **주의(실행 검증으로 확인된 사항):** `point_base_link.header.stamp`를 `self.get_clock().now()`(정확한 "지금" 시각)로 채우면, TF 버퍼에 그 정확한 시각의 샘플이 아직 없어 `transform()`이 `ExtrapolationException`으로 거의 항상 실패한다(시뮬레이션 시간과 실제 시간이 안 맞는 경우는 물론, `use_sim_time:=true`로 시간 축을 맞춰도 TF가 이산적인 주기로만 발행되기 때문에 여전히 레이스가 남는다 — 직접 재현 확인됨). 위쪽의 `lookup_transform('map', 'base_link', Time())` 호출처럼 `Time()`(빈 타임스탬프 = "가장 최근 것 달라")를 그대로 쓴다.

```python
import rclpy
import tf2_geometry_msgs  # noqa: F401 -- PointStamped 변환 등록을 위해 필요
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from tf2_ros import TransformListener


class TfLookupNode(Node):
    def __init__(self):
        super().__init__('tf_lookup_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self._on_timer)

    def _on_timer(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(f'map -> base_link TF 조회 실패: {exc}')
            return

        t = transform.transform.translation
        self.get_logger().info(f'robot pose in map: x={t.x:.3f}, y={t.y:.3f}')

        point_base_link = PointStamped()
        point_base_link.header.frame_id = 'base_link'
        point_base_link.header.stamp = Time().to_msg()
        point_base_link.point.x = 1.0
        point_base_link.point.y = 0.0
        point_base_link.point.z = 0.0

        try:
            point_map = self.tf_buffer.transform(point_base_link, 'map')
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(f'point 변환 실패: {exc}')
            return

        self.get_logger().info(
            'base_link 기준 1m 앞 지점의 map 좌표: '
            f'x={point_map.point.x:.3f}, y={point_map.point.y:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TfLookupNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: `docs/02_tf_transform.md` 작성**

```markdown
# 02 — TF 기반 좌표 변환 예제

## 재현 대상

- `rokey-I1-mini/src/vehicle_approach/vehicle_approach/vehicle_approach_node.py`의 `_lookup_robot_position`: `tf_buffer.lookup_transform('map', base_frame, ...)`로 로봇의 map 좌표를 얻는 패턴.
- `rokey-I1-mini/src/vehicle_approach/vehicle_approach/pipeline.py`의 `self.tf_buffer.transform(point_camera, 'map')`: 카메라 프레임의 점을 map 좌표로 변환하는 패턴. `tf2_geometry_msgs`를 import만 해두면(`# noqa: F401`) `PointStamped` 변환이 `Buffer.transform()`에 자동 등록된다 — 실제 프로젝트와 동일한 이유로 이 import가 필요하다.

## 사전조건

`nav2_fundamentals`의 lesson01 스택 실행 중 (TF 트리에 `map`→`odom`→`base_link`가 살아있어야 함):

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 launch nav2_fundamentals lesson01_bringup.launch.py
```

## 실행 명령

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 run nav2_applied_practice tf_lookup_node
```

## 관찰 포인트

1. RViz에서 2D Pose Estimate로 초기 위치를 잡기 **전**에 실행하면, `map -> base_link TF 조회 실패` 경고가 반복 출력되는 것을 확인한다 — AMCL이 아직 `map`→`odom` TF를 broadcast하지 않았기 때문이다.
2. 2D Pose Estimate로 초기 위치를 잡은 **후**에는 매초 `robot pose in map: x=..., y=...`와 `base_link 기준 1m 앞 지점의 map 좌표: x=..., y=...`가 출력되는 것을 확인한다.
3. 로봇을 teleop으로 움직이면서 두 로그 값이 실시간으로 갱신되는지 확인한다.

## 이해 확인 질문

- `lookup_transform`(변환 행렬 자체를 얻음)과 `transform`(특정 점을 다른 좌표계로 변환함)은 어떻게 다른가? `pipeline.py`는 왜 후자를 쓸까?
- 실제 프로젝트에서는 `camera_frame`(오크디)→`base_link`, `base_link`→`map` 두 TF가 연결되어 있어야 `pipeline.py`의 한 줄짜리 `transform()` 호출로 카메라 좌표가 곧바로 map 좌표로 바뀐다. 왜 두 TF를 수동으로 곱하지 않고 `tf2`가 자동으로 합성하게 두는 게 나을까?
```

- [ ] **Step 3: 실행 검증**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_applied_practice
source install/setup.bash
ros2 launch nav2_fundamentals lesson01_bringup.launch.py
```

별도 터미널:

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 run nav2_applied_practice tf_lookup_node
```

Expected: 초기 위치 지정 전엔 TF 조회 실패 경고, 지정 후엔 매초 로봇 map 좌표와 1m 앞 지점 map 좌표 로그 출력.

- [ ] **Step 4: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_applied_practice/nav2_applied_practice/tf_lookup_node.py \
        src/nav2_applied_practice/docs/02_tf_transform.md
git commit -m "feat(nav2_applied_practice): add tf_lookup_node and doc"
```

---

### Task 6: `goal_resend_demo_node` — goal 재전송/완료취소 로직 실습

**Files:**
- Create: `~/nav2_study_ws/src/nav2_applied_practice/nav2_applied_practice/goal_resend_demo_node.py`
- Create: `~/nav2_study_ws/src/nav2_applied_practice/docs/03_goal_resend_and_cancel.md`

**Interfaces:**
- Consumes: Task 4의 `compute_goal_pose`, `should_resend_goal`, `is_approach_complete`
- Produces: 콘솔 스크립트 `goal_resend_demo_node` (Task 1의 `setup.py`에 이미 등록됨)

재현 대상: `rokey-I1-mini/src/vehicle_approach/vehicle_approach/pipeline.py`의 `process_frame`(재전송 판정 + `_last_sent_goal` 갱신 시점)과 `vehicle_approach_node.py`의 `_send_goal`(`server_is_ready()` 논블로킹 체크)·`_on_synchronized`의 취소 로직(`goal_handle.cancel_goal_async()`)을 하나의 노드로 합쳐 재현한다. `/clicked_point`(RViz "Publish Point" 툴)를 실제 차량 탐지 결과 대신 쓴다.

- [ ] **Step 1: `goal_resend_demo_node.py` 작성**

```python
import rclpy
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from tf2_ros import TransformListener

from nav2_applied_practice.goal_calculation import (
    compute_goal_pose,
    is_approach_complete,
    should_resend_goal,
)


class GoalResendDemoNode(Node):
    def __init__(self, action_client=None):
        super().__init__('goal_resend_demo_node')

        self.declare_parameter('goal_resend_threshold_m', 0.3)
        self.declare_parameter('approach_completion_threshold_m', 0.5)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_client = action_client or ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self._last_sent_goal: tuple[float, float] | None = None
        self._current_goal_handle = None

        self.create_subscription(
            PointStamped, '/clicked_point', self._on_clicked_point, 10
        )

    def _robot_position(self) -> tuple[float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(f'map -> base_link TF 조회 실패: {exc}')
            return None
        t = transform.transform.translation
        return (t.x, t.y)

    def _on_clicked_point(self, msg: PointStamped) -> None:
        target = (msg.point.x, msg.point.y)
        robot_position = self._robot_position()
        if robot_position is None:
            return
        robot_x, robot_y = robot_position

        completion_threshold = self.get_parameter('approach_completion_threshold_m').value
        if is_approach_complete(robot_x, robot_y, target[0], target[1], completion_threshold):
            self._cancel_current_goal()
            self.get_logger().info('목표 지점 도착으로 판단 — goal 취소')
            return

        resend_threshold = self.get_parameter('goal_resend_threshold_m').value
        goal_pose = None
        if self._last_sent_goal is None or should_resend_goal(
            target, self._last_sent_goal, resend_threshold
        ):
            goal_pose = compute_goal_pose(target[0], target[1], robot_x, robot_y)
            # 재현 대상(pipeline.py)과 동일하게, "보내기로 결정"한 시점에
            # 곧바로 last_sent_goal을 갱신한다. 아래 _send_goal에서 실제 전송이
            # 스킵되더라도 이 값은 이미 갱신된 뒤다 (docs/03에서 다루는 갭).
            self._last_sent_goal = target

        if goal_pose is not None:
            self._send_goal(goal_pose)

    def _send_goal(self, goal_pose) -> None:
        if not self._action_client.server_is_ready():
            self.get_logger().warn('navigate_to_pose 서버가 아직 준비 안 됨 — goal 전송 스킵')
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.get_logger().info(
            f'goal 전송: x={goal_pose.pose.position.x:.3f}, y={goal_pose.pose.position.y:.3f}'
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('goal 거부됨 (Nav2)')
            return
        self._current_goal_handle = goal_handle

    def _cancel_current_goal(self) -> None:
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = GoalResendDemoNode()
    try:
        rclpy.spin(node)
    finally:
        node._action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: `docs/03_goal_resend_and_cancel.md` 작성**

```markdown
# 03 — goal 재전송/완료취소 로직 실습

## 재현 대상

- `rokey-I1-mini/src/vehicle_approach/vehicle_approach/pipeline.py`의 `process_frame`: `should_resend_goal`로 재전송 여부를 판정하고, 재전송하기로 **결정한 시점**에 곧바로 `self._last_sent_goal`을 갱신한다.
- `rokey-I1-mini/src/vehicle_approach/vehicle_approach/vehicle_approach_node.py`의 `_send_goal`: `server_is_ready()`(논블로킹 체크)가 `False`면 실제 전송(`send_goal_async`)을 하지 않고 그냥 건너뛴다.
- 같은 파일의 `_on_synchronized`: 접근 완료(`is_approach_complete`) 판정 시 진행 중이던 goal을 `cancel_goal_async()`로 취소한다.

이 노드는 웹캠/오크디 탐지 대신 RViz의 **"Publish Point"** 툴로 지도 위를 클릭하면 나오는 `/clicked_point`를 "감지된 목표 위치"로 사용한다.

## 알려진 갭 (의도적으로 재현, 고치지 않음)

`_on_clicked_point`를 보면 `_last_sent_goal`이 재전송 "결정" 시점에 갱신되고, 그 다음 `_send_goal`에서 `server_is_ready()`가 `False`면 실제 전송은 스킵된다. 이 경우 다음 클릭이 재전송 임계값(`goal_resend_threshold_m`)보다 가깝다면 재전송 자체가 다시는 시도되지 않는다 — goal이 사실상 영구히 드롭될 수 있다. `rokey-I1-mini/docs/NEXT_STEPS.md` §2에 기록된 실제 프로젝트의 보류 이슈와 정확히 같은 갭이다.

## 사전조건

`nav2_fundamentals`의 lesson01 스택 실행 중, RViz에서 2D Pose Estimate로 초기 위치 지정 완료.

## 실행 명령

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 run nav2_applied_practice goal_resend_demo_node --ros-args \
  --params-file ~/nav2_study_ws/src/nav2_applied_practice/config/params.yaml
```

## 관찰 포인트

1. RViz 상단 툴바에서 "Publish Point" 툴(십자 모양 아이콘)을 고른 뒤 지도 위 빈 공간을 클릭한다. 터미널에 `goal 전송: x=..., y=...` 로그가 뜨고 로봇이 그 지점으로 이동을 시작하는 것을 확인한다.
2. 로봇이 이동하는 도중, 방금 클릭한 지점에서 `goal_resend_threshold_m`(기본 0.3m)보다 가까운 다른 지점을 다시 클릭해본다 — `이전 목표와 충분히 가까움` 같은 재전송 스킵 로그 대신, 코드를 보면 알 수 있듯 `should_resend_goal`이 `False`를 반환해 아무 로그도 없이 조용히 무시되는 것을 확인한다(실제로는 재전송이 안 일어나는 게 정상 동작이다).
3. 로봇이 클릭 지점 `approach_completion_threshold_m`(기본 0.5m) 이내로 접근하면 `목표 지점 도착으로 판단 — goal 취소` 로그가 뜨고 로봇이 멈추는 것을 확인한다.
4. (갭 재현) Nav2 스택을 아직 안 띄운 상태에서 이 노드를 먼저 실행하고 지도 위를 클릭하면(테스트를 위해 `ros2 launch nav2_fundamentals lesson01_bringup.launch.py`를 잠깐 지연시켜 실행) `navigate_to_pose 서버가 아직 준비 안 됨 — goal 전송 스킵` 경고가 뜨는 것을 확인한다. 이후 Nav2가 완전히 뜬 뒤 같은 지점을 다시 클릭하지 않으면 이 goal은 영원히 전송되지 않는다 — 위에서 설명한 갭을 직접 재현한 것이다.

## 이해 확인 질문

- 이 갭을 고치려면 `_last_sent_goal` 갱신 시점을 "결정 시점"이 아니라 "실제 전송 성공 시점"으로 옮겨야 한다. 지금 구조(순수 함수 `should_resend_goal` + 노드의 상태 갱신)에서 그렇게 바꾸려면 어디를 손대야 할지 생각해보자.
- 실제 프로젝트(`NEXT_STEPS.md`)는 이 갭을 "3노드 시스템에서는 `vehicle_mission_node`가 먼저 Nav2를 검증해두므로 실무 위험이 낮다"고 판단하고 보류했다. 이번 데모처럼 Nav2가 없는 상태에서 바로 이 노드만 단독 실행하는 상황이라면 그 판단이 여전히 유효할까?
```

- [ ] **Step 3: 실행 검증**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_applied_practice
source install/setup.bash
ros2 launch nav2_fundamentals lesson01_bringup.launch.py
```

별도 터미널, 2D Pose Estimate로 초기 위치 지정 후:

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 run nav2_applied_practice goal_resend_demo_node --ros-args \
  --params-file ~/nav2_study_ws/src/nav2_applied_practice/config/params.yaml
```

Expected: RViz "Publish Point"로 클릭 시 goal 전송 로그, 로봇 이동, 근접 시 취소 로그 출력. `docs/03_goal_resend_and_cancel.md`의 4개 관찰 포인트를 모두 실제로 확인.

- [ ] **Step 4: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_applied_practice/nav2_applied_practice/goal_resend_demo_node.py \
        src/nav2_applied_practice/docs/03_goal_resend_and_cancel.md
git commit -m "feat(nav2_applied_practice): add goal_resend_demo_node and doc"
```

---

### Task 7: 전체 빌드 + 테스트 스위트 최종 확인

**Files:**
- 없음 (검증 전용 태스크)

**Interfaces:**
- Consumes: Task 1~6에서 만든 전체 패키지

- [ ] **Step 1: 전체 워크스페이스 빌드**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install
```

Expected: `Summary: 2 packages finished` (`nav2_fundamentals`, `nav2_applied_practice`), 에러 없음.

- [ ] **Step 2: pytest 전체 실행**

```bash
cd ~/nav2_study_ws
colcon test --packages-select nav2_applied_practice
colcon test-result --verbose
```

Expected: `test_pose_utils.py`(4개) + `test_goal_calculation.py`(8개) = 12개 테스트 전부 PASS, 실패 0건.

- [ ] **Step 3: 커밋 (변경 사항이 있는 경우에만)**

```bash
cd ~/nav2_study_ws
git status
```

새로 생성/수정된 파일이 없다면 커밋할 필요 없음 — 이 태스크는 검증 전용이다.
