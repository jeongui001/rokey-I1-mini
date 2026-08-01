# vehicle_approach_node Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `/vehicle_approach/enable`이 true가 된 후, 오크디 RGB-Depth-CameraInfo를 동기화해 차량을 탐지하고, 뎁스 보정 → 역투영+TF 변환 → 이동평균 → goal 계산을 거쳐 Nav2로 반복 접근하다가 임계 거리 이내에서 정지하는 `vehicle_approach_node`를 구현한다.

**Architecture:** 스펙 §5.2.1이 명시한 대로 동기화 콜백은 오케스트레이션만 담당하고, 뎁스 보정·역투영·이동평균·goal 계산은 각각 순수 함수/클래스로 분리한다. 이 4개 레이어를 묶는 `VehicleApproachPipeline`도 하드웨어에 직접 의존하지 않는 순수 클래스로 만들어(TF는 `tf2_ros.Buffer`를 주입받고, `set_transform_static`으로 알려진 변환을 미리 넣어두면 실제 TF 브로드캐스터 없이도 테스트 가능) pytest로 전 구간을 검증한다. `VehicleApproachNode`(rclpy)는 `message_filters.ApproximateTimeSynchronizer`·YOLO 추론·TF 리스너·Nav2 액션 클라이언트 배선만 담당하는 얇은 래퍼다.

**Tech Stack:** ROS2 rclpy (Humble 가정), `message_filters`, `cv_bridge`, `tf2_ros`/`tf2_geometry_msgs`, `nav2_msgs/action/NavigateToPose`, `ultralytics` YOLO, `numpy`, `pytest`.

## Global Constraints

- 대상 스펙: `docs/superpowers/specs/2026-08-01-vehicle-approach-design.md` §3.1(vehicle_approach_node), §3.2(enable 게이팅), §4(인터페이스), §5.2(1~7층), §5.2.1~5.2.5.
- 패키지는 `src/vehicle_approach/`에 ament_python으로 둔다. 다른 두 패키지(webcam_perception, vehicle_mission)와 코드 의존성이 없다 — 표준 메시지로만 런타임 통신하므로 이 플랜만 단독으로 빌드·테스트 가능하다. `detection.py`(Detection dataclass + confidence 기반 선택)는 `webcam_perception`의 것과 로직이 유사하지만, 패키지 독립성을 위해 별도로 둔다(3~5줄짜리 유틸을 위한 공용 라이브러리 패키지는 만들지 않는다 — YAGNI).
- YOLO 탐지는 오크디 온보드가 아니라 **호스트에서 RGB 스트림에 대해** 실행한다(스펙 §5.2.1) — 이 전제 때문에 4~5층에서 K 행렬 기반 수동 역투영이 필요하다.
- RGB·Depth·CameraInfo는 `message_filters.ApproximateTimeSynchronizer(slop=0.15)`로 동기화한다(스펙 §5.2.1). 동기화 콜백은 오케스트레이션만 하고, 뎁스 보정(§5.2.2)·역투영+TF 변환(§5.2.3)·이동평균(§5.2.4)·goal 계산(§5.2.5)은 각각 별도 함수/클래스로 분리해 호출한다 — 콜백에 5개 층 로직을 인라인으로 쌓지 않는다.
- 뎁스 보정식은 스펙 §5.2.2에 고정 값으로 주어져 있다(TBD 아님): `보정된_거리 = 0.795 × 측정_뎁스 + 0.131`, 0.6m 미만은 사용하지 않음(반환 `None`). 이 판정은 예외 처리가 아니라 정상 동작의 일부다.
- 역투영 공식(§5.2.3): `X=(u-cx)*z/fx, Y=(v-cy)*z/fy, Z=z` → camera_frame 기준 3D 좌표. 이후 `tf2_ros.Buffer.transform()` 한 번 호출로 map 좌표까지 자동 합성 변환한다(두 TF를 수동으로 곱하지 않는다).
- `/vehicle_approach/detection_center`(모니터링용, `geometry_msgs/Point`, 헤더 없음)는 **이동평균 이전, 매 프레임의 원시 map 좌표**를 발행한다 — 스펙 §4가 "이 값은 항상 최신 오크디 프레임 기준이라고 전제한다"고 명시하므로, 지연이 섞이는 이동평균값이 아니라 5층 출력(평균화 이전)을 쓴다. 실제 nav goal 계산에는 6층 출력(이동평균 이후)을 쓴다(§5.2.4~5.2.5).
- `/vehicle_approach/enable`(`std_msgs/Bool`, `transient_local`)이 true가 되기 전까지 동기화 콜백은 아무 처리도 하지 않는다(스펙 §3.2).
- goal 재전송은 변화량 임계치 이상일 때만 하고, 명시적 취소 없이 새 goal을 보낸다(Nav2가 자동 preempt, 스펙 §5.2.5 — 매 프레임 명시적 cancel 후 재전송은 race condition을 유발하므로 지양). 접근 완료 판정(로봇-차량 거리 ≤ 임계치) 시에만 명시적으로 `cancel_goal_async()`를 호출해 로봇을 정지시킨다.
- 스펙 §1.2/§8에 따라 **TF 조회 실패, 탐지 실패 등 예외 상황 처리는 범위 밖**이다 — 이 플랜은 그런 실패에 대한 try/except나 폴백을 추가하지 않는다(예: TF lookup 실패 시 예외가 그대로 전파되도록 둔다).
- 스펙 §7의 TBD 값(confidence threshold, 이동평균 윈도우 N, goal 재전송 임계치, 접근 완료 판정 거리 임계치)은 ROS2 파라미터로 노출하고 `config/params.yaml`에 임시 기본값 + `# TBD` 주석으로 표시한다.
- YOLO 런타임은 `ultralytics.YOLO`를 가정한다. `ultralytics`, `opencv-python`은 표준 rosdep 키가 없으므로 `package.xml`에 넣지 않고 `pip install`로 별도 설치가 필요함을 문서화한다(가정 사항).

---

### Task 1: 패키지 스캐폴드 + depth_correction + detection 유틸리티

**Files:**
- Create: `src/vehicle_approach/package.xml`
- Create: `src/vehicle_approach/setup.py`
- Create: `src/vehicle_approach/setup.cfg`
- Create: `src/vehicle_approach/resource/vehicle_approach`
- Create: `src/vehicle_approach/vehicle_approach/__init__.py`
- Create: `src/vehicle_approach/vehicle_approach/depth_correction.py`
- Create: `src/vehicle_approach/vehicle_approach/detection.py`
- Test: `src/vehicle_approach/test/test_depth_correction.py`
- Test: `src/vehicle_approach/test/test_detection.py`

**Interfaces:**
- Produces: `depth_correction.correct_depth(raw_depth_m: float) -> float | None`
- Produces: `detection.Detection` dataclass (`x1, y1, x2, y2, confidence: float`)
- Produces: `detection.select_best_detection(detections: list[Detection], confidence_threshold: float) -> Detection | None`
- Produces: `detection.bbox_center(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]`

- [ ] **Step 1: 패키지 스캐폴드 생성**

`src/vehicle_approach/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vehicle_approach</name>
  <version>0.0.1</version>
  <description>오크디 탐지 기반 뎁스보정/역투영/이동평균/goal계산으로 차량에 접근하는 노드</description>
  <maintainer email="hwangjeongui01@gmail.com">hwangjeongui</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>action_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>message_filters</depend>
  <depend>cv_bridge</depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`src/vehicle_approach/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'vehicle_approach'

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
    description='오크디 탐지 기반 뎁스보정/역투영/이동평균/goal계산으로 차량에 접근하는 노드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_approach_node = vehicle_approach.vehicle_approach_node:main',
        ],
    },
)
```

`src/vehicle_approach/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/vehicle_approach
[install]
install_scripts=$base/lib/vehicle_approach
```

`src/vehicle_approach/resource/vehicle_approach`: 빈 파일.

`src/vehicle_approach/vehicle_approach/__init__.py`: 빈 파일.

- [ ] **Step 2: 실패하는 테스트 작성 (depth_correction)**

`src/vehicle_approach/test/test_depth_correction.py`:
```python
import math

from vehicle_approach.depth_correction import correct_depth


def test_below_minimum_sensing_range_returns_none():
    assert correct_depth(0.5) is None


def test_at_minimum_sensing_range_is_corrected():
    result = correct_depth(0.6)
    assert result is not None
    assert math.isclose(result, 0.608, abs_tol=1e-4)


def test_known_sample_point_from_spec_validation_table():
    # 스펙 §5.2.2 검증표: 측정 0.84 -> 보정 0.799
    result = correct_depth(0.84)
    assert math.isclose(result, 0.799, abs_tol=1e-3)
```

- [ ] **Step 3: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_depth_correction.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_approach.depth_correction'`

- [ ] **Step 4: depth_correction.py 구현 (스펙 §5.2.2 의사코드 그대로)**

`src/vehicle_approach/vehicle_approach/depth_correction.py`:
```python
def correct_depth(raw_depth_m: float) -> float | None:
    if raw_depth_m < 0.6:
        return None  # 사용 안 함 (카메라 최소 센싱 거리 한계, 정상 동작)
    return 0.795 * raw_depth_m + 0.131
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_depth_correction.py -v`
Expected: PASS (3 passed)

- [ ] **Step 6: 실패하는 테스트 작성 (detection)**

`src/vehicle_approach/test/test_detection.py`:
```python
from vehicle_approach.detection import Detection, bbox_center, select_best_detection


def test_bbox_center():
    assert bbox_center(0.0, 0.0, 10.0, 20.0) == (5.0, 10.0)


def test_select_best_detection_returns_none_when_empty():
    assert select_best_detection([], confidence_threshold=0.5) is None


def test_select_best_detection_filters_below_threshold():
    detections = [Detection(x1=0.0, y1=0.0, x2=10.0, y2=10.0, confidence=0.3)]
    assert select_best_detection(detections, confidence_threshold=0.5) is None


def test_select_best_detection_picks_highest_confidence():
    low = Detection(x1=0.0, y1=0.0, x2=10.0, y2=10.0, confidence=0.6)
    high = Detection(x1=20.0, y1=20.0, x2=30.0, y2=30.0, confidence=0.9)
    result = select_best_detection([low, high], confidence_threshold=0.5)
    assert result is high
```

- [ ] **Step 7: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_detection.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_approach.detection'`

- [ ] **Step 8: detection.py 구현**

`src/vehicle_approach/vehicle_approach/detection.py`:
```python
from dataclasses import dataclass


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float


def bbox_center(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]:
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def select_best_detection(
    detections: list[Detection], confidence_threshold: float
) -> Detection | None:
    candidates = [d for d in detections if d.confidence >= confidence_threshold]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.confidence)
```

- [ ] **Step 9: 테스트 통과 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_detection.py -v`
Expected: PASS (4 passed)

- [ ] **Step 10: 커밋**

```bash
git add src/vehicle_approach/package.xml src/vehicle_approach/setup.py src/vehicle_approach/setup.cfg \
  src/vehicle_approach/resource/vehicle_approach src/vehicle_approach/vehicle_approach/__init__.py \
  src/vehicle_approach/vehicle_approach/depth_correction.py src/vehicle_approach/vehicle_approach/detection.py \
  src/vehicle_approach/test/test_depth_correction.py src/vehicle_approach/test/test_detection.py
git commit -m "feat(vehicle_approach): add package scaffold, depth correction and detection utils"
```

---

### Task 2: 역투영 (backprojection, 5층 전반부)

**Files:**
- Create: `src/vehicle_approach/vehicle_approach/backprojection.py`
- Test: `src/vehicle_approach/test/test_backprojection.py`

**Interfaces:**
- Produces: `backprojection.backproject(u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float) -> tuple[float, float, float]`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/vehicle_approach/test/test_backprojection.py`:
```python
import math

from vehicle_approach.backprojection import backproject


def test_backproject_principal_point_has_zero_lateral_offset():
    x, y, z = backproject(u=320.0, v=240.0, z=1.0, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert z == 1.0


def test_backproject_offset_pixel():
    x, y, z = backproject(u=420.0, v=240.0, z=2.0, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    assert math.isclose(x, 0.4, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert z == 2.0
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_backprojection.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_approach.backprojection'`

- [ ] **Step 3: backprojection.py 구현 (스펙 §5.2.3 공식 그대로)**

`src/vehicle_approach/vehicle_approach/backprojection.py`:
```python
def backproject(
    u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float
) -> tuple[float, float, float]:
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return (x, y, z)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_backprojection.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/vehicle_approach/vehicle_approach/backprojection.py src/vehicle_approach/test/test_backprojection.py
git commit -m "feat(vehicle_approach): add pinhole backprojection"
```

---

### Task 3: 이동평균 필터 (6층)

**Files:**
- Create: `src/vehicle_approach/vehicle_approach/moving_average.py`
- Test: `src/vehicle_approach/test/test_moving_average.py`

**Interfaces:**
- Produces: `moving_average.MovingAverageFilter(window_size: int)` — 메서드 `add(x: float, y: float) -> None`, `value() -> tuple[float, float]`, `reset() -> None`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/vehicle_approach/test/test_moving_average.py`:
```python
import math

from vehicle_approach.moving_average import MovingAverageFilter


def test_single_sample_returns_itself():
    filt = MovingAverageFilter(window_size=3)
    filt.add(1.0, 2.0)
    x, y = filt.value()
    assert math.isclose(x, 1.0)
    assert math.isclose(y, 2.0)


def test_averages_within_window():
    filt = MovingAverageFilter(window_size=3)
    filt.add(1.0, 1.0)
    filt.add(2.0, 2.0)
    filt.add(3.0, 3.0)
    x, y = filt.value()
    assert math.isclose(x, 2.0)
    assert math.isclose(y, 2.0)


def test_drops_oldest_sample_beyond_window():
    filt = MovingAverageFilter(window_size=2)
    filt.add(1.0, 1.0)
    filt.add(2.0, 2.0)
    filt.add(3.0, 3.0)
    x, y = filt.value()
    assert math.isclose(x, 2.5)
    assert math.isclose(y, 2.5)


def test_reset_clears_samples():
    filt = MovingAverageFilter(window_size=2)
    filt.add(1.0, 1.0)
    filt.reset()
    filt.add(5.0, 5.0)
    x, y = filt.value()
    assert math.isclose(x, 5.0)
    assert math.isclose(y, 5.0)
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_moving_average.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_approach.moving_average'`

- [ ] **Step 3: moving_average.py 구현**

`src/vehicle_approach/vehicle_approach/moving_average.py`:
```python
class MovingAverageFilter:
    def __init__(self, window_size: int):
        self.window_size = window_size
        self._samples: list[tuple[float, float]] = []

    def add(self, x: float, y: float) -> None:
        self._samples.append((x, y))
        if len(self._samples) > self.window_size:
            self._samples.pop(0)

    def value(self) -> tuple[float, float]:
        xs = [s[0] for s in self._samples]
        ys = [s[1] for s in self._samples]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    def reset(self) -> None:
        self._samples = []
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_moving_average.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/vehicle_approach/vehicle_approach/moving_average.py src/vehicle_approach/test/test_moving_average.py
git commit -m "feat(vehicle_approach): add sliding-window moving average filter"
```

---

### Task 4: goal 계산 (7층 — 방향/재전송/완료 판정)

**Files:**
- Create: `src/vehicle_approach/vehicle_approach/goal_calculation.py`
- Test: `src/vehicle_approach/test/test_goal_calculation.py`

**Interfaces:**
- Produces: `goal_calculation.compute_goal_pose(vehicle_x: float, vehicle_y: float, robot_x: float, robot_y: float) -> geometry_msgs.msg.PoseStamped`
- Produces: `goal_calculation.should_resend_goal(new_position: tuple[float, float], last_sent_position: tuple[float, float], threshold_m: float) -> bool`
- Produces: `goal_calculation.is_approach_complete(robot_x: float, robot_y: float, vehicle_x: float, vehicle_y: float, threshold_m: float) -> bool`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/vehicle_approach/test/test_goal_calculation.py`:
```python
import math

from vehicle_approach.goal_calculation import (
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


def test_is_approach_complete_within_threshold():
    assert is_approach_complete(0.0, 0.0, 0.3, 0.0, threshold_m=0.5) is True


def test_is_approach_complete_outside_threshold():
    assert is_approach_complete(0.0, 0.0, 1.0, 0.0, threshold_m=0.5) is False
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_goal_calculation.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_approach.goal_calculation'`

- [ ] **Step 3: goal_calculation.py 구현 (스펙 §5.2.5)**

`src/vehicle_approach/vehicle_approach/goal_calculation.py`:
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

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_goal_calculation.py -v`
Expected: PASS (6 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/vehicle_approach/vehicle_approach/goal_calculation.py src/vehicle_approach/test/test_goal_calculation.py
git commit -m "feat(vehicle_approach): add goal pose, resend and completion judgment"
```

---

### Task 5: VehicleApproachPipeline (순수 오케스트레이션, TF 포함)

**Files:**
- Create: `src/vehicle_approach/vehicle_approach/pipeline.py`
- Test: `src/vehicle_approach/test/test_pipeline.py`

**Interfaces:**
- Consumes: `detection.Detection`, `detection.bbox_center`, `detection.select_best_detection`, `depth_correction.correct_depth`, `backprojection.backproject`, `moving_average.MovingAverageFilter`, `goal_calculation.compute_goal_pose/should_resend_goal/is_approach_complete`, `tf2_ros.Buffer`
- Produces: `pipeline.ApproachResult` dataclass (`goal_pose: PoseStamped | None, detection_center: Point | None, completed: bool`)
- Produces: `pipeline.VehicleApproachPipeline(confidence_threshold, tf_buffer, camera_frame, moving_average, resend_threshold_m, completion_threshold_m)` — 메서드 `process_frame(detections, depth_image, fx, fy, cx, cy, stamp, robot_x, robot_y) -> ApproachResult`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/vehicle_approach/test/test_pipeline.py`:
```python
from builtin_interfaces.msg import Time

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer

from vehicle_approach.detection import Detection
from vehicle_approach.moving_average import MovingAverageFilter
from vehicle_approach.pipeline import VehicleApproachPipeline


def _make_buffer() -> Buffer:
    buffer = Buffer()

    # 로봇(base_link)이 map (2, 0)에 위치
    base_to_map = TransformStamped()
    base_to_map.header.frame_id = 'map'
    base_to_map.child_frame_id = 'base_link'
    base_to_map.transform.translation.x = 2.0
    base_to_map.transform.translation.y = 0.0
    base_to_map.transform.rotation.w = 1.0
    buffer.set_transform_static(base_to_map, 'test')

    # 카메라 광학축(Z, 정면)을 base_link의 X(정면)에 맞추는 Y축 90도 회전, 위치는 base_link 원점과 동일
    camera_to_base = TransformStamped()
    camera_to_base.header.frame_id = 'base_link'
    camera_to_base.child_frame_id = 'camera_frame'
    camera_to_base.transform.rotation.y = 0.70710678
    camera_to_base.transform.rotation.w = 0.70710678
    buffer.set_transform_static(camera_to_base, 'test')

    return buffer


def _make_pipeline(resend_threshold_m=0.05, completion_threshold_m=0.1, window_size=1):
    return VehicleApproachPipeline(
        confidence_threshold=0.5,
        tf_buffer=_make_buffer(),
        camera_frame='camera_frame',
        moving_average=MovingAverageFilter(window_size=window_size),
        resend_threshold_m=resend_threshold_m,
        completion_threshold_m=completion_threshold_m,
    )


def _stamp() -> Time:
    return Time(sec=0, nanosec=0)


def test_no_detection_returns_no_goal():
    pipeline = _make_pipeline()
    depth_image = np.full((480, 640), 2000, dtype=np.uint16)
    result = pipeline.process_frame(
        [], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )
    assert result.goal_pose is None
    assert result.detection_center is None
    assert result.completed is False


def test_depth_below_minimum_range_returns_no_goal():
    pipeline = _make_pipeline()
    depth_image = np.full((480, 640), 500, dtype=np.uint16)  # 0.5m < 0.6m 최소 센싱 거리
    detection = Detection(x1=300.0, y1=220.0, x2=340.0, y2=260.0, confidence=0.9)
    result = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )
    assert result.goal_pose is None
    assert result.detection_center is None


def test_detected_vehicle_publishes_center_and_sends_goal_facing_it():
    pipeline = _make_pipeline(resend_threshold_m=0.05, completion_threshold_m=0.1)
    depth_image = np.full((480, 640), 2000, dtype=np.uint16)  # raw 2.0m -> 보정 1.721m
    detection = Detection(x1=300.0, y1=220.0, x2=340.0, y2=260.0, confidence=0.9)  # bbox 중심 == 주점(320,240)

    result = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )

    # 카메라 정면 1.721m -> base_link X축 +1.721m -> map (2+1.721, 0) = (3.721, 0)
    assert result.detection_center is not None
    assert result.detection_center.x == pytest.approx(3.721, abs=1e-3)
    assert result.detection_center.y == pytest.approx(0.0, abs=1e-6)

    assert result.goal_pose is not None
    assert result.goal_pose.pose.position.x == pytest.approx(3.721, abs=1e-3)
    assert result.goal_pose.pose.orientation.z == pytest.approx(0.0, abs=1e-6)
    assert result.goal_pose.pose.orientation.w == pytest.approx(1.0, abs=1e-6)
    assert result.completed is False


def test_repeated_same_position_does_not_resend_goal():
    pipeline = _make_pipeline(resend_threshold_m=0.05, completion_threshold_m=0.1)
    depth_image = np.full((480, 640), 2000, dtype=np.uint16)
    detection = Detection(x1=300.0, y1=220.0, x2=340.0, y2=260.0, confidence=0.9)

    first = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )
    second = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )

    assert first.goal_pose is not None
    assert second.goal_pose is None  # 변화량이 재전송 임계치 미만이면 재전송하지 않음 (스펙 §5.2.5)


def test_close_distance_marks_completed_and_stops_sending_goal():
    pipeline = _make_pipeline(completion_threshold_m=3.0)  # 실제 거리(1.721m)보다 큰 임계치
    depth_image = np.full((480, 640), 2000, dtype=np.uint16)
    detection = Detection(x1=300.0, y1=220.0, x2=340.0, y2=260.0, confidence=0.9)

    result = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )

    assert result.completed is True
    assert result.goal_pose is None
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_pipeline.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_approach.pipeline'`

- [ ] **Step 3: pipeline.py 구현**

`src/vehicle_approach/vehicle_approach/pipeline.py`:
```python
from dataclasses import dataclass

import numpy as np
import tf2_geometry_msgs  # noqa: F401 -- PointStamped 변환 등록을 위해 필요
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from tf2_ros import Buffer

from vehicle_approach.backprojection import backproject
from vehicle_approach.depth_correction import correct_depth
from vehicle_approach.detection import Detection, bbox_center, select_best_detection
from vehicle_approach.goal_calculation import (
    compute_goal_pose,
    is_approach_complete,
    should_resend_goal,
)
from vehicle_approach.moving_average import MovingAverageFilter


@dataclass
class ApproachResult:
    goal_pose: PoseStamped | None
    detection_center: Point | None
    completed: bool


class VehicleApproachPipeline:
    def __init__(
        self,
        confidence_threshold: float,
        tf_buffer: Buffer,
        camera_frame: str,
        moving_average: MovingAverageFilter,
        resend_threshold_m: float,
        completion_threshold_m: float,
    ):
        self.confidence_threshold = confidence_threshold
        self.tf_buffer = tf_buffer
        self.camera_frame = camera_frame
        self.moving_average = moving_average
        self.resend_threshold_m = resend_threshold_m
        self.completion_threshold_m = completion_threshold_m
        self._last_sent_goal: tuple[float, float] | None = None

    def process_frame(
        self,
        detections: list[Detection],
        depth_image: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        stamp,
        robot_x: float,
        robot_y: float,
    ) -> ApproachResult:
        best = select_best_detection(detections, self.confidence_threshold)
        if best is None:
            return ApproachResult(goal_pose=None, detection_center=None, completed=False)

        u, v = bbox_center(best.x1, best.y1, best.x2, best.y2)
        raw_depth_m = float(depth_image[int(round(v)), int(round(u))]) / 1000.0
        corrected = correct_depth(raw_depth_m)
        if corrected is None:
            return ApproachResult(goal_pose=None, detection_center=None, completed=False)

        x, y, z = backproject(u, v, corrected, fx, fy, cx, cy)
        point_camera = PointStamped()
        point_camera.header.frame_id = self.camera_frame
        point_camera.header.stamp = stamp
        point_camera.point.x = x
        point_camera.point.y = y
        point_camera.point.z = z
        point_map = self.tf_buffer.transform(point_camera, 'map')

        # 모니터링용 detection_center는 이동평균 이전, 매 프레임 원시 값 (스펙 §4)
        detection_center = Point(x=point_map.point.x, y=point_map.point.y, z=0.0)

        self.moving_average.add(point_map.point.x, point_map.point.y)
        avg_x, avg_y = self.moving_average.value()

        if is_approach_complete(robot_x, robot_y, avg_x, avg_y, self.completion_threshold_m):
            return ApproachResult(goal_pose=None, detection_center=detection_center, completed=True)

        goal_pose = None
        if self._last_sent_goal is None or should_resend_goal(
            (avg_x, avg_y), self._last_sent_goal, self.resend_threshold_m
        ):
            goal_pose = compute_goal_pose(avg_x, avg_y, robot_x, robot_y)
            self._last_sent_goal = (avg_x, avg_y)

        return ApproachResult(goal_pose=goal_pose, detection_center=detection_center, completed=False)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_pipeline.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/vehicle_approach/vehicle_approach/pipeline.py src/vehicle_approach/test/test_pipeline.py
git commit -m "feat(vehicle_approach): add VehicleApproachPipeline orchestration with TF transform"
```

---

### Task 6: VehicleApproachNode (rclpy 래퍼) + 파라미터 파일

**Files:**
- Create: `src/vehicle_approach/vehicle_approach/vehicle_approach_node.py`
- Create: `src/vehicle_approach/config/params.yaml`
- Test: `src/vehicle_approach/test/test_vehicle_approach_node.py`

**Interfaces:**
- Consumes: `pipeline.VehicleApproachPipeline`, `pipeline.ApproachResult`, `detection.Detection`, `moving_average.MovingAverageFilter`
- Produces: `vehicle_approach_node.VehicleApproachNode(detector=None, action_client=None)` (rclpy.Node), 진입점 `vehicle_approach_node.main()`

- [ ] **Step 1: 실패하는 노드 테스트 작성**

`src/vehicle_approach/test/test_vehicle_approach_node.py`:
```python
from types import SimpleNamespace

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


class _FakeActionClient:
    def __init__(self):
        self.sent_goals = []

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        future = rclpy.task.Future()
        future.set_result(SimpleNamespace(accepted=True))
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
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_vehicle_approach_node.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_approach.vehicle_approach_node'`

- [ ] **Step 3: vehicle_approach_node.py 구현**

`src/vehicle_approach/vehicle_approach/vehicle_approach_node.py`:
```python
import rclpy
import tf2_geometry_msgs  # noqa: F401
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from vehicle_approach.detection import Detection
from vehicle_approach.moving_average import MovingAverageFilter
from vehicle_approach.pipeline import VehicleApproachPipeline

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - 배포 환경에서만 필요
    YOLO = None


ENABLE_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class VehicleApproachNode(Node):
    def __init__(self, detector=None, action_client=None):
        super().__init__('vehicle_approach_node')

        self.declare_parameter('yolo_weights_path', '')
        self.declare_parameter('vehicle_class_id', 0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('moving_average_window', 5)
        self.declare_parameter('goal_resend_threshold_m', 0.1)
        self.declare_parameter('approach_completion_threshold_m', 0.5)
        self.declare_parameter('rgb_topic', '/oakd/rgb/image_raw')
        self.declare_parameter('depth_topic', '/oakd/stereo/image_raw')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/camera_info')

        self._enabled = False
        self.create_subscription(Bool, '/vehicle_approach/enable', self._on_enable, ENABLE_QOS)

        self._detection_center_publisher = self.create_publisher(
            Point, '/vehicle_approach/detection_center', 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pipeline = VehicleApproachPipeline(
            confidence_threshold=self.get_parameter('confidence_threshold').value,
            tf_buffer=self._tf_buffer,
            camera_frame=self.get_parameter('camera_frame').value,
            moving_average=MovingAverageFilter(
                window_size=self.get_parameter('moving_average_window').value
            ),
            resend_threshold_m=self.get_parameter('goal_resend_threshold_m').value,
            completion_threshold_m=self.get_parameter('approach_completion_threshold_m').value,
        )

        self._vehicle_class_id = self.get_parameter('vehicle_class_id').value
        if detector is None:
            detector = YOLO(self.get_parameter('yolo_weights_path').value)
        self._detector = detector
        self._bridge = CvBridge()

        self._action_client = action_client or ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self._current_goal_handle = None

        rgb_sub = Subscriber(self, Image, self.get_parameter('rgb_topic').value)
        depth_sub = Subscriber(self, Image, self.get_parameter('depth_topic').value)
        info_sub = Subscriber(self, CameraInfo, self.get_parameter('camera_info_topic').value)
        self._sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.15
        )
        self._sync.registerCallback(self._on_synchronized)

    def _on_enable(self, msg: Bool) -> None:
        self._enabled = msg.data

    def _on_synchronized(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        # 오케스트레이션만 담당한다 -- 뎁스보정/역투영+TF/이동평균/goal계산은
        # VehicleApproachPipeline과 그 내부 함수들이 각각 수행한다 (스펙 §5.2.1)
        if not self._enabled:
            return

        detections = self._run_detector(rgb_msg)
        depth_image = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        fx, fy, cx, cy = info_msg.k[0], info_msg.k[4], info_msg.k[2], info_msg.k[5]
        robot_x, robot_y = self._lookup_robot_position()

        result = self._pipeline.process_frame(
            detections, depth_image, fx, fy, cx, cy,
            stamp=rgb_msg.header.stamp,
            robot_x=robot_x, robot_y=robot_y,
        )

        if result.detection_center is not None:
            self._detection_center_publisher.publish(result.detection_center)

        if result.completed:
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle = None
            return

        if result.goal_pose is not None:
            self._send_goal(result.goal_pose)

    def _run_detector(self, rgb_msg: Image) -> list[Detection]:
        frame = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
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

    def _lookup_robot_position(self) -> tuple[float, float]:
        base_frame = self.get_parameter('base_frame').value
        transform = self._tf_buffer.lookup_transform('map', base_frame, rclpy.time.Time())
        return (transform.transform.translation.x, transform.transform.translation.y)

    def _send_goal(self, goal_pose) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle.accepted:
            self._current_goal_handle = goal_handle


def main(args=None):
    rclpy.init(args=args)
    node = VehicleApproachNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: config/params.yaml 작성**

`src/vehicle_approach/config/params.yaml`:
```yaml
vehicle_approach_node:
  ros__parameters:
    yolo_weights_path: "/absolute/path/to/oakd_yolo_weights.pt"
    vehicle_class_id: 0
    camera_frame: "camera_frame"
    base_frame: "base_link"
    rgb_topic: "/oakd/rgb/image_raw"
    depth_topic: "/oakd/stereo/image_raw"
    camera_info_topic: "/oakd/rgb/camera_info"

    # 아래 값들은 스펙 §7 TBD 항목 — 실측 캘리브레이션 전 임시 기본값이다.
    confidence_threshold: 0.5             # TBD: YOLO confidence threshold (웹캠과 공용 정책)
    moving_average_window: 5              # TBD: 이동평균 슬라이딩 윈도우 크기 N
    goal_resend_threshold_m: 0.1          # TBD: goal 재전송 변화량 임계치(m)
    approach_completion_threshold_m: 0.5  # TBD: 접근 완료 판정 거리 임계치(m) — Nav2 xy_goal_tolerance보다 충분히 커야 함(스펙 §5.2.5)
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `PYTHONPATH=src/vehicle_approach python3 -m pytest src/vehicle_approach/test/test_vehicle_approach_node.py -v`
Expected: PASS (2 passed)

- [ ] **Step 6: colcon 빌드/테스트로 패키지 전체 검증**

Run:
```bash
cd <workspace_root>
colcon build --packages-select vehicle_approach --symlink-install
colcon test --packages-select vehicle_approach
colcon test-result --verbose
```
Expected: 빌드 성공, 이전 6개 태스크의 모든 테스트(depth_correction 3, detection 4, backprojection 2, moving_average 4, goal_calculation 6, pipeline 5, node 2 = 총 26개) PASS.

- [ ] **Step 7: 커밋**

```bash
git add src/vehicle_approach/vehicle_approach/vehicle_approach_node.py \
  src/vehicle_approach/config/params.yaml \
  src/vehicle_approach/test/test_vehicle_approach_node.py \
  src/vehicle_approach/setup.py
git commit -m "feat(vehicle_approach): add rclpy node wrapper and params file"
```

- [ ] **Step 8 (선택, 하드웨어 필요 — 수동 검증):**

실제 오크디 + Nav2 스택이 기동된 환경에서:
```bash
ros2 run vehicle_approach vehicle_approach_node --ros-args --params-file src/vehicle_approach/config/params.yaml
ros2 topic pub /vehicle_approach/enable std_msgs/msg/Bool "{data: true}" --once
ros2 topic echo /vehicle_approach/detection_center
```
차량 앞에서 로봇이 반복적으로 goal을 갱신하며 접근하다가, 접근 완료 임계치 이내에서 정지하는지 육안으로 확인한다. 이 절차는 스펙 §1.2/§8에 따라 자동화 테스트 범위 밖이다.

---

## Self-Review 메모 (플랜 작성자용, 실행 시 삭제 가능)

- **스펙 커버리지**: §3.1(vehicle_approach_node 역할) → Task 6, §3.2(enable 게이팅) → Task 6, §4(detection_center/Nav2 액션 인터페이스) → Task 5·6, §5.2.1(호스트 YOLO, 동기화, 오케스트레이션 분리) → Task 6, §5.2.2(뎁스 보정) → Task 1, §5.2.3(역투영+TF) → Task 2·5, §5.2.4(이동평균) → Task 3, §5.2.5(goal 계산/재전송/완료판정) → Task 4·5. 모두 커버.
- 실제 오크디/Nav2 하드웨어 검증, 실측 캘리브레이션 값 확정, TF/탐지 실패 처리는 스펙 §1.2/§8에서 명시적으로 범위 밖이므로 이 플랜에도 포함하지 않았다 — `_lookup_robot_position`이 TF 조회 실패 시 예외를 그대로 전파하는 것은 의도된 설계다.
