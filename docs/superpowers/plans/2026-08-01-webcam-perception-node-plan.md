# webcam_perception_node Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 웹캠 ROI를 상시 감시하다가 차량이 들어와 정지하면, 픽셀→map 변환으로 위치를 계산해 `/webcam/vehicle_initial_pose`로 1회 발행하는 `webcam_perception_node`를 구현한다.

**Architecture:** ROI 판정·정지 판정·호모그래피 변환을 각각 순수 함수/클래스로 분리하고(하드웨어·ROS 의존성 없음, pytest로 완전 검증), 이를 `VehicleStopPipeline`이라는 순수 오케스트레이션 클래스로 묶는다. `WebcamPerceptionNode`(rclpy)는 카메라 캡처·YOLO 추론·토픽 발행만 담당하는 얇은 래퍼로, `capture`/`detector`를 생성자 인자로 주입 가능하게 해 하드웨어 없이도 노드 배선을 테스트한다.

**Tech Stack:** ROS2 rclpy (Humble 가정 — 다른 배포판이면 QoS/타이머 API 확인 필요), Python 3, OpenCV(`cv2`), `ultralytics` YOLO, `pytest`, `numpy`.

## Global Constraints

- 대상 스펙: `docs/superpowers/specs/2026-08-01-vehicle-approach-design.md` §3.1(webcam_perception_node), §5.1(W1~W3층), §4(인터페이스).
- 워크스페이스 루트는 이 저장소 루트이며, 패키지는 `src/webcam_perception/`에 ament_python 패키지로 둔다(`.vscode/colcon.env` 존재로 colcon 워크스페이스로 판단, `src/` 디렉터리는 아직 없으므로 이 플랜이 최초 생성).
- 발행 토픽 `/webcam/vehicle_initial_pose`는 `geometry_msgs/PointStamped`, `frame_id="map"`, QoS `transient_local`, `depth=1` — 스펙 §4 그대로 적용. 이 값은 로깅용이며 로봇 이동 목표 계산에는 쓰이지 않는다(스펙 §1.3).
- 정지 판정은 프레임 수가 아니라 경과 시간(T초) 기준(스펙 §5.1) — `StopDetector`는 타임스탬프 기반으로 구현한다.
- 다중 탐지 시 confidence가 가장 높은 탐지를 선택한다(스펙 §5.1). confidence threshold는 TBD.
- 픽셀→map 변환(W3)에는 바운딩박스의 **하단 중심**을 쓰고, 정지 판정(W2)의 추적점은 바운딩박스 **중심**을 쓴다 — 스펙 §5.1이 두 레이어에서 서로 다른 기준점을 명시하므로 구분해서 구현한다.
- "위치 변화가 픽셀 임계치 이내"의 구체적 계산식은 스펙에 명시되어 있지 않다. 이 플랜은 슬라이딩 윈도우 내 x/y 각각의 (최댓값-최솟값) spread가 임계치 이하인지로 정의한다(설계 결정, §7 TBD 값과는 별개로 알고리즘 형태 자체는 지금 확정 필요).
- 스펙 §7의 TBD 값(정지판정 T초/픽셀임계치, confidence threshold, 호모그래피 대응점)은 ROS2 파라미터로 노출하고 `config/params.yaml`에 임시 기본값 + `# TBD` 주석으로 표시한다. 실측 캘리브레이션 후 이 파일만 갱신하면 되도록 코드에는 하드코딩하지 않는다.
- YOLO 런타임은 `ultralytics.YOLO`를 가정한다(모델 학습·클래스 정의는 스펙 §8에서 범위 밖이므로 가중치 파일 경로만 파라미터로 받는다).
- 이 플랜은 다른 두 플랜(vehicle_mission_node, vehicle_approach_node)과 코드 의존성이 없다 — 오직 표준 메시지(`geometry_msgs/PointStamped`)로만 런타임 통신하므로 이 플랜만 단독으로 빌드·테스트 가능하다.

---

### Task 1: 패키지 스캐폴드 + geometry/detection 유틸리티

**Files:**
- Create: `src/webcam_perception/package.xml`
- Create: `src/webcam_perception/setup.py`
- Create: `src/webcam_perception/setup.cfg`
- Create: `src/webcam_perception/resource/webcam_perception`
- Create: `src/webcam_perception/webcam_perception/__init__.py`
- Create: `src/webcam_perception/webcam_perception/geometry.py`
- Create: `src/webcam_perception/webcam_perception/detection.py`
- Test: `src/webcam_perception/test/test_geometry.py`
- Test: `src/webcam_perception/test/test_detection.py`

**Interfaces:**
- Produces: `geometry.bbox_center(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]`
- Produces: `geometry.bbox_bottom_center(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]`
- Produces: `geometry.point_in_roi(point: tuple[float, float], roi: tuple[float, float, float, float]) -> bool`
- Produces: `detection.Detection` dataclass (`x1, y1, x2, y2, confidence: float`)
- Produces: `detection.select_best_detection(detections: list[Detection], confidence_threshold: float) -> Detection | None`

- [ ] **Step 1: 패키지 스캐폴드 생성**

`src/webcam_perception/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>webcam_perception</name>
  <version>0.0.1</version>
  <description>웹캠 ROI 감시, 정지 판정, 픽셀→map 변환 후 차량 초기 위치 발행</description>
  <maintainer email="hwangjeongui01@gmail.com">hwangjeongui</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`src/webcam_perception/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'webcam_perception'

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
    description='웹캠 ROI 감시, 정지 판정, 픽셀→map 변환 후 차량 초기 위치 발행',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_perception_node = webcam_perception.webcam_perception_node:main',
        ],
    },
)
```

`src/webcam_perception/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/webcam_perception
[install]
install_scripts=$base/lib/webcam_perception
```

`src/webcam_perception/resource/webcam_perception`: 빈 파일(패키지 인덱스 마커).

`src/webcam_perception/webcam_perception/__init__.py`: 빈 파일.

- [ ] **Step 2: geometry.py 실패하는 테스트 작성**

`src/webcam_perception/test/test_geometry.py`:
```python
from webcam_perception.geometry import bbox_bottom_center, bbox_center, point_in_roi


def test_bbox_center():
    assert bbox_center(0.0, 0.0, 10.0, 20.0) == (5.0, 10.0)


def test_bbox_bottom_center():
    assert bbox_bottom_center(0.0, 0.0, 10.0, 20.0) == (5.0, 20.0)


def test_point_in_roi_inside():
    assert point_in_roi((5.0, 5.0), (0.0, 0.0, 10.0, 10.0)) is True


def test_point_in_roi_outside():
    assert point_in_roi((15.0, 5.0), (0.0, 0.0, 10.0, 10.0)) is False


def test_point_in_roi_boundary_is_inside():
    assert point_in_roi((0.0, 0.0), (0.0, 0.0, 10.0, 10.0)) is True
```

- [ ] **Step 3: 테스트 실패 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_geometry.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'webcam_perception.geometry'`

- [ ] **Step 4: geometry.py 구현**

`src/webcam_perception/webcam_perception/geometry.py`:
```python
def bbox_center(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]:
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def bbox_bottom_center(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]:
    return ((x1 + x2) / 2.0, y2)


def point_in_roi(
    point: tuple[float, float], roi: tuple[float, float, float, float]
) -> bool:
    x, y = point
    x_min, y_min, x_max, y_max = roi
    return x_min <= x <= x_max and y_min <= y <= y_max
```

- [ ] **Step 5: geometry.py 테스트 통과 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_geometry.py -v`
Expected: PASS (5 passed)

- [ ] **Step 6: detection.py 실패하는 테스트 작성**

`src/webcam_perception/test/test_detection.py`:
```python
from webcam_perception.detection import Detection, select_best_detection


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

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_detection.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'webcam_perception.detection'`

- [ ] **Step 8: detection.py 구현**

`src/webcam_perception/webcam_perception/detection.py`:
```python
from dataclasses import dataclass


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float


def select_best_detection(
    detections: list[Detection], confidence_threshold: float
) -> Detection | None:
    candidates = [d for d in detections if d.confidence >= confidence_threshold]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.confidence)
```

- [ ] **Step 9: detection.py 테스트 통과 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_detection.py -v`
Expected: PASS (3 passed)

- [ ] **Step 10: 커밋**

```bash
git add src/webcam_perception/package.xml src/webcam_perception/setup.py src/webcam_perception/setup.cfg \
  src/webcam_perception/resource/webcam_perception src/webcam_perception/webcam_perception/__init__.py \
  src/webcam_perception/webcam_perception/geometry.py src/webcam_perception/webcam_perception/detection.py \
  src/webcam_perception/test/test_geometry.py src/webcam_perception/test/test_detection.py
git commit -m "feat(webcam_perception): add package scaffold and bbox/detection utils"
```

---

### Task 2: 시간 기반 정지 판정(StopDetector)

**Files:**
- Create: `src/webcam_perception/webcam_perception/stop_detector.py`
- Test: `src/webcam_perception/test/test_stop_detector.py`

**Interfaces:**
- Consumes: 없음 (독립 모듈)
- Produces: `stop_detector.StopDetector(duration_s: float, pixel_threshold: float)` — 메서드 `update(t: float, x: float, y: float) -> bool`, `reset() -> None`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/webcam_perception/test/test_stop_detector.py`:
```python
from webcam_perception.stop_detector import StopDetector


def test_not_stopped_before_duration_elapsed():
    detector = StopDetector(duration_s=2.0, pixel_threshold=5.0)
    assert detector.update(t=0.0, x=100.0, y=100.0) is False
    assert detector.update(t=1.0, x=100.0, y=100.0) is False


def test_stopped_after_duration_within_threshold():
    detector = StopDetector(duration_s=2.0, pixel_threshold=5.0)
    detector.update(t=0.0, x=100.0, y=100.0)
    detector.update(t=1.0, x=101.0, y=100.0)
    assert detector.update(t=2.0, x=100.0, y=101.0) is True


def test_not_stopped_when_position_jumps_beyond_threshold():
    detector = StopDetector(duration_s=2.0, pixel_threshold=5.0)
    detector.update(t=0.0, x=100.0, y=100.0)
    detector.update(t=1.0, x=100.0, y=100.0)
    assert detector.update(t=2.0, x=200.0, y=100.0) is False


def test_reset_clears_history():
    detector = StopDetector(duration_s=2.0, pixel_threshold=5.0)
    detector.update(t=0.0, x=100.0, y=100.0)
    detector.update(t=2.0, x=100.0, y=100.0)
    detector.reset()
    assert detector.update(t=2.1, x=100.0, y=100.0) is False
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_stop_detector.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'webcam_perception.stop_detector'`

- [ ] **Step 3: StopDetector 구현**

`src/webcam_perception/webcam_perception/stop_detector.py`:
```python
class StopDetector:
    def __init__(self, duration_s: float, pixel_threshold: float):
        self.duration_s = duration_s
        self.pixel_threshold = pixel_threshold
        self._samples: list[tuple[float, float, float]] = []
        self._first_update_t: float | None = None

    def update(self, t: float, x: float, y: float) -> bool:
        if self._first_update_t is None:
            self._first_update_t = t
        self._samples.append((t, x, y))
        cutoff = t - self.duration_s
        self._samples = [s for s in self._samples if s[0] >= cutoff]

        if t - self._first_update_t < self.duration_s:
            return False

        xs = [s[1] for s in self._samples]
        ys = [s[2] for s in self._samples]
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
        return spread <= self.pixel_threshold

    def reset(self) -> None:
        self._samples = []
        self._first_update_t = None
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_stop_detector.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/webcam_perception/webcam_perception/stop_detector.py src/webcam_perception/test/test_stop_detector.py
git commit -m "feat(webcam_perception): add time-based stop detector"
```

---

### Task 3: 호모그래피 픽셀→map 변환

**Files:**
- Create: `src/webcam_perception/webcam_perception/homography.py`
- Test: `src/webcam_perception/test/test_homography.py`

**Interfaces:**
- Consumes: 없음 (독립 모듈, `cv2`/`numpy`만 사용)
- Produces: `homography.build_homography_matrix(pixel_points: list[tuple[float, float]], map_points: list[tuple[float, float]]) -> numpy.ndarray`
- Produces: `homography.pixel_to_map(matrix: numpy.ndarray, pixel: tuple[float, float]) -> tuple[float, float]`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/webcam_perception/test/test_homography.py`:
```python
import math

from webcam_perception.homography import build_homography_matrix, pixel_to_map


def test_pixel_to_map_scale_and_translate():
    # 픽셀 (0,0)-(100,100) 정사각형이 map (0,0)-(1,1) 정사각형에 대응하는 단순 스케일 매칭
    pixel_points = [(0.0, 0.0), (100.0, 0.0), (0.0, 100.0), (100.0, 100.0)]
    map_points = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)]
    matrix = build_homography_matrix(pixel_points, map_points)

    x, y = pixel_to_map(matrix, (50.0, 50.0))

    assert math.isclose(x, 0.5, abs_tol=1e-6)
    assert math.isclose(y, 0.5, abs_tol=1e-6)


def test_pixel_to_map_corner_point():
    pixel_points = [(0.0, 0.0), (100.0, 0.0), (0.0, 100.0), (100.0, 100.0)]
    map_points = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)]
    matrix = build_homography_matrix(pixel_points, map_points)

    x, y = pixel_to_map(matrix, (100.0, 100.0))

    assert math.isclose(x, 1.0, abs_tol=1e-6)
    assert math.isclose(y, 1.0, abs_tol=1e-6)
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_homography.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'webcam_perception.homography'`

- [ ] **Step 3: homography.py 구현**

`src/webcam_perception/webcam_perception/homography.py`:
```python
import cv2
import numpy as np


def build_homography_matrix(
    pixel_points: list[tuple[float, float]], map_points: list[tuple[float, float]]
) -> np.ndarray:
    src = np.array(pixel_points, dtype=np.float64)
    dst = np.array(map_points, dtype=np.float64)
    matrix, _ = cv2.findHomography(src, dst)
    return matrix


def pixel_to_map(matrix: np.ndarray, pixel: tuple[float, float]) -> tuple[float, float]:
    src = np.array([[pixel]], dtype=np.float64)
    dst = cv2.perspectiveTransform(src, matrix)
    x, y = dst[0][0]
    return (float(x), float(y))
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_homography.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/webcam_perception/webcam_perception/homography.py src/webcam_perception/test/test_homography.py
git commit -m "feat(webcam_perception): add pixel-to-map homography transform"
```

---

### Task 4: VehicleStopPipeline (순수 오케스트레이션)

**Files:**
- Create: `src/webcam_perception/webcam_perception/pipeline.py`
- Test: `src/webcam_perception/test/test_pipeline.py`

**Interfaces:**
- Consumes: `detection.Detection`, `detection.select_best_detection`, `geometry.bbox_center`, `geometry.bbox_bottom_center`, `geometry.point_in_roi`, `homography.pixel_to_map`, `stop_detector.StopDetector`
- Produces: `pipeline.VehicleStopPipeline(roi: tuple[float, float, float, float], confidence_threshold: float, homography_matrix: numpy.ndarray, stop_detector: StopDetector)` — 메서드 `process_detections(detections: list[Detection], now_sec: float) -> tuple[float, float] | None`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/webcam_perception/test/test_pipeline.py`:
```python
from webcam_perception.detection import Detection
from webcam_perception.homography import build_homography_matrix
from webcam_perception.pipeline import VehicleStopPipeline
from webcam_perception.stop_detector import StopDetector


def _make_pipeline(duration_s=2.0, pixel_threshold=5.0, confidence_threshold=0.5):
    pixel_points = [(0.0, 0.0), (100.0, 0.0), (0.0, 100.0), (100.0, 100.0)]
    map_points = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)]
    matrix = build_homography_matrix(pixel_points, map_points)
    return VehicleStopPipeline(
        roi=(0.0, 0.0, 100.0, 100.0),
        confidence_threshold=confidence_threshold,
        homography_matrix=matrix,
        stop_detector=StopDetector(duration_s=duration_s, pixel_threshold=pixel_threshold),
    )


def test_no_detections_returns_none():
    pipeline = _make_pipeline()
    assert pipeline.process_detections([], now_sec=0.0) is None


def test_detection_outside_roi_returns_none():
    pipeline = _make_pipeline()
    detection = Detection(x1=200.0, y1=200.0, x2=220.0, y2=220.0, confidence=0.9)
    assert pipeline.process_detections([detection], now_sec=0.0) is None


def test_detection_below_confidence_returns_none():
    pipeline = _make_pipeline(confidence_threshold=0.8)
    detection = Detection(x1=40.0, y1=40.0, x2=60.0, y2=60.0, confidence=0.5)
    assert pipeline.process_detections([detection], now_sec=0.0) is None


def test_not_stopped_yet_returns_none():
    pipeline = _make_pipeline(duration_s=2.0)
    detection = Detection(x1=40.0, y1=40.0, x2=60.0, y2=60.0, confidence=0.9)
    assert pipeline.process_detections([detection], now_sec=0.0) is None


def test_stopped_publishes_once_then_suppresses_repeat():
    pipeline = _make_pipeline(duration_s=2.0, pixel_threshold=5.0)
    detection = Detection(x1=40.0, y1=40.0, x2=60.0, y2=60.0, confidence=0.9)

    assert pipeline.process_detections([detection], now_sec=0.0) is None
    assert pipeline.process_detections([detection], now_sec=1.0) is None
    result = pipeline.process_detections([detection], now_sec=2.0)

    assert result is not None
    x, y = result
    assert 0.0 <= x <= 1.0
    assert 0.0 <= y <= 1.0

    # 계속 정지 상태 유지 중이면 재발행하지 않는다 (이벤트당 1회, 스펙 §2 step2)
    assert pipeline.process_detections([detection], now_sec=3.0) is None


def test_leaving_roi_then_stopping_again_republishes():
    pipeline = _make_pipeline(duration_s=2.0, pixel_threshold=5.0)
    detection = Detection(x1=40.0, y1=40.0, x2=60.0, y2=60.0, confidence=0.9)
    outside = Detection(x1=200.0, y1=200.0, x2=220.0, y2=220.0, confidence=0.9)

    pipeline.process_detections([detection], now_sec=0.0)
    pipeline.process_detections([detection], now_sec=1.0)
    first = pipeline.process_detections([detection], now_sec=2.0)
    assert first is not None

    pipeline.process_detections([outside], now_sec=3.0)  # 차량이 ROI를 벗어남 → 새 이벤트 준비

    pipeline.process_detections([detection], now_sec=4.0)
    pipeline.process_detections([detection], now_sec=5.0)
    second = pipeline.process_detections([detection], now_sec=6.0)
    assert second is not None
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_pipeline.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'webcam_perception.pipeline'`

- [ ] **Step 3: pipeline.py 구현**

`src/webcam_perception/webcam_perception/pipeline.py`:
```python
import numpy as np

from webcam_perception.detection import Detection, select_best_detection
from webcam_perception.geometry import bbox_bottom_center, bbox_center, point_in_roi
from webcam_perception.homography import pixel_to_map
from webcam_perception.stop_detector import StopDetector


class VehicleStopPipeline:
    def __init__(
        self,
        roi: tuple[float, float, float, float],
        confidence_threshold: float,
        homography_matrix: np.ndarray,
        stop_detector: StopDetector,
    ):
        self.roi = roi
        self.confidence_threshold = confidence_threshold
        self.homography_matrix = homography_matrix
        self.stop_detector = stop_detector
        self._published_for_current_stop = False

    def process_detections(
        self, detections: list[Detection], now_sec: float
    ) -> tuple[float, float] | None:
        in_roi = [
            d for d in detections
            if point_in_roi(bbox_center(d.x1, d.y1, d.x2, d.y2), self.roi)
        ]
        best = select_best_detection(in_roi, self.confidence_threshold)

        if best is None:
            self.stop_detector.reset()
            self._published_for_current_stop = False
            return None

        cx, cy = bbox_center(best.x1, best.y1, best.x2, best.y2)
        stopped = self.stop_detector.update(now_sec, cx, cy)

        if not stopped:
            self._published_for_current_stop = False
            return None

        if self._published_for_current_stop:
            return None

        bx, by = bbox_bottom_center(best.x1, best.y1, best.x2, best.y2)
        map_x, map_y = pixel_to_map(self.homography_matrix, (bx, by))
        self._published_for_current_stop = True
        return (map_x, map_y)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_pipeline.py -v`
Expected: PASS (6 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/webcam_perception/webcam_perception/pipeline.py src/webcam_perception/test/test_pipeline.py
git commit -m "feat(webcam_perception): add VehicleStopPipeline orchestration"
```

---

### Task 5: WebcamPerceptionNode (rclpy 래퍼) + 파라미터 파일

**Files:**
- Create: `src/webcam_perception/webcam_perception/webcam_perception_node.py`
- Create: `src/webcam_perception/config/params.yaml`
- Test: `src/webcam_perception/test/test_webcam_perception_node.py`

**Interfaces:**
- Consumes: `pipeline.VehicleStopPipeline`, `stop_detector.StopDetector`, `homography.build_homography_matrix`, `detection.Detection`
- Produces: `webcam_perception_node.WebcamPerceptionNode(capture=None, detector=None)` (rclpy.Node), 진입점 `webcam_perception_node.main()`

- [ ] **Step 1: 실패하는 노드 테스트 작성**

`src/webcam_perception/test/test_webcam_perception_node.py`:
```python
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
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_webcam_perception_node.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'webcam_perception.webcam_perception_node'`

- [ ] **Step 3: webcam_perception_node.py 구현**

`src/webcam_perception/webcam_perception/webcam_perception_node.py`:
```python
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
            detector = YOLO(self.get_parameter('yolo_weights_path').value)
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
```

- [ ] **Step 4: config/params.yaml 작성**

`src/webcam_perception/config/params.yaml`:
```yaml
webcam_perception_node:
  ros__parameters:
    camera_index: 0
    yolo_weights_path: "/absolute/path/to/webcam_yolo_weights.pt"
    vehicle_class_id: 0
    capture_period_s: 0.1

    # 아래 값들은 스펙 §7 TBD 항목 — 실측 캘리브레이션 전 임시 기본값이다.
    # 실측 후 이 파일만 갱신하면 되며 코드 변경은 필요 없다.
    roi: [0.0, 0.0, 640.0, 480.0]          # TBD: 실제 웹캠 ROI 사각형(px)
    stop_duration_s: 2.0                    # TBD: 정지 판정 지속 시간(초)
    stop_pixel_threshold: 5.0               # TBD: 정지 판정 픽셀 변화 임계치
    confidence_threshold: 0.5               # TBD: YOLO confidence threshold
    homography_pixel_points: [0.0, 0.0, 640.0, 0.0, 0.0, 480.0, 640.0, 480.0]  # TBD: 캘리브레이션 대응점(px)
    homography_map_points: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0]            # TBD: 캘리브레이션 대응점(map)
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `PYTHONPATH=src/webcam_perception python3 -m pytest src/webcam_perception/test/test_webcam_perception_node.py -v`
Expected: PASS (1 passed)

- [ ] **Step 6: colcon 빌드/테스트로 패키지 전체 검증**

Run:
```bash
cd <workspace_root>
colcon build --packages-select webcam_perception --symlink-install
colcon test --packages-select webcam_perception
colcon test-result --verbose
```
Expected: 빌드 성공, 이전 5개 태스크의 모든 테스트(geometry 5, detection 3, stop_detector 4, homography 2, pipeline 6, node 1 = 총 21개) PASS.

- [ ] **Step 7: 커밋**

```bash
git add src/webcam_perception/webcam_perception/webcam_perception_node.py \
  src/webcam_perception/config/params.yaml \
  src/webcam_perception/test/test_webcam_perception_node.py \
  src/webcam_perception/setup.py
git commit -m "feat(webcam_perception): add rclpy node wrapper and params file"
```

---

## Self-Review 메모 (플랜 작성자용, 실행 시 삭제 가능)

- **스펙 커버리지**: §3.1(webcam_perception_node 역할) → Task 5, §5.1 W1(RGB 스트림)/W2(ROI+정지판정+confidence 선택)/W3(호모그래피+발행) → Task 1,2,3,4, §4 인터페이스(토픽/QoS/발행 시점) → Task 5. 모두 커버.
- 실제 카메라/YOLO 하드웨어 검증, 실측 캘리브레이션 값 확정은 스펙 §1.2/§8에서 명시적으로 범위 밖이므로 이 플랜에도 포함하지 않았다 — Task 6 colcon 테스트는 어디까지나 코드 배선 검증이지 실물 검증이 아니다.
