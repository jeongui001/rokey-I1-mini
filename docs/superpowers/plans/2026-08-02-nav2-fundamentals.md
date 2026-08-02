# nav2_fundamentals Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** TurtleBot3 + Gazebo 시뮬레이션 위에서 Nav2의 핵심 개념(전체 아키텍처, SLAM, AMCL, 코스트맵, 플래너/컨트롤러, 행동 트리+Waypoint Follower)을 순서대로 실습할 수 있는 ROS2 패키지 `nav2_fundamentals`를 만든다.

**Architecture:** 새 독립 워크스페이스 `~/nav2_study_ws/src/nav2_fundamentals` 아래, 이미 설치된 `turtlebot3_gazebo`/`turtlebot3_navigation2`/`slam_toolbox`/`nav2_bringup`/`nav2_simple_commander`를 그대로 활용해 각 레슨을 하나의 `ros2 launch` 명령으로 실행 가능하게 묶는다. 커스텀 코드는 최소화하고(런치 파일 + 파라미터 오버라이드 + waypoint 데모 스크립트 1개), 나머지는 한국어 설명 문서로 무엇을 관찰해야 하는지 안내한다.

**Tech Stack:** ROS2 Humble, Gazebo Classic 11, ament_python, Python 3.10, `nav2_simple_commander`

## Global Constraints

- 워크스페이스 경로: `~/nav2_study_ws` (colcon 워크스페이스, `rokey-I1-mini` 저장소와 무관)
- `TURTLEBOT3_MODEL=waffle`이 이미 `~/.bashrc`에 설정되어 있음 (재확인만 하면 됨, 새로 추가하지 않음)
- 이 패키지는 자동화 테스트를 만들지 않는다 — 각 레슨은 시뮬레이션 실행 + 문서에 명시된 관찰 포인트로 수동 검증한다 (스펙 §6)
- `nav2_applied_practice` 없이 이 패키지 단독으로 완결된 학습 결과물이어야 한다
- 패키지 스타일은 `rokey-I1-mini`의 기존 ROS2 패키지(`vehicle_mission` 등)의 `setup.py`/`package.xml` 포맷을 따른다 (참고용, 의존은 없음)

---

### Task 1: 워크스페이스 및 패키지 스캐폴딩

**Files:**
- Create: `~/nav2_study_ws/src/nav2_fundamentals/package.xml`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/setup.py`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/setup.cfg`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/resource/nav2_fundamentals`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/nav2_fundamentals/__init__.py`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/docs/README.md`

**Interfaces:**
- Produces: `colcon build --packages-select nav2_fundamentals`로 빌드되는 ament_python 패키지 `nav2_fundamentals`. 이후 태스크들은 이 패키지의 `launch/`, `config/` 디렉터리에 파일을 추가한다.

- [ ] **Step 1: 워크스페이스 디렉터리 생성**

```bash
mkdir -p ~/nav2_study_ws/src/nav2_fundamentals/nav2_fundamentals
mkdir -p ~/nav2_study_ws/src/nav2_fundamentals/launch
mkdir -p ~/nav2_study_ws/src/nav2_fundamentals/config
mkdir -p ~/nav2_study_ws/src/nav2_fundamentals/docs
mkdir -p ~/nav2_study_ws/src/nav2_fundamentals/resource
mkdir -p ~/nav2_study_ws/maps
touch ~/nav2_study_ws/src/nav2_fundamentals/resource/nav2_fundamentals
touch ~/nav2_study_ws/src/nav2_fundamentals/nav2_fundamentals/__init__.py
```

- [ ] **Step 2: `package.xml` 작성**

`~/nav2_study_ws/src/nav2_fundamentals/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>nav2_fundamentals</name>
  <version>0.0.1</version>
  <description>TurtleBot3 + Gazebo 시뮬레이션으로 Nav2 핵심 개념을 순서대로 학습하는 커리큘럼 패키지</description>
  <maintainer email="hwangjeongui01@gmail.com">hwangjeongui</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>turtlebot3_gazebo</exec_depend>
  <exec_depend>turtlebot3_navigation2</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_simple_commander</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: `setup.py` 작성**

`~/nav2_study_ws/src/nav2_fundamentals/setup.py`:

```python
from glob import glob

from setuptools import find_packages, setup

package_name = 'nav2_fundamentals'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwangjeongui',
    maintainer_email='hwangjeongui01@gmail.com',
    description='TurtleBot3 + Gazebo 시뮬레이션으로 Nav2 핵심 개념을 순서대로 학습하는 커리큘럼 패키지',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'waypoint_demo = nav2_fundamentals.waypoint_demo:main',
        ],
    },
)
```

- [ ] **Step 4: `setup.cfg` 작성**

`~/nav2_study_ws/src/nav2_fundamentals/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/nav2_fundamentals
[install]
install_scripts=$base/lib/nav2_fundamentals
```

- [ ] **Step 5: `docs/README.md`에 커리큘럼 개요 뼈대 작성**

`~/nav2_study_ws/src/nav2_fundamentals/docs/README.md`:

```markdown
# nav2_fundamentals — Nav2 학습 커리큘럼

TurtleBot3 + Gazebo 시뮬레이션으로 Nav2를 순서대로 익히는 6개 레슨.

## 사전 준비

- `echo $TURTLEBOT3_MODEL` 실행 시 `waffle`이 출력되어야 한다 (이미 `~/.bashrc`에 설정됨).
- 매 레슨 실행 전: `source /opt/ros/humble/setup.bash && source ~/nav2_study_ws/install/setup.bash`

## 레슨 목록

| 레슨 | 실행 명령 | 다루는 개념 |
|---|---|---|
| 01 | `ros2 launch nav2_fundamentals lesson01_bringup.launch.py` | Nav2 아키텍처 개요, RViz로 goal 보내기 |
| 02 | `ros2 launch nav2_fundamentals lesson02_slam.launch.py` | SLAM으로 맵 만들기 |
| 03 | `ros2 launch nav2_fundamentals lesson03_amcl.launch.py` | AMCL 로컬라이제이션 |
| 04 | `ros2 launch nav2_fundamentals lesson04_costmap.launch.py costmap_profile:=small` | 코스트맵(인플레이션) 비교 |
| 05 | `ros2 launch nav2_fundamentals lesson05_planner_controller.launch.py` | 플래너 vs 컨트롤러 |
| 06 | `ros2 launch nav2_fundamentals lesson06_bt_waypoint.launch.py` | 행동 트리 + Waypoint Follower |

각 레슨의 상세 설명은 `docs/lessonNN.md`를 참고. (아래 태스크에서 채워짐)
```

- [ ] **Step 6: 워크스페이스 빌드로 스캐폴딩 검증**

```bash
cd ~/nav2_study_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select nav2_fundamentals
```

Expected: `Summary: 1 package finished` 출력, 에러 없음.

- [ ] **Step 7: git 초기화 및 커밋**

```bash
cd ~/nav2_study_ws
git init
cat > .gitignore <<'EOF'
build/
install/
log/
maps/
EOF
git add src/nav2_fundamentals .gitignore
git commit -m "chore: scaffold nav2_fundamentals ament_python package"
```

---

### Task 2: Lesson 01 — Nav2 아키텍처 개요 + 첫 goal 전송

**Files:**
- Create: `~/nav2_study_ws/src/nav2_fundamentals/launch/lesson01_bringup.launch.py`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/docs/lesson01.md`

**Interfaces:**
- Consumes: Task 1에서 만든 패키지 스캐폴딩
- Produces: `launch/lesson01_bringup.launch.py` — 이후 Task 6(lesson05)에서 그대로 재사용(include)됨

- [ ] **Step 1: `lesson01_bringup.launch.py` 작성**

TurtleBot3 Gazebo World + `turtlebot3_navigation2`의 표준 nav2 bringup(내부적으로 번들 맵 `turtlebot3_navigation2/map/map.yaml`과 RViz를 포함)을 하나로 묶는다.

> **주의(실행 검증으로 확인된 사항):** `navigation2.launch.py`는 `map`/`params_file`을 `LaunchConfiguration('map', default=...)` 자기참조 패턴으로 선언한다. 이 패턴은 최상위(top-level)로 직접 실행할 때는 문제없지만, 지금처럼 다른 launch 파일에서 `IncludeLaunchDescription`으로 포함하면서 `map`/`params_file`을 명시적으로 넘기지 않으면 `[Errno 2] No such file or directory: ''`로 실패한다(직접 재현 확인됨). 따라서 아래 코드는 `map`/`params_file` 경로를 직접 계산해 명시적으로 넘긴다 — `use_sim_time`만 넘기는 버전은 쓰지 않는다.

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    tb3_gazebo_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
    tb3_nav2_launch = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')

    map_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')
    turtlebot3_model = os.environ['TURTLEBOT3_MODEL']
    params_file = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'param',
        os.environ.get('ROS_DISTRO', 'humble'), f'{turtlebot3_model}.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_gazebo_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_nav2_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_dir,
                'params_file': params_file,
            }.items()),
    ])
```

- [ ] **Step 2: `docs/lesson01.md` 작성**

```markdown
# Lesson 01 — Nav2 아키텍처 개요 + 첫 goal 전송

## 목표

Nav2 스택 전체(로컬라이제이션 + 코스트맵 + 플래너 + 컨트롤러 + 행동 트리)를 블랙박스로 한 번 통째로 실행해보고, RViz에서 goal을 클릭해 로봇이 실제로 움직이는 것을 확인한다. 내부 구성 요소는 이후 레슨(02~06)에서 하나씩 뜯어본다.

## 사전조건

- `echo $TURTLEBOT3_MODEL` → `waffle` 확인
- Task 1의 `colcon build`가 성공한 상태

## 실행 명령

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 launch nav2_fundamentals lesson01_bringup.launch.py
```

Gazebo와 RViz2 창이 함께 뜬다. RViz에는 이미 번들된 맵(`turtlebot3_navigation2/map/map.yaml`)이 로드되어 있다.

## 관찰 포인트

1. Gazebo에서 로봇의 스폰 위치는 `(x=-2.0, y=-0.5)`다. RViz 상단 툴바의 **2D Pose Estimate**로 지도 위 같은 위치를 클릭하고 로봇이 바라보는 방향으로 드래그해 초기 위치를 맞춰준다.
2. 파티클(작은 화살표 무리)이 로봇 주변에 모여드는 것을 확인한다 — AMCL이 초기 위치를 좁혀가는 과정(Lesson 03에서 자세히 다룸).
3. RViz 상단 툴바의 **2D Goal Pose**로 지도 위 다른 지점을 클릭한다 — 초록색 경로(전역 경로, Lesson 05)가 그려지고 로봇이 그 경로를 따라 이동하는 것(로컬 추종, Lesson 05)을 확인한다.
4. 로봇이 장애물 근처를 지날 때 완전히 붙어가지 않고 일정 거리를 두는 것을 확인한다 — 코스트맵의 인플레이션(Lesson 04에서 자세히 다룸).

## 이해 확인 질문

- 지금 본 동작 중, "어디로 갈지 경로를 계산하는 부분"과 "그 경로를 실제로 따라가며 장애물을 피하는 부분"은 같은 구성 요소일까, 다른 구성 요소일까?
- 로봇이 자기 위치를 아는 것(로컬라이제이션)과 지도를 아는 것(맵)은 왜 서로 다른 문제일까?
```

- [ ] **Step 3: 실행 검증**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_fundamentals
source install/setup.bash
ros2 launch nav2_fundamentals lesson01_bringup.launch.py
```

Expected: Gazebo에 TurtleBot3 world와 로봇이 스폰되고, RViz2에 맵이 로드된 채로 열림. `docs/lesson01.md`에 적힌 대로 2D Pose Estimate → 2D Goal Pose를 시도해 로봇이 실제로 이동하는지 확인한 뒤 `Ctrl+C`로 종료.

- [ ] **Step 4: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_fundamentals/launch/lesson01_bringup.launch.py src/nav2_fundamentals/docs/lesson01.md
git commit -m "feat(nav2_fundamentals): add lesson01 bringup launch and doc"
```

---

### Task 3: Lesson 02 — SLAM으로 맵 만들기

**Files:**
- Create: `~/nav2_study_ws/src/nav2_fundamentals/launch/lesson02_slam.launch.py`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/docs/lesson02.md`

**Interfaces:**
- Consumes: Task 1 스캐폴딩
- Produces: 사용자가 직접 만든 맵 파일(`~/nav2_study_ws/maps/my_map.yaml`) — Task 4(lesson03)에서 선택적으로 사용

- [ ] **Step 1: `lesson02_slam.launch.py` 작성**

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    tb3_gazebo_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    rviz_config = os.path.join(
        get_package_share_directory('slam_toolbox'), 'config', 'slam_toolbox_default.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_gazebo_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
```

- [ ] **Step 2: `docs/lesson02.md` 작성**

```markdown
# Lesson 02 — SLAM으로 맵 만들기

## 목표

맵이 아직 없는 상태에서, 로봇을 직접 움직이며 `slam_toolbox`가 라이다 스캔으로 맵을 실시간으로 채워나가는 것을 관찰하고, 완성된 맵을 파일로 저장한다.

## 사전조건

- Task 1의 `colcon build` 완료

## 실행 명령

터미널 1:

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 launch nav2_fundamentals lesson02_slam.launch.py
```

터미널 2 (로봇 조종):

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## 관찰 포인트

1. RViz에서 처음에는 로봇 주변 아주 좁은 영역만 맵으로 채워져 있다(회색=미탐색, 흰색=빈 공간, 검은색=장애물).
2. `w`/`a`/`s`/`d`(teleop 안내에 따름)로 로봇을 이리저리 움직이며 맵 전체(사각형 아레나와 내부 장애물들)가 흰색/검은색으로 채워지는 것을 확인한다. 벽 근처를 한 바�퀴 돌면 충분하다.
3. 맵이 웬만큼 채워지면 teleop을 멈추고, 터미널 3에서 맵을 저장한다:

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/nav2_study_ws/maps/my_map --ros-args -p save_map_timeout:=10000.0
```

4. `~/nav2_study_ws/maps/my_map.yaml`과 `my_map.pgm`이 생성되었는지 확인한다.

## 이해 확인 질문

- SLAM은 "지도를 만드는 것"과 "그 지도 위에서 내 위치를 아는 것"을 동시에 한다고 알려져 있다(Simultaneous Localization And Mapping). 지금 이 과정에서 로봇이 "자기 위치"를 알아야 새 스캔을 기존 맵에 정확히 이어붙일 수 있는데, 왜 그런지 생각해보자.
- Lesson 01에서는 이미 완성된 맵을 불러와 AMCL로 위치추정만 했다. SLAM(이번 레슨)과 AMCL(다음 레슨)의 차이는 정확히 무엇인가?
```

- [ ] **Step 3: 실행 검증**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_fundamentals
source install/setup.bash
ros2 launch nav2_fundamentals lesson02_slam.launch.py
```

Expected: Gazebo + RViz(slam_toolbox 기본 설정) 실행. 별도 터미널에서 `teleop_keyboard`로 조종하며 RViz의 맵이 실시간으로 채워지는 것을 확인. `map_saver_cli` 실행 후 `ls ~/nav2_study_ws/maps/`에 `my_map.yaml`, `my_map.pgm` 존재 확인.

- [ ] **Step 4: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_fundamentals/launch/lesson02_slam.launch.py src/nav2_fundamentals/docs/lesson02.md
git commit -m "feat(nav2_fundamentals): add lesson02 SLAM launch and doc"
```

---

### Task 4: Lesson 03 — AMCL 로컬라이제이션

**Files:**
- Create: `~/nav2_study_ws/src/nav2_fundamentals/launch/lesson03_amcl.launch.py`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/docs/lesson03.md`

**Interfaces:**
- Consumes: Task 1 스캐폴딩. 선택적으로 Task 3에서 만든 `~/nav2_study_ws/maps/my_map.yaml`
- Produces: 없음 (마지막 학습 레슨)

- [ ] **Step 1: `lesson03_amcl.launch.py` 작성**

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map')

    default_map = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')

    tb3_gazebo_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
    localization_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='AMCL이 사용할 맵 yaml 경로 (lesson02에서 만든 맵으로도 시도해볼 수 있음)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_gazebo_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
            }.items()),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
```

- [ ] **Step 2: `docs/lesson03.md` 작성**

```markdown
# Lesson 03 — AMCL 로컬라이제이션

## 목표

이미 만들어진 맵 위에서, 라이다 스캔만으로 로봇이 "지금 지도 상 어디에 있는지"를 추정하는 AMCL(Adaptive Monte Carlo Localization)의 동작(파티클 필터 수렴 과정)을 관찰한다.

## 사전조건

- Task 1의 `colcon build` 완료
- (선택) Lesson 02에서 만든 `~/nav2_study_ws/maps/my_map.yaml`

## 실행 명령

번들된 기본 맵으로:

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 launch nav2_fundamentals lesson03_amcl.launch.py
```

Lesson 02에서 만든 자기 맵으로:

```bash
ros2 launch nav2_fundamentals lesson03_amcl.launch.py map:=$HOME/nav2_study_ws/maps/my_map.yaml
```

## 관찰 포인트

1. RViz에서 **2D Pose Estimate**로 로봇의 대략적인 실제 위치(`x=-2.0, y=-0.5` 부근)를 클릭해 초기 추정을 준다.
2. 로봇 주변에 흩뿌려진 화살표(파티클)들이 처음엔 넓게 퍼져 있다가, 로봇을 teleop으로 조금씩 움직이면(`ros2 run turtlebot3_teleop teleop_keyboard`) 점점 한 곳으로 모여드는 것을 확인한다 — 이것이 "파티클 필터 수렴"이다.
3. 파티클이 좁게 수렴한 뒤에는, 로봇을 더 움직여도 로봇 모델(빨간 화살표)이 지도 상 정확한 위치를 계속 따라가는지 확인한다.

## 이해 확인 질문

- 왜 처음에 2D Pose Estimate로 "대략적인" 위치만 줘도 정확한 위치로 수렴할 수 있을까? (힌트: 파티클이 여러 후보 위치에 대해 "이 스캔이 이 위치에서 나왔을 가능성"을 계속 평가한다)
- Lesson 02(SLAM)에서는 맵과 위치를 동시에 추정했다. 이번 레슨(AMCL)은 맵이 이미 고정되어 있다는 점에서 무엇이 더 간단해졌는가?
```

- [ ] **Step 3: 실행 검증**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_fundamentals
source install/setup.bash
ros2 launch nav2_fundamentals lesson03_amcl.launch.py
```

Expected: Gazebo + RViz(nav2_default_view) 실행, 맵 로드됨. 2D Pose Estimate 후 파티클이 수렴하는 것을 확인.

- [ ] **Step 4: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_fundamentals/launch/lesson03_amcl.launch.py src/nav2_fundamentals/docs/lesson03.md
git commit -m "feat(nav2_fundamentals): add lesson03 AMCL launch and doc"
```

---

### Task 5: Lesson 04 — 코스트맵(인플레이션) 비교

**Files:**
- Create: `~/nav2_study_ws/src/nav2_fundamentals/config/costmap_small_inflation.yaml`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/config/costmap_large_inflation.yaml`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/launch/lesson04_costmap.launch.py`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/docs/lesson04.md`

**Interfaces:**
- Consumes: Task 1 스캐폴딩
- Produces: 없음

이 태스크는 `ros-humble-turtlebot3-navigation2` 2.3.6 패키지에 번들된 `param/humble/waffle.yaml`(350줄)을 베이스로, 코스트맵 인플레이션 관련 값 4곳만 정확히 바꾼 두 변형을 만든다. 아래 줄 번호는 이 설치본 기준으로 미리 확인된 값이다.

- 176번째 줄 근처(local costmap inflation): `inflation_radius: 1.0` (186번째 줄), `cost_scaling_factor: 3.0` (187번째 줄)
- 277번째 줄 근처(global costmap inflation): `cost_scaling_factor: 3.0` (279번째 줄), `inflation_radius: 0.55` (280번째 줄)

- [ ] **Step 1: 베이스 파일을 두 변형으로 복사**

```bash
mkdir -p ~/nav2_study_ws/src/nav2_fundamentals/config
BASE=/opt/ros/humble/share/turtlebot3_navigation2/param/humble/waffle.yaml
cp "$BASE" ~/nav2_study_ws/src/nav2_fundamentals/config/costmap_small_inflation.yaml
cp "$BASE" ~/nav2_study_ws/src/nav2_fundamentals/config/costmap_large_inflation.yaml
```

- [ ] **Step 2: 변경 전 원본 값 확인 (안전장치)**

```bash
sed -n '186,187p;279,280p' ~/nav2_study_ws/src/nav2_fundamentals/config/costmap_small_inflation.yaml
```

Expected 출력:
```
        inflation_radius: 1.0
        cost_scaling_factor: 3.0
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

이 출력이 다르면(패키지 버전 차이로 줄 번호가 어긋난 것이므로) 아래 `sed` 대신 `grep -n "inflation_radius\|cost_scaling_factor" <파일>`로 실제 줄 번호를 다시 확인하고 그 번호로 진행한다.

- [ ] **Step 3: `costmap_small_inflation.yaml` — 인플레이션을 좁게(로봇이 장애물에 가깝게 붙어 이동)**

```bash
cd ~/nav2_study_ws/src/nav2_fundamentals/config
sed -i '186s/inflation_radius: 1.0/inflation_radius: 0.15/' costmap_small_inflation.yaml
sed -i '187s/cost_scaling_factor: 3.0/cost_scaling_factor: 5.0/' costmap_small_inflation.yaml
sed -i '279s/cost_scaling_factor: 3.0/cost_scaling_factor: 5.0/' costmap_small_inflation.yaml
sed -i '280s/inflation_radius: 0.55/inflation_radius: 0.15/' costmap_small_inflation.yaml
```

- [ ] **Step 4: `costmap_large_inflation.yaml` — 인플레이션을 넓게(로봇이 장애물에서 멀찍이 거리를 둠)**

```bash
sed -i '186s/inflation_radius: 1.0/inflation_radius: 1.5/' costmap_large_inflation.yaml
sed -i '280s/inflation_radius: 0.55/inflation_radius: 1.2/' costmap_large_inflation.yaml
```

- [ ] **Step 5: 두 파일의 변경 결과 확인**

```bash
diff /opt/ros/humble/share/turtlebot3_navigation2/param/humble/waffle.yaml costmap_small_inflation.yaml
diff /opt/ros/humble/share/turtlebot3_navigation2/param/humble/waffle.yaml costmap_large_inflation.yaml
```

Expected: `costmap_small_inflation.yaml`은 4줄(186,187,279,280) 차이, `costmap_large_inflation.yaml`은 2줄(186,280) 차이만 나야 한다. 그 외 줄이 하나라도 다르면 실수로 다른 부분을 건드린 것이므로 원본에서 다시 복사해 처음부터 진행한다.

- [ ] **Step 6: `lesson04_costmap.launch.py` 작성**

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    costmap_profile = LaunchConfiguration('costmap_profile', default='small')

    default_map = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')
    config_dir = os.path.join(get_package_share_directory('nav2_fundamentals'), 'config')

    tb3_gazebo_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
    bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    params_file = PythonExpression([
        "'", config_dir, "/costmap_' + '", costmap_profile, "' + '_inflation.yaml'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'costmap_profile', default_value='small',
            description="'small' 또는 'large' — config/costmap_<profile>_inflation.yaml 사용"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_gazebo_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'map': default_map,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
            }.items()),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
```

- [ ] **Step 7: `setup.py`의 `data_files`가 `config/*.yaml`을 이미 포함하는지 확인**

Task 1의 `setup.py`에 이미 `('share/' + package_name + '/config', glob('config/*.yaml'))` 항목이 있으므로 추가 수정 불필요. `colcon build` 후 아래로 확인:

```bash
ls ~/nav2_study_ws/install/nav2_fundamentals/share/nav2_fundamentals/config/
```

Expected: `costmap_small_inflation.yaml`, `costmap_large_inflation.yaml` 둘 다 보여야 함.

- [ ] **Step 8: `docs/lesson04.md` 작성**

```markdown
# Lesson 04 — 코스트맵(인플레이션) 비교

## 목표

코스트맵의 "인플레이션(inflation)" — 장애물 주변에 가상의 비용(cost)을 부풀려서 로봇이 일정 거리를 두고 지나가게 만드는 개념 — 을 두 극단적인 설정으로 비교해서 눈으로 확인한다.

## 사전조건

- Task 1의 `colcon build` 완료, Lesson 03까지의 개념(로컬라이제이션) 이해

## 실행 명령

좁은 인플레이션(로봇이 장애물에 바짝 붙어 이동):

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 launch nav2_fundamentals lesson04_costmap.launch.py costmap_profile:=small
```

넓은 인플레이션(로봇이 장애물에서 멀찍이 거리를 둠):

```bash
ros2 launch nav2_fundamentals lesson04_costmap.launch.py costmap_profile:=large
```

## 관찰 포인트

1. RViz의 **Global Costmap**/**Local Costmap** 디스플레이(파란~보라색 그라데이션)를 켠 상태에서, 장애물 주변에 색이 번져 있는 폭을 두 설정에서 비교한다. `small`은 장애물 벽에 거의 붙어서만 색이 있고, `large`는 넓은 영역까지 색이 번져 있다.
2. 2D Pose Estimate로 초기 위치를 잡고, 장애물 근처를 지나가는 2D Goal Pose를 보내본다. `large` 설정에서는 로봇이 장애물에서 훨씬 멀리 돌아가거나, 통로가 좁은 곳에서는 아예 경로를 못 찾을 수도 있다.
3. `small` 설정에서는 로봇이 장애물에 상대적으로 가깝게 붙어 지나가는 것을 확인한다.

## 이해 확인 질문

- 인플레이션 반경을 너무 좁게 잡으면 실제로 어떤 위험이 생길 수 있을까? 너무 넓게 잡으면 어떤 불편이 생길까?
- `cost_scaling_factor`는 값이 클수록 장애물에서 멀어질수록 비용이 더 빠르게 낮아진다는 뜻이다(지수적 감쇠). 이번에 바꾼 두 파라미터(`inflation_radius`, `cost_scaling_factor`)가 서로 어떻게 다른 역할을 하는지 설명해보자.
```

- [ ] **Step 9: 실행 검증**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_fundamentals
source install/setup.bash
ros2 launch nav2_fundamentals lesson04_costmap.launch.py costmap_profile:=small
```

Expected: Gazebo + RViz 실행, 코스트맵 인플레이션이 좁게 보임. `Ctrl+C` 후 `costmap_profile:=large`로 재실행해 인플레이션이 눈에 띄게 넓어진 것을 확인.

- [ ] **Step 10: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_fundamentals/config/costmap_small_inflation.yaml \
        src/nav2_fundamentals/config/costmap_large_inflation.yaml \
        src/nav2_fundamentals/launch/lesson04_costmap.launch.py \
        src/nav2_fundamentals/docs/lesson04.md
git commit -m "feat(nav2_fundamentals): add lesson04 costmap comparison launch, configs, and doc"
```

---

### Task 6: Lesson 05 — 플래너 vs 컨트롤러

**Files:**
- Create: `~/nav2_study_ws/src/nav2_fundamentals/launch/lesson05_planner_controller.launch.py`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/docs/lesson05.md`

**Interfaces:**
- Consumes: Task 2의 `launch/lesson01_bringup.launch.py` (그대로 include해서 재사용)
- Produces: 없음

플래너(전역 경로 계획)와 컨트롤러(로컬 추종)의 차이는 새로운 노드 구성이 필요한 게 아니라, Lesson 01과 동일한 nav2 스택에서 RViz의 서로 다른 디스플레이(`Path`=전역 경로, `Local Plan`/`Trajectories`=로컬 추종 후보)를 비교 관찰하는 것으로 확인한다. 이 두 디스플레이는 `nav2_default_view.rviz`(Lesson 01이 쓰는 설정)에 이미 포함되어 있다.

- [ ] **Step 1: `lesson05_planner_controller.launch.py` 작성 (lesson01 재사용)**

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 플래너(전역 경로)와 컨트롤러(로컬 추종)는 별도 노드 구성이 필요 없다 —
    # lesson01과 동일한 nav2 스택에서 RViz의 Path/Local Plan/Trajectories
    # 디스플레이를 비교 관찰하는 것으로 확인하므로 lesson01을 그대로 재사용한다.
    lesson01_launch = os.path.join(
        get_package_share_directory('nav2_fundamentals'), 'launch', 'lesson01_bringup.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lesson01_launch)),
    ])
```

- [ ] **Step 2: `docs/lesson05.md` 작성**

```markdown
# Lesson 05 — 플래너 vs 컨트롤러

## 목표

Nav2에서 "어디로 갈지 전체 경로를 한 번에 계산하는 플래너(Planner)"와 "그 경로를 따라가며 매 순간 실제 속도 명령을 만들어내는 컨트롤러(Controller)"가 서로 다른 역할을 한다는 것을 RViz 디스플레이 비교로 확인한다.

## 사전조건

- Lesson 01 완료 (동일한 launch 파일을 재사용함)

## 실행 명령

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 launch nav2_fundamentals lesson05_planner_controller.launch.py
```

## 관찰 포인트

1. 2D Pose Estimate로 초기 위치를 잡은 뒤, 멀리 떨어진 2D Goal Pose를 보낸다.
2. RViz의 **Path** 디스플레이(초록색 선) — 이것이 플래너가 한 번에 계산한 전역 경로다. 목표를 보내는 순간 한 번에 쫙 그려지는 것을 확인한다.
3. RViz의 **Local Plan** 디스플레이(다른 색 짧은 선)와 **Trajectories** 디스플레이 — 이것이 컨트롤러가 매 제어 주기(수십 ms)마다 다시 계산하는 짧은 로컬 추종 경로/후보 궤적들이다. 로봇이 이동하는 동안 이 선이 계속 갱신되는 것을 확인한다.
4. 로봇 이동 경로에 (Gazebo에서) 예상치 못한 장애물이 끼어들었다고 가정하면, 전역 경로(Path)는 크게 안 바뀌어도 로컬 추종(Local Plan)은 매 순간 그 장애물을 피하려고 계속 재계산된다 — 텔레옵으로 다른 터틀봇을 근처에 스폰하거나, 단순히 로봇 진행 경로 앞쪽 코스트맵의 Local Plan이 전역 Path에서 살짝 벗어나 장애물을 피해가는 구간이 있는지 관찰하는 것으로도 충분하다.

## 이해 확인 질문

- 플래너는 왜 "전역"이어야 하고, 컨트롤러는 왜 "로컬(짧은 구간)"이어야 할까? 둘 다 매번 지도 전체를 다시 계산한다면 어떤 문제가 생길까?
- Lesson 04에서 본 코스트맵은 플래너와 컨트롤러 중 어느 쪽(또는 둘 다)이 사용할까?
```

- [ ] **Step 3: 실행 검증**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_fundamentals
source install/setup.bash
ros2 launch nav2_fundamentals lesson05_planner_controller.launch.py
```

Expected: lesson01과 동일하게 Gazebo+RViz 실행됨. goal 전송 후 Path(전역)와 Local Plan(로컬)이 각각 관찰됨.

- [ ] **Step 4: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_fundamentals/launch/lesson05_planner_controller.launch.py src/nav2_fundamentals/docs/lesson05.md
git commit -m "feat(nav2_fundamentals): add lesson05 planner-vs-controller launch and doc"
```

---

### Task 7: Lesson 06 — 행동 트리(BT) + Waypoint Follower

**Files:**
- Create: `~/nav2_study_ws/src/nav2_fundamentals/nav2_fundamentals/waypoint_demo.py`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/launch/lesson06_bt_waypoint.launch.py`
- Create: `~/nav2_study_ws/src/nav2_fundamentals/docs/lesson06.md`

**Interfaces:**
- Consumes: Task 2의 `launch/lesson01_bringup.launch.py`, `nav2_simple_commander.robot_navigator.BasicNavigator`
- Produces: 콘솔 스크립트 `waypoint_demo` (Task 1의 `setup.py` entry_points에 이미 등록됨)

번들 맵(`turtlebot3_navigation2/map/map.yaml`, 해상도 0.05m/px, origin `(-10,-10)`)의 실제 빈 공간(장애물에서 0.2m 이상 여유가 있는 픽셀)을 미리 계산해 확인한 4개 waypoint를 사용한다: `(-1.5, 1.15)`, `(1.05, 1.45)`, `(0.95, -1.45)`, `(-1.5, -1.15)`.

- [ ] **Step 1: `waypoint_demo.py` 작성**

```python
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

WAYPOINTS = [
    (-1.5, 1.15),
    (1.05, 1.45),
    (0.95, -1.45),
    (-1.5, -1.15),
]


def _pose(navigator: BasicNavigator, x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()

    initial_pose = _pose(navigator, -2.0, -0.5)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    waypoints = [_pose(navigator, x, y) for x, y in WAYPOINTS]
    navigator.followWaypoints(waypoints)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('모든 waypoint 도달 완료')
    elif result == TaskResult.CANCELED:
        print('취소됨')
    elif result == TaskResult.FAILED:
        print('실패')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: `lesson06_bt_waypoint.launch.py` 작성 (lesson01 재사용)**

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # BT와 Waypoint Follower도 lesson01과 동일한 nav2 스택 위에서 동작한다.
    # waypoint_demo 노드는 nav2가 완전히 활성화된 뒤 별도 터미널에서 수동 실행한다
    # (docs/lesson06.md 참고) — 자동 재시도가 필요 없는 1회성 데모이기 때문.
    lesson01_launch = os.path.join(
        get_package_share_directory('nav2_fundamentals'), 'launch', 'lesson01_bringup.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lesson01_launch)),
    ])
```

- [ ] **Step 3: `docs/lesson06.md` 작성**

```markdown
# Lesson 06 — 행동 트리(BT) + Waypoint Follower

## 목표

Nav2가 "goal 하나로 이동"이라는 동작을 내부적으로 행동 트리(Behavior Tree)로 조립해서 실행한다는 것을 실제 BT XML로 확인하고, 여러 waypoint를 순서대로 도는 Waypoint Follower를 실행해본다.

## 사전조건

- Task 1의 `colcon build` 완료

## 1. 기본 행동 트리 들여다보기

Nav2가 기본으로 쓰는 BT 파일을 열어본다:

```bash
cat /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_distance.xml
```

내용은 대략 이렇게 생겼다:

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <DistanceController distance="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </DistanceController>
      <FollowPath path="{path}"  controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

읽는 법: `PipelineSequence`는 자식들을 항상 순서대로, 매 tick마다 다시 확인하며 실행하는 노드다. `DistanceController`는 로봇이 1m 이동할 때마다 안의 `ComputePathToPose`(플래너 호출, Lesson 05)를 다시 실행해 전역 경로를 갱신한다. `FollowPath`는 그 경로를 컨트롤러(Lesson 05)에게 넘겨 실제로 따라가게 한다. 즉, "이동하면서 주기적으로 경로를 재계산한다"는 정책 자체가 BT로 표현되어 있다.

## 2. Waypoint Follower 실행

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 launch nav2_fundamentals lesson06_bt_waypoint.launch.py
```

Gazebo/RViz가 완전히 뜨고 코스트맵이 채워질 때까지(수 초) 기다린 뒤, 별도 터미널에서:

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 run nav2_fundamentals waypoint_demo
```

## 관찰 포인트

1. `waypoint_demo`가 초기 pose를 설정하고 Nav2가 활성화되길 기다린 뒤, 4개 waypoint를 순서대로 도는 것을 RViz에서 확인한다.
2. 각 waypoint에 도착할 때마다 goal이 이어서 다음 waypoint로 바뀌는 것을 확인한다 — 사람이 매번 2D Goal Pose를 클릭하지 않아도 자동으로 이어진다.
3. 터미널에 `모든 waypoint 도달 완료`가 출력되면 성공.

## 이해 확인 질문

- 이번 레슨에서 실행한 `navigate_w_replanning_distance.xml`은 "1m마다 경로 재계산"이라는 정책을 담고 있었다. 만약 이 정책을 "장애물이 새로 나타났을 때만 재계산"으로 바꾸고 싶다면, 코드를 고치는 것과 이 BT XML을 고치는 것 중 어느 쪽이 더 간단할까? 왜 Nav2가 이런 방식(BT)을 택했을지 생각해보자.
- `followWaypoints`는 내부적으로 각 waypoint마다 Lesson 01~05에서 본 전체 스택(로컬라이제이션→코스트맵→플래너→컨트롤러)을 반복 호출한다. 이 6개 레슨이 어떻게 하나로 이어지는지 스스로 설명해보자.
```

- [ ] **Step 4: 실행 검증**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install --packages-select nav2_fundamentals
source install/setup.bash
ros2 launch nav2_fundamentals lesson06_bt_waypoint.launch.py
```

다른 터미널에서 코스트맵이 채워진 뒤:

```bash
source ~/nav2_study_ws/install/setup.bash
ros2 run nav2_fundamentals waypoint_demo
```

Expected: 로봇이 4개 waypoint를 순서대로 이동, 터미널에 `모든 waypoint 도달 완료` 출력.

- [ ] **Step 5: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_fundamentals/nav2_fundamentals/waypoint_demo.py \
        src/nav2_fundamentals/launch/lesson06_bt_waypoint.launch.py \
        src/nav2_fundamentals/docs/lesson06.md
git commit -m "feat(nav2_fundamentals): add lesson06 BT/waypoint follower demo, launch, and doc"
```

---

### Task 8: README 커리큘럼 개요 마무리 + 전체 빌드 확인

**Files:**
- Modify: `~/nav2_study_ws/src/nav2_fundamentals/docs/README.md`

**Interfaces:**
- Consumes: Task 2~7에서 만든 모든 레슨 문서/launch 파일
- Produces: 없음 (패키지 완결)

- [ ] **Step 1: README에 사전 준비 사항과 레슨 간 연결 설명 보강**

`~/nav2_study_ws/src/nav2_fundamentals/docs/README.md`의 "레슨 목록" 표 아래에 다음 절을 추가한다:

```markdown
## 진행 순서

레슨은 01→06 순서대로 진행하는 것을 전제로 한다. 특히 02(SLAM)에서 만든 맵은 03(AMCL)에서 선택적으로 재사용할 수 있고, 05·06은 01의 launch 파일을 그대로 재사용한다 — 별도 노드 구성이 아니라 같은 스택을 다른 각도(코스트맵 파라미터, BT/waypoint)로 관찰하는 것이 목적이기 때문이다.

## 문제가 생기면

- Gazebo 창이 회색으로 멈춰 있으면 GPU 렌더링 문제일 수 있다 — `glxinfo -B | grep "direct rendering"`으로 `Yes`인지 확인.
- `ros2 launch`가 패키지를 못 찾으면 `source ~/nav2_study_ws/install/setup.bash`를 안 했을 가능성이 크다.
```

- [ ] **Step 2: 전체 워크스페이스 빌드로 최종 확인**

```bash
cd ~/nav2_study_ws
colcon build --symlink-install
```

Expected: `Summary: 1 package finished [...]`, 에러 없음.

- [ ] **Step 3: 커밋**

```bash
cd ~/nav2_study_ws
git add src/nav2_fundamentals/docs/README.md
git commit -m "docs(nav2_fundamentals): finalize curriculum README"
```
