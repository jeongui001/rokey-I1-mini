# 실행 방식

워크스페이스는 이미 빌드되어 있음 (`install/` 존재). 코드를 수정했다면 먼저 빌드:

```bash
cd ~/b3_int1_mini
colcon build --symlink-install
source install/setup.bash
```

## 0. Nav2 실행 (사전 준비)

아래 노드들이 Nav2 액션(`/robot11/navigate_to_pose`)에 의존하므로, 먼저 터미널 3개에서 Nav2 스택을 띄워야 함.

```bash
# 1) localization (맵 기반 위치 추정)
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot11 map:=$HOME/b3_int1_mini/our_maps.yaml

# 2) rviz 시각화
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot11

# 3) nav2 (경로 계획/제어)
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot11
```

## 1. `ros2 run`으로 개별 실행 (노드별로 따로 띄우고 싶을 때)

각 노드는 `config/params.yaml` 상단에 실행 커맨드가 주석으로 남아 있음. `--params-file` 없이 실행하면 코드 기본값(TBD 값)으로 조용히 동작하니 **반드시 지정할 것**.

```bash
# 1) 웹캠 인식 노드 — 차량 초기 위치 발행
# debug_view 기본값 true라 ROI 사각형 + 탐지 박스를 보여주는 창(webcam_perception_debug)이 뜸
ros2 run webcam_perception webcam_perception_node --ros-args --params-file src/webcam_perception/config/params.yaml

# 2) 미션 상태 전이 노드 — 대기 지점 이동 후 접근 노드 활성화
ros2 run vehicle_mission vehicle_mission_node --ros-args --params-file src/vehicle_mission/config/params.yaml

# 3) 차량 접근 노드 — 오크디 탐지 기반 Nav2 goal 계산/전송
# robot11 네임스페이스 하드코딩: /tf, /tf_static은 노드 코드에서 remap이 안 되므로 실행 시 -r로 명시
ros2 run vehicle_approach vehicle_approach_node --ros-args --params-file src/vehicle_approach/config/params.yaml -r /tf:=/robot11/tf -r /tf_static:=/robot11/tf_static
```

터미널 3개에서 각각 실행.

## 2. `ros2 launch`로 한 번에 실행

`vehicle_mission` 패키지에 `bringup.launch.py`를 추가해 세 노드(`webcam_perception_node` → `vehicle_mission_node` → `vehicle_approach_node`)를 한 번에 띄움. 파라미터는 각 패키지에 설치된 `config/params.yaml`을 자동으로 찾아서 넣으므로 경로를 따로 지정할 필요 없음.

```bash
ros2 launch vehicle_mission bringup.launch.py
```

파라미터 값을 바꾸려면 `src/<패키지>/config/params.yaml`을 수정한 뒤 `colcon build --packages-select <패키지>`로 다시 빌드해야 반영됨(symlink-install이면 yaml만 수정 시 재빌드 불필요).

## 주의사항

- `vehicle_approach_node`는 오크디 카메라, Nav2, TF(`camera_frame`↔`base_link`)가 실제로 배선되어 있어야 정상 동작함. 하드웨어 연결 전이면 에러가 날 수 있음 (`docs/NEXT_STEPS.md` §4 참고).
- `homography_calibration_tool`은 캘리브레이션용 별도 도구라 위 실행 목록에서 제외함.
- 로봇 하드웨어(터틀봇) 쪽 토픽/액션이 전부 `/robot11` 네임스페이스 아래에 있어 다음 항목들을 `robot11`로 하드코딩해둠:
  - `vehicle_mission_node`, `vehicle_approach_node`의 Nav2 액션 이름 → `/robot11/navigate_to_pose`
  - `vehicle_approach/config/params.yaml`의 `rgb_topic`/`depth_topic`/`camera_info_topic` → `/robot11/oakd/...`
  - `vehicle_approach_node`의 TF 구독(`/tf`, `/tf_static`) → `/robot11/tf`, `/robot11/tf_static`으로 remap (launch에서는 `bringup.launch.py`에 이미 반영, `ros2 run` 단독 실행 시엔 위처럼 `-r` 플래그 필요)
  - 로봇 번호가 바뀌면 이 네 곳을 함께 수정해야 함.
- `webcam_perception_node`의 `debug_view` 파라미터 기본값은 `true` — ROI 사각형(청록)과 탐지 박스(최적 탐지는 초록, 나머지는 빨강 + confidence)를 `cv2.imshow` 창으로 띄워줌. 화면(디스플레이) 없는 환경에서 끄려면 `-p debug_view:=false` 추가.
