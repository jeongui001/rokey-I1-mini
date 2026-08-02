# Nav2 학습용 패키지 — 설계 스펙

> 이 문서는 브레인스토밍 대화를 바탕으로 확정한 설계입니다. 실제 구현물(`nav2_study_ws`)은 이 저장소(`rokey-I1-mini`)와는 별도의 독립 워크스페이스로 만들어지며, 이 스펙만 학습 목적 기록으로 이 저장소에 보관합니다.

## 1. 개요

### 1.1 목적

`rokey-I1-mini`의 실제 차량 접근 프로젝트(`src/vehicle_approach`, `src/vehicle_mission`)는 Nav2를 이미 사용 중이지만, 작성자가 Nav2 자체를 체계적으로 배운 적은 없다. 실제 하드웨어(터틀봇4 + 오크디)에 의존하지 않고, TurtleBot3 + Gazebo 시뮬레이션으로 (1) Nav2 전반의 개념을 순서대로 익히고, (2) 실제 프로젝트에서 이미 쓰고 있는 Nav2 API 패턴(액션 클라이언트, TF 변환, goal 재전송/취소 판정)을 손으로 재현하며 이해한다.

### 1.2 범위

- 이동 대상은 TurtleBot3(Gazebo 시뮬레이션)이다. 실제 터틀봇4/오크디/웹캠 하드웨어는 다루지 않는다.
- 결과물은 완성된 코드 + 한국어 설명 문서다. 빈칸을 채우는 실습형 스텁은 만들지 않는다.
- 두 개의 독립적인 학습 트랙(패키지)으로 나눈다: Nav2 개념 전반을 다루는 `nav2_fundamentals`와, 실제 프로젝트 패턴을 재현하는 `nav2_applied_practice`.
- `nav2_applied_practice`는 실제 프로젝트의 알고리즘을 그대로 재현한다. `docs/NEXT_STEPS.md`에 기록된 보류 이슈(goal 전송 "결정" 시점과 실제 전송 성공 시점 사이의 갭)를 고치지 않고 학습 포인트로만 문서에 남긴다 — 이 스펙의 범위는 실제 버그 수정이 아니다.
- 자동화 테스트는 `nav2_applied_practice`의 순수 로직 함수(`pose_utils`, `goal_calculation`)에만 pytest로 작성한다. ROS 노드 자체와 `nav2_fundamentals`는 시뮬레이션에서 직접 실행/관찰로 검증하며 자동 테스트를 만들지 않는다.

### 1.3 환경 확인 결과 (사전 점검 완료)

- OS: Ubuntu 22.04 (Jammy), ROS2 Humble
- Gazebo Classic 11, `ros-humble-turtlebot3-*`, `ros-humble-navigation2` 모두 apt로 이미 설치되어 있음
- GPU: 내장 AMD Radeon(Renoir), Mesa direct rendering 정상 — TB3 시뮬레이션 구동에 문제없음
- 참고: `~/vslam_ws`에 이미 다른 목적으로 만들어둔 turtlebot3 소스 빌드 워크스페이스가 존재하지만, 이번 학습 워크스페이스와는 무관하게 완전히 새로 만든다.

## 2. 워크스페이스 구조

```
~/nav2_study_ws/
└── src/
    ├── nav2_fundamentals/
    └── nav2_applied_practice/
```

- 현재 `rokey-I1-mini` 저장소와는 별도의 colcon 워크스페이스다. `rokey-I1-mini`의 코드에 대한 의존성은 없다(문서에서 파일 경로를 텍스트로 참조할 뿐).
- 두 패키지는 TurtleBot3 + Nav2 + Gazebo가 이미 설치되어 있다는 전제 위에서, 그 위에 얹는 launch 파일/파라미터/노드만 담는다.

## 3. `nav2_fundamentals` — Nav2 개념 순차 커리큘럼

6개 레슨을 순서대로 진행한다. 각 레슨은 실행 가능한 launch 파일 하나 + 한국어 설명 문서 하나로 구성된다.

```
nav2_fundamentals/
├── launch/
│   ├── lesson01_bringup.launch.py
│   ├── lesson02_slam.launch.py
│   ├── lesson03_amcl.launch.py
│   ├── lesson04_costmap.launch.py
│   ├── lesson05_planner_controller.launch.py
│   └── lesson06_bt_waypoint.launch.py
├── config/            # 레슨별 nav2 파라미터 오버라이드
├── docs/
│   ├── lesson01.md ... lesson06.md
│   └── README.md      # 커리큘럼 개요, 사전 준비(TURTLEBOT3_MODEL 환경변수 등)
├── package.xml
├── setup.py
└── resource/nav2_fundamentals
```

| 레슨 | 다루는 개념 | 실행 결과로 확인하는 것 |
|---|---|---|
| 01 | Nav2 아키텍처 개요, TB3 Gazebo 실행 | RViz2에서 "2D Goal Pose"로 goal을 보내고 로봇이 이동하는 것을 확인 |
| 02 | SLAM (slam_toolbox) | 텔레옵으로 맵을 만들고 `.pgm`/`.yaml`로 저장 |
| 03 | AMCL 로컬라이제이션 | 만든 맵을 불러와 초기 pose 지정 후 파티클이 수렴하는 것을 확인 |
| 04 | 코스트맵 (global/local) | 서로 다른 인플레이션 반경 설정 두 세트를 비교해 코스트맵 시각화 차이 관찰 |
| 05 | 플래너 vs 컨트롤러 | 전역 경로(플래너)와 로컬 추종 경로(컨트롤러)의 역할 차이를 RViz에서 구분 |
| 06 | 행동 트리(BT) + Waypoint Follower | 기본 BT XML 구조 확인, 여러 waypoint를 순서대로 도는 것을 실행 |

각 `docs/lessonNN.md`는 다음 형식을 따른다: 목표 / 사전조건 / 실행 명령 / 관찰 포인트 / 이해 확인 질문(답은 각자 확인).

## 4. `nav2_applied_practice` — 실제 프로젝트 패턴 재현

```
nav2_applied_practice/
├── nav2_applied_practice/
│   ├── __init__.py
│   ├── navigate_to_pose_client_node.py   # ← vehicle_mission_node.py 패턴 재현
│   ├── pose_utils.py                     # ← vehicle_mission/pose_utils.py 재현
│   ├── tf_lookup_node.py                 # ← vehicle_approach/pipeline.py의 tf2_ros.Buffer 사용 재현
│   ├── goal_calculation.py               # ← vehicle_approach/goal_calculation.py 재현 (순수 함수)
│   └── goal_resend_demo_node.py          # ← pipeline.py의 재전송/취소 로직 재현
├── config/params.yaml
├── docs/
│   ├── 01_action_client.md
│   ├── 02_tf_transform.md
│   └── 03_goal_resend_and_cancel.md
├── test/
│   ├── test_pose_utils.py
│   └── test_goal_calculation.py
├── package.xml
├── setup.py
└── resource/nav2_applied_practice
```

각 `docs/0N_*.md`에는 대응되는 실제 프로젝트 파일(`vehicle_mission_node.py`, `pipeline.py` 등)의 위치를 명시해 학습자가 실제 코드와 나란히 비교할 수 있게 한다.

### 4.1 `navigate_to_pose_client_node`

`vehicle_mission_node.py`와 동일한 패턴: 파라미터로 목표 pose(x, y, yaw) 선언 → `ActionClient(NavigateToPose, 'navigate_to_pose')` → `wait_for_server()` → `send_goal_async()` → 거부 시 에러 로그, 수락 시 결과 콜백에서 성공/실패 로깅.

### 4.2 `tf_lookup_node`

`pipeline.py`의 `tf2_ros.Buffer` 사용 패턴 재현: 주기적으로 `map → base_link` TF를 조회하고, 하드코딩된 `PointStamped`를 `tf2_geometry_msgs.do_transform_point`로 변환해 로그로 출력한다.

### 4.3 `goal_calculation` + `goal_resend_demo_node`

`goal_calculation.py`는 `vehicle_approach/goal_calculation.py`의 `should_resend_goal`류 순수 함수를 재현한다(입력: 새 목표 좌표, 마지막 전송 좌표, 재전송 임계값 / 출력: bool).

`goal_resend_demo_node`는 이 로직을 실제 시뮬레이션에서 체험하는 노드다:

**데이터 흐름:**
1. RViz의 "Publish Point" 툴로 지도 위를 클릭 → `/clicked_point`(`geometry_msgs/PointStamped`) 발행 — 실제 프로젝트의 차량 탐지 결과 역할을 대신함
2. `goal_resend_demo_node`가 `/clicked_point`를 구독, `goal_calculation.should_resend_goal`로 이전 전송 goal과 비교해 재전송 여부 판정
3. 재전송 결정 시 `navigate_to_pose_client_node`와 같은 방식으로 `NavigateToPose` goal 전송
4. 주기적으로 `tf_lookup_node`와 같은 방식으로 `map → base_link` TF를 조회해 로봇-목표 거리 계산
5. 거리가 완료 임계값 이하로 좁혀지면 진행 중인 goal을 취소하고 "도착" 로그 출력

**알려진 갭 (의도적으로 재현, 수정하지 않음):** goal을 "보내기로 결정"한 시점에 마지막 전송 좌표를 즉시 갱신하는데, 그 직후 액션 서버가 준비되지 않아 실제 전송을 건너뛰면 판정 로직상으로는 이미 "보냈다"고 기록된다. `docs/NEXT_STEPS.md` §2에 기록된 실제 프로젝트의 보류 이슈와 동일한 패턴이다. `03_goal_resend_and_cancel.md`에 이 갭이 왜 생기는지, 어떤 조건에서 실제로 문제가 되는지 관찰해보라는 학습 포인트로 남긴다.

## 5. 에러 처리

- Nav2 액션 서버 미준비: `wait_for_server()` 호출로 대기 + 준비 안 됐을 때 경고 로그 (재시도 루프나 타임아웃 후 종료 같은 추가 로직은 넣지 않음 — 실제 프로젝트 패턴 그대로)
- TF 조회 실패(프레임 아직 브로드캐스트 전): `tf2_ros`의 `LookupException`/`ConnectivityException`/`ExtrapolationException`을 캐치해 해당 주기만 건너뛰고 로그, 크래시하지 않음
- Nav2가 goal을 거부: 에러 로그만 남기고 다음 입력(다음 클릭) 대기, 재시도 로직 없음

## 6. 테스트

- `nav2_applied_practice/test/test_pose_utils.py`: `pose_utils`의 좌표/yaw → `PoseStamped` 변환 순수 함수를 테이블 기반으로 검증 (실제 프로젝트 `test_pose_utils.py`와 동일한 스타일)
- `nav2_applied_practice/test/test_goal_calculation.py`: `should_resend_goal`류 함수를 임계값 경계값 포함해 검증
- 그 외(노드 자체, `nav2_fundamentals` 전체)는 자동 테스트 없이 TB3 시뮬레이션 실행 + 각 문서에 명시된 "이렇게 보이면 성공" 체크포인트로 수동 검증
