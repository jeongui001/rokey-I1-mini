# 다음 작업 (2026-08-02 세션 종료 시점)

> 브랜치 `feature/vehicle-approach-nodes`, main 미병합. 이 문서는 다음 세션 시작할 때 참고용.

## 1. 현재 상태

| 패키지 | 태스크 | 테스트 | 최종 리뷰 |
|---|---|---|---|
| `src/webcam_perception` | 5/5 완료 | 23개 통과 | 클린 |
| `src/vehicle_mission` | 3/3 완료 | 10개 통과 | 클린 |
| `src/vehicle_approach` | 6/6 완료 | 28개 통과 | 이슈 1건 보류(parked) |

3개 ROS2(Humble) 패키지 모두 코드 의존성 없이 독립 빌드·테스트 가능. 각 패키지는 `docs/superpowers/plans/2026-08-01-*.md` 플랜 문서 기준으로 TDD + 서브에이전트 리뷰 루프를 거쳐 구현됨. 작업 디렉터리는 깨끗하고 main에는 병합하지 않았다.

## 2. 검토가 필요한 보류(parked) 이슈 1건

**위치:** `src/vehicle_approach/vehicle_approach/vehicle_approach_node.py`의 `_send_goal`(Nav2 `server_is_ready()` 체크)과 `pipeline.py`의 `_last_sent_goal` 기록 시점의 상호작용.

**증상:** goal을 보내기로 "결정"한 시점에 `pipeline.py`가 곧바로 `_last_sent_goal`을 갱신하는데, 이후 노드의 `server_is_ready()` 체크가 실패하면 실제 전송(`send_goal_async`)은 건너뛴다. 그런데 파이프라인은 이미 "보냈다"고 기록했으므로, 차량 위치가 `goal_resend_threshold_m`만큼 움직이기 전까지는 다시 시도하지 않는다 → Nav2 서버가 그 순간 아직 안 떠 있으면 goal이 영구적으로 드롭될 수 있음.

**현재 판단(커밋 로그 및 `git log`의 vehicle_approach 관련 커밋 참고):** 실제 3노드 시스템에서는 `vehicle_mission_node`가 같은 Nav2 액션 서버를 먼저 `wait_for_server()`로 기다려 성공적으로 써본 뒤에야 `/vehicle_approach/enable=true`를 발행하므로, `vehicle_approach_node`가 프레임을 처리할 시점엔 Nav2가 이미 한 번 검증된 상태 — 실무 위험은 낮다고 보고 이번 패스에서는 보류했다. 제대로 고치려면 "노드가 실제 전송 성공 여부를 파이프라인에 알려준 뒤에만 `_last_sent_goal`을 갱신" 하는 식으로 pipeline↔node 인터페이스를 다시 설계해야 함(한 줄 패치로 안 됨).

**할 일:** 이 판단에 동의하는지 검토. 동의하면 그대로 두거나 이슈로만 기록. 동의 안 하면 pipeline/node 인터페이스 재설계 필요.

## 3. 실측 캘리브레이션 필요 (배포 전 필수)

스펙 §7의 TBD 값들이 각 패키지의 `config/params.yaml`에 임시 기본값 + `# TBD` 주석으로 들어가 있다. 실측 후 이 YAML 파일들만 갱신하면 되고 코드 변경은 필요 없음.

각 값이 "무슨 뜻인지"와 "현장에서 어떻게 재는지"를 같이 적어둔다.

### `src/webcam_perception/config/params.yaml`

| 완료 | 값 | 무슨 뜻인가 | 어떻게 재는가 |
|---|---|---|---|
| O | `yolo_weights_path` | 웹캠 영상으로 학습시킨 YOLO 모델 파일(`.pt`) 경로 | 실측 아님 — 학습된 가중치 파일을 서버에 두고 절대경로만 적으면 됨 |
| X | `roi` | 웹캠 화면에서 "차량이 실제로 지나다니는 구간"만 잘라 보는 사각형. `[x_min, y_min, x_max, y_max]` (픽셀) | 웹캠을 실제 설치 위치에 달아놓고 영상을 띄운 뒤, 차량이 지나는 영역의 좌상단·우하단 픽셀 좌표를 읽어서 넣음 |
| X | `stop_duration_s` | 차량이 몇 초 동안 안 움직여야 "정지했다"고 판정할지(초) | 실제 차량이 서는 데 걸리는 평균 시간을 관찰해서 정함. 너무 짧으면 신호대기 등에도 오탐, 너무 길면 반응이 늦어짐 |
| X | `stop_pixel_threshold` | 정지 판정 시 "이 정도 픽셀 이내로 흔들리면 그냥 정지로 본다"는 허용 오차(px) | 차량이 실제로 서 있을 때 YOLO bbox 중심이 프레임마다 몇 px씩 흔들리는지 로그로 찍어보고, 그 흔들림 폭보다 살짝 크게 잡음 |
| X | `confidence_threshold` | YOLO가 "이건 차량이 맞다"고 확신하는 최소 점수(0~1) | 낮추면 오탐(차량 아닌 걸 차량으로) 늘고, 높이면 놓침(차량인데 못 잡음) 늘어남. 실제 영상 몇 개 돌려보면서 값 조절 |
| X | `homography_pixel_points` / `homography_map_points` | "웹캠 화면의 이 픽셀 지점 = 실제 지도(map)의 이 좌표"라는 대응점 쌍. 최소 4쌍(보통 바닥의 사각형 4개 코너) | 로봇 map 상 좌표를 이미 알고 있는 바닥 지점 4곳 이상을 정하고, 웹캠 화면에서 그 지점들의 픽셀 좌표를 읽어서 `homography_pixel_points`에, 실제 map 좌표(m)를 `homography_map_points`에 순서 맞춰 넣음 |

### `src/vehicle_mission/config/params.yaml`

| 완료 | 값 | 무슨 뜻인가 | 어떻게 재는가 |
|---|---|---|---|
| X | `waypoint_x` / `waypoint_y` | 로봇이 차량을 기다리는 대기 지점의 map 좌표(m) | RViz에서 map 위에 원하는 대기 지점을 클릭해 좌표를 읽거나, 로봇을 실제로 그 자리에 세운 뒤 `/tf`(map→base_link)에서 좌표를 뽑음 |
| X | `waypoint_yaw` | 그 대기 지점에서 로봇이 바라볼 방향(라디안) | 위와 동일한 방법으로, 로봇이 차량 쪽을 향하게 세운 뒤 헤딩값을 읽음 |

### `src/vehicle_approach/config/params.yaml`

| 완료 | 값 | 무슨 뜻인가 | 어떻게 재는가 |
|---|---|---|---|
| O | `yolo_weights_path` | 오크디(depth 카메라) RGB 영상 기준으로 학습된 YOLO 가중치 파일 경로 | 실측 아님 — 절대경로만 적으면 됨 |
| X | `camera_frame` | 오크디 카메라의 실제 TF 프레임 이름. 지금 값 `"camera_frame"`은 진짜 TF 트리엔 없는 자리표시자 | 오크디 xacro를 로봇 URDF에 붙인 뒤(§4의 1회성 작업), `ros2 run tf2_tools view_frames` 등으로 실제 RGB optical frame 이름을 확인해서 넣음 (depth frame이 아니라 **RGB optical frame**이어야 함) |
| X | `confidence_threshold` | 웹캠과 같은 개념(YOLO 확신도 임계치) | 웹캠 항목과 동일한 방식으로 오크디 영상 기준으로 조정 |
| X | `moving_average_window` | 차량 위치를 몇 프레임 평균 내서 떨림을 줄일지(프레임 수) | 값이 크면 안정적이지만 반응이 느려지고, 작으면 반응은 빠르지만 위치가 흔들림. 실제 영상 보면서 절충점 찾음 |
| X | `goal_resend_threshold_m` | 차량 위치가 이만큼(m) 이상 움직여야 Nav2한테 새 목적지를 다시 보냄 | 너무 작으면 매 프레임 목적지 재전송(스팸), 너무 크면 차량이 움직여도 로봇이 반응 안 함. 정지 차량 기준이므로 작은 값(수 cm~수십 cm)이면 충분 |
| X | `approach_completion_threshold_m` | 로봇-차량 거리가 이 이내(m)로 좁혀지면 "도착"으로 보고 Nav2 goal을 취소해 정지 | **주의:** 기본값(0.5m)이 depth 보정식의 최소 센싱 거리(0.6m→보정 후 0.608m)보다 작아서, 지금 값 그대로 두면 "도착" 판정이 수학적으로 절대 안 됨. 반드시 0.608m보다 크게, 그리고 Nav2의 `xy_goal_tolerance`보다도 크게 잡아야 함(그래야 Nav2가 먼저 "도착" 처리해버리는 경합을 피함) |

## 4. 배포 전 확인할 것 (코드는 이미 준비됨, 실행 전제 조건)

- `src/vehicle_approach/config/params.yaml`의 `depth_topic`이 `rgb_topic`과 **픽셀 정렬 + 동일 해상도**이고 **16UC1(mm) 인코딩**이어야 함 — depthai-ros 쪽 `i_align_depth: true` 설정 확인 (파일 상단 주석에 명시해둠).
- 오크디 xacro를 로봇 URDF에 통합하는 1회성 작업 (camera_frame↔base_link TF 연결, 스펙 §3.1 각주).
- 각 노드는 `--ros-args --params-file <path>` 없이 `ros2 run`만 하면 코드 내 기본값(위 TBD 값들)으로 조용히 동작하므로, 실행 시 반드시 params-file을 지정할 것 (각 `config/params.yaml` 상단에 정확한 실행 커맨드 주석으로 남겨둠).

## 5. 그 외 남은 것

- `main`으로 병합 여부 결정 (현재 미병합 상태 유지 중).
- 전체 워크스페이스 통합 빌드 한 번 확인: `colcon build --symlink-install` (지금까지는 패키지별로만 빌드/테스트함).
- 실물 하드웨어(웹캠, 오크디, 터틀봇, Nav2 스택) 연동 수동 검증 — 스펙 §1.2/§8에서 테스트 절차는 범위 밖으로 명시되어 있어 자동화 테스트에는 없음. 각 플랜 문서의 마지막 "선택, 하드웨어 필요" 스텝에 수동 검증 커맨드가 적혀 있음.
