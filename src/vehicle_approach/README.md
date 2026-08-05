# vehicle_approach

## OAK-D 탐지 영상 (고속 표시)

추론/Depth 동기화 처리와 화면 표시 경로를 분리했다. 따라서 검출 박스는 최신
YOLO 결과를 유지하면서 RGB 프레임은 OAK-D 입력 속도로 계속 갱신된다. 기본으로
OpenCV 창과 `/vehicle_approach/annotated_image` 토픽을 모두 제공하며 화면 왼쪽 위에
실측 `DISPLAY ... FPS`를 표시한다.

YOLO 검출은 추종 활성화 여부와 무관하게 실행되므로, `enable=false`인 초기 단계에도
OAK-D 화면에서 RC카 bbox와 confidence를 확인할 수 있다.

```bash
ros2 topic hz /vehicle_approach/annotated_image
ros2 run rqt_image_view rqt_image_view /vehicle_approach/annotated_image
```

실제 표시 FPS의 상한은 OAK-D의 RGB 발행 FPS이다. RGB 토픽이 30 Hz 이상이면 이
화면도 약 30 FPS로 갱신되며, 먼저 아래 명령으로 카메라 입력 속도를 확인할 수 있다.

```bash
ros2 topic hz /robot11/oakd/rgb/image_raw
```
3
`/vehicle_approach/enable`이 true가 되면 오크디(AMR 카메라)로 차량을 탐지해 뎁스 보정 →
역투영+TF 변환 → 이동평균 → goal 계산을 거쳐 Nav2로 반복 접근하다가, 임계 거리 이내에서 정지하는
패키지. `docs/PROJECT_NOTES.md`의 **§3 nav 파이프라인 2~7층**에 해당한다.

## 카메라 전환 순서

1. 웹캠 전체 ROI 검출로 미션을 시작하고 웹캠 map 좌표로 초기 접근한다.
2. OAK-D가 RC카를 처음 탐지하면 `OAK-D FOLLOW`로 한 번만 전환한다.
3. 이후에는 웹캠 검출 유무와 웹캠 좌표를 무시하고 OAK-D RGB·Depth만 사용한다.
4. OAK-D가 RC카를 놓치면 현재 Nav2 goal을 취소하고 정지한 채 OAK-D 재탐지를 기다린다.
   웹캠 좌표로 되돌아가지 않는다.

## 학습 순서

1. `vehicle_approach/detection.py` — 탐지 결과(`Detection`), bbox 중심 계산, confidence 기준 최적 탐지 선택 (3층)
2. `vehicle_approach/depth_correction.py` — 원본 뎁스값에 선형 보정식 적용 (4층, 뎁스 보정 알고리즘)
3. `vehicle_approach/backprojection.py` — 픽셀좌표+깊이+내부파라미터 → camera_frame 기준 3D 좌표 역투영 (5층)
4. `vehicle_approach/moving_average.py` — 차량 map 좌표 이력을 슬라이딩 윈도우로 평균화 (6층)
5. `vehicle_approach/goal_calculation.py` — 목표 pose 계산, goal 재전송 여부, 접근 완료 판정 (7층)
6. `vehicle_approach/pipeline.py` — 위 요소들을 한 프레임 단위로 묶어 실행하는 `VehicleApproachPipeline`. 역투영 결과를 TF(camera_frame→map)로 체이닝하는 지점(5층 마지막 단계)이 여기 있음
7. `vehicle_approach/vehicle_approach_node.py` — 오크디 RGB/Depth/CameraInfo 동기화 구독, TF 버퍼, Nav2 액션 클라이언트를 연결해 `pipeline`을 매 프레임 호출하는 ROS2 노드

## 참고 문서

- `docs/PROJECT_NOTES.md` §3 (2~7층), §4 (좌표계/데이터 명세)
- `docs/superpowers/specs/2026-08-01-vehicle-approach-design.md`
- `docs/superpowers/plans/2026-08-01-vehicle-approach-node-plan.md`
- `docs/NEXT_STEPS.md` §2 — pipeline↔node 인터페이스 관련 보류 이슈, §3 — `config/params.yaml` 실측 필요 값
- `test/` — 파일별 단위 테스트가 위 순서와 1:1로 대응됨
