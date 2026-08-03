# vehicle_approach

`/vehicle_approach/enable`이 true가 되면 오크디(AMR 카메라)로 차량을 탐지해 뎁스 보정 →
역투영+TF 변환 → 이동평균 → goal 계산을 거쳐 Nav2로 반복 접근하다가, 임계 거리 이내에서 정지하는
패키지. `docs/PROJECT_NOTES.md`의 **§3 nav 파이프라인 2~7층**에 해당한다.

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
