# webcam_perception

웹캠으로 지정 ROI를 상시 감시하다가 차량이 들어와 정지하면, 픽셀 좌표를 map 좌표로 변환해
`/webcam/vehicle_initial_pose`로 1회 발행하는 패키지. `docs/PROJECT_NOTES.md`의 **§2 웹캠
파이프라인 (W1~W3층)** 에 해당한다.

## 학습 순서

1. `webcam_perception/detection.py` — 탐지 결과(`Detection`)와 confidence 기준 최적 탐지 선택 (W2층)
2. `webcam_perception/geometry.py` — bbox 중심/하단중심 계산, ROI 내부 판정 (W2층)
3. `webcam_perception/homography.py` — 픽셀↔map 좌표 변환 행렬 계산 및 적용 (W3층)
4. `webcam_perception/stop_detector.py` — 연속 프레임 위치 이력으로 정지 여부 판정
5. `webcam_perception/pipeline.py` — 위 요소들을 묶어 "ROI 진입 → 정지 판정 → map 좌표 산출"까지 처리하는 `VehicleStopPipeline` (W1~W3층 통합)
6. `webcam_perception/webcam_perception_node.py` — YOLO 추론, 웹캠 구독, `pipeline`을 호출해 최종 위치를 1회 발행하는 ROS2 노드

## 참고 문서

- `docs/PROJECT_NOTES.md` §2 (웹캠 파이프라인), §4 (좌표계/데이터 명세)
- `docs/superpowers/plans/2026-08-01-webcam-perception-node-plan.md`
- `config/params.yaml` — ROI, YOLO 가중치 경로, homography 대응점 등 실행 파라미터
- `test/` — 파일별 단위 테스트가 위 순서와 1:1로 대응됨
