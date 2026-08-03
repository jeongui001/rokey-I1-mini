# vehicle_mission
1
로봇을 map 상 고정된 대기 지점으로 이동시키고, 도착하면 `/vehicle_approach/enable`을 발행해
`vehicle_approach_node`를 활성화하는 패키지. `docs/PROJECT_NOTES.md` §1 정상 플로우의
"로봇이 대기 지점으로 이동" 단계를 담당하며, 노트에는 별도 레이어 번호가 없어 코드 의존관계
순으로 정리했다.

## 학습 순서

1. `vehicle_mission/pose_utils.py` — (x, y, yaw) → `PoseStamped` 변환
2. `vehicle_mission/nav_result.py` — Nav2 액션 결과 상태를 성공/실패로 판정
3. `vehicle_mission/vehicle_mission_node.py` — 웹캠 초기 위치 구독(로깅 전용, 이동 목표 계산에는 미사용), 대기 지점으로 Nav2 goal 전송, 도착 시 `/vehicle_approach/enable` 발행까지 담당하는 ROS2 노드

## 참고 문서

- `docs/PROJECT_NOTES.md` §1 (정상 플로우)
- `docs/superpowers/plans/2026-08-01-vehicle-mission-node-plan.md`
- `config/params.yaml` — 대기 지점 좌표(`waypoint_x/y/yaw`)
- `test/` — 파일별 단위 테스트가 위 순서와 1:1로 대응됨
