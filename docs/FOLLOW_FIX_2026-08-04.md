# RC카 추종 시작 불가 수정 내역 (2026-08-04)

## 수정 사항

1. 외부 웹캠 지속 발행
   - `/webcam/vehicle_detected`: 매 프레임 True/False heartbeat
   - `/webcam/vehicle_map_pose`: RC카가 보이는 동안 매 프레임 map 좌표
   - 구버전 `/webcam/vehicle_initial_pose` 정지 one-shot 로직은 기본 비활성화

2. OAK-D compressedDepth 사용
   - `/robot11/oakd/stereo/image_raw/compressedDepth`
   - `16UC1; compressedDepth` PNG를 직접 디코딩해 meter로 변환

3. 첫 웨이포인트 위치 도달 fallback
   - Nav2가 최종 yaw 정렬 때문에 SUCCEEDED를 못 주더라도
   - 로봇이 `(-1.5, -2.5)`로부터 0.35 m 안이면 추종 활성화

4. 즉시 웹캠 가이드
   - `/vehicle_approach/enable=true`가 들어오면
   - 최신 웹캠 map 좌표가 있으면 OAK-D 동기화를 기다리지 않고 첫 Nav2 goal 시도

5. 방향 설정
   - 도킹 상태 initial yaw: `1.3708`
   - 언도킹 후 waypoint yaw: `-1.7708`

## 정상 로그 순서

```text
automatic initial pose: ... yaw=1.371
undock succeeded
sending first waypoint: x=-1.500, y=-2.500, yaw=-1.771
first waypoint reached; OAK-D/webcam follower enabled
vehicle approach enabled
webcam goal: x=..., y=...
```

OAK-D가 RC카를 잡으면 이후 다음 로그가 나옵니다.

```text
OAK-D FOLLOW
oakd goal: x=..., y=...
```

## 토픽 확인

```bash
ros2 topic hz /webcam/vehicle_detected
ros2 topic hz /webcam/vehicle_map_pose
ros2 topic echo /vehicle_approach/enable
ros2 topic hz /robot11/oakd/stereo/image_raw/compressedDepth
```
