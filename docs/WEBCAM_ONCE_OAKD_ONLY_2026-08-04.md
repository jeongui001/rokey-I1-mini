# 초기 웹캠 1회 사용 후 OAK-D 전용 추종

동작 순서는 다음과 같다.

1. 웹캠의 전체 `640×480` ROI에서 RC카를 탐지한다.
2. 웹캠 검출을 미션 시작 조건과 초기 위치 가이드로 사용한다.
3. AMR의 OAK-D가 RC카를 한 번 탐지하면 OAK-D 전용 추종으로 영구 전환한다.
4. 전환 이후 웹캠의 검출 상태와 map 좌표는 이동 판단에 사용하지 않는다.
5. OAK-D가 RC카를 놓치면 Nav2 goal을 취소하고 정지하며, OAK-D가 다시 탐지하면 추종을 재개한다.

OAK-D 검출 화면은 추종 활성화 전에도 YOLO를 계속 수행한다. 따라서 아래 토픽에서
bbox, confidence, 거리와 현재 모드를 확인할 수 있다.

```bash
ros2 run rqt_image_view rqt_image_view /vehicle_approach/annotated_image
```

정상 전환 시 로그에 다음 문구가 한 번 출력된다.

```text
OAK-D detected RC car: webcam guidance permanently disabled
```
