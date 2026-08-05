# AMR(OAK-D) 탐지 영상 고속 표시 변경

## 변경 내용

- OAK-D RGB 표시 콜백을 RGB/Depth/CameraInfo 동기화 및 YOLO 추론 콜백과 분리
- ROS 2 `MultiThreadedExecutor(4)` 적용
- 최신 YOLO 박스를 다음 추론 결과가 나올 때까지 RGB 프레임 위에 유지
- `/vehicle_approach/annotated_image` 발행
- OpenCV 화면에 실제 표시 FPS 출력

추종 좌표 계산에는 기존과 동일하게 시간 동기화된 RGB, Depth, CameraInfo를
사용한다. 따라서 화면을 빠르게 만드는 변경이 거리 계산의 시간 정합성을 없애지
않는다.

## 실행

기존과 동일하게 실행한다.

```bash
cd ~/b3_int1_mini
colcon build --symlink-install
source install/setup.bash
ros2 launch vehicle_mission nav2_with_oakd.launch.py
```

기본 `debug_view: true`이므로 `vehicle_approach_debug` 창이 열린다. 별도 ROS 영상
뷰어를 사용하려면 다음을 실행한다.

```bash
ros2 run rqt_image_view rqt_image_view /vehicle_approach/annotated_image
```

## FPS 확인

```bash
ros2 topic hz /robot11/oakd/rgb/image_raw
ros2 topic hz /vehicle_approach/annotated_image

화면 왼쪽 위의 `DISPLAY`는 annotated 영상 발행/표시 주기이고,
`INPUT`은 OAK-D에서 실제로 새 RGB 프레임이 들어오는 주기이다. 표시 타이머는
30 Hz로 동작하지만 `INPUT`이 10 FPS라면 새 장면은 초당 10번만 갱신된다.
이 경우 OAK-D 드라이버의 RGB FPS 설정도 30으로 올려야 실제 장면도 30 FPS가 된다.
ros2 topic hz /vehicle_approach/annotated_image
```

두 번째 값은 첫 번째 값보다 높아질 수 없다. OAK-D RGB가 30 Hz로 발행되면 탐지
화면도 약 30 FPS로 갱신된다. 입력이 15 Hz라면 카메라 launch 설정의 RGB FPS를
30 이상으로 먼저 변경해야 한다.

## 파라미터

`src/vehicle_approach/config/params.yaml`:

```yaml
debug_view: true
publish_annotated_image: true
annotated_image_topic: "/vehicle_approach/annotated_image"
```

네트워크/CPU 부하를 줄이면서 OpenCV 창만 사용할 때는
`publish_annotated_image: false`로 바꿀 수 있다.
