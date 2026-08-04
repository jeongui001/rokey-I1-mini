# rokey-I1-mini
[두산로보틱스 부트캠프 로키] 8기 b-3조 지능1 미니프로젝트


## 2026-08-04 추종 안정화 수정

- `/webcam/vehicle_detected`와 `/webcam/vehicle_map_pose`는 RC카가 보이는 동안 계속 발행됩니다.
- 구버전 정지 판정 one-shot 토픽은 기본적으로 비활성화했습니다.
- OAK-D Depth는 `/robot11/oakd/stereo/image_raw/compressedDepth`의 `16UC1; compressedDepth`를 직접 디코딩합니다.
- 첫 웨이포인트의 최종 회전만 실패했더라도 목표점 0.35 m 안이면 추종을 활성화합니다.
- 추종이 활성화되는 순간 OAK-D 동기화를 기다리지 않고 최신 웹캠 map 좌표로 첫 가이드 goal을 즉시 시도합니다.
- 도킹 초기 yaw는 `1.3708`, 언도킹 후 첫 웨이포인트 yaw는 `-1.7708`입니다.
