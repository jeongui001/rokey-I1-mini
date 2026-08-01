from webcam_perception.stop_detector import StopDetector


def test_not_stopped_before_duration_elapsed():
    detector = StopDetector(duration_s=2.0, pixel_threshold=5.0)
    assert detector.update(t=0.0, x=100.0, y=100.0) is False
    assert detector.update(t=1.0, x=100.0, y=100.0) is False


def test_stopped_after_duration_within_threshold():
    detector = StopDetector(duration_s=2.0, pixel_threshold=5.0)
    detector.update(t=0.0, x=100.0, y=100.0)
    detector.update(t=1.0, x=101.0, y=100.0)
    assert detector.update(t=2.0, x=100.0, y=101.0) is True


def test_not_stopped_when_position_jumps_beyond_threshold():
    detector = StopDetector(duration_s=2.0, pixel_threshold=5.0)
    detector.update(t=0.0, x=100.0, y=100.0)
    detector.update(t=1.0, x=100.0, y=100.0)
    assert detector.update(t=2.0, x=200.0, y=100.0) is False


def test_reset_clears_history():
    detector = StopDetector(duration_s=2.0, pixel_threshold=5.0)
    detector.update(t=0.0, x=100.0, y=100.0)
    detector.update(t=2.0, x=100.0, y=100.0)
    detector.reset()
    assert detector.update(t=2.1, x=100.0, y=100.0) is False


def test_not_stopped_after_processing_stall_with_moving_point():
    # 차량은 계속 이동 중(정지한 적 없음)인데, 처리 지연(예: 느린 YOLO 추론)으로
    # duration_s보다 긴 공백이 생기면 재개 시점 슬라이딩 윈도우가 사실상
    # 샘플 1개로 쪼그라들어 spread가 0에 수렴한다. "첫 호출 이후 경과 시간"만으로
    # duration 게이트를 판단하면 이 상황에서 오탐(false positive)으로 정지 판정이 나온다.
    # 현재 윈도우가 실제로 duration_s만큼의 연속 데이터를 커버하는지를 봐야 한다.
    detector = StopDetector(duration_s=2.0, pixel_threshold=5.0)
    detector.update(t=0.0, x=0.0, y=0.0)
    detector.update(t=0.5, x=50.0, y=0.0)
    # 처리 지연 발생: t=0.5 이후 9.5초 만에 재개, 그 사이 차량은 계속 이동
    assert detector.update(t=10.0, x=500.0, y=0.0) is False
