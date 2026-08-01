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
