from webcam_perception.detection import Detection, select_best_detection


def test_select_best_detection_returns_none_when_empty():
    assert select_best_detection([], confidence_threshold=0.5) is None


def test_select_best_detection_filters_below_threshold():
    detections = [Detection(x1=0.0, y1=0.0, x2=10.0, y2=10.0, confidence=0.3)]
    assert select_best_detection(detections, confidence_threshold=0.5) is None


def test_select_best_detection_picks_highest_confidence():
    low = Detection(x1=0.0, y1=0.0, x2=10.0, y2=10.0, confidence=0.6)
    high = Detection(x1=20.0, y1=20.0, x2=30.0, y2=30.0, confidence=0.9)
    result = select_best_detection([low, high], confidence_threshold=0.5)
    assert result is high
