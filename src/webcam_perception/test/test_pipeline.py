import math

from webcam_perception.detection import Detection
from webcam_perception.homography import build_homography_matrix
from webcam_perception.pipeline import VehicleStopPipeline
from webcam_perception.stop_detector import StopDetector


def _make_pipeline(duration_s=2.0, pixel_threshold=5.0, confidence_threshold=0.5):
    pixel_points = [(0.0, 0.0), (100.0, 0.0), (0.0, 100.0), (100.0, 100.0)]
    map_points = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)]
    matrix = build_homography_matrix(pixel_points, map_points)
    return VehicleStopPipeline(
        roi=(0.0, 0.0, 100.0, 100.0),
        confidence_threshold=confidence_threshold,
        homography_matrix=matrix,
        stop_detector=StopDetector(duration_s=duration_s, pixel_threshold=pixel_threshold),
    )


def test_no_detections_returns_none():
    pipeline = _make_pipeline()
    assert pipeline.process_detections([], now_sec=0.0) is None


def test_detection_outside_roi_returns_none():
    pipeline = _make_pipeline()
    detection = Detection(x1=200.0, y1=200.0, x2=220.0, y2=220.0, confidence=0.9)
    assert pipeline.process_detections([detection], now_sec=0.0) is None


def test_detection_below_confidence_returns_none():
    pipeline = _make_pipeline(confidence_threshold=0.8)
    detection = Detection(x1=40.0, y1=40.0, x2=60.0, y2=60.0, confidence=0.5)
    assert pipeline.process_detections([detection], now_sec=0.0) is None


def test_not_stopped_yet_returns_none():
    pipeline = _make_pipeline(duration_s=2.0)
    detection = Detection(x1=40.0, y1=40.0, x2=60.0, y2=60.0, confidence=0.9)
    assert pipeline.process_detections([detection], now_sec=0.0) is None


def test_stopped_publishes_once_then_suppresses_repeat():
    pipeline = _make_pipeline(duration_s=2.0, pixel_threshold=5.0)
    detection = Detection(x1=40.0, y1=40.0, x2=60.0, y2=60.0, confidence=0.9)

    assert pipeline.process_detections([detection], now_sec=0.0) is None
    assert pipeline.process_detections([detection], now_sec=1.0) is None
    result = pipeline.process_detections([detection], now_sec=2.0)

    assert result is not None
    x, y = result
    assert math.isclose(x, 0.5, abs_tol=1e-6)
    assert math.isclose(y, 0.6, abs_tol=1e-6)

    # 계속 정지 상태 유지 중이면 재발행하지 않는다 (이벤트당 1회, 스펙 §2 step2)
    assert pipeline.process_detections([detection], now_sec=3.0) is None


def test_leaving_roi_then_stopping_again_republishes():
    pipeline = _make_pipeline(duration_s=2.0, pixel_threshold=5.0)
    detection = Detection(x1=40.0, y1=40.0, x2=60.0, y2=60.0, confidence=0.9)
    outside = Detection(x1=200.0, y1=200.0, x2=220.0, y2=220.0, confidence=0.9)

    pipeline.process_detections([detection], now_sec=0.0)
    pipeline.process_detections([detection], now_sec=1.0)
    first = pipeline.process_detections([detection], now_sec=2.0)
    assert first is not None

    pipeline.process_detections([outside], now_sec=3.0)  # 차량이 ROI를 벗어남 → 새 이벤트 준비

    pipeline.process_detections([detection], now_sec=4.0)
    pipeline.process_detections([detection], now_sec=5.0)
    second = pipeline.process_detections([detection], now_sec=6.0)
    assert second is not None
