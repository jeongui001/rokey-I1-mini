import numpy as np

from webcam_perception.detection import Detection, select_best_detection
from webcam_perception.geometry import bbox_bottom_center, bbox_center, point_in_roi
from webcam_perception.homography import pixel_to_map
from webcam_perception.stop_detector import StopDetector


class VehicleStopPipeline:
    def __init__(
        self,
        roi: tuple[float, float, float, float],
        confidence_threshold: float,
        homography_matrix: np.ndarray,
        stop_detector: StopDetector,
    ):
        self.roi = roi
        self.confidence_threshold = confidence_threshold
        self.homography_matrix = homography_matrix
        self.stop_detector = stop_detector
        self._published_for_current_stop = False

    def process_detections(
        self, detections: list[Detection], now_sec: float
    ) -> tuple[float, float] | None:
        in_roi = [
            d for d in detections
            if point_in_roi(bbox_center(d.x1, d.y1, d.x2, d.y2), self.roi)
        ]
        best = select_best_detection(in_roi, self.confidence_threshold)

        if best is None:
            self.stop_detector.reset()
            self._published_for_current_stop = False
            return None

        cx, cy = bbox_center(best.x1, best.y1, best.x2, best.y2)
        stopped = self.stop_detector.update(now_sec, cx, cy)

        if not stopped:
            return None

        if self._published_for_current_stop:
            return None

        bx, by = bbox_bottom_center(best.x1, best.y1, best.x2, best.y2)
        map_x, map_y = pixel_to_map(self.homography_matrix, (bx, by))
        self._published_for_current_stop = True
        return (map_x, map_y)
