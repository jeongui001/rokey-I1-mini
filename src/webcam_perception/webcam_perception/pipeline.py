import logging

import numpy as np

from webcam_perception.detection import Detection, select_best_detection
from webcam_perception.geometry import bbox_bottom_center, bbox_center, point_in_roi
from webcam_perception.homography import pixel_to_map
from webcam_perception.stop_detector import StopDetector

logger = logging.getLogger(__name__)


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
        logger.info(f'ROI 필터링: 전체 {len(detections)}건 -> ROI 내 {len(in_roi)}건')

        best = select_best_detection(in_roi, self.confidence_threshold)

        if best is None:
            logger.info(f'ROI 내 임계값({self.confidence_threshold:.2f}) 이상 detection 없음')
            self.stop_detector.reset()
            self._published_for_current_stop = False
            return None
        logger.info(f'best detection 선택: confidence={best.confidence:.2f}')

        cx, cy = bbox_center(best.x1, best.y1, best.x2, best.y2)
        stopped = self.stop_detector.update(now_sec, cx, cy)
        logger.info(f'정지 판정: stopped={stopped}')

        if not stopped:
            return None

        if self._published_for_current_stop:
            logger.info('이번 정지 구간에서 이미 publish함 -> 스킵')
            return None

        bx, by = bbox_bottom_center(best.x1, best.y1, best.x2, best.y2)
        try:
            map_x, map_y = pixel_to_map(self.homography_matrix, (bx, by))
        except Exception as ex:
            logger.error(f'homography 투영 실패: {ex}')
            return None
        logger.info(f'homography 투영 성공 및 publish 준비: map=({map_x:.3f}, {map_y:.3f})')
        self._published_for_current_stop = True
        return (map_x, map_y)
