import logging
from dataclasses import dataclass

import numpy as np
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from tf2_ros import Buffer

from vehicle_approach.backprojection import backproject
from vehicle_approach.depth_correction import correct_depth
from vehicle_approach.detection import Detection, bbox_center, select_best_detection
from vehicle_approach.goal_calculation import (
    compute_goal_pose,
    is_approach_complete,
    should_resend_goal,
)
from vehicle_approach.moving_average import MovingAverageFilter

logger = logging.getLogger(__name__)


@dataclass
class ApproachResult:
    goal_pose: PoseStamped | None
    detection_center: Point | None
    completed: bool
    target_visible: bool
    target_distance_m: float | None


class VehicleApproachPipeline:
    def __init__(
        self,
        confidence_threshold: float,
        tf_buffer: Buffer,
        camera_frame: str,
        moving_average: MovingAverageFilter,
        resend_threshold_m: float,
        follow_distance_m: float,
        distance_deadband_m: float,
        bbox_inner_ratio: float,
        target_min_depth_m: float,
        target_max_depth_m: float,
        apply_depth_correction: bool,
    ):
        self.confidence_threshold = confidence_threshold
        self.tf_buffer = tf_buffer
        self.camera_frame = camera_frame
        self.moving_average = moving_average
        self.resend_threshold_m = resend_threshold_m
        self.follow_distance_m = follow_distance_m
        self.distance_deadband_m = distance_deadband_m
        self.bbox_inner_ratio = bbox_inner_ratio
        self.target_min_depth_m = target_min_depth_m
        self.target_max_depth_m = target_max_depth_m
        self.apply_depth_correction = apply_depth_correction
        self._last_sent_goal: tuple[float, float] | None = None

    def reset_tracking(self) -> None:
        self.moving_average.reset()
        self._last_sent_goal = None

    def process_frame(
        self,
        detections: list[Detection],
        depth_image_m: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        stamp,
        robot_x: float,
        robot_y: float,
    ) -> ApproachResult:
        best = select_best_detection(detections, self.confidence_threshold)
        if best is None:
            return ApproachResult(None, None, False, False, None)

        u, v = bbox_center(best.x1, best.y1, best.x2, best.y2)
        x1, y1, x2, y2 = self._inner_bbox(best, depth_image_m.shape)
        if x2 <= x1 or y2 <= y1:
            return ApproachResult(None, None, False, False, None)

        roi = depth_image_m[y1:y2, x1:x2]
        valid = roi[
            np.isfinite(roi)
            & (roi >= self.target_min_depth_m)
            & (roi <= self.target_max_depth_m)
        ]
        if valid.size < 5:
            return ApproachResult(None, None, False, False, None)

        raw_depth_m = float(np.median(valid))
        corrected = correct_depth(raw_depth_m) if self.apply_depth_correction else raw_depth_m
        if corrected is None:
            return ApproachResult(None, None, False, False, None)

        x, y, z = backproject(u, v, corrected, fx, fy, cx, cy)
        point_camera = PointStamped()
        point_camera.header.frame_id = self.camera_frame
        point_camera.header.stamp = stamp
        point_camera.point.x = x
        point_camera.point.y = y
        point_camera.point.z = z
        point_map = self.tf_buffer.transform(point_camera, 'map')

        detection_center = Point(
            x=point_map.point.x,
            y=point_map.point.y,
            z=0.0,
        )
        self.moving_average.add(point_map.point.x, point_map.point.y)
        avg_x, avg_y = self.moving_average.value()
        target_distance = float(np.hypot(avg_x - robot_x, avg_y - robot_y))

        complete_threshold = self.follow_distance_m + self.distance_deadband_m
        if is_approach_complete(
            robot_x,
            robot_y,
            avg_x,
            avg_y,
            complete_threshold,
        ):
            return ApproachResult(
                None,
                detection_center,
                True,
                True,
                target_distance,
            )

        goal_pose = compute_goal_pose(
            avg_x,
            avg_y,
            robot_x,
            robot_y,
            stand_off_distance_m=self.follow_distance_m,
        )
        if goal_pose is None:
            return ApproachResult(None, detection_center, True, True, target_distance)

        goal_xy = (
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
        )
        if self._last_sent_goal is not None and not should_resend_goal(
            goal_xy,
            self._last_sent_goal,
            self.resend_threshold_m,
        ):
            goal_pose = None
        else:
            self._last_sent_goal = goal_xy

        return ApproachResult(
            goal_pose,
            detection_center,
            False,
            True,
            target_distance,
        )

    def _inner_bbox(self, detection: Detection, image_shape) -> tuple[int, int, int, int]:
        height, width = image_shape
        x1 = max(0, min(width - 1, int(round(detection.x1))))
        y1 = max(0, min(height - 1, int(round(detection.y1))))
        x2 = max(0, min(width, int(round(detection.x2))))
        y2 = max(0, min(height, int(round(detection.y2))))

        ratio = min(max(float(self.bbox_inner_ratio), 0.2), 1.0)
        margin_x = int((x2 - x1) * (1.0 - ratio) * 0.5)
        margin_y = int((y2 - y1) * (1.0 - ratio) * 0.5)
        return x1 + margin_x, y1 + margin_y, x2 - margin_x, y2 - margin_y
