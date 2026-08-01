from dataclasses import dataclass

import numpy as np
import tf2_geometry_msgs  # noqa: F401 -- PointStamped 변환 등록을 위해 필요
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


@dataclass
class ApproachResult:
    goal_pose: PoseStamped | None
    detection_center: Point | None
    completed: bool


class VehicleApproachPipeline:
    def __init__(
        self,
        confidence_threshold: float,
        tf_buffer: Buffer,
        camera_frame: str,
        moving_average: MovingAverageFilter,
        resend_threshold_m: float,
        completion_threshold_m: float,
    ):
        self.confidence_threshold = confidence_threshold
        self.tf_buffer = tf_buffer
        self.camera_frame = camera_frame
        self.moving_average = moving_average
        self.resend_threshold_m = resend_threshold_m
        self.completion_threshold_m = completion_threshold_m
        self._last_sent_goal: tuple[float, float] | None = None

    def process_frame(
        self,
        detections: list[Detection],
        depth_image: np.ndarray,
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
            return ApproachResult(goal_pose=None, detection_center=None, completed=False)

        u, v = bbox_center(best.x1, best.y1, best.x2, best.y2)
        raw_depth_m = float(depth_image[int(round(v)), int(round(u))]) / 1000.0
        corrected = correct_depth(raw_depth_m)
        if corrected is None:
            return ApproachResult(goal_pose=None, detection_center=None, completed=False)

        x, y, z = backproject(u, v, corrected, fx, fy, cx, cy)
        point_camera = PointStamped()
        point_camera.header.frame_id = self.camera_frame
        point_camera.header.stamp = stamp
        point_camera.point.x = x
        point_camera.point.y = y
        point_camera.point.z = z
        point_map = self.tf_buffer.transform(point_camera, 'map')

        # 모니터링용 detection_center는 이동평균 이전, 매 프레임 원시 값 (스펙 §4)
        detection_center = Point(x=point_map.point.x, y=point_map.point.y, z=0.0)

        self.moving_average.add(point_map.point.x, point_map.point.y)
        avg_x, avg_y = self.moving_average.value()

        if is_approach_complete(robot_x, robot_y, avg_x, avg_y, self.completion_threshold_m):
            return ApproachResult(goal_pose=None, detection_center=detection_center, completed=True)

        goal_pose = None
        if self._last_sent_goal is None or should_resend_goal(
            (avg_x, avg_y), self._last_sent_goal, self.resend_threshold_m
        ):
            goal_pose = compute_goal_pose(avg_x, avg_y, robot_x, robot_y)
            self._last_sent_goal = (avg_x, avg_y)

        return ApproachResult(goal_pose=goal_pose, detection_center=detection_center, completed=False)
