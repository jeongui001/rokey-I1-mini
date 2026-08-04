from builtin_interfaces.msg import Time

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer

from vehicle_approach.detection import Detection
from vehicle_approach.moving_average import MovingAverageFilter
from vehicle_approach.pipeline import VehicleApproachPipeline


def _buffer():
    buffer = Buffer()
    base = TransformStamped()
    base.header.frame_id = 'map'
    base.child_frame_id = 'base_link'
    base.transform.translation.x = 2.0
    base.transform.rotation.w = 1.0
    buffer.set_transform_static(base, 'test')

    camera = TransformStamped()
    camera.header.frame_id = 'base_link'
    camera.child_frame_id = 'camera_frame'
    camera.transform.rotation.y = 0.70710678
    camera.transform.rotation.w = 0.70710678
    buffer.set_transform_static(camera, 'test')
    return buffer


def _pipeline(follow=0.7, deadband=0.08):
    return VehicleApproachPipeline(
        confidence_threshold=0.5,
        tf_buffer=_buffer(),
        camera_frame='camera_frame',
        moving_average=MovingAverageFilter(window_size=1),
        resend_threshold_m=0.05,
        follow_distance_m=follow,
        distance_deadband_m=deadband,
        bbox_inner_ratio=0.6,
        target_min_depth_m=0.2,
        target_max_depth_m=4.0,
        apply_depth_correction=True,
    )


def test_goal_is_before_vehicle():
    pipeline = _pipeline()
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    detection = Detection(300.0, 220.0, 340.0, 260.0, 0.9)
    result = pipeline.process_frame(
        [detection], depth, 500.0, 500.0, 320.0, 240.0,
        Time(), 2.0, 0.0,
    )
    assert result.target_visible
    assert result.goal_pose is not None
    # corrected depth 1.721m, target map x 3.721; 0.7m 앞 goal은 3.021
    assert result.goal_pose.pose.position.x == pytest.approx(3.021, abs=1e-3)


def test_close_target_is_completed():
    pipeline = _pipeline(follow=2.0)
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    detection = Detection(300.0, 220.0, 340.0, 260.0, 0.9)
    result = pipeline.process_frame(
        [detection], depth, 500.0, 500.0, 320.0, 240.0,
        Time(), 2.0, 0.0,
    )
    assert result.completed
    assert result.goal_pose is None
