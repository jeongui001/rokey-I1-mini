from builtin_interfaces.msg import Time

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer

from vehicle_approach.detection import Detection
from vehicle_approach.moving_average import MovingAverageFilter
from vehicle_approach.pipeline import VehicleApproachPipeline


def _make_buffer() -> Buffer:
    buffer = Buffer()

    # 로봇(base_link)이 map (2, 0)에 위치
    base_to_map = TransformStamped()
    base_to_map.header.frame_id = 'map'
    base_to_map.child_frame_id = 'base_link'
    base_to_map.transform.translation.x = 2.0
    base_to_map.transform.translation.y = 0.0
    base_to_map.transform.rotation.w = 1.0
    buffer.set_transform_static(base_to_map, 'test')

    # 카메라 광학축(Z, 정면)을 base_link의 X(정면)에 맞추는 Y축 90도 회전, 위치는 base_link 원점과 동일
    camera_to_base = TransformStamped()
    camera_to_base.header.frame_id = 'base_link'
    camera_to_base.child_frame_id = 'camera_frame'
    camera_to_base.transform.rotation.y = 0.70710678
    camera_to_base.transform.rotation.w = 0.70710678
    buffer.set_transform_static(camera_to_base, 'test')

    return buffer


def _make_pipeline(resend_threshold_m=0.05, completion_threshold_m=0.1, window_size=1):
    return VehicleApproachPipeline(
        confidence_threshold=0.5,
        tf_buffer=_make_buffer(),
        camera_frame='camera_frame',
        moving_average=MovingAverageFilter(window_size=window_size),
        resend_threshold_m=resend_threshold_m,
        completion_threshold_m=completion_threshold_m,
    )


def _stamp() -> Time:
    return Time(sec=0, nanosec=0)


def test_no_detection_returns_no_goal():
    pipeline = _make_pipeline()
    depth_image = np.full((480, 640), 2000, dtype=np.uint16)
    result = pipeline.process_frame(
        [], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )
    assert result.goal_pose is None
    assert result.detection_center is None
    assert result.completed is False


def test_depth_below_minimum_range_returns_no_goal():
    pipeline = _make_pipeline()
    depth_image = np.full((480, 640), 500, dtype=np.uint16)  # 0.5m < 0.6m 최소 센싱 거리
    detection = Detection(x1=300.0, y1=220.0, x2=340.0, y2=260.0, confidence=0.9)
    result = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )
    assert result.goal_pose is None
    assert result.detection_center is None


def test_detected_vehicle_publishes_center_and_sends_goal_facing_it():
    pipeline = _make_pipeline(resend_threshold_m=0.05, completion_threshold_m=0.1)
    depth_image = np.full((480, 640), 2000, dtype=np.uint16)  # raw 2.0m -> 보정 1.721m
    detection = Detection(x1=300.0, y1=220.0, x2=340.0, y2=260.0, confidence=0.9)  # bbox 중심 == 주점(320,240)

    result = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )

    # 카메라 정면 1.721m -> base_link X축 +1.721m -> map (2+1.721, 0) = (3.721, 0)
    assert result.detection_center is not None
    assert result.detection_center.x == pytest.approx(3.721, abs=1e-3)
    assert result.detection_center.y == pytest.approx(0.0, abs=1e-6)

    assert result.goal_pose is not None
    assert result.goal_pose.pose.position.x == pytest.approx(3.721, abs=1e-3)
    assert result.goal_pose.pose.orientation.z == pytest.approx(0.0, abs=1e-6)
    assert result.goal_pose.pose.orientation.w == pytest.approx(1.0, abs=1e-6)
    assert result.completed is False


def test_repeated_same_position_does_not_resend_goal():
    pipeline = _make_pipeline(resend_threshold_m=0.05, completion_threshold_m=0.1)
    depth_image = np.full((480, 640), 2000, dtype=np.uint16)
    detection = Detection(x1=300.0, y1=220.0, x2=340.0, y2=260.0, confidence=0.9)

    first = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )
    second = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )

    assert first.goal_pose is not None
    assert second.goal_pose is None  # 변화량이 재전송 임계치 미만이면 재전송하지 않음 (스펙 §5.2.5)


def test_close_distance_marks_completed_and_stops_sending_goal():
    pipeline = _make_pipeline(completion_threshold_m=3.0)  # 실제 거리(1.721m)보다 큰 임계치
    depth_image = np.full((480, 640), 2000, dtype=np.uint16)
    detection = Detection(x1=300.0, y1=220.0, x2=340.0, y2=260.0, confidence=0.9)

    result = pipeline.process_frame(
        [detection], depth_image, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        stamp=_stamp(), robot_x=2.0, robot_y=0.0,
    )

    assert result.completed is True
    assert result.goal_pose is None
