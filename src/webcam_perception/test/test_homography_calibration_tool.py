import math

import pytest

from webcam_perception.homography_calibration_tool import (
    compute_display_scale,
    pixel_to_world,
    update_params_yaml_text,
)


def test_pixel_to_world_no_rotation():
    # origin=(왼쪽 아래 픽셀의 world pose), yaw=0, resolution=0.05
    origin = (-3.54, -1.11, 0.0)
    resolution = 0.05
    image_height = 100

    # 왼쪽 아래 픽셀(0, height) -> origin 그대로
    x, y = pixel_to_world(0.0, 100.0, image_height, resolution, origin)
    assert math.isclose(x, -3.54, abs_tol=1e-9)
    assert math.isclose(y, -1.11, abs_tol=1e-9)

    # 오른쪽으로 20px, 위로 10px 이동
    x, y = pixel_to_world(20.0, 90.0, image_height, resolution, origin)
    assert math.isclose(x, -3.54 + 20 * 0.05, abs_tol=1e-9)
    assert math.isclose(y, -1.11 + 10 * 0.05, abs_tol=1e-9)


def test_pixel_to_world_with_yaw():
    origin = (0.0, 0.0, math.pi / 2)
    resolution = 1.0
    image_height = 10

    # 90도 회전: local (dx=1, dy=0) -> world (0, 1)
    x, y = pixel_to_world(1.0, 10.0, image_height, resolution, origin)
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 1.0, abs_tol=1e-9)


def test_compute_display_scale_no_downscale_when_small():
    assert compute_display_scale(640, 480) == 1.0


def test_compute_display_scale_downscales_large_image():
    scale = compute_display_scale(1800, 900)
    assert scale < 1.0
    assert math.isclose(1800 * scale, 900.0, abs_tol=1e-6)


def test_update_params_yaml_text_replaces_only_target_lines():
    original = (
        'webcam_perception_node:\n'
        '  ros__parameters:\n'
        '    camera_index: 0\n'
        '    roi: [0.0, 0.0, 640.0, 480.0]          # TBD: 실제 웹캠 ROI 사각형(px)\n'
        '    homography_pixel_points: [0.0, 0.0, 640.0, 0.0, 0.0, 480.0, 640.0, 480.0]'
        '  # TBD: 캘리브레이션 대응점(px)\n'
        '    homography_map_points: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0]'
        '            # TBD: 캘리브레이션 대응점(map)\n'
    )
    pixel_points = [(1.0, 2.0), (3.0, 4.0)]
    map_points = [(0.1, 0.2), (0.3, 0.4)]

    updated = update_params_yaml_text(original, pixel_points, map_points)

    assert 'camera_index: 0\n' in updated
    assert 'roi: [0.0, 0.0, 640.0, 480.0]' in updated
    assert 'homography_pixel_points: [1.0000, 2.0000, 3.0000, 4.0000]' in updated
    assert 'homography_map_points: [0.1000, 0.2000, 0.3000, 0.4000]' in updated
    # 라인 뒤 주석은 유지
    assert '# TBD: 캘리브레이션 대응점(px)' in updated
    assert '# TBD: 캘리브레이션 대응점(map)' in updated


def test_update_params_yaml_text_raises_when_key_missing():
    text_without_map_key = 'webcam_perception_node:\n  ros__parameters:\n    camera_index: 0\n'

    with pytest.raises(ValueError):
        update_params_yaml_text(text_without_map_key, [(1.0, 2.0)], [(0.1, 0.2)])
