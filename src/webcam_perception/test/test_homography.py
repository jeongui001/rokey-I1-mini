import math

from webcam_perception.homography import build_homography_matrix, pixel_to_map


def test_pixel_to_map_scale_and_translate():
    # 픽셀 (0,0)-(100,100) 정사각형이 map (0,0)-(1,1) 정사각형에 대응하는 단순 스케일 매칭
    pixel_points = [(0.0, 0.0), (100.0, 0.0), (0.0, 100.0), (100.0, 100.0)]
    map_points = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)]
    matrix = build_homography_matrix(pixel_points, map_points)

    x, y = pixel_to_map(matrix, (50.0, 50.0))

    assert math.isclose(x, 0.5, abs_tol=1e-6)
    assert math.isclose(y, 0.5, abs_tol=1e-6)


def test_pixel_to_map_corner_point():
    pixel_points = [(0.0, 0.0), (100.0, 0.0), (0.0, 100.0), (100.0, 100.0)]
    map_points = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)]
    matrix = build_homography_matrix(pixel_points, map_points)

    x, y = pixel_to_map(matrix, (100.0, 100.0))

    assert math.isclose(x, 1.0, abs_tol=1e-6)
    assert math.isclose(y, 1.0, abs_tol=1e-6)
