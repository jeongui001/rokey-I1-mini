import cv2
import numpy as np


def build_homography_matrix(
    pixel_points: list[tuple[float, float]], map_points: list[tuple[float, float]]
) -> np.ndarray:
    src = np.array(pixel_points, dtype=np.float64)
    dst = np.array(map_points, dtype=np.float64)
    matrix, _ = cv2.findHomography(src, dst)
    return matrix


def pixel_to_map(matrix: np.ndarray, pixel: tuple[float, float]) -> tuple[float, float]:
    src = np.array([[pixel]], dtype=np.float64)
    dst = cv2.perspectiveTransform(src, matrix)
    x, y = dst[0][0]
    return (float(x), float(y))
