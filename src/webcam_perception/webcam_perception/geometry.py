def bbox_center(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]:
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def bbox_bottom_center(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]:
    return ((x1 + x2) / 2.0, y2)


def point_in_roi(
    point: tuple[float, float], roi: tuple[float, float, float, float]
) -> bool:
    x, y = point
    x_min, y_min, x_max, y_max = roi
    return x_min <= x <= x_max and y_min <= y <= y_max
