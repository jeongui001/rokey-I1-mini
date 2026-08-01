def backproject(
    u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float
) -> tuple[float, float, float]:
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return (x, y, z)
