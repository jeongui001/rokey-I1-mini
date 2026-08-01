import math

from vehicle_approach.backprojection import backproject


def test_backproject_principal_point_has_zero_lateral_offset():
    x, y, z = backproject(u=320.0, v=240.0, z=1.0, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert z == 1.0


def test_backproject_offset_pixel():
    x, y, z = backproject(u=420.0, v=240.0, z=2.0, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    assert math.isclose(x, 0.4, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert z == 2.0
