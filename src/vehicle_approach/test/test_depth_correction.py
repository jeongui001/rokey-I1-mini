import math

from vehicle_approach.depth_correction import correct_depth


def test_below_minimum_sensing_range_returns_none():
    assert correct_depth(0.5) is None


def test_at_minimum_sensing_range_is_corrected():
    result = correct_depth(0.6)
    assert result is not None
    assert math.isclose(result, 0.608, abs_tol=1e-4)


def test_known_sample_point_from_spec_validation_table():
    # 스펙 §5.2.2 검증표: 측정 0.84 -> 보정 0.799
    result = correct_depth(0.84)
    assert math.isclose(result, 0.799, abs_tol=1e-3)
