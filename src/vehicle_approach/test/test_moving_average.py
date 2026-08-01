import math

from vehicle_approach.moving_average import MovingAverageFilter


def test_single_sample_returns_itself():
    filt = MovingAverageFilter(window_size=3)
    filt.add(1.0, 2.0)
    x, y = filt.value()
    assert math.isclose(x, 1.0)
    assert math.isclose(y, 2.0)


def test_averages_within_window():
    filt = MovingAverageFilter(window_size=3)
    filt.add(1.0, 1.0)
    filt.add(2.0, 2.0)
    filt.add(3.0, 3.0)
    x, y = filt.value()
    assert math.isclose(x, 2.0)
    assert math.isclose(y, 2.0)


def test_drops_oldest_sample_beyond_window():
    filt = MovingAverageFilter(window_size=2)
    filt.add(1.0, 1.0)
    filt.add(2.0, 2.0)
    filt.add(3.0, 3.0)
    x, y = filt.value()
    assert math.isclose(x, 2.5)
    assert math.isclose(y, 2.5)


def test_reset_clears_samples():
    filt = MovingAverageFilter(window_size=2)
    filt.add(1.0, 1.0)
    filt.reset()
    filt.add(5.0, 5.0)
    x, y = filt.value()
    assert math.isclose(x, 5.0)
    assert math.isclose(y, 5.0)
