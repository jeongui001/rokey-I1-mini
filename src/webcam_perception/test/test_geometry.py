from webcam_perception.geometry import bbox_bottom_center, bbox_center, point_in_roi


def test_bbox_center():
    assert bbox_center(0.0, 0.0, 10.0, 20.0) == (5.0, 10.0)


def test_bbox_bottom_center():
    assert bbox_bottom_center(0.0, 0.0, 10.0, 20.0) == (5.0, 20.0)


def test_point_in_roi_inside():
    assert point_in_roi((5.0, 5.0), (0.0, 0.0, 10.0, 10.0)) is True


def test_point_in_roi_outside():
    assert point_in_roi((15.0, 5.0), (0.0, 0.0, 10.0, 10.0)) is False


def test_point_in_roi_boundary_is_inside():
    assert point_in_roi((0.0, 0.0), (0.0, 0.0, 10.0, 10.0)) is True
