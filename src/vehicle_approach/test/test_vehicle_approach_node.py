import numpy as np
from sensor_msgs.msg import Image

from vehicle_approach.vehicle_approach_node import VehicleApproachNode


def test_scale_detections_preserves_relative_bbox():
    from vehicle_approach.detection import Detection
    scaled = VehicleApproachNode._scale_detections(
        [Detection(100.0, 100.0, 200.0, 200.0, 0.9)],
        (480, 640),
        (240, 320),
    )[0]
    assert scaled.x1 == 50.0
    assert scaled.y1 == 50.0
    assert scaled.x2 == 100.0
    assert scaled.y2 == 100.0


def test_compressed_depth_16uc1_png_to_meters():
    import cv2
    from sensor_msgs.msg import CompressedImage

    depth_mm = np.array([[0, 1000], [2500, 4000]], dtype=np.uint16)
    ok, encoded = cv2.imencode('.png', depth_mm)
    assert ok

    msg = CompressedImage()
    msg.format = '16UC1; compressedDepth'
    # compressed_depth_image_transport가 붙이는 헤더가 있어도
    # PNG signature를 찾아 디코딩하는지 검증한다.
    msg.data = bytes(12) + encoded.tobytes()

    depth_m = VehicleApproachNode._compressed_depth_to_meters(msg)
    assert np.allclose(
        depth_m,
        np.array([[0.0, 1.0], [2.5, 4.0]], dtype=np.float32),
    )
