def correct_depth(raw_depth_m: float) -> float | None:
    if raw_depth_m < 0.6:
        return None  # 사용 안 함 (카메라 최소 센싱 거리 한계, 정상 동작)
    return 0.795 * raw_depth_m + 0.131
