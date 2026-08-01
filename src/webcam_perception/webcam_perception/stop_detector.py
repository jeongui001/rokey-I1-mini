class StopDetector:
    def __init__(self, duration_s: float, pixel_threshold: float):
        self.duration_s = duration_s
        self.pixel_threshold = pixel_threshold
        self._samples: list[tuple[float, float, float]] = []

    def update(self, t: float, x: float, y: float) -> bool:
        self._samples.append((t, x, y))
        cutoff = t - self.duration_s

        # anchor: cutoff 시점 이전(또는 시점)의 가장 최근 샘플 — 현재 윈도우가
        # 실제로 duration_s만큼의 연속 데이터를 커버한다는 증거로 사용한다.
        # 처리 지연(느린 추론 등)으로 공백이 생기면 anchor가 없어 False를 반환하며,
        # 이는 "첫 호출 이후 경과 시간"만 보는 것과 달리 처리 지연에 의한 오탐을 막는다.
        anchor = None
        kept = []
        for s in self._samples:
            if s[0] <= cutoff:
                anchor = s
            else:
                kept.append(s)

        if anchor is None:
            self._samples = kept
            return False

        self._samples = [anchor] + kept
        xs = [s[1] for s in self._samples]
        ys = [s[2] for s in self._samples]
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
        return spread <= self.pixel_threshold

    def reset(self) -> None:
        self._samples = []
