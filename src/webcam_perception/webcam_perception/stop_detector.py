class StopDetector:
    def __init__(self, duration_s: float, pixel_threshold: float):
        self.duration_s = duration_s
        self.pixel_threshold = pixel_threshold
        self._samples: list[tuple[float, float, float]] = []
        self._first_update_t: float | None = None

    def update(self, t: float, x: float, y: float) -> bool:
        if self._first_update_t is None:
            self._first_update_t = t
        self._samples.append((t, x, y))
        cutoff = t - self.duration_s
        self._samples = [s for s in self._samples if s[0] >= cutoff]

        if t - self._first_update_t < self.duration_s:
            return False

        xs = [s[1] for s in self._samples]
        ys = [s[2] for s in self._samples]
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
        return spread <= self.pixel_threshold

    def reset(self) -> None:
        self._samples = []
        self._first_update_t = None
