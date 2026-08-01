class MovingAverageFilter:
    def __init__(self, window_size: int):
        self.window_size = window_size
        self._samples: list[tuple[float, float]] = []

    def add(self, x: float, y: float) -> None:
        self._samples.append((x, y))
        if len(self._samples) > self.window_size:
            self._samples.pop(0)

    def value(self) -> tuple[float, float]:
        xs = [s[0] for s in self._samples]
        ys = [s[1] for s in self._samples]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    def reset(self) -> None:
        self._samples = []
