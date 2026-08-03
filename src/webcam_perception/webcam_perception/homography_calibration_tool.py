"""맵 이미지와 웹캠 이미지에서 대응점을 클릭으로 찍어
homography_pixel_points / homography_map_points 를 계산하고
params.yaml에 반영하는 캘리브레이션 도구.

사용법:
python3 -m webcam_perception.homography_calibration_tool \
--map-yaml /path/to/map.yaml --camera-index 0

조작:
    - 웹캠 미리보기 창에서 SPACE: 현재 프레임 고정
    - 맵 창 클릭 -> 웹캠 창에서 대응점 클릭, 이를 반복 (최소 4쌍)
    - z: 마지막 포인트/쌍 취소
    - r: 전체 초기화
    - s: 4쌍 이상일 때 params.yaml 저장
    - q / ESC: 종료 (저장 안 함)
"""

import argparse
import math
import re
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np
import yaml

_MAX_DISPLAY_SIDE = 900


def load_map_yaml(map_yaml_path: Path) -> tuple[np.ndarray, float, tuple[float, float, float]]:
    with open(map_yaml_path) as f:
        meta = yaml.safe_load(f)

    image_path = (map_yaml_path.parent / meta['image']).resolve()
    image = cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(f'맵 이미지를 열 수 없습니다: {image_path}')

    resolution = float(meta['resolution'])
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    origin = (float(origin[0]), float(origin[1]), float(origin[2]))
    return image, resolution, origin


def pixel_to_world(
    px: float, py: float, image_height: int, resolution: float,
    origin: tuple[float, float, float],
) -> tuple[float, float]:
    """map.yaml의 origin(왼쪽 아래 픽셀의 world pose)/resolution 기준으로
    맵 이미지 픽셀(px, py; py는 이미지 위쪽이 0)을 world 좌표(m)로 변환."""
    origin_x, origin_y, yaw = origin
    dx = px * resolution
    dy = (image_height - py) * resolution
    cos_t, sin_t = math.cos(yaw), math.sin(yaw)
    world_x = origin_x + dx * cos_t - dy * sin_t
    world_y = origin_y + dx * sin_t + dy * cos_t
    return (world_x, world_y)


def compute_display_scale(width: int, height: int) -> float:
    longer_side = max(width, height)
    if longer_side <= _MAX_DISPLAY_SIDE:
        return 1.0
    return _MAX_DISPLAY_SIDE / longer_side


def update_params_yaml_text(
    text: str, pixel_points: list[tuple[float, float]], map_points: list[tuple[float, float]],
) -> str:
    flat_pixel = [v for point in pixel_points for v in point]
    flat_map = [v for point in map_points for v in point]

    def _sub(key: str, values: list[float], text: str) -> str:
        formatted = ', '.join(f'{v:.4f}' for v in values)
        new_text, count = re.subn(
            rf'^(\s*{key}\s*:\s*)\[[^\]]*\]',
            rf'\g<1>[{formatted}]',
            text, count=1, flags=re.MULTILINE,
        )
        if count == 0:
            raise ValueError(f'{key} 라인을 params.yaml에서 찾지 못했습니다')
        return new_text

    text = _sub('homography_pixel_points', flat_pixel, text)
    text = _sub('homography_map_points', flat_map, text)
    return text


class _Pair(NamedTuple):
    webcam_pixel: tuple[float, float]
    map_pixel: tuple[float, float]
    map_world: tuple[float, float]


def _draw_indexed_point(display: np.ndarray, dx: int, dy: int, index: int) -> None:
    cv2.circle(display, (dx, dy), 5, (0, 255, 0), -1)
    cv2.putText(display, str(index + 1), (dx + 6, dy - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


class _CalibrationSession:
    def __init__(self, map_image: np.ndarray, map_scale: float, resolution: float,
                 origin: tuple[float, float, float]):
        self.map_scale = map_scale
        self.image_height = map_image.shape[0]
        self.resolution = resolution
        self.origin = origin
        self.map_display_base = cv2.resize(
            map_image, None, fx=map_scale, fy=map_scale, interpolation=cv2.INTER_AREA,
        ) if map_scale != 1.0 else map_image.copy()

        self.webcam_base: np.ndarray | None = None
        self.pairs: list[_Pair] = []
        self.pending_map_pixel: tuple[float, float] | None = None
        self.awaiting = 'map'

    def on_map_click(self, x: int, y: int) -> None:
        if self.awaiting != 'map':
            return
        self.pending_map_pixel = (x / self.map_scale, y / self.map_scale)
        self.awaiting = 'webcam'

    def on_webcam_click(self, x: int, y: int) -> None:
        if self.awaiting != 'webcam' or self.pending_map_pixel is None:
            return
        map_world = pixel_to_world(
            *self.pending_map_pixel, self.image_height, self.resolution, self.origin,
        )
        self.pairs.append(_Pair((float(x), float(y)), self.pending_map_pixel, map_world))
        self.pending_map_pixel = None
        self.awaiting = 'map'

    def undo(self) -> None:
        if self.awaiting == 'webcam':
            self.pending_map_pixel = None
            self.awaiting = 'map'
        elif self.pairs:
            self.pairs.pop()

    def reset(self) -> None:
        self.pairs.clear()
        self.pending_map_pixel = None
        self.awaiting = 'map'

    def render_map(self) -> np.ndarray:
        display = self.map_display_base.copy()

        for i, pair in enumerate(self.pairs):
            px, py = pair.map_pixel
            _draw_indexed_point(display, int(px * self.map_scale), int(py * self.map_scale), i)

        if self.pending_map_pixel is not None:
            px, py = self.pending_map_pixel
            dx, dy = int(px * self.map_scale), int(py * self.map_scale)
            cv2.circle(display, (dx, dy), 5, (0, 165, 255), -1)

        status = (
            f'pairs={len(self.pairs)}  '
            f"{'맵 클릭 대기' if self.awaiting == 'map' else '웹캠 대응점 대기'}"
        )
        cv2.putText(display, status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)
        return display

    def render_webcam(self) -> np.ndarray:
        display = self.webcam_base.copy()
        for i, pair in enumerate(self.pairs):
            x, y = pair.webcam_pixel
            _draw_indexed_point(display, int(x), int(y), i)
        return display


def _freeze_webcam_frame(camera_index: int) -> np.ndarray:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f'웹캠(index={camera_index})을 열 수 없습니다')

    window = 'webcam (SPACE: freeze)'
    frame = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('웹캠 프레임을 읽을 수 없습니다')
            cv2.imshow(window, frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord(' '):
                break
            if key in (ord('q'), 27):
                raise SystemExit('사용자가 웹캠 고정을 취소했습니다')
    finally:
        cap.release()
        cv2.destroyWindow(window)
    return frame


def run_calibration(map_yaml: Path, camera_index: int, params_file: Path) -> None:
    map_image, resolution, origin = load_map_yaml(map_yaml)
    map_scale = compute_display_scale(map_image.shape[1], map_image.shape[0])

    webcam_frame = _freeze_webcam_frame(camera_index)

    session = _CalibrationSession(map_image, map_scale, resolution, origin)
    session.webcam_base = webcam_frame

    map_window, webcam_window = 'map', 'webcam'
    cv2.namedWindow(map_window)
    cv2.namedWindow(webcam_window)

    def _map_mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            session.on_map_click(x, y)

    def _webcam_mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            session.on_webcam_click(x, y)

    cv2.setMouseCallback(map_window, _map_mouse)
    cv2.setMouseCallback(webcam_window, _webcam_mouse)

    print('맵 클릭 -> 웹캠 대응점 클릭 순서로 최소 4쌍을 찍으세요.')
    print('z: undo  r: reset  s: save(params.yaml)  q/ESC: quit(저장 안함)')

    while True:
        cv2.imshow(map_window, session.render_map())
        cv2.imshow(webcam_window, session.render_webcam())
        key = cv2.waitKey(30) & 0xFF

        if key == ord('z'):
            session.undo()
        elif key == ord('r'):
            session.reset()
        elif key == ord('s'):
            if len(session.pairs) < 4:
                print(f'점이 부족합니다 ({len(session.pairs)}/4)')
                continue
            pixel_points = [pair.webcam_pixel for pair in session.pairs]
            map_points = [pair.map_world for pair in session.pairs]
            text = params_file.read_text()
            new_text = update_params_yaml_text(text, pixel_points, map_points)
            params_file.write_text(new_text)
            print(f'{params_file} 저장 완료 ({len(session.pairs)}쌍)')
            for i, pair in enumerate(session.pairs):
                print(f'  [{i + 1}] pixel={pair.webcam_pixel} -> map={pair.map_world}')
            break
        elif key in (ord('q'), 27):
            print('저장하지 않고 종료합니다.')
            break

    cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-yaml', required=True, type=Path, help='nav2 map.yaml 경로')
    parser.add_argument('--camera-index', type=int, default=0)
    parser.add_argument(
        '--params-file', type=Path,
        default=Path(__file__).resolve().parent.parent / 'config' / 'params.yaml',
    )
    args = parser.parse_args()
    run_calibration(args.map_yaml, args.camera_index, args.params_file)


if __name__ == '__main__':
    main()
