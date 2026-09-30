#!/usr/bin/env python3
"""Draw the LiDAR points of a range of recorded frames onto their camera images.

Set FOLDER, START_FRAME and END_FRAME below, then run:  python3 checklidarcamera.py
Each frame is saved as OUTPUT_DIR/<run>/<frame 8 digits>.jpg.

The LiDAR scan matched to each frame, its decoding, the road plane and the camera
projection are main_live.py's own (camera_calib_best_effort.json), so the picture is
what the analysis sees. Points are coloured by height above the fitted road plane:
blue at -HEIGHT_RANGE_CM or lower, green at 0, red at +HEIGHT_RANGE_CM or higher.

The road plane is fitted to the LiDAR firing nearest the camera's optical axis (0) and
PLANE_SIDE_FIRINGS firings on each side. Lines through the points of firings -100, 0
and +100 (top to bottom) show where those plane points lie.
"""

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

import main_live as ml

# =============================================================================
# SETTINGS
# =============================================================================

FOLDER = "/mnt/ssd/porthole_runs/20260930/20260930_1530"  # a run folder, or just "20260930_1530"
START_FRAME = 0  # first frame number (frames/00000000.jpg), inclusive
END_FRAME = 30  # last frame number, inclusive
OUTPUT_DIR = "/mnt/ssd/lidar_camera_check"
HEIGHT_RANGE_CM = 5.0


def jsonl(path):
    rows = []
    if Path(path).is_file():
        with open(path, encoding="utf-8") as handle:
            for line in handle:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    pass  # a line the collector is still writing
    return rows


def run_folder():
    folder = Path(FOLDER)
    if not folder.is_dir():
        folder = ml.ROOT / FOLDER[:8] / FOLDER
    if not (folder / "frames" / "frames.jsonl").is_file():
        sys.exit(f"Not a recording folder: {FOLDER}")
    return folder


def draw_legend(image):
    x0, y0, width, height = 20, 70, 360, 22
    ramp = np.linspace(0, 255, width).astype(np.uint8).reshape(1, -1)
    image[y0:y0 + height, x0:x0 + width] = cv2.applyColorMap(ramp, cv2.COLORMAP_TURBO)
    for text, x in ((f"-{HEIGHT_RANGE_CM:g} cm", x0), ("0", x0 + width // 2 - 6),
                    (f"+{HEIGHT_RANGE_CM:g} cm", x0 + width - 70)):
        cv2.putText(image, text, (x, y0 + height + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)


def plane_firings(points, center_raw):
    """(first, centre, last) firing of main_live's plane points, or None when the scan
    does not reach the camera's optical axis. The centre is found as plane_points does."""
    used = ml.plane_points(points, center_raw)
    if used is None:
        return None
    firings = points.firing[used]
    error = np.abs((points.firing_azimuth_raw - center_raw + 18000) % 36000 - 18000)
    return int(firings.min()), int(np.argmin(error)), int(firings.max())


def draw_plane_range(image, points, pixels, visible, firings):
    """Lines through the points of the first, centre and last plane firing, top to bottom."""
    first, center, last = firings
    for firing, color in ((first, (255, 0, 255)), (center, (255, 255, 0)), (last, (255, 0, 255))):
        on = visible & (points.firing == firing)
        line = pixels[on][np.argsort(pixels[on][:, 1])].astype(np.int32)
        if len(line) < 2:
            continue
        cv2.polylines(image, [line.reshape(-1, 1, 2)], False, (0, 0, 0), 7, cv2.LINE_AA)
        cv2.polylines(image, [line.reshape(-1, 1, 2)], False, color, 3, cv2.LINE_AA)
        label = f"{firing - center:+d}" if firing != center else "0"
        u, v = line[0]
        cv2.putText(image, label, (int(u) - 25, int(v) - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(image, label, (int(u) - 25, int(v) - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    color, 2, cv2.LINE_AA)


def overlay(image, row, pcaps, camera, angles, cache):
    """Draw the frame's LiDAR scan onto image; returns a one-line summary."""
    candidates = ml.scan_candidates(pcaps, ml.frame_time(row) - ml.OFFSET_SEC, cache)
    if not candidates:
        return "no LiDAR scan yet (its 10 s file may still be recording)"
    _, sources, start, end, delta = min(candidates, key=lambda c: c[0])
    if abs(delta) > ml.MAX_SCAN_CENTER_DELTA_SEC:
        return f"nearest LiDAR scan is {delta * 1000:+.0f} ms away; main_live skips it"
    points = ml.decode_scan(sources, angles, start - 1e-06, end + 1e-06)
    try:
        heights = ml.flatten(points, camera.axis_azimuth_raw)["road_xyz_m"][:, 2]
        values = np.clip(heights * 100 / HEIGHT_RANGE_CM, -1, 1) * 127.5 + 127.5
        colors = cv2.applyColorMap(values.astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO)[:, 0]
        plane = ""
        draw_legend(image)
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        colors = np.full((len(points), 3), 255, np.uint8)
        plane = f", road plane unavailable ({exc})"
    pixels, visible = camera.project(points.xyz)
    h, w = image.shape[:2]
    visible &= (pixels[:, 0] >= 0) & (pixels[:, 0] < w) & (pixels[:, 1] >= 0) & (pixels[:, 1] < h)
    for (u, v), color in zip(pixels[visible].astype(int), colors[visible]):
        cv2.circle(image, (int(u), int(v)), 2, color.tolist(), -1, cv2.LINE_AA)
    firings = plane_firings(points, camera.axis_azimuth_raw)
    if firings is None:
        plane += ", camera axis not in this scan (plane from all points)"
    else:
        draw_plane_range(image, points, pixels, visible, firings)
        plane += f", plane firings {firings[0] - firings[1]:+d}..{firings[2] - firings[1]:+d}"
    return f"LiDAR scan {delta * 1000:+.1f} ms, {int(visible.sum())} of {len(points)} points in the image{plane}"


def main():
    run = run_folder()
    rows = [r for r in jsonl(run / "frames" / "frames.jsonl")
            if START_FRAME <= int(r["frame_index"]) <= END_FRAME]
    if not rows:
        sys.exit(f"No frames {START_FRAME}..{END_FRAME} in {run}")
    pcaps = [dict(r, path=str(run / "lidar" / r["path"])) for r in jsonl(run / "lidar" / "pcaps.jsonl")]
    camera = ml.LiDAR2Camera(ml.CAMERA_CALIBRATION)
    angles = ml.lidar__load_calibration(ml.LIDAR_CALIBRATION)
    output = Path(OUTPUT_DIR) / run.name
    output.mkdir(parents=True, exist_ok=True)
    cache = {}
    for row in rows:
        index = int(row["frame_index"])
        image = cv2.imread(str(run / row["path"]))
        if image is None:
            print(f"{index:08d}: image missing ({row['path']})")
            continue
        summary = overlay(image, row, pcaps, camera, angles, cache)
        clock = time.strftime("%H:%M:%S", time.localtime(ml.frame_time(row)))
        caption = f"{run.name}  frame {index}  {clock}  {summary}"
        cv2.rectangle(image, (0, 0), (image.shape[1], 50), (0, 0, 0), -1)
        cv2.putText(image, caption, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        path = output / f"{index:08d}.jpg"
        cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        print(f"{index:08d}: {summary} -> {path}")
    print(f"Saved {len(rows)} frames in {output}")


if __name__ == "__main__":
    main()
