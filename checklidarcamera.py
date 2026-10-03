#!/usr/bin/env python3
"""Draw the LiDAR points of recorded frames onto their camera images, coloured by height above
the road as pothole_check.py measures it, with the model's areas and pothole_check's verdicts.

Set FOLDER (a run folder, a run name, or a date such as "20261002" for every run of that day),
START_FRAME and END_FRAME below, then run:  python3 checklidarcamera.py
The model runs on the NPU, so main_live.py must be stopped. Each frame is saved as
OUTPUT_DIR/<run>/<frame 8 digits>.jpg; the model potholes of a run and their verdicts are listed
in OUTPUT_DIR/<run>/potholes.csv.

Everything comes from pothole_check.py and the main_live.py helpers it uses: the LiDAR scan
matched to the image time and moved to it along the road, heights along the car's road normal,
and per LiDAR line a straight road through the same line's road points 2-15 cm on both sides.
- Points in a model area (pothole: red outline, crack: cyan): the road beside the whole area,
  exactly as the pothole check measures it. Each pothole is labelled confirmed / measured, not a
  pothole / not judged, and its deep points (DEPTH_M or more on deep lines) are ringed yellow.
  The road points that check used (2-15 cm beside the area on each measured line) are ringed
  white and coloured against that same road; on their own they would be grey, as one of their
  sides is the area, which never counts as road.
- Every other point: the same road with the point itself as the area (2-15 cm on both sides of
  it). This shows ridges, edges and dips a few cm wide; inside a wider depression the model did
  not mark, the road of each point follows the floor (only a model area's road measures that).
Colour: blue at -HEIGHT_RANGE_CM or lower, green at 0 (the road beside), red at +HEIGHT_RANGE_CM;
small grey dots where the road beside is missing, uneven or unsteady (pothole_check's checks).
"""

import csv
import fcntl
import multiprocessing
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

import pothole_check as pc
import senddata as sd

ml = pc.ml

# =============================================================================
# SETTINGS
# =============================================================================

FOLDER = "20261002"  # a run folder, a run name such as "20261002_1010", or a date for all its runs
START_FRAME = 0  # first frame number (frames/00000000.jpg), inclusive
END_FRAME = None  # last frame number, inclusive; None: to the end of each run
OUTPUT_DIR = "/mnt/ssd/lidar_camera_check"
HEIGHT_RANGE_CM = 3.0
WORKERS = 3  # drawing processes (the model itself runs on the NPU)


def runs():
    folder = Path(FOLDER)
    if not folder.is_dir():
        folder = ml.ROOT / FOLDER[:8] / FOLDER if len(FOLDER) > 8 else ml.ROOT / FOLDER
    if (folder / "frames" / "frames.jsonl").is_file():
        return [folder]
    found = sorted(p for p in folder.iterdir() if (p / "frames" / "frames.jsonl").is_file()) if folder.is_dir() else []
    if not found:
        sys.exit(f"Not a recording folder or date: {FOLDER}")
    return found


def draw_legend(image):
    x0, y0, width, height = 20, 70, 360, 22
    ramp = np.linspace(0, 255, width).astype(np.uint8).reshape(1, -1)
    image[y0:y0 + height, x0:x0 + width] = cv2.applyColorMap(ramp, cv2.COLORMAP_TURBO)
    for text, x in ((f"-{HEIGHT_RANGE_CM:g} cm", x0), ("0", x0 + width // 2 - 6),
                    (f"+{HEIGHT_RANGE_CM:g} cm", x0 + width - 70)):
        cv2.putText(image, text, (x, y0 + height + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "height above the road beside it (pothole_check); grey: no usable road", (x0, y0 + height + 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "white ring: road points a model area's check used", (x0, y0 + height + 78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)


def put_label(image, text, x, y, color):
    width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 5)[0][0]
    x = max(10, min(int(x), image.shape[1] - width - 10))  # whole label inside the image
    for thickness, c in ((5, (0, 0, 0)), (2, color)):
        cv2.putText(image, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, thickness, cv2.LINE_AA)


CAMERA = ANGLES = None
STATE = {}


def init_worker():
    global CAMERA, ANGLES
    cv2.setNumThreads(1)
    CAMERA = ml.LiDAR2Camera(ml.CAMERA_CALIBRATION)
    ANGLES = ml.lidar__load_calibration(ml.LIDAR_CALIBRATION)


def run_state(run):
    if run not in STATE:
        STATE.clear()
        _, pcaps = sd.pcaps_for(run)
        STATE[run] = (pcaps, ml.RecordingAlignment(ml.GpsTail(run).read() or {}), {})
    return STATE[run]


def draw(run, frame, output):
    """Draw one frame (pothole_check frame record) and save it; returns its pothole rows."""
    row = frame["row"]
    index = int(row["frame_index"])
    image = cv2.imread(str(run / row["path"]))
    pcaps, alignment, bounds = run_state(run)
    scan = pc.Scan(frame, run, pcaps, alignment, bounds, CAMERA, ANGLES)
    clock = time.strftime("%H:%M:%S", time.localtime(ml.frame_time(row)))
    found = []
    if not scan.ok:
        summary = "no LiDAR scan within 55 ms of the image (main_live skips it)"
        for k, item in enumerate(frame["detections"]):
            if item["class_id"] == 1:
                found.append(dict(frame=index, time=clock, detection=k, confidence=round(item["confidence"], 2),
                                  verdict="not judged: no LiDAR scan", usable_lines=0, deep_lines=0, deepest_cm=""))
    else:
        depth = pc.point_depths(scan.geometry, scan.points.channel, scan.visible, scan.road_ok)
        deep, labels, beside = [], [], []
        for k, (d, mask) in enumerate(zip(scan.detections, scan.masks)):
            inside = scan.inside(mask)
            lines = scan.lines(inside, detail=True) if inside.any() else []
            for line in lines:  # the area's own road (the pothole check)
                depth[line["area_index"]] = line["depth"] if line["measured"] else np.nan
                if line["measured"]:  # the road points it used, against that road
                    depth[line["left_index"] + line["right_index"]] = line["road_depth"]
                    beside += line["left_index"] + line["right_index"]
                if d.class_id == 1 and line["measured"] and line["run"] >= pc.MIN_RUN:
                    deep += [i for i, v in zip(line["area_index"], line["depth"]) if v >= pc.DEPTH_M]
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, (0, 0, 255) if d.class_id == 1 else (255, 255, 0), 2)
            if d.class_id != 1:
                continue
            used = [line for line in lines if line["measured"]]
            deep_lines = sum(line["run"] >= pc.MIN_RUN for line in used)
            deepest = max((line["max_depth_m"] for line in used), default=None)
            if pc.is_pothole(lines):
                verdict = "pothole confirmed"
            elif len(used) >= pc.MIN_LINES:
                verdict = "measured, not a pothole"
            else:
                verdict = "not judged: no LiDAR points" if not inside.any() else f"not judged: {len(used)} usable line(s)"
            found.append(dict(frame=index, time=clock, detection=k, confidence=round(d.confidence, 2), verdict=verdict,
                              usable_lines=len(used), deep_lines=deep_lines,
                              deepest_cm="" if deepest is None else round(100 * deepest, 1)))
            text = f"{verdict} ({deep_lines}/{len(used)} deep lines" + ("" if deepest is None else f", deepest {100 * deepest:.1f} cm") + ")"
            labels.append((f"pothole {d.confidence:.2f}: {text}", d.box_xyxy))
        height = -depth  # above the road beside
        value = np.clip(np.nan_to_num(height) * 100 / HEIGHT_RANGE_CM, -1, 1) * 127.5 + 127.5
        colors = cv2.applyColorMap(value.astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO)[:, 0]
        for i in np.flatnonzero(scan.visible):
            u, v = int(scan.uv[i, 0]), int(scan.uv[i, 1])
            if np.isnan(depth[i]):
                cv2.circle(image, (u, v), 1, (150, 150, 150), -1)
            else:
                cv2.circle(image, (u, v), 2, colors[i].tolist(), -1, cv2.LINE_AA)
        for i in beside:
            cv2.circle(image, (int(scan.uv[i, 0]), int(scan.uv[i, 1])), 4, (255, 255, 255), 1, cv2.LINE_AA)
        for i in deep:
            cv2.circle(image, (int(scan.uv[i, 0]), int(scan.uv[i, 1])), 5, (0, 255, 255), 1, cv2.LINE_AA)
        for text, box in labels:
            put_label(image, text, max(10, box[0]), max(130, box[1] - 10), (0, 0, 255))
        draw_legend(image)
        speed = alignment.speed_at(scan.target)
        summary = (f"LiDAR scan {scan.scan_delta * 1000:+.0f} ms, {int(scan.visible.sum())} points in the image, "
                   + (f"moved for {speed:.1f} m/s" if scan.moved and speed is not None else "not moved (no GPS speed)"))
    cv2.rectangle(image, (0, 0), (image.shape[1], 50), (0, 0, 0), -1)
    cv2.putText(image, f"{run.name}  frame {index}  {clock}  {summary}", (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(Path(output) / f"{index:08d}.jpg"), image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return found


def main():
    lock = open("/tmp/porthole_main_live.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("main_live.py or senddata.py is running; the model needs the NPU")
    pool = multiprocessing.Pool(WORKERS, initializer=init_worker)  # before the NPU is opened
    detector = ml.DXNNDetector(ml.MODEL_PATH, ml.CONFIDENCE_THRESHOLD)
    try:
        for run in runs():
            rows, _ = sd.pcaps_for(run)
            rows = [r for r in rows if int(r["frame_index"]) >= START_FRAME
                    and (END_FRAME is None or int(r["frame_index"]) <= END_FRAME) and (run / r["path"]).is_file()]
            output = Path(OUTPUT_DIR) / run.name
            output.mkdir(parents=True, exist_ok=True)

            def load(row, run=run):
                image = cv2.imread(str(run / row["path"]))
                if image is None:
                    raise FileNotFoundError(row["path"])
                return image

            pending, table, done, started = deque(), [], 0, time.time()
            for row, image, detections in detector.iter_detect(rows, load):
                frame = pc.frame_record(run.name, row, image.shape[:2], detections)
                pending.append(pool.apply_async(draw, (run, frame, str(output))))
                while len(pending) > 4 * WORKERS or (pending and pending[0].ready()):
                    table += pending.popleft().get()
                    done += 1
                    if done % 500 == 0:
                        print(f"{run.name}: {done}/{len(rows)} frames, {done / (time.time() - started):.1f} frames/s", flush=True)
            while pending:
                table += pending.popleft().get()
            with open(output / "potholes.csv", "w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, ["frame", "time", "detection", "confidence", "verdict",
                                                 "usable_lines", "deep_lines", "deepest_cm"])
                writer.writeheader()
                writer.writerows(table)
            verdicts = {}
            for item in table:
                key = item["verdict"].split(":")[0]
                verdicts[key] = verdicts.get(key, 0) + 1
            print(f"{run.name}: {len(rows)} frames drawn in {output} ({(time.time() - started) / 60:.0f} min); "
                  f"model potholes {len(table)}: {verdicts}", flush=True)
    finally:
        detector.dispose()
        pool.close()
        pool.join()
        lock.close()


if __name__ == "__main__":
    main()
