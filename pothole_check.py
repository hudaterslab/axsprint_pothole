#!/usr/bin/env python3
"""Drive check of live_pothole.py's pothole LiDAR rule (PotholeScan, line_depths, is_pothole,
see POTHOLE_DEPTH_M there) on a recorded day, and the per-point road depths that
checklidarcamera.py draws.

Every model pothole ends as confirmed, measured but not a pothole, or not judged (no LiDAR
scan for the image, no LiDAR point in the area, or fewer than POTHOLE_MIN_LINES usable lines).

The drive check (run on the NUC with the analysis terminal closed; the first run needs the NPU):
    python3 pothole_check.py
The model areas of every frame of DATE with a pothole are cached in CACHE_DIR, so changing the
rule in live_pothole.py and running again takes a minute. To measure the road estimate itself,
every real pothole area is also moved sideways in its image onto road without model areas
("fake" areas): the hidden points there are the true road, so their depth is the error of the
road estimate, and a pothole found there is false (unless the move landed on damage the model
did not mark). Nothing is uploaded and nothing is written to the recording SSD.
"""
import fcntl
import json
import pickle
from pathlib import Path

import numpy as np

import live_pothole as ml

# =============================================================================
# SETTINGS
# =============================================================================

DATE = "20261002"
FAKE_SHIFTS_PX = (-600, -300, 300, 600)  # sideways moves of a real area onto plain road
CACHE_DIR = Path("/tmp/pothole_check")


# =============================================================================
# ROAD DEPTH OF EVERY POINT (pictures)
# =============================================================================

def _row_median(values):
    """Median of every row, ignoring NaN (each row has at least one number)."""
    ordered = np.sort(values, axis=1)  # NaN last
    count = np.isfinite(values).sum(axis=1)
    rows = np.arange(len(values))
    return 0.5 * (ordered[rows, (count - 1) // 2] + ordered[rows, count // 2])


def _fit_lines(S, H, member, with_spread=True):
    """live_pothole.road_line for many point sets at once (one per row; member marks the set's
    points; S is measured from the row's own point): (road height at the row's point, spread,
    kept). Without with_spread the spread is not worked out (left infinite)."""
    keep = member.copy()
    total = member.sum(axis=1)
    road = np.zeros(len(S))
    spread = np.full(len(S), np.inf)
    rows = np.flatnonzero(total >= 2)  # the rows still being fitted
    for refit in range(3):
        if not len(rows):
            break
        s, h, kept = S[rows], H[rows], keep[rows]
        weight = kept.astype(float)
        for _ in range(5):
            sw, ss, sh = weight.sum(1), (weight * s).sum(1), (weight * h).sum(1)
            sss, ssh = (weight * s * s).sum(1), (weight * s * h).sum(1)
            det = sw * sss - ss * ss
            det = np.where(np.abs(det) > 1e-12, det, np.inf)
            a, b = (sw * ssh - ss * sh) / det, (sss * sh - ss * ssh) / det
            residual = h - (a[:, None] * s + b[:, None])
            weight = kept * np.minimum(1.0, ml.ROAD_HUBER_M / np.maximum(np.abs(residual), 1e-12))
        new = kept & (residual > -ml.ROAD_DROP_M)
        done = (new == kept).all(axis=1) | (refit == 2)
        few = ~done & (new.sum(axis=1) < np.maximum(6, total[rows] // 2))
        road[rows[done | few]] = b[done | few]
        if with_spread and done.any():  # the spread of exactly the kept points (few: stays inf)
            r = np.where(kept[done], residual[done], np.nan)
            spread[rows[done]] = 1.4826 * _row_median(np.abs(r - _row_median(r)[:, None]))
        go = ~(done | few)
        keep[rows[go]] = new[go]
        rows = rows[go]
    return road, spread, keep


def point_depths(geometry, channel, visible, road_ok):
    """The road of live_pothole.line_depths with every visible point as its own area, all points
    at once: for each point, the straight road through its line's road points ROAD_GUARD_M..
    ROAD_SIDE_M before and after it (up to ROAD_MAX_SIDE_M where sparse, sunken points dropped,
    the same side, spread and stability checks). Depth (road minus point, m) per point; NaN where
    the checks leave no usable road. For pictures: inside a depression wider than a few cm the
    road of each point follows the floor, so only line_depths (the whole model area) measures
    its depth."""
    height, flat, azimuth, step = geometry
    depth = np.full(len(height), np.nan)
    lines = []  # (points of the line in order, their distances along it, its road points)
    for c in np.unique(channel[visible]):
        on = np.flatnonzero((channel == c) & visible)
        on = on[np.argsort(azimuth[on], kind="stable")]
        road = road_ok[on]
        if road.sum() >= 2 * ml.ROAD_MIN_POINTS:
            lines.append((on, ml.along_line(flat[on], azimuth[on], step), road))
    if not lines:
        return depth
    on = np.concatenate([line[0] for line in lines])
    s = np.concatenate([line[1] for line in lines])
    sr = np.concatenate([line[1][line[2]] for line in lines])  # road points of all lines
    hr = np.concatenate([height[line[0][line[2]]] for line in lines])
    point_at = np.r_[0, np.cumsum([len(line[0]) for line in lines])]
    road_at = np.r_[0, np.cumsum([int(line[2].sum()) for line in lines])]
    per_side = 16  # road points on a side at most (2-15 cm holds up to about 13)

    def search(value, side):
        """Index into sr of each point's value within the point's own line."""
        out = np.empty(len(value), int)
        for n in range(len(lines)):
            a, b, r0, r1 = point_at[n], point_at[n + 1], road_at[n], road_at[n + 1]
            out[a:b] = r0 + np.searchsorted(sr[r0:r1], value[a:b], side)
        return out

    def fit(side, guard, with_spread=False):
        """fit() of line_depths for every point: (road height there, usable, spread, kept points)."""
        hi, lo = search(s - guard, "right"), search(s - side, "left")
        lo = np.where(hi - lo >= ml.ROAD_MIN_POINTS, lo,
                      np.maximum(search(s - ml.ROAD_MAX_SIDE_M, "left"), hi - ml.ROAD_MIN_POINTS))
        lo = np.maximum(lo, hi - per_side)
        lo_r, hi_r = search(s + guard, "left"), search(s + side, "right")
        hi_r = np.where(hi_r - lo_r >= ml.ROAD_MIN_POINTS, hi_r,
                        np.minimum(search(s + ml.ROAD_MAX_SIDE_M, "right"), lo_r + ml.ROAD_MIN_POINTS))
        hi_r = np.minimum(hi_r, lo_r + per_side)
        k = np.arange(per_side)
        left, right = hi[:, None] - 1 - k, lo_r[:, None] + k
        member = np.concatenate([left >= lo[:, None], right < hi_r[:, None]], axis=1)
        index = np.where(member, np.concatenate([left, right], axis=1), 0)
        S, H = sr[index] - s[:, None], hr[index]

        def sides_ok(mask, need):
            good = np.ones(len(s), bool)
            for part in (slice(0, per_side), slice(per_side, None)):
                m = mask[:, part]
                width = np.where(m, S[:, part], -np.inf).max(axis=1) - np.where(m, S[:, part], np.inf).min(axis=1)
                good &= (m.sum(axis=1) >= need) & (width >= ml.ROAD_MIN_SPAN_M)
            return good

        supported = sides_ok(member, ml.ROAD_MIN_POINTS)
        road, spread, keep = _fit_lines(S, H, member & supported[:, None], with_spread)
        supported &= sides_ok(keep, ml.ROAD_MIN_KEPT)  # dropping sunken points must leave each side supported
        return road, supported, spread, np.sort(np.where(keep, index, -1), axis=1)

    road_h, ok, spread, kept = fit(ml.ROAD_SIDE_M, ml.ROAD_GUARD_M, with_spread=True)
    wide_h, wide_ok, _, _ = fit(ml.ROAD_SIDE_M, ml.ROAD_WIDE_GUARD_M)  # required
    short_h, short_ok, _, short_kept = fit(ml.ROAD_SIDE_M * 2 / 3, ml.ROAD_GUARD_M)  # only when its points differ
    change = np.abs(wide_h - road_h)
    differs = short_ok & (short_kept != kept).any(axis=1)
    change = np.where(differs, np.maximum(change, np.abs(short_h - road_h)), change)
    ok &= (spread <= ml.ROAD_MAX_SPREAD_M) & wide_ok & (change <= ml.ROAD_MAX_CHANGE_M)
    depth[on[ok]] = road_h[ok] - height[on[ok]]
    return depth


# =============================================================================
# DRIVE CHECK
# =============================================================================

def runs():
    day = ml.ROOT / DATE
    return sorted(p for p in day.iterdir() if (p / "frames/frames.jsonl").exists())


def collect():
    """Model areas of every frame of DATE that has a pothole (NPU; the analysis must be stopped)."""
    import senddata as sd

    detector = ml.DXNNDetector(ml.MODEL_PATH, ml.CONFIDENCE_THRESHOLD)
    frames = []
    try:
        for run in runs():
            rows, _ = sd.pcaps_for(run)

            def load(row, run=run):
                path = Path(row["path"])
                image = ml.cv2.imread(str(path if path.is_absolute() else run / path))
                if image is None:
                    raise FileNotFoundError(path)
                return image

            for row, image, detections in detector.iter_detect(rows, load):
                if any(int(d.class_id) == 1 for d in detections):
                    frames.append(frame_record(run.name, row, image.shape[:2], detections))
            print(f"{run.name}: {len(rows)} frames, {sum(f['run'] == run.name for f in frames)} with potholes", flush=True)
    finally:
        detector.dispose()
    return frames


def frame_record(run_name, row, shape, detections):
    """A frame and its model detections, each mask kept as its packed box crop (as cached)."""
    items = []
    for d in detections:
        x0, y0, x1, y1 = (int(v) for v in np.clip(np.round(d.box_xyxy), 0, None))
        crop = np.asarray(d.mask, bool)[y0:y1 + 1, x0:x1 + 1] if d.mask is not None else None
        items.append(dict(class_id=int(d.class_id), confidence=float(d.confidence),
                          box=[float(v) for v in d.box_xyxy], origin=(x0, y0),
                          crop=None if crop is None else (crop.shape, np.packbits(crop))))
    return dict(run=run_name, row=row, shape=shape, detections=items)


def full_mask(item, shape):
    mask = np.zeros(shape, bool)
    if item["crop"] is not None:
        crop_shape, bits = item["crop"]
        crop = np.unpackbits(bits)[:crop_shape[0] * crop_shape[1]].reshape(crop_shape).astype(bool)
        x0, y0 = item["origin"]
        h, w = mask[y0:y0 + crop_shape[0], x0:x0 + crop_shape[1]].shape
        mask[y0:y0 + h, x0:x0 + w] = crop[:h, :w]
    return mask


def shifted(mask, dx):
    out = np.zeros_like(mask)
    if dx > 0:
        out[:, dx:] = mask[:, :-dx]
    else:
        out[:, :dx] = mask[:, -dx:]
    return out


class Scan(ml.PotholeScan):
    """A recorded frame for the check: the scan matched to the image time as the analysis
    matches it, with the frame's model masks (frame: a frame_record)."""

    def __init__(self, frame, run, pcaps, alignment, bounds, camera, angles):
        self.frame, self.camera, self.alignment = frame, camera, alignment
        row, shape = frame["row"], frame["shape"]
        self.target = target = ml.frame_time(row) - ml.OFFSET_SEC
        self.masks = [full_mask(item, shape) for item in frame["detections"]]
        self.detections = [ml.Detection(item["class_id"], item["confidence"], np.asarray(item["box"]),
                                        np.empty(0), mask) for item, mask in zip(frame["detections"], self.masks)]
        candidates = ml.scan_candidates(pcaps, target, bounds)
        self.ok = bool(candidates) and abs(min(candidates, key=lambda c: c[0])[4]) <= ml.MAX_SCAN_CENTER_DELTA_SEC
        if not self.ok:
            return
        _, sources, start, end, delta = min(candidates, key=lambda c: c[0])
        self.scan_delta = delta
        self.place(ml.decode_scan(sources, angles, start - 1e-06, end + 1e-06), shape)


def check(frames):
    import senddata as sd

    camera = ml.LiDAR2Camera(ml.CAMERA_CALIBRATION)
    angles = ml.lidar__load_calibration(ml.LIDAR_CALIBRATION)
    real, fake = [], []
    for run in runs():
        todo = [f for f in frames if f["run"] == run.name]
        if not todo:
            continue
        _, pcaps = sd.pcaps_for(run)
        alignment = ml.RecordingAlignment(ml.GpsTail(run).read() or {})
        bounds = {}
        for frame in todo:
            scan = Scan(frame, run, pcaps, alignment, bounds, camera, angles)
            base = dict(run=run.name, frame=int(frame["row"]["frame_index"]))
            if not scan.ok:  # no LiDAR scan close enough to the image: counted, not judged
                real.extend(dict(base, detection=k, conf=round(item["confidence"], 3), box=[int(round(v)) for v in item["box"]],
                                 scan=False, points=0, accepted=False, lines=[])
                            for k, item in enumerate(frame["detections"]) if item["class_id"] == 1)
                continue
            for k, (d, mask) in enumerate(zip(scan.detections, scan.masks)):
                if d.class_id != 1:
                    continue
                inside = scan.inside(mask)
                lines = scan.lines(inside) if inside.any() else []
                real.append(dict(base, detection=k, conf=round(d.confidence, 3),
                                 box=[int(round(float(v))) for v in d.box_xyxy], scan=True, points=int(inside.sum()),
                                 accepted=ml.is_pothole(lines), lines=lines))
                for dx in FAKE_SHIFTS_PX:  # the same area moved sideways onto plain road
                    moved = shifted(mask, dx)
                    if moved.sum() < 0.9 * mask.sum() or (moved & scan.union).any():
                        continue
                    inside = scan.inside(moved)
                    if inside.sum() < 10:
                        continue
                    lines = scan.lines(inside, scan.road_ok & ~inside, detail=True)
                    used = [line for line in lines if line["measured"]]
                    fake.append(dict(base, detection=k, shift=dx, points=int(inside.sum()), lines=len(lines),
                                     measured=len(used) >= ml.POTHOLE_MIN_LINES, accepted=ml.is_pothole(lines),
                                     one_point=ml.is_pothole(lines, 1),
                                     error_m=[round(v, 5) for line in used for v in line["depth"]]))
        print(f"{run.name}: {len(todo)} frames checked", flush=True)
    return real, fake


def summary(real, fake):
    seen = [r for r in real if r["points"]]
    print(f"\n[모델 포트홀 {len(real)}개, 그중 영역 안에 라이다 점이 있는 것 {len(seen)}개]")
    measured = [sum(line["measured"] for line in r["lines"]) for r in real]
    unjudged = [r for r, m in zip(real, measured) if not r["accepted"] and m < ml.POTHOLE_MIN_LINES]
    confirmed = sum(r["accepted"] for r in real)
    print(f"  포트홀 확인 {confirmed}, 쟀지만 포트홀 아님 {len(real) - confirmed - len(unjudged)}, 판정 불가 {len(unjudged)} "
          f"(라이다 스캔 없음 {sum(not r['scan'] for r in unjudged)}, "
          f"영역 안 라이다 점 없음 {sum(r['scan'] and not r['points'] for r in unjudged)}, "
          f"쓸 수 있는 줄 {ml.POTHOLE_MIN_LINES}개 미만 {sum(bool(r['points']) for r in unjudged)})")
    if seen:
        used = [sum(line["measured"] for line in r["lines"]) for r in seen]
        print(f"  영역당 지나는 줄 중앙값 {np.median([len(r['lines']) for r in seen]):.0f}, 쓴 줄 중앙값 {np.median(used):.0f}, "
              f"쓴 줄 2개 미만 {sum(u < 2 for u in used)}개")
    why = {}
    for r in seen:
        for line in r["lines"]:
            if not line["measured"]:
                why[line["why"]] = why.get(line["why"], 0) + 1
    print(f"  안 쓴 줄 이유: {why}")
    print(f"\n[가짜 영역 {len(fake)}개: 진짜 포트홀 영역을 같은 사진의 옆 도로로 옮긴 것, 숨긴 점 = 진짜 도로]")
    ok = [f for f in fake if f["measured"]]
    print(f"  판정 가능 {len(ok)}/{len(fake)} | 포트홀로 판정 {sum(f['accepted'] for f in fake)}개 "
          f"(판정 가능한 것 중 {sum(f['accepted'] for f in ok)}/{len(ok)}), 점 하나로 판정하면 {sum(f['one_point'] for f in fake)}개")
    err = 1000 * np.array([e for f in fake for e in f["error_m"]])
    if len(err):
        print(f"  잰 점 {len(err)}개에서 기준 도로 오차 (+ = 도로를 높게 잡음, 그만큼 깊게 잼): "
              f"치우침(중앙값) {np.median(err):+.1f} mm, |오차| 95% {np.percentile(np.abs(err), 95):.1f} mm, "
              f"{ml.POTHOLE_DEPTH_M * 100:g} cm 이상 깊게 나온 점 {np.mean(err >= ml.POTHOLE_DEPTH_M * 1000):.1%}")


def main():
    CACHE_DIR.mkdir(exist_ok=True)
    cache = CACHE_DIR / f"areas_{DATE}.pkl"
    if not cache.exists():
        lock = open("/tmp/porthole_live_pothole.lock", "a")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)  # the analysis/senddata must not run (NPU)
        frames = collect()
        temporary = cache.with_suffix(".partial")
        temporary.write_bytes(pickle.dumps(frames))
        temporary.replace(cache)
        lock.close()
    frames = pickle.loads(cache.read_bytes())
    real, fake = check(frames)
    (CACHE_DIR / f"result_{DATE}.json").write_text(json.dumps(dict(real=real, fake=fake)))
    summary(real, fake)


if __name__ == "__main__":
    main()
