#!/usr/bin/env python3
"""New pothole LiDAR check, designed from scratch (2026-10-02), tried on a recorded drive
before anything in main_live.py changes (main_live.py is left as it is).

Rule, for every pothole area the model finds:
1. Height: the LiDAR is tilted, so a point's height is its position along the road normal
   measured on the car (main_live's RoadPlaneSettings.expected_normal), not the sensor z.
2. Road under the pothole, separately for every LiDAR line (channel) crossing the area: a
   straight line through the same line's road points beside the area, from ROAD_GUARD_M to
   ROAD_SIDE_M before its first and after its last point (distances measured on the road),
   outside every model area; where the points are sparse (far to the sides of the image),
   each side reaches further, up to ROAD_MAX_SIDE_M, until it has ROAD_MIN_POINTS points. Points further than ROAD_HUBER_M off the line count less, and
   points more than ROAD_DROP_M below it are dropped and the line drawn again, so the sunken
   rim of a pothole cannot pull the road down. The road bends by 3-7 mm over 1 m but is close
   to straight over 15 cm, and each line has its own fixed offset (+-1-2 mm), hence one
   short straight road per line rather than one plane.
   A line is not used when either side has fewer than ROAD_MIN_POINTS road points within
   ROAD_MAX_SIDE_M or they span less than ROAD_MIN_SPAN_M, when the area is longer than
   ROAD_MAX_AREA_M along it, when
   its road points scatter more than ROAD_MAX_SPREAD_M around the line (or more than half of
   them lie ROAD_DROP_M below it), or when the line under
   the area moves more than ROAD_MAX_CHANGE_M with a side two thirds as long or a guard of
   ROAD_WIDE_GUARD_M.
3. Depth of each area point: road line height minus point height.
4. Pothole: at least MIN_LINES used lines have MIN_RUN neighbouring points (consecutive
   firings, nothing missing between them) DEPTH_M or more deep.
Every model pothole ends as confirmed, measured but not a pothole, or not judged (no LiDAR
scan for the image, no LiDAR point in the area, or fewer than MIN_LINES usable lines).

The drive check (run on the NUC with main_live.py stopped; the first run needs the NPU):
    python3 pothole_check.py
The model areas of every frame with a pothole are cached in CACHE_DIR, so changing the rule
and running again takes a minute. Each pothole is also judged by main_live.py as deployed.
To measure the road estimate itself, every real pothole area is also moved sideways in its
image onto road without model areas ("fake" areas): the hidden points there are the true
road, so their depth is the error of the road estimate, and a pothole found there is false
(unless the move landed on damage the model did not mark). Nothing is uploaded and nothing
is written to the recording SSD.
"""
import fcntl
import json
import pickle
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "/home/hudaters/Desktop/live_detection")
import main_live as ml  # noqa: E402

# =============================================================================
# SETTINGS
# =============================================================================

DATE = "20261002"
DEPTH_M = 0.014  # a point this far below its line's road is deep
MIN_LINES = 2  # deep lines needed for a pothole
MIN_RUN = 3  # neighbouring deep points that make a line deep
ROAD_SIDE_M = 0.15  # road points of a line: up to this far before / after the area
ROAD_GUARD_M = 0.02  # ... but not closer to the area than this
ROAD_MIN_POINTS = 6  # road points needed on each side
ROAD_MAX_SIDE_M = 0.30  # a side reaches up to this far when it has too few points within ROAD_SIDE_M
ROAD_MIN_SPAN_M = 0.04  # the road points of a side span at least this much
ROAD_MIN_KEPT = 4  # road points a side must keep after the sunken ones are dropped
ROAD_MAX_AREA_M = 1.00  # the area's length along a line; longer: road under it unknown
ROAD_HUBER_M = 0.006  # road points further off the line than this count less
ROAD_DROP_M = 0.005  # road points this far below the line are dropped (sunken rim)
ROAD_MAX_SPREAD_M = 0.006  # robust spread of the road points around the line
ROAD_MAX_CHANGE_M = 0.003  # change of the road line under the area in the stability check
ROAD_WIDE_GUARD_M = 0.05  # guard of the stability check
ROAD_NORMAL = np.asarray(ml.RoadPlaneSettings().expected_normal, float)
ROAD_NORMAL = ROAD_NORMAL / np.linalg.norm(ROAD_NORMAL)
FAKE_SHIFTS_PX = (-600, -300, 300, 600)  # sideways moves of a real area onto plain road
CACHE_DIR = Path("/tmp/pothole_check")


# =============================================================================
# THE CHECK
# =============================================================================

def scan_geometry(points, camera, xyz=None):
    """Every point's height along ROAD_NORMAL (as measured), position on the road plane (from
    xyz, the points moved to the image time, so that distances match the image areas) and
    azimuth from the camera axis (raw 0.01 degree), and the azimuth step between firings."""
    height = points.xyz @ ROAD_NORMAL
    moved = points.xyz if xyz is None else xyz
    flat = moved - np.outer(moved @ ROAD_NORMAL, ROAD_NORMAL)
    raw = points.firing_azimuth_raw.astype(np.int64)
    steps = np.diff(raw) % 36000
    step = float(np.median(steps[steps > 0])) if np.any(steps > 0) else 36.0
    azimuth = (raw[points.firing] - camera.axis_azimuth_raw + 18000) % 36000 - 18000
    return height, flat, azimuth, step


def along_line(flat, azimuth, step):
    """Distance (m) along the road from the first of a line's points (in firing order). The
    tilted sensor sweeps the road 2-3 times faster than its azimuth suggests, so the distance
    is measured on the road: the path through the points, each smoothed with its neighbours
    (centred, up to 2 on each side, never across missing firings, so straight paths stay
    straight); across missing firings, the straight distance between the points either side."""
    count = len(flat)
    if count < 2:
        return np.zeros(count)
    neighbour = np.diff(azimuth) <= 1.5 * step
    smooth = flat.astype(float).copy()
    starts = np.r_[0, np.flatnonzero(~neighbour) + 1]
    for a, b in zip(starts, np.r_[starts[1:], count]):
        i = np.arange(b - a)
        half = np.minimum(np.minimum(i, b - a - 1 - i), 2)
        total = np.vstack([np.zeros(3), np.cumsum(flat[a:b], axis=0)])
        smooth[a:b] = (total[i + half + 1] - total[i - half]) / (2 * half + 1)[:, None]
    path = np.where(neighbour, np.linalg.norm(np.diff(smooth, axis=0), axis=1),
                    np.linalg.norm(np.diff(flat, axis=0), axis=1))
    return np.r_[0.0, np.cumsum(path)]


def road_line(s, h):
    """Robust straight line h = a * s + b through road points: Huber weights, then up to two
    refits without the points more than ROAD_DROP_M below it. Returns ((a, b), spread, kept):
    the spread is of exactly the kept points the line was fitted to; it is infinite (road not
    usable) when the sunken points to drop would leave fewer than half of them (or 6)."""
    design = np.column_stack([s, np.ones(len(s))])
    keep = np.ones(len(s), bool)
    for refit in range(3):
        weight = keep.astype(float)
        for _ in range(5):
            root = np.sqrt(weight)
            coef = np.linalg.lstsq(design * root[:, None], h * root, rcond=None)[0]
            residual = h - design @ coef
            weight = keep * np.minimum(1.0, ROAD_HUBER_M / np.maximum(np.abs(residual), 1e-12))
        new = keep & (residual > -ROAD_DROP_M)
        if np.array_equal(new, keep) or refit == 2:
            break
        if new.sum() < max(6, len(s) // 2):
            return coef, float("inf"), keep
        keep = new
    r = residual[keep]
    return coef, 1.4826 * float(np.median(np.abs(r - np.median(r)))), keep


def line_depths(geometry, channel, visible, inside, road_ok, detail=False):
    """Depth below its own road (see the module docstring) of the area points of every LiDAR
    line crossing the area. inside: the area's points; road_ok: points that may serve as road
    (visible in the image, outside every model area). One dict per line; with detail, also
    the point indices of the area and of the road on both sides, and the road line."""
    height, flat, azimuth, step = geometry
    lines = []
    for c in np.unique(channel[inside]):
        on = np.flatnonzero((channel == c) & visible)
        on = on[np.argsort(azimuth[on], kind="stable")]
        area = inside[on]
        s = along_line(flat[on], azimuth[on], step)
        h, road = height[on], road_ok[on] & ~area
        s0, s1 = s[area].min(), s[area].max()
        line = dict(channel=int(c), points=int(area.sum()), length_m=float(s1 - s0), measured=False,
                    why="", deep=0, run=0, max_depth_m=None, median_depth_m=None)
        lines.append(line)
        if detail:
            line.update(area_index=on[area].tolist())
        if s1 - s0 > ROAD_MAX_AREA_M:
            line["why"] = "area too long"
            continue

        def side_points(distance, side, guard):
            """Road points from guard to side away from the area, or the nearest ROAD_MIN_POINTS
            within ROAD_MAX_SIDE_M when there are too few; None when even those are missing."""
            near = np.flatnonzero(road & (distance >= guard) & (distance <= ROAD_MAX_SIDE_M))
            near = near[np.argsort(distance[near], kind="stable")]
            chosen = near[distance[near] <= side]
            if len(chosen) < ROAD_MIN_POINTS:
                chosen = near[:ROAD_MIN_POINTS]
            if len(chosen) < ROAD_MIN_POINTS or np.ptp(s[chosen]) < ROAD_MIN_SPAN_M:
                return None
            return chosen

        def fit(side, guard):
            """(road line, spread, left, right) through the road points of both sides that
            the line kept, or None when a side has too few of them."""
            left, right = side_points(s0 - s, side, guard), side_points(s - s1, side, guard)
            if left is None or right is None:
                return None
            both = np.r_[left, right]
            coef, spread, kept = road_line(s[both], h[both])
            left, right = left[kept[:len(left)]], right[kept[len(left):]]
            for side_kept in (left, right):  # dropping sunken points must leave each side supported
                if len(side_kept) < ROAD_MIN_KEPT or np.ptp(s[side_kept]) < ROAD_MIN_SPAN_M:
                    return None
            return coef, spread, left, right

        main = fit(ROAD_SIDE_M, ROAD_GUARD_M)
        if main is None:
            line["why"] = "road missing on a side"
            continue
        coef, spread, left, right = main
        if detail:
            beside = np.r_[left, right]
            line.update(left_index=on[left].tolist(), right_index=on[right].tolist(),
                        road_coef=[float(v) for v in coef], s_area=s[area].tolist(),
                        road_depth=(coef[0] * s[beside] + coef[1] - h[beside]).tolist())  # of left + right
        if spread > ROAD_MAX_SPREAD_M:
            line["why"] = "road uneven"
            continue
        road_h = coef[0] * s[area] + coef[1]
        wide = fit(ROAD_SIDE_M, ROAD_WIDE_GUARD_M)  # without the 5 cm next to the area: required
        if wide is None:
            line["why"] = "stability unavailable"
            continue
        change = float(np.max(np.abs(wide[0][0] * s[area] + wide[0][1] - road_h)))
        short = fit(ROAD_SIDE_M * 2 / 3, ROAD_GUARD_M)  # shorter sides: only when they differ
        if short is not None and not (np.array_equal(short[2], left) and np.array_equal(short[3], right)):
            change = max(change, float(np.max(np.abs(short[0][0] * s[area] + short[0][1] - road_h))))
        if change > ROAD_MAX_CHANGE_M:
            line["why"] = "road unstable"
            continue
        depth = road_h - h[area]
        deep = depth >= DEPTH_M
        gap = np.diff(azimuth[on][area])
        neighbour = np.r_[False, (gap > 0) & (gap <= 1.5 * step)]  # the next firing, not the same one again
        run = best = 0
        for is_deep, is_next in zip(deep, neighbour):
            run = run + 1 if is_deep and is_next and run else int(is_deep)
            best = max(best, run)
        line.update(measured=True, deep=int(deep.sum()), run=best, max_depth_m=float(depth.max()),
                    median_depth_m=float(np.median(depth)), spread_m=spread, change_m=change)
        if detail:
            line.update(depth=depth.tolist())
    return lines


def is_pothole(lines, min_run=MIN_RUN):
    return sum(1 for line in lines if line["measured"] and line["run"] >= min_run) >= MIN_LINES


def _row_median(values):
    """Median of every row, ignoring NaN (each row has at least one number)."""
    ordered = np.sort(values, axis=1)  # NaN last
    count = np.isfinite(values).sum(axis=1)
    rows = np.arange(len(values))
    return 0.5 * (ordered[rows, (count - 1) // 2] + ordered[rows, count // 2])


def _fit_lines(S, H, member, with_spread=True):
    """road_line for many point sets at once (one per row; member marks the set's points; S is
    measured from the row's own point): (road height at the row's point, spread, kept). Without
    with_spread the spread is not worked out (left infinite)."""
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
            weight = kept * np.minimum(1.0, ROAD_HUBER_M / np.maximum(np.abs(residual), 1e-12))
        new = kept & (residual > -ROAD_DROP_M)
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
    """The road of line_depths with every visible point as its own area, all points at once: for
    each point, the straight road through its line's road points ROAD_GUARD_M..ROAD_SIDE_M before
    and after it (up to ROAD_MAX_SIDE_M where sparse, sunken points dropped, the same side,
    spread and stability checks). Depth (road minus point, m) per point; NaN where the checks
    leave no usable road. For pictures: inside a depression wider than a few cm the road of each
    point follows the floor, so only line_depths (the whole model area) measures its depth."""
    height, flat, azimuth, step = geometry
    depth = np.full(len(height), np.nan)
    lines = []  # (points of the line in order, their distances along it, its road points)
    for c in np.unique(channel[visible]):
        on = np.flatnonzero((channel == c) & visible)
        on = on[np.argsort(azimuth[on], kind="stable")]
        road = road_ok[on]
        if road.sum() >= 2 * ROAD_MIN_POINTS:
            lines.append((on, along_line(flat[on], azimuth[on], step), road))
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
        lo = np.where(hi - lo >= ROAD_MIN_POINTS, lo, np.maximum(search(s - ROAD_MAX_SIDE_M, "left"), hi - ROAD_MIN_POINTS))
        lo = np.maximum(lo, hi - per_side)
        lo_r, hi_r = search(s + guard, "left"), search(s + side, "right")
        hi_r = np.where(hi_r - lo_r >= ROAD_MIN_POINTS, hi_r, np.minimum(search(s + ROAD_MAX_SIDE_M, "right"), lo_r + ROAD_MIN_POINTS))
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
                good &= (m.sum(axis=1) >= need) & (width >= ROAD_MIN_SPAN_M)
            return good

        supported = sides_ok(member, ROAD_MIN_POINTS)
        road, spread, keep = _fit_lines(S, H, member & supported[:, None], with_spread)
        supported &= sides_ok(keep, ROAD_MIN_KEPT)  # dropping sunken points must leave each side supported
        return road, supported, spread, np.sort(np.where(keep, index, -1), axis=1)

    road_h, ok, spread, kept = fit(ROAD_SIDE_M, ROAD_GUARD_M, with_spread=True)
    wide_h, wide_ok, _, _ = fit(ROAD_SIDE_M, ROAD_WIDE_GUARD_M)  # required
    short_h, short_ok, _, short_kept = fit(ROAD_SIDE_M * 2 / 3, ROAD_GUARD_M)  # only when its points differ
    change = np.abs(wide_h - road_h)
    differs = short_ok & (short_kept != kept).any(axis=1)
    change = np.where(differs, np.maximum(change, np.abs(short_h - road_h)), change)
    ok &= (spread <= ROAD_MAX_SPREAD_M) & wide_ok & (change <= ROAD_MAX_CHANGE_M)
    depth[on[ok]] = road_h[ok] - height[on[ok]]
    return depth


# =============================================================================
# DRIVE CHECK
# =============================================================================

def runs():
    day = ml.ROOT / DATE
    return sorted(p for p in day.iterdir() if (p / "frames/frames.jsonl").exists())


def collect():
    """Model areas of every frame of DATE that has a pothole (NPU; main_live.py must be stopped)."""
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


class Scan:
    """The LiDAR side of one recorded frame for the new check: the scan matched to the image time,
    its points moved to that time along the road (car's road normal) and projected into the image.
    `now` (main_live's fitted plane, for the comparison only) is made when first asked for."""

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

    @classmethod
    def from_points(cls, points, masks, camera, alignment, target, shape):
        """The scan of a live frame, already decoded, with the frame's model masks (live_pothole.py)."""
        scan = cls.__new__(cls)
        scan.frame, scan.camera, scan.alignment, scan.target = None, camera, alignment, target
        scan.masks, scan.detections, scan.ok, scan.scan_delta = masks, [], True, None
        scan.place(points, shape)
        return scan

    def place(self, points, shape):
        """Move the decoded scan to the image time along the road (heights stay as measured), work
        out its geometry and project it into the image."""
        self.points = points
        road, rotation, _ = ml.lidar__road_aligned_xyz(points.xyz, ROAD_NORMAL, 0.0)
        raw = dict(sensor_xyz_m=points.xyz, road_xyz_m=road, road_rotation=rotation,
                   plane_normal=ROAD_NORMAL, point_timestamps=points.timestamp)
        try:
            moved = self.alignment.compensate(raw, self.camera, self.target)
            self.moved = True
        except ValueError:
            moved, self.moved = raw, False
        self.xyz, self.road_xyz = moved["sensor_xyz_m"], moved["road_xyz_m"]
        self.geometry = scan_geometry(points, self.camera, self.xyz)
        self.pixels, visible = self.camera.project(self.xyz)
        h, w = shape
        self.visible = visible & (self.pixels[:, 0] >= 0) & (self.pixels[:, 0] < w) & (self.pixels[:, 1] >= 0) & (self.pixels[:, 1] < h)
        self.uv = np.where(self.visible[:, None], self.pixels, 0).astype(int)
        self.union = np.logical_or.reduce(self.masks) if self.masks else np.zeros(shape, bool)
        self.road_ok = self.visible & ~self.union[self.uv[:, 1], self.uv[:, 0]]

    @property
    def now(self):
        """main_live.py's own fitted road plane and motion compensation (comparison only)."""
        if not hasattr(self, "_now"):
            try:
                plane = ml.flatten(self.points, self.camera.axis_azimuth_raw)
                try:
                    self._now = self.alignment.compensate(plane, self.camera, self.target)
                except ValueError:
                    self._now = plane
            except (ValueError, RuntimeError, np.linalg.LinAlgError):
                self._now = None
        return self._now

    def inside(self, mask):
        return self.visible & mask[self.uv[:, 1], self.uv[:, 0]]

    def now_depths(self, inside, exclusion):
        """{point index: depth} of the area points against main_live's ring plane (its own
        projection, road coordinates and paint zone), or {} without a plane or ring."""
        if self.now is None:
            return {}
        if not hasattr(self, "now_pixels"):
            pixels, visible = self.camera.project(self.now["sensor_xyz_m"])
            h, w = self.frame["shape"]
            self.now_valid = (visible & np.isfinite(pixels).all(axis=1) & (pixels[:, 0] >= 0) & (pixels[:, 0] < w)
                              & (pixels[:, 1] >= 0) & (pixels[:, 1] < h))
            self.now_pixels, self.now_paint = pixels, ml.paint_zone(self.points)
        indices = np.flatnonzero(inside & self.now_valid)
        if not len(indices):
            return {}
        measured, _ = ml.measure_mask_local_depth(self.now["road_xyz_m"], self.now_pixels, np.flatnonzero(self.now_valid),
                                                  indices, exclusion, ml.POTHOLE_DEPTH_M, self.now_paint)
        return {} if measured is None else dict(zip(indices.tolist(), (-measured[:, 2]).tolist()))

    def lines(self, inside, road_ok=None, detail=False):
        return line_depths(self.geometry, self.points.channel, self.visible, inside,
                           self.road_ok if road_ok is None else road_ok, detail)


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
                                 points=0, now="no_scan", now_accepted=False, now_deep_points=None, new_accepted=False, lines=[])
                            for k, item in enumerate(frame["detections"]) if item["class_id"] == 1)
                continue
            _, _, audit = ml.validate_objects(scan.detections, scan.now, camera, ml.POTHOLE_DEPTH_M)
            for k, (d, mask, a) in enumerate(zip(scan.detections, scan.masks, audit)):
                if d.class_id != 1:
                    continue
                inside = scan.inside(mask)
                lines = scan.lines(inside) if inside.any() else []
                local = json.loads(a.get("local_evidence_json") or "{}")
                real.append(dict(base, detection=k, conf=round(d.confidence, 3),
                                 box=[int(round(float(v))) for v in d.box_xyxy], points=int(inside.sum()),
                                 now=a["reason"], now_accepted=bool(a["accepted"]),
                                 now_deep_points=local.get("points_at_threshold"),
                                 new_accepted=is_pothole(lines), lines=lines))
                for dx in FAKE_SHIFTS_PX:  # the same area moved sideways onto plain road
                    moved = shifted(mask, dx)
                    if moved.sum() < 0.9 * mask.sum() or (moved & scan.union).any():
                        continue
                    inside = scan.inside(moved)
                    if inside.sum() < 10:
                        continue
                    lines = scan.lines(inside, scan.road_ok & ~inside, detail=True)
                    extra = ml.Detection(1, d.confidence, np.asarray(d.box_xyxy) + [dx, 0, dx, 0], np.empty(0), moved)
                    _, _, audit2 = ml.validate_objects(scan.detections + [extra], scan.now, camera, ml.POTHOLE_DEPTH_M)
                    used = [line for line in lines if line["measured"]]
                    new_depth = {i: dd for line in used for i, dd in zip(line["area_index"], line["depth"])}
                    now_depth = scan.now_depths(inside, scan.union | moved)
                    common = sorted(new_depth.keys() & now_depth.keys())
                    fake.append(dict(base, detection=k, shift=dx, points=int(inside.sum()), lines=len(lines),
                                     new_measured=len(used) >= MIN_LINES, new_accepted=is_pothole(lines),
                                     new_one_point=is_pothole(lines, 1), now_measured=bool(now_depth),
                                     now_accepted=bool(audit2[-1]["accepted"]),
                                     new_error_m=[round(new_depth[i], 5) for i in common],
                                     now_error_m=[round(now_depth[i], 5) for i in common]))
        print(f"{run.name}: {len(todo)} frames checked", flush=True)
    return real, fake


def summary(real, fake):
    seen = [r for r in real if r["points"]]
    print(f"\n[모델 포트홀 {len(real)}개, 그중 영역 안에 라이다 점이 있는 것 {len(seen)}개]")
    print(f"  지금 main_live: 포트홀 {sum(r['now_accepted'] for r in real)}개 | 새 방식: {sum(r['new_accepted'] for r in real)}개")
    measured = [sum(line["measured"] for line in r["lines"]) for r in real]
    unjudged = [r for r, m in zip(real, measured) if not r["new_accepted"] and m < MIN_LINES]
    confirmed = sum(r["new_accepted"] for r in real)
    print(f"  새 방식 결과: 포트홀 확인 {confirmed}, 쟀지만 포트홀 아님 {len(real) - confirmed - len(unjudged)}, 판정 불가 {len(unjudged)} "
          f"(라이다 스캔 없음 {sum(r['now'] == 'no_scan' for r in unjudged)}, "
          f"영역 안 라이다 점 없음 {sum(r['now'] != 'no_scan' and not r['points'] for r in unjudged)}, "
          f"쓸 수 있는 줄 {MIN_LINES}개 미만 {sum(bool(r['points']) for r in unjudged)})")
    print(f"  둘 다 포트홀 {sum(r['now_accepted'] and r['new_accepted'] for r in real)}, 지금만 {sum(r['now_accepted'] and not r['new_accepted'] for r in real)}, "
          f"새 방식만 {sum(r['new_accepted'] and not r['now_accepted'] for r in real)}")
    used = [sum(line["measured"] for line in r["lines"]) for r in seen]
    print(f"  영역당 지나는 줄 중앙값 {np.median([len(r['lines']) for r in seen]):.0f}, 쓴 줄 중앙값 {np.median(used):.0f}, 쓴 줄 2개 미만 {sum(u < 2 for u in used)}개")
    why = {}
    for r in seen:
        for line in r["lines"]:
            if not line["measured"]:
                why[line["why"]] = why.get(line["why"], 0) + 1
    print(f"  안 쓴 줄 이유: {why}")
    print(f"\n[가짜 영역 {len(fake)}개: 진짜 포트홀 영역을 같은 사진의 옆 도로로 옮긴 것, 숨긴 점 = 진짜 도로]")
    for name, measured, accepted in (("지금 main_live", "now_measured", "now_accepted"), ("새 방식", "new_measured", "new_accepted")):
        ok = [f for f in fake if f[measured]]
        print(f"  {name}: 판정 가능 {len(ok)}/{len(fake)} | 포트홀로 판정 {sum(f[accepted] for f in fake)}개 "
              f"(판정 가능한 것 중 {sum(f[accepted] for f in ok)}/{len(ok)})")
    new_err = 1000 * np.array([e for f in fake for e in f["new_error_m"]])
    now_err = 1000 * np.array([e for f in fake for e in f["now_error_m"]])
    print(f"  같은 점 {len(new_err)}개에서 기준 도로 오차 (+ = 도로를 높게 잡음, 그만큼 깊게 잼):")
    for name, err in (("지금 main_live", now_err), ("새 방식", new_err)):
        print(f"    {name}: 치우침(중앙값) {np.median(err):+.1f} mm, |오차| 95% {np.percentile(np.abs(err), 95):.1f} mm, "
              f"1 cm 이상 깊게 나온 점 {np.mean(err >= 10):.1%}")


def main():
    CACHE_DIR.mkdir(exist_ok=True)
    cache = CACHE_DIR / f"areas_{DATE}.pkl"
    if not cache.exists():
        lock = open("/tmp/porthole_main_live.lock", "a")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)  # main_live/senddata must not run (NPU)
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
