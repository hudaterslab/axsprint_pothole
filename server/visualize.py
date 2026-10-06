#!/usr/bin/env python3
"""Server tool (run on the upload server, not on the terminal): python3 visualize.py

Draw the terminal's (live_pothole.py) report frames the way codecode/replay_certifcate.py
draws its replayed/lidar_images: the received JPG with the detection areas and the
LiDAR points of the frame's PCAP, colored by height from the fitted road plane.

live_pothole.py uploads one frame_<KST date>_<time>_<ms>.jpg/.json/.pcap (until
2026-10-02: frame_<capture ns>_<frame 8 digits>)
per reported frame to model_detections/porthole_live_analysis*/<date>/<run>/
certifcate/. Images go to OUTPUT_ROOT/<upload folder>/<date>/<run>/ with the
same name; frames already drawn are skipped, so running it again only draws
new uploads. Needs NumPy and Pillow (no OpenCV).

Drawing follows replay_certifcate.py (render_frame, draw_height_legend,
height_rgb). XT32 decoding, the camera model, the road plane with its checks and
the motion compensation are those of live_pothole.py, so the points land where the
analysis saw them: when the JSON carries gps.speed_mps, the points are moved to
the moment the image was taken (capture time minus OFFSET_SEC) as live_pothole.py
moved them. When the road plane fails live_pothole.py's checks, the frame is drawn
without LiDAR and the reason is printed. Each pothole's label shows the terminal's
pothole check (line_depths): how many LiDAR lines across it are deep, and its deepest point.
The top right shows the frame's GPS from the JSON (latitude, longitude, speed).
"""

from __future__ import annotations

import csv
import json
import math
import os
import struct
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from datetime import datetime, timedelta, timezone
from functools import lru_cache
from pathlib import Path

import numpy as np

# =============================================================================
# USER SETTINGS
# =============================================================================

# The data folder: on the server this script is <data folder>/code_server/visualize.py,
# so it is the folder above the script's; elsewhere (e.g. a Windows PC through the
# network share) set the PORTHOLE_ROOT environment variable to it.
PORTHOLE_ROOT = Path(os.environ.get("PORTHOLE_ROOT") or Path(__file__).resolve().parent.parent)
UPLOAD_ROOT = PORTHOLE_ROOT / "model_detections"  # the terminal's PORTHOLE_UPLOAD_DIR
UPLOAD_PREFIX = "porthole_live_analysis"  # the terminal's upload folders, including _test and moved-aside ones
OUTPUT_ROOT = PORTHOLE_ROOT / "model_detections_lidar_images"
LIDAR_CALIBRATION = PORTHOLE_ROOT / "XT32_Angle_Correction_File.csv"
WORKERS = min(24, os.cpu_count() or 1)
OVERWRITE = False  # True redraws frames that already have an image
ALPHA = 0.25  # detection area fill opacity, as replay_certifcate.py
HEIGHT_RANGE_CM = 5.0  # color range ±cm, as replay_certifcate.py

# Camera model of the terminal, identical to its camera_calib_best_effort.json (read
# by live_pothole.py): the camera sits 9 cm directly below the LiDAR with its axes aligned
# (optical axis = LiDAR -Y). Update both together if the camera or LiDAR mount changes.
CAMERA = dict(
    fx=909.7,
    fy=909.7,
    cx=984.3,
    cy=535.2,
    k1=-0.0308,
    k2=-0.0088,
    R_sensor_to_cam=[
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0],
    ],
    t_cam=[0.0, 0.0, -0.09],
)

# Colors and height scale of replay_certifcate.py (RGB).
KINDS = {0: "crack", 1: "pothole"}
COLORS = {"pothole": (244, 82, 68), "crack": (0, 178, 160)}
HEIGHT_STOPS = (
    (-1.0, (38, 83, 219)),
    (-0.5, (37, 184, 220)),
    (0.0, (42, 210, 105)),
    (0.5, (255, 208, 57)),
    (1.0, (224, 55, 45)),
)
HEIGHT_BIN_CM = 0.1

# =============================================================================
# XT32 decoding (same as live_pothole.py)
# =============================================================================

XT32_CHANNELS = 32
XT32_PAYLOAD_LEN = 1080
XT32_BODY_OFFSET = 12
XT32_BLOCK_LEN = 130
XT32_BLOCKS = 8
XT32_TAIL_OFFSET = 1052
XT32_FIRETIME_US = [1.512 * i + 6.0 for i in range(32)]  # Hesai XT firetime correction file: 6.0 .. 52.872 us
XT_COORD_H_M = 0.0315
XT_COORD_B_M = 0.013
RETURN_SELECTION = "last"
PCAP_FORMATS = {  # byte order and record time divisor (microsecond or nanosecond PCAP)
    b"\xd4\xc3\xb2\xa1": ("<", 1000000),
    b"\xa1\xb2\xc3\xd4": (">", 1000000),
    b"M<\xb2\xa1": ("<", 1000000000),
    b"\xa1\xb2<M": (">", 1000000000),
}


def load_xt32_calibration(path):
    """(elevation_deg, azimuth_offset_deg) for channels 1..32."""
    elevation, azimuth = [None] * XT32_CHANNELS, [None] * XT32_CHANNELS
    with Path(path).open("r", encoding="utf-8-sig", newline="") as handle:
        for row in list(csv.reader(handle))[1:]:
            try:
                channel, el, az = int(row[0]), float(row[1]), float(row[2])
            except (IndexError, ValueError):
                continue
            if 1 <= channel <= XT32_CHANNELS:
                elevation[channel - 1], azimuth[channel - 1] = el, az
    if None in elevation or None in azimuth:
        raise ValueError(f"XT32 calibration for channels 1..32 not found in {path}")
    return elevation, azimuth


def xt32_payloads(pcap_path):
    """(record time, UDP payload) of every XT32 data packet in a classic Ethernet PCAP."""
    with Path(pcap_path).open("rb") as handle:
        header = handle.read(24)
        if len(header) != 24 or header[:4] not in PCAP_FORMATS:
            raise ValueError("not a classic PCAP")
        endian, divisor = PCAP_FORMATS[header[:4]]
        while True:
            record = handle.read(16)
            if len(record) < 16:
                return
            seconds, fraction, size, _ = struct.unpack(endian + "IIII", record)
            frame = handle.read(size)
            if len(frame) < 42 or frame[12:14] != b"\x08\x00" or frame[23] != 17:
                continue
            udp = 14 + (frame[14] & 15) * 4
            length = struct.unpack("!H", frame[udp + 4 : udp + 6])[0]
            payload = frame[udp + 8 : udp + length]
            if len(payload) == XT32_PAYLOAD_LEN and payload[:4] == b"\xee\xff\x06\x01":
                yield seconds + fraction / divisor, payload


def return_label(mode, block):
    pair = block % 2
    return {
        59: "last" if pair == 0 else "first",
        57: "last" if pair == 0 else "strongest",
        60: "strongest" if pair == 0 else "first",
        56: "last",
        55: "strongest",
    }.get(mode, f"0x{mode:02X}")


@lru_cache(maxsize=1)
def azimuth_table():
    radians = [math.radians(k / 100.0) for k in range(36000)]
    return np.array([math.sin(a) for a in radians]), np.array([math.cos(a) for a in radians])


def decode_xt32(pcap_path, calibration):
    """Sensor XYZ (m) of every kept return, as live_pothole.py decodes; the mask of the
    points its road plane is fitted to (None when the scan misses the camera's optical
    axis): the firing nearest PLANE_CENTER_AZIMUTH_RAW and PLANE_SIDE_FIRINGS firings
    on each side, every channel (plane_points); each point's PCAP record time
    (point_timestamps); and for the pothole check each point's channel, firing index and
    the raw azimuth of every firing (ScanPoints)."""
    records = list(xt32_payloads(pcap_path))
    if not records:
        raise ValueError("PCAP has no XT32 packets")
    packet_times = np.array([t for t, _ in records], float)
    payloads = [p for _, p in records]
    count = len(payloads)
    data = np.frombuffer(b"".join(payloads), np.uint8).reshape(count, XT32_PAYLOAD_LEN)
    data = data.astype(np.int64)
    tail = data[:, XT32_TAIL_OFFSET:]
    mode = tail[:, 10]
    rpm = tail[:, 11] | tail[:, 12] << 8
    blocks = data[:, XT32_BODY_OFFSET:XT32_TAIL_OFFSET].reshape(count, XT32_BLOCKS, XT32_BLOCK_LEN)
    raw_az100 = blocks[:, :, 0] | blocks[:, :, 1] << 8
    lasers = blocks[:, :, 2:].reshape(count, XT32_BLOCKS, XT32_CHANNELS, 4)
    distance = (lasers[..., 0] | lasers[..., 1] << 8) * data[:, 9, None, None] / 1000.0
    selected = {m: [return_label(int(m), b) == RETURN_SELECTION for b in range(XT32_BLOCKS)]
                for m in set(mode.tolist())}
    block_ok = np.array([selected[m] for m in mode.tolist()], bool)
    keep = block_ok[:, :, None] & (distance > 0.1) & (distance <= 200.0)

    elevation, offset = calibration
    firetime = rpm[:, None] * np.asarray(XT32_FIRETIME_US) * 6e-06
    az100 = (np.asarray(offset) * 100.0 + raw_az100[:, :, None] + (firetime * 100.0)[:, None, :])
    az100 = az100.astype(np.int64) % 36000
    sin_table, cos_table = azimuth_table()
    sin_az, cos_az = sin_table[az100], cos_table[az100]
    geometry = []
    for c in range(XT32_CHANNELS):
        el, off = math.radians(elevation[c]), math.radians(abs(offset[c]))
        cos_el = math.cos(el)
        geometry.append((cos_el, math.sin(el),
                         XT_COORD_H_M * math.cos(off) * cos_el - XT_COORD_B_M * math.sin(off) * cos_el))
    cos_el, sin_el, correction = np.array(geometry).T
    corrected = distance - correction
    xy = corrected * cos_el
    x = xy * sin_az - XT_COORD_B_M * cos_az + XT_COORD_H_M * sin_az
    y = xy * cos_az + XT_COORD_B_M * sin_az + XT_COORD_H_M * cos_az
    z = corrected * sin_el
    xyz = np.column_stack([x[keep], y[keep], z[keep]])
    times = np.broadcast_to(packet_times[:, None, None], keep.shape)[keep]
    # A firing is one kept-return block (dual return: one of each block pair).
    firing = np.cumsum(block_ok.ravel()).reshape(block_ok.shape) - 1
    point_firing = np.broadcast_to(firing[:, :, None], keep.shape)[keep]
    channel = np.broadcast_to(np.arange(1, XT32_CHANNELS + 1)[None, None, :], keep.shape)[keep]
    firing_azimuth = raw_az100[block_ok]
    firings = (channel, point_firing, firing_azimuth)
    if not len(firing_azimuth):
        return xyz, None, times, firings
    error = np.abs((firing_azimuth - PLANE_CENTER_AZIMUTH_RAW + 18000) % 36000 - 18000)
    center = int(np.argmin(error))
    if error[center] > 100:
        return xyz, None, times, firings
    return xyz, np.abs(point_firing - center) <= PLANE_SIDE_FIRINGS, times, firings


def project(xyz):
    """Pixels of sensor XYZ through the fisheye camera model of live_pothole.py."""
    R = np.asarray(CAMERA["R_sensor_to_cam"], float)
    camera = (xyz - np.asarray(CAMERA["t_cam"], float)) @ R.T
    depth = camera[:, 2]
    valid = depth > 1e-06
    safe = np.where(valid, depth, 1.0)
    a, b = camera[:, 0] / safe, camera[:, 1] / safe
    radius = np.hypot(a, b)
    theta = np.arctan(radius)
    distorted = theta * (1.0 + CAMERA["k1"] * theta**2 + CAMERA["k2"] * theta**4)
    scale = np.where(radius > 1e-09, distorted / radius, 1.0)
    pixels = np.column_stack([CAMERA["fx"] * a * scale + CAMERA["cx"],
                              CAMERA["fy"] * b * scale + CAMERA["cy"]])
    return pixels, valid & np.isfinite(pixels).all(axis=1)


# =============================================================================
# Road plane and motion compensation: live_pothole.py's fit_spatial_road_plane(),
# flatten() checks and RecordingAlignment.compensate()
# =============================================================================

EXPECTED_NORMAL = (-0.012, 0.8, 0.6)  # live_pothole's RoadPlaneSettings: road normal measured on the car
# Plane points as live_pothole.py: the firing nearest the camera's optical axis and this
# many firings on each side (201 firings, all 32 channels), like lcz_to_pcap.py.
PLANE_SIDE_FIRINGS, CELL_SIZE_M = 100, 0.35
# The optical axis of CAMERA as a raw LiDAR azimuth in 0.01 degree (180 degrees).
PLANE_CENTER_AZIMUTH_RAW = round(math.degrees(math.atan2(
    CAMERA["R_sensor_to_cam"][2][0], CAMERA["R_sensor_to_cam"][2][1])) * 100) % 36000
# live_pothole.py's plane checks: grid cells, share of cells within PLANE_SUPPORT_M of the
# plane, and tilt from EXPECTED_NORMAL.
PLANE_MIN_CELLS, PLANE_SUPPORT_M, PLANE_MIN_SUPPORT, PLANE_MAX_TILT_DEG = 10, 0.04, 0.6, 15.0
MOTION_MAX_PACKET_AGE_SEC = 0.15
# live_pothole.py's OFFSET_SEC: the camera stamps a frame this long after taking it, so the
# points are moved to the file name's capture time minus this.
OFFSET_SEC = 0.033
# live_pothole.py's pothole check (PotholeScan, line_depths, is_pothole; see the settings
# there): every LiDAR line crossing the pothole gets a straight road through the same line's
# road points beside the area; a line is deep when POTHOLE_MIN_RUN neighbouring points are
# POTHOLE_DEPTH_M or more below it, and the terminal confirms the pothole when at least
# POTHOLE_MIN_LINES lines are deep. Heights are along the road normal measured on the car.
POTHOLE_DEPTH_M, POTHOLE_MIN_LINES, POTHOLE_MIN_RUN = 0.014, 2, 3
ROAD_SIDE_M, ROAD_GUARD_M, ROAD_MIN_POINTS, ROAD_MAX_SIDE_M = 0.15, 0.02, 6, 0.30
ROAD_MIN_SPAN_M, ROAD_MIN_KEPT, ROAD_MAX_AREA_M, ROAD_HUBER_M = 0.04, 4, 1.00, 0.006
ROAD_DROP_M, ROAD_MAX_SPREAD_M, ROAD_MAX_CHANGE_M, ROAD_WIDE_GUARD_M = 0.005, 0.006, 0.003, 0.05
ROAD_NORMAL = np.asarray(EXPECTED_NORMAL, float) / np.linalg.norm(EXPECTED_NORMAL)


def road_plane(xyz, selected):
    """(unit normal, offset) of the road plane fitted to the selected points (all points
    when selected is None), or ValueError when it fails live_pothole.py's checks."""
    n = np.asarray(EXPECTED_NORMAL, float)
    n /= np.linalg.norm(n)
    axis = np.array([1.0, 0.0, 0.0]) - n[0] * n
    axis /= np.linalg.norm(axis)
    R = np.array([axis, np.cross(n, axis), n])
    local = xyz @ R.T
    finite = np.isfinite(local).all(axis=1)
    points = local[finite if selected is None else finite & selected]
    if len(points) < 3 or np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
        points = local[finite]
    if len(points) < 3 or np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
        raise ValueError("plane_unavailable: fewer than three non-collinear finite points")
    keys = np.floor(points[:, :2] / CELL_SIZE_M).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    cells = np.asarray([np.median(points[inverse == i], axis=0)
                        for i in range(int(inverse.max()) + 1)], float).reshape(-1, 3)
    grid_cells = len(cells)
    if len(cells) < 3 or np.linalg.matrix_rank(cells - cells.mean(axis=0)) < 2:
        cells, grid_cells = points.copy(), 0
    A = np.column_stack([cells[:, :2], np.ones(len(cells))])
    target = cells[:, 2]
    triples = np.random.default_rng(20260907).integers(0, len(cells), size=(768, 3))
    matrices = A[triples]
    valid = abs(np.linalg.det(matrices)) > 0.02
    coef = np.linalg.solve(matrices[valid], target[triples[valid]][..., None])[..., 0]
    coef = np.vstack([coef, np.linalg.lstsq(A, target, rcond=None)[0]])
    residual = np.abs(A @ coef.T - target[:, None]) / np.sqrt(1 + (coef[:, :2] ** 2).sum(axis=1))
    trimmed = np.sort(residual, axis=0)[: max(3, int(0.7 * len(cells)))]
    loss = trimmed.mean(axis=0) + 0.25 * np.quantile(residual, 0.75, axis=0)
    beta = coef[int(np.argmin(loss))]
    for _ in range(8):
        root = np.sqrt(np.minimum(1.0, 0.015 / np.maximum(abs(A @ beta - target), 1e-12)))
        new = np.linalg.lstsq(A * root[:, None], target * root, rcond=None)[0]
        converged = np.linalg.norm(new - beta) < 1e-08
        beta = new
        if converged:
            break
    scale = np.sqrt(1 + beta[:2] @ beta[:2])
    support = float(np.mean(np.abs(A @ beta - target) / scale <= PLANE_SUPPORT_M))
    tilt = float(np.degrees(np.arctan(np.sqrt(beta[:2] @ beta[:2]))))
    if grid_cells < PLANE_MIN_CELLS or support < PLANE_MIN_SUPPORT or tilt > PLANE_MAX_TILT_DEG:
        raise ValueError(f"plane_unreliable: {grid_cells} cells, {support:.0%} within "
                         f"{PLANE_SUPPORT_M * 100:g} cm, tilt {tilt:.1f} deg from the mount")
    return np.array([-beta[0], -beta[1], 1.0]) @ R / scale, -float(beta[2]) / scale


def compensate(xyz, times, normal, target, speed):
    """(XYZ moved to the image time, None) as live_pothole.py moves them: by the GPS speed
    along the camera's forward direction on the road; or (None, reason) where
    live_pothole.py would not compensate either."""
    if speed is None:
        return None, "no gps.speed_mps in the JSON"
    direction = np.asarray(CAMERA["R_sensor_to_cam"], float)[2].copy()
    direction -= (direction @ normal) * normal
    length = float(np.linalg.norm(direction))
    if length < 0.1:
        return None, "camera direction cannot be projected onto the road"
    velocity = direction / length * float(speed)
    dt = float(target) - times
    if not np.isfinite(dt).all() or np.max(np.abs(dt), initial=0) > MOTION_MAX_PACKET_AGE_SEC:
        return None, "LiDAR packets more than 150 ms from the image"
    return xyz + dt[:, None] * velocity, None


def along_line(flat, azimuth, step):
    """live_pothole.py's along_line(): distance (m) along the road from the first of a line's
    points (in firing order), on the road through the points smoothed with up to 2 neighbours
    each side, never across missing firings (straight across those)."""
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
    """live_pothole.py's road_line(): robust straight road h = a * s + b through road points,
    ((a, b), spread of the kept points (infinite: not usable), kept)."""
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


def line_depths(geometry, channel, visible, inside, road_ok):
    """live_pothole.py's line_depths() (without its detail): for every LiDAR line crossing the
    area (inside), whether its road beside was measured, its longest run of deep points and
    its deepest point (m)."""
    height, flat, azimuth, step = geometry
    lines = []
    for c in np.unique(channel[inside]):
        on = np.flatnonzero((channel == c) & visible)
        on = on[np.argsort(azimuth[on], kind="stable")]
        area = inside[on]
        s = along_line(flat[on], azimuth[on], step)
        h, road = height[on], road_ok[on] & ~area
        s0, s1 = s[area].min(), s[area].max()
        line = dict(measured=False, run=0, max_depth_m=None)
        lines.append(line)
        if s1 - s0 > ROAD_MAX_AREA_M:
            continue

        def side_points(distance, side, guard):
            near = np.flatnonzero(road & (distance >= guard) & (distance <= ROAD_MAX_SIDE_M))
            near = near[np.argsort(distance[near], kind="stable")]
            chosen = near[distance[near] <= side]
            if len(chosen) < ROAD_MIN_POINTS:
                chosen = near[:ROAD_MIN_POINTS]
            if len(chosen) < ROAD_MIN_POINTS or np.ptp(s[chosen]) < ROAD_MIN_SPAN_M:
                return None
            return chosen

        def fit(side, guard):
            left, right = side_points(s0 - s, side, guard), side_points(s - s1, side, guard)
            if left is None or right is None:
                return None
            both = np.r_[left, right]
            coef, spread, kept = road_line(s[both], h[both])
            left, right = left[kept[:len(left)]], right[kept[len(left):]]
            for side_kept in (left, right):
                if len(side_kept) < ROAD_MIN_KEPT or np.ptp(s[side_kept]) < ROAD_MIN_SPAN_M:
                    return None
            return coef, spread, left, right

        main = fit(ROAD_SIDE_M, ROAD_GUARD_M)
        if main is None:
            continue
        coef, spread, left, right = main
        if spread > ROAD_MAX_SPREAD_M:
            continue
        road_h = coef[0] * s[area] + coef[1]
        wide = fit(ROAD_SIDE_M, ROAD_WIDE_GUARD_M)
        if wide is None:
            continue
        change = float(np.max(np.abs(wide[0][0] * s[area] + wide[0][1] - road_h)))
        short = fit(ROAD_SIDE_M * 2 / 3, ROAD_GUARD_M)
        if short is not None and not (np.array_equal(short[2], left) and np.array_equal(short[3], right)):
            change = max(change, float(np.max(np.abs(short[0][0] * s[area] + short[0][1] - road_h))))
        if change > ROAD_MAX_CHANGE_M:
            continue
        depth = road_h - h[area]
        deep = depth >= POTHOLE_DEPTH_M
        gap = np.diff(azimuth[on][area])
        neighbour = np.r_[False, (gap > 0) & (gap <= 1.5 * step)]
        run = best = 0
        for is_deep, is_next in zip(deep, neighbour):
            run = run + 1 if is_deep and is_next and run else int(is_deep)
            best = max(best, run)
        line.update(measured=True, run=best, max_depth_m=float(depth.max()))
    return lines


def pothole_checks(xyz, times, firings, target, speed, masks, potholes, size):
    """live_pothole.py's pothole check of the frame's potholes: {annotation id: (deep lines,
    measured lines, lines crossing it, deepest depth in m or None, LiDAR points in it)}. As on
    the terminal, the points are moved to the image time along the road normal measured on the
    car (not the fitted plane) and every model area (masks) is kept out of the road."""
    channel, firing, firing_azimuth = firings
    moved, _ = compensate(xyz, times, ROAD_NORMAL, target, speed)
    moved = xyz if moved is None else moved
    pixels, visible = project(moved)
    visible &= (pixels[:, 0] >= 0) & (pixels[:, 0] < size[0]) & (pixels[:, 1] >= 0) & (pixels[:, 1] < size[1])
    uv = np.where(visible[:, None], pixels, 0).astype(int)
    road_ok = visible & ~np.logical_or.reduce(masks)[uv[:, 1], uv[:, 0]]
    raw = np.asarray(firing_azimuth, np.int64)
    steps = np.diff(raw) % 36000
    step = float(np.median(steps[steps > 0])) if np.any(steps > 0) else 36.0
    geometry = (xyz @ ROAD_NORMAL, moved - np.outer(moved @ ROAD_NORMAL, ROAD_NORMAL),
                (raw[firing] - PLANE_CENTER_AZIMUTH_RAW + 18000) % 36000 - 18000, step)
    checks = {}
    for ann_id, mask in potholes:
        inside = visible & mask[uv[:, 1], uv[:, 0]]
        crossing = line_depths(geometry, channel, visible, inside, road_ok) if inside.any() else []
        used = [line for line in crossing if line["measured"]]
        checks[ann_id] = (sum(line["run"] >= POTHOLE_MIN_RUN for line in used), len(used), len(crossing),
                          max((line["max_depth_m"] for line in used), default=None), int(inside.sum()))
    return checks


KST = timezone(timedelta(hours=9))


def capture_time(stem):
    """Camera time (Unix seconds) in the terminal's file name: frame_<KST date>_<time>_<ms>, or
    frame_<capture ns>_<frame> before 2026-10-02. The new names keep milliseconds (<= 1 cm
    of compensation at 10 m/s)."""
    parts = stem.split("_")
    if len(parts) == 3:
        return int(parts[1]) / 1e9
    moment = datetime.strptime(parts[1] + parts[2], "%Y%m%d%H%M%S").replace(tzinfo=KST)
    return moment.timestamp() + int(parts[3]) / 1000


# =============================================================================
# Drawing (replay_certifcate.py)
# =============================================================================


def height_rgb(height_m, range_cm=HEIGHT_RANGE_CM):
    """Fixed signed-height scale; only colors saturate, never measured heights."""
    cm = math.floor(height_m * 100.0 / HEIGHT_BIN_CM + 1e-10) * HEIGHT_BIN_CM
    level = max(-1.0, min(1.0, cm / range_cm))
    for (left, c0), (right, c1) in zip(HEIGHT_STOPS, HEIGHT_STOPS[1:]):
        if level <= right:
            mix = (level - left) / (right - left)
            return tuple(int(round(a + (b - a) * mix)) for a, b in zip(c0, c1))
    return HEIGHT_STOPS[-1][1]


def font_for(size):
    from PIL import ImageFont

    for name in ("DejaVuSans.ttf", "Arial.ttf", "C:/Windows/Fonts/arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def draw_height_legend(canvas, range_cm, note=""):
    from PIL import ImageDraw

    width, height = canvas.size
    margin = max(8, round(height / 60))
    panel_width = min(510, width - 2 * margin)
    if panel_width < 180:
        return
    font = font_for(max(12, round(height / 65)))
    painter = ImageDraw.Draw(canvas)
    left, top = margin, margin
    row = max(19, round(height / 40))
    rows = 4 if note else 3
    painter.rectangle((left, top, left + panel_width, top + row * rows + 16), fill=(12, 24, 34, 235))
    if note:
        painter.text((left + 10, top + row * 3 + 8), note, font=font, fill=(255, 255, 255, 255))
    painter.text((left + 10, top + 6), "Height from fitted road plane (cm)", font=font,
                 fill=(255, 255, 255, 255))
    x0, y0 = left + 12, top + row + 8
    bar_width = panel_width - 24
    for x in range(bar_width):
        cm = (2 * x / max(1, bar_width - 1) - 1) * range_cm
        painter.line((x0 + x, y0, x0 + x, y0 + row - 4), fill=(*height_rgb(cm / 100, range_cm), 255))
    for x, label in ((x0, f"<= -{range_cm:g}"), (x0 + bar_width / 2 - 5, "0"),
                     (x0 + bar_width - 72, f">= +{range_cm:g}")):
        painter.text((x, y0 + row), label, font=font, fill=(255, 255, 255, 255))


def draw_gps_panel(canvas, gps):
    """The frame's GPS from the JSON, top right: latitude, longitude and speed."""
    from PIL import ImageDraw

    width, height = canvas.size
    margin = max(8, round(height / 60))
    font = font_for(max(12, round(height / 65)))
    painter = ImageDraw.Draw(canvas)
    gps = gps or {}
    latitude, longitude, speed = gps.get("latitude_deg"), gps.get("longitude_deg"), gps.get("speed_mps")
    lines = [f"GPS {latitude:.6f}, {longitude:.6f}" if latitude is not None and longitude is not None
             else "GPS: no fix"]
    if speed is not None:
        lines.append(f"speed {speed:.1f} m/s ({speed * 3.6:.0f} km/h)")
    if hasattr(painter, "textbbox"):
        text_width = max(painter.textbbox((0, 0), line, font=font)[2] for line in lines)
    else:  # Pillow 7.x
        text_width = max(painter.textsize(line, font=font)[0] for line in lines)
    row = max(19, round(height / 40))
    left = width - margin - text_width - 20
    painter.rectangle((left, margin, width - margin, margin + row * len(lines) + 12), fill=(12, 24, 34, 235))
    for i, line in enumerate(lines):
        painter.text((left + 10, margin + 6 + i * row), line, font=font, fill=(255, 255, 255, 255))


_CALIBRATION = None


def _init_worker(calibration_path):
    global _CALIBRATION
    _CALIBRATION = load_xt32_calibration(calibration_path)


def render(json_path, output_path):
    """Draw one frame; returns why LiDAR is missing, or None."""
    from PIL import Image, ImageDraw

    json_path, output_path = Path(json_path), Path(output_path)
    doc = json.loads(json_path.read_text(encoding="utf-8"))
    with Image.open(json_path.with_suffix(".jpg")) as source:
        image = source.convert("RGB")
    width, height = image.size
    objects = []
    for ann in doc["annotations"]:
        polygons = [[(int(p[i]), int(p[i + 1])) for i in range(0, len(p) - 1, 2)]
                    for p in ann.get("segmentation") or [] if len(p) >= 6]
        x, y, w, h = (int(v) for v in ann["bbox"])
        objects.append((KINDS[int(ann["category_id"])], ann, polygons, (x, y, x + w, y + h)))

    canvas = image.convert("RGBA")
    line_width = max(2, round(min(width, height) / 300))
    for kind, _, polygons, _ in objects:
        layer = Image.new("RGBA", image.size)
        painter = ImageDraw.Draw(layer)
        for polygon in polygons:
            painter.polygon(polygon, fill=(*COLORS[kind], round(ALPHA * 255)))
        canvas = Image.alpha_composite(canvas, layer)
    painter = ImageDraw.Draw(canvas)

    masks = []
    for _, _, polygons, _ in objects:
        mask = Image.new("1", (width, height), 0)
        for polygon in polygons:
            ImageDraw.Draw(mask).polygon(polygon, fill=1)
        masks.append(np.array(mask, dtype=bool))
    lidar_missing, motion, checks = None, "", {}
    try:
        xyz, selected, times, firings = decode_xt32(json_path.with_suffix(".pcap"), _CALIBRATION)
        speed = (doc.get("gps") or {}).get("speed_mps")
        target = capture_time(json_path.stem) - OFFSET_SEC
        # The terminal's pothole check, which does not need the fitted road plane.
        potholes = [(ann["id"], mask) for (kind, ann, _, _), mask in zip(objects, masks) if kind == "pothole"]
        if potholes:
            checks = pothole_checks(xyz, times, firings, target, speed, masks, potholes, (width, height))
        normal, offset = road_plane(xyz, selected)
        heights = xyz @ normal + offset  # compensation moves points along the road only
        moved, reason = compensate(xyz, times, normal, target, speed)
        motion = (f"motion compensated at {speed:.1f} m/s" if moved is not None
                  else f"not motion compensated: {reason}")
        pixels, valid = project(xyz if moved is None else moved)
        valid &= ((pixels[:, 0] >= 0) & (pixels[:, 0] < width)
                  & (pixels[:, 1] >= 0) & (pixels[:, 1] < height))
        if not valid.any():
            raise ValueError("matched scan has no points in the camera image")
        for (u, v), height_m in zip(pixels[valid].tolist(), heights[valid].tolist()):
            painter.ellipse((u - 1, v - 1, u + 1, v + 1), fill=(*height_rgb(height_m), 255))
    except (OSError, ValueError) as exc:
        lidar_missing = str(exc)

    font = font_for(max(14, round(height / 48)))
    for kind, ann, polygons, (x0, y0, x1, y1) in objects:
        color = (*COLORS[kind], 255)
        for polygon in polygons:
            painter.line(polygon + [polygon[0]], fill=color, width=line_width)
        painter.rectangle((x0, y0, x1, y1), outline=color, width=line_width)
        lines = [f"{kind.upper()} #{ann['id']}"]
        if ann.get("confidence") is not None:
            lines[0] += f" conf {float(ann['confidence']):.2f}"
        if kind == "pothole" and ann["id"] in checks:
            deep, measured, crossing, deepest, points = checks[ann["id"]]
            lines.append(
                "LiDAR: no points in the area" if not points
                else f"LiDAR: road beside measured on {measured} of {crossing} lines" if measured < POTHOLE_MIN_LINES
                else f"deep lines ({POTHOLE_DEPTH_M * 100:g} cm x {POTHOLE_MIN_RUN} points): {deep} of {measured}"
                     f" | deepest {100 * deepest:.1f} cm")
        boxes = []
        for line in lines:
            if hasattr(painter, "textbbox"):
                boxes.append(painter.textbbox((0, 0), line, font=font))
            else:  # Pillow 7.x
                boxes.append((0, 0, *painter.textsize(line, font=font)))
        line_height = max(b[3] - b[1] for b in boxes) + 6
        label_width = max(b[2] - b[0] for b in boxes) + 12
        label_height = line_height * len(lines) + 4
        x = max(0, min(x0, width - label_width))
        y = max(0, min(y0 - label_height, height - label_height))
        painter.rectangle((x, y, x + label_width, y + label_height), fill=color)
        for i, (line, (left, top, _, _)) in enumerate(zip(lines, boxes)):
            painter.text((x + 6 - left, y + 5 + i * line_height - top), line, font=font,
                         fill=(255, 255, 255, 255))
    if lidar_missing is None:
        draw_height_legend(canvas, HEIGHT_RANGE_CM, motion)
    draw_gps_panel(canvas, doc.get("gps"))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(output_path.name + ".tmp")
    canvas.convert("RGB").save(temporary, format="JPEG", quality=95, subsampling=0)
    os.replace(temporary, output_path)
    return lidar_missing


def _render_job(job):
    json_path, output_path = job
    try:
        return json_path, render(json_path, output_path), None
    except Exception as exc:  # one bad frame must not stop the rest
        return json_path, None, f"{type(exc).__name__}: {exc}"


# =============================================================================
# Main
# =============================================================================


def upload_folders(upload_root):
    return sorted(p for p in upload_root.iterdir() if p.is_dir() and p.name.startswith(UPLOAD_PREFIX))


def run(upload_root=UPLOAD_ROOT, output_root=OUTPUT_ROOT, lidar_calibration=LIDAR_CALIBRATION,
        workers=WORKERS, overwrite=OVERWRITE):
    upload_root, output_root = Path(upload_root), Path(output_root)
    if not upload_root.is_dir():
        print(f"Nothing uploaded by the terminal yet: {upload_root} does not exist.")
        return
    load_xt32_calibration(lidar_calibration)  # fail early on a bad calibration file
    import PIL  # noqa: F401  (python3 -m pip install Pillow)

    jobs, total = [], 0
    for folder in upload_folders(upload_root):
        for json_path in sorted(folder.rglob("frame_*.json")):
            if json_path.parent.name != "certifcate" or not json_path.with_suffix(".jpg").is_file():
                continue
            total += 1
            target = output_root / folder.name / json_path.parent.parent.relative_to(folder) / (
                json_path.stem + ".jpg")
            if overwrite or not target.exists() or target.stat().st_mtime < json_path.stat().st_mtime:
                jobs.append((str(json_path), str(target)))
    print(f"Uploads : {upload_root}/{UPLOAD_PREFIX}*  ({total:,} report frames, {len(jobs):,} to draw)")
    print(f"Output  : {output_root}")
    if not jobs:
        return

    no_lidar, failures, done = [], [], 0
    with ProcessPoolExecutor(max_workers=max(1, int(workers)), initializer=_init_worker,
                             initargs=(str(lidar_calibration),)) as executor:
        pending, queue = set(), iter(jobs)
        while True:
            while len(pending) < max(8, int(workers) * 4):
                job = next(queue, None)
                if job is None:
                    break
                pending.add(executor.submit(_render_job, job))
            if not pending:
                break
            finished, pending = wait(pending, return_when=FIRST_COMPLETED)
            for future in finished:
                json_path, lidar_missing, error = future.result()
                done += 1
                if error:
                    failures.append((json_path, error))
                elif lidar_missing:
                    no_lidar.append((json_path, lidar_missing))
                if done % 100 == 0 or done == len(jobs):
                    print(f"Drawn   : {done:,}/{len(jobs):,}", flush=True)
    for json_path, reason in no_lidar:
        print(f"[NO LIDAR] {Path(json_path).name}: {reason}")
    for json_path, error in failures:
        print(f"[FAILED] {json_path}: {error}")


if __name__ == "__main__":
    run()
