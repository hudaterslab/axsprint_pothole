#!/usr/bin/env python3
"""Live pothole detection with pothole_check.py's LiDAR rule.

Everything is main_live.py as it is - camera model, tracking, CSV, uploads, LiDAR scan matching
and motion compensation - except how a model pothole is confirmed. Instead of main_live's road
plane and ring, every LiDAR line crossing the pothole gets a straight road through the same line's
road points beside it, and the pothole counts when at least MIN_LINES lines have MIN_RUN
neighbouring points DEPTH_M or more below their road (pothole_check.py). Cracks are reported as
before. main_live.py itself is unchanged and still runs on its own.

The analysis terminal starts it at login through run_main_live.sh; by hand:
    python3 live_pothole.py
senddata.py imports it, so a resent drive is judged the same way.
"""

import json

import numpy as np

import main_live as ml
import pothole_check as pc

_main_live_stage = ml.lidar_stage


def pothole_evidence(lines, inside, previous):
    """main_live's audit fields of one model pothole, judged by pothole_check (lines: line_depths
    with detail; previous: main_live's own audit row, for the fields that stay)."""
    used = [line for line in lines if line["measured"]]
    keep = pc.is_pothole(lines)
    deepest = max((line["max_depth_m"] for line in used), default=None)
    if keep:
        reason = "depth_pass"
    elif not inside.any():
        reason = "no_mask_points"
    elif len(used) < pc.MIN_LINES:
        reason = "local_reference_unavailable"  # the road beside could not be measured
    else:
        reason = "depth_below_threshold"
    local = dict(rule="pothole_check", depth_m=pc.DEPTH_M, min_lines=pc.MIN_LINES, min_run=pc.MIN_RUN,
                 usable_lines=len(used), deep_lines=sum(line["run"] >= pc.MIN_RUN for line in used),
                 lines=[{name: line[name] for name in ("channel", "points", "measured", "why", "run", "deep",
                                                       "max_depth_m", "median_depth_m")} for line in lines])
    return dict(previous, method="model_mask_line_road_depth", accepted=keep, reason=reason, max_depth_m=deepest,
                threshold_depth_m=pc.DEPTH_M, mask_points=int(inside.sum()), local_evidence_json=json.dumps(local),
                validation_state="accepted" if keep else "unobserved" if reason == "no_mask_points" else "withheld")


def lidar_stage(detections, target, scan, camera, angles, alignment, cache):
    """main_live's lidar_stage, with every model pothole judged again by pothole_check."""
    lidar = _main_live_stage(detections, target, scan, camera, angles, alignment, cache)
    potholes = [i for i, d in enumerate(detections) if int(d.class_id) == 1]
    if not potholes:
        return lidar
    audit = list(lidar["audit"])
    objects = dict(zip(lidar["accepted"], lidar["objects"]))
    points = None
    if scan is not None and abs(scan[3]) <= ml.MAX_SCAN_CENTER_DELTA_SEC:
        sources, start, end, _ = scan
        points = (cache.get((tuple(map(str, sources)), start, end)) or (None,))[0]  # decoded by main_live
    masks = [None if d.mask is None else np.asarray(d.mask, bool) for d in detections]
    shape = next((m.shape for m in masks if m is not None), None)
    if points is None or shape is None:  # no LiDAR scan for the image: nothing is confirmed
        for i in potholes:
            audit[i] = dict(audit[i], accepted=False, validation_state="unobserved")
            objects.pop(i, None)
    else:
        masks = [np.zeros(shape, bool) if m is None else m for m in masks]
        frame = pc.Scan.from_points(points, masks, camera, alignment, target, shape)
        for i in potholes:
            inside = frame.inside(masks[i])
            lines = frame.lines(inside, detail=True) if inside.any() else []
            audit[i] = evidence = pothole_evidence(lines, inside, audit[i])
            objects.pop(i, None)
            if evidence["accepted"]:
                used = [line for line in lines if line["measured"]]
                verified = np.array([p for line in used for p in line["area_index"]], int)
                road = np.c_[frame.road_xyz[verified, :2], -np.array([v for line in used for v in line["depth"]])]
                objects[i] = dict(class_id=1, status="model_mask", point_indices=np.flatnonzero(inside),
                                  filtered_points=int(inside.sum()), verified_point_indices=verified,
                                  verified_road_xyz_m=road,
                                  measurement_record=ml.damage_geometry(road, include_depth=True),
                                  validation={k: v for k, v in evidence.items()
                                              if k not in ("model_object_id", "class_id", "confidence", "box_xyxy")})
        lidar = dict(lidar, has_result=True)  # the scan was used, so its time goes into the report
    accepted = sorted(objects)
    return dict(lidar, audit=audit, accepted=accepted, objects=[objects[i] for i in accepted])


ml.lidar_stage = lidar_stage

if __name__ == "__main__":
    ml.run_service()
