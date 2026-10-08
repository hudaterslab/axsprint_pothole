"""Live pothole detection: the camera model finds cracks and potholes, and the LiDAR
confirms each pothole (PotholeScan, line_depths).

Run: python3 -u live_pothole.py  (the analysis terminal starts it at login, run_live_pothole.sh)
Input: the collector's /mnt/ssd/porthole_runs/YYYYMMDD/run/{frames,lidar,gps,meta}
Only frames recorded after the start are analysed, in order; older ones are skipped.
Detected images are listed in a CSV in each date folder (DETECTIONS_CSV), and their
JPG/JSON/PCAP go to the server when it is reachable; nothing else is kept on the terminal.
The finished recordings themselves go to the server's raw-data folder as they are
(RawUploader), only when .env has PORTHOLE_RAW_UPLOAD=true.
No other project Python file is imported or executed.
Model/calibration files and installed NumPy/OpenCV/DEEPX runtime are data/runtime
dependencies. PTP synchronizes clocks; GPS speed, when available, compensates
vehicle motion between LiDAR packets and the image.
"""

from __future__ import annotations
import os

for _thread_env in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
):
    os.environ.setdefault(_thread_env, "1")
from collections import OrderedDict
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from dataclasses import field
from datetime import datetime
from datetime import timezone
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace
from typing import Iterator
from typing import Optional
from zoneinfo import ZoneInfo
import csv
import cv2
import hashlib
import json
import math
import numpy as np
import queue
import re
import select
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import zlib

try:  # libjpeg-turbo like OpenCV: the same pixels for about a third less CPU (pip install simplejpeg)
    import simplejpeg
except ImportError:  # not installed: OpenCV reads the pictures
    simplejpeg = None

PROJECT = Path(__file__).resolve().parent

ROOT = Path("/mnt/ssd/porthole_runs")  # collector recordings

# The only file the analysis keeps on the terminal, one per date folder of the recordings
# (ROOT/YYYYMMDD/): time, image and damage types of each frame with newly confirmed
# damage, whether or not the server could be reached.
DETECTIONS_CSV = "porthole_detections.csv"

# A frame's JPG/JSON/PCAP wait here, in RAM (/tmp is tmpfs), only until they are sent.
UPLOAD_STAGING = Path("/tmp/porthole_upload")

REMOTE_FOLDER = "porthole_live_analysis"  # server: <PORTHOLE_UPLOAD_DIR>/<this>/<run>/certifcate

UPLOAD_QUEUE_LIMIT = 100  # frames waiting to be sent; more are only listed in the CSV

OFFLINE_RETRY_SEC = 30  # after a failed send, new detections are only listed in the CSV

BATCH_SIZE = 16  # frames per inference batch

POLL_SECONDS = 0.5

# The camera stamps each frame about one frame period (33 ms at 30 fps) after taking it,
# so LiDAR scans are matched and motion-compensated to camera time minus this. Measured
# on the 2026-10-02 drive by lining up road paint in LiDAR reflectivity and the image:
# 33 ms at 3-10 m/s, the same at every speed (a timing offset, not a calibration error).
OFFSET_SEC = 0.033

ALIGNMENT_PROFILE_ID = "ptp_live_recorded_utc_v1"

MODEL_PATH = PROJECT / "best_seg.dxnn"

FUSION_OPENCV_THREADS = 2  # 추적 CPU 병렬화; 별도 추론 프로세스의 기본 스레드는 1개.

CAMERA_CALIBRATION = PROJECT / "camera_calib_best_effort.json"  # lens and camera pose

LIDAR_CALIBRATION = PROJECT / "XT32_Angle_Correction_File.csv"

CONFIDENCE_THRESHOLD = 0.40  # model detections below this are ignored (0.25 until 2026-10-02)

SCAN_SEARCH_HALF_WINDOW_SEC = 0.1

MAX_SCAN_CENTER_DELTA_SEC = 0.055

SCAN_GAP_SEC = 0.005

# A frame waits at most this long (after it was taken) for the LiDAR file covering it. PCAPs are
# sealed every 10 s, so an older frame's scan is not coming (the LiDAR sent nothing): it is
# analysed without LiDAR - cracks are still reported - instead of stalling all later frames.
LIDAR_WAIT_MAX_SEC = 60

# Each frame is analysed this long after it arrived, as the frames come: a steady load. Without it
# the 300 frames a sealed 10 s PCAP releases were analysed at once, at about twice real time, and
# the CPU went from ~20% to ~50% and back every 10 s. 11 s covers the 10 s PCAP and its sealing,
# so frames do not wait longer than before for LiDAR; a backlog is still worked off at full speed.
PACE_DELAY_SEC = 11

# A gap between analysed pictures longer than this (camera or PTP outage) starts tracking afresh:
# the car has moved on, and an old track must not swallow new damage at the same spot in the image.
TRACK_GAP_SEC = 1.0

# Road plane points: the LiDAR firing nearest the camera's optical axis and this many
# firings on each side (201 firings, all 32 channels), as codecode/lcz_to_pcap.py crops
# a scan.
PLANE_SIDE_FIRINGS = 100

# Road plane checks: fewer cells, less support or more tilt from the mount means the fit
# locked onto something else (a truck, the bonnet), so that frame's crack measurements are
# left out. Potholes do not use the plane (line_depths).
PLANE_MIN_CELLS = 10

PLANE_SUPPORT_M = 0.04  # a 35 cm cell supports the plane when its median is this close

PLANE_MIN_SUPPORT = 0.6

PLANE_MAX_TILT_DEG = 15.0  # from RoadPlaneSettings.expected_normal

# Pothole check (PotholeScan, line_depths, is_pothole), for every pothole area of the model:
# - Height is a point's position along the road normal measured on the car (ROAD_NORMAL),
#   not the sensor z, as the LiDAR is tilted.
# - Every LiDAR line (channel) crossing the area gets its own road: a straight line through
#   the same line's road points beside the area, from ROAD_GUARD_M to ROAD_SIDE_M before its
#   first and after its last point (distances on the road), outside every model area; where
#   points are sparse a side reaches up to ROAD_MAX_SIDE_M for ROAD_MIN_POINTS points. Points
#   further than ROAD_HUBER_M off the line count less, points more than ROAD_DROP_M below it
#   are dropped and the line drawn again, so the sunken rim of a pothole cannot pull the road
#   down. The road bends 3-7 mm over 1 m but is close to straight over 15 cm, and each line has
#   its own fixed offset (+-1-2 mm), hence one short road per line rather than one plane.
# - A line is not used when a side has too few road points, the area is longer than
#   ROAD_MAX_AREA_M along it, its road points scatter more than ROAD_MAX_SPREAD_M around the
#   line, or the road under the area moves more than ROAD_MAX_CHANGE_M with sides two thirds
#   as long or a guard of ROAD_WIDE_GUARD_M.
# - The pothole counts when at least POTHOLE_MIN_LINES used lines have POTHOLE_MIN_RUN
#   neighbouring points (consecutive firings) POTHOLE_DEPTH_M or more below their road.
# On the 2026-10-02 drive the real pothole (frames 7622-7624) passed and all lane paint was
# filtered out.
POTHOLE_DEPTH_M = 0.014  # a point this far below its line's road is deep
POTHOLE_MIN_LINES = 2  # deep lines needed for a pothole
POTHOLE_MIN_RUN = 3  # neighbouring deep points that make a line deep
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


def server_transport_source():
    """Build the stdlib-only receiver from the functions implemented in this file."""
    import inspect

    imports = "import hashlib, json, os, shutil, sys, time, uuid, zlib\nfrom pathlib import Path\n"
    constants = f"CHUNK = {CHUNK!r}\nMAX_HEADER = {MAX_HEADER!r}\nMAX_FRAME = {MAX_FRAME!r}\nPROTOCOL = {PROTOCOL!r}\n"
    source = "\n".join(
        inspect.getsource(f)
        for f in [
            triplet_error,
            validate_manifest,
            clock_stable,
            content_aliases,
            fsync_dir,
            decode_wire_payload,
            receive_frame_payload,
            persist_frame_payload,
            serve,
        ]
    )
    return imports + constants + source + "\nserve(sys.argv[2])\n"


def clean(x):
    if isinstance(x, np.ndarray):
        return clean(x.tolist())
    if isinstance(x, np.generic):
        return clean(x.item())
    if isinstance(x, dict):
        return {str(k): clean(v) for (k, v) in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    if isinstance(x, float) and (not np.isfinite(x)):
        return None
    return x


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


CHUNK = 64 * 1024

MAX_HEADER = 4 * 1024 * 1024

MAX_FRAME = 256 * 1024 * 1024

PROTOCOL = "porthole_artifacts_v4"


def triplet_error(files):
    """Only one same-stem JPEG/JSON/nonempty PCAP may reach the receiver."""
    if not isinstance(files, dict) or len(files) != 3:
        return "Upload requires exactly one JPG, one JSON and one PCAP"
    if any(not isinstance(name, str) for name in files):
        return "Invalid artifact filename"
    if {Path(name).suffix for name in files} != {".jpg", ".json", ".pcap"}:
        return "Upload only accepts JPG, JSON and PCAP"
    if len({Path(name).stem for name in files}) != 1:
        return "JPG, JSON and PCAP must have the same basename"
    pcap = next(info for name, info in files.items() if Path(name).suffix == ".pcap")
    if not isinstance(pcap, dict) or type(pcap.get("bytes")) is not int or pcap["bytes"] <= 24:
        return "Empty PCAP cannot be uploaded"
    return ""


def validate_manifest(manifest):
    token = manifest.get("_receive_token", "")
    if len(token) != 32 or any((c not in "0123456789abcdef" for c in token)):
        raise ValueError("Invalid receipt token")
    if type(manifest.get("frame_index")) is not int:
        raise ValueError("Invalid frame index")
    files = manifest.get("files")
    if not isinstance(files, dict) or not files or len(files) > 4096:
        raise ValueError("Invalid artifact list")
    error = triplet_error(files)
    if error:
        raise ValueError(error)
    total = 0
    for name, info in files.items():
        if (
            not isinstance(name, str)
            or not name
            or name in (".", "..")
            or ("/" in name)
            or ("\\" in name)
            or ("\x00" in name)
            or (Path(name).name != name)
            or Path(name).suffix not in (".jpg", ".json", ".pcap")
        ):
            raise ValueError("Invalid artifact filename")
        size = info.get("bytes")
        digest = info.get("sha256", "")
        if (
            type(size) is not int
            or size < 0
            or len(digest) != 64
            or any((c not in "0123456789abcdef" for c in digest))
        ):
            raise ValueError("Invalid artifact size/hash")
        total += size
    if total > MAX_FRAME:
        raise ValueError("Artifact frame exceeds transport limit")
    return total


def clock_stable(wall, mono):
    return abs(time.time_ns() - wall - (time.monotonic_ns() - mono)) < 10000000


def content_aliases(files):
    """Send identical image bytes once, retaining every final artifact file."""
    seen, aliases = ({}, {})
    for name, info in files.items():
        identity = (info["bytes"], info["sha256"])
        if identity in seen:
            aliases[name] = seen[identity]
        else:
            seen[identity] = name
    return aliases


def fsync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def payload_chunks(source, files, aliases):
    """Read and verify the exact bytes before transmission, coalescing small files."""
    pending = bytearray()
    for name, info in files.items():
        digest = hashlib.sha256()
        with (source / name).open("rb") as stream:
            if os.fstat(stream.fileno()).st_size != info["bytes"]:
                raise ValueError("Source artifact size changed")
            remaining = info["bytes"]
            while remaining:
                block = stream.read(min(CHUNK - len(pending), remaining))
                if not block:
                    raise EOFError("Source artifact truncated")
                digest.update(block)
                remaining -= len(block)
                if name not in aliases:
                    pending.extend(block)
                    if len(pending) == CHUNK:
                        yield bytes(pending)
                        pending.clear()
            if stream.read(1) or digest.hexdigest() != info["sha256"]:
                raise ValueError("Source artifact checksum changed: " + name)
    if pending:
        yield bytes(pending)


@dataclass(frozen=True)
class PreparedUpload:
    manifest: dict
    aliases: dict
    chunks: tuple
    payload_bytes: int
    wire_bytes: int
    wire_encoding: str
    wire_segments: tuple
    preparation_ms: float


def prepare_upload(source, manifest):
    """Complete source I/O, hashes and optional lossless encoding before send time."""
    begin = time.monotonic_ns()
    manifest = dict(manifest, _receive_token=uuid.uuid4().hex)
    manifest["files"] = {name: dict(info) for name, info in manifest["files"].items()}
    validate_manifest(manifest)
    aliases = content_aliases(manifest["files"])
    expected = sum(info["bytes"] for name, info in manifest["files"].items() if name not in aliases)
    encoded_files, segments = [], []
    for name, info in manifest["files"].items():
        raw = b"".join(payload_chunks(Path(source), {name: info}, aliases))
        if name in aliases:
            continue  # Alias content was verified but is sent only once.
        encoded, encoding = raw, "identity"
        # JPEG already has compression. Recompressing it wasted most CPU time.
        if Path(name).suffix.lower() in (".json", ".pcap") and len(raw) >= 1024:
            compressed = zlib.compress(raw, 1)
            if len(compressed) <= len(raw) - max(256, int(len(raw) * 0.05)):
                encoded, encoding = compressed, "zlib"
        encoded_files.append(encoded)
        segments.append(dict(name=name, wire_encoding=encoding, wire_bytes=len(encoded)))
    wire = b"".join(encoded_files)
    encoding = "files" if any(s["wire_encoding"] == "zlib" for s in segments) else "identity"
    return PreparedUpload(
        manifest,
        aliases,
        (wire,),
        expected,
        len(wire),
        encoding,
        tuple(segments),
        (time.monotonic_ns() - begin) / 1e6,
    )


def decode_wire_payload(wire, encoding, payload_bytes):
    if encoding == "identity":
        if len(wire) != payload_bytes:
            raise ValueError("Uncompressed payload size mismatch")
        return wire
    if encoding != "zlib" or not 0 < len(wire) < payload_bytes:
        raise ValueError("Compressed payload size mismatch")
    decoder = zlib.decompressobj()
    payload = decoder.decompress(wire, payload_bytes + 1)
    if (
        len(payload) != payload_bytes
        or not decoder.eof
        or decoder.unconsumed_tail
        or decoder.unused_data
    ):
        raise ValueError("Invalid or oversized decoded payload")
    return payload


def receive_frame_payload(stream, request, payload_bytes):
    """Bounded receive/decode. Receipt means all ORIGINAL payload bytes are ready."""
    encoding = request.get("wire_encoding")
    wire_bytes = request.get("wire_bytes")
    if (
        encoding not in ("identity", "zlib", "files")
        or type(wire_bytes) is not int
        or not 0 <= wire_bytes <= MAX_FRAME
    ):
        raise ValueError("Invalid payload encoding/size")
    if encoding == "identity" and wire_bytes != payload_bytes:
        raise ValueError("Uncompressed payload size mismatch")
    if encoding == "zlib" and not 0 < wire_bytes < payload_bytes:
        raise ValueError("Compressed payload size mismatch")
    segments = []
    if encoding == "files":
        aliases = content_aliases(request["files"])
        names = [name for name in request["files"] if name not in aliases]
        segments = request.get("wire_segments")
        if (
            not isinstance(segments, list)
            or len(segments) != len(names)
            or any(not isinstance(s, dict) for s in segments)
            or [s.get("name") for s in segments] != names
        ):
            raise ValueError("Invalid file payload mapping")
        for segment in segments:
            size = segment.get("wire_bytes")
            kind = segment.get("wire_encoding")
            raw_size = request["files"][segment["name"]]["bytes"]
            if (
                type(size) is not int
                or kind not in ("identity", "zlib")
                or (kind == "identity" and size != raw_size)
                or (kind == "zlib" and not 0 < size < raw_size)
            ):
                raise ValueError("Invalid file payload encoding/size")
        if sum(s["wire_bytes"] for s in segments) != wire_bytes:
            raise ValueError("File payload sizes do not match wire bytes")
    wall, mono = time.time_ns(), time.monotonic_ns()
    wire = stream.read(wire_bytes)
    wire_received_ns = time.monotonic_ns()
    if len(wire) != wire_bytes:
        raise EOFError("Artifact payload disconnected")
    if encoding == "files":
        offset, restored = 0, []
        for segment in segments:
            end = offset + segment["wire_bytes"]
            restored.append(
                decode_wire_payload(
                    wire[offset:end],
                    segment["wire_encoding"],
                    request["files"][segment["name"]]["bytes"],
                )
            )
            offset = end
        payload = b"".join(restored)
        if len(payload) != payload_bytes:
            raise ValueError("Restored payload size mismatch")
    else:
        payload = decode_wire_payload(wire, encoding, payload_bytes)
    # Keep decode time inside receive_gap: do not report compressed bytes as a
    # complete original JPG/JSON/PCAP payload before they have been restored.
    received_wall, received_mono = time.time_ns(), time.monotonic_ns()
    timing = dict(
        server_received_epoch_ns=received_wall,
        server_received_monotonic_ns=received_mono,
        server_receiver_clock_stable=clock_stable(wall, mono),
        server_payload_receive_ms=(received_mono - mono) / 1e6,
        server_wire_receive_ms=(wire_received_ns - mono) / 1e6,
        server_payload_decode_ms=(received_mono - wire_received_ns) / 1e6,
        wire_bytes=wire_bytes,
        wire_encoding=encoding,
    )
    return payload, timing, wall, mono


def persist_frame_payload(target, request, aliases, payload):
    """Verify and persist original files after complete in-memory receipt."""
    stage = target / (".pending_upload_" + request["_receive_token"])
    stage.mkdir(mode=448)
    staged, payload_offset, write_ns = [], 0, 0
    payload_view = memoryview(payload)
    try:
        for name, info in request["files"].items():
            path = stage / name
            staged.append(path)
            if name in aliases:
                started = time.monotonic_ns()
                shutil.copyfile(stage / aliases[name], path)
                write_ns += time.monotonic_ns() - started
                continue
            remaining = info["bytes"]
            with path.open("xb", buffering=0) as output:
                while remaining:
                    size = min(CHUNK, remaining)
                    view = payload_view[payload_offset : payload_offset + size]
                    payload_offset += size
                    started = time.monotonic_ns()
                    while view:
                        written = output.write(view)
                        if not written:
                            raise OSError("Artifact write failed")
                        view = view[written:]
                    write_ns += time.monotonic_ns() - started
                    remaining -= size
        for name, info in request["files"].items():
            digest = hashlib.sha256()
            with (stage / name).open("rb") as source:
                for block in iter(lambda: source.read(CHUNK), b""):
                    digest.update(block)
                if (
                    os.fstat(source.fileno()).st_size != info["bytes"]
                    or digest.hexdigest() != info["sha256"]
                ):
                    raise ValueError("Server artifact checksum mismatch: " + name)
                os.fsync(source.fileno())
        for name in request["files"]:
            os.replace(stage / name, target / name)
        fsync_dir(target)
        return write_ns / 1e6
    finally:
        for path in staged:
            path.unlink(missing_ok=True)
        stage.rmdir()


def serve(target):
    target = Path(target)
    target.mkdir(parents=True, exist_ok=True)
    stream = sys.stdin.buffer

    def reply(payload):
        print(json.dumps(payload, separators=(",", ":")), flush=True)

    reply(dict(ready=PROTOCOL))
    while True:
        line = stream.readline(MAX_HEADER + 1)
        if not line:
            return
        if len(line) > MAX_HEADER or not line.endswith(b"\n"):
            raise ValueError("Invalid protocol header")
        request = json.loads(line)
        if request.get("op") == "probe":
            t2 = time.time_ns()
            reply(dict(t2=t2, m2=time.monotonic_ns(), t3=time.time_ns()))
            continue
        if request.get("op") != "frame":
            raise ValueError("Unknown operation")
        total = validate_manifest(request)
        aliases = content_aliases(request["files"])
        if request.get("aliases", {}) != aliases:
            raise ValueError("Invalid duplicate-content mapping")
        payload_bytes = sum(
            info["bytes"] for name, info in request["files"].items() if name not in aliases
        )
        payload, timing, wall, mono = receive_frame_payload(stream, request, payload_bytes)
        write_ms = persist_frame_payload(target, request, aliases, payload)
        reply(
            dict(
                token=request["_receive_token"],
                frame_index=request["frame_index"],
                files=len(request["files"]),
                bytes=total,
                payload_bytes=payload_bytes,
                **timing,
                server_receipt_clock_stable=clock_stable(
                    timing["server_received_epoch_ns"],
                    timing["server_received_monotonic_ns"],
                ),
                server_clock_stable=clock_stable(wall, mono),
                server_verified_epoch_ns=time.time_ns(),
                hashes_verified=True,
                definition="complete_payload_in_memory_before_file_write_v3",
                transport=PROTOCOL,
                server_payload_write_ms=write_ms,
            )
        )


class PersistentUploader:
    def __init__(self, options, relative, timeout=60):
        self.options, self.relative, self.timeout = (options, relative, timeout)
        self.proc = None
        self.buffer = bytearray()

    def _write(self, payload, deadline):
        view = memoryview(payload)
        fd = self.proc.stdin.fileno()
        while view:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
                raise TimeoutError("SSH upload write timed out")
            try:
                n = os.write(fd, view[:CHUNK])
            except BlockingIOError:
                continue
            if not n:
                raise ConnectionError("SSH upload disconnected")
            view = view[n:]

    def _json(self, payload, deadline):
        encoded = json.dumps(payload, separators=(",", ":")).encode() + b"\n"
        if len(encoded) > MAX_HEADER:
            raise ValueError("Upload header too large")
        self._write(encoded, deadline)

    def _read(self, deadline):
        fd = self.proc.stdout.fileno()
        while b"\n" not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
                raise TimeoutError("SSH upload response timed out")
            block = os.read(fd, CHUNK)
            if not block:
                raise ConnectionError("SSH upload disconnected before receipt")
            self.buffer.extend(block)
            if len(self.buffer) > MAX_HEADER:
                raise ValueError("Upload response too large")
        line, _, remainder = self.buffer.partition(b"\n")
        self.buffer = bytearray(remainder)
        return json.loads(line)

    def connect(self):
        if self.proc is not None and self.proc.poll() is None:
            return
        self.close()
        target = self.options.destination.rstrip("/") + "/" + self.relative.as_posix()
        command = [
            "ssh",
            "-T",
            "-i",
            str(self.options.key),
            "-o",
            "BatchMode=yes",
            "-o",
            "StrictHostKeyChecking=yes",
            "-o",
            "ConnectTimeout=10",
            "-o",
            "ServerAliveInterval=15",
            "-o",
            "ServerAliveCountMax=2",
            f"{self.options.user}@{self.options.host}",
            "python3 -u -c "
            + shlex.quote(SERVER_TRANSPORT_SOURCE)
            + " --serve "
            + shlex.quote(target),
        ]
        self.proc = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0
        )
        os.set_blocking(self.proc.stdin.fileno(), False)
        self.buffer.clear()
        try:
            if self._read(time.monotonic() + self.timeout) != {"ready": PROTOCOL}:
                raise RuntimeError("SSH upload handshake failed")
        except Exception:
            self.close()
            raise

    def _send_payload(self, prepared, deadline):
        rate = self.options.bw_kib * 1024
        wall, mono = time.time_ns(), time.monotonic_ns()
        started, sent, wait_ns = mono / 1e9, 0, 0
        for chunk in prepared.chunks:
            for offset in range(0, len(chunk), CHUNK):
                block = memoryview(chunk)[offset : offset + CHUNK]
                delay = started + max(0, sent + len(block) - CHUNK) / rate - time.monotonic()
                if delay > 0:
                    if time.monotonic() + delay >= deadline:
                        raise TimeoutError("Upload bandwidth deadline exceeded")
                    before = time.monotonic_ns()
                    time.sleep(delay)
                    wait_ns += time.monotonic_ns() - before
                self._write(block, deadline)
                sent += len(block)
        if sent != prepared.wire_bytes:
            raise ValueError("Sent payload size mismatch")
        return dict(
            client_payload_started_epoch_ns=wall,
            client_payload_started_monotonic_ns=mono,
            client_payload_send_ms=(time.monotonic_ns() - mono) / 1e6,
            client_bandwidth_wait_ms=wait_ns / 1e6,
            client_payload_prepare_ms=prepared.preparation_ms,
        )

    def _clock_receipt(self, result, deadline):
        samples, probes = [], []
        for _ in range(5):
            t1, m1 = time.time_ns(), time.monotonic_ns()
            self._json({"op": "probe"}, deadline)
            probes.append((t1, m1))
        for t1, m1 in probes:
            probe = self._read(deadline)
            t4, m4 = time.time_ns(), time.monotonic_ns()
            if (
                abs(
                    probe["t2"]
                    - result["server_received_epoch_ns"]
                    - (probe["m2"] - result["server_received_monotonic_ns"])
                )
                >= 10000000
            ):
                result["server_receipt_clock_stable"] = False
            if abs(t4 - t1 - (m4 - m1)) < 10000000:
                rtt = t4 - t1 - (probe["t3"] - probe["t2"])
                if rtt >= 0:
                    samples.append((rtt, (probe["t2"] - t1 + probe["t3"] - t4) / 2))
        result.update(
            client_ack_epoch_ns=time.time_ns(),
            client_ack_monotonic_ns=time.monotonic_ns(),
        )
        if samples and result.get("server_clock_stable"):
            rtt, offset = min(samples)
            result.update(clock_offset_ns=offset, clock_uncertainty_ms=rtt / 2000000.0)

    def upload(self, source, manifest):
        try:
            prepared = prepare_upload(source, manifest)
            self.connect()
            deadline = time.monotonic() + max(
                self.timeout,
                prepared.wire_bytes / (self.options.bw_kib * 1024) * 2 + 10,
            )
            request = prepared.manifest
            self._json(
                dict(
                    op="frame",
                    frame_index=request["frame_index"],
                    _receive_token=request["_receive_token"],
                    files=request["files"],
                    aliases=prepared.aliases,
                    wire_bytes=prepared.wire_bytes,
                    wire_encoding=prepared.wire_encoding,
                    wire_segments=prepared.wire_segments,
                ),
                deadline,
            )
            send_timing = self._send_payload(prepared, deadline)
            result = self._read(deadline)
            expected = dict(
                token=request["_receive_token"],
                frame_index=request["frame_index"],
                files=len(request["files"]),
                bytes=sum(f["bytes"] for f in request["files"].values()),
                payload_bytes=prepared.payload_bytes,
                wire_bytes=prepared.wire_bytes,
                wire_encoding=prepared.wire_encoding,
                hashes_verified=True,
                transport=PROTOCOL,
            )
            if any(result.get(key) != value for key, value in expected.items()):
                raise ValueError("Invalid server persistence receipt")
            result.update(send_timing)
            self._clock_receipt(result, deadline)
            return result
        except Exception:
            self.close()
            raise

    def close(self):
        proc, self.proc = self.proc, None
        self.buffer.clear()
        if proc is None:
            return
        proc.stdin.close()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        proc.stdout.close()


class PermanentUploadError(Exception):
    """The server rejected this frame; sending the same request again cannot succeed."""


class _RefuseRedirect(urllib.request.HTTPRedirectHandler):
    # urllib would repeat a redirected POST as a bodiless GET and report success.
    def redirect_request(self, *args, **kwargs):
        return None


class HttpUploader:
    """POST one frame's JPG, JSON and PCAP to PORTHOLE_API_URL as multipart/form-data.

    Parts are named jpg, json and pcap; X-Record-Id carries the JSON record_id so
    the server can drop repeats, and PORTHOLE_API_TOKEN, if set, is sent as a
    Bearer token. 2xx and 409 (already received) count as delivered. Malformed
    requests (400, 413, 415, 422) are permanent; everything else, including
    redirects, counts as a failed send (UploadWorker then drops the frames waiting).
    """

    CONTENT_TYPES = dict(jpg="image/jpeg", json="application/json",
                         pcap="application/vnd.tcpdump.pcap")
    PERMANENT_STATUS = frozenset((400, 413, 415, 422))

    def __init__(self, options, timeout=60):
        self.options, self.timeout = (options, timeout)
        self.opener = urllib.request.build_opener(_RefuseRedirect)

    def upload(self, source, manifest):
        parts = {}
        for name, info in manifest["files"].items():
            data = (Path(source) / name).read_bytes()
            if len(data) != info["bytes"] or hashlib.sha256(data).hexdigest() != info["sha256"]:
                raise ValueError(f"Artifact changed after commit: {name}")
            parts[Path(name).suffix.lstrip(".").lower()] = (name, data)
        if set(parts) != set(self.CONTENT_TYPES):
            raise PermanentUploadError("Upload needs exactly one JPG, JSON and PCAP")
        record_id = json.loads(parts["json"][1])["record_id"]
        boundary = uuid.uuid4().hex
        body = bytearray()
        for field, content_type in self.CONTENT_TYPES.items():
            name, data = parts[field]
            body += (
                f'--{boundary}\r\nContent-Disposition: form-data; name="{field}"; '
                f'filename="{name}"\r\nContent-Type: {content_type}\r\n\r\n'
            ).encode()
            body += data + b"\r\n"
        body += f"--{boundary}--\r\n".encode()
        headers = {"Content-Type": f"multipart/form-data; boundary={boundary}",
                   "X-Record-Id": record_id}
        if self.options.api_token:
            headers["Authorization"] = f"Bearer {self.options.api_token}"
        request = urllib.request.Request(
            self.options.api_url, data=bytes(body), headers=headers, method="POST"
        )
        started = time.monotonic_ns()
        try:
            with self.opener.open(request, timeout=self.timeout) as response:
                status, text = (response.status, response.read(MAX_HEADER))
        except urllib.error.HTTPError as exc:
            detail = exc.read(512).decode(errors="replace")
            if exc.code in self.PERMANENT_STATUS:
                raise PermanentUploadError(f"HTTP {exc.code}: {detail}") from None
            if exc.code != 409:
                raise RuntimeError(f"API upload failed: HTTP {exc.code}: {detail}") from None
            status, text = (409, b"already received")
        return dict(
            transport="http_multipart_v1", url=self.options.api_url, record_id=record_id,
            http_status=status, response=text.decode(errors="replace")[:2000],
            files=len(parts), bytes=len(body),
            client_send_ms=(time.monotonic_ns() - started) / 1e6,
            completed_epoch_ns=time.time_ns(),
        )

    def close(self):
        pass


class LiDAR2Camera:
    """Project sensor XYZ with the lens and camera pose of the calibration JSON."""

    def __init__(self, calibration_file):
        self.path = Path(calibration_file).expanduser().resolve()
        data = json.loads(self.path.read_text(encoding="utf-8"))
        model = str(data.get("model", ""))
        if model and (not model.startswith("fisheye_equidistant")):
            raise ValueError(f"unsupported camera model: {model}")
        self.fx, self.fy = (float(data["fx"]), float(data["fy"]))
        self.cx, self.cy = (float(data["cx"]), float(data["cy"]))
        self.k1, self.k2 = (float(data["D"][0]), float(data["D"][1]))
        self.R = np.asarray(data["R_sensor_to_cam"], dtype=np.float64)
        self.t = np.asarray(data["t_cam"], dtype=np.float64).reshape(3)
        if self.R.shape != (3, 3):
            raise ValueError("R_sensor_to_cam must be 3x3")
        if (
            not np.isfinite(self.R).all()
            or not np.isfinite(self.t).all()
            or not np.allclose(self.R @ self.R.T, np.eye(3), rtol=0, atol=1e-8)
            or abs(np.linalg.det(self.R) - 1) > 1e-8
        ):
            raise ValueError("invalid camera rotation/translation")
        self.extrinsics = {
            "schema": "sensor_to_camera_extrinsics_v1",
            "id": self.path.name,
            "R_sensor_to_cam": self.R.tolist(),
            "t_cam": self.t.tolist(),
        }
        # Raw LiDAR azimuth (0.01 degree) of the optical axis; plane_points centres on it.
        self.axis_azimuth_raw = round(math.degrees(math.atan2(self.R[2, 0], self.R[2, 1])) * 100) % 36000

    def convert_3D_to_camera_coords(self, sensor_xyz):
        points = np.asarray(sensor_xyz, dtype=np.float64)
        return (points - self.t) @ self.R.T

    def project(self, sensor_xyz):
        camera = self.convert_3D_to_camera_coords(sensor_xyz)
        depth = camera[:, 2]
        valid = depth > 1e-06
        safe = np.where(valid, depth, 1.0)
        a, b = (camera[:, 0] / safe, camera[:, 1] / safe)
        radius = np.hypot(a, b)
        theta = np.arctan(radius)
        distorted = theta * (1.0 + self.k1 * theta**2 + self.k2 * theta**4)
        scale = np.where(radius > 1e-09, distorted / radius, 1.0)
        pixels = np.column_stack([self.fx * a * scale + self.cx, self.fy * b * scale + self.cy])
        valid &= np.isfinite(pixels).all(axis=1)
        return (pixels, valid)


model_infer__CLASS_NAMES = ("Crack", "Pothole")

SUSPECT_CLASS_IDS = frozenset((0, 1))

MASK_COEFFICIENTS = 32


@dataclass(frozen=True)
class LetterboxContext:
    original_width: int
    original_height: int
    scale: float
    pad_x: int
    pad_y: int
    resized_width: int
    resized_height: int


@dataclass
class Detection:
    class_id: int
    confidence: float
    box_xyxy: np.ndarray
    mask_coefficients: np.ndarray
    mask: Optional[np.ndarray] = None


def letterbox_rgb_uint8(image_bgr: np.ndarray, out: np.ndarray) -> tuple[np.ndarray, LetterboxContext]:
    """Fill out, a (1, 640, 640, 3) uint8 array, with the NHWC RGB model input."""
    size = 640
    height, width = image_bgr.shape[:2]
    scale = min(size / width, size / height)
    resized_width = max(1, min(size, int(round(width * scale))))
    resized_height = max(1, min(size, int(round(height * scale))))
    if image_bgr.ndim == 3 and (width, height) == (3 * resized_width, 3 * resized_height):
        # The camera's 1920x1080: cv2.INTER_LINEAR at exactly a third takes source pixel
        # 3 * i + 1 with weight 1 (its fixed-point weights are 2048 and 0), so taking those
        # pixels gives the same bytes at a third of the CPU time.
        resized = np.ascontiguousarray(image_bgr[1::3, 1::3])
    else:
        resized = cv2.resize(image_bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    pad_x = (size - resized_width) // 2
    pad_y = (size - resized_height) // 2
    input_nhwc = out
    input_nhwc.fill(114)
    # cvtColor swaps the channels as resized[:, :, ::-1] would, without numpy's slow strided copy.
    input_nhwc[0, pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = cv2.cvtColor(
        resized, cv2.COLOR_BGR2RGB
    )
    context = LetterboxContext(
        original_width=width,
        original_height=height,
        scale=scale,
        pad_x=pad_x,
        pad_y=pad_y,
        resized_width=resized_width,
        resized_height=resized_height,
    )
    return (input_nhwc, context)


def box_iou(box_a: np.ndarray, box_b: np.ndarray) -> float:
    x1 = max(float(box_a[0]), float(box_b[0]))
    y1 = max(float(box_a[1]), float(box_b[1]))
    x2 = min(float(box_a[2]), float(box_b[2]))
    y2 = min(float(box_a[3]), float(box_b[3]))
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, float(box_a[2] - box_a[0])) * max(0.0, float(box_a[3] - box_a[1]))
    area_b = max(0.0, float(box_b[2] - box_b[0])) * max(0.0, float(box_b[3] - box_b[1]))
    union = area_a + area_b - intersection
    return intersection / union if union > 0.0 else 0.0


def class_aware_nms(detections: list[Detection], iou_threshold: float) -> list[Detection]:
    kept: list[Detection] = []
    for class_id in sorted({item.class_id for item in detections}):
        remaining = sorted(
            (item for item in detections if item.class_id == class_id),
            key=lambda item: -item.confidence,
        )
        while remaining:
            winner = remaining.pop(0)
            kept.append(winner)
            remaining = [
                item
                for item in remaining
                if box_iou(winner.box_xyxy, item.box_xyxy) <= iou_threshold
            ]
    return kept


def decode_detections(
    output0: np.ndarray,
    context: LetterboxContext,
    confidence_threshold: float,
    iou_threshold: float,
    class_names=model_infer__CLASS_NAMES,
    selected_class_ids=SUSPECT_CLASS_IDS,
) -> list[Detection]:
    channels = 4 + len(class_names) + MASK_COEFFICIENTS
    prediction = np.asarray(output0)
    if prediction.ndim == 3:
        prediction = prediction[0]
    if prediction.shape == (8400, channels):
        prediction = prediction.T
    if prediction.shape != (channels, 8400):
        raise ValueError(f"unexpected detection output shape: {prediction.shape}")
    class_scores = prediction[4 : 4 + len(class_names)]
    class_ids = np.argmax(class_scores, axis=0)
    confidences = class_scores.max(axis=0)  # the score at class_ids
    keep = confidences >= confidence_threshold
    if not set(range(len(class_names))) <= set(selected_class_ids):  # this model: every class
        keep &= np.isin(class_ids, np.fromiter(selected_class_ids, dtype=np.int64))
    selected = np.flatnonzero(keep)
    detections: list[Detection] = []
    for index in selected:
        center_x, center_y, width, height = prediction[:4, index]
        model_box = np.asarray(
            [
                center_x - width / 2.0,
                center_y - height / 2.0,
                center_x + width / 2.0,
                center_y + height / 2.0,
            ],
            dtype=np.float32,
        )
        model_box[[0, 2]] = (model_box[[0, 2]] - context.pad_x) / context.scale
        model_box[[1, 3]] = (model_box[[1, 3]] - context.pad_y) / context.scale
        model_box[[0, 2]] = np.clip(model_box[[0, 2]], 0, context.original_width - 1)
        model_box[[1, 3]] = np.clip(model_box[[1, 3]], 0, context.original_height - 1)
        if model_box[2] - model_box[0] < 2 or model_box[3] - model_box[1] < 2:
            continue
        detections.append(
            Detection(
                class_id=int(class_ids[index]),
                confidence=float(confidences[index]),
                box_xyxy=model_box,
                mask_coefficients=prediction[
                    4 + len(class_names) : channels, index
                ].astype(np.float32, copy=True),
            )
        )
    return class_aware_nms(detections, iou_threshold)


def attach_masks(
    detections: list[Detection],
    output1: np.ndarray,
    context: LetterboxContext,
    threshold: float,
) -> None:
    if not detections:
        return
    prototypes = np.asarray(output1)
    if prototypes.ndim == 4:
        prototypes = prototypes[0]
    if prototypes.shape != (32, 160, 160):
        raise ValueError(f"unexpected mask prototype shape: {prototypes.shape}")
    coefficients = np.stack([item.mask_coefficients for item in detections])
    logits = coefficients @ prototypes.reshape(32, -1)
    probabilities = 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))
    probability_maps = probabilities.reshape(-1, 160, 160)
    left = int(round(context.pad_x * 160 / 640))
    top = int(round(context.pad_y * 160 / 640))
    right = int(round((context.pad_x + context.resized_width) * 160 / 640))
    bottom = int(round((context.pad_y + context.resized_height) * 160 / 640))
    right = max(left + 1, min(160, right))
    bottom = max(top + 1, min(160, bottom))
    for detection, probability_map in zip(detections, probability_maps):
        unpadded = probability_map[top:bottom, left:right]
        restored = cv2.resize(
            unpadded,
            (context.original_width, context.original_height),
            interpolation=cv2.INTER_LINEAR,
        )
        x1, y1, x2, y2 = np.rint(detection.box_xyxy).astype(int)
        box = (slice(max(0, y1), y2 + 1), slice(max(0, x1), x2 + 1))
        cropped = np.zeros(restored.shape, bool)
        cropped[box] = restored[box] >= threshold  # the mask is kept inside the box only
        detection.mask = cropped


class DXNNDetector:
    def __init__(self, model_path, confidence=0.25, nms_iou=0.45, mask_threshold=0.5,
                 class_names=model_infer__CLASS_NAMES, selected_class_ids=SUSPECT_CLASS_IDS):
        import dx_engine  # the NPU runtime, needed only once analysis starts

        self.model_path = Path(model_path).expanduser().resolve()
        self.class_names = tuple(class_names)
        self.selected_class_ids = frozenset(selected_class_ids)
        if not self.selected_class_ids or not self.selected_class_ids <= set(range(len(self.class_names))):
            raise ValueError("Invalid model class filter")
        self.engines = []
        try:
            for core in (0, 1, 2):
                option = dx_engine.InferenceOption()
                option.bound_option = getattr(dx_engine.InferenceOption.BOUND_OPTION, f"NPU_{core}")
                self.engines.append(dx_engine.InferenceEngine(str(self.model_path), option))
        except Exception:
            for engine in self.engines:
                engine.dispose()
            raise
        self.engine = self.engines[0]
        # DXRT 3.3 keeps about 400 KB for every request made with a new input
        # array (none when one is reused), so requests take their input from
        # fixed buffers, one per request in flight.
        self.inputs = [np.empty((1, 640, 640, 3), dtype=np.uint8) for _ in range(3)]
        self.confidence = float(confidence)
        self.nms_iou = float(nms_iou)
        self.mask_threshold = float(mask_threshold)
        inputs = self.engine.get_input_tensors_info()
        if len(inputs) != 1 or tuple(inputs[0]["shape"]) != (1, 640, 640, 3):
            self.dispose()
            raise RuntimeError(f"unexpected DXNN inputs: {inputs}")
        if np.dtype(inputs[0]["dtype"]) != np.dtype(np.uint8):
            self.dispose()
            raise RuntimeError(f"DXNN input must be uint8: {inputs}")
        outputs = self.engine.get_output_tensors_info()
        channels = 4 + len(self.class_names) + MASK_COEFFICIENTS
        if (len(outputs) != 2 or tuple(outputs[0]["shape"]) not in
                ((1, channels, 8400), (1, 8400, channels)) or
                tuple(outputs[1]["shape"]) != (1, 32, 160, 160)):
            self.dispose()
            raise RuntimeError(f"Unexpected outputs for {self.model_path.name}: {outputs}")

    def decode(self, outputs, context):
        detections = decode_detections(outputs[0], context, self.confidence, self.nms_iou,
                                       self.class_names, self.selected_class_ids)
        attach_masks(detections, outputs[1], context, self.mask_threshold)
        return detections

    def iter_detect(self, records, load_image):
        """Keep at most three native requests in flight; yield in source order.

        Pending entries own input tensors until wait() completes. Refill before
        yielding, so NPU execution overlaps the caller's depth/metadata work.
        """
        source = iter(records)
        pending = deque()
        exhausted = False
        next_engine = 0
        submitted = 0

        def submit_one():
            nonlocal exhausted, next_engine, submitted
            try:
                record = next(source)
            except StopIteration:
                exhausted = True
                return
            image = load_image(record)
            # At most three requests are pending, so a buffer comes round
            # again only after its request has been waited for.
            tensor, context = letterbox_rgb_uint8(image, self.inputs[submitted % 3])
            submitted += 1
            engine = self.engines[next_engine]
            next_engine = (next_engine + 1) % len(self.engines)
            job_id = engine.run_async(tensor)
            pending.append((engine, job_id, record, image, tensor, context))

        try:
            while len(pending) < 3 and (not exhausted):
                submit_one()
            while pending:
                engine, job_id, record, image, tensor, context = pending[0]
                outputs = engine.wait(job_id)
                pending.popleft()
                detections = self.decode(outputs, context)
                if not exhausted:
                    submit_one()
                yield (record, image, detections)
        finally:
            first_error = None
            while pending:
                engine, job_id, record, image, tensor, context = pending.popleft()
                try:
                    engine.wait(job_id)
                except Exception as exc:
                    if first_error is None:
                        first_error = exc
            if first_error is not None:
                raise first_error

    def dispose(self):
        for engine in self.engines:
            if hasattr(engine, "dispose"):
                engine.dispose()


DEFAULTS = dict(
    max_missed_frames=5,
    association_iou=0.25,
    image_width=640,
    feature_count=320,
    min_motion_inliers=12,
    max_fb_error_px=1.5,
)


def bounds(row):
    values = [float(row[k]) for k in ("x1", "y1", "x2", "y2")]
    if (
        not all((math.isfinite(x) for x in values))
        or values[2] <= values[0]
        or values[3] <= values[1]
    ):
        raise ValueError("Invalid tracking box")
    return np.asarray(values, dtype=float)


def overlap(a, b):
    inter = max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0.0, min(a[3], b[3]) - max(a[1], b[1])
    )
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def assignment(weights):
    """Maximum weight one-to-one matching; zero edges are unmatched.

    Rectangular Hungarian algorithm with independent dummy columns. No extra
    SciPy dependency is required on the terminal.
    """
    n = len(weights)
    m0 = len(weights[0]) if n else 0
    if not n or not m0:
        return []
    m = m0 + n
    cost = [[-float(v) for v in row] + [0.0] * n for row in weights]
    u, v, p, way = ([0.0] * (n + 1), [0.0] * (m + 1), [0] * (m + 1), [0] * (m + 1))
    for i in range(1, n + 1):
        p[0], j0 = (i, 0)
        minimum, used = ([float("inf")] * (m + 1), [False] * (m + 1))
        while True:
            used[j0] = True
            i0, delta, j1 = (p[j0], float("inf"), 0)
            for j in range(1, m + 1):
                if not used[j]:
                    cur = cost[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minimum[j]:
                        minimum[j], way[j] = (cur, j0)
                    if minimum[j] < delta:
                        delta, j1 = (minimum[j], j)
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minimum[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
    return [(p[j] - 1, j - 1) for j in range(1, m0 + 1) if p[j] and weights[p[j] - 1][j - 1] > 0]


@dataclass
class Track:
    track_id: int
    class_id: int
    box: np.ndarray
    last_frame: int
    confirmed: bool = False
    confirmed_frame: int | None = None
    confirmed_depth_m: float | None = None
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(4))
    last_detection_box: np.ndarray | None = None


class ObjectTracker:
    def __init__(self):
        self.settings = DEFAULTS
        self.active = {}
        self.next_id = 1
        self.previous = None
        self.previous_points = None
        self.last_frame = None

    def _motion(self, image, needed):
        if not needed:
            # No track and no detection: no motion now, and none on the next frame either
            # (it would need this frame's features), so this frame is not even shrunk.
            self.previous = self.previous_points = None
            return None
        height, width = image.shape[:2]
        scale = self.settings["image_width"] / width
        small = cv2.resize(
            image,
            (self.settings["image_width"], round(height * scale)),
            interpolation=cv2.INTER_AREA,
        )
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
        matrix = None
        if (
            needed
            and self.previous is not None
            and (self.previous.shape == gray.shape)
            and (self.previous_points is not None)
        ):
            source = self.previous_points
            target, good, _ = cv2.calcOpticalFlowPyrLK(
                self.previous,
                gray,
                source,
                None,
                winSize=(31, 31),
                maxLevel=4,
                criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 0.01),
            )
            if target is not None:
                back, back_good, _ = cv2.calcOpticalFlowPyrLK(
                    gray, self.previous, target, None, winSize=(31, 31), maxLevel=4
                )
                if back is not None:
                    ok = good.ravel().astype(bool) & back_good.ravel().astype(bool)
                    ok &= (
                        np.linalg.norm(back - source, axis=2).ravel()
                        <= self.settings["max_fb_error_px"]
                    )
                    ok &= np.isfinite(target).all(axis=(1, 2))
                    a, b = (source[ok], target[ok])
                    if len(a) >= self.settings["min_motion_inliers"]:
                        cv2.setRNGSeed(71)
                        homography, inliers = cv2.findHomography(a, b, cv2.RANSAC, 2.5)
                        count = int(inliers.sum()) if inliers is not None else 0
                        selected = (
                            a[inliers.ravel().astype(bool)].reshape(-1, 2)
                            if inliers is not None
                            else np.empty((0, 2))
                        )
                        spread = np.ptp(selected, axis=0) if len(selected) else np.zeros(2)
                        if (
                            homography is not None
                            and np.isfinite(homography).all()
                            and (count >= self.settings["min_motion_inliers"])
                            and (count / len(a) >= 0.25)
                            and (spread[0] >= gray.shape[1] * 0.15)
                            and (spread[1] >= gray.shape[0] * 0.1)
                        ):
                            scaling = np.diag([scale, scale, 1.0])
                            matrix = np.linalg.inv(scaling) @ homography @ scaling
        mask = np.zeros(gray.shape, np.uint8)
        h, w = gray.shape
        mask[int(0.12 * h) : int(0.74 * h), int(0.12 * w) : int(0.9 * w)] = 255
        self.previous_points = (
            cv2.goodFeaturesToTrack(gray, self.settings["feature_count"], 0.01, 8, mask=mask)
            if needed
            else None
        )
        self.previous = gray
        return matrix

    @staticmethod
    def _warp(box, matrix):
        x1, y1, x2, y2 = box
        corners = np.float32([[[x1, y1], [x2, y1], [x2, y2], [x1, y2]]])
        warped = cv2.perspectiveTransform(corners, matrix)[0]
        result = np.r_[warped.min(axis=0), warped.max(axis=0)]
        if not np.isfinite(result).all() or (result[2:] <= result[:2]).any():
            return box.copy()
        ratio = np.prod(result[2:] - result[:2]) / np.prod(box[2:] - box[:2])
        return result if 0.1 <= ratio <= 10 else box.copy()

    def update(self, frame, image, detections, observations):
        """Return a row for every detection; no future data."""
        frame = int(frame)
        if self.last_frame is not None and frame <= self.last_frame:
            raise ValueError("New passage/replay requires a new tracker; frame IDs must increase")
        if len(detections) != len(observations):
            raise ValueError("Tracking observation count mismatch")
        parsed = [(int(d["class_id"]), bounds(d)) for d in detections]
        if any((cls not in (0, 1) for (cls, _) in parsed)):
            raise ValueError("Unsupported tracking class")
        for tid, track in list(self.active.items()):
            if frame - track.last_frame > self.settings["max_missed_frames"] + 1:
                del self.active[tid]
        matrix = self._motion(image, bool(self.active or detections))
        for track in self.active.values():
            track.box = (
                self._warp(track.box, matrix) if matrix is not None else track.box + track.velocity
            )
        tracks = list(self.active.values())
        weights = [
            [overlap(t.box, b) if t.class_id == cls else 0.0 for (cls, b) in parsed] for t in tracks
        ]
        weights = [
            [v if v >= self.settings["association_iou"] else 0.0 for v in row] for row in weights
        ]
        matched = {j: (tracks[i], weights[i][j]) for (i, j) in assignment(weights)}
        rows = []
        for j, (cls, box) in enumerate(parsed):
            if j in matched:
                track, score = matched[j]
                gap = frame - track.last_frame
                track.velocity = (box - track.last_detection_box) / gap
                track.box = box.copy()
                track.last_frame = frame
            else:
                track = Track(self.next_id, cls, box.copy(), frame)
                self.next_id += 1
                self.active[track.track_id] = track
                score, gap = (None, 0)
            track.last_detection_box = box.copy()
            was_confirmed = track.confirmed
            observation = observations[j]
            depth = observation.get("max_depth_m")
            reason = observation.get("reason")
            valid_depth = (
                cls == 1
                and depth is not None
                and math.isfinite(float(depth))
                and (reason in ("depth_pass", "depth_below_threshold"))
            )
            if bool(observation.get("accepted")):
                if cls == 1 and (
                    not valid_depth
                    or reason != "depth_pass"
                    or float(depth) < float(observation["threshold_depth_m"])
                    or (int(observation.get("mask_points", 0)) <= 0)
                ):
                    raise ValueError("Cannot latch confirmation without valid LiDAR evidence")
                if not track.confirmed:
                    track.confirmed = True
                    track.confirmed_frame = frame
                    track.confirmed_depth_m = float(depth) if cls == 1 else None
            rows.append(
                dict(
                    frame_index=frame,
                    object_id=j,
                    class_id=cls,
                    **dict(zip(("x1", "y1", "x2", "y2"), map(float, box))),
                    track_id=track.track_id,
                    association_iou=score,
                    gap_frames=gap,
                    new_track=gap == 0,
                    confirmed=track.confirmed,
                    newly_confirmed=track.confirmed and (not was_confirmed),
                    confirmed_frame=track.confirmed_frame,
                    confirmed_depth_m=track.confirmed_depth_m,
                    observation_state=(
                        "confirmed"
                        if observation and observation.get("accepted")
                        else "observed_below_threshold" if valid_depth else "unobserved"
                    ),
                    state="confirmed" if track.confirmed else "pending",
                    valid_depth_observation=valid_depth,
                )
            )
        self.last_frame = frame
        return rows


KST = ZoneInfo("Asia/Seoul")

ALLOWED_DAMAGE_TYPES = {"road_damage", "pothole", "linear_crack"}

DAMAGE_TYPE_KO = {"pothole": "포트홀", "linear_crack": "크랙", "road_damage": "러팅"}


def _load_local_env() -> None:
    """Load a sibling .env without overriding explicitly set environment values.

    As in a shell, a later assignment in the file wins over an earlier one.
    """
    env_path = Path(__file__).with_name(".env")
    if not env_path.is_file():
        return
    values = {}
    for raw_line in env_path.read_text(encoding="utf-8-sig").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        if not re.fullmatch("[A-Za-z_][A-Za-z0-9_]*", key):
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and (value[0] in "\"'"):
            value = value[1:-1]
        values[key] = value
    for key, value in values.items():
        os.environ.setdefault(key, value)


_load_local_env()


def camera_time_kst(timestamp: float) -> datetime:
    """Interpret dataset timestamps as Unix seconds and return aware KST."""
    return datetime.fromtimestamp(float(timestamp), timezone.utc).astimezone(KST)


def _finite_float(value) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def build_damage_payload(
    *,
    damage_type: str,
    camera_timestamp: float,
    lidar_timestamp: float,
    time_error_sec: float,
    cluster: dict,
    bbox: tuple[int, int, int, int],
    polygons_px: list[list[dict]],
    gps: dict,
    vehicle_type: str,
    sensor_mount_height_cm: float,
    sensor_mount_angles_xyz_deg: dict,
) -> dict:
    damage_type = str(damage_type).strip().lower()
    if damage_type not in ALLOWED_DAMAGE_TYPES:
        raise ValueError(f"Unsupported damage type: {damage_type}")
    camera_dt = camera_time_kst(camera_timestamp)
    lidar_dt = camera_time_kst(lidar_timestamp)
    x0, y0, x1, y1 = (int(value) for value in bbox)
    # No weather service is configured on the terminals.
    compact_weather = {
        "source": "KMA_DATA_GO_KR_ULTRA_SHORT_NOWCAST",
        "status": "unavailable",
        "condition": None,
        "temperature_c": None,
        "humidity_pct": None,
        "reason": "No KMA weather API is configured",
    }
    return {
        "type": DAMAGE_TYPE_KO[damage_type],
        "vehicle": str(vehicle_type),
        "size": {
            "length_m": _finite_float(cluster.get("length_m")),
            "width_m": _finite_float(cluster.get("width_m")),
            "area_m2": _finite_float(cluster.get("hull_area_m2")),
        },
        "depth": {
            "median_cm": _finite_float(cluster.get("median_depth_cm")),
            "p90_cm": _finite_float(cluster.get("p90_depth_cm")),
            "max_cm": _finite_float(cluster.get("max_depth_cm")),
        },
        "weather": compact_weather,
        "sensor_mount_height_cm": float(sensor_mount_height_cm),
        "sensor_mount_angle_xyz_deg": {
            "x": _finite_float(sensor_mount_angles_xyz_deg.get("x")),
            "y": _finite_float(sensor_mount_angles_xyz_deg.get("y")),
            "z": _finite_float(sensor_mount_angles_xyz_deg.get("z")),
        },
        "bbox_px": {
            "upper_left": {"x": x0, "y": y0},
            "lower_right": {"x": x1, "y": y1},
            "width": x1 - x0 + 1,
            "height": y1 - y0 + 1,
        },
        "segmentation_px": polygons_px,
        "gps": {
            "latitude_deg": _finite_float(gps.get("latitude_deg")),
            "longitude_deg": _finite_float(gps.get("longitude_deg")),
            "altitude_m": _finite_float(gps.get("altitude_m")),
            "speed_kmh": _finite_float(gps.get("speed_kmh")),
            "course_deg": _finite_float(gps.get("course_deg")),
            "fix_quality": gps.get("fix_quality"),
            "satellites": gps.get("satellites"),
            "hdop": _finite_float(gps.get("hdop")),
            "time_error_ms": _finite_float(gps.get("time_error_ms")),
        },
        "time": {
            "camera_epoch_sec": float(camera_timestamp),
            "camera_kst": camera_dt.isoformat(timespec="milliseconds"),
            "lidar_epoch_sec": float(lidar_timestamp),
            "lidar_kst": lidar_dt.isoformat(timespec="milliseconds"),
            "camera_lidar_error_ms": float(time_error_sec) * 1000.0,
        },
    }


VEHICLE_TYPE = os.environ.get("DAMAGE_EXPORT_VEHICLE_TYPE", "현대 팰리세이드")

SENSOR_MOUNT_HEIGHT_CM = 190.0

SENSOR_MOUNT_ANGLE_XYZ_DEG = dict(x=42.2415070919, y=3.1563042048, z=-173.626590518)


def damage_geometry(road_xyz, include_depth):
    road = np.asarray(road_xyz, dtype=float).reshape(-1, 3)
    road = road[np.isfinite(road).all(axis=1)]
    geometry = {}
    if len(road) >= 3:
        xy = road[:, :2]
        _, _, axes = np.linalg.svd(xy - xy.mean(axis=0), full_matrices=False)
        projected = (xy - xy.mean(axis=0)) @ axes.T
        spans = np.percentile(projected, 95, axis=0) - np.percentile(projected, 5, axis=0)
        geometry.update(
            length_m=float(max(spans)),
            width_m=float(min(spans)),
            hull_area_m2=float(cv2.contourArea(cv2.convexHull(xy.astype(np.float32)))),
        )
    if include_depth and len(road):
        depths = -road[:, 2] * 100.0
        geometry.update(
            median_depth_cm=float(np.median(depths)),
            p90_depth_cm=float(np.percentile(depths, 90)),
            max_depth_cm=float(depths.max()),
        )
    return geometry


def mask_polygons(mask):
    if mask is None:
        return []
    contours, _ = cv2.findContours(
        np.asarray(mask, dtype=np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    return [
        [dict(x=int(x), y=int(y)) for (x, y) in contour.reshape(-1, 2)]
        for contour in contours
        if len(contour) >= 3
    ]


class DamageArtifactExporter:
    """Per-damage records of one frame; report() points directory at the frame's upload folder."""

    def __init__(self, camera):
        self.directory = None
        self.gps_streams = {}
        self.camera_extrinsics = camera.extrinsics
        self.calibration_ids = {
            "camera": {
                "name": camera.path.name,
                "sha256": sha(camera.path),
            },
            "lidar": {
                "name": Path(LIDAR_CALIBRATION).name,
                "sha256": sha(LIDAR_CALIBRATION),
            },
        }

    def save(
        self,
        timestamp,
        detections,
        fused,
        point_count,
        road_xyz,
        lidar_timestamp,
        payloads,
        scan_start=None,
        scan_end=None,
        target_lidar_timestamp=None,
        camera_timing=None,
        motion_compensation=None,
    ):
        """Append one damage record per detection to payloads."""
        if len(detections) != len(fused):
            raise ValueError("Detection/fusion count mismatch")
        if not detections:
            return
        lidar_metadata = dict(
            schema="pandar_xt32_pcap_v1",
            pcap_files=[],
            status="pcap_unavailable",  # report() attaches the scan PCAP
            decoded_on_terminal=lidar_timestamp is not None,
            scan_start_epoch_sec=None if scan_start is None else scan_start - 1e-6,
            scan_end_epoch_sec=None if scan_end is None else scan_end + 1e-6,
            target_epoch_sec=target_lidar_timestamp,
            camera_lidar_offset_sec=OFFSET_SEC,
            terminal_scan_point_count=point_count,
            return_selection=lidar__RETURN_SELECTION,
            channels=sorted(lidar___DETECTION_CHANNEL_SET),
            apply_firetime=lidar__APPLY_FIRETIME,
            coordinate_correction=lidar__COORDINATE_CORRECTION,
            calibration_files=self.calibration_ids,
            camera_extrinsics=self.camera_extrinsics,
            camera_timing=camera_timing,
            motion_compensation=motion_compensation,
            pcap_copy_policy="selected_source_file_bytes_unchanged",
        )
        # GPS at the exposure moment, the time the LiDAR is aligned to.
        gps = lidar__gps_values_for_camera(
            self.gps_streams, timestamp if target_lidar_timestamp is None else target_lidar_timestamp)
        for detection, obj in zip(detections, fused):
            pothole = int(detection.class_id) == 1
            if pothole and (not obj.get("validation", {}).get("accepted")):
                raise ValueError("Unverified pothole cannot be exported")
            damage_type = "pothole" if pothole else "linear_crack"
            indices = np.asarray(
                obj.get("verified_point_indices" if pothole else "point_indices", []),
                dtype=int,
            )
            if pothole:
                object_road = obj.get("verified_road_xyz_m", np.empty((0, 3)))
            else:
                object_road = (
                    np.empty((0, 3)) if road_xyz is None else np.asarray(road_xyz)[indices]
                )
            geometry = (
                dict(obj["measurement_record"])
                if pothole
                else damage_geometry(object_road, include_depth=False)
            )
            payload = build_damage_payload(
                damage_type=damage_type,
                camera_timestamp=timestamp,
                lidar_timestamp=(timestamp if lidar_timestamp is None else lidar_timestamp),
                time_error_sec=(0.0 if lidar_timestamp is None else lidar_timestamp - timestamp),
                cluster=geometry,
                bbox=tuple(np.rint(detection.box_xyxy).astype(int)),
                polygons_px=mask_polygons(detection.mask),
                gps=gps,
                vehicle_type=VEHICLE_TYPE,
                sensor_mount_height_cm=SENSOR_MOUNT_HEIGHT_CM,
                sensor_mount_angles_xyz_deg=SENSOR_MOUNT_ANGLE_XYZ_DEG,
            )
            payload["lidar"] = dict(lidar_metadata, object_point_count=len(indices))
            if lidar_timestamp is None:
                payload["time"].update(
                    lidar_epoch_sec=None, lidar_kst=None, camera_lidar_error_ms=None
                )
            if obj.get("tracking"):
                payload["tracking"] = {
                    **obj["tracking"],
                    "scope": "one_passage_per_replay_loop",
                    "report_policy": "first_confirmation_only",
                }
            payloads.append(payload)


def matched_gps_position(streams, timestamp):
    """Position at timestamp between the valid NMEA fixes around it (RMC, else GGA);
    latitude/longitude are None without one."""
    for kind, valid in (("RMC", gps_rmc_valid), ("GGA", gps_gga_valid)):
        position = gps_interpolated(streams.get(kind), timestamp, valid,
                                    ("latitude_deg", "longitude_deg"))
        if position is not None:
            return position
    return dict(latitude_deg=None, longitude_deg=None)


def terminal_id():
    """Terminal name used in record_id (PORTHOLE_TERMINAL_ID, else the host name)."""
    return os.getenv("PORTHOLE_TERMINAL_ID", "").strip() or socket.gethostname()


def save_detection_frame(directory, source_image, image, row, key,
                         damage_detections, damage_payloads, gps, pcap_files):
    """Frame JSON in the terminal-server data spec (단말기-서버 데이터 명세서, 2026-09-28).

    Only record_id, categories, images, annotations, gps and lidar.pcap_files are
    sent; missing measurements are null, not 0. gps.speed_mps (added to the spec on
    2026-10-02) is the GPS speed motion compensation used, so a viewer can repeat it.
    """
    if not damage_detections:
        return []
    if len(damage_detections) != len(damage_payloads):
        raise ValueError("Damage annotation/payload mismatch")
    if any(int(d.class_id) not in SUSPECT_CLASS_IDS for d in damage_detections):
        raise ValueError("Unsupported category in damage frame export")
    height, width = image.shape[:2]
    index = int(row["frame_index"])
    timestamp_ns = int(row["timestamp_ns"])
    stem = detection_frame_stem(row)
    image_path, json_path = directory / (stem + ".jpg"), directory / (stem + ".json")
    if image_path.exists() or json_path.exists():
        raise FileExistsError(stem)
    source_image = Path(source_image)
    if source_image.suffix.lower() in (".jpg", ".jpeg"):
        shutil.copyfile(source_image, image_path)
    elif not cv2.imwrite(str(image_path), image):
        raise IOError(f"Cannot save source image: {source_image}")
    annotations = []
    for d, old in zip(damage_detections, damage_payloads):
        x1, y1, x2, y2 = map(float, d.box_xyxy)
        polygons = [[c for point in polygon for c in (point["x"], point["y"])]
                    for polygon in mask_polygons(d.mask)]
        annotations.append(dict(
            id=len(annotations) + 1, image_id=1, category_id=int(d.class_id),
            confidence=float(d.confidence),
            bbox=[round(x1), round(y1), round(x2 - x1), round(y2 - y1)],
            segmentation=polygons,
            measurements=dict(
                size=dict(length_m=old["size"]["length_m"], width_m=old["size"]["width_m"],
                          area_m2=old["size"]["area_m2"]),
                depth=dict(median_cm=old["depth"]["median_cm"]),
            ),
        ))
    captured = datetime.fromtimestamp(timestamp_ns // 1_000_000_000, timezone.utc)
    payload = dict(
        record_id=f"{terminal_id()}/{Path(key).name}/{index}",
        categories=[dict(id=i, name=name) for i, name in enumerate(model_infer__CLASS_NAMES)],
        images=[dict(id=1, width=width, height=height, file_name=image_path.name,
                     date_captured=captured.strftime("%Y-%m-%dT%H:%M:%SZ"))],
        annotations=annotations,
        gps=dict(latitude_deg=gps.get("latitude_deg"), longitude_deg=gps.get("longitude_deg"),
                 speed_mps=gps.get("speed_mps")),
        lidar=dict(pcap_files=[dict(name=f["name"]) for f in pcap_files]),
    )
    # Indented, so the file reads well when opened on the server.
    atomic_json(json_path, clean(payload), indent=2)
    return [image_path, json_path]


def detection_frame_stem(row):
    """frame_<KST date>_<time>_<ms> of the camera time, e.g. frame_20261002_101345_123.

    The same moment as the CSV time; frames are 33 ms apart, so the milliseconds keep the
    names of one run unique. The frame number is in the JSON record_id.
    """
    ns = int(row["timestamp_ns"])
    moment = datetime.fromtimestamp(ns // 1_000_000_000, timezone.utc).astimezone(KST)
    return f"frame_{moment:%Y%m%d_%H%M%S}_{ns // 1_000_000 % 1000:03d}"


def save_scan_pcap(destination, sources, start, end):
    """Select scan packets, preserving raw record headers/timestamps/payloads.

    A scan crossing file rotation is concatenated into one classic PCAP.
    Never generate an empty substitute when LiDAR evidence is unavailable.
    """
    sources = list(dict.fromkeys(Path(p).resolve() for p in sources))
    if not sources or start is None or end is None:
        return None
    if not math.isfinite(start) or not math.isfinite(end) or end < start:
        raise ValueError("Invalid scan interval for PCAP export")
    destination = Path(destination)
    if destination.exists():
        raise FileExistsError(destination)
    temporary = destination.with_suffix(".pcap.partial")
    formats = {
        b"\xd4\xc3\xb2\xa1": ("<", 1000000),
        b"\xa1\xb2\xc3\xd4": (">", 1000000),
        b"M<\xb2\xa1": ("<", 1000000000),
        b"\xa1\xb2<M": (">", 1000000000),
    }
    count, header, first, last, previous = 0, None, None, None, None
    signatures = [(p.stat().st_size, p.stat().st_mtime_ns, p.stat().st_ino) for p in sources]
    try:
        with temporary.open("xb") as output:
            for path, signature in zip(sources, signatures):
                with path.open("rb") as source:
                    current = source.read(24)
                    if len(current) != 24 or current[:4] not in formats:
                        raise ValueError("Invalid classic PCAP header")
                    endian, divisor = formats[current[:4]]
                    if struct.unpack(endian + "IHHIIII", current)[-1] != 1:
                        raise ValueError("Only Ethernet PCAP is supported")
                    if header is not None and current != header:
                        raise ValueError("Incompatible PCAP headers across scan sources")
                    if header is None:
                        header = current
                        output.write(header)
                    while True:
                        record = source.read(16)
                        if not record:
                            break
                        if len(record) != 16:
                            raise ValueError("Truncated PCAP record header")
                        seconds, fraction, size, _ = struct.unpack(endian + "IIII", record)
                        if size > signature[0] - source.tell():
                            raise ValueError("Truncated PCAP packet")
                        timestamp = seconds + fraction / divisor
                        if not start <= timestamp <= end:
                            source.seek(size, 1)
                            continue
                        payload = source.read(size)
                        if len(payload) != size:
                            raise ValueError("Truncated PCAP packet")
                        if len(sources) > 1 and previous is not None and timestamp < previous:
                            raise ValueError("PCAP scan sources are not chronologically ordered")
                        previous = timestamp
                        first = timestamp if first is None else min(first, timestamp)
                        last = timestamp if last is None else max(last, timestamp)
                        output.write(record)
                        output.write(payload)
                        count += 1
            output.flush()
            os.fsync(output.fileno())
        for path, signature in zip(sources, signatures):
            stat = path.stat()
            if (stat.st_size, stat.st_mtime_ns, stat.st_ino) != signature:
                raise ValueError("PCAP source changed during export")
        if not count:
            return None
        temporary.replace(destination)
        return dict(name=destination.name, sha256=sha(destination),
                    bytes=destination.stat().st_size, packet_count=count,
                    first_timestamp=first, last_timestamp=last,
                    source_names=[p.name for p in sources],
                    copy_policy="selected_scan_raw_records_unchanged")
    finally:
        temporary.unlink(missing_ok=True)


@dataclass(frozen=True)
class RoadPlaneSettings:
    # Road normal in LiDAR coordinates as mounted on the car: the mean of 793 moving scans
    # of the 2026-10-02 drive (all within 5.4 deg, median 0.5 deg). The old value
    # (-0.016, 0.666, 0.746) was 11.4 deg off, leaving PLANE_MAX_TILT_DEG little room on
    # slopes. Measure it again if the sensor mount angle changes.
    expected_normal: tuple = (-0.012, 0.8, 0.6)
    cell_size_m: float = 0.35


def prior_rotation(settings):
    n = np.asarray(settings.expected_normal, float)
    if n.shape != (3,) or not np.isfinite(n).all() or np.linalg.norm(n) < 1e-09:
        raise ValueError("Invalid expected road normal")
    n = n / np.linalg.norm(n)
    x = np.array([1.0, 0.0, 0.0])
    x -= x @ n * n
    if np.linalg.norm(x) < 1e-09:
        raise ValueError("Road normal parallel to sensor X")
    x /= np.linalg.norm(x)
    return np.array([x, np.cross(n, x), n])


def fit_spatial_road_plane(xyz, selected=None, settings=RoadPlaneSettings()):
    """Road plane (unit normal, offset, quality) through the median points of a grid on
    the selected points (plane_points), or on every point when they cannot carry a plane.

    quality: cells (grid cells that voted), support (share of cells within
    PLANE_SUPPORT_M of the plane) and tilt_deg (angle from settings.expected_normal).
    """
    xyz = np.asarray(xyz, float)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("XYZ/channel shape mismatch")
    R = prior_rotation(settings)
    local = xyz @ R.T
    finite = np.isfinite(local).all(axis=1)
    points = local[finite if selected is None else finite & selected]
    if len(points) < 3 or np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
        points = local[finite]
    if len(points) < 3 or np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
        raise ValueError("plane_unavailable: fewer than three non-collinear finite points")
    keys = np.floor(points[:, :2] / settings.cell_size_m).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    cells = []
    for i in range(int(inverse.max()) + 1):
        group = points[inverse == i]
        cells.append(np.median(group, axis=0))
    cells = np.asarray(cells, float).reshape(-1, 3)
    grid_cells = len(cells)
    if len(cells) < 3 or np.linalg.matrix_rank(cells - cells.mean(axis=0)) < 2:
        cells, grid_cells = points.copy(), 0  # no spread of cells: the checks withhold this plane
    A = np.column_stack([cells[:, :2], np.ones(len(cells))])
    target = cells[:, 2]
    rng = np.random.default_rng(20260907)
    triples = rng.integers(0, len(cells), size=(768, 3))
    matrices = A[triples]
    valid = abs(np.linalg.det(matrices)) > 0.02
    coef = np.linalg.solve(matrices[valid], target[triples[valid]][..., None])[..., 0]
    coef = np.vstack([coef, np.linalg.lstsq(A, target, rcond=None)[0]])
    residual = np.abs(A @ coef.T - target[:, None]) / np.sqrt(1 + (coef[:, :2] ** 2).sum(axis=1))
    sorted_residual = np.sort(residual, axis=0)
    loss = np.mean(sorted_residual[: max(3, int(0.7 * len(cells)))], axis=0) + 0.25 * np.quantile(
        residual, 0.75, axis=0
    )
    beta = coef[int(np.argmin(loss))]
    for _ in range(8):
        delta = A @ beta - target
        weights = np.minimum(1.0, 0.015 / np.maximum(abs(delta), 1e-12))
        root = np.sqrt(weights)
        new = np.linalg.lstsq(A * root[:, None], target * root, rcond=None)[0]
        if np.linalg.norm(new - beta) < 1e-08:
            beta = new
            break
        beta = new
    scale = np.sqrt(1 + beta[:2] @ beta[:2])
    normal = np.array([-beta[0], -beta[1], 1.0]) @ R / scale
    offset = -float(beta[2]) / scale
    quality = dict(
        cells=grid_cells,
        support=float(np.mean(np.abs(A @ beta - target) / scale <= PLANE_SUPPORT_M)),
        tilt_deg=float(np.degrees(np.arctan(np.sqrt(beta[:2] @ beta[:2])))),
    )
    return (normal, offset, quality)


ROAD_NORMAL = np.asarray(RoadPlaneSettings().expected_normal, float)
ROAD_NORMAL = ROAD_NORMAL / np.linalg.norm(ROAD_NORMAL)


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
    """Depth below its own road (see POTHOLE_DEPTH_M) of the area points of every LiDAR line
    crossing the area. inside: the area's points; road_ok: points that may serve as road
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
        deep = depth >= POTHOLE_DEPTH_M
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


def is_pothole(lines, min_run=POTHOLE_MIN_RUN):
    return sum(1 for line in lines if line["measured"] and line["run"] >= min_run) >= POTHOLE_MIN_LINES


class PotholeScan:
    """The LiDAR side of one frame for the pothole check: the decoded scan moved to the image
    time along the road (heights stay as measured) and projected into the image, and the points
    that may serve as road (visible in the image, outside every model area)."""

    def __init__(self, points, masks, camera, alignment, target, shape):
        self.camera, self.alignment, self.target, self.masks = camera, alignment, target, masks
        self.place(points, shape)

    def place(self, points, shape):
        """Move the decoded scan to the image time along the road (heights stay as measured), work
        out its geometry and project it into the image."""
        self.points = points
        road, rotation, _ = lidar__road_aligned_xyz(points.xyz, ROAD_NORMAL, 0.0)
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
        self.shape, self._union = shape, None
        covered = np.zeros(len(self.uv), bool)  # in any model area, looked up at the points only
        for mask in self.masks:
            covered |= mask[self.uv[:, 1], self.uv[:, 0]]
        self.road_ok = self.visible & ~covered

    @property
    def union(self):
        """Every model area of the frame as one image mask (pothole_check.py)."""
        if self._union is None:
            self._union = np.logical_or.reduce(self.masks) if self.masks else np.zeros(self.shape, bool)
        return self._union

    def inside(self, mask):
        return self.visible & mask[self.uv[:, 1], self.uv[:, 0]]

    def lines(self, inside, road_ok=None, detail=False):
        return line_depths(self.geometry, self.points.channel, self.visible, inside,
                           self.road_ok if road_ok is None else road_ok, detail)


def pothole_evidence(lines, inside):
    """Audit fields of one model pothole (lines: line_depths with detail)."""
    used = [line for line in lines if line["measured"]]
    keep = is_pothole(lines)
    if keep:
        reason = "depth_pass"
    elif not inside.any():
        reason = "no_mask_points"
    elif len(used) < POTHOLE_MIN_LINES:
        reason = "local_reference_unavailable"  # the road beside could not be measured
    else:
        reason = "depth_below_threshold"
    local = dict(rule="line_road", depth_m=POTHOLE_DEPTH_M, min_lines=POTHOLE_MIN_LINES,
                 min_run=POTHOLE_MIN_RUN, usable_lines=len(used),
                 deep_lines=sum(line["run"] >= POTHOLE_MIN_RUN for line in used),
                 lines=[{name: line[name] for name in ("channel", "points", "measured", "why", "run", "deep",
                                                       "max_depth_m", "median_depth_m")} for line in lines])
    return dict(method="model_mask_line_road_depth", accepted=keep, reason=reason,
                max_depth_m=max((line["max_depth_m"] for line in used), default=None),
                threshold_depth_m=POTHOLE_DEPTH_M, mask_points=int(inside.sum()),
                local_evidence_json=json.dumps(local),
                validation_state="accepted" if keep else "unobserved" if reason == "no_mask_points" else "withheld")


lidar__DETECTION_CHANNEL_IDS = tuple(range(1, 33))

lidar___DETECTION_CHANNEL_SET = frozenset(lidar__DETECTION_CHANNEL_IDS)

lidar__GPS_MAX_TIME_ERROR_SEC = 1.5

lidar__RETURN_SELECTION = "last"

lidar__APPLY_FIRETIME = True

lidar__COORDINATE_CORRECTION = True

lidar__PCAP_GLOBAL_LEN = 24

lidar__PCAP_RECORD_LEN = 16

lidar__XT32_PAYLOAD_LEN = 1080

lidar__XT32_BODY_OFFSET = 12

lidar__XT32_BLOCK_LEN = 130

lidar__XT32_BLOCKS = 8

lidar__XT32_CHANNELS = 32

lidar__XT32_TAIL_OFFSET = 1052

lidar__XT32_LASER_FIRETIME_US = [1.512 * i + 6.0 for i in range(32)]  # Hesai XT firetime correction file: 6.0 .. 52.872 us

lidar__XT_COORD_H_M = 0.0315

lidar__XT_COORD_B_M = 0.013


@dataclass(frozen=True)
class lidar__Calibration:
    elevation_deg: list[float]
    azimuth_offset_deg: list[float]


@dataclass(frozen=True)
class lidar__PcapRecord:
    packet_index: int
    timestamp: float
    frame: bytes


def lidar__load_calibration(path: Path) -> lidar__Calibration:
    path = path.expanduser().resolve()
    elevations: list[Optional[float]] = [None] * lidar__XT32_CHANNELS
    azimuths: list[Optional[float]] = [None] * lidar__XT32_CHANNELS
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.reader(f))
    parsed = 0
    for row in rows[1:]:
        if len(row) < 3:
            continue
        try:
            channel = int(row[0].strip())
            elevation = float(row[1].strip())
            azimuth = float(row[2].strip())
        except ValueError:
            continue
        if 1 <= channel <= lidar__XT32_CHANNELS:
            elevations[channel - 1] = elevation
            azimuths[channel - 1] = azimuth
            parsed += 1
    if (
        parsed != lidar__XT32_CHANNELS
        or any((v is None for v in elevations))
        or any((v is None for v in azimuths))
    ):
        raise ValueError(f"Expected calibration for channels 1..32 in {path}; parsed {parsed} rows")
    return lidar__Calibration(
        elevation_deg=[float(v) for v in elevations],
        azimuth_offset_deg=[float(v) for v in azimuths],
    )


def lidar__iter_pcap(path: Path) -> Iterator[lidar__PcapRecord]:
    with path.open("rb") as f:
        gh = f.read(lidar__PCAP_GLOBAL_LEN)
        if len(gh) != lidar__PCAP_GLOBAL_LEN:
            raise ValueError("PCAP global header is missing/truncated")
        magic = gh[:4]
        if magic == b"\xd4\xc3\xb2\xa1":
            endian, ts_div = ("<", 1000000.0)
        elif magic == b"\xa1\xb2\xc3\xd4":
            endian, ts_div = (">", 1000000.0)
        elif magic == b"M<\xb2\xa1":
            endian, ts_div = ("<", 1000000000.0)
        elif magic == b"\xa1\xb2<M":
            endian, ts_div = (">", 1000000000.0)
        else:
            raise ValueError(f"Unsupported classic-PCAP magic: {magic.hex()}")
        _, _, _, _, _, _, network = struct.unpack(endian + "IHHIIII", gh)
        if network != 1:
            raise ValueError(f"Only Ethernet PCAP is supported (DLT={network})")
        packet_index = 0
        while True:
            rh = f.read(lidar__PCAP_RECORD_LEN)
            if not rh:
                break
            if len(rh) != lidar__PCAP_RECORD_LEN:
                raise ValueError("Truncated PCAP record header")
            ts_sec, ts_frac, incl_len, _ = struct.unpack(endian + "IIII", rh)
            frame = f.read(incl_len)
            if len(frame) != incl_len:
                raise ValueError("Truncated PCAP frame")
            yield lidar__PcapRecord(
                packet_index=packet_index,
                timestamp=ts_sec + ts_frac / ts_div,
                frame=frame,
            )
            packet_index += 1


def lidar__decode_udp(frame: bytes) -> bytes:
    """UDP payload of an Ethernet/IPv4 frame."""
    if len(frame) < 42:
        raise ValueError("Ethernet frame is too short")
    if struct.unpack("!H", frame[12:14])[0] != 2048:
        raise ValueError("Ethernet payload is not IPv4")
    ip_start = 14
    ihl = (frame[ip_start] & 15) * 4
    if ihl < 20 or frame[ip_start + 9] != 17:
        raise ValueError("IPv4 payload is not valid UDP")
    udp_start = ip_start + ihl
    _, _, udp_len, _ = struct.unpack("!HHHH", frame[udp_start : udp_start + 8])
    return frame[udp_start + 8 : udp_start + udp_len]


def lidar__return_label(mode: int, block_index: int) -> str:
    pair_pos = block_index % 2
    if mode == 59:
        return "last" if pair_pos == 0 else "first"
    if mode == 57:
        return "last" if pair_pos == 0 else "strongest"
    if mode == 60:
        return "strongest" if pair_pos == 0 else "first"
    if mode == 56:
        return "last"
    if mode == 55:
        return "strongest"
    return f"0x{mode:02X}"


@lru_cache(maxsize=1)
def _azimuth_table():
    """math.sin/cos of every 0.01-degree azimuth an XT32 point can have."""
    radians = [math.radians(k / 100.0) for k in range(36000)]
    return (np.array([math.sin(a) for a in radians]), np.array([math.cos(a) for a in radians]))


@lru_cache(maxsize=256)
def _channel_geometry(elevation_deg, azimuth_offset_deg, height, baseline):
    el = math.radians(elevation_deg)
    abs_off = math.radians(abs(azimuth_offset_deg))
    cos_el = math.cos(el)
    return (
        cos_el,
        math.sin(el),
        height * math.cos(abs_off) * cos_el - baseline * math.sin(abs_off) * cos_el,
    )


@dataclass(frozen=True)
class ScanPoints:
    """Decoded points of one scan, in packet, block, channel order."""

    timestamp: np.ndarray  # PCAP record time of the packet
    channel: np.ndarray  # 1-based laser channel
    xyz: np.ndarray  # sensor coordinates in metres, shape (n, 3)
    firing: np.ndarray  # index of the point's firing in firing_azimuth_raw
    firing_azimuth_raw: np.ndarray  # raw azimuth (0.01 degree) of every firing, scan order

    def __len__(self):
        return len(self.channel)


def decode_xt32_records(records, calibration: lidar__Calibration) -> ScanPoints:
    """Decode XT32 packets into points, all firings of all packets at once.

    Filters, operation order and sin/cos values follow the per-point formulas
    (azimuth: offset + raw + fire time, truncated to 0.01 degree; coordinates
    corrected for the sensor's optical centre), so every value is bit-identical
    to computing them one point at a time.
    """
    payloads, times = [], []
    for rec in records:
        payload = lidar__decode_udp(rec.frame)
        if len(payload) != lidar__XT32_PAYLOAD_LEN or payload[:4] != b"\xee\xff\x06\x01":
            continue
        if payload[6] != 32 or payload[7] != 8:
            raise ValueError(
                f"Packet {rec.packet_index}: expected XT32 32/8, got {payload[6]}/{payload[7]}"
            )
        payloads.append(payload)
        times.append(rec.timestamp)
    count = len(payloads)
    data = np.frombuffer(b"".join(payloads), dtype=np.uint8)
    data = data.reshape(count, lidar__XT32_PAYLOAD_LEN).astype(np.int64)
    tail = data[:, lidar__XT32_TAIL_OFFSET:]
    return_mode = tail[:, 10]
    spin_speed_rpm = tail[:, 11] | tail[:, 12] << 8
    blocks =data[:, lidar__XT32_BODY_OFFSET : lidar__XT32_TAIL_OFFSET].reshape(
        count, lidar__XT32_BLOCKS, lidar__XT32_BLOCK_LEN
    )
    raw_az100 = blocks[:, :, 0] | blocks[:, :, 1] << 8
    lasers = blocks[:, :, 2:].reshape(count, lidar__XT32_BLOCKS, lidar__XT32_CHANNELS, 4)
    distance_m = (lasers[..., 0] | lasers[..., 1] << 8) * data[:, 9, None, None] / 1000.0

    selected = {
        mode: [lidar__return_label(int(mode), b) == lidar__RETURN_SELECTION
               for b in range(lidar__XT32_BLOCKS)]
        for mode in set(return_mode.tolist())
    }
    block_ok = np.array([selected[mode] for mode in return_mode.tolist()], dtype=bool)
    channel_ok = np.array([c + 1 in lidar___DETECTION_CHANNEL_SET
                           for c in range(lidar__XT32_CHANNELS)])
    keep = (block_ok.reshape(count, lidar__XT32_BLOCKS, 1) & channel_ok
            & (distance_m > 0.1) & (distance_m <= 200.0))

    offset_deg = np.asarray(calibration.azimuth_offset_deg, dtype=np.float64)
    # Laser fire times (lidar__APPLY_FIRETIME) and the optical-centre correction
    # (lidar__COORDINATE_CORRECTION) are always applied.
    firetime_deg = spin_speed_rpm[:, None] * np.asarray(lidar__XT32_LASER_FIRETIME_US) * 6e-06
    az100 = (offset_deg * 100.0 + raw_az100[:, :, None] + (firetime_deg * 100.0)[:, None, :])
    az100 = az100.astype(np.int64) % 36000
    sin_table, cos_table = _azimuth_table()
    sin_az, cos_az = (sin_table[az100], cos_table[az100])
    cos_el, sin_el, correction = np.array([
        _channel_geometry(calibration.elevation_deg[c], calibration.azimuth_offset_deg[c],
                          lidar__XT_COORD_H_M, lidar__XT_COORD_B_M)
        for c in range(lidar__XT32_CHANNELS)
    ]).T
    corrected_distance = distance_m - correction
    xy = corrected_distance * cos_el
    x = xy * sin_az - lidar__XT_COORD_B_M * cos_az + lidar__XT_COORD_H_M * sin_az
    y = xy * cos_az + lidar__XT_COORD_B_M * sin_az + lidar__XT_COORD_H_M * cos_az
    z = corrected_distance * sin_el

    shape = keep.shape

    def per_point(values, axes):
        return np.broadcast_to(np.asarray(values).reshape(axes), shape)[keep]

    # A firing is one kept-return block (dual return: one of each block pair).
    firing = np.cumsum(block_ok.ravel()).reshape(block_ok.shape) - 1
    return ScanPoints(
        timestamp=per_point(np.asarray(times, dtype=np.float64), (count, 1, 1)),
        channel=per_point(np.arange(1, lidar__XT32_CHANNELS + 1), (1, 1, lidar__XT32_CHANNELS)),
        xyz=np.column_stack([x[keep], y[keep], z[keep]]),
        firing=per_point(firing, (count, lidar__XT32_BLOCKS, 1)),
        firing_azimuth_raw=raw_az100[block_ok],
    )


def lidar__road_alignment_transform(
    normal: np.ndarray, offset: float
) -> tuple[np.ndarray, np.ndarray]:
    """
    Build a rigid transform from sensor XYZ to a road-aligned coordinate frame.

    Input road plane:
        normal . p + offset = 0

    Output frame:
        road X : sensor +X projected onto the road plane
        road Y : completes a right-handed coordinate frame in the road plane
        road Z : road-plane normal
        Z = 0  : fitted road plane

    Returns:
        rotation_rows : shape (3, 3). Each row is one road axis expressed in
                        sensor coordinates.
        plane_origin   : the closest point on the fitted road plane to the
                        sensor origin, expressed in sensor coordinates.

    A sensor point p is transformed with:
        p_road = (p - plane_origin) @ rotation_rows.T

    Therefore p_road[:, 2] is exactly the signed orthogonal distance from the
    fitted road plane. Negative Z means below the road plane.
    """
    n = np.asarray(normal, dtype=np.float64).reshape(3)
    n_norm = float(np.linalg.norm(n))
    if not np.isfinite(n_norm) or n_norm < 1e-12:
        raise ValueError("Road plane normal is invalid")
    n = n / n_norm
    sensor_x = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    road_x = sensor_x - float(sensor_x @ n) * n
    road_x_norm = float(np.linalg.norm(road_x))
    if road_x_norm < 1e-09:
        sensor_y = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        road_x = sensor_y - float(sensor_y @ n) * n
        road_x_norm = float(np.linalg.norm(road_x))
    if road_x_norm < 1e-09:
        raise ValueError("Could not construct road X axis")
    road_x /= road_x_norm
    road_y = np.cross(n, road_x)
    road_y /= np.linalg.norm(road_y)
    rotation_rows = np.vstack([road_x, road_y, n])
    plane_origin = -float(offset) * n
    return (rotation_rows, plane_origin)


def lidar__road_aligned_xyz(
    xyz: np.ndarray, normal: np.ndarray, offset: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Transform sensor XYZ into explicit road-aligned XYZ."""
    xyz = np.asarray(xyz, dtype=np.float64)
    rotation_rows, plane_origin = lidar__road_alignment_transform(normal, offset)
    corrected = (xyz - plane_origin[None, :]) @ rotation_rows.T
    return (corrected, rotation_rows, plane_origin)


def lidar__nearest_gps_record(
    stream: Optional[tuple[np.ndarray, list[dict]]], camera_timestamp: float
) -> tuple[Optional[dict], float]:
    if stream is None:
        return (None, float("inf"))
    times, records = stream
    if times.size == 0:
        return (None, float("inf"))
    pos = int(np.searchsorted(times, camera_timestamp))
    candidates: list[int] = []
    if pos < len(times):
        candidates.append(pos)
    if pos > 0:
        candidates.append(pos - 1)
    best_pos = min(candidates, key=lambda index: abs(float(times[index]) - camera_timestamp))
    error = abs(float(times[best_pos]) - camera_timestamp)
    if error > lidar__GPS_MAX_TIME_ERROR_SEC:
        return (None, error)
    return (records[best_pos], error)


def gps_has_position(record):
    return (record.get("valid") is not False and
            _finite_float(record.get("latitude_deg")) is not None and
            _finite_float(record.get("longitude_deg")) is not None and
            abs(float(record["latitude_deg"])) <= 90 and abs(float(record["longitude_deg"])) <= 180)


def gps_rmc_valid(record):
    return gps_has_position(record) and str(record.get("gps_status", record.get("status", ""))).upper() == "A"


def gps_gga_valid(record):
    return gps_has_position(record) and (_finite_float(record.get("fix_quality")) or 0) > 0


def gps_interpolated(stream, target, valid, keys):
    """Values of keys at target from the valid fixes of a stream ordered by fix time:
    interpolated between the fixes on both sides when they are at most
    lidar__GPS_MAX_TIME_ERROR_SEC apart, else those of the nearer fix within that time of
    target (a run's first and last moments). None without one."""
    if stream is None:
        return None
    times, records = stream
    k = int(np.searchsorted(times, target))
    before = next((records[i] for i in range(k - 1, -1, -1) if valid(records[i])), None)
    after = next((records[i] for i in range(k, len(records)) if valid(records[i])), None)
    if (before is not None and after is not None and
            after["fix_time"] - before["fix_time"] <= lidar__GPS_MAX_TIME_ERROR_SEC):
        span = after["fix_time"] - before["fix_time"]
        weight = 0.0 if span <= 0 else (target - before["fix_time"]) / span
        values = {}
        for key in keys:
            a, b = _finite_float(before.get(key)), _finite_float(after.get(key))
            values[key] = None if a is None or b is None else a + (b - a) * weight
        return values
    near = [r for r in (before, after)
            if r is not None and abs(r["fix_time"] - target) <= lidar__GPS_MAX_TIME_ERROR_SEC]
    if not near:
        return None
    nearest = min(near, key=lambda r: abs(r["fix_time"] - target))
    return {key: _finite_float(nearest.get(key)) for key in keys}


def lidar__gps_values_for_camera(
    gps_streams: dict[str, tuple[np.ndarray, list[dict]]], camera_timestamp: float
) -> dict:
    """GPS of one picture at its time: position, altitude and speed interpolated between the
    fixes around it, quality, satellites, HDOP and course from the nearest sentence."""
    gga, gga_error = lidar__nearest_gps_record(gps_streams.get("GGA"), camera_timestamp)
    rmc, rmc_error = lidar__nearest_gps_record(gps_streams.get("RMC"), camera_timestamp)
    moving = gps_interpolated(gps_streams.get("RMC"), camera_timestamp, gps_rmc_valid,
                              ("latitude_deg", "longitude_deg", "speed_mps"))
    fixed = gps_interpolated(gps_streams.get("GGA"), camera_timestamp, gps_gga_valid,
                             ("latitude_deg", "longitude_deg", "altitude_m"))
    position = moving if moving is not None else fixed
    errors = [
        error for (record, error) in ((gga, gga_error), (rmc, rmc_error)) if record is not None
    ]
    speed_mps = None if moving is None else moving["speed_mps"]
    return {
        "latitude_deg": None if position is None else position["latitude_deg"],
        "longitude_deg": None if position is None else position["longitude_deg"],
        "speed_kmh": None if speed_mps is None else speed_mps * 3.6,
        "course_deg": None if rmc is None else rmc.get("course_deg"),
        "altitude_m": None if fixed is None else fixed["altitude_m"],
        "fix_quality": None if gga is None else gga.get("fix_quality"),
        "satellites": None if gga is None else gga.get("satellites"),
        "hdop": None if gga is None else gga.get("hdop"),
        "time_error_ms": None if not errors else max(errors) * 1000.0,
    }


def decode_scan(source, calibration, start, end):
    records = iter_scan_records(source, start, end)
    try:
        return decode_xt32_records(records, calibration)
    finally:
        records.close()


@lru_cache(maxsize=16)
def _pcap_record_index(path, size, mtime_ns):
    """Global header plus payload offset, length and time of every record of a
    classic PCAP. Parsed once per file version: size and mtime are in the key."""
    formats = {
        b"\xd4\xc3\xb2\xa1": ("<", 1000000.0),
        b"\xa1\xb2\xc3\xd4": (">", 1000000.0),
        b"M<\xb2\xa1": ("<", 1000000000.0),
        b"\xa1\xb2<M": (">", 1000000000.0),
    }
    with open(path, "rb") as stream:
        header = stream.read(lidar__PCAP_GLOBAL_LEN)
        if len(header) != lidar__PCAP_GLOBAL_LEN or header[:4] not in formats:
            raise ValueError("Invalid classic PCAP header")
        endian, divisor = formats[header[:4]]
        network = struct.unpack(endian + "IHHIIII", header)[-1]
        if network != 1:
            raise ValueError(f"Only Ethernet PCAP is supported (DLT={network})")
        offsets, sizes, seconds, fractions, truncated = ([], [], [], [], False)
        while True:
            record = stream.read(lidar__PCAP_RECORD_LEN)
            if not record:
                break
            if len(record) != lidar__PCAP_RECORD_LEN:
                truncated = True
                break
            second, fraction, length, _ = struct.unpack(endian + "IIII", record)
            offsets.append(stream.tell())
            sizes.append(length)
            seconds.append(second)
            fractions.append(fraction)
            stream.seek(length, 1)
    # Same value as seconds + fraction / divisor for each record.
    times = np.asarray(seconds, dtype=np.float64) + np.asarray(fractions, dtype=np.float64) / divisor
    return (header, offsets, sizes, times, truncated)


def iter_scan_records(source, start, end):
    """Stream the same inclusive slice as copy_scan_pcap, without temporary I/O.

    Indices are relative to the selected records, including non-XT32 packets,
    and continue across source files. Never stop at a high timestamp: a single
    source is allowed to contain out-of-order packets, as in the legacy path.
    Only records in [start, end] are read, located with the per-file index.
    """
    sources = [source] if isinstance(source, (str, os.PathLike)) else list(source)
    if not sources:
        raise ValueError("No PCAP sources")
    global_header = None
    previous_timestamp = None
    packet_index = 0
    for path in sources:
        stat = os.stat(path)
        header, offsets, sizes, times, truncated = _pcap_record_index(
            str(path), stat.st_size, stat.st_mtime_ns
        )
        if global_header is not None and header != global_header:
            raise ValueError("Incompatible PCAP headers across scan sources")
        global_header = header
        if truncated:
            raise ValueError("Truncated PCAP record header")
        with Path(path).open("rb") as stream:
            for i in np.flatnonzero((times >= start) & (times <= end)).tolist():
                timestamp = float(times[i])
                stream.seek(offsets[i])
                payload = stream.read(sizes[i])
                if len(payload) != sizes[i]:
                    raise ValueError("Truncated PCAP packet")
                if (
                    len(sources) > 1
                    and previous_timestamp is not None
                    and (timestamp < previous_timestamp)
                ):
                    raise ValueError("PCAP scan sources are not chronologically ordered")
                previous_timestamp = timestamp
                yield lidar__PcapRecord(packet_index, timestamp, payload)
                packet_index += 1


def pcap_scan_time_bounds(path: Path, gap_sec: float = 0.005) -> list[tuple[float, float]]:
    """Return per-file scan fragment time bounds detected from packet gaps.

    XT32 scans in this run contain a burst of packets (~22 ms) followed by a
    quiet gap (~28 ms). The packet sequence counter is continuous in the
    recorded PCAP, so sequence rollover cannot be used as a scan boundary.
    First/last fragments may be partial; merge_scan_fragments joins files.
    """
    gap_sec = float(gap_sec)
    if gap_sec <= 0.0:
        raise ValueError("scan gap must be positive")
    scans: list[tuple[float, float]] = []
    start: Optional[float] = None
    end: Optional[float] = None
    previous_timestamp: Optional[float] = None
    for rec in lidar__iter_pcap(path):
        try:
            payload = lidar__decode_udp(rec.frame)
        except ValueError:
            continue
        if len(payload) != lidar__XT32_PAYLOAD_LEN or payload[:4] != b"\xee\xff\x06\x01":
            continue
        timestamp = float(rec.timestamp)
        if (
            start is not None
            and previous_timestamp is not None
            and (timestamp - previous_timestamp > gap_sec)
        ):
            scans.append((start, end))
            start = None
        if start is None:
            start = timestamp
        end = timestamp
        previous_timestamp = timestamp
    if start is not None and end is not None:
        scans.append((start, end))
    return scans


def merge_scan_fragments(fragments, gap_sec=0.005):
    """Join (path,start,end) fragments into (source tuple,start,end) sweeps.

    Join only different files with non-overlapping continuous packet times.
    Real quiet gaps remain separate scans.
    """
    if not np.isfinite(gap_sec) or gap_sec <= 0:
        raise ValueError("scan gap must be positive")
    merged = []
    for path, start, end in sorted(fragments, key=lambda item: (item[1], item[2])):
        path = Path(path)
        if not np.isfinite(start) or not np.isfinite(end) or end < start:
            raise ValueError("Invalid scan fragment bounds")
        if merged:
            sources, first, last = merged[-1]
            if path not in sources and 0 <= start - last <= gap_sec:
                merged[-1] = (sources + (path,), first, end)
                continue
        merged.append(((path,), start, end))
    return merged


def plane_points(points, center_raw):
    """Mask of the points the road plane is fitted to, or None when the scan does not
    reach the camera's optical axis (within 1 degree).

    The firing nearest center_raw (LiDAR2Camera.axis_azimuth_raw) and PLANE_SIDE_FIRINGS
    firings on each side, every channel, as codecode/lcz_to_pcap.py selects them.
    """
    azimuth = points.firing_azimuth_raw
    if not len(azimuth):
        return None
    error = np.abs((azimuth - center_raw + 18000) % 36000 - 18000)
    center = int(np.argmin(error))
    if error[center] > 100:
        return None
    return np.abs(points.firing - center) <= PLANE_SIDE_FIRINGS


def flatten(points, center_raw):
    """Road-aligned points of a scan; raises ValueError when the road plane fails its checks."""
    if len(points) < 3:
        raise ValueError("insufficient_lidar_points")
    xyz = points.xyz
    normal, offset, quality = fit_spatial_road_plane(xyz, plane_points(points, center_raw))
    if (quality["cells"] < PLANE_MIN_CELLS or quality["support"] < PLANE_MIN_SUPPORT
            or quality["tilt_deg"] > PLANE_MAX_TILT_DEG):
        raise ValueError(
            f"plane_unreliable: {quality['cells']} cells, {quality['support']:.0%} within "
            f"{PLANE_SUPPORT_M * 100:g} cm, tilt {quality['tilt_deg']:.1f} deg from the mount")
    road, rotation, _ = lidar__road_aligned_xyz(xyz, normal, offset)
    return dict(
        points=points,
        point_timestamps=points.timestamp,
        sensor_xyz_m=xyz,
        road_xyz_m=road,
        plane_normal=normal,
        road_rotation=rotation,
        plane_quality=quality,
    )


def validate_objects(detections, result, camera, points, alignment, target):
    """Cracks are the model's, measured on the road plane (result); a pothole needs its LiDAR
    lines to dip below the road beside it (PotholeScan, line_depths, is_pothole). points: the
    frame's decoded scan, None when there is none close enough to the image."""
    pixels, visible = (
        camera.project(result["sensor_xyz_m"])
        if result is not None
        else (np.empty((0, 2)), np.empty(0, bool))
    )
    masks = [None if d.mask is None else np.asarray(d.mask, bool) for d in detections]
    shape = next((m.shape for m in masks if m is not None), None)
    frame = None  # the scan for the potholes, made when the frame has one
    if points is not None and shape is not None and any(int(d.class_id) == 1 for d in detections):
        frame = PotholeScan(points, [np.zeros(shape, bool) if m is None else m for m in masks],
                            camera, alignment, target, shape)
    accepted, objects, audit = ([], [], [])
    for i, (d, mask) in enumerate(zip(detections, masks)):
        obj = None
        if int(d.class_id) == 0:
            indices = np.empty(0, int)
            if result is not None and mask is not None:
                h, w = mask.shape
                valid = (
                    visible
                    & np.isfinite(pixels).all(axis=1)
                    & np.isfinite(result["sensor_xyz_m"]).all(axis=1)
                    & np.isfinite(result["road_xyz_m"]).all(axis=1)
                    & (pixels[:, 0] >= 0)
                    & (pixels[:, 0] < w)
                    & (pixels[:, 1] >= 0)
                    & (pixels[:, 1] < h)
                )
                indices = np.flatnonzero(valid)
                uv = pixels[indices].astype(int)
                indices = indices[mask[uv[:, 1], uv[:, 0]]]
            road = np.empty((0, 3)) if result is None else result["road_xyz_m"][indices]
            evidence = dict(method="model_only", accepted=True, reason="model_crack",
                            mask_points=len(indices), validation_state="accepted")
            obj = dict(class_id=0, status="model_mask", point_indices=indices, filtered_points=len(indices),
                       verified_point_indices=indices, verified_road_xyz_m=road,
                       measurement_record=damage_geometry(road, include_depth=False), validation=evidence)
        elif frame is None:  # no LiDAR scan for the image: nothing is confirmed
            evidence = dict(method="model_mask_line_road_depth", accepted=False, reason="lidar_unavailable",
                            max_depth_m=None, threshold_depth_m=POTHOLE_DEPTH_M, mask_points=0,
                            validation_state="unobserved")
        else:
            inside = frame.inside(masks[i] if mask is not None else np.zeros(shape, bool))
            lines = frame.lines(inside, detail=True) if inside.any() else []
            evidence = pothole_evidence(lines, inside)
            if evidence["accepted"]:
                used = [line for line in lines if line["measured"]]
                verified = np.array([p for line in used for p in line["area_index"]], int)
                road = np.c_[frame.road_xyz[verified, :2], -np.array([v for line in used for v in line["depth"]])]
                obj = dict(class_id=1, status="model_mask", point_indices=np.flatnonzero(inside),
                           filtered_points=int(inside.sum()), verified_point_indices=verified,
                           verified_road_xyz_m=road, measurement_record=damage_geometry(road, include_depth=True),
                           validation=evidence)
        audit.append(dict(model_object_id=i, class_id=int(d.class_id), confidence=float(d.confidence),
                          box_xyxy=np.asarray(d.box_xyxy).tolist(), **evidence))
        if obj is not None:
            accepted.append(d)
            objects.append(obj)
    return (accepted, objects, audit, frame is not None)


def scan_candidates(pcaps, center, cache):
    """(|delta|, sources, start, end, delta) of each scan near center; pcaps hold absolute paths."""
    fragments = []
    for item in pcaps:
        if (
            float(item["last_timestamp"]) < center - SCAN_SEARCH_HALF_WINDOW_SEC
            or float(item["first_timestamp"]) > center + SCAN_SEARCH_HALF_WINDOW_SEC
        ):
            continue
        p = Path(item["path"])
        if str(p) not in cache:
            cache[str(p)] = pcap_scan_time_bounds(p, SCAN_GAP_SEC)
        fragments.extend(((p, start, end) for (start, end) in cache[str(p)]))
    candidates = []
    for sources, start, end in merge_scan_fragments(fragments, SCAN_GAP_SEC):
        delta = (start + end) * 0.5 - center
        if abs(delta) <= SCAN_SEARCH_HALF_WINDOW_SEC:
            candidates.append((abs(delta), sources, start, end, delta))
    return candidates


class RecordingAlignment:
    """GPS speeds of a run, to compensate vehicle motion between LiDAR packets and the image."""

    def __init__(self, gps_streams):
        speed_records = []
        stream = gps_streams.get("RMC")
        for record in (() if stream is None else stream[1]):
            if not record.get("valid") or not record.get("checksum_valid"):
                continue
            try:
                row = (float(record["fix_time"]), float(record["speed_mps"]))
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(row[0]) and math.isfinite(row[1]) and row[1] >= 0:
                speed_records.append(row)
        self.speeds = np.asarray(sorted(speed_records), dtype=float).reshape(-1, 2)

    def speed_at(self, target):
        """GPS speed (m/s) at target: interpolated between valid speeds on both sides within
        1.5 s of each other, else the nearer valid speed within 1.5 s of target (a run's first
        and last second, whose other neighbour is in the next or previous run's GPS file), else
        None."""
        k = int(np.searchsorted(self.speeds[:, 0], target))
        if 0 < k < len(self.speeds) and self.speeds[k, 0] - self.speeds[k - 1, 0] <= 1.5:
            return float(np.interp(target, self.speeds[:, 0], self.speeds[:, 1]))
        near = [i for i in (k - 1, k) if 0 <= i < len(self.speeds) and abs(self.speeds[i, 0] - target) <= 1.5]
        if not near:
            return None
        return float(self.speeds[min(near, key=lambda i: abs(self.speeds[i, 0] - target)), 1])

    def compensate(self, raw, camera, target):
        speed = self.speed_at(target)
        if speed is None:
            raise ValueError("Motion compensation requires bracketing valid GPS speeds")
        normal = raw["plane_normal"]
        direction = camera.R[2].copy()
        direction -= (direction @ normal) * normal
        length = float(np.linalg.norm(direction))
        if length < 0.1:
            raise ValueError("Camera direction cannot be projected onto the road")
        velocity = direction / length * speed
        timestamps = raw["point_timestamps"]
        dt = float(target) - timestamps
        if not np.isfinite(dt).all() or np.max(np.abs(dt), initial=0) > 0.15:
            raise ValueError("Motion compensation exceeds 150ms packet age")
        shift = dt[:, None] * velocity
        road = raw["road_xyz_m"] + shift @ raw["road_rotation"].T
        # Translation is tangent to the fitted plane; preserve measured heights.
        road[:, 2] = raw["road_xyz_m"][:, 2]
        return dict(
            raw,
            sensor_xyz_m=raw["sensor_xyz_m"] + shift,
            road_xyz_m=road,
            motion_compensation=dict(
                schema="packet_translation_v1",
                profile_id=ALIGNMENT_PROFILE_ID,
                target_epoch_sec=float(target),
                velocity_sensor_mps=velocity.tolist(),
                formula="xyz_target = xyz_raw + (target_epoch_sec - packet_epoch_sec) * velocity_sensor_mps",
                timestamp_basis="pcap_packet_record_timestamp",
                method="gps_speed_camera_forward_projected_on_road",
                approximate=True,
                independently_moving_objects_corrected=False,
                raw_pcap_unchanged=True,
            ),
        )


def atomic_json(path, value, durable=True, indent=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=indent)
        if indent is not None:
            stream.write("\n")
        if durable:
            stream.flush()
            os.fsync(stream.fileno())
    os.replace(tmp, path)


def frame_time(row):
    if row.get("timestamp_source") != "camera_ptp_rtcp_utc" or not row.get("timestamp_ns"):
        raise ValueError("PTP camera timestamp missing")
    value = int(row["timestamp_ns"]) / 1000000000.0
    if not math.isfinite(value):
        raise ValueError("Invalid camera timestamp")
    return value


# NMEA measured longer ago than this when it arrived is not used: the RTK router sends the
# sentences it buffered (over a minute of them) in one burst when a connection opens.
GPS_MAX_AGE_SEC = 2.0


def gps_fix_time(row):
    """Epoch second at which the receiver measured an NMEA row: its UTC time of day on the
    UTC day of arrival (or the day before or after, around midnight). None without one."""
    text = str(row.get("gps_utc_time") or "")
    arrival = float(row["timestamp"])
    if len(text) < 6 or not math.isfinite(arrival):
        return None
    fix = math.floor(arrival / 86400.0) * 86400.0 + (
        int(text[0:2]) * 3600 + int(text[2:4]) * 60 + float(text[4:]))
    return min((fix - 86400.0, fix, fix + 86400.0), key=lambda value: abs(value - arrival))


class GpsTail:
    """Read newly committed NMEA rows once, retaining GGA/RMC with a valid checksum, ordered by
    the receiver's fix time (row["fix_time"]) instead of the arrival time.

    A damaged line has no fields; kept, it would hide the good fix next to it from the
    nearest-row lookups. A row that arrived more than GPS_MAX_AGE_SEC after its fix (or
    before it, so with a wrong receiver clock) is not kept.
    """

    def __init__(self, run):
        self.path = run / "gps/gps.jsonl"
        self.position = 0
        self.grouped = {"GGA": [], "RMC": []}
        self.identity = None

    def read(self):
        changed = False
        if not self.path.exists():
            return None
        stat = self.path.stat()
        identity = (stat.st_dev, stat.st_ino)
        if self.identity is not None and (
            identity != self.identity or stat.st_size < self.position
        ):
            raise RuntimeError(f"GPS manifest replaced or truncated: {self.path}")
        self.identity = identity
        with self.path.open("rb") as stream:
            stream.seek(self.position)
            while True:
                line = stream.readline()
                if not line.endswith(b"\n"):
                    break
                self.position = stream.tell()
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                    kind = str(row.get("sentence_type", "")).upper()
                    if kind not in self.grouped or row.get("checksum_valid") is False:
                        continue
                    fix = gps_fix_time(row)
                    if fix is None or not -1.0 <= float(row["timestamp"]) - fix <= GPS_MAX_AGE_SEC:
                        continue
                    row["fix_time"] = fix
                    self.grouped[kind].append(row)
                    changed = True
                except (ValueError, TypeError, KeyError):
                    continue
        if not changed:
            return None
        streams = {}
        for kind, rows in self.grouped.items():
            rows.sort(key=lambda row: row["fix_time"])
            streams[kind] = (np.asarray([row["fix_time"] for row in rows]), rows)
        return streams


IMAGE_PREFETCH = 4  # JPEGs decoded ahead in a thread; cv2.imread releases the GIL


def lidar_stage(detections, target, scan, camera, angles, alignment, cache):
    """LiDAR part of one frame: decode the selected scan, fit the road plane (crack
    measurements), compensate vehicle motion and validate the detections.

    The cache only keeps decoded scans it would otherwise decode again.
    scan is None or (sources, start, end, delta) from LiveProcessor.frame_scan.
    """
    result, points, scanned, error = None, [], None, ""
    status = "no_model_detection" if not detections else "lidar_scan_missing"
    motion = dict(applied=False, reason="not_required")
    if detections and scan is not None:
        sources, start, end, delta = scan
        status = "lidar_scan_too_far"
        if abs(delta) <= MAX_SCAN_CENTER_DELTA_SEC:
            try:
                cache_key = (tuple(map(str, sources)), start, end)
                if cache_key not in cache:
                    points = decode_scan(sources, angles, start - 1e-06, end + 1e-06)
                    plane_error = ""
                    try:
                        plane = flatten(points, camera.axis_azimuth_raw)
                    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                        plane, plane_error = None, str(exc)
                    cache[cache_key] = (points, plane, plane_error)
                points, result, plane_error = cache[cache_key]
                scanned = points  # potholes use the scan even when the road plane fails
                cache.move_to_end(cache_key)
                while len(cache) > 4:
                    cache.popitem(last=False)
                if plane_error:
                    raise ValueError(plane_error)
                status = "ok"
                try:
                    result = alignment.compensate(result, camera, target)
                    motion = dict(applied=True, **result.get("motion_compensation", {}))
                except ValueError as exc:
                    motion = dict(applied=False, reason=str(exc))
            except (OSError, ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                result = None
                status, error = ("plane_or_scan_invalid", str(exc))
    accepted, objects, audit, judged = validate_objects(detections, result, camera, scanned,
                                                        alignment, target)
    return dict(
        status=status,
        error=error,
        motion=motion,
        point_count=len(points),
        has_result=result is not None or judged,  # judged: the scan's time goes into the report
        road_xyz_m=None if result is None else result["road_xyz_m"],
        accepted=[i for i, row in enumerate(audit) if row["accepted"]],
        objects=objects,
        audit=audit,
    )


class LiveProcessor:

    def __init__(self, log, uploader):
        self.log, self.uploader = (log, uploader)
        cv2.setNumThreads(FUSION_OPENCV_THREADS)
        self.camera = LiDAR2Camera(CAMERA_CALIBRATION)
        self.angles = lidar__load_calibration(LIDAR_CALIBRATION)
        self.detector = DXNNDetector(MODEL_PATH, CONFIDENCE_THRESHOLD)
        self.tracker = ObjectTracker()
        self.sequence = 0
        self.last_frame_time = None  # capture time of the last tracked picture (TRACK_GAP_SEC)
        self.session_id = uuid.uuid4().hex
        self.scan_bounds = {}
        self.scan_cache = OrderedDict()
        self.exporter = None
        self.run = None
        self.alignment = None
        self.gps = None
        self.closed = False
        # JPEGs are decoded ahead in a thread; everything else stays in source order here.
        self.loader = ThreadPoolExecutor(1, thread_name_prefix="live-image")

    def close(self):
        if not self.closed:
            self.closed = True
            self.loader.shutdown(wait=True, cancel_futures=True)
            self.detector.dispose()

    def refresh_run(self, run):
        if self.run != run:
            self.run = run
            self.exporter = DamageArtifactExporter(self.camera)
            self.gps = GpsTail(run)
            self.alignment = RecordingAlignment({})
        streams = self.gps.read()
        if streams is not None:
            self.exporter.gps_streams = streams
            self.alignment = RecordingAlignment(streams)

    def prune_pcaps(self, pcaps):
        paths = {str(Path(row["path"])) for row in pcaps}
        self.scan_bounds = {
            path: value for path, value in self.scan_bounds.items() if path in paths
        }

    def infer(self, run, records):

        def read_image(row):
            path = Path(row["path"])
            path = path if path.is_absolute() else run / path
            image = None
            if simplejpeg is not None:
                try:
                    image = simplejpeg.decode_jpeg(path.read_bytes(), colorspace="BGR")
                except (OSError, ValueError, RuntimeError):
                    image = None  # OpenCV decides, as before, what a damaged or missing file gives
            if image is None:
                image = cv2.imread(str(path))
            if image is None:  # a broken JPEG: analysed as a blank picture, not a crash and restart
                print(f"[ANALYSIS] unreadable image, analysed as blank: {path}", flush=True)
                image = np.zeros((1080, 1920, 3), np.uint8)
            return image

        # iter_detect loads records strictly in order; decode the next few ahead.
        records = list(records)
        upcoming, queued = iter(records), deque()

        def load_image(row):
            while len(queued) < IMAGE_PREFETCH:
                following = next(upcoming, None)
                if following is None:
                    break
                queued.append((following, self.loader.submit(read_image, following)))
            queued_row, image = queued.popleft()
            if queued_row is not row:
                raise RuntimeError("Image prefetch lost the frame order")
            return image.result()

        return self.detector.iter_detect(records, load_image)

    def frame_scan(self, row, detections, pcaps):
        """Camera time, LiDAR target time and the nearest scan (None if there is none)."""
        if any(int(d.class_id) not in SUSPECT_CLASS_IDS for d in detections):
            raise ValueError("This terminal supports only Crack/Pothole classes 0/1")
        timestamp = frame_time(row)
        target = timestamp - OFFSET_SEC
        scan = None
        if detections:
            try:
                candidates = scan_candidates(pcaps, target, self.scan_bounds)
            except (OSError, ValueError) as exc:  # a PCAP gone or unreadable: no scan, no crash
                print(f"[ANALYSIS] LiDAR files unreadable, frame {row.get('frame_index')}: {exc}", flush=True)
                candidates = []
            if candidates:
                _, sources, start, end, delta = min(candidates, key=lambda item: item[0])
                scan = (sources, start, end, delta)
        return (timestamp, target, scan)

    def submit(self, run, row, image, detections, pcaps):
        """LiDAR part of one frame (see lidar_stage); report() does the rest."""
        started = time.monotonic()
        timestamp, target, scan = self.frame_scan(row, detections, pcaps)
        lidar = lidar_stage(detections, target, scan, self.camera, self.angles, self.alignment,
                            self.scan_cache)
        return SimpleNamespace(row=row, image=image, detections=detections, timestamp=timestamp,
                               target=target, scan=scan, lidar=lidar, started=started)

    def analyse(self, run, key, stream, pcaps, stopping):
        """Report the frames of an inference stream in source order, yielding each summary."""
        for row, image, detections in stream:
            if stopping():
                return
            frame = self.submit(run, row, image, detections, pcaps)
            if stopping():
                return
            wait_for_collector(stopping)
            yield self.report(run, key, frame)

    def report(self, run, key, frame):
        """Track one submitted frame; callers keep source order.

        A frame with newly confirmed damage is listed in the CSV and, when the uploader
        takes it, its JPG/JSON/PCAP are made in UPLOAD_STAGING for sending.
        """
        row, image, detections = (frame.row, frame.image, frame.detections)
        timestamp, target, scan = (frame.timestamp, frame.target, frame.scan)
        lidar = frame.lidar
        index = int(row["frame_index"])
        image_path = Path(row["path"])
        if not image_path.is_absolute():
            image_path = run / image_path
        sources, start, end, _ = ((), None, None, None) if scan is None else scan
        scan_time = None if scan is None else (start + end) * 0.5
        status, motion = (lidar["status"], lidar["motion"])
        accepted = [detections[i] for i in lidar["accepted"]]
        objects, audit = (lidar["objects"], lidar["audit"])
        self.sequence += 1
        if self.last_frame_time is not None and not 0 < timestamp - self.last_frame_time <= TRACK_GAP_SEC:
            next_id = self.tracker.next_id
            self.tracker = ObjectTracker()
            self.tracker.next_id = next_id  # track numbers go on: one session never reuses one
        self.last_frame_time = timestamp
        tracked = self.tracker.update(
            self.sequence,
            image,
            [
                dict(class_id=int(d.class_id), **dict(zip(("x1", "y1", "x2", "y2"), d.box_xyxy)))
                for d in detections
            ],
            audit,
        )
        report_detections, report_objects = ([], [])
        accepted_index = 0
        for track, observation in zip(tracked, audit):
            if observation["accepted"]:
                if track["newly_confirmed"]:
                    report_detections.append(accepted[accepted_index])
                    report_objects.append(
                        dict(
                            objects[accepted_index],
                            tracking=dict(
                                track,
                                live_session_id=self.session_id,
                                source_run=key,
                                source_frame_index=index,
                            ),
                        )
                    )
                accepted_index += 1
        summary = dict(frame_index=index, status=status, model_count=len(detections),
                       reported_count=len(report_detections))
        if not report_detections:
            return summary
        self.log.add(key, timestamp, image_path, report_detections)
        if self.uploader is None or not self.uploader.accepting():
            return summary
        folder = UPLOAD_STAGING / key / f"{index:08d}"
        try:
            self.exporter.directory = folder / "certifcate"
            self.exporter.directory.mkdir(parents=True)
            damage_payloads, artifacts = [], []
            self.exporter.save(
                timestamp,
                report_detections,
                report_objects,
                lidar["point_count"],
                lidar["road_xyz_m"],
                scan_time if lidar["has_result"] else None,
                damage_payloads,
                scan_start=start,
                scan_end=end,
                target_lidar_timestamp=target,
                camera_timing=dict(
                    profile_id=ALIGNMENT_PROFILE_ID,
                    mode="ptp",
                    timestamp_ns=row.get("timestamp_ns"),
                    timestamp_source=row.get("timestamp_source"),
                    recorded_epoch_sec=timestamp,
                    alignment_camera_epoch_sec=timestamp,
                ),
                motion_compensation=motion,
            )
            for payload in damage_payloads:
                if payload.get("tracking"):
                    payload["tracking"]["scope"] = "live_process_session_across_run_rotation"
            pcap_path = self.exporter.directory / (detection_frame_stem(row) + ".pcap")
            pcap_metadata = save_scan_pcap(
                pcap_path, sources,
                None if start is None else start - 1e-6,
                None if end is None else end + 1e-6,
            )
            pcap_files = [] if pcap_metadata is None else [pcap_metadata]
            if pcap_metadata is not None:
                artifacts.append(pcap_path)
            for payload in damage_payloads:
                payload["lidar"].update(
                    pcap_files=pcap_files, status=status if pcap_files else "pcap_unavailable",
                    pcap_copy_policy="selected_scan_raw_records_unchanged",
                )
            gps = matched_gps_position(self.exporter.gps_streams, target)  # exposure moment
            speed = self.alignment.speed_at(target)  # the speed motion compensation used
            gps["speed_mps"] = None if speed is None else round(speed, 3)
            pair = save_detection_frame(
                self.exporter.directory, image_path, image, row, key,
                report_detections, damage_payloads, gps, pcap_files,
            )
            artifacts.extend(pair)
            files = {p.name: dict(bytes=p.stat().st_size, sha256=sha(p)) for p in artifacts}
            reason = triplet_error(files)
            if reason:  # e.g. no LiDAR scan for the PCAP: the server would refuse it
                raise ValueError(reason)
        except Exception as exc:
            # The frame stays in the CSV; one frame the server cannot get must not stop
            # the analysis (a restart would skip every frame recorded meanwhile).
            print(f"[UPLOAD] not sent, only in the CSV: run={key} frame={index}: {exc}", flush=True)
            shutil.rmtree(folder, ignore_errors=True)
            return summary
        self.uploader.send(key, index, folder, dict(frame_index=index, files=files, frame_log={}))
        return summary


class DetectionLog:
    """<date folder>/DETECTIONS_CSV: time (KST), image (path in the date folder) and damage
    types of each frame with newly confirmed damage. The date folder is the run's, so a run
    that goes past midnight stays with its own date."""

    def __init__(self, root):
        self.root = Path(root)
        self.path = self.stream = None

    def open(self, path):
        self.close()
        self.path = path
        if self.path.exists():
            with self.path.open("r+b") as stream:
                size = stream.seek(0, os.SEEK_END)
                stream.seek(max(0, size - 65536))
                tail = stream.read()
                if tail and not tail.endswith(b"\n"):  # a row a power cut left unfinished
                    stream.truncate(size - len(tail) + tail.rfind(b"\n") + 1)
                    os.fdatasync(stream.fileno())
        new = not self.path.exists() or self.path.stat().st_size == 0
        self.stream = self.path.open("a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.stream)
        if new:
            self.writer.writerow(("time", "image", "objects"))
            self.sync()
            fsync_dir(self.path.parent)

    def sync(self):
        # Detections are rare and the CSV is the only record kept, so each row goes to
        # the disk at once: a power cut right after a detection must not lose it.
        self.stream.flush()
        os.fdatasync(self.stream.fileno())

    def add(self, key, timestamp, image_path, detections):
        folder = self.root / Path(key).parts[0]
        path = folder / DETECTIONS_CSV
        if path != self.path or not path.exists():  # a new date, or deleted while running
            self.open(path)
        image = Path(image_path)
        if image.is_relative_to(folder):
            image = image.relative_to(folder)
        kinds = sorted({"pothole" if int(d.class_id) == 1 else "crack" for d in detections})
        self.writer.writerow((camera_time_kst(timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                              image.as_posix(), "+".join(kinds)))
        self.sync()

    def close(self):
        if self.stream:
            self.stream.close()
            self.stream = None


UPLOAD_SETTINGS = (
    "PORTHOLE_UPLOAD_HOST",
    "PORTHOLE_UPLOAD_USER",
    "PORTHOLE_UPLOAD_DIR",
    "PORTHOLE_UPLOAD_KEY",
)


def upload_options():
    """Upload settings come only from the environment or the sibling .env.

    PORTHOLE_API_URL selects the HTTP API (JPG/JSON/PCAP per the terminal-server
    data spec); without it the SSH receiver settings are used.
    """
    bw_kib = int(os.getenv("PORTHOLE_UPLOAD_BW_KIB", "24576"))
    if bw_kib <= 0:
        raise ValueError("PORTHOLE_UPLOAD_BW_KIB must be positive")
    api_url = os.getenv("PORTHOLE_API_URL", "").strip()
    if api_url:
        return SimpleNamespace(
            mode="api", api_url=api_url, api_token=os.getenv("PORTHOLE_API_TOKEN", "").strip(),
            bw_kib=bw_kib,
        )
    missing = [name for name in UPLOAD_SETTINGS if not os.getenv(name, "").strip()]
    if missing:
        raise ValueError(
            "upload is not configured; set PORTHOLE_API_URL or " + ", ".join(missing) + " in .env"
        )
    return SimpleNamespace(
        mode="ssh",
        host=os.environ["PORTHOLE_UPLOAD_HOST"].strip(),
        user=os.environ["PORTHOLE_UPLOAD_USER"].strip(),
        destination=os.environ["PORTHOLE_UPLOAD_DIR"].strip(),
        key=Path(os.environ["PORTHOLE_UPLOAD_KEY"].strip()).expanduser(),
        bw_kib=bw_kib,
    )


class UploadWorker:
    """Send reported frames from UPLOAD_STAGING, oldest first, and delete each one after.

    HTTP API when PORTHOLE_API_URL is set, else the embedded byte/hash-verified SSH
    transport. Nothing is kept for later: when a send fails (no network, server down)
    the frames waiting are dropped and, for OFFLINE_RETRY_SEC, new detections are only
    listed in the CSV. A frame the server rejects is dropped as well.
    """

    def __init__(self, timeout=30):
        self.timeout = timeout
        self.jobs = queue.Queue()
        self.offline_until = 0.0
        self.stop_event = threading.Event()
        self.error = ""
        self.busy = False  # a frame is being sent
        self.thread = threading.Thread(target=self.run, name="live-artifact-upload", daemon=True)
        try:
            self.options = upload_options()
        except ValueError as exc:
            self.enabled, self.error = (False, str(exc))
            print(f"[UPLOAD] off, detections are only listed in the CSV: {exc}", flush=True)
            return
        self.enabled = True
        self.thread.start()

    def accepting(self):
        """Whether a newly reported frame should be prepared for sending."""
        return (self.enabled and time.monotonic() >= self.offline_until
                and self.jobs.qsize() < UPLOAD_QUEUE_LIMIT)

    def send(self, key, index, folder, manifest):
        self.jobs.put((key, index, Path(folder), manifest))

    def pending(self):
        return self.jobs.qsize()

    def waiting(self):
        """Whether a frame is being sent or waits to be (RawUploader holds back meanwhile)."""
        return self.busy or not self.jobs.empty()

    def drop_waiting(self):
        while True:
            try:
                folder = self.jobs.get_nowait()[2]
            except queue.Empty:
                return
            shutil.rmtree(folder, ignore_errors=True)

    def run(self):
        uploader, active_key = (None, None)
        options = self.options
        try:
            target = options.api_url if options.mode == "api" else f"{options.user}@{options.host}"
            print(f"[UPLOAD] {options.mode} -> {target}", flush=True)
            while not self.stop_event.is_set():
                try:
                    key, index, folder, manifest = self.jobs.get(timeout=1)
                except queue.Empty:
                    continue
                if time.monotonic() < self.offline_until:  # queued just as a send failed
                    shutil.rmtree(folder, ignore_errors=True)
                    continue
                self.busy = True
                try:
                    if key != active_key:
                        if uploader:
                            uploader.close()
                        uploader = (
                            HttpUploader(options, timeout=self.timeout)
                            if options.mode == "api"
                            else PersistentUploader(
                                options, Path(REMOTE_FOLDER) / key / "certifcate", timeout=self.timeout
                            )
                        )
                        active_key = key
                    receipt = uploader.upload(folder / "certifcate", manifest)
                    self.error = ""
                    print(f"[UPLOAD] verified run={key} frame={index} files={receipt['files']}", flush=True)
                except PermanentUploadError as exc:
                    print(f"[UPLOAD] rejected by server, dropped run={key} frame={index}: {exc}", flush=True)
                except Exception as exc:
                    self.error = str(exc)
                    self.offline_until = time.monotonic() + OFFLINE_RETRY_SEC
                    dropped = 1 + self.jobs.qsize()
                    self.drop_waiting()
                    print(f"[UPLOAD] cannot send ({exc}); dropped {dropped} frame(s), "
                          f"CSV only for {OFFLINE_RETRY_SEC} s", flush=True)
                    if uploader:
                        uploader.close()
                    uploader, active_key = (None, None)
                finally:
                    shutil.rmtree(folder, ignore_errors=True)
                    self.busy = False
        except Exception as exc:
            self.enabled, self.error = (False, str(exc))
            print(f"[UPLOAD] worker stopped, detections are only listed in the CSV: {exc}", flush=True)
        finally:
            self.drop_waiting()
            if uploader:
                uploader.close()

    def close(self, drain_seconds=3):
        """Give waiting frames a moment, then stop; the whole shutdown has to fit in the 15 s the
        analysis terminal allows before it kills the process (the CSV rows are already on disk)."""
        deadline = time.monotonic() + drain_seconds
        while self.jobs.qsize() and self.thread.is_alive() and time.monotonic() < deadline:
            time.sleep(0.1)
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=5)
        self.drop_waiting()


# The finished recordings themselves also go to the server, as they are on the SSD (RawUploader),
# when .env has PORTHOLE_RAW_UPLOAD=true; unset or false (the default), they stay on the terminal.
RAW_DONE = PROJECT / "var" / "raw_uploaded.txt"  # runs sent completely, one "<date>/<run>" a line

RAW_LOCK = Path("/tmp/porthole_raw_upload.lock")

RAW_POLL_SEC = 60  # a run is finished every 10 minutes

# A finished run is sent once its run_meta.jsonl is this old: the collector seals the folder's last
# LiDAR file in a thread of its own, which can end after the run_finished record.
RAW_SETTLE_SEC = 60

RAW_RETRY_MAX_SEC = 3600  # a run that keeps failing is tried again after 1, 2, 4 ... minutes, at most this

# rsync exit statuses of a server that cannot be reached (ssh, connection, timeout): no run is to
# blame, so all are simply tried again at the next look.
RAW_UNREACHABLE = frozenset((5, 10, 12, 30, 35, 255))


def raw_upload_options():
    """Where the recordings go: PORTHOLE_RAW_DIR on the server of the SSH upload settings
    (PORTHOLE_UPLOAD_HOST, _USER, _KEY, also when PORTHOLE_API_URL sends the detections).
    Only with PORTHOLE_RAW_UPLOAD=true; unset or false, nothing is sent."""
    switch = os.getenv("PORTHOLE_RAW_UPLOAD", "").strip()
    if switch.lower() != "true":
        raise ValueError(f"PORTHOLE_RAW_UPLOAD={switch or 'false'} (true in .env sends them)")
    names = ("PORTHOLE_RAW_DIR", "PORTHOLE_UPLOAD_HOST", "PORTHOLE_UPLOAD_USER", "PORTHOLE_UPLOAD_KEY")
    missing = [name for name in names if not os.getenv(name, "").strip()]
    if missing:
        raise ValueError("set " + ", ".join(missing) + " in .env")
    bw_kib = int(os.getenv("PORTHOLE_UPLOAD_BW_KIB", "24576"))
    if bw_kib <= 0:
        raise ValueError("PORTHOLE_UPLOAD_BW_KIB must be positive")
    return SimpleNamespace(
        destination=os.environ["PORTHOLE_RAW_DIR"].strip().rstrip("/"),
        host=os.environ["PORTHOLE_UPLOAD_HOST"].strip(),
        user=os.environ["PORTHOLE_UPLOAD_USER"].strip(),
        key=Path(os.environ["PORTHOLE_UPLOAD_KEY"].strip()).expanduser(),
        bw_kib=bw_kib,
    )


def collector_busy(held):
    """Whether the recorder's queue is under pressure, as wait_for_collector judges it (held:
    the caller already holds back, which lowers the level at which it may go on)."""
    try:
        row = json.loads(Path("/dev/shm/porthole_collector_pressure.json").read_text())
        os.kill(int(row["pid"]), 0)
        stale = time.time() - float(row["updated_at"]) > 2
        pressure = int(row.get("queue_pending", 0))
        return bool(row.get("active", True)) and (stale or pressure >= (9 if held else 32))
    except (OSError, ValueError, KeyError, json.JSONDecodeError):
        return False


class RawUploader:
    """Send every finished recording to <PORTHOLE_RAW_DIR>/<date>/<run>/ on the server, as it is
    on the SSD, oldest first, and the date folders' CSVs whenever they change.

    Started with the analysis of the first new frame, so only once the collector records with
    PTP ready; a run in which nothing was recorded (closed before that) is not sent. rsync resumes
    a run cut short and leaves out the files the server already has; runs sent completely are
    listed in RAW_DONE and not looked at again. A run that fails on its own is tried again later
    (RAW_RETRY_MAX_SEC) while the others go on. rsync is held while live detections wait to be
    sent or the recorder's queue is under pressure, runs at the analysis' low CPU and disk
    priority, and is killed with this process (setpriv --pdeathsig), held or not.
    """

    def __init__(self, root, detections):
        self.root, self.detections = Path(root), detections
        self.stop_event = threading.Event()
        self.unreachable = False
        self.thread = threading.Thread(target=self.run, name="raw-upload", daemon=True)
        try:
            self.options = raw_upload_options()
        except ValueError as exc:
            print(f"[RAW] recordings are not sent: {exc}", flush=True)
            return
        self.thread.start()

    def finished_runs(self, done):
        """(key, path) of the finished recordings not sent yet, oldest first, once settled."""
        for run in discover(self.root):
            key = run.relative_to(self.root).as_posix()
            if key in done:
                continue
            meta = run / "meta/run_meta.jsonl"
            try:
                if time.time() - meta.stat().st_mtime < RAW_SETTLE_SEC:
                    continue
                with meta.open("rb") as stream:
                    finished = any(json.loads(line).get("event") == "run_finished"
                                   for line in stream if line.endswith(b"\n") and line.strip())
            except (OSError, ValueError):
                finished = False
            if finished:
                yield key, run

    @staticmethod
    def recorded(run):
        """Whether the collector saved camera frames or LiDAR files in the run."""
        return any((run / name).is_file() and (run / name).stat().st_size
                   for name in ("frames/frames.jsonl", "lidar/pcaps.jsonl"))

    def rsync(self, source):
        """rsync one path (<root>/./<date>/...) to the server folder, held while it should wait:
        (exit status, GB sent, last error line), or (None, None, "") when stopping."""
        ssh = shlex.join(["ssh", "-i", str(self.options.key), "-o", "BatchMode=yes",
                          "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=10",
                          "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=4"])
        command = ["setpriv", "--pdeathsig", "KILL",
                   "rsync", "-rt", "--omit-dir-times", "--relative", "--partial-dir=.rsync-partial",
                   "--timeout=600", f"--bwlimit={self.options.bw_kib}", "--stats", "-e", ssh, source,
                   f"{self.options.user}@{self.options.host}:{self.options.destination}/"]
        with tempfile.TemporaryFile("w+") as output:
            proc = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=output,
                                    stderr=subprocess.STDOUT, text=True)
            held = False
            try:
                while proc.poll() is None:
                    if self.stop_event.is_set():
                        return None, None, ""
                    hold = (self.detections is not None and self.detections.waiting()) or collector_busy(held)
                    if hold != held:
                        proc.send_signal(signal.SIGSTOP if hold else signal.SIGCONT)
                        held = hold
                    self.stop_event.wait(0.5)
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGCONT)
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
            output.seek(0)
            text = output.read()
        sent = re.search(r"Total bytes sent: ([\d,]+)", text)
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        error = "" if proc.returncode == 0 else next(
            (line for line in reversed(lines) if line.startswith(("rsync", "ssh"))), lines[-1] if lines else "")
        return proc.returncode, None if sent is None else int(sent.group(1).replace(",", "")) / 1e9, error

    def reach(self, status, error):
        """Note whether the server answered (status of an rsync); False when it cannot be reached."""
        if status in RAW_UNREACHABLE:
            if not self.unreachable:
                print(f"[RAW] server not reachable (rsync exit {status}: {error}); "
                      f"trying again every {RAW_POLL_SEC} s", flush=True)
            self.unreachable = True
            return False
        if self.unreachable:
            print("[RAW] server reachable again", flush=True)
        self.unreachable = False
        return True

    def send_runs(self, done, failures, retry_at):
        """Send the finished runs not sent yet; False when the server cannot be reached."""
        for key, run in self.finished_runs(done):
            if self.stop_event.is_set():
                return False
            if time.monotonic() < retry_at.get(key, 0):
                continue
            if self.recorded(run):
                started = time.monotonic()
                status, gb, error = self.rsync(f"{self.root}/./{key}")
                if status is None or not self.reach(status, error):
                    return False
                if status:
                    failures[key] = failures.get(key, 0) + 1
                    wait = min(RAW_POLL_SEC * 2 ** (failures[key] - 1), RAW_RETRY_MAX_SEC)
                    retry_at[key] = time.monotonic() + wait
                    print(f"[RAW] {key} not sent completely (rsync exit {status}: {error}); "
                          f"trying it again in {wait:.0f} s", flush=True)
                    continue
                print(f"[RAW] sent {key} in {time.monotonic() - started:.0f} s"
                      + ("" if gb is None else f" ({gb:.2f} GB)"), flush=True)
            with RAW_DONE.open("a") as stream:
                stream.write(key + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            done.add(key)
        return True

    def send_csvs(self, done, sent):
        """The date folders' CSVs of the sent runs, again whenever one changes (the analysis can
        list detections after their run was sent)."""
        for date in sorted({key.split("/")[0] for key in done}):
            try:
                stat = (self.root / date / DETECTIONS_CSV).stat()
            except FileNotFoundError:
                continue
            if sent.get(date) == (stat.st_size, stat.st_mtime_ns):
                continue
            status, _, error = self.rsync(f"{self.root}/./{date}/{DETECTIONS_CSV}")
            if status is None or not self.reach(status, error):
                return
            if status == 0:
                sent[date] = (stat.st_size, stat.st_mtime_ns)

    def run(self):
        import fcntl

        print(f"[RAW] recordings -> {self.options.user}@{self.options.host}:"
              f"{self.options.destination}/<date>/<run>", flush=True)
        try:
            done = set(RAW_DONE.read_text().split())
        except OSError:
            done = set()
        RAW_DONE.parent.mkdir(exist_ok=True)
        failures, retry_at, sent_csv = {}, {}, {}
        with RAW_LOCK.open("a") as lock:
            while not self.stop_event.is_set():
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:  # another sender (e.g. one started by hand) is running
                    self.stop_event.wait(RAW_POLL_SEC)
                    continue
                try:
                    if self.send_runs(done, failures, retry_at):
                        self.send_csvs(done, sent_csv)
                except Exception as exc:  # e.g. a run deleted meanwhile: look again later
                    print(f"[RAW] {type(exc).__name__}: {exc}; trying again in {RAW_POLL_SEC} s", flush=True)
                finally:
                    fcntl.flock(lock, fcntl.LOCK_UN)
                self.stop_event.wait(RAW_POLL_SEC)

    def close(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=6)


class JsonlTail:

    def __init__(self, path):
        self.path, self.position, self.rows = (Path(path), 0, [])
        self.identity = None

    def read(self):
        if not self.path.exists():
            return self.rows
        stat = self.path.stat()
        identity = (stat.st_dev, stat.st_ino)
        if self.identity is not None and (
            identity != self.identity or stat.st_size < self.position
        ):
            raise RuntimeError(f"Source manifest replaced or truncated: {self.path}")
        self.identity = identity
        with self.path.open("rb") as stream:
            stream.seek(self.position)
            while True:
                line = stream.readline()
                if not line.endswith(b"\n"):
                    break
                if line.strip():
                    self.rows.append(json.loads(line))
                self.position = stream.tell()
        return self.rows


class Run:

    def __init__(self, path):
        self.path = Path(path)
        self.frames = JsonlTail(self.path / "frames/frames.jsonl")
        self.events = JsonlTail(self.path / "meta/run_meta.jsonl")
        self.pcap_dir = self.path / "lidar"
        self.pcaps = JsonlTail(self.pcap_dir / "pcaps.jsonl")
        self.closed = False
        self.validated_rows = 0
        self.bad_pcaps = set()

    def refresh(self, with_frames=True):
        rows = self.frames.read() if with_frames else []
        for i in range(self.validated_rows, len(rows)):
            if i and int(rows[i]["frame_index"]) <= int(rows[i - 1]["frame_index"]):
                raise ValueError(f"Unordered or duplicate source frame IDs: {self.path}")
        self.validated_rows = len(rows)
        self.pcaps.read()
        self.closed = any((row.get("event") == "run_finished" for row in self.events.read()))
        if self.closed and with_frames:
            self.frames.read()
            self.pcaps.read()

    def pcap_records(self):
        """The listed PCAPs; one that is gone or not the listed size is left out (said once), so
        the frames it covers go without LiDAR instead of the analysis stopping on every start."""
        for row in self.pcaps.rows:
            path = Path(row["path"])
            if not path.is_absolute():
                path = self.pcap_dir / path
            try:
                if not path.is_file():
                    raise FileNotFoundError("missing")
                if "disk_bytes" in row and path.stat().st_size != int(row["disk_bytes"]):
                    raise ValueError(f"{path.stat().st_size} bytes, listed {row['disk_bytes']}")
            except (OSError, ValueError) as exc:
                if path not in self.bad_pcaps:
                    self.bad_pcaps.add(path)
                    print(f"[ANALYSIS] listed PCAP left out ({exc}): {path}", flush=True)
                continue
            yield dict(row, path=str(path))


def ready_frames(run, count, pcaps, allow_tail):
    watermark = max((float(p["last_timestamp"]) for p in pcaps), default=float("-inf"))
    now, now_monotonic = time.time(), time.monotonic()
    selected = []
    for row in run.frames.rows[count:]:
        timestamp = frame_time(row)
        # How long ago the collector got the frame, on the shared monotonic clock (no clock step
        # moves it); the capture time against the host clock only for rows without it.
        arrived = row.get("arrival_monotonic")
        age = now_monotonic - float(arrived) if arrived is not None else now - timestamp
        if not allow_tail and (age < PACE_DELAY_SEC or (
                timestamp - OFFSET_SEC + SCAN_SEARCH_HALF_WINDOW_SEC > watermark and age < LIDAR_WAIT_MAX_SEC)):
            break
        selected.append(row)
        if len(selected) >= BATCH_SIZE:
            break
    return selected


def count_manifest(path, entry):
    """Count a disk-backed queue without retaining future frame dictionaries."""
    stat = path.stat()
    identity = [stat.st_dev, stat.st_ino]
    if entry.get("manifest_identity", identity) != identity or stat.st_size < entry.get(
        "manifest_position", 0
    ):
        raise RuntimeError(f"Queue manifest replaced or truncated: {path}")
    entry["manifest_identity"] = identity
    if stat.st_size == entry.get("manifest_position", 0):
        return
    with path.open("rb") as stream:
        stream.seek(entry.get("manifest_position", 0))
        last = None
        while True:
            line = stream.readline()
            if not line.endswith(b"\n"):
                break
            entry["manifest_position"] = stream.tell()
            if line.strip():
                entry["recorded_rows"] = entry.get("recorded_rows", 0) + 1
                last = line
        if last is not None:
            row = json.loads(last)
            entry["last_recorded_timestamp"] = float(row.get("capture_timestamp", row["timestamp"]))


def source_signature(run):
    if not run.frames.rows:
        return None
    first = run.frames.rows[0]
    return hashlib.sha256(json.dumps(first, sort_keys=True).encode()).hexdigest()


def wait_for_collector(stopping):
    """Yield CPU work while the recorder's queue is under pressure."""
    path = Path("/dev/shm/porthole_collector_pressure.json")
    paused = False
    while not stopping():
        try:
            row = json.loads(path.read_text())
            os.kill(int(row["pid"]), 0)
            active = row.get("active", True)
            pressure = int(row.get("queue_pending", 0))
            stale = time.time() - float(row["updated_at"]) > 2
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            return
        if not active or (not stale and pressure < (9 if paused else 32)):
            return
        if not paused:
            print(f"[ANALYSIS] waiting for collector: queue={pressure}", flush=True)
        paused = True
        time.sleep(0.25)
    raise InterruptedError("Live analysis stopping")


_DISCOVERED = {}  # frames.jsonl -> (inode, PTP recording or not, run folder), once its first row is complete


def discover(root):
    """PTP recordings under root/YYYYMMDD/<run>; others (e.g. new_data) are skipped.

    The loop asks twice a second and the SSD keeps every run, so a manifest's first row, which
    decides and never changes, is read only once per file (a replaced file is read again)."""
    result = set()
    for path in root.glob("*/*/frames/frames.jsonl"):
        try:
            inode = path.stat().st_ino
        except FileNotFoundError:  # deleted while listing
            continue
        known = _DISCOVERED.get(path)
        if known is None or known[0] != inode:
            try:
                with path.open("rb") as stream:
                    first = stream.readline()
            except FileNotFoundError:  # deleted while listing
                continue
            run = path.parent.parent.resolve()
            if first.endswith(b"\n") and first.strip():
                try:
                    ptp = json.loads(first).get("timestamp_source") == "camera_ptp_rtcp_utc"
                except (ValueError, AttributeError):
                    # A damaged first row (a power cut or a storage fault) would otherwise stop
                    # every start here: that run is left out, the others are analysed.
                    print(f"[ANALYSIS] first row of {path} is unreadable; that run is skipped", flush=True)
                    ptp = False
                known = (inode, ptp, run)
                _DISCOVERED[path] = known
            else:
                known = (inode, True, run)  # no first row yet: taken, and read again next time
        if known[1]:
            result.add(known[2])
    return sorted(result, key=lambda p: str(p))


def run_service(root=ROOT, upload=True, max_frames=0):
    """Analyse the frames written from the start on until stopped (max_frames: tests).

    Frames already in frames.jsonl at the start, e.g. the ones a power cut left
    unanalysed, are skipped, and nothing carries over to the next start. The boundary
    is each manifest's size at the start, so it does not depend on any clock.
    """
    import fcntl
    from contextlib import ExitStack, closing

    root = Path(root).resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"Recording root is unavailable: {root}")
    if str(root).startswith("/mnt/ssd/") and (not os.path.ismount("/mnt/ssd")):
        raise RuntimeError("SSD is not mounted")
    start_bytes = {}  # run -> size of its frames.jsonl at the start
    for manifest in root.glob("*/*/frames/frames.jsonl"):
        try:
            start_bytes[str(manifest.parent.parent.relative_to(root))] = manifest.stat().st_size
        except FileNotFoundError:
            pass
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    with ExitStack() as resources:
        lock = resources.enter_context(open("/tmp/porthole_live_pothole.lock", "a"))
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(sig, stop)
            resources.callback(signal.signal, sig, previous)
        shutil.rmtree(UPLOAD_STAGING, ignore_errors=True)  # frames an earlier start left unsent
        log = resources.enter_context(closing(DetectionLog(root)))
        # The recorder always comes first: lowest CPU share and idle-class disk I/O.
        os.nice(10)
        if shutil.which("ionice"):
            subprocess.run(["ionice", "-c", "3", "-p", str(os.getpid())], check=False)
        processor = uploader = raw = None
        runs, entries = ({}, {})
        boundary_run = None
        processed = 0
        started = time.monotonic()
        last_status = 0
        previous_pending = None
        source_finished_at = None
        print(
            f"[LIVE] root={root} upload={upload}; frames written from now on; "
            f"detections: {root}/<date>/{DETECTIONS_CSV}",
            flush=True,
        )
        try:
            while not stopping:
                paths = discover(root)
                for path in paths:
                    key = str(path.relative_to(root))
                    if key not in entries:
                        # Taken up once frames are written after the start.
                        try:
                            size = (path / "frames/frames.jsonl").stat().st_size
                        except FileNotFoundError:
                            continue
                        if size <= start_bytes.get(key, 0):
                            continue
                        entries[key] = dict(
                            processed_rows=None,  # set when the run is first read, below
                            recorded_rows=0,
                            complete=False,
                            first_frame_sha256=None,
                        )
                    if not entries[key]["complete"]:
                        count_manifest(path / "frames/frames.jsonl", entries[key])
                active_keys = [key for key in sorted(entries) if not entries[key]["complete"]]
                for key in list(runs):
                    if key not in active_keys[:2]:
                        del runs[key]
                for key in active_keys[:2]:
                    runs.setdefault(key, Run(root / key))
                for key, run in runs.items():
                    run.refresh()
                    entry = entries[key]
                    count_manifest(run.frames.path, entry)
                    first_hash = source_signature(run)
                    if entry["first_frame_sha256"] not in (None, first_hash):
                        raise ValueError(f"Source recording was replaced: {run.path}")
                    entry["first_frame_sha256"] = first_hash
                    if entry["processed_rows"] is None:
                        # Rows already written at the start are skipped, not analysed.
                        with run.frames.path.open("rb") as stream:
                            head = stream.read(start_bytes.get(key, 0))
                        entry["processed_rows"] = sum(
                            1 for line in head.split(b"\n")[:-1] if line.strip()
                        )
                extra = []
                if runs:
                    first_path = runs[min(runs)].path
                    before = [p for p in paths if str(p) < str(first_path)]
                    if before:
                        previous_path = before[-1]
                        if boundary_run is None or boundary_run.path != previous_path:
                            boundary_run = Run(previous_path)
                        boundary_run.refresh(with_frames=False)
                        extra = list(boundary_run.pcap_records())[-2:]
                pcaps = sorted(
                    extra + [p for run in runs.values() for p in run.pcap_records()],
                    key=lambda p: float(p["first_timestamp"]),
                )
                closed = (
                    bool(runs)
                    and len(active_keys) <= len(runs)
                    and all((run.closed for run in runs.values()))
                )
                source_finished_at = source_finished_at or time.monotonic() if closed else None
                allow_tail = (
                    source_finished_at is not None
                    and time.monotonic() - source_finished_at >= POLL_SECONDS
                )
                did_work = short_batch = False
                for key in sorted(runs):
                    run, entry = (runs[key], entries[key])
                    count = entry["processed_rows"]
                    if count == len(run.frames.rows):
                        entry["complete"] = run.closed
                        continue
                    ready = ready_frames(run, count, pcaps, allow_tail and run.closed)
                    if not ready:
                        break
                    wait_for_collector(lambda: stopping)
                    if processor is None:
                        uploader = UploadWorker() if upload else None
                        processor = LiveProcessor(log, uploader)
                        # The collector records (its PTP is ready): the recordings go out from now.
                        raw = RawUploader(root, uploader) if upload else None
                    processor.prune_pcaps(pcaps)
                    processor.refresh_run(run.path)
                    if max_frames:
                        ready = ready[: max_frames - processed]
                    short_batch = len(ready) < BATCH_SIZE  # caught up with PACE_DELAY_SEC
                    stream = processor.infer(run.path, ready)
                    reported = processor.analyse(run.path, key, stream, pcaps, lambda: stopping)
                    try:
                        for _ in reported:
                            entry["processed_rows"] += 1
                            processed += 1
                            did_work = True
                            if max_frames and processed >= max_frames:
                                stopping = True
                                break
                    finally:
                        reported.close()
                        stream.close()
                    break
                pending = sum(
                    entry["recorded_rows"] - entry["processed_rows"]
                    for entry in entries.values()
                    if entry["processed_rows"] is not None
                )
                oldest = next(
                    (
                        frame_time(run.frames.rows[entries[key]["processed_rows"]])
                        for key, run in sorted(runs.items())
                        if entries[key]["processed_rows"] < len(run.frames.rows)
                    ),
                    None,
                )
                oldest_age = 0 if oldest is None else max(0, time.time() - oldest)
                now = time.monotonic()
                if now - last_status >= 5 or stopping:
                    print(
                        f"[ANALYSIS {time.strftime('%H:%M:%S')}] processed={processed} pending={pending} avgFPS={processed / max(0.001, now - started):.2f} oldestAge={oldest_age:.1f}s uploadQ={0 if uploader is None else uploader.pending()}",
                        flush=True,
                    )
                    if previous_pending is not None and pending > previous_pending + 150:
                        print(
                            "[ANALYSIS] backlog increasing; recordings are retained and frames are not skipped",
                            flush=True,
                        )
                    previous_pending, last_status = (pending, now)
                if not did_work:
                    time.sleep(POLL_SECONDS)
                elif short_batch:
                    # Let the next frames come due, so that they go to the NPU in batches of about
                    # 15, not one or two each pass of this loop (each pass also rereads the manifests).
                    time.sleep(POLL_SECONDS / 2)
        except InterruptedError:
            if not stopping:
                raise
        finally:
            if processor:
                processor.close()
            if raw:
                raw.close()
            if uploader:
                uploader.close()
            shutil.rmtree(UPLOAD_STAGING, ignore_errors=True)
            print(f"[LIVE] stopped; processed={processed}", flush=True)


# The SSH receiver is built once, from this file as it was when the process started:
# auto_update.py may replace live_pothole.py while the analysis runs, and inspect would then
# read the new file at the old line numbers.
SERVER_TRANSPORT_SOURCE = server_transport_source()


if __name__ == "__main__":
    run_service()
