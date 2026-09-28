"""Standalone live camera/LiDAR analysis for Desktop/ptp_pothole recordings.

Run: python3 -u main_live.py
Input: /mnt/ssd/porthole_runs/YYYYMMDD/run/{frames,lidar,gps,meta}
Every committed frame is processed in FIFO order; checkpoints and upload retries
survive restarts. No other project Python file is imported or executed.
Model/calibration files and installed NumPy/OpenCV/DEEPX runtime are data/runtime
dependencies. PTP synchronizes clocks; GPS speed, when available, compensates
vehicle motion between LiDAR packets and the image (--motion auto, the default).
--motion required drops LiDAR depth without GPS speed; --motion off never compensates.
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
from dataclasses import dataclass
from dataclasses import field
from datetime import datetime
from datetime import timedelta
from datetime import timezone
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace
from typing import Iterator
from typing import Optional
from zoneinfo import ZoneInfo
import csv
import cv2
import dx_engine
import hashlib
import json
import math
import numpy as np
import re
import select
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import zlib
import argparse

POTHOLE_DEPTH_M = 0.005

PROJECT = Path(__file__).resolve().parent

OFFSET_SEC = 0.0

CAMERA_TIMING_MODE = "recorded"

LIDAR_MOTION_ENABLED = True

ALIGNMENT_PROFILE_ID = "ptp_live_recorded_utc_v1"

MODEL_PATH = PROJECT / "best_seg.dxnn"

FUSION_OPENCV_THREADS = 2  # 추적 CPU 병렬화; 별도 추론 프로세스의 기본 스레드는 1개.

CAMERA_CALIBRATION = PROJECT / "camera_calib_best_effort_v30.json"

CAMERA_SPATIAL_CALIBRATION = {
    "id": "stationary_trial_20260914",
    "R_sensor_to_cam": [
        [-0.9994184668078344, -0.03355021647665391, 0.006091894438319107],
        [-0.005763490804555829, -0.009876448571861112, -0.9999346168311972],
        [0.03360818913931191, -0.9993882322390921, 0.00967733872504181],
    ],
    "t_cam": [-0.028875, 0.139125, -0.11437500000000003],
}

LIDAR_CALIBRATION = PROJECT / "XT32_Angle_Correction_File.csv"

SAVE_IMAGES = True

POTHOLE_MIN_DEPTH_M = POTHOLE_DEPTH_M

CONFIDENCE_THRESHOLD = 0.25

SCAN_SEARCH_HALF_WINDOW_SEC = 0.1

MAX_SCAN_CENTER_DELTA_SEC = 0.055

SCAN_GAP_SEC = 0.005


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
        return "Upload requires exactly one JPG, one JSON and one PCAP; retained locally"
    if any(not isinstance(name, str) for name in files):
        return "Invalid artifact filename"
    if {Path(name).suffix for name in files} != {".jpg", ".json", ".pcap"}:
        return "Upload only accepts JPG, JSON and PCAP"
    if len({Path(name).stem for name in files}) != 1:
        return "JPG, JSON and PCAP must have the same basename"
    pcap = next(info for name, info in files.items() if Path(name).suffix == ".pcap")
    if not isinstance(pcap, dict) or type(pcap.get("bytes")) is not int or pcap["bytes"] <= 24:
        return "Empty PCAP cannot be uploaded; retained locally"
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


def prepare_upload(source, manifest, compression_level=1):
    """Complete source I/O, hashes and optional lossless encoding before send time."""
    begin = time.monotonic_ns()
    manifest = dict(manifest, _receive_token=uuid.uuid4().hex)
    manifest["files"] = {name: dict(info) for name, info in manifest["files"].items()}
    validate_manifest(manifest)
    aliases = content_aliases(manifest["files"])
    expected = sum(info["bytes"] for name, info in manifest["files"].items() if name not in aliases)
    if type(compression_level) is not int or not 0 <= compression_level <= 9:
        raise ValueError("Invalid upload compression level")
    encoded_files, segments = [], []
    for name, info in manifest["files"].items():
        raw = b"".join(payload_chunks(Path(source), {name: info}, aliases))
        if name in aliases:
            continue  # Alias content was verified but is sent only once.
        encoded, encoding = raw, "identity"
        # JPEG already has compression. Recompressing it wasted most CPU time.
        if (
            compression_level
            and Path(name).suffix.lower() in (".json", ".pcap")
            and len(raw) >= 1024
        ):
            compressed = zlib.compress(raw, compression_level)
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
    def __init__(self, options, relative, command=None, timeout=60):
        self.options, self.relative, self.command, self.timeout = (
            options,
            relative,
            command,
            timeout,
        )
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
        command = self.command or [
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
            + shlex.quote(server_transport_source())
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
            prepared = prepare_upload(
                source, manifest, getattr(self.options, "compression_level", 1)
            )
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

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.close()


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
    redirects, is retried.
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
    """Use JSON intrinsics and configured extrinsics to project sensor XYZ."""

    def __init__(self, calibration_file):
        self.path = Path(calibration_file).expanduser().resolve()
        data = json.loads(self.path.read_text(encoding="utf-8"))
        model = str(data.get("model", ""))
        if model and (not model.startswith("fisheye_equidistant")):
            raise ValueError(f"unsupported camera model: {model}")
        self.fx, self.fy = (float(data["fx"]), float(data["fy"]))
        self.cx, self.cy = (float(data["cx"]), float(data["cy"]))
        self.k1, self.k2 = (float(data["D"][0]), float(data["D"][1]))
        spatial = data if CAMERA_SPATIAL_CALIBRATION is None else CAMERA_SPATIAL_CALIBRATION
        self.R = np.asarray(spatial["R_sensor_to_cam"], dtype=np.float64)
        self.t = np.asarray(spatial["t_cam"], dtype=np.float64).reshape(3)
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
            "id": spatial.get("id", "calibration_file"),
            "R_sensor_to_cam": self.R.tolist(),
            "t_cam": self.t.tolist(),
        }

    def convert_3D_to_camera_coords(self, sensor_xyz, print_info=False):
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

    def convert_3D_to_2D(self, sensor_xyz, print_info=False):
        return self.project(sensor_xyz)[0]


model_infer__CLASS_NAMES = ("Crack", "Pothole")

SUSPECT_CLASS_IDS = frozenset((0, 1))

MASK_COEFFICIENTS = 32

DETECTION_CHANNELS = 4 + len(model_infer__CLASS_NAMES) + MASK_COEFFICIENTS


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
    track_id: Optional[str] = None
    occurrence_index: int = 1
    mask_path: Optional[str] = None


def letterbox_rgb_uint8(
    image_bgr: np.ndarray, size: int = 640
) -> tuple[np.ndarray, LetterboxContext]:
    """Return contiguous NHWC RGB uint8 without keeping an original-image copy."""
    height, width = image_bgr.shape[:2]
    scale = min(size / width, size / height)
    resized_width = max(1, min(size, int(round(width * scale))))
    resized_height = max(1, min(size, int(round(height * scale))))
    resized = cv2.resize(image_bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x = (size - resized_width) // 2
    pad_y = (size - resized_height) // 2
    canvas[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized
    input_nhwc = np.ascontiguousarray(canvas[:, :, ::-1][None, ...])
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
    confidences = class_scores[class_ids, np.arange(class_scores.shape[1])]
    selected = np.flatnonzero(
        (confidences >= confidence_threshold)
        & np.isin(class_ids, np.fromiter(selected_class_ids, dtype=np.int64))
    )
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
        mask = restored >= threshold
        x1, y1, x2, y2 = np.rint(detection.box_xyxy).astype(int)
        cropped = np.zeros_like(mask)
        cropped[max(0, y1) : y2 + 1, max(0, x1) : x2 + 1] = mask[
            max(0, y1) : y2 + 1, max(0, x1) : x2 + 1
        ]
        detection.mask = cropped


class DXNNDetector:
    def __init__(self, model_path, confidence=0.25, nms_iou=0.45, mask_threshold=0.5,
                 class_names=model_infer__CLASS_NAMES, selected_class_ids=SUSPECT_CLASS_IDS):
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
        self.core_totals = [dict(completed=0, npu_us=0, latency_us=0) for _ in self.engines]
        self.core_report_at = time.monotonic()
        self.core_report_totals = [dict(row) for row in self.core_totals]
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

        self.metadata = dict(
            file_name=self.model_path.name, sha256=sha(self.model_path), runtime="DXRT",
            python_runtime_version=getattr(dx_engine, "__version__", None),
            class_names=list(self.class_names), class_filter=sorted(self.selected_class_ids),
            input_shape=[1, 640, 640, 3], input_layout="NHWC", input_dtype="uint8",
            color_order="RGB", resize="centered_letterbox", pad_value=114,
            confidence_threshold=self.confidence, nms_iou=self.nms_iou,
            mask_threshold=self.mask_threshold,
        )

    def decode(self, outputs, context):
        detections = decode_detections(outputs[0], context, self.confidence, self.nms_iou,
                                       self.class_names, self.selected_class_ids)
        attach_masks(detections, outputs[1], context, self.mask_threshold)
        return detections

    def detect(self, frame):
        tensor, context = letterbox_rgb_uint8(frame)
        outputs = self.engine.run(tensor)
        self._record_completion(self.engine)
        return self.decode(outputs, context)

    def iter_detect(self, records, load_image, wait_for_collector=lambda: None):
        """Keep at most three native requests in flight; yield in source order.

        Pending entries own input tensors until wait() completes. Refill before
        yielding, so NPU execution overlaps the caller's depth/metadata work.
        """
        source = iter(records)
        pending = deque()
        exhausted = False
        next_engine = 0

        def submit_one():
            nonlocal exhausted, next_engine
            try:
                record = next(source)
            except StopIteration:
                exhausted = True
                return
            wait_for_collector()
            image = load_image(record)
            tensor, context = letterbox_rgb_uint8(image)
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
                self._record_completion(engine)
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

    def _record_completion(self, engine):
        row = self.core_totals[self.engines.index(engine)]
        row["completed"] += 1
        row["npu_us"] += int(engine.get_npu_inference_time())
        row["latency_us"] += int(engine.get_latency())

    def core_metrics(self):
        now = time.monotonic()
        elapsed = now - self.core_report_at
        result = []
        for core, (row, before) in enumerate(zip(self.core_totals, self.core_report_totals)):
            count = row["completed"] - before["completed"]
            result.append(
                dict(
                    core=core,
                    **row,
                    interval_completed=count,
                    interval_seconds=elapsed,
                    fps=count / elapsed if elapsed else 0,
                    npu_ms=((row["npu_us"] - before["npu_us"]) / count / 1000 if count else None),
                    latency_ms=(
                        (row["latency_us"] - before["latency_us"]) / count / 1000 if count else None
                    ),
                )
            )
        self.core_report_at = now
        self.core_report_totals = [dict(row) for row in self.core_totals]
        return result

    def dispose(self):
        for engine in self.engines:
            if hasattr(engine, "dispose"):
                engine.dispose()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.dispose()


VERSION = "road_flow_iou_v2"

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
    first_frame: int
    last_frame: int
    observations: int = 0
    observed_depth_frames: int = 0
    unobserved_frames: int = 0
    confirmed: bool = False
    confirmed_frame: int | None = None
    confirmed_depth_m: float | None = None
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(4))
    last_detection_box: np.ndarray | None = None

    def summary(self, closed_frame, reason):
        return dict(
            track_id=self.track_id,
            class_id=self.class_id,
            first_frame=self.first_frame,
            last_frame=self.last_frame,
            closed_frame=closed_frame,
            close_reason=reason,
            observations=self.observations,
            observed_depth_frames=self.observed_depth_frames,
            unobserved_frames=self.unobserved_frames,
            confirmed=self.confirmed,
            confirmed_frame=self.confirmed_frame,
            confirmed_depth_m=self.confirmed_depth_m,
            state=(
                "confirmed"
                if self.confirmed
                else ("unobserved" if not self.observed_depth_frames else "observed_not_confirmed")
            ),
        )


class ObjectTracker:
    def __init__(self, **settings):
        unknown = set(settings) - set(DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown tracking settings: {unknown}")
        self.settings = {**DEFAULTS, **settings}
        if (
            not 0 < self.settings["association_iou"] <= 1
            or not 0 <= self.settings["max_missed_frames"] <= 30
        ):
            raise ValueError("Invalid association settings")
        self.active = {}
        self.next_id = 1
        self.previous = None
        self.previous_points = None
        self.last_frame = None
        self.last_motion = {}

    def configuration(self):
        return dict(
            version=VERSION,
            **self.settings,
            identity_inputs="camera_pixels_class_bbox_only",
            confirmation_policy="first_valid_frame_depth_pass_latched",
            scope="one_passage_per_replay_loop",
            unobserved_policy="retain_pending_then_report_separately_not_drop_gt",
        )

    def _motion(self, image, needed):
        height, width = image.shape[:2]
        scale = self.settings["image_width"] / width
        small = cv2.resize(
            image,
            (self.settings["image_width"], round(height * scale)),
            interpolation=cv2.INTER_AREA,
        )
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
        matrix = None
        detail = dict(method="no_motion", valid=False, inliers=0)
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
                        detail = dict(
                            method="road_fit_rejected",
                            valid=False,
                            inliers=count,
                            pairs=len(a),
                        )
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
                            detail = dict(
                                method="road_lk_homography",
                                valid=True,
                                inliers=count,
                                pairs=len(a),
                            )
        mask = np.zeros(gray.shape, np.uint8)
        h, w = gray.shape
        mask[int(0.12 * h) : int(0.74 * h), int(0.12 * w) : int(0.9 * w)] = 255
        self.previous_points = (
            cv2.goodFeaturesToTrack(gray, self.settings["feature_count"], 0.01, 8, mask=mask)
            if needed
            else None
        )
        self.previous = gray
        self.last_motion = detail
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

    def update(self, frame, image, detections, observations=None):
        """Return a row for every detection plus closed tracks; no future data."""
        frame = int(frame)
        if self.last_frame is not None and frame <= self.last_frame:
            raise ValueError("New passage/replay requires a new tracker; frame IDs must increase")
        if observations is not None and len(detections) != len(observations):
            raise ValueError("Tracking observation count mismatch")
        parsed = [(int(d["class_id"]), bounds(d)) for d in detections]
        if any((cls not in (0, 1) for (cls, _) in parsed)):
            raise ValueError("Unsupported tracking class")
        closed = []
        for tid, track in list(self.active.items()):
            if frame - track.last_frame > self.settings["max_missed_frames"] + 1:
                closed.append(track.summary(frame, "missed_frame_limit"))
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
                track = Track(self.next_id, cls, box.copy(), frame, frame)
                self.next_id += 1
                self.active[track.track_id] = track
                score, gap = (None, 0)
            track.last_detection_box = box.copy()
            track.observations += 1
            was_confirmed = track.confirmed
            observation = observations[j] if observations is not None else None
            depth = None if observation is None else observation.get("max_depth_m")
            reason = None if observation is None else observation.get("reason")
            valid_depth = (
                cls == 1
                and depth is not None
                and math.isfinite(float(depth))
                and (reason in ("depth_pass", "depth_below_threshold"))
            )
            if valid_depth:
                track.observed_depth_frames += 1
            elif cls == 1:
                track.unobserved_frames += 1
            if observation is not None and bool(observation.get("accepted")):
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
        return (rows, closed)

    def finish(self):
        rows = [track.summary(self.last_frame, "passage_end") for track in self.active.values()]
        self.active.clear()
        return rows


KST = ZoneInfo("Asia/Seoul")

ALLOWED_DAMAGE_TYPES = {"road_damage", "pothole", "linear_crack"}

DAMAGE_TYPE_KO = {"pothole": "포트홀", "linear_crack": "크랙", "road_damage": "러팅"}

KMA_BASE_URL_ENV = "KMA_BASE_URL"

KMA_SERVICE_KEY_ENV = "KMA_SERVICE_KEY"

DEFAULT_KMA_BASE_URL = "https://apis.data.go.kr/1360000/VilageFcstInfoService_2.0"

KMA_ASOS_BASE_URL_ENV = "KMA_ASOS_BASE_URL"

KMA_ASOS_SERVICE_KEY_ENV = "KMA_ASOS_SERVICE_KEY"

DEFAULT_KMA_ASOS_BASE_URL = "https://apis.data.go.kr/1360000/AsosHourlyInfoService"

ASOS_STATIONS_PATH = Path(__file__).with_name("asos_stations.csv")


def _load_local_env(path: Optional[Path] = None) -> None:
    """Load a sibling .env without overriding explicitly set environment values.

    As in a shell, a later assignment in the file wins over an earlier one.
    """
    env_path = Path(path) if path is not None else Path(__file__).with_name(".env")
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


def artifact_stem(
    camera_timestamp: float, camera_index: int, damage_type: str, cluster_id: int
) -> str:
    damage_type = str(damage_type).strip().lower()
    if damage_type not in ALLOWED_DAMAGE_TYPES:
        raise ValueError(
            f"damage_type must be one of {sorted(ALLOWED_DAMAGE_TYPES)}, got {damage_type!r}"
        )
    instant = camera_time_kst(camera_timestamp)
    milliseconds = instant.microsecond // 1000
    time_part = instant.strftime("%Y%m%d_%H%M%S") + f"_{milliseconds:03d}"
    return f"{time_part}_{damage_type}_c{int(camera_index):08d}_d{int(cluster_id):03d}"


def _finite_float(value) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def write_damage_json(path: Path, payload: dict) -> None:
    path = Path(path)
    partial = path.with_suffix(path.suffix + ".partial")
    partial.unlink(missing_ok=True)
    partial.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    partial.replace(path)


def build_damage_payload(
    *,
    damage_type: str,
    camera_index: int,
    camera_timestamp: float,
    lidar_frame_id: int,
    lidar_timestamp: float,
    time_error_sec: float,
    cluster: dict,
    bbox: tuple[int, int, int, int],
    polygons_px: list[list[dict]],
    gps: dict,
    weather: dict,
    vehicle_type: str,
    sensor_mount_height_cm: float,
    sensor_mount_angles_xyz_deg: dict,
    source_image: str,
    json_name: str,
    image_name: str,
    sensor_mount_angle_convention: str = "",
    sensor_mount_angle_source: str = "",
) -> dict:
    damage_type = str(damage_type).strip().lower()
    if damage_type not in ALLOWED_DAMAGE_TYPES:
        raise ValueError(f"Unsupported damage type: {damage_type}")
    camera_dt = camera_time_kst(camera_timestamp)
    lidar_dt = camera_time_kst(lidar_timestamp)
    x0, y0, x1, y1 = (int(value) for value in bbox)
    compact_weather = {
        "source": weather.get("source"),
        "status": weather.get("status"),
        "condition": weather.get("condition"),
        "temperature_c": _finite_float(weather.get("temperature_c")),
        "humidity_pct": _finite_float(weather.get("humidity_pct")),
    }
    if weather.get("status") != "ok":
        compact_weather["reason"] = weather.get("reason")
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


def unavailable_weather(reason: str, source: str = "KMA_DATA_GO_KR_ULTRA_SHORT_NOWCAST") -> dict:
    return {"source": str(source), "status": "unavailable", "reason": str(reason)}


def _haversine_km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius_km = 6371.0088
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * radius_km * math.asin(math.sqrt(a))


def latlon_to_kma_grid(latitude_deg: float, longitude_deg: float) -> tuple[int, int]:
    """Convert WGS84 latitude/longitude to KMA DFS 5 km grid coordinates."""
    re_km = 6371.00877
    grid_km = 5.0
    slat1 = math.radians(30.0)
    slat2 = math.radians(60.0)
    olon = math.radians(126.0)
    olat = math.radians(38.0)
    xo = 43.0
    yo = 136.0
    sn = math.log(math.cos(slat1) / math.cos(slat2)) / math.log(
        math.tan(math.pi * 0.25 + slat2 * 0.5) / math.tan(math.pi * 0.25 + slat1 * 0.5)
    )
    sf = math.tan(math.pi * 0.25 + slat1 * 0.5) ** sn * math.cos(slat1) / sn
    ro = re_km / grid_km * sf / math.tan(math.pi * 0.25 + olat * 0.5) ** sn
    ra = re_km / grid_km * sf / math.tan(math.pi * 0.25 + math.radians(latitude_deg) * 0.5) ** sn
    theta = math.radians(longitude_deg) - olon
    if theta > math.pi:
        theta -= 2.0 * math.pi
    if theta < -math.pi:
        theta += 2.0 * math.pi
    theta *= sn
    nx = int(math.floor(ra * math.sin(theta) + xo + 0.5))
    ny = int(math.floor(ro - ra * math.cos(theta) + yo + 0.5))
    return (nx, ny)


PTY_CONDITIONS = {
    0: "no_precipitation",
    1: "rain",
    2: "rain_and_snow",
    3: "snow",
    5: "raindrop",
    6: "raindrop_and_snow_flurry",
    7: "snow_flurry",
}


def _numeric_observation(value):
    number = _finite_float(value)
    return number if number is not None else value


def _safe_api_error(exc: Exception) -> str:
    """Describe request failures without leaking a service key embedded in a URL."""
    if isinstance(exc, urllib.error.HTTPError):
        return f"HTTPError: HTTP {exc.code}"
    if isinstance(exc, urllib.error.URLError):
        return f"URLError: {exc.reason}"
    return f"{type(exc).__name__}: {exc}"


class KmaVilageClient:
    """Failure-tolerant getUltraSrtNcst client with grid/hour caching."""

    def __init__(
        self,
        service_key: Optional[str] = None,
        base_url: Optional[str] = None,
        timeout_sec: float = 15.0,
    ):
        self.service_key = (
            str(service_key).strip()
            if service_key is not None
            else os.environ.get(KMA_SERVICE_KEY_ENV, "").strip()
        )
        self.base_url = (
            str(base_url).strip()
            if base_url is not None
            else os.environ.get(KMA_BASE_URL_ENV, DEFAULT_KMA_BASE_URL).strip()
        ).rstrip("/")
        self.timeout_sec = float(timeout_sec)
        self._weather_cache: dict[tuple[str, str, int, int], dict] = {}

    @property
    def enabled(self) -> bool:
        return bool(self.service_key and self.base_url)

    def _request(self, params: dict) -> dict:
        query = urllib.parse.urlencode({**params, "serviceKey": self.service_key})
        request = urllib.request.Request(
            self.base_url + "/getUltraSrtNcst?" + query,
            headers={"User-Agent": "unicons-road-damage-export/2.0"},
        )
        with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
            return json.loads(response.read().decode("utf-8"))

    @staticmethod
    def _base_instant(camera_timestamp: float) -> datetime:
        instant = camera_time_kst(camera_timestamp)
        if instant.minute < 10:
            instant -= timedelta(hours=1)
        return instant.replace(minute=0, second=0, microsecond=0)

    def weather_for(self, camera_timestamp: float, latitude_deg, longitude_deg) -> dict:
        if not self.enabled:
            return unavailable_weather(f"{KMA_BASE_URL_ENV} or {KMA_SERVICE_KEY_ENV} is not set")
        latitude = _finite_float(latitude_deg)
        longitude = _finite_float(longitude_deg)
        if latitude is None or longitude is None:
            return unavailable_weather("GPS latitude/longitude is unavailable")
        instant = self._base_instant(camera_timestamp)
        nx, ny = latlon_to_kma_grid(latitude, longitude)
        base_date = instant.strftime("%Y%m%d")
        base_time = instant.strftime("%H00")
        cache_key = (base_date, base_time, nx, ny)
        cached = self._weather_cache.get(cache_key)
        if cached is not None:
            return dict(cached)
        try:
            document = self._request(
                {
                    "pageNo": 1,
                    "numOfRows": 1000,
                    "dataType": "JSON",
                    "base_date": base_date,
                    "base_time": base_time,
                    "nx": nx,
                    "ny": ny,
                }
            )
            response = document.get("response", {})
            header = response.get("header", {})
            result_code = str(header.get("resultCode", ""))
            if result_code != "00":
                message = str(header.get("resultMsg", "KMA request failed"))
                raise RuntimeError(f"KMA {result_code}: {message}")
            items = response.get("body", {}).get("items", {}).get("item", [])
            if isinstance(items, dict):
                items = [items]
            if not items:
                raise RuntimeError("KMA response contained no observation items")
            values = {str(item.get("category", "")): item.get("obsrValue") for item in items}
            pty_value = _finite_float(values.get("PTY"))
            pty_code = int(pty_value) if pty_value is not None else None
            first = items[0]
            payload = {
                "source": "KMA_DATA_GO_KR_ULTRA_SHORT_NOWCAST",
                "status": "ok",
                "grid": {"nx": nx, "ny": ny},
                "requested_base_at_kst": instant.isoformat(),
                "observed_at_kst": f"{first.get('baseDate', base_date)}{first.get('baseTime', base_time)}",
                "condition": PTY_CONDITIONS.get(pty_code, "unknown"),
                "precipitation_type_code": pty_code,
                "temperature_c": _numeric_observation(values.get("T1H")),
                "humidity_pct": _numeric_observation(values.get("REH")),
                "precipitation_1h": _numeric_observation(values.get("RN1")),
                "wind_direction_deg": _numeric_observation(values.get("VEC")),
                "wind_speed_m_s": _numeric_observation(values.get("WSD")),
                "wind_east_west_m_s": _numeric_observation(values.get("UUU")),
                "wind_north_south_m_s": _numeric_observation(values.get("VVV")),
            }
        except Exception as exc:
            payload = unavailable_weather(_safe_api_error(exc))
            payload["grid"] = {"nx": nx, "ny": ny}
            payload["requested_base_at_kst"] = instant.isoformat()
        self._weather_cache[cache_key] = payload
        return dict(payload)


def load_asos_stations(path: Optional[Path] = None) -> list[dict]:
    """Load the checked-in KMA ASOS station coordinate snapshot."""
    station_path = Path(path) if path is not None else ASOS_STATIONS_PATH
    stations = []
    with station_path.open("r", newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            station_id = _finite_float(row.get("station_id"))
            latitude = _finite_float(row.get("latitude_deg"))
            longitude = _finite_float(row.get("longitude_deg"))
            if station_id is None or latitude is None or longitude is None:
                continue
            stations.append(
                {
                    "id": int(station_id),
                    "name_en": str(row.get("station_name_en", "")).strip(),
                    "latitude_deg": latitude,
                    "longitude_deg": longitude,
                }
            )
    if not stations:
        raise RuntimeError(f"No usable ASOS stations in {station_path}")
    return stations


def nearest_asos_station(latitude_deg: float, longitude_deg: float, stations: list[dict]) -> dict:
    """Return a copy of the geographically nearest ASOS station."""
    station = min(
        stations,
        key=lambda item: _haversine_km(
            latitude_deg,
            longitude_deg,
            float(item["latitude_deg"]),
            float(item["longitude_deg"]),
        ),
    )
    result = dict(station)
    result["distance_km"] = _haversine_km(
        latitude_deg,
        longitude_deg,
        float(station["latitude_deg"]),
        float(station["longitude_deg"]),
    )
    return result


def _weather_condition_from_asos(item: dict) -> str:
    snow = max(
        float(_finite_float(item.get("dsnw")) or 0.0),
        float(_finite_float(item.get("hr3Fhsc")) or 0.0),
    )
    precipitation = _finite_float(item.get("rn"))
    cloud_cover = _finite_float(item.get("dc10Tca"))
    if snow > 0.0:
        return "snow"
    if precipitation is not None and precipitation > 0.0:
        return "rain"
    if cloud_cover is None:
        return "unknown"
    if cloud_cover >= 8.0:
        return "cloudy"
    if cloud_cover >= 3.0:
        return "partly_cloudy"
    return "clear"


class KmaAsosHistoricalClient:
    """ASOS hourly history selected by nearest station to capture GPS."""

    def __init__(
        self,
        service_key: Optional[str] = None,
        base_url: Optional[str] = None,
        stations_path: Optional[Path] = None,
        timeout_sec: float = 15.0,
    ):
        encoded_key = (
            str(service_key).strip()
            if service_key is not None
            else os.environ.get(KMA_ASOS_SERVICE_KEY_ENV, "").strip()
        )
        self.service_key = urllib.parse.unquote(encoded_key)
        self.base_url = (
            str(base_url).strip()
            if base_url is not None
            else os.environ.get(KMA_ASOS_BASE_URL_ENV, DEFAULT_KMA_ASOS_BASE_URL).strip()
        ).rstrip("/")
        self.timeout_sec = float(timeout_sec)
        self.stations_path = (
            Path(stations_path) if stations_path is not None else ASOS_STATIONS_PATH
        )
        self._stations: Optional[list[dict]] = None
        self._weather_cache: dict[tuple[str, str, int], dict] = {}

    @property
    def enabled(self) -> bool:
        return bool(self.service_key and self.base_url and self.stations_path.is_file())

    def _request(self, params: dict) -> dict:
        query = urllib.parse.urlencode({**params, "serviceKey": self.service_key})
        request = urllib.request.Request(
            self.base_url + "/getWthrDataList?" + query,
            headers={"User-Agent": "unicons-road-damage-export/2.0"},
        )
        with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
            return json.loads(response.read().decode("utf-8", errors="replace"))

    def stations(self) -> list[dict]:
        if self._stations is None:
            self._stations = load_asos_stations(self.stations_path)
        return self._stations

    def weather_for(self, camera_timestamp: float, latitude_deg, longitude_deg) -> dict:
        source = "KMA_DATA_GO_KR_ASOS_HOURLY"
        if not self.enabled:
            return unavailable_weather(
                f"{KMA_ASOS_BASE_URL_ENV} or {KMA_ASOS_SERVICE_KEY_ENV} is not set",
                source,
            )
        latitude = _finite_float(latitude_deg)
        longitude = _finite_float(longitude_deg)
        if latitude is None or longitude is None:
            return unavailable_weather("GPS latitude/longitude is unavailable", source)
        instant = camera_time_kst(camera_timestamp).replace(minute=0, second=0, microsecond=0)
        try:
            station = nearest_asos_station(latitude, longitude, self.stations())
        except Exception as exc:
            return unavailable_weather(_safe_api_error(exc), source)
        date = instant.strftime("%Y%m%d")
        hour = instant.strftime("%H")
        station_id = int(station["id"])
        cache_key = (date, hour, station_id)
        cached = self._weather_cache.get(cache_key)
        if cached is not None:
            return dict(cached)
        try:
            document = self._request(
                {
                    "pageNo": 1,
                    "numOfRows": 10,
                    "dataType": "JSON",
                    "dataCd": "ASOS",
                    "dateCd": "HR",
                    "startDt": date,
                    "startHh": hour,
                    "endDt": date,
                    "endHh": hour,
                    "stnIds": station_id,
                }
            )
            response = document.get("response", {})
            header = response.get("header", {})
            result_code = str(header.get("resultCode", ""))
            if result_code != "00":
                message = str(header.get("resultMsg", "KMA ASOS request failed"))
                raise RuntimeError(f"KMA ASOS {result_code}: {message}")
            items = response.get("body", {}).get("items", {}).get("item", [])
            if isinstance(items, dict):
                items = [items]
            if not items:
                raise RuntimeError("KMA ASOS response contained no observations")
            item = items[0]
            station_name = str(item.get("stnNm") or station.get("name_en") or "")
            payload = {
                "source": source,
                "status": "ok",
                "station": {
                    "id": station_id,
                    "name": station_name,
                    "latitude_deg": float(station["latitude_deg"]),
                    "longitude_deg": float(station["longitude_deg"]),
                    "gps_distance_km": float(station["distance_km"]),
                    "selection_method": "nearest_haversine_to_capture_gps",
                },
                "requested_at_kst": instant.isoformat(),
                "observed_at_kst": str(item.get("tm") or ""),
                "condition": _weather_condition_from_asos(item),
                "temperature_c": _finite_float(item.get("ta")),
                "humidity_pct": _finite_float(item.get("hm")),
                "precipitation_1h_mm": _finite_float(item.get("rn")),
                "wind_direction_deg": _finite_float(item.get("wd")),
                "wind_speed_m_s": _finite_float(item.get("ws")),
                "local_pressure_hpa": _finite_float(item.get("pa")),
                "sea_level_pressure_hpa": _finite_float(item.get("ps")),
                "cloud_cover_tenths": _finite_float(item.get("dc10Tca")),
                "visibility_10m": _finite_float(item.get("vs")),
                "snow_depth_cm": _finite_float(item.get("dsnw")),
                "weather_phenomenon_code": item.get("dmstMtphNo") or None,
            }
        except Exception as exc:
            payload = unavailable_weather(_safe_api_error(exc), source)
            payload["station"] = {
                "id": station_id,
                "name": station.get("name_en", ""),
                "latitude_deg": float(station["latitude_deg"]),
                "longitude_deg": float(station["longitude_deg"]),
                "gps_distance_km": float(station["distance_km"]),
                "selection_method": "nearest_haversine_to_capture_gps",
            }
            payload["requested_at_kst"] = instant.isoformat()
        self._weather_cache[cache_key] = payload
        return dict(payload)


class KmaWeatherClient:
    """Use GPS-grid nowcast for recent captures and nearest ASOS for history."""

    def __init__(
        self,
        nowcast: Optional[KmaVilageClient] = None,
        historical: Optional[KmaAsosHistoricalClient] = None,
    ):
        self.nowcast = nowcast or KmaVilageClient()
        self.historical = historical or KmaAsosHistoricalClient()

    @property
    def enabled(self) -> bool:
        return self.nowcast.enabled or self.historical.enabled

    def weather_for(self, camera_timestamp: float, latitude_deg, longitude_deg) -> dict:
        capture = camera_time_kst(camera_timestamp)
        age_hours = (datetime.now(KST) - capture).total_seconds() / 3600.0
        if age_hours <= 24.0 and self.nowcast.enabled:
            recent = self.nowcast.weather_for(camera_timestamp, latitude_deg, longitude_deg)
            if recent.get("status") == "ok":
                return recent
        if self.historical.enabled:
            return self.historical.weather_for(camera_timestamp, latitude_deg, longitude_deg)
        if self.nowcast.enabled:
            return self.nowcast.weather_for(camera_timestamp, latitude_deg, longitude_deg)
        return unavailable_weather("No KMA weather API is configured")


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
    def __init__(self, output, run_dir, save_images=True, camera=None):
        self.save_images = save_images
        self.directory = Path(output) / "certifcate"
        self.directory.mkdir(parents=True, exist_ok=True)
        _, self.gps_streams = lidar__load_gps_streams(Path(run_dir))
        self.weather = KmaWeatherClient()
        self.pcap_cache = {}
        camera = LiDAR2Camera(CAMERA_CALIBRATION) if camera is None else camera
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

    def stage_pcaps(self, paths):
        """Copy selected source PCAPs byte-for-byte; never serialize points to CSV."""
        files, metadata = [], []
        for source in map(Path, paths):
            source = source.resolve()
            stat = source.stat()
            signature = (
                stat.st_dev,
                stat.st_ino,
                stat.st_size,
                stat.st_mtime_ns,
                stat.st_ctime_ns,
            )
            cached = self.pcap_cache.get(source)
            if cached is None or cached[0] != signature:
                digest = sha(source)
                destination = self.directory / ("lidar_" + digest + ".pcap")
                temporary = destination.with_suffix(".pcap.partial")
                if not destination.exists():
                    try:
                        shutil.copyfile(source, temporary)
                        if sha(temporary) != digest:
                            raise ValueError("PCAP source changed during artifact copy")
                        temporary.replace(destination)
                    finally:
                        temporary.unlink(missing_ok=True)
                elif sha(destination) != digest:
                    raise ValueError("Existing artifact PCAP checksum differs")
                after = source.stat()
                if signature != (
                    after.st_dev,
                    after.st_ino,
                    after.st_size,
                    after.st_mtime_ns,
                    after.st_ctime_ns,
                ):
                    raise ValueError("PCAP source changed during export")
                item = dict(
                    name=destination.name,
                    source_name=source.name,
                    sha256=digest,
                    bytes=stat.st_size,
                )
                cached = (signature, destination, item)
                self.pcap_cache[source] = cached
            files.append(cached[1])
            metadata.append(cached[2])
        return files, metadata

    def save(
        self,
        index,
        timestamp,
        source_image,
        detections,
        fused,
        points,
        sensor_xyz,
        road_xyz,
        lidar_timestamp,
        pcap_sources=(),
        scan_start=None,
        scan_end=None,
        scan_status="unavailable",
        target_lidar_timestamp=None,
        camera_timing=None,
        motion_compensation=None,
        payloads=None,
    ):
        if len(detections) != len(fused):
            raise ValueError("Detection/fusion count mismatch")
        if not detections:
            return []
        # The live frame exporter attaches a single scan PCAP after collecting
        # all object payloads. The legacy standalone exporter keeps its own path.
        pcap_files, pcap_metadata = ([], []) if payloads is not None else self.stage_pcaps(pcap_sources)
        written = list(dict.fromkeys(pcap_files))
        lidar_metadata = dict(
            schema="pandar_xt32_pcap_v1",
            pcap_files=pcap_metadata,
            status=scan_status if pcap_metadata else "pcap_unavailable",
            decoded_on_terminal=lidar_timestamp is not None,
            scan_start_epoch_sec=None if scan_start is None else scan_start - 1e-6,
            scan_end_epoch_sec=None if scan_end is None else scan_end + 1e-6,
            target_epoch_sec=target_lidar_timestamp,
            camera_lidar_offset_sec=OFFSET_SEC,
            terminal_scan_point_count=len(points),
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
        gps = lidar__gps_values_for_camera(self.gps_streams, timestamp)
        weather = self.weather.weather_for(
            timestamp, gps.get("latitude_deg"), gps.get("longitude_deg")
        )
        for object_id, (detection, obj) in enumerate(zip(detections, fused)):
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
            stem = artifact_stem(timestamp, index, damage_type, object_id)
            image_path = self.directory / (stem + ".jpg")
            json_path = self.directory / (stem + ".json")
            if any((path.exists() for path in (image_path, json_path))):
                raise FileExistsError(f"Artifact already exists: {stem}")
            payload = build_damage_payload(
                damage_type=damage_type,
                camera_index=index,
                camera_timestamp=timestamp,
                lidar_frame_id=index,
                lidar_timestamp=(timestamp if lidar_timestamp is None else lidar_timestamp),
                time_error_sec=(0.0 if lidar_timestamp is None else lidar_timestamp - timestamp),
                cluster=geometry,
                bbox=tuple(np.rint(detection.box_xyxy).astype(int)),
                polygons_px=mask_polygons(detection.mask),
                gps=gps,
                weather=weather,
                vehicle_type=VEHICLE_TYPE,
                sensor_mount_height_cm=SENSOR_MOUNT_HEIGHT_CM,
                sensor_mount_angles_xyz_deg=SENSOR_MOUNT_ANGLE_XYZ_DEG,
                source_image=str(source_image),
                json_name=json_path.name,
                image_name=image_path.name,
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
            if payloads is not None:
                # Keep damage measurements in one JSON/image pair per frame.
                payloads.append(payload)
                continue
            temporary = image_path.with_name(stem + ".partial.jpg")
            try:
                if self.save_images:
                    if Path(source_image).suffix.lower() in (".jpg", ".jpeg"):
                        shutil.copyfile(source_image, temporary)
                    else:
                        image = cv2.imread(str(source_image))
                        if image is None or not cv2.imwrite(str(temporary), image):
                            raise IOError(f"Cannot save source image: {source_image}")
                if self.save_images:
                    temporary.replace(image_path)
                write_damage_json(json_path, payload)
                written.extend([json_path] + ([image_path] if self.save_images else []))
            except Exception:
                for path in (temporary, image_path, json_path):
                    path.unlink(missing_ok=True)
                raise
        return written


def matched_gps_metadata(streams, timestamp):
    """Vehicle position with explicit NMEA validity; missing fixes stay null."""
    gga, _ = lidar__nearest_gps_record(streams.get("GGA"), timestamp)
    rmc, _ = lidar__nearest_gps_record(streams.get("RMC"), timestamp)
    def position_valid(record):
        return (record is not None and
                record.get("valid") is not False and record.get("checksum_valid") is not False and
                _finite_float(record.get("latitude_deg")) is not None and
                _finite_float(record.get("longitude_deg")) is not None and
                abs(float(record["latitude_deg"])) <= 90 and
                abs(float(record["longitude_deg"])) <= 180)
    valid_rmc = position_valid(rmc) and str(rmc.get("gps_status", rmc.get("status", ""))).upper() == "A"
    valid_gga = position_valid(gga) and (_finite_float(gga.get("fix_quality")) or 0) > 0
    position = rmc if valid_rmc else gga if valid_gga else None
    data = lidar__gps_values_for_camera(streams, timestamp)
    data.update(valid=position is not None, position_reference="vehicle", crs="WGS84",
                timestamp_source="host_receive_usb_nmea", ptp_synchronized=False,
                latitude_deg=None if position is None else position["latitude_deg"],
                longitude_deg=None if position is None else position["longitude_deg"],
                host_receive_epoch_sec=None if position is None else float(position["timestamp"]),
                time_error_ms=None if position is None else
                (float(position["timestamp"]) - timestamp) * 1000,
                altitude_m=data["altitude_m"] if valid_gga else None,
                speed_kmh=data["speed_kmh"] if valid_rmc else None,
                course_deg=data["course_deg"] if valid_rmc else None,
                reason="valid_fix" if position is not None else "no_valid_nearby_fix")
    return clean(data)


def terminal_id():
    """Terminal name used in record_id (PORTHOLE_TERMINAL_ID, else the host name)."""
    return os.getenv("PORTHOLE_TERMINAL_ID", "").strip() or socket.gethostname()


def save_detection_frame(directory, source_image, image, row, timestamp, key,
                         damage_detections, damage_payloads, gps, lidar):
    """Frame JSON in the terminal-server data spec (단말기-서버 데이터 명세서, 2026-09-28).

    Only record_id, categories, images, annotations, gps and lidar.pcap_files are
    sent; missing measurements are null, not 0. The full per-damage details stay
    in the frame's local result.json.
    """
    if not damage_detections:
        return []
    if len(damage_detections) != len(damage_payloads):
        raise ValueError("Damage annotation/payload mismatch")
    if any(int(d.class_id) not in SUSPECT_CLASS_IDS for d in damage_detections):
        raise ValueError("Unsupported category in damage frame export")
    height, width = image.shape[:2]
    index = int(row["frame_index"])
    timestamp_ns = int(row.get("timestamp_ns") or round(timestamp * 1e9))
    stem = detection_frame_stem(row, timestamp)
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
        gps=dict(latitude_deg=gps.get("latitude_deg"), longitude_deg=gps.get("longitude_deg")),
        lidar=dict(pcap_files=[dict(name=f["name"]) for f in lidar.get("pcap_files", [])]),
    )
    atomic_json(json_path, clean(payload))
    return [image_path, json_path]


def detection_frame_stem(row, timestamp):
    timestamp_ns = int(row.get("timestamp_ns") or round(timestamp * 1e9))
    return f"frame_{timestamp_ns}_{int(row['frame_index']):08d}"


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


def local_depth__evaluate(
    road, pixels, visible_indices, indices, mask, exclusion, channels, threshold
):
    details = dict(
        version="local_reference_v1",
        min_inner_points=10,
        min_deep_points=5,
        min_deep_channels=2,
        min_deep_fraction=0.1,
        ring_radius_px=20,
        inner_erosion_px=3,
        cell_size_m=0.05,
        reference_is_ground_truth=False,
    )

    def stop(reason):
        return (False, reason, None, details)

    inner = cv2.erode(mask.astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    uv = pixels[indices].astype(int)
    inside = inner[uv[:, 1], uv[:, 0]]
    target = indices[inside]
    details["inner_points"] = len(target)
    if len(target) < 10:
        return stop("insufficient_inner_support")
    ring = cv2.dilate(mask.astype(np.uint8), np.ones((41, 41), np.uint8)).astype(bool) & ~exclusion
    pv = pixels[visible_indices].astype(int)
    ref = visible_indices[ring[pv[:, 1], pv[:, 0]]]
    details["reference_points"] = len(ref)
    if len(ref) < 30 or len(np.unique(channels[ref])) < 3:
        return stop("local_reference_insufficient")
    xy = road[ref, :2]
    origin = np.median(road[target, :2], axis=0)
    cells = np.floor((xy - origin) / 0.05).astype(int)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    representatives = np.array(
        [
            np.median(road[ref[inverse == i]], axis=0)
            for i in range(inverse.max() + 1)
            if np.sum(inverse == i) >= 2
        ]
    )
    details["reference_cells"] = len(representatives)
    if len(representatives) < 12:
        return stop("local_reference_insufficient")
    design = np.column_stack((representatives[:, :2] - origin, np.ones(len(representatives))))
    if np.linalg.cond(design) > 100:
        return stop("local_reference_degenerate")
    coef = np.linalg.lstsq(design, representatives[:, 2], rcond=None)[0]
    for _ in range(8):
        residual = representatives[:, 2] - design @ coef
        scale = max(0.002, 1.4826 * np.median(abs(residual - np.median(residual))))
        weights = np.minimum(1.0, 1.5 * scale / np.maximum(abs(residual), 1e-12))
        coef = np.linalg.lstsq(
            design * np.sqrt(weights[:, None]),
            representatives[:, 2] * np.sqrt(weights),
            rcond=None,
        )[0]
    norm = float(np.sqrt(1 + np.sum(coef[:2] ** 2)))
    residual = (representatives[:, 2] - design @ coef) / norm
    noise = float(np.quantile(abs(residual), 0.9))
    details.update(
        reference_p90_abs_m=noise,
        local_coefficients=coef.tolist(),
        local_origin_xy_m=origin.tolist(),
        local_tilt_deg=float(np.degrees(np.arctan(np.linalg.norm(coef[:2])))),
    )
    if noise > threshold / 2 or details["local_tilt_deg"] > 20:
        return stop("local_reference_unreliable")
    support = representatives[abs(residual) <= threshold, :2]
    hull = cv2.convexHull(support.astype(np.float32))
    covered = np.array(
        [cv2.pointPolygonTest(hull, tuple(map(float, p)), False) >= 0 for p in road[target, :2]]
    )
    details["inner_reference_hull_coverage"] = float(covered.mean())
    if covered.mean() < 0.9:
        return stop("local_reference_one_sided")
    measured = road[indices].copy()
    measured[:, 2] = (measured[:, 2] - ((measured[:, :2] - origin) @ coef[:2] + coef[2])) / norm
    depths = -measured[inside, 2]
    deep = depths >= threshold + noise
    p95 = float(np.quantile(depths, 0.95))
    details.update(
        local_inner_p95_depth_m=p95,
        uncertainty_margin_m=noise,
        conservative_p95_depth_m=p95 - noise,
        deep_inner_points=int(deep.sum()),
        deep_inner_channels=len(np.unique(channels[target[deep]])),
        deep_inner_fraction=float(deep.mean()),
        local_max_depth_m=float(np.max(-measured[:, 2])),
    )
    keep = (
        p95 - noise >= threshold
        and deep.sum() >= 5
        and (deep.mean() >= 0.1)
        and (details["deep_inner_channels"] >= 2)
    )
    return (
        bool(keep),
        "local_depth_pass" if keep else "local_depth_insufficient",
        measured,
        details,
    )


@dataclass(frozen=True)
class RoadPlaneSettings:
    expected_normal: tuple = (-0.016, 0.666, 0.746)
    lateral_limit_m: float = 1.8
    forward_min_m: float = 1.0
    forward_max_m: float = 3.8
    height_min_m: float = -2.5
    height_max_m: float = -0.8
    cell_size_m: float = 0.35
    min_cell_points: int = 8
    min_cells: int = 24
    max_angle_deg: float = 15.0
    support_distance_m: float = 0.04
    max_p75_m: float = 0.04
    max_negative_p10_m: float = 0.075
    min_support_fraction: float = 0.75


class RoadPlaneUnavailable(ValueError):
    def __init__(self, reason, info):
        super().__init__("plane_unavailable: " + reason)
        self.plane_info = info


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


def fit_spatial_road_plane(xyz, channels, settings=RoadPlaneSettings()):
    xyz = np.asarray(xyz, float)
    channels = np.asarray(channels)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or channels.shape != (len(xyz),):
        raise ValueError("XYZ/channel shape mismatch")
    R = prior_rotation(settings)
    local = xyz @ R.T
    x, y, z = local.T
    keep = (
        np.isfinite(local).all(axis=1)
        & (abs(x) <= settings.lateral_limit_m)
        & (y <= -settings.forward_min_m)
        & (y >= -settings.forward_max_m)
        & (z >= settings.height_min_m)
        & (z <= settings.height_max_m)
    )
    points = local[keep]
    ch = channels[keep]
    info = dict(
        fit_method="spatial_grid_robust_plane",
        plane_valid=False,
        normal_prior_source="empirical_installed_rig",
        road_candidate_points=len(points),
    )

    def fail(reason):
        raise RoadPlaneUnavailable(reason, info)

    if len(points) < 3 or np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
        keep = np.isfinite(local).all(axis=1)
        points = local[keep]
        ch = channels[keep]
        info["candidate_source"] = "all_finite_points"
    else:
        info["candidate_source"] = "road_roi"
    if len(points) < 3 or np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
        fail("fewer than three non-collinear finite points")
    keys = np.floor(points[:, :2] / settings.cell_size_m).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    cells = []
    for i in range(int(inverse.max()) + 1):
        group = points[inverse == i]
        cells.append(np.median(group, axis=0))
    cells = np.asarray(cells, float).reshape(-1, 3)
    info["grid_cells"] = len(cells)
    if len(cells) < 3 or np.linalg.matrix_rank(cells - cells.mean(axis=0)) < 2:
        cells = points.copy()
        info["representative_fallback"] = "raw_points"
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
    cell_residual = (target - A @ beta) / scale
    support = abs(cell_residual) <= settings.support_distance_m
    support_cells = cells[support]
    angle = float(np.degrees(np.arctan(np.linalg.norm(beta[:2]))))
    q = np.quantile(abs(cell_residual), [0.5, 0.75, 0.9])
    left = int(np.sum(support_cells[:, 0] < -0.4))
    right = int(np.sum(support_cells[:, 0] > 0.4))
    span = np.ptp(support_cells[:, :2], axis=0) if len(support_cells) else np.zeros(2)
    raw_support = (
        abs(points @ np.array([-beta[0], -beta[1], 1.0]) / scale + offset)
        <= settings.support_distance_m
    )
    channel_count = len(np.unique(ch[raw_support]))
    check = dict(
        normal_angle_deg=angle,
        support_points=int(raw_support.sum()),
        left_support=left,
        right_support=right,
        support_channels=channel_count,
        support_cells=int(support.sum()),
        support_fraction=float(support.mean()),
        lateral_span_m=float(span[0]),
        forward_span_m=float(span[1]),
        cell_p50_abs_m=float(q[0]),
        cell_p75_abs_m=float(q[1]),
        cell_p90_abs_m=float(q[2]),
        cell_signed_p10_m=float(np.quantile(cell_residual, 0.1)),
    )
    info.update(
        road_support_validation=check,
        selected_candidate=dict(
            source="spatial_grid",
            cell_count=len(cells),
            cell_p90_abs_m=float(q[2]),
            cell_median_abs_m=float(q[0]),
            left_support=left,
            right_support=right,
            normal_angle_deg=angle,
        ),
        final_plane_normal=normal.tolist(),
        final_plane_offset_m=offset,
    )
    info["plane_valid"] = True
    return (normal, offset, info)


def measure_mask_local_depth(road, pixels, visible_indices, indices, mask, exclusion):
    """Local surrounding reference; no interior/channel/depth-support gates."""
    ring = cv2.dilate(mask.astype(np.uint8), np.ones((41, 41), np.uint8)).astype(bool) & ~exclusion
    uv = pixels[visible_indices].astype(int)
    ref = visible_indices[ring[uv[:, 1], uv[:, 0]]]
    info = dict(
        depth_reference="local_surrounding_plane",
        reference_points=len(ref),
        ring_radius_px=20,
    )
    if len(ref) < 3:
        return (None, dict(info, error="insufficient_reference_points"))
    origin = np.median(road[indices, :2], axis=0)
    cells = np.floor((road[ref, :2] - origin) / 0.05).astype(int)
    _, inv = np.unique(cells, axis=0, return_inverse=True)
    reps = np.array([np.median(road[ref[inv == i]], axis=0) for i in range(inv.max() + 1)])
    A = np.column_stack((reps[:, :2] - origin, np.ones(len(reps))))
    if np.linalg.matrix_rank(A) < 3:
        return (None, dict(info, error="reference_plane_undefined"))
    coef = np.linalg.lstsq(A, reps[:, 2], rcond=None)[0]
    for _ in range(8):
        residual = reps[:, 2] - A @ coef
        scale = max(0.002, 1.4826 * np.median(abs(residual - np.median(residual))))
        weights = np.sqrt(np.minimum(1.0, 1.5 * scale / np.maximum(abs(residual), 1e-12)))
        coef = np.linalg.lstsq(A * weights[:, None], reps[:, 2] * weights, rcond=None)[0]
    norm = np.sqrt(1 + np.sum(coef[:2] ** 2))
    measured = road[indices].copy()
    measured[:, 2] = (measured[:, 2] - ((measured[:, :2] - origin) @ coef[:2] + coef[2])) / norm
    info.update(
        local_coefficients=coef.tolist(),
        local_origin_xy_m=origin.tolist(),
        reference_p90_abs_m=float(np.quantile(abs(reps[:, 2] - A @ coef) / norm, 0.9)),
        max_depth_m=float(np.max(-measured[:, 2])),
    )
    return (measured, info)


lidar__DETECTION_CHANNEL_IDS = tuple(range(1, 33))

lidar___DETECTION_CHANNEL_SET = frozenset(lidar__DETECTION_CHANNEL_IDS)

lidar__GPS_MAX_TIME_ERROR_SEC = 1.5

lidar__PLANE_FALLBACK_CENTRAL_FIRING_FRACTION = 0.6

lidar__PLANE_FIRING_QUALITY_BIN_COUNT = 6

lidar__RETURN_SELECTION = "last"

lidar__APPLY_FIRETIME = True

lidar__COORDINATE_CORRECTION = True

lidar__MAX_FRAMES: Optional[int] = None

lidar__PCAP_GLOBAL_LEN = 24

lidar__PCAP_RECORD_LEN = 16

lidar__XT32_PAYLOAD_LEN = 1080

lidar__XT32_BODY_OFFSET = 12

lidar__XT32_BLOCK_LEN = 130

lidar__XT32_BLOCKS = 8

lidar__XT32_CHANNELS = 32

lidar__XT32_TAIL_OFFSET = 1052

lidar__DEFAULT_ELEVATION_DEG = [float(v) for v in range(15, -17, -1)]

lidar__DEFAULT_AZIMUTH_OFFSET_DEG = [0.0] * lidar__XT32_CHANNELS

lidar__XT32_LASER_FIRETIME_US = [1.512 * i + 0.368 for i in range(32)]

lidar__XT_COORD_H_M = 0.0315

lidar__XT_COORD_B_M = 0.013

lidar__DUAL_RETURN_MODES = {57, 59, 60}


@dataclass(frozen=True)
class lidar__Calibration:
    elevation_deg: list[float]
    azimuth_offset_deg: list[float]
    source: str


@dataclass(frozen=True)
class lidar__PcapRecord:
    packet_index: int
    timestamp: float
    frame: bytes


@dataclass(frozen=True)
class lidar__UdpPacket:
    payload: bytes
    src_port: int
    dst_port: int


def lidar__load_calibration(path: Optional[Path]) -> lidar__Calibration:
    if path is None:
        return lidar__Calibration(
            elevation_deg=lidar__DEFAULT_ELEVATION_DEG.copy(),
            azimuth_offset_deg=lidar__DEFAULT_AZIMUTH_OFFSET_DEG.copy(),
            source="Hesai PandarXT32 nominal calibration",
        )
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
        source=str(path),
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


def lidar__decode_udp(frame: bytes) -> lidar__UdpPacket:
    if len(frame) < 42:
        raise ValueError("Ethernet frame is too short")
    if struct.unpack("!H", frame[12:14])[0] != 2048:
        raise ValueError("Ethernet payload is not IPv4")
    ip_start = 14
    ihl = (frame[ip_start] & 15) * 4
    if ihl < 20 or frame[ip_start + 9] != 17:
        raise ValueError("IPv4 payload is not valid UDP")
    udp_start = ip_start + ihl
    src_port, dst_port, udp_len, _ = struct.unpack("!HHHH", frame[udp_start : udp_start + 8])
    payload = frame[udp_start + 8 : udp_start + udp_len]
    return lidar__UdpPacket(payload=payload, src_port=src_port, dst_port=dst_port)


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


def lidar__corrected_azimuth_deg(
    raw_az100: int, channel_index: int, spin_speed_rpm: float, azimuth_offset_deg: float
) -> float:
    firetime_deg = 0.0
    if lidar__APPLY_FIRETIME:
        firetime_deg = spin_speed_rpm * lidar__XT32_LASER_FIRETIME_US[channel_index] * 6e-06
    az100 = int(azimuth_offset_deg * 100.0 + raw_az100 + firetime_deg * 100.0) % 36000
    return az100 / 100.0


def lidar__xyz_standard(
    distance_m: float, azimuth_deg: float, elevation_deg: float
) -> tuple[float, float, float]:
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    xy = distance_m * math.cos(el)
    return (xy * math.sin(az), xy * math.cos(az), distance_m * math.sin(el))


def lidar__xyz_coordinate_corrected(
    distance_m: float,
    azimuth_deg: float,
    elevation_deg: float,
    azimuth_offset_deg: float,
) -> tuple[float, float, float]:
    sin_az, cos_az = _azimuth_trig(azimuth_deg)
    cos_el, sin_el, correction = _channel_geometry(
        elevation_deg, azimuth_offset_deg, lidar__XT_COORD_H_M, lidar__XT_COORD_B_M
    )
    corrected_distance = distance_m - correction
    xy = corrected_distance * cos_el
    x = xy * sin_az - lidar__XT_COORD_B_M * cos_az + lidar__XT_COORD_H_M * sin_az
    y = xy * cos_az + lidar__XT_COORD_B_M * sin_az + lidar__XT_COORD_H_M * cos_az
    z = corrected_distance * sin_el
    return (x, y, z)


@lru_cache(maxsize=36000)
def _azimuth_trig(azimuth_deg):
    az = math.radians(azimuth_deg)
    return (math.sin(az), math.cos(az))


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


def decode_records_to_frames(records, calibration: lidar__Calibration) -> list[list[dict]]:
    """
    Recover frames from UDP sequence reset.

    The earlier frame-PCAP writer resets tail sequence to 0 at each frame,
    so the merged PCAP has a clear boundary whenever sequence decreases.
    """
    frames: list[list[dict]] = []
    current_points: list[dict] = []
    previous_sequence: Optional[int] = None
    for rec in records:
        udp = lidar__decode_udp(rec.frame)
        payload = udp.payload
        if len(payload) != lidar__XT32_PAYLOAD_LEN:
            continue
        if payload[:4] != b"\xee\xff\x06\x01":
            continue
        laser_num = payload[6]
        block_num = payload[7]
        dist_unit_mm = payload[9]
        if laser_num != 32 or block_num != 8:
            raise ValueError(
                f"Packet {rec.packet_index}: expected XT32 32/8, got {laser_num}/{block_num}"
            )
        tail = payload[lidar__XT32_TAIL_OFFSET : lidar__XT32_TAIL_OFFSET + 28]
        return_mode = tail[10]
        spin_speed_rpm = struct.unpack_from("<H", tail, 11)[0]
        sequence = struct.unpack_from("<I", tail, 24)[0]
        if previous_sequence is not None and sequence < previous_sequence and current_points:
            frames.append(current_points)
            current_points = []
            if lidar__MAX_FRAMES is not None and len(frames) >= lidar__MAX_FRAMES:
                return frames
        previous_sequence = sequence
        for block_index in range(lidar__XT32_BLOCKS):
            start = lidar__XT32_BODY_OFFSET + block_index * lidar__XT32_BLOCK_LEN
            block = payload[start : start + lidar__XT32_BLOCK_LEN]
            raw_az100 = struct.unpack_from("<H", block, 0)[0]
            rlabel = lidar__return_label(return_mode, block_index)
            if lidar__RETURN_SELECTION != "all" and rlabel != lidar__RETURN_SELECTION:
                continue
            for channel_index in range(lidar__XT32_CHANNELS):
                channel_id = channel_index + 1
                if channel_id not in lidar___DETECTION_CHANNEL_SET:
                    continue
                off = 2 + channel_index * 4
                distance_raw = struct.unpack_from("<H", block, off)[0]
                intensity = block[off + 2]
                confidence = block[off + 3]
                distance_m = distance_raw * dist_unit_mm / 1000.0
                if not 0.1 < distance_m <= 200.0:
                    continue
                elevation_deg = calibration.elevation_deg[channel_index]
                az_offset_deg = calibration.azimuth_offset_deg[channel_index]
                azimuth_deg = lidar__corrected_azimuth_deg(
                    raw_az100, channel_index, spin_speed_rpm, az_offset_deg
                )
                if lidar__COORDINATE_CORRECTION:
                    x, y, z = lidar__xyz_coordinate_corrected(
                        distance_m, azimuth_deg, elevation_deg, az_offset_deg
                    )
                else:
                    x, y, z = lidar__xyz_standard(distance_m, azimuth_deg, elevation_deg)
                current_points.append(
                    {
                        "packet_index": rec.packet_index,
                        "sequence": sequence,
                        "timestamp": rec.timestamp,
                        "block": block_index + 1,
                        "return": rlabel,
                        "return_mode": return_mode,
                        "channel": channel_id,
                        "azimuth_deg": azimuth_deg,
                        "distance_m": distance_m,
                        "x_m": x,
                        "y_m": y,
                        "z_m": z,
                        "intensity": intensity,
                        "confidence": confidence,
                    }
                )
    if current_points and (lidar__MAX_FRAMES is None or len(frames) < lidar__MAX_FRAMES):
        frames.append(current_points)
    return frames


def lidar___channel_plane_quality(
    signed_residual_m: np.ndarray, channels: np.ndarray
) -> tuple[float, float, dict[int, float]]:
    """Measure whether one fitted plane is centred on every scanline.

    In the measured failure frames, raw RANSAC inlier count could still look
    plausible because a shoulder plane contained many points.  The decisive
    symptom was different: the median return in nearly every channel sat
    10-20 cm on the same side of the fitted plane.  A real road plane keeps the
    median of those per-channel medians close to zero.
    """
    signed = np.asarray(signed_residual_m, dtype=np.float64)
    channel_values = np.asarray(channels, dtype=np.int16)
    if signed.shape != channel_values.shape:
        raise ValueError("signed_residual_m/channels shape mismatch")
    medians: dict[int, float] = {}
    for channel in sorted((int(value) for value in np.unique(channel_values))):
        values = signed[channel_values == channel]
        values = values[np.isfinite(values)]
        if values.size:
            medians[channel] = float(np.median(values))
    if not medians:
        return (float("nan"), float("nan"), medians)
    values = np.asarray(list(medians.values()), dtype=np.float64)
    return (float(np.median(values)), float(np.percentile(np.abs(values), 90)), medians)


def lidar___firing_plane_quality(
    signed_residual_m: np.ndarray,
    firing_key_array: np.ndarray,
    fraction: float = lidar__PLANE_FALLBACK_CENTRAL_FIRING_FRACTION,
    bin_count: int = lidar__PLANE_FIRING_QUALITY_BIN_COUNT,
) -> dict:
    """Measure smooth horizontal plane drift without using roadside edges."""
    signed = np.asarray(signed_residual_m, dtype=np.float64)
    keys = np.asarray(firing_key_array, dtype=np.int64)
    empty = {
        "span_m": float("nan"),
        "endpoint_delta_m": float("nan"),
        "linear_r2": float("nan"),
        "bin_medians_m": [],
    }
    if signed.ndim != 1 or keys.shape != (len(signed), 3) or (not len(signed)):
        return empty
    positions, inverse = np.unique(keys[:, :2], axis=0, return_inverse=True)
    keep_fraction = min(1.0, max(0.05, float(fraction)))
    keep_count = min(len(positions), max(3, int(round(len(positions) * keep_fraction))))
    central_start = max(0, (len(positions) - keep_count) // 2)
    actual_bin_count = min(max(3, int(bin_count)), keep_count)
    if actual_bin_count < 3:
        return empty
    medians: list[float] = []
    for bin_index in range(actual_bin_count):
        start = central_start + int(round(bin_index * keep_count / actual_bin_count))
        stop = central_start + int(round((bin_index + 1) * keep_count / actual_bin_count))
        mask = (inverse >= start) & (inverse < stop)
        values = signed[mask]
        values = values[np.isfinite(values)]
        if not values.size:
            return empty
        medians.append(float(np.median(values)))
    values = np.asarray(medians, dtype=np.float64)
    x = np.arange(len(values), dtype=np.float64)
    coefficients = np.polyfit(x, values, 1)
    predicted = np.polyval(coefficients, x)
    total = float(np.sum((values - np.mean(values)) ** 2))
    unexplained = float(np.sum((values - predicted) ** 2))
    linear_r2 = 1.0 - unexplained / total if total > 1e-12 else 0.0
    return {
        "span_m": float(np.ptp(values)),
        "endpoint_delta_m": float(values[-1] - values[0]),
        "linear_r2": float(linear_r2),
        "bin_medians_m": medians,
    }


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


def lidar__firing_keys(points: list[dict]) -> np.ndarray:
    """
    One identity per laser firing.

    Dual-return data stores Last and First for the same firing in adjacent
    blocks, so block index is halved to make both returns share a key. This is
    what makes MIN_CLUSTER_POINTS a count of real firings rather than rows.
    """
    return np.asarray(
        [
            (
                int(p["sequence"]),
                (
                    (int(p["block"]) - 1) // 2
                    if int(p["return_mode"]) in lidar__DUAL_RETURN_MODES
                    else int(p["block"]) - 1
                ),
                int(p["channel"]),
            )
            for p in points
        ],
        dtype=np.int64,
    )


def lidar__load_gps_streams(
    dataset_root: Path,
) -> tuple[Optional[Path], dict[str, tuple[np.ndarray, list[dict]]]]:
    """Load timestamp-sorted GGA/RMC records used by the camera overlay."""
    candidates = [dataset_root / "gps" / "gps.jsonl", dataset_root / "gps.jsonl"]
    gps_path = next((path for path in candidates if path.is_file()), None)
    if gps_path is None:
        found = list(dataset_root.rglob("gps.jsonl"))
        gps_path = found[0] if found else None
    if gps_path is None:
        return (None, {})
    grouped: dict[str, list[dict]] = {"GGA": [], "RMC": []}
    with gps_path.open("r", encoding="utf-8", errors="replace") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                item = json.loads(line)
                sentence_type = str(item.get("sentence_type", "")).upper()
                timestamp = float(item["timestamp"])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                print(f"[WARN] skip malformed gps.jsonl line {line_no}")
                continue
            if sentence_type in grouped and np.isfinite(timestamp):
                grouped[sentence_type].append(item)
    streams: dict[str, tuple[np.ndarray, list[dict]]] = {}
    for sentence_type, records in grouped.items():
        records.sort(key=lambda item: float(item["timestamp"]))
        times = np.asarray([float(item["timestamp"]) for item in records], dtype=np.float64)
        streams[sentence_type] = (times, records)
    return (gps_path, streams)


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


def lidar__gps_values_for_camera(
    gps_streams: dict[str, tuple[np.ndarray, list[dict]]], camera_timestamp: float
) -> dict:
    gga, gga_error = lidar__nearest_gps_record(gps_streams.get("GGA"), camera_timestamp)
    rmc, rmc_error = lidar__nearest_gps_record(gps_streams.get("RMC"), camera_timestamp)
    position = rmc if rmc is not None else gga
    errors = [
        error for (record, error) in ((gga, gga_error), (rmc, rmc_error)) if record is not None
    ]
    speed_kmh = None
    if rmc is not None:
        if rmc.get("speed_mps") is not None:
            speed_kmh = float(rmc["speed_mps"]) * 3.6
        elif rmc.get("speed_kmh") is not None:
            speed_kmh = float(rmc["speed_kmh"])
    return {
        "latitude_deg": None if position is None else position.get("latitude_deg"),
        "longitude_deg": None if position is None else position.get("longitude_deg"),
        "speed_kmh": speed_kmh,
        "course_deg": None if rmc is None else rmc.get("course_deg"),
        "altitude_m": None if gga is None else gga.get("altitude_m"),
        "fix_quality": None if gga is None else gga.get("fix_quality"),
        "satellites": None if gga is None else gga.get("satellites"),
        "hdop": None if gga is None else gga.get("hdop"),
        "time_error_ms": None if not errors else max(errors) * 1000.0,
    }


def decode_scan(source, calibration, start, end):
    records = iter_scan_records(source, start, end)
    try:
        frames = decode_records_to_frames(records, calibration)
        for _ in records:
            pass
    finally:
        records.close()
    return [point for frame in frames for point in frame]


def iter_scan_records(source, start, end):
    """Stream the same inclusive slice as copy_scan_pcap, without temporary I/O.

    Indices are relative to the selected records, including non-XT32 packets,
    and continue across source files. Never stop at a high timestamp: a single
    source is allowed to contain out-of-order packets, as in the legacy path.
    """
    sources = [source] if isinstance(source, (str, os.PathLike)) else list(source)
    if not sources:
        raise ValueError("No PCAP sources")
    formats = {
        b"\xd4\xc3\xb2\xa1": ("<", 1000000.0),
        b"\xa1\xb2\xc3\xd4": (">", 1000000.0),
        b"M<\xb2\xa1": ("<", 1000000000.0),
        b"\xa1\xb2<M": (">", 1000000000.0),
    }
    global_header = None
    previous_timestamp = None
    packet_index = 0
    for path in sources:
        with Path(path).open("rb") as stream:
            header = stream.read(lidar__PCAP_GLOBAL_LEN)
            if len(header) != lidar__PCAP_GLOBAL_LEN or header[:4] not in formats:
                raise ValueError("Invalid classic PCAP header")
            endian, divisor = formats[header[:4]]
            network = struct.unpack(endian + "IHHIIII", header)[-1]
            if network != 1:
                raise ValueError(f"Only Ethernet PCAP is supported (DLT={network})")
            if global_header is not None and header != global_header:
                raise ValueError("Incompatible PCAP headers across scan sources")
            global_header = header
            while True:
                record = stream.read(lidar__PCAP_RECORD_LEN)
                if not record:
                    break
                if len(record) != lidar__PCAP_RECORD_LEN:
                    raise ValueError("Truncated PCAP record header")
                seconds, fraction, size, _ = struct.unpack(endian + "IIII", record)
                timestamp = seconds + fraction / divisor
                if not start <= timestamp <= end:
                    stream.seek(size, 1)
                    continue
                payload = stream.read(size)
                if len(payload) != size:
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
            udp = lidar__decode_udp(rec.frame)
        except ValueError:
            continue
        payload = udp.payload
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


def flatten(points, frame_id):
    if len(points) < 3:
        raise ValueError("insufficient_lidar_points")
    xyz = np.asarray([[p["x_m"], p["y_m"], p["z_m"]] for p in points], dtype=float)
    channels = np.asarray([p["channel"] for p in points])
    normal, offset, info = fit_spatial_road_plane(xyz, channels)
    signed = xyz @ normal + offset
    center, p90, _ = lidar___channel_plane_quality(signed, channels)
    info.update(
        channel_center_error_m=float(center),
        channel_p90_abs_m=float(p90),
        final_firing_quality=lidar___firing_plane_quality(signed, lidar__firing_keys(points)),
        residual_std_m=float(np.std(signed[np.abs(signed) <= 0.04])),
        all_residual_std_m=float(np.std(signed)),
    )
    road, rotation, origin = lidar__road_aligned_xyz(xyz, normal, offset)
    return dict(
        points=points,
        point_timestamps=np.asarray([p["timestamp"] for p in points], dtype=float),
        sensor_xyz_m=xyz,
        road_xyz_m=road,
        plane_normal=normal,
        plane_offset=offset,
        plane_info=info,
        road_rotation=rotation,
        road_origin_sensor_m=origin,
    )


def validate_objects(
    detections, result, camera, threshold_m=POTHOLE_DEPTH_M, active_intervals=None
):
    """Accept a pothole when any finite mask point reaches the depth threshold."""
    import time

    active_start = time.monotonic_ns()
    if not np.isfinite(threshold_m) or threshold_m <= 0:
        raise ValueError("invalid depth threshold")
    pixels, visible = (
        camera.project(result["sensor_xyz_m"])
        if result is not None
        else (np.empty((0, 2)), np.empty(0, bool))
    )
    masks = [np.asarray(d.mask, bool) for d in detections if d.mask is not None]
    exclusion = np.logical_or.reduce(masks) if masks else None
    accepted, objects, audit = ([], [], [])
    for i, d in enumerate(detections):
        indices = np.empty(0, int)
        if result is not None and d.mask is not None:
            mask = np.asarray(d.mask, bool)
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
        maximum = float(np.max(-road[:, 2])) if len(road) else None
        keep = int(d.class_id) == 0 or (maximum is not None and maximum >= threshold_m)
        reason = (
            "model_crack"
            if int(d.class_id) == 0
            else (
                "plane_unavailable"
                if result is None
                else (
                    "no_mask_points"
                    if not len(indices)
                    else "depth_pass" if keep else "depth_below_threshold"
                )
            )
        )
        local = {}
        if int(d.class_id) == 1 and len(indices):
            if active_intervals is not None:
                active_intervals.append((active_start, time.monotonic_ns()))
            channels = np.asarray([p["channel"] for p in result["points"]])
            _, _, _, local = local_depth__evaluate(
                result["road_xyz_m"],
                pixels,
                np.flatnonzero(valid),
                indices,
                mask,
                exclusion,
                channels,
                threshold_m,
            )
            active_start = time.monotonic_ns()
        global_maximum = maximum
        if int(d.class_id) == 1 and len(indices):
            measured, measurement = measure_mask_local_depth(
                result["road_xyz_m"],
                pixels,
                np.flatnonzero(valid),
                indices,
                mask,
                exclusion,
            )
            local["active_measurement"] = measurement
            road = np.empty((0, 3)) if measured is None else measured
            maximum = None if measured is None else float(np.max(-measured[:, 2]))
            keep = maximum is not None and maximum >= threshold_m
            reason = (
                "local_reference_unavailable"
                if maximum is None
                else "depth_pass" if keep else "depth_below_threshold"
            )
        if active_intervals is not None:
            active_intervals.append((active_start, time.monotonic_ns()))
        evidence = dict(
            method="model_only" if d.class_id == 0 else "model_mask_local_max_depth",
            accepted=keep,
            reason=reason,
            max_depth_m=maximum,
            threshold_depth_m=threshold_m,
            mask_points=len(indices),
            global_max_depth_m=global_maximum,
            local_evidence_json=json.dumps(local),
            validation_state=(
                "accepted" if keep else "unobserved" if reason == "no_mask_points" else "withheld"
            ),
        )
        audit.append(
            dict(
                model_object_id=i,
                class_id=int(d.class_id),
                confidence=float(d.confidence),
                box_xyxy=np.asarray(d.box_xyxy).tolist(),
                **evidence,
            )
        )
        if keep:
            accepted.append(d)
            objects.append(
                dict(
                    class_id=int(d.class_id),
                    status="model_mask",
                    point_indices=indices,
                    filtered_points=len(indices),
                    verified_point_indices=indices,
                    verified_road_xyz_m=road,
                    measurement_record=damage_geometry(road, include_depth=d.class_id == 1),
                    validation=evidence,
                )
            )
        active_start = time.monotonic_ns()
    if active_intervals is not None:
        active_intervals.append((active_start, time.monotonic_ns()))
    return (accepted, objects, audit)


def scan_candidates(pcaps, pcap_dir, center, frame_bounds, cache, args):
    candidates = []
    fragments = []
    for item in pcaps:
        if (
            float(item["last_timestamp"]) < center - args.scan_search_half_window_sec
            or float(item["first_timestamp"]) > center + args.scan_search_half_window_sec
        ):
            continue
        p = Path(item["path"])
        if not p.is_absolute():
            p = pcap_dir / p
        if p.name in frame_bounds:
            start, end = frame_bounds[p.name]
            delta = (start + end) * 0.5 - center
            if abs(delta) <= args.scan_search_half_window_sec:
                candidates.append((abs(delta), (p,), start, end, delta))
            continue
        if str(p) not in cache:
            cache[str(p)] = pcap_scan_time_bounds(p, args.scan_gap_sec)
        fragments.extend(((p, start, end) for (start, end) in cache[str(p)]))
    for sources, start, end in merge_scan_fragments(fragments, args.scan_gap_sec):
        delta = (start + end) * 0.5 - center
        if abs(delta) <= args.scan_search_half_window_sec:
            candidates.append((abs(delta), sources, start, end, delta))
    return candidates


class RecordingAlignment:
    """Reproduce the selected new_data trial without mutating cached raw scans."""

    def __init__(self, frames, gps_streams):
        self.mode = CAMERA_TIMING_MODE
        if self.mode not in ("recorded", "uniform_source_clock"):
            raise ValueError("Unknown camera timing mode")
        self.clock = None
        if self.mode == "uniform_source_clock":
            ordered = sorted(frames, key=lambda f: float(f["timestamp"]))
            ids = np.asarray([f["source_frame_id"] for f in ordered], dtype=float)
            times = np.asarray([f["timestamp"] for f in ordered], dtype=float)
            if (
                len(ids) < 2
                or not np.isfinite(ids).all()
                or not np.isfinite(times).all()
                or np.any(np.diff(ids) <= 0)
            ):
                raise ValueError("Uniform camera clock requires ordered source frame IDs")
            period, intercept = np.polyfit(ids - ids[0], times - times[0], 1)
            if not np.isfinite(period) or period <= 0:
                raise ValueError("Invalid fitted frame period")
            self.clock = dict(
                schema="uniform_source_clock_v1",
                first_source_frame_id=int(ids[0]),
                reference_epoch_sec=float(times[0] + intercept),
                period_sec=float(period),
                fit_frame_count=len(ids),
                estimated_not_measured_exposure=True,
            )
        speed_records = []
        stream = gps_streams.get("RMC")
        for record in (() if stream is None else stream[1]):
            if not record.get("valid") or not record.get("checksum_valid"):
                continue
            try:
                row = (float(record["timestamp"]), float(record["speed_mps"]))
            except (KeyError, TypeError, ValueError):
                continue
            if np.isfinite(row).all() and row[1] >= 0:
                speed_records.append(row)
        self.speeds = np.asarray(sorted(speed_records), dtype=float).reshape(-1, 2)

    def image_time(self, record):
        if self.clock is None:
            return float(record.get("capture_timestamp", record["timestamp"]))
        return (
            self.clock["reference_epoch_sec"]
            + (int(record["source_frame_id"]) - self.clock["first_source_frame_id"])
            * self.clock["period_sec"]
        )

    def timing_metadata(self, record):
        return dict(
            profile_id=ALIGNMENT_PROFILE_ID,
            mode=self.mode,
            clock=self.clock,
            source_frame_id=record.get("source_frame_id"),
            recorded_epoch_sec=float(record["timestamp"]),
            alignment_camera_epoch_sec=self.image_time(record),
        )

    def compensate(self, raw, camera, target):
        if not LIDAR_MOTION_ENABLED:
            return raw
        k = int(np.searchsorted(self.speeds[:, 0], target))
        if k == 0 or k == len(self.speeds) or self.speeds[k, 0] - self.speeds[k - 1, 0] > 1.5:
            raise ValueError("Motion compensation requires bracketing valid GPS speeds")
        speed = float(np.interp(target, self.speeds[:, 0], self.speeds[:, 1]))
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


def box(row):
    b = [float(row[k]) for k in ["x1", "y1", "x2", "y2"]]
    import math

    if not all(map(math.isfinite, b)) or b[2] <= b[0] or b[3] <= b[1]:
        raise ValueError("Invalid bounding box")
    return b


def configure_live(offset_sec=0.0):
    """Configure the embedded recorded-clock pipeline in this interpreter."""
    global OFFSET_SEC
    OFFSET_SEC = offset_sec
    cv2.setNumThreads(FUSION_OPENCV_THREADS)


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)


def frame_time(row, mode="ptp"):
    if mode == "ptp":
        if row.get("timestamp_source") != "camera_ptp_rtcp_utc" or not row.get("timestamp_ns"):
            raise ValueError(
                "PTP camera timestamp missing; select --timestamp-mode recorded explicitly for legacy data"
            )
        value = int(row["timestamp_ns"]) / 1000000000.0
    else:
        value = float(row.get("capture_timestamp", row["timestamp"]))
    if not math.isfinite(value):
        raise ValueError("Invalid camera timestamp")
    return value


class GpsTail:
    """Read newly committed NMEA rows once, retaining only GGA/RMC."""

    def __init__(self, run):
        self.path = next(
            (p for p in (run / "gps/gps.jsonl", run / "gps.jsonl") if p.exists()),
            run / "gps/gps.jsonl",
        )
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
                    if kind in self.grouped and math.isfinite(float(row["timestamp"])):
                        self.grouped[kind].append(row)
                        changed = True
                except (ValueError, TypeError, KeyError):
                    continue
        if not changed:
            return None
        streams = {}
        for kind, rows in self.grouped.items():
            rows.sort(key=lambda row: float(row["timestamp"]))
            streams[kind] = (np.asarray([float(row["timestamp"]) for row in rows]), rows)
        return streams


class LiveProcessor:

    def __init__(self, options):
        self.options = options
        configure_live(options.offset_sec)
        self.camera = LiDAR2Camera(CAMERA_CALIBRATION)
        self.angles = lidar__load_calibration(LIDAR_CALIBRATION)
        self.detector = DXNNDetector(MODEL_PATH, CONFIDENCE_THRESHOLD)
        self.tracker = ObjectTracker()
        self.sequence = 0
        self.session_id = uuid.uuid4().hex
        self.scan_bounds = {}
        self.scan_cache = OrderedDict()
        self.exporter = None
        self.run = None
        self.alignment = None
        self.gps = None
        self.closed = False

    def close(self):
        if not self.closed:
            self.closed = True
            self.detector.dispose()

    def refresh_run(self, run):
        if self.run != run:
            self.run = run
            work = self.options.output / ".work"
            self.exporter = DamageArtifactExporter(
                work, work / "no_recording_input", save_images=SAVE_IMAGES, camera=self.camera
            )
            self.gps = GpsTail(run)
            self.alignment = RecordingAlignment([], {})
        streams = self.gps.read()
        if streams is not None:
            self.exporter.gps_streams = streams
            self.alignment = RecordingAlignment([], streams)

    def prune_pcaps(self, pcaps):
        paths = {str(Path(row["path"])) for row in pcaps}
        self.scan_bounds = {
            path: value for path, value in self.scan_bounds.items() if path in paths
        }

    def infer(self, run, records):

        def load_image(row):
            path = Path(row["path"])
            path = path if path.is_absolute() else run / path
            image = cv2.imread(str(path))
            if image is None:
                raise FileNotFoundError(path)
            return image

        return self.detector.iter_detect(records, load_image)

    def process(self, run, key, row, image, detections, pcaps):
        started = time.monotonic()
        if any(int(d.class_id) not in SUSPECT_CLASS_IDS for d in detections):
            raise ValueError("This terminal supports only Crack/Pothole classes 0/1")
        timestamp = frame_time(row, self.options.timestamp_mode)
        target = timestamp - self.options.offset_sec
        index = int(row["frame_index"])
        image_path = Path(row["path"])
        if not image_path.is_absolute():
            image_path = run / image_path
        args = SimpleNamespace(
            scan_search_half_window_sec=SCAN_SEARCH_HALF_WINDOW_SEC, scan_gap_sec=SCAN_GAP_SEC
        )
        result = None
        points, sources = ([], ())
        start = end = scan_time = delta = None
        error = ""
        status = "no_model_detection" if not detections else "lidar_scan_missing"
        motion = dict(applied=False, reason="not_required")
        if detections:
            candidates = scan_candidates(pcaps, run, target, {}, self.scan_bounds, args)
            if candidates:
                _, sources, start, end, delta = min(candidates, key=lambda item: item[0])
                scan_time = (start + end) * 0.5
                status = "lidar_scan_too_far"
                if abs(delta) <= MAX_SCAN_CENTER_DELTA_SEC:
                    try:
                        cache_key = (tuple(map(str, sources)), start, end)
                        if cache_key not in self.scan_cache:
                            points = decode_scan(sources, self.angles, start - 1e-06, end + 1e-06)
                            plane_error = ""
                            try:
                                plane = flatten(points, index)
                            except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                                plane, plane_error = None, str(exc)
                            self.scan_cache[cache_key] = (points, plane, plane_error)
                        points, result, plane_error = self.scan_cache[cache_key]
                        self.scan_cache.move_to_end(cache_key)
                        while len(self.scan_cache) > 4:
                            self.scan_cache.popitem(last=False)
                        if plane_error:
                            raise ValueError(plane_error)
                        status = "ok"
                        motion = dict(applied=False, reason="disabled")
                        if self.options.motion != "off":
                            try:
                                result = self.alignment.compensate(result, self.camera, target)
                                motion = dict(applied=True, **result.get("motion_compensation", {}))
                            except ValueError as exc:
                                if self.options.motion == "required":
                                    raise
                                motion = dict(applied=False, reason=str(exc))
                    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                        result = None
                        status, error = ("plane_or_scan_invalid", str(exc))
        accepted, objects, audit = validate_objects(
            detections, result, self.camera, POTHOLE_MIN_DEPTH_M
        )
        self.sequence += 1
        tracked, _ = self.tracker.update(
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
        folder = self.options.output / "runs" / key / "frames" / f"{index:08d}"
        if folder.exists():
            raise FileExistsError(f"Frame already committed: {folder}")
        stage = folder.with_name("." + folder.name + "." + uuid.uuid4().hex + ".pending")
        stage.mkdir(parents=True)
        try:
            self.exporter.directory = stage / "certifcate"
            self.exporter.directory.mkdir()
            self.exporter.pcap_cache.clear()
            damage_payloads = []
            artifacts = self.exporter.save(
                index,
                timestamp,
                image_path,
                report_detections,
                report_objects,
                points,
                np.empty((0, 3)) if result is None else result["sensor_xyz_m"],
                None if result is None else result["road_xyz_m"],
                scan_time if result is not None else None,
                pcap_sources=sources,
                scan_start=start,
                scan_end=end,
                scan_status=status,
                target_lidar_timestamp=target,
                camera_timing=dict(
                    profile_id=ALIGNMENT_PROFILE_ID,
                    mode=self.options.timestamp_mode,
                    timestamp_ns=row.get("timestamp_ns"),
                    timestamp_source=row.get("timestamp_source"),
                    recorded_epoch_sec=timestamp,
                    alignment_camera_epoch_sec=timestamp,
                ),
                motion_compensation=motion,
                payloads=damage_payloads,
            )
            for payload in damage_payloads:
                if payload.get("tracking"):
                    payload["tracking"]["scope"] = "live_process_session_across_run_rotation"
            if report_detections:
                pcap_path = self.exporter.directory / (detection_frame_stem(row, timestamp) + ".pcap")
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
                lidar_metadata = dict(
                    status="available" if len(points) else "unavailable",
                    analysis_status=status, scan_start_epoch_sec=start, scan_end_epoch_sec=end,
                    scan_reference_epoch_sec=scan_time,
                    camera_lidar_delta_ms=None if delta is None else delta * 1000,
                    delta_definition="scan_midpoint_minus_alignment_target",
                    timestamp_basis="pcap_record_sensor_utc", point_count=len(points),
                    pcap_attached=bool(pcap_files),
                    pcap_files=pcap_files,
                    pcap_copy_policy="selected_scan_raw_records_unchanged",
                    source_pcap_files=[dict(name=Path(p).name, source_run=str(Path(p).parent.parent))
                                       for p in sources],
                    calibration_files=self.exporter.calibration_ids,
                    camera_extrinsics=self.exporter.camera_extrinsics,
                    motion_compensation=motion,
                )
                pair = save_detection_frame(
                    self.exporter.directory, image_path, image, row, timestamp, key,
                    report_detections, damage_payloads,
                    matched_gps_metadata(self.exporter.gps_streams, timestamp),
                    lidar_metadata,
                )
                artifacts.extend(pair)
            files = {p.name: dict(bytes=p.stat().st_size, sha256=sha(p)) for p in artifacts}
            for path in artifacts:
                with path.open("rb") as stream:
                    os.fsync(stream.fileno())
            fsync_dir(self.exporter.directory)
            manifest = dict(frame_index=index, files=files, frame_log={}) if files else None
            summary = dict(
                schema_version=3,
                source_run=key,
                frame_index=index,
                source_frame_id=row.get("source_frame_id"),
                camera_timestamp=timestamp,
                camera_timestamp_ns=row.get("timestamp_ns"),
                target_lidar_timestamp=target,
                offset_sec=self.options.offset_sec,
                scan_start=start,
                scan_end=end,
                scan_delta_ms=None if delta is None else delta * 1000,
                status=status,
                error=error,
                model_count=len(detections),
                accepted_count=len(accepted),
                reported_count=len(report_detections),
                total_model_count=len(detections),
                total_reported_count=len(report_detections),
                damage_report_policy="first_confirmation_only",
                objects=audit,
                damage_details=damage_payloads,
                tracks=tracked,
                live_sequence=self.sequence,
                live_session_id=self.session_id,
                motion_compensation=motion,
                upload_enabled=self.options.upload,
                upload_manifest=manifest,
                upload_blocked_reason=triplet_error(files) if files else "",
                processing_seconds=time.monotonic() - started,
            )
            atomic_json(stage / "result.json", clean(summary))
            fsync_dir(stage)
            stage.replace(folder)
            fsync_dir(folder.parent)
            if self.options.upload and manifest:
                queue_upload(self.options.output, key, folder)
            return summary
        except BaseException:
            if stage.exists():
                shutil.rmtree(stage)
            raise


def queue_upload(output, key, folder):
    """Idempotent outbox creation; committed result.json is the recovery source."""
    output, folder = (Path(output), Path(folder))
    relative = folder.relative_to(output)
    task = output / "runs" / "outbox" / key / (folder.name + ".json")
    if not (folder / "upload_receipt.json").exists() and (not task.exists()):
        atomic_json(task, dict(frame_folder=relative.as_posix(), source_run=key))


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
    """Durable retry queue: HTTP API when PORTHOLE_API_URL is set, else the
    embedded byte/hash-verified SSH transport."""

    def __init__(self, output, timeout=30):
        self.output, self.timeout = (Path(output), timeout)
        self.stop_event = threading.Event()
        self.error = ""
        self.thread = threading.Thread(target=self.run, name="live-artifact-upload", daemon=True)
        self.thread.start()

    def run(self):
        uploader, active_key = (None, None)
        try:
            try:
                options = upload_options()
            except ValueError as exc:
                # Not configured: keep analysing and keep the outbox until .env
                # is filled in and main_live.py restarts.
                self.error = str(exc)
                self.stop_event.wait()
                return
            target = options.api_url if options.mode == "api" else f"{options.user}@{options.host}"
            print(f"[UPLOAD] {options.mode} -> {target}", flush=True)
            while not self.stop_event.is_set():
                jobs = sorted((self.output / "runs" / "outbox").glob("**/*.json"))
                if not jobs:
                    self.stop_event.wait(1)
                    continue
                blocked_reason = ""
                for job in jobs:
                    if self.stop_event.is_set():
                        break
                    try:
                        task = json.loads(job.read_text())
                        folder = (self.output / task["frame_folder"]).resolve()
                        folder.relative_to(self.output.resolve())
                        if (folder / "upload_receipt.json").exists():
                            job.unlink()
                            continue
                        row = json.loads((folder / "result.json").read_text())
                        reason = triplet_error(row["upload_manifest"].get("files"))
                        if reason:
                            blocked_reason = reason
                            self.error = reason
                            atomic_json(self.output / "runs" / "upload_status.json", dict(
                                updated_at=time.time(), error=reason, pending=len(jobs),
                                frame_folder=task["frame_folder"], retry=True,
                                state="waiting_for_complete_triplet"))
                            continue  # Keep this job, but do not block complete later frames.
                        key = task["source_run"]
                        if key != active_key:
                            if uploader:
                                uploader.close()
                            uploader = (
                                HttpUploader(options, timeout=self.timeout)
                                if options.mode == "api"
                                else PersistentUploader(
                                    options,
                                    Path(self.output.name) / key / "certifcate",
                                    timeout=self.timeout,
                                )
                            )
                            active_key = key
                        receipt = uploader.upload(folder / "certifcate", row["upload_manifest"])
                        atomic_json(folder / "upload_receipt.json", receipt)
                        job.unlink()
                        self.error = ""
                        print(
                            f"[UPLOAD] verified run={key} frame={row['frame_index']} files={receipt['files']}",
                            flush=True,
                        )
                    except PermanentUploadError as exc:
                        # Retrying cannot help and would block later frames; park it.
                        failed = self.output / "runs" / "outbox_failed" / job.relative_to(
                            self.output / "runs" / "outbox"
                        )
                        failed.parent.mkdir(parents=True, exist_ok=True)
                        job.replace(failed)
                        atomic_json(failed.with_suffix(".error.json"),
                                    dict(failed_at=time.time(), error=str(exc)))
                        print(f"[UPLOAD] rejected by server, moved to {failed}: {exc}", flush=True)
                    except Exception as exc:
                        self.error = str(exc)
                        atomic_json(
                            self.output / "runs" / "upload_status.json",
                            dict(
                                updated_at=time.time(),
                                error=self.error,
                                pending=len(jobs),
                                retry=True,
                            ),
                        )
                        print(f"[UPLOAD] retry pending: {exc}", flush=True)
                        if uploader:
                            uploader.close()
                        uploader, active_key = (None, None)
                        self.stop_event.wait(5)
                        break
                if blocked_reason:
                    self.error = blocked_reason
                    self.stop_event.wait(5)
        except Exception as exc:
            self.error = str(exc)
            print(f"[UPLOAD] worker stopped: {exc}", flush=True)
        finally:
            if uploader:
                uploader.close()

    def close(self, drain_seconds=10):
        deadline = time.monotonic() + drain_seconds
        while (
            any((self.output / "runs" / "outbox").glob("**/*.json"))
            and time.monotonic() < deadline
            and (not self.error)
        ):
            time.sleep(0.1)
        self.stop_event.set()
        self.thread.join(timeout=self.timeout + 5)


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
        layouts = [
            self.path / "lidar",
            self.path / "lidar/lidar_pcap",
            self.path / "lidar/lidar_pcap_all",
        ]
        self.pcap_dir = next((p for p in layouts if (p / "pcaps.jsonl").exists()), layouts[0])
        self.pcaps = JsonlTail(self.pcap_dir / "pcaps.jsonl")
        self.closed = False
        self.validated_rows = 0

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
        for row in self.pcaps.rows:
            path = Path(row["path"])
            if not path.is_absolute():
                path = self.pcap_dir / path
            if not path.is_file():
                raise FileNotFoundError(f"Committed PCAP missing: {path}")
            if "disk_bytes" in row and path.stat().st_size != int(row["disk_bytes"]):
                raise RuntimeError(f"Committed PCAP size mismatch: {path}")
            yield dict(row, path=str(path))


def ready_frames(run, count, pcaps, options, allow_tail):
    watermark = max((float(p["last_timestamp"]) for p in pcaps), default=float("-inf"))
    selected = []
    for row in run.frames.rows[count:]:
        timestamp = frame_time(row, options.timestamp_mode)
        if (
            not allow_tail
            and timestamp - options.offset_sec + SCAN_SEARCH_HALF_WINDOW_SEC > watermark
        ):
            break
        selected.append(row)
        if len(selected) >= options.batch_size:
            break
    return list(selected)


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


def recover_committed(output, key, run, count, upload):
    """Recover a commit that survived a crash before the checkpoint write."""
    while count < len(run.frames.rows):
        row = run.frames.rows[count]
        folder = output / "runs" / key / "frames" / f"{int(row['frame_index']):08d}"
        path = folder / "result.json"
        if not path.exists():
            break
        saved = json.loads(path.read_text())
        if saved["frame_index"] != int(row["frame_index"]) or saved["source_run"] != key:
            raise ValueError(f"Committed frame does not match source: {path}")
        if upload and saved.get("upload_enabled") and saved.get("upload_manifest"):
            queue_upload(output, key, folder)
        count += 1
    return count


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


def discover(root, mode="ptp"):
    result = set()
    for pattern in ("*/frames/frames.jsonl", "*/*/frames/frames.jsonl"):
        for path in root.glob(pattern):
            if mode == "ptp":
                with path.open("rb") as stream:
                    first = stream.readline()
                if first.endswith(b"\n") and first.strip():
                    if json.loads(first).get("timestamp_source") != "camera_ptp_rtcp_utc":
                        continue
            result.add(path.parent.parent.resolve())
    return sorted(result, key=lambda p: str(p))


# This known previous build has the same analysis rules, but kept metadata at
# the output root. Only that build may resume through the layout migration.
LEGACY_LAYOUT_CODE_SHA256 = "6eb2b3507263a16f00d32223f6b93074d87e73098f5035cd64ac1b918b1ecee0"
PRE_FRAME_EXPORT_CODE_SHA256 = "8ae0b269b564f59b632e1516daed0e1a93d6cfdff6310476433a33bddeca8141"

# Migration metadata only: this previous model is never opened or inferred.
PREVIOUS_DUAL_CODE_SHA256 = "72a6d05fecd2ae712dec75448fa9d33ee9d9ef0075709672b0d20c02f0b33ac6"
PRE_TRIPLET_CODE_SHA256 = "2d8e64973a5d7b6c9f23b599e7b3a774c51994a357c0dff2c7f50ef40334af3a"
PREVIOUS_EXTRA_MODEL_HASHES = {
    "pothole_best2.dxnn": "559bf8a7776ab373307ea1ce8b14a16c73f12ef075ae363aa806249a77160086"
}


def analysis_signature(options):
    return dict(
        root=str(options.root.resolve()),
        run=None if options.run is None else str(options.run.resolve()),
        timestamp_mode=options.timestamp_mode,
        offset_sec=options.offset_sec,
        motion=options.motion,
        code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        model_hashes={p.name: sha(p) if p.is_file() else None
                      for p in (MODEL_PATH,)},
    )


class OutputSettingsChanged(ValueError):
    """The output checkpoint was written by other code, settings or input."""


def prepare_run_metadata(output, signature):
    """Move old metadata under runs, preserving frame paths and queue positions.

    Caller holds worker.lock, shared with the previous version. Each move is
    atomic; a restart can finish a partially completed migration. Conflicting
    files or incompatible analysis settings are rejected before any moves.
    """
    output = Path(output)
    destination = output / "runs"
    old_state, state_path = output / "state.json", destination / "state.json"
    state = None
    for path in (state_path, old_state):
        if not path.exists():
            continue
        candidate = json.loads(path.read_text())
        expected = dict(signature)
        if candidate.get("schema_version") == 3:
            expected["code_sha256"] = LEGACY_LAYOUT_CODE_SHA256
            expected.pop("model_hashes", None)
        elif candidate.get("schema_version") == 4:
            expected["code_sha256"] = PRE_FRAME_EXPORT_CODE_SHA256
            expected.pop("model_hashes", None)
        elif candidate.get("schema_version") == 5:
            expected["code_sha256"] = PREVIOUS_DUAL_CODE_SHA256
            expected["model_hashes"] = {**signature["model_hashes"], **PREVIOUS_EXTRA_MODEL_HASHES}
        elif candidate.get("schema_version") == 6:
            expected["code_sha256"] = PRE_TRIPLET_CODE_SHA256
        elif candidate.get("schema_version") != 7:
            raise ValueError(f"Unsupported live checkpoint: {path}")
        if candidate.get("configuration") != expected:
            raise OutputSettingsChanged(
                "Live output has different source/settings; choose a new --output"
            )
        if state is not None and candidate != state:
            raise ValueError("Conflicting root and runs checkpoints; no files were moved")
        state = candidate

    moves = []
    for name in ("state.json", "status.json", "upload_status.json"):
        source = output / name
        if source.exists():
            moves.append((source, destination / name))
    old_outbox = output / "outbox"
    if old_outbox.exists():
        for source in sorted(old_outbox.rglob("*")):
            if source.is_file():
                moves.append((source, destination / "outbox" / source.relative_to(old_outbox)))
    for source, target in moves:
        if source.is_symlink() or target.is_symlink():
            raise ValueError("Metadata migration does not follow symbolic links")
        if target.exists() and (not target.is_file() or source.read_bytes() != target.read_bytes()):
            raise ValueError(f"Conflicting metadata destination: {target}")
    destination.mkdir(parents=True, exist_ok=True)
    for source, target in moves:
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            source.unlink()  # Identical duplicate, checked above.
        else:
            source.replace(target)
        fsync_dir(target.parent)
        fsync_dir(source.parent)
    if old_outbox.exists():
        for directory in sorted(old_outbox.rglob("*"), key=lambda p: len(p.parts), reverse=True):
            if directory.is_dir():
                directory.rmdir()
        old_outbox.rmdir()
        fsync_dir(output)
    if state is not None and state["schema_version"] in (3, 4, 5, 6):
        state.setdefault("upgrades", []).append(dict(
            at_epoch_sec=time.time(), previous_schema=state["schema_version"],
            previous_configuration=state["configuration"],
            processed_rows_at_upgrade={key: row.get("processed_rows", 0)
                                       for key, row in state.get("runs", {}).items()},
            policy="triplet_upload_preserve_previous_results_and_upload_queue",
        ))
        state["schema_version"] = 7
        state["configuration"] = signature
        atomic_json(state_path, state)
        fsync_dir(destination)
    return state


def run_service(options, processor_factory=LiveProcessor):
    import fcntl
    from contextlib import ExitStack

    root, output = (options.root.resolve(), options.output.resolve())
    options.output = output
    if not root.is_dir():
        raise FileNotFoundError(f"Recording root is unavailable: {root}")
    if str(output).startswith("/mnt/ssd/") and (not os.path.ismount("/mnt/ssd")):
        raise RuntimeError("SSD is not mounted")
    output.mkdir(parents=True, exist_ok=True)
    state_path = output / "runs" / "state.json"
    signature = analysis_signature(options)
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    with ExitStack() as resources:
        lock = resources.enter_context((output / "worker.lock").open("a"))
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(sig, stop)
            resources.callback(signal.signal, sig, previous)
        previous_output = previous_uploader = None
        try:
            state = prepare_run_metadata(output, signature)
        except OutputSettingsChanged:
            # main_live.py, its settings, input or model changed since this output
            # was written. Keep those results in a dated folder (the old lock stays
            # held) and start a fresh output, as a new --output would.
            stamp = time.strftime("%Y%m%d_%H%M%S")
            previous_output = output.with_name(f"{output.name}_{stamp}")
            suffix = 1
            while previous_output.exists():
                previous_output = output.with_name(f"{output.name}_{stamp}_{suffix}")
                suffix += 1
            output.rename(previous_output)
            output.mkdir()
            lock = resources.enter_context((output / "worker.lock").open("a"))
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            print(
                f"[LIVE] analysis code/settings changed; previous output moved to {previous_output}",
                flush=True,
            )
            state = None
            # Uploads still queued there keep going from the moved folder.
            if options.upload and any((previous_output / "runs" / "outbox").glob("**/*.json")):
                previous_uploader = UploadWorker(previous_output)
        if state is None:
            paths = (
                [options.run.resolve()] if options.run else discover(root, options.timestamp_mode)
            )
            latest = (
                max(paths, key=lambda p: (p / "frames/frames.jsonl").stat().st_mtime)
                if paths
                else None
            )
            state = dict(
                schema_version=7,
                configuration=signature,
                watch_since=time.time(),
                initial_run=str(latest) if latest else None,
                ignored_runs=[str(p.relative_to(root)) for p in paths if p != latest],
                runs={},
            )
            atomic_json(state_path, state)
        if options.nice:
            os.nice(options.nice)
            if shutil.which("ionice"):
                subprocess.run(["ionice", "-c", "3", "-p", str(os.getpid())], check=False)
        processor = uploader = None
        failure = ""
        runs = {}
        boundary_run = None
        processed = 0
        started = time.monotonic()
        last_status = 0
        previous_pending = None
        source_finished_at = None
        print(
            f"[LIVE] root={root} output={output} upload={options.upload} timestamp={options.timestamp_mode} offset={options.offset_sec}s motion={options.motion}; disk-backed FIFO, no skipped frames",
            flush=True,
        )
        try:
            while not stopping:
                paths = (
                    [options.run.resolve()]
                    if options.run
                    else discover(root, options.timestamp_mode)
                )
                for path in paths:
                    key = str(path.relative_to(root))
                    if key not in state["runs"]:
                        if key in state["ignored_runs"]:
                            continue
                        state["runs"][key] = dict(
                            processed_rows=0,
                            recorded_rows=0,
                            complete=False,
                            first_frame_sha256=None,
                        )
                    if not state["runs"][key]["complete"]:
                        count_manifest(path / "frames/frames.jsonl", state["runs"][key])
                active_keys = [
                    key for key in sorted(state["runs"]) if not state["runs"][key]["complete"]
                ]
                for key in list(runs):
                    if key not in active_keys[:2]:
                        del runs[key]
                for key in active_keys[:2]:
                    runs.setdefault(key, Run(root / key))
                for key, run in runs.items():
                    run.refresh()
                    entry = state["runs"][key]
                    count_manifest(run.frames.path, entry)
                    first_hash = source_signature(run)
                    if entry["first_frame_sha256"] not in (None, first_hash):
                        raise ValueError(f"Source recording was replaced: {run.path}")
                    entry["first_frame_sha256"] = first_hash
                    count = recover_committed(
                        output, key, run, entry["processed_rows"], options.upload
                    )
                    if count > len(run.frames.rows):
                        raise ValueError(f"Source manifest shrank since checkpoint: {key}")
                    entry["processed_rows"] = count
                extra = []
                if runs and (not options.run):
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
                    and time.monotonic() - source_finished_at >= options.poll_seconds
                )
                did_work = False
                for key in sorted(runs):
                    run, entry = (runs[key], state["runs"][key])
                    count = entry["processed_rows"]
                    if count == len(run.frames.rows):
                        entry["complete"] = run.closed
                        continue
                    ready = ready_frames(run, count, pcaps, options, allow_tail and run.closed)
                    if not ready:
                        break
                    if shutil.disk_usage(output).free < options.min_free_gb * 1024**3:
                        print(
                            "[LIVE] paused: insufficient free space for analysis output", flush=True
                        )
                        break
                    wait_for_collector(lambda: stopping)
                    if processor is None:
                        processor = processor_factory(options)
                    processor.prune_pcaps(pcaps)
                    if options.upload and uploader is None:
                        uploader = UploadWorker(output)
                    processor.refresh_run(run.path)
                    if options.max_frames:
                        ready = ready[: options.max_frames - processed]
                    stream = processor.infer(run.path, ready)
                    try:
                        for row, image, detections in stream:
                            if stopping:
                                break
                            wait_for_collector(lambda: stopping)
                            processor.process(run.path, key, row, image, detections, pcaps)
                            entry["processed_rows"] += 1
                            processed += 1
                            did_work = True
                            atomic_json(state_path, state)
                            if options.max_frames and processed >= options.max_frames:
                                stopping = True
                                break
                    finally:
                        stream.close()
                    break
                pending = sum(
                    (
                        entry["recorded_rows"] - entry["processed_rows"]
                        for entry in state["runs"].values()
                    )
                )
                oldest = next(
                    (
                        frame_time(
                            run.frames.rows[state["runs"][key]["processed_rows"]],
                            options.timestamp_mode,
                        )
                        for key, run in sorted(runs.items())
                        if state["runs"][key]["processed_rows"] < len(run.frames.rows)
                    ),
                    None,
                )
                oldest_age = 0 if oldest is None else max(0, time.time() - oldest)
                now = time.monotonic()
                if now - last_status >= 5 or stopping:
                    status = dict(
                        updated_at=time.time(),
                        state="stopping" if stopping else "running",
                        processed_this_session=processed,
                        pending_frames=pending,
                        oldest_pending_age_seconds=oldest_age,
                        full_runs_in_memory=len(runs),
                        average_fps=processed / max(0.001, now - started),
                        upload_pending=sum(
                            (1 for _ in (output / "runs" / "outbox").glob("**/*.json"))
                        ),
                        upload_error="" if uploader is None else uploader.error,
                        runs=state["runs"],
                    )
                    atomic_json(output / "runs" / "status.json", status)
                    atomic_json(state_path, state)
                    print(
                        f"[ANALYSIS {time.strftime('%H:%M:%S')}] processed={processed} pending={pending} avgFPS={status['average_fps']:.2f} oldestAge={oldest_age:.1f}s uploadQ={status['upload_pending']}",
                        flush=True,
                    )
                    if previous_pending is not None and pending > previous_pending + 150:
                        print(
                            "[ANALYSIS] backlog increasing; recordings are retained and frames are not skipped",
                            flush=True,
                        )
                    previous_pending, last_status = (pending, now)
                if (
                    options.upload
                    and uploader is None
                    and any((output / "runs" / "outbox").glob("**/*.json"))
                ):
                    uploader = UploadWorker(output)
                if uploader is not None and (not uploader.thread.is_alive()):
                    raise RuntimeError(
                        f"Upload worker stopped; pending files retained: {uploader.error}"
                    )
                if (
                    options.once
                    and pending == 0
                    and state["runs"]
                    and all((entry["complete"] for entry in state["runs"].values()))
                ):
                    break
                if not did_work:
                    time.sleep(options.poll_seconds)
        except InterruptedError:
            if not stopping:
                raise
        except Exception as exc:
            failure = str(exc)
            raise
        finally:
            atomic_json(state_path, state)
            if processor:
                processor.close()
            if uploader:
                uploader.close(options.drain_seconds)
            if previous_uploader:
                previous_uploader.close(options.drain_seconds)
            atomic_json(
                output / "runs" / "status.json",
                dict(
                    updated_at=time.time(),
                    state="failed" if failure else "stopped",
                    error=failure,
                    processed_this_session=processed,
                    average_fps=processed / max(0.001, time.monotonic() - started),
                    pending_frames=sum(
                        (
                            entry["recorded_rows"] - entry["processed_rows"]
                            for entry in state["runs"].values()
                        )
                    ),
                    upload_pending=sum((1 for _ in (output / "runs" / "outbox").glob("**/*.json"))),
                    upload_error="" if uploader is None else uploader.error,
                    runs=state["runs"],
                ),
            )
            print(
                f"[LIVE] stopped; processed={processed}; committed checkpoint and pending uploads retained",
                flush=True,
            )


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Standalone PTP camera/LiDAR FIFO analysis and server upload."
    )
    p.add_argument("--root", type=Path, default=Path("/mnt/ssd/porthole_runs"))
    p.add_argument("--run", type=Path, help="Follow one recording instead of discovering new runs")
    p.add_argument("--output", type=Path, default=Path("/mnt/ssd/porthole_live_analysis"))
    p.add_argument("--upload", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--timestamp-mode", choices=["ptp", "recorded"], default="ptp")
    p.add_argument("--offset-sec", type=float, default=0.0)
    p.add_argument("--motion", choices=["auto", "required", "off"], default="auto")
    p.add_argument("--batch-size", type=int, default=6)
    p.add_argument("--poll-seconds", type=float, default=0.5)
    p.add_argument("--min-free-gb", type=float, default=10)
    p.add_argument("--nice", type=int, default=10)
    p.add_argument("--max-frames", type=int, default=0)
    p.add_argument(
        "--once",
        action="store_true",
        help="Exit when the selected recordings have ended and drained",
    )
    p.add_argument("--drain-seconds", type=float, default=10)
    options = p.parse_args(argv)
    if options.timestamp_mode == "ptp" and options.offset_sec != 0:
        p.error("PTP mode requires zero empirical time offset")
    if (
        not all(
            (
                math.isfinite(x)
                for x in (
                    options.offset_sec,
                    options.poll_seconds,
                    options.min_free_gb,
                    options.drain_seconds,
                )
            )
        )
        or options.batch_size < 1
        or options.poll_seconds <= 0
        or (options.max_frames < 0)
        or (options.min_free_gb < 0)
        or (options.drain_seconds < 0)
        or (not 0 <= options.nice <= 19)
    ):
        p.error("Invalid batch, polling, frame limit, disk reserve, or nice setting")
    if options.run:
        options.run.resolve().relative_to(options.root.resolve())
    return options


def main():
    run_service(parse_args())


if __name__ == "__main__":
    main()
