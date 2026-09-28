"""CU22 hardware decoding, RTCP timestamps and a bounded frame queue.

The GStreamer worker runs this module with --worker and PYTHONNOUSERSITE=1.
OpenCV/numpy are loaded only by the collector when it converts NV12 pixels.
"""

from __future__ import annotations
import collections
import fcntl
import json
import os
import select
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from ptp import GUARD, NS, RtpClock, sender_reports


class CameraStream:
    def __init__(self, url):
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        self.Gst = Gst
        self.clock = RtpClock()
        self.lock = threading.Lock()
        self.pts_map = collections.OrderedDict()
        self.reports = []
        self.sessions = set()
        self.errors = []
        gi.require_version("GstVideo", "1.0")
        from gi.repository import GstVideo

        self.GstVideo = GstVideo
        decoder = next(
            (n for n in ("vah264dec", "vaapih264dec") if Gst.ElementFactory.find(n)), None
        )
        if decoder is None:
            raise RuntimeError("No hardware H264 decoder available")
        print(
            "[camera] hardware_decode=" + decoder + " output=NV12 pts=RTP",
            file=sys.stderr,
            flush=True,
        )
        output = (
            decoder
            + (" ! vaapipostproc" if decoder == "vaapih264dec" else "")
            + " ! video/x-raw,format=NV12"
        )
        self.pipeline = Gst.parse_launch(
            "rtspsrc name=source protocols=tcp latency=50 buffer-mode=none tcp-timeout=3000000 ! rtph264depay name=depay ! h264parse config-interval=-1 ! "
            + output
            + " ! appsink name=frames sync=false max-buffers=128 drop=false"
        )
        self.pipeline.get_by_name("source").set_property("location", url)
        self.pipeline.get_by_name("source").connect("new-manager", self._manager)
        self.pipeline.get_by_name("depay").get_static_pad("sink").add_probe(
            Gst.PadProbeType.BUFFER, self._rtp_probe
        )
        self.sink = self.pipeline.get_by_name("frames")
        self.bus = self.pipeline.get_bus()

    def _rtp_probe(self, pad, info):
        b = info.get_buffer()
        if b is not None and b.pts != self.Gst.CLOCK_TIME_NONE and b.get_size() >= 12:
            head = b.extract_dup(0, 12)
            if head[0] >> 6 == 2:
                rtp, ssrc = struct.unpack_from("!II", head, 4)
                with self.lock:
                    self.pts_map[int(b.pts)] = (rtp, ssrc)
                    while len(self.pts_map) > 1024:
                        self.pts_map.popitem(last=False)
        return self.Gst.PadProbeReturn.OK

    def _manager(self, source, manager):
        manager.connect("on-new-ssrc", self._session)

    def _session(self, manager, index, ssrc):
        if index not in self.sessions:
            manager.emit("get-internal-session", index).connect("on-receiving-rtcp", self._rtcp)
            self.sessions.add(index)

    def _rtcp(self, session, buffer):
        try:
            for report in sender_reports(buffer.extract_dup(0, buffer.get_size())):
                utc = time.time_ns()
                mono = time.monotonic_ns()
                with self.lock:
                    valid = self.clock.update(report, utc, mono)
                    self.reports.append(
                        dict(
                            report,
                            host_utc_ns=utc,
                            host_monotonic_ns=mono,
                            mapping_ready=valid,
                            mapping_error=self.clock.last_error,
                        )
                    )
                    # Drained by the collector; bound memory if its writer fails.
                    if len(self.reports) > 256:
                        del self.reports[:-256]
        except Exception as exc:
            self.errors.append(str(exc))

    def start(self):
        if self.pipeline.set_state(self.Gst.State.PLAYING) == self.Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Camera pipeline failed to start")

    def read(self):
        sample = self.sink.emit("try-pull-sample", 100 * self.Gst.MSECOND)
        if sample is None:
            msg = self.bus.pop_filtered(self.Gst.MessageType.ERROR | self.Gst.MessageType.EOS)
            if msg:
                raise RuntimeError(
                    str(msg.parse_error())
                    if msg.type == self.Gst.MessageType.ERROR
                    else "Camera EOS"
                )
            return None
        b = sample.get_buffer()
        mono = time.monotonic_ns()
        arrival = time.time_ns()
        pts = int(b.pts)
        with self.lock:
            rtp_info = self.pts_map.get(pts)
            timestamp, error = (
                self.clock.timestamp(*rtp_info, mono)
                if rtp_info
                else (None, "rtp_pts_mapping_missing")
            )
            reports = self.reports
            self.reports = []
        payload = b.extract_dup(0, b.get_size())
        extra = {}
        info = self.GstVideo.VideoInfo.new_from_caps(sample.get_caps())
        w, h = info.width, info.height
        meta = self.GstVideo.buffer_get_video_meta(b)
        stride = meta.stride if meta else info.stride
        offset = meta.offset if meta else info.offset
        if info.finfo.name != "NV12" or w % 2 or h % 2:
            raise RuntimeError("Invalid NV12 format")
        if stride[0] == w and stride[1] == w and offset[1] == offset[0] + w * h:
            payload = payload[offset[0] : offset[0] + w * h * 3 // 2]
        else:
            payload = b"".join(
                payload[offset[p] + r * stride[p] : offset[p] + r * stride[p] + w]
                for p, rows in ((0, h), (1, h // 2))
                for r in range(rows)
            )
        extra = dict(width=w, height=h)
        return dict(
            payload=payload,
            pts_ns=pts,
            arrival_utc_ns=arrival,
            **extra,
            arrival_monotonic_ns=mono,
            rtp=rtp_info[0] if rtp_info else None,
            ssrc=rtp_info[1] if rtp_info else None,
            camera_utc_ns=timestamp,
            timestamp_source="rtcp_sender_report" if timestamp is not None else None,
            timestamp_error=error,
            is_keyframe=not b.has_flags(self.Gst.BufferFlags.DELTA_UNIT),
            reports=reports,
        )

    def close(self):
        self.pipeline.set_state(self.Gst.State.NULL)


HEADER = struct.Struct("<II")


class GstFrame:
    def __init__(self, pts, width, height, arrival, mono, payload):
        self.pts = None if pts < 0 else pts
        self.width, self.height = width, height
        self.arrival, self.arrival_monotonic = arrival, mono
        self.payload = payload

    def to_ndarray(self, format="bgr24"):
        import cv2
        import numpy as np

        if format != "bgr24":
            raise ValueError(format)
        nv12 = np.frombuffer(self.payload, np.uint8).reshape(self.height * 3 // 2, self.width)
        return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)


class GstContainer:
    def __init__(self, url, timeout=15):
        env = os.environ.copy()
        # System gi and VA drivers run without user OpenCV/FFmpeg libraries.
        env["PYTHONNOUSERSITE"] = "1"
        self.process = subprocess.Popen(
            ["/usr/bin/python3", "-u", str(Path(__file__).resolve()), "--worker"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            bufsize=0,
            env=env,
        )
        try:
            fcntl.fcntl(self.process.stdout.fileno(), fcntl.F_SETPIPE_SZ, 1024 * 1024)
        except OSError:
            pass
        self.closed = False
        self.first = None
        self.decoder = "GStreamer VA-API (see startup log)"
        try:
            self.process.stdin.write((json.dumps({"url": url}) + "\n").encode())
            self.process.stdin.close()
            self.first = self._frame(timeout)
            self.stream = SimpleNamespace(
                time_base=1e-9,
                thread_type=None,
                codec_context=SimpleNamespace(width=self.first.width, height=self.first.height),
            )
        except BaseException:
            self.close()
            raise

    def _read(self, size, deadline):
        data = bytearray(size)
        view = memoryview(data)
        done = 0
        while done < size:
            if self.closed:
                raise EOFError("Hardware camera closed")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Hardware camera frame timeout")
            ready, _, _ = select.select([self.process.stdout], [], [], min(0.2, remaining))
            if not ready:
                continue
            count = self.process.stdout.readinto(view[done:])
            if not count:
                raise EOFError(f"Hardware decoder exited: {self.process.poll()}")
            done += count
        return data

    def _frame(self, timeout):
        deadline = time.monotonic() + timeout
        metadata_size, size = HEADER.unpack(self._read(HEADER.size, deadline))
        if not 0 < metadata_size < 1024 * 1024:
            raise RuntimeError("Invalid metadata size")
        metadata = json.loads(self._read(metadata_size, deadline))
        pts = metadata["pts_ns"]
        w = metadata["width"]
        h = metadata["height"]
        arrival = metadata["arrival_utc_ns"] / 1e9
        mono = metadata["arrival_monotonic_ns"] / 1e9
        if not (0 < w <= 8192 and 0 < h <= 8192 and w % 2 == h % 2 == 0 and size == w * h * 3 // 2):
            raise RuntimeError("Invalid hardware frame header")
        frame = GstFrame(pts, w, h, arrival, mono, self._read(size, deadline))
        frame.metadata = metadata
        return frame

    def decode(self, video=0):
        while not self.closed:
            frame, self.first = self.first, None
            yield frame if frame is not None else self._frame(3)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process.stdout.close()
        if self.process.stdin is not None and not self.process.stdin.closed:
            self.process.stdin.close()
        self.first = None


class CameraCapture:
    """Read PTP-qualified frames while preserving capture timestamps and order."""

    nonblocking_read = True

    def __init__(self, url, buffer_size=8, watchdog_sec=5.0, connection_timeout_sec=1.0):
        self.url = url
        self.watchdog_sec = watchdog_sec
        self.connection_timeout_sec = connection_timeout_sec
        self.lock = threading.Lock()
        self.frames = deque()
        self.buffer_size = max(1, buffer_size)

        self.running = True
        self.opened = False
        self.last_error = None
        self.started_at = time.time()
        self.frame_timestamp = 0.0
        self.frame_id = 0
        self.decoded_count = 0
        self.delivered_count = 0
        self.selected_count = 0
        self.skipped_count = 0
        self.overwritten_count = 0
        self.read_failure_count = 0
        self.pts_missing_count = 0
        self.pts_settling_skipped_count = 0
        self.settle_timeout_count = 0
        self.timestamp_backstep_dropped_count = 0
        self.buffer_high_watermark = 0
        self.last_info = {}
        self.grabbed_frame = None
        self.time_base = None
        self.next_capture_deadline = None
        self.width = 0
        self.height = 0
        self.startup_settled = False
        self.first_pts_arrival = None
        self.stream_reset_count = 0
        self.last_emitted_capture = None

        # Open here, not on the worker thread: the collector calls isOpened()
        # immediately after constructing a capture and treats False as a failed
        # connection, so an asynchronous open reads as a dead camera.
        self.container, self.stream = self._open()
        self.stream.thread_type = "AUTO"
        self.time_base = float(self.stream.time_base) if self.stream.time_base else 0.0
        self.width = self.stream.codec_context.width
        self.height = self.stream.codec_context.height
        self.opened = True

        self.thread = threading.Thread(target=self._run, name="ptp-camera-capture", daemon=True)
        self.thread.start()

    def isOpened(self):
        with self.lock:
            if not self.running or not self.opened or not self.thread.is_alive():
                return False
            reference = self.frame_timestamp or self.started_at
        if self.watchdog_sec <= 0:
            return True
        return time.time() - reference <= self.watchdog_sec

    def read(self):
        with self.lock:
            if not self.frames:
                return False, None
            image, info = self.frames.popleft()
            self.last_info = info
            self.delivered_count += 1
            return True, image

    def get_last_frame_info(self):
        with self.lock:
            return dict(self.last_info)

    def stats(self):
        with self.lock:
            result = {
                "mode": "gstreamer_vaapi_ptp",
                "hardware_decoder": getattr(self.container, "decoder", None),
                "decoded_frames": self.decoded_count,
                "delivered_frames": self.delivered_count,
                "selected_frames": self.selected_count,
                "skipped_frames": self.skipped_count,
                "overwritten_frames": self.overwritten_count,
                "read_failures": self.read_failure_count,
                "pts_missing_frames": self.pts_missing_count,
                "pts_settling_skipped_frames": self.pts_settling_skipped_count,
                "pts_settle_timeouts": self.settle_timeout_count,
                "pts_stream_resets": self.stream_reset_count,
                "timestamp_backstep_dropped_frames": (self.timestamp_backstep_dropped_count),
                "pts_startup_settled": self.startup_settled,
                "buffer_capacity": self.buffer_size,
                "buffer_pending": len(self.frames),
                "buffer_high_watermark": self.buffer_high_watermark,
                "camera_time_base": self.time_base,
                "last_frame_timestamp": self.frame_timestamp,
                "last_error": self.last_error,
            }
        result.update(
            mode="gstreamer_vaapi_ptp",
            ptp_qualification=GUARD.qualification(),
            ptp_rejected=dict(GUARD.rejected),
        )
        return result

    def grab(self):
        ok, frame = self.read()
        self.grabbed_frame = frame if ok else None
        return ok

    def retrieve(self):
        frame = self.grabbed_frame
        if frame is None:
            return False, None
        self.grabbed_frame = None
        return True, frame

    def release(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=6.0)

    def _open(self):
        container = GstContainer(self.url, timeout=max(15.0, self.connection_timeout_sec * 5))
        return container, container.stream

    def _run(self):
        try:
            for frame in self.container.decode(video=0):
                if not self.running:
                    break
                meta = frame.metadata
                utc = meta["camera_utc_ns"]
                with self.lock:
                    self.decoded_count += 1
                    self.frame_timestamp = frame.arrival
                for report in meta["reports"]:
                    GUARD.log("rtcp_sender_reports.jsonl", report)
                ok, reason = GUARD.qualification()
                if utc is None or not ok:
                    GUARD.reject("camera_" + (meta["timestamp_error"] or reason))
                    self.pts_settling_skipped_count += 1
                    continue
                if self.last_emitted_capture is not None and utc <= self.last_emitted_capture:
                    self.timestamp_backstep_dropped_count += 1
                    continue
                image = frame.to_ndarray()
                self.startup_settled = True
                info = dict(
                    capture_timestamp=utc / NS,
                    timestamp_ns=utc,
                    arrival_timestamp=frame.arrival,
                    arrival_monotonic=frame.arrival_monotonic,
                    arrival_timestamp_ns=meta["arrival_utc_ns"],
                    camera_pts=frame.pts,
                    camera_pts_seconds=frame.pts / NS,
                    camera_time_base=1e-9,
                    pts_epoch_offset=(utc - frame.pts) / NS,
                    pts_offset_converged=True,
                    camera_rtp_timestamp=meta["rtp"],
                    camera_ssrc=meta["ssrc"],
                    timestamp_source="camera_ptp_rtcp_utc",
                    capture_mode="gstreamer_vaapi_ptp",
                    ptp_grandmaster=GUARD.status()[0]["camera"]["grandmaster_identity"],
                )
                with self.lock:
                    self.frame_id += 1
                    self.selected_count += 1
                    info["source_frame_id"] = self.frame_id
                    if len(self.frames) >= self.buffer_size:
                        self.frames.popleft()
                        self.overwritten_count += 1
                    self.frames.append((image, info))
                    self.last_emitted_capture = utc
                    self.buffer_high_watermark = max(self.buffer_high_watermark, len(self.frames))
        except Exception as exc:
            self.last_error = repr(exc)
            self.read_failure_count += 1
        finally:
            self.opened = False
            self.container.close()


def worker_main():
    request = json.loads(sys.stdin.readline())
    stream = CameraStream(request["url"])
    out = sys.stdout.buffer
    stream.start()
    try:
        while True:
            frame = stream.read()
            if frame is None:
                continue
            payload = frame.pop("payload")
            metadata = json.dumps(frame).encode()
            out.write(struct.pack("<II", len(metadata), len(payload)))
            out.write(metadata)
            out.write(payload)
            out.flush()
    finally:
        stream.close()


if __name__ == "__main__":
    if sys.argv[1:] != ["--worker"]:
        raise SystemExit("This module is the camera worker; start collect_data.py instead.")
    worker_main()
