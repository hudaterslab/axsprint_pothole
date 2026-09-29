"""Unmodified XT32 packet diagnostics and durable, rotating PCAP storage."""

from __future__ import annotations
import calendar
import json
import os
import queue
import socket
import struct
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

NS = 1_000_000_000

PACKET_HEADER_SIZE = 12

BLOCK_SIZE = 130

TAIL_SIZE = 28

FULL_PACKET_SIZE = 1080

_TAIL_MOTOR = struct.Struct("<H")

_TAIL_U32 = struct.Struct("<I")


def has_tail(payload: bytes) -> bool:
    """True when the payload still carries a 28-byte tail after whole blocks."""
    if len(payload) < PACKET_HEADER_SIZE + TAIL_SIZE:
        return False
    return (len(payload) - PACKET_HEADER_SIZE - TAIL_SIZE) % BLOCK_SIZE == 0


def block_count(payload: bytes) -> int:
    body = len(payload) - PACKET_HEADER_SIZE - (TAIL_SIZE if has_tail(payload) else 0)
    return max(0, body // BLOCK_SIZE)


_TAIL_FIELDS = struct.Struct("<10xBH6sIBI")


class SequenceTracker:
    """Count real packet loss using the sensor's own sequence counter.

    Kernel drop counters only see datagrams the kernel had to discard; a packet
    lost on the wire never reaches them.  The sensor's sequence number catches
    both.
    """

    def __init__(self):
        self.first = None
        self.last = None
        self.received = 0
        self.missing = 0
        self.gap_events = 0
        self.wrap_count = 0
        self.backward_count = 0

    def update(self, sequence: int | None):
        if sequence is None:
            return
        sequence = int(sequence)
        self.received += 1
        if self.last is None:
            self.first = sequence
            self.last = sequence
            return
        step = sequence - self.last
        if step == 1:
            self.last = sequence
            return
        if step > 1:
            self.missing += step - 1
            self.gap_events += 1
        elif step < 0:
            # u32 wrap is a step of about -2**32; anything else is reordering.
            if self.last > 0xF0000000 and sequence < 0x10000000:
                self.wrap_count += 1
            else:
                self.backward_count += 1
        self.last = sequence

    def summary(self) -> dict:
        expected = self.received + self.missing
        return {
            "first_sequence": self.first,
            "last_sequence": self.last,
            "received": self.received,
            "missing": self.missing,
            "gap_events": self.gap_events,
            "wrap_count": self.wrap_count,
            "backward_count": self.backward_count,
            "loss_ratio": (self.missing / expected) if expected > 0 else 0.0,
        }


class ClockOffsetTracker:
    """Track ``host - sensor`` so a run records its own clock relationship."""

    def __init__(self):
        self.count = 0
        self.min = None
        self.max = None
        self.total = 0.0
        self.first = None
        self.last = None
        self.first_host = None
        self.last_host = None

    def update(self, host_timestamp: float, sensor_ts: float | None):
        if sensor_ts is None:
            return
        delta = float(host_timestamp) - float(sensor_ts)
        self.count += 1
        self.total += delta
        if self.min is None or delta < self.min:
            self.min = delta
        if self.max is None or delta > self.max:
            self.max = delta
        if self.first is None:
            self.first = delta
            self.first_host = float(host_timestamp)
        self.last = delta
        self.last_host = float(host_timestamp)

    def summary(self) -> dict:
        if self.count == 0:
            return {"samples": 0}
        span = (
            (self.last_host - self.first_host)
            if self.first_host is not None and self.last_host is not None
            else 0.0
        )
        drift = (self.last - self.first) if self.first is not None else 0.0
        return {
            "samples": self.count,
            "host_minus_sensor_min": self.min,
            "host_minus_sensor_max": self.max,
            "host_minus_sensor_mean": self.total / self.count,
            "host_minus_sensor_first": self.first,
            "host_minus_sensor_last": self.last,
            "observed_span_sec": span,
            "drift_sec": drift,
            "drift_ppm": (drift / span * 1e6) if span > 1.0 else None,
        }


_CALENDAR_SECONDS: dict[bytes, int] = {}


def _calendar_seconds(utc: bytes) -> int:
    """UNIX seconds of the tail's six date bytes; ValueError if not a date.

    The conversion costs more than the rest of a packet's checks and its input
    changes once a second, so valid dates are memoised.
    """
    seconds = _CALENDAR_SECONDS.get(utc)
    if seconds is None:
        year, month, day, hour, minute, second = utc
        date = datetime(1900 + year, month, day, hour, minute, second, tzinfo=timezone.utc)
        seconds = calendar.timegm(date.utctimetuple())
        if len(_CALENDAR_SECONDS) > 4096:
            _CALENDAR_SECONDS.clear()
        _CALENDAR_SECONDS[utc] = seconds
    return seconds


def xt32_clock(payload: bytes) -> tuple[int, int, float]:
    """Hesai protocol 6.1: full 32-channel, 8-block datagram and native UTC tail.

    Returns (native ns, sequence, native seconds); the tail is unpacked once.
    """
    if len(payload) != 1080 or payload[:4] != b"\xee\xff\x06\x01" or payload[6:8] != b"\x20\x08":
        raise ValueError("Expected a complete PandarXT 32-channel packet")
    _mode, _rpm, utc, microseconds, _factory, sequence = _TAIL_FIELDS.unpack_from(payload, 1052)
    if microseconds >= 1_000_000:
        raise ValueError("Invalid LiDAR microseconds")
    seconds = _calendar_seconds(utc)
    return seconds * NS + microseconds * 1000, sequence, seconds + microseconds / 1e6


PCAP_MAGIC_USEC = 0xA1B2C3D4

PCAP_VERSION_MAJOR = 2

PCAP_VERSION_MINOR = 4

PCAP_SNAPLEN = 262144

LINKTYPE_ETHERNET = 1

PCAP_GLOBAL_HEADER = struct.Struct("<IHHiIII")

PCAP_RECORD_HEADER = struct.Struct("<IIII")

ETHERNET_HEADER_SIZE = 14

IPV4_HEADER_SIZE = 20

UDP_HEADER_SIZE = 8

LINK_HEADER_SIZE = ETHERNET_HEADER_SIZE + IPV4_HEADER_SIZE + UDP_HEADER_SIZE

DEFAULT_SRC_MAC = bytes.fromhex("02005e000001")

DEFAULT_DST_MAC = bytes.fromhex("02005e000002")

DEFAULT_SRC_IP = "192.168.1.201"

DEFAULT_SRC_PORT = 10000

DEFAULT_DST_IP = "192.168.1.100"

DEFAULT_DST_PORT = 2368


def _fsync_directory(path: Path) -> None:
    """Persist a rename on POSIX; Windows does not allow directory fsync."""
    if os.name == "nt":
        return
    descriptor = None
    try:
        descriptor = os.open(str(path), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        os.fsync(descriptor)
    except (OSError, AttributeError):
        pass
    finally:
        if descriptor is not None:
            os.close(descriptor)


def pcap_global_header() -> bytes:
    return PCAP_GLOBAL_HEADER.pack(
        PCAP_MAGIC_USEC,
        PCAP_VERSION_MAJOR,
        PCAP_VERSION_MINOR,
        0,
        0,
        PCAP_SNAPLEN,
        LINKTYPE_ETHERNET,
    )


def _ipv4_checksum(header: bytes) -> int:
    total = 0
    for offset in range(0, len(header), 2):
        total += (header[offset] << 8) | header[offset + 1]
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def _packed_ip(text: str) -> bytes:
    try:
        return socket.inet_aton(str(text))
    except OSError:
        return socket.inet_aton(DEFAULT_SRC_IP)


class LinkHeaderBuilder:
    """Build the 42 constant-ish bytes that precede each stored payload."""

    def __init__(
        self,
        src_ip: str = DEFAULT_SRC_IP,
        src_port: int = DEFAULT_SRC_PORT,
        dst_ip: str = DEFAULT_DST_IP,
        dst_port: int = DEFAULT_DST_PORT,
    ):
        self.src_mac = DEFAULT_SRC_MAC
        self.dst_mac = DEFAULT_DST_MAC
        self.set_peer(src_ip, src_port, dst_ip, dst_port)

    def set_peer(self, src_ip, src_port, dst_ip=None, dst_port=None):
        self.src_ip = str(src_ip)
        self.src_port = int(src_port) & 0xFFFF
        if dst_ip is not None:
            self.dst_ip = str(dst_ip)
        if dst_port is not None:
            self.dst_port = int(dst_port) & 0xFFFF
        self._src_packed = _packed_ip(self.src_ip)
        self._dst_packed = _packed_ip(self.dst_ip)
        self._ethernet = self.dst_mac + self.src_mac + b"\x08\x00"
        self._cache = {}

    def header_for(self, payload_length: int) -> bytes:
        cached = self._cache.get(payload_length)
        if cached is not None:
            return cached
        udp_length = UDP_HEADER_SIZE + payload_length
        ip_total = IPV4_HEADER_SIZE + udp_length
        ip_without_checksum = struct.pack(
            ">BBHHHBBH4s4s",
            0x45,
            0x00,
            ip_total,
            0x0000,
            0x4000,  # Don't Fragment; these datagrams are never fragmented.
            64,
            socket.IPPROTO_UDP,
            0,
            self._src_packed,
            self._dst_packed,
        )
        checksum = _ipv4_checksum(ip_without_checksum)
        ip_header = (
            ip_without_checksum[:10] + struct.pack(">H", checksum) + ip_without_checksum[12:]
        )
        # UDP checksum 0 means "not computed", which is legal over IPv4 and is
        # what every dissector expects from a synthesised capture.
        udp_header = struct.pack(">HHHH", self.src_port, self.dst_port, udp_length, 0)
        header = self._ethernet + ip_header + udp_header
        if len(self._cache) < 64:
            self._cache[payload_length] = header
        return header


def pcap_record(timestamp: float, payload: bytes, link_header: bytes) -> bytes:
    seconds = int(timestamp)
    microseconds = int(round((timestamp - seconds) * 1_000_000))
    if microseconds >= 1_000_000:  # rounding can carry into the next second
        seconds += 1
        microseconds -= 1_000_000
    elif microseconds < 0:
        seconds -= 1
        microseconds += 1_000_000
    length = len(link_header) + len(payload)
    return PCAP_RECORD_HEADER.pack(seconds, microseconds, length, length) + link_header + payload


class PcapLidarWriter:
    """Append packets to rotating pcap files without blocking the receiver.

    The receive thread only appends bytes to a small buffer.  Whole buffers are
    handed to a writer thread, which owns the open file.  Rotation is decided
    from packet timestamps, not wall time, so a file's name and its contents
    always agree.
    """

    def __init__(
        self,
        directory: Path,
        seconds_per_file: float = 10.0,
        flush_bytes: int = 1 << 20,
        gzip_output: bool = False,
        max_pending: int = 8,
        peer: LinkHeaderBuilder | None = None,
    ):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.directory / "pcaps.jsonl"
        self.seconds_per_file = max(0.5, float(seconds_per_file))
        self.flush_bytes = max(4096, int(flush_bytes))
        if gzip_output:
            raise ValueError("Only uncompressed PCAP is supported")
        self.gzip_output = False
        self.suffix = ".pcap"
        self.peer = peer or LinkHeaderBuilder()
        self.sync_bytes = max(self.flush_bytes, 4 << 20)
        self._unsynced = 0

        self.jobs = queue.Queue(maxsize=max(2, int(max_pending)))
        self.lock = threading.Lock()
        self.closed = False

        self.buffer = bytearray()
        self.sequence = self._next_sequence()
        self.file_first_timestamp = None
        self.file_last_timestamp = None
        self.file_packet_count = 0
        self.file_payload_bytes = 0

        self.file_count = 0
        self.packet_count = 0
        self.payload_bytes = 0
        self.disk_bytes = 0
        self.queue_high_watermark = 0
        self.queue_block_count = 0
        self.queue_block_seconds = 0.0
        self.writer_errors = []

        self.thread = threading.Thread(target=self._worker, name="lidar-pcap-writer", daemon=True)
        self.thread.start()

    def _next_sequence(self) -> int:
        sequences = []
        for path in self.directory.glob(f"*{self.suffix}"):
            stem = path.name[: -len(self.suffix)]
            try:
                sequences.append(int(stem))
            except ValueError:
                continue
        return max(sequences, default=-1) + 1

    def _raise_if_writer_failed(self):
        with self.lock:
            errors = list(self.writer_errors)
        if errors:
            raise RuntimeError(f"LiDAR pcap writer failed: {errors}")

    def set_peer(self, src_ip, src_port, dst_ip=None, dst_port=None):
        self.peer.set_peer(src_ip, src_port, dst_ip, dst_port)

    def write_packet(self, timestamp: float, payload: bytes):
        if self.writer_errors:
            self._raise_if_writer_failed()
        if self.closed:
            raise RuntimeError("pcap writer is already closed")
        if not payload:
            return
        timestamp = float(timestamp)
        if (
            self.file_first_timestamp is not None
            and timestamp - self.file_first_timestamp >= self.seconds_per_file
        ):
            self._seal_current()
        if self.file_first_timestamp is None:
            self.file_first_timestamp = timestamp

        self.buffer.extend(pcap_record(timestamp, payload, self.peer.header_for(len(payload))))
        self.file_last_timestamp = timestamp
        self.file_packet_count += 1
        self.file_payload_bytes += len(payload)
        if len(self.buffer) >= self.flush_bytes:
            self._submit({"kind": "data", "blob": bytes(self.buffer)})
            self.buffer = bytearray()

    def _submit(self, job):
        before = time.perf_counter()
        if self.jobs.full():
            with self.lock:
                self.queue_block_count += 1
        self.jobs.put(job)
        blocked = time.perf_counter() - before
        with self.lock:
            self.queue_block_seconds += blocked
            self.queue_high_watermark = max(self.queue_high_watermark, self.jobs.qsize())

    def _seal_current(self):
        if self.file_packet_count == 0:
            self.buffer = bytearray()
            return
        job = {
            "kind": "seal",
            "blob": bytes(self.buffer),
            "sequence": self.sequence,
            "first_timestamp": float(self.file_first_timestamp),
            "last_timestamp": float(self.file_last_timestamp),
            "packet_count": self.file_packet_count,
            "payload_bytes": self.file_payload_bytes,
        }
        self.buffer = bytearray()
        self.sequence += 1
        self.file_first_timestamp = None
        self.file_last_timestamp = None
        self.file_packet_count = 0
        self.file_payload_bytes = 0
        self._submit(job)

    def rotate(self):
        """Close the current file so a run/clock boundary starts a new one."""
        self._raise_if_writer_failed()
        if self.closed:
            raise RuntimeError("pcap writer is already closed")
        self._seal_current()

    # ------------------------------------------------------------------
    # writer thread
    # ------------------------------------------------------------------

    def _worker(self):
        handle = None
        temp_path = None
        try:
            while True:
                job = self.jobs.get()
                try:
                    if job is None:
                        return
                    handle, temp_path = self._apply(job, handle, temp_path)
                except Exception as exc:
                    with self.lock:
                        self.writer_errors.append(repr(exc))
                finally:
                    self.jobs.task_done()
        finally:
            if handle is not None:
                try:
                    handle.close()
                except Exception:
                    pass

    def _open_file(self, sequence: int):
        filename = f"{sequence:08d}{self.suffix}"
        temp_path = self.directory / f"{filename}.tmp"
        handle = temp_path.open("wb")
        handle.write(pcap_global_header())
        return handle, temp_path, filename

    def _sync(self, handle, full: bool):
        """Push written bytes to the device.

        A single fsync at seal time would flush a whole file at once - tens of
        megabytes - and while that syscall runs this thread stops draining the
        queue, which backs up into the receive loop and shows up as kernel
        packet drops.  Syncing incrementally keeps each pause small and leaves
        almost nothing for the final one.
        """
        try:
            handle.flush()
        except (OSError, ValueError):
            return
        raw = getattr(handle, "fileobj", None) or getattr(handle, "myfileobj", None)
        try:
            fileno = raw.fileno() if raw is not None else handle.fileno()
        except (OSError, ValueError, AttributeError):
            return
        try:
            if full:
                os.fsync(fileno)
            else:
                os.fdatasync(fileno)
        except (OSError, ValueError, AttributeError):
            pass

    def _apply(self, job, handle, temp_path):
        if handle is None:
            handle, temp_path, _ = self._open_file(job.get("sequence", self.sequence))
            self._unsynced = 0
        if job["blob"]:
            handle.write(job["blob"])
            self._unsynced += len(job["blob"])
        if job["kind"] != "seal":
            if self._unsynced >= self.sync_bytes:
                self._sync(handle, full=False)
                self._unsynced = 0
            return handle, temp_path

        self._sync(handle, full=True)
        self._unsynced = 0
        handle.close()
        filename = f"{job['sequence']:08d}{self.suffix}"
        final_path = self.directory / filename
        os.replace(temp_path, final_path)
        _fsync_directory(self.directory)
        disk_bytes = final_path.stat().st_size

        record = {
            "sequence": job["sequence"],
            "path": filename,
            "first_timestamp": job["first_timestamp"],
            "last_timestamp": job["last_timestamp"],
            "packet_count": job["packet_count"],
            "payload_bytes": job["payload_bytes"],
            "disk_bytes": disk_bytes,
            "link_type": "ethernet",
            "codec": "pcap",
            "src": f"{self.peer.src_ip}:{self.peer.src_port}",
            "dst": f"{self.peer.dst_ip}:{self.peer.dst_port}",
        }
        with self.manifest_path.open("a", encoding="utf-8") as manifest:
            manifest.write(json.dumps(record, ensure_ascii=False) + "\n")
            manifest.flush()
            os.fsync(manifest.fileno())

        with self.lock:
            self.file_count += 1
            self.packet_count += job["packet_count"]
            self.payload_bytes += job["payload_bytes"]
            self.disk_bytes += disk_bytes
        return None, None

    def close(self):
        if self.closed:
            return
        self.closed = True
        self._seal_current()
        self.jobs.join()
        self.jobs.put(None)
        self.thread.join(timeout=15.0)
        if self.thread.is_alive():
            raise RuntimeError("LiDAR pcap writer did not stop")
        if self.writer_errors:
            raise RuntimeError(f"LiDAR pcap writer errors: {self.writer_errors}")

    def stats(self):
        with self.lock:
            return {
                "storage_format": "pcap",
                "pcap_file_count": self.file_count,
                "pcap_packet_count": self.packet_count,
                "pcap_payload_bytes": self.payload_bytes,
                "pcap_disk_bytes": self.disk_bytes,
                "pcap_disk_ratio": (
                    self.disk_bytes / self.payload_bytes if self.payload_bytes > 0 else 0.0
                ),
                "pcap_queue_high_watermark": self.queue_high_watermark,
                "pcap_queue_block_count": self.queue_block_count,
                "pcap_queue_block_seconds": self.queue_block_seconds,
                "pcap_writer_errors": list(self.writer_errors),
            }
