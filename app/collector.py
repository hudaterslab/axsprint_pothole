"""PTP-only camera, LiDAR and GPS collection with durable storage and run rotation.

PCAP files are saved directly under each run/lidar directory.
"""

import json
import os
import queue
import select
import signal
import shutil
import socket
import struct
import subprocess
import threading
import time
from glob import glob
from collections import deque
from contextlib import ExitStack
from concurrent.futures import ThreadPoolExecutor, wait
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
os.environ.setdefault("OPENCV_FFMPEG_DEBUG", "0")
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;3000000")

import cv2
import yaml

import lidar as xt32_packet
from camera import CameraCapture
from ptp import GUARD, NS, native_to_utc
from lidar import LinkHeaderBuilder, PcapLidarWriter, xt32_timestamp

CONFIG_PATH = os.environ.get(
    "CONFIG_PATH", str(Path(__file__).resolve().parents[1] / "config.yaml")
)


def load_config(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    if config is None:
        raise ValueError(f"Config file is empty: {path}")

    return config


def config_bool(key: str, default: bool) -> bool:
    value = config.get(key, default)
    if value is None or value == "":
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


config = load_config(CONFIG_PATH)

# This runtime deliberately supports only the deployed PTP/PCAP pipeline.
for key, expected in {
    "camera_read_mode": "gstreamer_vaapi",
    "camera_manifest_timestamp": "capture",
    "lidar_save_mode": "continuous",
    "lidar_storage_format": "pcap",
    "save_raw_video_copy": False,
    "camera_decode_enabled": True,
    "show_window": False,
    "drop_bad_frames": False,
    "lidar_pcap_gzip": False,
    "lidar_camera_offset_sec": 0.0,
}.items():
    if config.get(key, expected) != expected:
        raise ValueError(f"{key} must be {expected!r} for PTP collection")


def resolve_save_dir(
    settings: Dict[str, Any],
    config_path: str | Path = CONFIG_PATH,
) -> Path:
    """Resolve save_dir consistently relative to the configuration file."""
    # The foreground supervisor resolves the one safe external USB drive at
    # startup.  Its environment override takes precedence over the static
    # config path; direct/manual runs keep using config.yaml unchanged.
    raw_value = os.environ.get("PORTHOLE_SAVE_DIR")
    if raw_value is None:
        raw_value = settings.get("save_dir", settings.get("runs_dir", "runs"))
    value = str(raw_value or "").strip()
    if not value:
        raise ValueError("save_dir must not be empty")
    path = Path(os.path.expandvars(value)).expanduser()
    if not path.is_absolute():
        path = Path(config_path).expanduser().resolve().parent / path
    return path.resolve()


RTSP_PROFILE = str(config.get("rtsp_profile", "main")).strip().lower()
if RTSP_PROFILE in {"low", "sub", "substream"}:
    RTSP_URL = (
        config.get("rtsp_url_low") or config.get("rtsp_substream_url") or config.get("rtsp_url")
    )
else:
    RTSP_URL = config.get("rtsp_url_main") or config.get("rtsp_url")
CAMERA_READ_MODE = str(config.get("camera_read_mode", "gstreamer_vaapi")).lower()
CAMERA_LATEST_FRAME_ENABLED = config_bool("camera_latest_frame_enabled", True)
CAMERA_FRAME_BUFFER_SIZE = max(1, int(config.get("camera_frame_buffer_size", 8)))
CAMERA_WATCHDOG_SEC = float(config.get("camera_watchdog_sec", 30.0))
CAMERA_CONNECTION_TIMEOUT_SEC = float(config.get("camera_connection_timeout_sec", 1.0))
CAMERA_RECONNECT_INITIAL_SEC = float(config.get("camera_reconnect_initial_sec", 1.0))
CAMERA_RECONNECT_MAX_SEC = float(config.get("camera_reconnect_max_sec", 30.0))
JPEG_QUALITY = int(config.get("jpeg_quality", 80))
FRAME_SAVE_FPS = float(config.get("frame_save_fps", 5.0))
CAMERA_ASYNC_WRITE = config_bool("camera_async_write", True)
CAMERA_WRITE_QUEUE_SIZE = int(config.get("camera_write_queue_size", 128))
SHOW_WINDOW = bool(config.get("show_window", False))
CAMERA_DECODE_ENABLED = bool(config.get("camera_decode_enabled", True))
SAVE_RAW_VIDEO_COPY = bool(config.get("save_raw_video_copy", config.get("save_raw_video", False)))
BBOX_FLIP_X = config_bool("bbox_flip_x", False)
DISPLAY_FLIP_X = config_bool("display_flip_x", False)
BBOX_SOURCE_SIZE = str(config.get("bbox_source_size", "") or "")
BBOX_LETTERBOX_SIZE = str(config.get("bbox_letterbox_size", "") or "")

LIDAR_ENABLED = bool(config.get("lidar_enabled", True))
LIDAR_BIND_IP = config.get("lidar_bind_ip", "0.0.0.0")
LIDAR_UDP_PORT = int(config.get("lidar_udp_port", 2368))
LIDAR_LOG_INTERVAL_SEC = float(config.get("lidar_log_interval_sec", 5))
LIDAR_CONNECTION_TIMEOUT_SEC = float(config.get("lidar_connection_timeout_sec", 0.5))
LIDAR_RECONNECT_INITIAL_SEC = float(config.get("lidar_reconnect_initial_sec", 1.0))
LIDAR_RECONNECT_MAX_SEC = float(config.get("lidar_reconnect_max_sec", 30.0))
LIDAR_SOCKET_RCVBUF_BYTES = int(config.get("lidar_socket_rcvbuf_bytes", 4 * 1024 * 1024))
LIDAR_FOV_FILTER_ENABLED = bool(config.get("lidar_fov_filter_enabled", False))
LIDAR_CAMERA_FORWARD_OFFSET_DEG = float(config.get("camera_forward_offset_deg", 0.0))
LIDAR_H_MARGIN_DEG = float(config.get("lidar_h_margin_deg", 55.0))
LIDAR_SAVE_MODE = str(config.get("lidar_save_mode", "continuous")).lower()
LIDAR_CAMERA_OFFSET_SEC = float(config.get("lidar_camera_offset_sec", 0.0))
LIDAR_STORAGE_FORMAT = str(config.get("lidar_storage_format", "pcap")).lower()
LIDAR_PCAP_SECONDS = float(config.get("lidar_pcap_seconds", 10.0))
LIDAR_PCAP_GZIP = config_bool("lidar_pcap_gzip", False)
LIDAR_PCAP_QUEUE_SIZE = int(config.get("lidar_pcap_queue_size", 8))
# SO_TIMESTAMPNS puts the kernel's receive time on each datagram, which removes
# the Python scheduling delay between the packet arriving and recvfrom returning.
LIDAR_KERNEL_TIMESTAMP = config_bool("lidar_kernel_timestamp", True)

GPS_ENABLED = config_bool("gps_enabled", False)
GPS_DEVICE = str(config.get("gps_device", "/dev/ttyUSB0")).strip()
GPS_PREFERRED_DEVICE = str(config.get("gps_preferred_device", "")).strip()
GPS_BAUDRATE = int(config.get("gps_baudrate", 115200))
GPS_CONNECTION_TIMEOUT_SEC = float(config.get("gps_connection_timeout_sec", 3.0))
GPS_RECONNECT_INITIAL_SEC = float(config.get("gps_reconnect_initial_sec", 1.0))
GPS_RECONNECT_MAX_SEC = float(config.get("gps_reconnect_max_sec", 30.0))

# --- camera capture clock ------------------------------------------------
# The RTSP presentation timestamp is the camera's own sampling clock.  Host
# arrival time is that plus a delay which varies by hundreds of milliseconds,
# so PTS is the only field that preserves the spacing between frames.
CAMERA_PTS_ENABLED = config_bool("camera_pts_enabled", True)
# How fast the offset floor may rise. Covers camera-vs-host crystal drift
# (measured 0.23 ms/h) without letting sampling luck move the estimate.
CAMERA_DURABLE_WRITES = config_bool("camera_durable_writes", True)
# "capture" puts the PTS-derived time in the manifest's `timestamp`; "arrival"
# keeps the old host-arrival meaning.  Both values are always recorded.
CAMERA_MANIFEST_TIMESTAMP = str(config.get("camera_manifest_timestamp", "capture")).strip().lower()

# 0 disables rotation and keeps one folder per power-on (the old behaviour).
RUN_ROTATION_MINUTES = float(config.get("run_rotation_minutes", 60.0))

# How early the next folder is created and handed to the writers.  It only has
# to beat the main loop's own lateness, which is milliseconds.
ROTATION_ARM_LEAD_SEC = float(config.get("run_rotation_arm_lead_sec", 5.0))
# How long to wait for a frame to cross the boundary before handing the
# camera side over anyway.  Measured lag is about 100 ms; this only matters
# when the camera is not delivering frames at all.
CAMERA_HANDOVER_GRACE_SEC = float(config.get("camera_handover_grace_sec", 5.0))

# Startup write-speed probe; 0 disables it.
STORAGE_PREFLIGHT_MIB = int(config.get("storage_preflight_mib", 4))
STORAGE_MIN_WRITE_MIBS = float(config.get("storage_min_write_mibs", 15.0))

RECORD_ENABLED = bool(config.get("record_enabled", True))
SAVE_DIR = resolve_save_dir(config)
# 이전 도구가 참조하는 이름을 깨지 않기 위한 호환 alias.
RUNS_DIR = SAVE_DIR
SAVE_FRAMES = bool(config.get("save_frames", True))
SAVE_LIDAR_RAW = bool(config.get("save_lidar_raw", True))

MIN_FREE_GB = float(config.get("min_free_gb", 5.0))
DISK_CHECK_INTERVAL_SEC = float(config.get("disk_check_interval_sec", 30.0))
HEALTH_LOG_INTERVAL_SEC = float(config.get("health_log_interval_sec", 60.0))
HEALTH_WARN_CPU_TEMP_C = float(config.get("health_warn_cpu_temp_c", 80.0))
CAMERA_WARMUP_SEC = float(config.get("camera_warmup_sec", 1.5))
DROP_BAD_FRAMES = bool(config.get("drop_bad_frames", True))

if not RTSP_URL:
    raise ValueError("config.yaml is missing rtsp_url")

if FRAME_SAVE_FPS <= 0:
    raise ValueError("frame_save_fps must be greater than 0")
if CAMERA_WRITE_QUEUE_SIZE <= 0:
    raise ValueError("camera_write_queue_size must be greater than 0")
if MIN_FREE_GB < 0 or DISK_CHECK_INTERVAL_SEC <= 0:
    raise ValueError("disk free-space settings are invalid")
if HEALTH_LOG_INTERVAL_SEC <= 0:
    raise ValueError("health_log_interval_sec must be greater than 0")

if (
    CAMERA_CONNECTION_TIMEOUT_SEC <= 0
    or LIDAR_CONNECTION_TIMEOUT_SEC <= 0
    or GPS_CONNECTION_TIMEOUT_SEC <= 0
):
    raise ValueError("camera/lidar/GPS connection timeouts must be greater than 0")
if CAMERA_WATCHDOG_SEC <= CAMERA_CONNECTION_TIMEOUT_SEC:
    raise ValueError("camera_watchdog_sec must be greater than camera_connection_timeout_sec")
if (
    CAMERA_RECONNECT_INITIAL_SEC <= 0
    or CAMERA_RECONNECT_MAX_SEC < CAMERA_RECONNECT_INITIAL_SEC
    or LIDAR_RECONNECT_INITIAL_SEC <= 0
    or LIDAR_RECONNECT_MAX_SEC < LIDAR_RECONNECT_INITIAL_SEC
    or GPS_RECONNECT_INITIAL_SEC <= 0
    or GPS_RECONNECT_MAX_SEC < GPS_RECONNECT_INITIAL_SEC
):
    raise ValueError("sensor reconnect delays are invalid")
if GPS_ENABLED and (not GPS_DEVICE or GPS_BAUDRATE <= 0):
    raise ValueError("gps_device/gps_baudrate settings are invalid")


if LIDAR_PCAP_SECONDS <= 0:
    raise ValueError("lidar_pcap_seconds must be greater than 0")
if RUN_ROTATION_MINUTES < 0:
    raise ValueError("run_rotation_minutes must be 0 (off) or positive")
if RUN_ROTATION_MINUTES > 0 and 1440 % RUN_ROTATION_MINUTES != 0:
    # Boundaries are aligned to the calendar day so folder names stay
    # predictable; a period that does not divide a day would drift.
    raise ValueError("run_rotation_minutes must divide 1440 evenly (e.g. 1, 5, 15, 30, 60)")


def rotation_period_sec() -> float:
    return RUN_ROTATION_MINUTES * 60.0


def run_id_for(timestamp: float) -> str:
    """Folder name for the rotation slot a timestamp belongs to.

    Rotation off keeps the historical power-on name.  Hourly rotation names the
    folder after the hour it covers, so 20260818_15 always means 15:00-16:00
    local time no matter when the collector started.
    """
    moment = datetime.fromtimestamp(timestamp)
    if RUN_ROTATION_MINUTES <= 0:
        return moment.strftime("%Y%m%d_%H%M%S")
    if RUN_ROTATION_MINUTES >= 60:
        return moment.strftime("%Y%m%d_%H")
    slot = (moment.minute // int(RUN_ROTATION_MINUTES)) * int(RUN_ROTATION_MINUTES)
    return f"{moment:%Y%m%d_%H}{slot:02d}"


def next_rotation_time(timestamp: float) -> float | None:
    """Wall-clock instant when the current folder stops accepting data."""
    if RUN_ROTATION_MINUTES <= 0:
        return None
    moment = datetime.fromtimestamp(timestamp)
    midnight = moment.replace(hour=0, minute=0, second=0, microsecond=0)
    period = rotation_period_sec()
    elapsed = timestamp - midnight.timestamp()
    return midnight.timestamp() + (int(elapsed // period) + 1) * period


def acquire_collector_lock():
    """Hold a host-wide lock so two collectors cannot share sensors/run files."""
    try:
        import fcntl
    except ImportError as error:
        raise RuntimeError(
            "collect_data.py single-instance locking requires Linux fcntl"
        ) from error

    lock_path = Path(
        os.environ.get(
            "PORTHOLE_COLLECTOR_LOCK",
            "/tmp/porthole_collect_data.lock",
        )
    )
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    handle = lock_path.open("a+", encoding="ascii")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        handle.seek(0)
        owner = handle.read().strip() or "unknown"
        handle.close()
        raise RuntimeError(
            "another collect_data.py instance is already running "
            f"(lock={lock_path}, owner_pid={owner})"
        ) from error
    handle.seek(0)
    handle.truncate()
    handle.write(f"{os.getpid()}\n")
    handle.flush()
    return handle


def measure_storage_write_mibs(path: Path, sample_mib: int) -> float | None:
    """Time a short direct write so a degraded disk is caught before a drive.

    This exists because a USB SSD was once found enumerated at 12 Mbit/s
    instead of 480 - the device was healthy, the link was not - and it wrote at
    0.5 MB/s.  Nothing in the collector noticed; it simply dropped most of the
    LiDAR packets.  The driving terminal is hard to reach, so the cheapest way
    to catch a repeat is to measure once at startup and say so out loud.
    """
    if sample_mib <= 0:
        return None
    probe = Path(path) / ".write_speed_probe"
    block = b"\0" * (1024 * 1024)
    try:
        # Buffered write plus fsync, not O_DIRECT: O_DIRECT needs page-aligned
        # buffers that os.write cannot supply, and fsync already forces the
        # bytes past the page cache and onto the device, which is what we are
        # timing.
        fd = os.open(str(probe), os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
        try:
            started = time.perf_counter()
            for _ in range(int(sample_mib)):
                os.write(fd, block)
            os.fsync(fd)
            elapsed = time.perf_counter() - started
        finally:
            os.close(fd)
        return sample_mib / elapsed if elapsed > 0 else None
    except OSError as exc:
        # A USB disk that re-enumerated under a new device name leaves its old
        # mount point in place but unwritable, and every read looks fine until
        # something actually touches the device.  Say so now, not after a drive.
        print(
            f"[STORAGE ERROR] cannot write to {path}: {exc}\n"
            "               If this is a USB disk, it may have re-enumerated "
            "under a new device name,\n"
            "               leaving a stale mount. Check: findmnt "
            f"{path}  and  lsblk",
            flush=True,
        )
        return None
    finally:
        try:
            probe.unlink()
        except OSError:
            pass


def check_free_space_gb(path: Path) -> float:
    try:
        path.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        raise RuntimeError(f"Cannot create save_dir {path}: {error}") from error
    if not path.is_dir():
        raise RuntimeError(f"save_dir is not a directory: {path}")
    if not os.access(path, os.W_OK | os.X_OK):
        raise RuntimeError(f"save_dir is not writable: {path}")
    usage = shutil.disk_usage(path)
    return usage.free / (1024**3)


def read_process_rss_mib() -> float | None:
    try:
        with open("/proc/self/status", "r", encoding="ascii") as handle:
            for line in handle:
                if line.startswith("VmRSS:"):
                    return float(line.split()[1]) / 1024.0
    except (OSError, ValueError, IndexError):
        return None
    return None


def read_cpu_temperature_c() -> float | None:
    temperatures = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            value = float(path.read_text(encoding="ascii").strip())
        except (OSError, ValueError):
            continue
        if value > 1000.0:
            value /= 1000.0
        if -20.0 <= value <= 150.0:
            temperatures.append(value)
    return max(temperatures) if temperatures else None


def read_cpu_throttled_status() -> str | None:
    """Return Raspberry Pi/CM throttling flags when vcgencmd is available."""
    executable = shutil.which("vcgencmd")
    if not executable:
        return None
    try:
        result = subprocess.run(
            [executable, "get_throttled"],
            check=False,
            capture_output=True,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    output = (result.stdout or "").strip()
    return output or None


class SystemMetricsSampler:
    """Collect potentially slow OS/firmware metrics away from the capture loop."""

    def __init__(self, interval_sec: float):
        self.interval_sec = max(5.0, float(interval_sec))
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.values = {}
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                load_average = list(os.getloadavg())
            except (OSError, AttributeError):
                load_average = None
            sample = {
                "sampled_timestamp": time.time(),
                "process_rss_mib": read_process_rss_mib(),
                "load_average": load_average,
                "cpu_temperature_c": read_cpu_temperature_c(),
                "cpu_throttled_status": read_cpu_throttled_status(),
            }
            with self.lock:
                self.values = sample
            self.stop_event.wait(self.interval_sec)

    def snapshot(self) -> dict:
        with self.lock:
            return dict(self.values)

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=3.0)


def fsync_directory(path: Path) -> None:
    """Persist a rename/directory entry when the platform supports it."""
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


def append_jsonl(path: Path, record: Dict[str, Any], *, durable: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(record, ensure_ascii=False) + "\n")
        if durable:
            file.flush()
            os.fsync(file.fileno())


def _nmea_float(value: str):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _nmea_coordinate(value: str, hemisphere: str):
    """Convert NMEA ddmm.mmmm/dddmm.mmmm coordinates to signed degrees."""
    numeric = _nmea_float(value)
    if numeric is None:
        return None
    degrees = int(numeric // 100)
    minutes = numeric - degrees * 100
    if minutes < 0 or minutes >= 60:
        return None
    result = degrees + minutes / 60.0
    direction = str(hemisphere or "").upper()
    if direction in {"S", "W"}:
        result = -result
    elif direction not in {"N", "E"}:
        return None
    return result


def parse_nmea_sentence(raw_line: str, received_timestamp: float) -> dict:
    """Parse one NMEA sentence without inventing a location when fix is absent."""
    raw = str(raw_line).strip()
    base = {
        "timestamp": float(received_timestamp),
        "raw": raw,
        "checksum_valid": False,
        "sentence_type": None,
        "talker": None,
        "valid": False,
        "latitude_deg": None,
        "longitude_deg": None,
    }
    if not raw.startswith("$") or "*" not in raw:
        base["parse_error"] = "missing_nmea_envelope"
        return base
    body, checksum_text = raw[1:].rsplit("*", 1)
    if len(checksum_text) < 2:
        base["parse_error"] = "missing_checksum"
        return base
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    try:
        expected = int(checksum_text[:2], 16)
    except ValueError:
        base["parse_error"] = "invalid_checksum_text"
        return base
    base["checksum_valid"] = checksum == expected
    fields = body.split(",")
    message_id = fields[0] if fields else ""
    if len(message_id) >= 5:
        base["talker"] = message_id[:2]
        base["sentence_type"] = message_id[-3:]
    if not base["checksum_valid"]:
        base["parse_error"] = "checksum_mismatch"
        return base

    sentence_type = base["sentence_type"]
    try:
        if sentence_type == "GGA":
            latitude = _nmea_coordinate(fields[2], fields[3])
            longitude = _nmea_coordinate(fields[4], fields[5])
            fix_quality = int(fields[6] or 0)
            base.update(
                {
                    "gps_utc_time": fields[1] or None,
                    "latitude_deg": latitude,
                    "longitude_deg": longitude,
                    "fix_quality": fix_quality,
                    "satellites": int(fields[7] or 0),
                    "hdop": _nmea_float(fields[8]),
                    "altitude_m": _nmea_float(fields[9]),
                    "valid": bool(
                        fix_quality > 0 and latitude is not None and longitude is not None
                    ),
                }
            )
        elif sentence_type == "RMC":
            latitude = _nmea_coordinate(fields[3], fields[4])
            longitude = _nmea_coordinate(fields[5], fields[6])
            status = fields[2].upper() if len(fields) > 2 else "V"
            speed_knots = _nmea_float(fields[7])
            base.update(
                {
                    "gps_utc_time": fields[1] or None,
                    "gps_status": status,
                    "latitude_deg": latitude,
                    "longitude_deg": longitude,
                    "speed_knots": speed_knots,
                    "speed_mps": (None if speed_knots is None else speed_knots * 0.514444),
                    "course_deg": _nmea_float(fields[8]),
                    "gps_date_ddmmyy": fields[9] or None,
                    "valid": bool(status == "A" and latitude is not None and longitude is not None),
                }
            )
        elif sentence_type == "GLL":
            latitude = _nmea_coordinate(fields[1], fields[2])
            longitude = _nmea_coordinate(fields[3], fields[4])
            status = fields[6].upper() if len(fields) > 6 else "V"
            base.update(
                {
                    "gps_utc_time": fields[5] or None,
                    "gps_status": status,
                    "latitude_deg": latitude,
                    "longitude_deg": longitude,
                    "valid": bool(status == "A" and latitude is not None and longitude is not None),
                }
            )
        elif sentence_type == "VTG":
            base.update(
                {
                    "course_deg": _nmea_float(fields[1]),
                    "speed_knots": _nmea_float(fields[5]),
                    "speed_kmh": _nmea_float(fields[7]),
                }
            )
        elif sentence_type == "GSA":
            base.update(
                {
                    "fix_type": int(fields[2] or 1),
                    "pdop": _nmea_float(fields[15]) if len(fields) > 15 else None,
                    "hdop": _nmea_float(fields[16]) if len(fields) > 16 else None,
                    "vdop": _nmea_float(fields[17]) if len(fields) > 17 else None,
                }
            )
    except (IndexError, ValueError) as exc:
        base["parse_error"] = f"malformed_{sentence_type or 'unknown'}:{exc}"
        base["valid"] = False
    return base


def discover_gps_serial_devices(
    device_setting: str = "auto",
    preferred_device: str = "",
) -> list[str]:
    """Return distinct GPS serial candidates, preferring stable by-id paths.

    ``auto`` first selects the configured stable preferred device when it is
    present.  Otherwise it intentionally refuses to guess when more than one
    USB serial device exists.  The recorder retries discovery, so unplug/replug
    and ttyUSB number changes do not require restarting camera/LiDAR collection.
    """
    setting = str(device_setting or "auto").strip()
    if setting.lower() != "auto":
        matches = sorted(glob(setting)) if any(ch in setting for ch in "*?[") else [setting]
        return [str(Path(path)) for path in matches]

    candidates = []
    seen_targets = set()
    for pattern in (
        "/dev/serial/by-id/*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ):
        for path in sorted(glob(pattern)):
            try:
                target = os.path.realpath(path)
            except OSError:
                continue
            if not target or target in seen_targets:
                continue
            seen_targets.add(target)
            candidates.append(str(Path(path)))

    preferred = str(preferred_device or "").strip()
    if preferred:
        preferred_paths = (
            sorted(glob(preferred)) if any(ch in preferred for ch in "*?[") else [preferred]
        )
        preferred_targets = {
            os.path.realpath(path) for path in preferred_paths if os.path.exists(path)
        }
        preferred_candidates = [
            path for path in candidates if os.path.realpath(path) in preferred_targets
        ]
        if len(preferred_candidates) == 1:
            return preferred_candidates
    return candidates


class GpsNmeaRecorder:
    """Read USB NMEA asynchronously so GPS latency cannot block camera/LiDAR."""

    def __init__(
        self,
        device: str,
        baudrate: int,
        recorder: "RunRecorder | None",
        preferred_device: str = "",
    ):
        self.device = str(device)
        self.preferred_device = str(preferred_device)
        self.active_device = None
        self.candidate_devices = []
        self.baudrate = int(baudrate)
        self.recorder = recorder
        self.stop_event = threading.Event()
        self.thread = None
        self.lock = threading.Lock()
        self.fd = None
        self.sentence_count = 0
        self.checksum_error_count = 0
        self.parse_error_count = 0
        self.valid_fix_count = 0
        self.receiver_open_count = 0
        self.last_sentence_timestamp = None
        self.last_valid_fix_timestamp = None
        self.latest_valid_fix = None
        # A bounded history lets a consumer pick the fix that existed at a
        # given moment rather than the newest one.
        self.valid_fix_history = deque(maxlen=2048)
        self.transport_errors = []
        self.storage_errors = []
        self.storage_disabled = False
        self.pending_recorder = None
        self.pending_boundary = None

    def start(self):
        self.thread = threading.Thread(
            target=self._run,
            name="gps-nmea-reader",
            daemon=True,
        )
        self.thread.start()

    def rebind_recorder(self, recorder: "RunRecorder", boundary: float | None = None):
        """Ask the reader thread to reopen its log in a new run folder.

        With a boundary, the switch waits until a sentence's own receive time
        crosses it, so a sentence received just before the hour is not filed
        under the next one.  The switch happens in the reader thread between
        sentences, so no partially written line is split across two files.
        """
        with self.lock:
            self.pending_recorder = recorder
            self.pending_boundary = None if boundary is None else float(boundary)

    def _configure_serial(self, fd: int):
        try:
            import termios
        except ImportError as exc:
            raise RuntimeError("GPS serial input requires Linux termios") from exc
        speed_name = f"B{self.baudrate}"
        speed = getattr(termios, speed_name, None)
        if speed is None:
            raise ValueError(f"unsupported GPS baudrate: {self.baudrate}")
        attrs = termios.tcgetattr(fd)
        attrs[0] = termios.IGNPAR
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 5
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)

    def _open(self):
        candidates = discover_gps_serial_devices(
            self.device,
            self.preferred_device,
        )
        with self.lock:
            self.candidate_devices = list(candidates)
        if not candidates:
            raise FileNotFoundError(f"no GPS serial device matches {self.device!r}")
        if len(candidates) != 1:
            raise RuntimeError(
                "ambiguous GPS serial devices; connect exactly one or set "
                f"gps_device explicitly: {candidates}"
            )
        active_device = candidates[0]
        fd = os.open(
            active_device,
            os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK,
        )
        try:
            self._configure_serial(fd)
        except Exception:
            os.close(fd)
            raise
        self.fd = fd
        with self.lock:
            self.active_device = active_device
        print(f"[GPS] serial opened: {active_device}", flush=True)

    def _close(self):
        fd, self.fd = self.fd, None
        with self.lock:
            self.active_device = None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _record(self, record: dict, handle):
        if handle is None or self.storage_disabled:
            return handle
        try:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            return handle
        except OSError as exc:
            with self.lock:
                self.storage_errors.append(f"{type(exc).__name__}: {exc}")
                self.storage_disabled = True
            print(
                f"[GPS WARN] GPS logging disabled after storage error: {exc}",
                flush=True,
            )
            try:
                handle.close()
            except OSError:
                pass
            return None

    def _switch_log(self, handle, recorder):
        self.recorder = recorder
        if handle is not None:
            try:
                handle.flush()
                os.fsync(handle.fileno())
            except OSError:
                pass
            finally:
                try:
                    handle.close()
                except OSError:
                    pass
        try:
            new_handle = recorder.gps_jsonl.open("a", encoding="utf-8", buffering=1)
        except OSError as exc:
            with self.lock:
                self.storage_errors.append(f"{type(exc).__name__}: {exc}")
                self.storage_disabled = True
            print(f"[GPS WARN] cannot open GPS log after rotation: {exc}", flush=True)
            return None
        # A rotation gives a fresh file, so an earlier write failure on the old
        # one must not keep logging switched off.
        with self.lock:
            self.storage_disabled = False
        return new_handle

    def _consume_line(self, raw: bytes, handle):
        received_timestamp = time.time()
        with self.lock:
            pending = self.pending_recorder
            boundary = self.pending_boundary
        if pending is not None and (boundary is None or received_timestamp >= boundary):
            with self.lock:
                self.pending_recorder = None
                self.pending_boundary = None
            handle = self._switch_log(handle, pending)
        text = raw.decode("ascii", errors="replace").strip()
        if not text:
            return handle
        record = parse_nmea_sentence(text, received_timestamp)
        record["monotonic_ns"] = time.monotonic_ns()
        with self.lock:
            self.sentence_count += 1
            self.last_sentence_timestamp = received_timestamp
            if not record.get("checksum_valid"):
                self.checksum_error_count += 1
            if record.get("parse_error"):
                self.parse_error_count += 1
            if record.get("valid"):
                self.valid_fix_count += 1
                self.last_valid_fix_timestamp = received_timestamp
                self.latest_valid_fix = dict(record)
                self.valid_fix_history.append(dict(record))
        return self._record(record, handle)

    def _run(self):
        handle = None
        if self.recorder is not None:
            try:
                handle = self.recorder.gps_jsonl.open(
                    "a",
                    encoding="utf-8",
                    buffering=1,
                )
            except OSError as exc:
                with self.lock:
                    self.storage_errors.append(f"{type(exc).__name__}: {exc}")
                    self.storage_disabled = True
                print(f"[GPS WARN] cannot open GPS log: {exc}", flush=True)

        buffer = bytearray()
        reconnect_attempt = 0
        try:
            while not self.stop_event.is_set():
                if self.fd is None:
                    try:
                        self._open()
                        reconnect_attempt = 0
                        with self.lock:
                            self.receiver_open_count += 1
                    except (OSError, ValueError, RuntimeError) as exc:
                        reconnect_attempt += 1
                        delay = min(
                            GPS_RECONNECT_MAX_SEC,
                            GPS_RECONNECT_INITIAL_SEC * (2 ** min(reconnect_attempt - 1, 10)),
                        )
                        with self.lock:
                            self.transport_errors.append(f"{type(exc).__name__}: {exc}")
                            if len(self.transport_errors) > 20:
                                self.transport_errors = self.transport_errors[-20:]
                        self.stop_event.wait(delay)
                        continue

                try:
                    readable, _, _ = select.select([self.fd], [], [], 0.5)
                    if not readable:
                        continue
                    chunk = os.read(self.fd, 4096)
                    if not chunk:
                        raise OSError("GPS serial device returned EOF")
                    buffer.extend(chunk)
                    while b"\n" in buffer:
                        raw, _, remainder = buffer.partition(b"\n")
                        buffer = bytearray(remainder)
                        handle = self._consume_line(raw.rstrip(b"\r"), handle)
                    if len(buffer) > 16384:
                        buffer.clear()
                        with self.lock:
                            self.parse_error_count += 1
                except (OSError, ValueError) as exc:
                    with self.lock:
                        self.transport_errors.append(f"{type(exc).__name__}: {exc}")
                        if len(self.transport_errors) > 20:
                            self.transport_errors = self.transport_errors[-20:]
                    self._close()
                    buffer.clear()
        finally:
            self._close()
            if handle is not None:
                try:
                    handle.flush()
                    os.fsync(handle.fileno())
                except OSError as exc:
                    with self.lock:
                        self.storage_errors.append(f"{type(exc).__name__}: {exc}")
                finally:
                    handle.close()

    def is_connected(self) -> bool:
        """Serial device presence, independent of NMEA traffic or satellite fix."""
        with self.lock:
            device = self.active_device
            opened = self.fd is not None
        return bool(opened and device and os.path.exists(device))

    def latest_sentence_time(self) -> float:
        with self.lock:
            return float(self.last_sentence_timestamp or 0.0)

    def valid_fix_at_or_before(self, timestamp: float) -> dict | None:
        target = float(timestamp)
        with self.lock:
            for fix in reversed(self.valid_fix_history):
                if float(fix.get("timestamp") or 0.0) <= target:
                    return dict(fix)
        return None

    def stats(self) -> dict:
        with self.lock:
            latest_fix = None if self.latest_valid_fix is None else dict(self.latest_valid_fix)
            return {
                "enabled": True,
                "device": self.active_device or self.device,
                "device_setting": self.device,
                "preferred_device": self.preferred_device,
                "active_device": self.active_device,
                "candidate_devices": list(self.candidate_devices),
                "baudrate": self.baudrate,
                "sentence_count": self.sentence_count,
                "checksum_error_count": self.checksum_error_count,
                "parse_error_count": self.parse_error_count,
                "valid_fix_count": self.valid_fix_count,
                "last_sentence_timestamp": self.last_sentence_timestamp,
                "last_valid_fix_timestamp": self.last_valid_fix_timestamp,
                "latest_valid_fix": latest_fix,
                "receiver_open_count": self.receiver_open_count,
                "receiver_restart_count": max(0, self.receiver_open_count - 1),
                "transport_errors": list(self.transport_errors),
                "storage_errors": list(self.storage_errors),
                "storage_disabled": self.storage_disabled,
            }

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=5.0)


def normalize_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


# Linux SO_TIMESTAMPNS: the kernel attaches a struct timespec to each datagram.
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
_TIMESPEC = struct.Struct("qq")
CMSG_TIMESTAMP_SPACE = socket.CMSG_SPACE(_TIMESPEC.size) if hasattr(socket, "CMSG_SPACE") else 64


def _kernel_receive_time(ancdata) -> float | None:
    for level, ctype, cdata in ancdata:
        if level == socket.SOL_SOCKET and ctype == SO_TIMESTAMPNS:
            if len(cdata) < _TIMESPEC.size:
                return None
            seconds, nanoseconds = _TIMESPEC.unpack_from(cdata, 0)
            return seconds + nanoseconds / 1e9
    return None


def lidar_payload_within_fov(
    payload: bytes,
    forward_offset_deg: float,
    h_margin_deg: float,
) -> bool:
    """True when any block in this datagram falls inside the camera's view.

    pcap keeps or drops whole datagrams instead of trimming blocks out of them:
    a datagram missing some of its blocks is no longer the packet the sensor
    sent, and Hesai/Wireshark parsers reject it.

    That costs almost nothing.  One datagram covers about 1.4 degrees, so it is
    nearly always wholly inside or wholly outside the window - measured against
    live traffic, the old block-level crop kept 44,606 of 100,006 datagrams at
    an average of 1077 of 1080 bytes.  Dropping by datagram gives the same
    saving and leaves every stored packet intact.
    """
    header_size = xt32_packet.PACKET_HEADER_SIZE
    block_size = xt32_packet.BLOCK_SIZE
    body_end = len(payload)
    if xt32_packet.has_tail(payload):
        body_end -= xt32_packet.TAIL_SIZE
    for offset in range(header_size, body_end - block_size + 1, block_size):
        azimuth_deg = struct.unpack_from("<H", payload, offset)[0] / 100.0
        if abs(normalize_angle_deg(azimuth_deg - forward_offset_deg)) <= h_margin_deg:
            return True
    return False


def linux_udp_socket_inode(sock: socket.socket) -> str | None:
    """Return the Linux /proc socket inode without changing recvfrom semantics."""
    try:
        target = os.readlink(f"/proc/self/fd/{sock.fileno()}")
    except (OSError, AttributeError):
        return None
    if target.startswith("socket:[") and target.endswith("]"):
        return target[8:-1]
    return None


def linux_udp_kernel_drops(socket_inode: str | None) -> int | None:
    """Read the socket's cumulative kernel drop counter at low frequency."""
    if not socket_inode:
        return None
    for table_path in ("/proc/net/udp", "/proc/net/udp6"):
        try:
            with open(table_path, "r", encoding="ascii") as table:
                next(table, None)
                for line in table:
                    fields = line.split()
                    if len(fields) >= 13 and fields[9] == socket_inode:
                        return int(fields[-1])
        except (OSError, ValueError):
            continue
    return None


class RunRecorder:
    def __init__(self, base_dir: Path, run_sequence: int = 0, started_at: float | None = None):
        started_at = time.time() if started_at is None else float(started_at)
        base_run_id = run_id_for(started_at)
        # Use the slot timestamp, including an armed next-day rotation.
        base_dir = Path(base_dir) / datetime.fromtimestamp(started_at).strftime("%Y%m%d")
        self.run_sequence = int(run_sequence)
        self.started_at = started_at
        self.run_id = ""
        self.run_dir = None
        for collision_index in range(1000):
            suffix = "" if collision_index == 0 else f"_{collision_index:02d}"
            run_id = f"{base_run_id}{suffix}"
            run_dir = base_dir / run_id
            try:
                run_dir.mkdir(parents=True, exist_ok=False)
            except FileExistsError:
                # A restart inside the same slot must never append to the
                # previous folder: its frame indices start at 1 again.
                continue
            self.run_id = run_id
            self.run_dir = run_dir
            break
        if self.run_dir is None:
            raise RuntimeError(f"could not allocate a unique run directory under {base_dir}")

        self.frames_dir = self.run_dir / "frames"
        self.lidar_dir = self.run_dir / "lidar"
        self.gps_dir = self.run_dir / "gps"
        self.meta_dir = self.run_dir / "meta"

        for directory in [self.frames_dir, self.lidar_dir, self.gps_dir, self.meta_dir]:
            directory.mkdir(parents=False, exist_ok=False)

        self.frames_jsonl = self.frames_dir / "frames.jsonl"
        self.connection_events_jsonl = self.meta_dir / "connection_events.jsonl"
        self.health_jsonl = self.meta_dir / "health.jsonl"
        self.lidar_pcap_dir = self.lidar_dir
        self.gps_jsonl = self.gps_dir / "gps.jsonl"

        with (self.meta_dir / "config_snapshot.yaml").open("w", encoding="utf-8") as file:
            yaml.safe_dump(config, file, allow_unicode=True, sort_keys=False)

        append_jsonl(
            self.meta_dir / "run_meta.jsonl",
            {
                "event": "run_started",
                "run_id": self.run_id,
                "timestamp": time.time(),
                "run_sequence": self.run_sequence,
                "run_rotation_minutes": RUN_ROTATION_MINUTES,
                "save_dir": str(SAVE_DIR),
                "rtsp_url": RTSP_URL,
                "camera_codec": config.get("camera_codec"),
                "camera_output_width": config.get("camera_output_width"),
                "camera_output_height": config.get("camera_output_height"),
                "camera_output_fps": config.get("camera_output_fps"),
                "camera_gop_size": config.get("camera_gop_size"),
                "camera_rc_type": config.get("camera_rc_type"),
                "camera_h264_profile": config.get("camera_h264_profile"),
                "camera_latest_frame_enabled": CAMERA_LATEST_FRAME_ENABLED,
                "camera_frame_buffer_size": CAMERA_FRAME_BUFFER_SIZE,
                "camera_connection_timeout_sec": CAMERA_CONNECTION_TIMEOUT_SEC,
                "camera_watchdog_sec": CAMERA_WATCHDOG_SEC,
                "camera_reconnect_initial_sec": CAMERA_RECONNECT_INITIAL_SEC,
                "camera_reconnect_max_sec": CAMERA_RECONNECT_MAX_SEC,
                "frame_save_fps": FRAME_SAVE_FPS,
                "camera_async_write": CAMERA_ASYNC_WRITE,
                "camera_write_queue_size": CAMERA_WRITE_QUEUE_SIZE,
                "bbox_flip_x": BBOX_FLIP_X,
                "display_flip_x": DISPLAY_FLIP_X,
                "bbox_source_size": BBOX_SOURCE_SIZE,
                "bbox_letterbox_size": BBOX_LETTERBOX_SIZE,
                "lidar_udp": f"{LIDAR_BIND_IP}:{LIDAR_UDP_PORT}",
                "lidar_socket_rcvbuf_requested_bytes": LIDAR_SOCKET_RCVBUF_BYTES,
                "lidar_connection_timeout_sec": LIDAR_CONNECTION_TIMEOUT_SEC,
                "lidar_reconnect_initial_sec": LIDAR_RECONNECT_INITIAL_SEC,
                "lidar_reconnect_max_sec": LIDAR_RECONNECT_MAX_SEC,
                "lidar_fov_filter_enabled": LIDAR_FOV_FILTER_ENABLED,
                "camera_forward_offset_deg": LIDAR_CAMERA_FORWARD_OFFSET_DEG,
                "lidar_h_margin_deg": LIDAR_H_MARGIN_DEG,
                "lidar_save_mode": LIDAR_SAVE_MODE,
                "lidar_camera_offset_sec": LIDAR_CAMERA_OFFSET_SEC,
                "lidar_storage_format": LIDAR_STORAGE_FORMAT,
                "lidar_pcap_seconds": LIDAR_PCAP_SECONDS,
                "lidar_pcap_gzip": LIDAR_PCAP_GZIP,
                "lidar_kernel_timestamp": LIDAR_KERNEL_TIMESTAMP,
                "camera_pts_enabled": CAMERA_PTS_ENABLED,
                "camera_handover_grace_sec": CAMERA_HANDOVER_GRACE_SEC,
                "camera_manifest_timestamp": CAMERA_MANIFEST_TIMESTAMP,
                "camera_durable_writes": CAMERA_DURABLE_WRITES,
                "camera_read_mode": CAMERA_READ_MODE,
                "gps_enabled": GPS_ENABLED,
                "gps_device": GPS_DEVICE,
                "gps_preferred_device": GPS_PREFERRED_DEVICE,
                "gps_baudrate": GPS_BAUDRATE,
                "gps_connection_timeout_sec": GPS_CONNECTION_TIMEOUT_SEC,
                "gps_reconnect_initial_sec": GPS_RECONNECT_INITIAL_SEC,
                "gps_reconnect_max_sec": GPS_RECONNECT_MAX_SEC,
                "storage_preflight_mib": STORAGE_PREFLIGHT_MIB,
                "storage_min_write_mibs": STORAGE_MIN_WRITE_MIBS,
                "min_free_gb": MIN_FREE_GB,
                "disk_check_interval_sec": DISK_CHECK_INTERVAL_SEC,
                "health_log_interval_sec": HEALTH_LOG_INTERVAL_SEC,
                "health_warn_cpu_temp_c": HEALTH_WARN_CPU_TEMP_C,
            },
        )
        self.frames_jsonl.touch(exist_ok=True)
        self.gps_jsonl.touch(exist_ok=True)
        GUARD.add_run(self)


class LidarRawRecorder:
    """LiDAR UDP raw packet만 파일에 덤프한다. 파싱/좌표변환은 하지 않는다."""

    def __init__(
        self, bind_ip: str, port: int, log_interval_sec: float, recorder: "RunRecorder | None"
    ):
        self.bind_ip = bind_ip
        self.port = port
        self.log_interval_sec = log_interval_sec
        self.recorder = recorder
        self.running = False
        self.stop_event = threading.Event()
        self.thread = None
        self.lock = threading.Lock()
        self.packet_count = 0
        self.byte_count = 0
        self.saved_packet_count = 0
        self.saved_byte_count = 0
        self.dropped_packet_count = 0
        self.kernel_drop_count = 0
        self.socket_rcvbuf_bytes = 0
        self.frame_window_count = 0
        self.frame_window_empty_count = 0
        self.pcap_writer = None
        self.pending_recorder = None
        self.pending_boundary = None
        self.pcap_peer = LinkHeaderBuilder(dst_port=int(port))
        self.peer_known = False
        self.sequence_tracker = xt32_packet.SequenceTracker()
        self.clock_tracker = xt32_packet.ClockOffsetTracker()
        self.tail_present_count = 0
        self.tail_missing_count = 0
        self.kernel_timestamp_count = 0
        self.kernel_timestamp_available = False
        self.rotation_count = 0
        self.storage_errors = []
        self.transport_errors = []
        self.receiver_restart_count = 0
        self.receiver_bound = False
        self.fatal_error = None
        self.storage_lock = threading.Lock()
        self.first_packet_timestamp = None
        self.last_packet_timestamp = None

    def _open_storage(self):
        if self.recorder is None or not SAVE_LIDAR_RAW:
            return
        self.pcap_writer = PcapLidarWriter(
            self.recorder.lidar_pcap_dir,
            seconds_per_file=LIDAR_PCAP_SECONDS,
            gzip_output=LIDAR_PCAP_GZIP,
            max_pending=LIDAR_PCAP_QUEUE_SIZE,
            peer=self.pcap_peer,
        )

    def schedule_rotation(self, recorder: "RunRecorder", boundary: float):
        """Arm a folder switch that happens on packet time, not wall time.

        The main loop notices the boundary a few milliseconds late, and in
        those milliseconds the sensor keeps sending.  Switching here, against
        each packet's own timestamp, keeps every packet in the folder its
        timestamp belongs to.
        """
        with self.storage_lock:
            self.pending_recorder = recorder
            self.pending_boundary = float(boundary)

    def _apply_pending_rotation_locked(self, timestamp: float) -> bool:
        """Swap writers if this packet belongs to the next folder. Lock held."""
        if self.pending_recorder is None or timestamp < self.pending_boundary:
            return False
        recorder = self.pending_recorder
        self.pending_recorder = None
        self.pending_boundary = None
        self._swap_storage_locked(recorder)
        return True

    def rebind_recorder(self, recorder: "RunRecorder"):
        """Point storage at a new run folder without pausing reception.

        The new writer is opened and swapped in first; the old one is closed
        afterwards on a helper thread.  Closing joins a writer thread and can
        take seconds under disk load, and the receive loop must not wait for it.
        """
        if not SAVE_LIDAR_RAW:
            self.recorder = recorder
            return
        with self.storage_lock:
            self._swap_storage_locked(recorder)

    def _swap_storage_locked(self, recorder: "RunRecorder"):
        if not SAVE_LIDAR_RAW:
            self.recorder = recorder
            return
        previous = None
        self.recorder = recorder
        previous = self.pcap_writer
        self.pcap_writer = PcapLidarWriter(
            recorder.lidar_pcap_dir,
            seconds_per_file=LIDAR_PCAP_SECONDS,
            gzip_output=LIDAR_PCAP_GZIP,
            max_pending=LIDAR_PCAP_QUEUE_SIZE,
            peer=self.pcap_peer,
        )
        with self.lock:
            self.rotation_count += 1

        def _close_previous():
            try:
                previous.close()
            except Exception as exc:
                with self.lock:
                    self.storage_errors.append(
                        f"rotation close failed: {type(exc).__name__}: {exc}"
                    )

        if previous is not None:
            threading.Thread(
                target=_close_previous, name="lidar-storage-close", daemon=True
            ).start()

    def note_peer(self, addr):
        """Remember the sensor's address so synthesised pcap headers are real."""
        if self.peer_known or not addr:
            return
        try:
            self.pcap_peer.set_peer(addr[0], addr[1], dst_port=self.port)
        except Exception:
            return
        self.peer_known = True

    def start(self, external_source: bool = False):
        """Start storage and optionally the built-in UDP receiver.

        ``external_source=True`` is reserved for an external live source. Its
        viewer already owns UDP/2368, so packets are fed through
        :meth:`ingest_packet` instead of opening a second competing socket.
        """
        self._open_storage()
        self.stop_event.clear()
        self.running = True
        if not external_source:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def start_external(self):
        self.start(external_source=True)

    def update_transport_stats(
        self,
        socket_rcvbuf_bytes: int | None = None,
        kernel_drop_count: int | None = None,
    ):
        """Attach socket health measured by an external UDP owner."""
        with self.lock:
            if socket_rcvbuf_bytes is not None:
                self.socket_rcvbuf_bytes = int(socket_rcvbuf_bytes)
            if kernel_drop_count is not None:
                self.kernel_drop_count = max(self.kernel_drop_count, int(kernel_drop_count))

    def stop(self):
        self.running = False
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3)
            if self.thread.is_alive():
                self.storage_errors.append("LiDAR receiver thread did not stop")
        if self.pcap_writer:
            try:
                self.pcap_writer.close()
            except Exception as exc:
                self.storage_errors.append(repr(exc))
                print(f"[LiDAR] pcap writer close failed: {exc}")
        if getattr(self, "clock_file", None):
            self.clock_file.close()
            self.clock_file = None

    def stats(self):
        with self.lock:
            result = {
                "packet_count": self.packet_count,
                "byte_count": self.byte_count,
                "saved_packet_count": self.saved_packet_count,
                "saved_byte_count": self.saved_byte_count,
                "fov_fully_filtered_datagram_count": self.dropped_packet_count,
                # Backward-compatible internal name. These are FoV-filtered
                # datagrams, not UDP/kernel packet loss.
                "dropped_packet_count": self.dropped_packet_count,
                "kernel_drop_count": self.kernel_drop_count,
                "socket_rcvbuf_bytes": self.socket_rcvbuf_bytes,
                "frame_window_count": self.frame_window_count,
                "frame_window_empty_count": self.frame_window_empty_count,
                "clock_step_count": 0,
                "aligned_window_request_count": 0,
                "aligned_non_anchor_count": 0,
                "aligned_covered_count": 0,
                "aligned_buffer_miss_count": 0,
                "aligned_empty_count": 0,
                "aligned_rejected_count": 0,
                "aligned_duplicate_packets_skipped": 0,
                "aligned_clock_invalid": False,
                "aligned_continuous_fallback": False,
                "fallback_trigger_timestamp": None,
                "fallback_start_timestamp": None,
                "fallback_reason": None,
                "fallback_packet_count": 0,
                "first_packet_timestamp": self.first_packet_timestamp,
                "last_packet_timestamp": self.last_packet_timestamp,
                "storage_errors": list(self.storage_errors),
                "transport_errors": list(self.transport_errors),
                "receiver_restart_count": self.receiver_restart_count,
                "receiver_bound": self.receiver_bound,
                "fatal_error": self.fatal_error,
                "run_rotation_count": self.rotation_count,
                "packet_tail_present_count": self.tail_present_count,
                "packet_tail_missing_count": self.tail_missing_count,
                "kernel_timestamp_available": self.kernel_timestamp_available,
                "kernel_timestamp_count": self.kernel_timestamp_count,
                "sensor_sequence": self.sequence_tracker.summary(),
                "sensor_clock": self.clock_tracker.summary(),
            }
        if self.pcap_writer:
            result.update(self.pcap_writer.stats())
        else:
            result["storage_format"] = "pcap"
        with self.lock:
            result["received_packet_count"] = getattr(self, "received_packet_count", 0)
            result["last_receive_timestamp"] = getattr(self, "last_receive_timestamp", 0.0)
        return result

    def latest_packet_timestamp(self):
        # Connection health describes UDP reception, even while PTP admission
        # pauses storage. Never substitute this host time for sensor timestamps.
        with self.lock:
            return getattr(self, "last_receive_timestamp", 0.0)

    def fatal_error_message(self) -> str | None:
        with self.lock:
            return self.fatal_error

    def _storage_enabled(self):
        return self.pcap_writer is not None

    def _write_packet(self, timestamp: float, payload: bytes):
        if not self._storage_enabled() or not payload:
            return
        with self.storage_lock:
            self._apply_pending_rotation_locked(timestamp)
            self.pcap_writer.write_packet(timestamp, payload)
        with self.lock:
            self.saved_packet_count += 1
            self.saved_byte_count += len(payload)

    def ingest_packet(self, timestamp, data, kernel_drop_count=None):
        """Accept only monotonic, PTP-qualified sensor time, then apply the whole-packet FoV."""
        with self.lock:
            self.last_receive_timestamp = timestamp
            self.received_packet_count = getattr(self, "received_packet_count", 0) + 1
        self.update_transport_stats(kernel_drop_count=kernel_drop_count)
        okay, reason = GUARD.qualification()
        if not okay:
            GUARD.reject("lidar_" + reason)
            return
        try:
            meta = xt32_timestamp(data)
            announce = GUARD.status()[1]
            utc = native_to_utc(meta["native_timestamp_ns"], "ptp", announce)
        except ValueError:
            GUARD.reject("lidar_invalid_clock")
            return
        arrival = round(timestamp * NS)
        if not -5_000_000 <= GUARD.master_time_ns() - meta["native_timestamp_ns"] <= NS:
            GUARD.reject("lidar_invalid_epoch")
            return
        if getattr(self, "last_ptp_ns", None) is not None and utc <= self.last_ptp_ns:
            GUARD.reject("lidar_nonmonotonic")
            return
        self.last_ptp_ns = utc
        recorder = GUARD.recorder(utc)
        path = recorder.meta_dir / "lidar_clock.bin" if recorder else None
        if path != getattr(self, "clock_path", None):
            if getattr(self, "clock_file", None):
                self.clock_file.close()
            self.clock_path = path
            self.clock_file = path.open("ab", buffering=1 << 20) if path else None
        if getattr(self, "clock_file", None):
            self.clock_file.write(
                struct.pack(
                    "<QQQII",
                    utc,
                    arrival,
                    meta["native_timestamp_ns"],
                    meta["sequence"],
                    announce["current_utc_offset"],
                )
            )
        timestamp = utc / NS
        tail = xt32_packet.parse_tail(data)
        sensor_ts = None if tail is None else xt32_packet.sensor_timestamp_from_tail(tail)
        with self.lock:
            if tail is None:
                self.tail_missing_count += 1
            else:
                self.tail_present_count += 1
                self.sequence_tracker.update(tail["sequence"])
                self.clock_tracker.update(timestamp, sensor_ts)
        save_data = data
        if LIDAR_FOV_FILTER_ENABLED and not lidar_payload_within_fov(
            data, LIDAR_CAMERA_FORWARD_OFFSET_DEG, LIDAR_H_MARGIN_DEG
        ):
            save_data = b""
        if self._storage_enabled() and save_data:
            self._write_packet(timestamp, save_data)
        with self.lock:
            if self.first_packet_timestamp is None:
                self.first_packet_timestamp = timestamp
            self.last_packet_timestamp = timestamp
            self.packet_count += 1
            self.byte_count += len(data)
            if not save_data:
                self.dropped_packet_count += 1
        return len(save_data)

    def _run(self):
        retry_delay = LIDAR_RECONNECT_INITIAL_SEC
        last_log = 0.0
        while self.running:
            sock = None
            socket_inode = None
            with self.lock:
                socket_kernel_drop_base = self.kernel_drop_count
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                if LIDAR_SOCKET_RCVBUF_BYTES > 0:
                    sock.setsockopt(
                        socket.SOL_SOCKET,
                        socket.SO_RCVBUF,
                        LIDAR_SOCKET_RCVBUF_BYTES,
                    )
                actual_rcvbuf = int(sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))
                sock.bind((self.bind_ip, self.port))
                sock.settimeout(1)
                socket_inode = linux_udp_socket_inode(sock)
                kernel_stamps = False
                if LIDAR_KERNEL_TIMESTAMP:
                    try:
                        sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
                        kernel_stamps = True
                    except OSError as exc:
                        print(
                            f"[LiDAR WARN] SO_TIMESTAMPNS unavailable ({exc}); "
                            "falling back to user-space receive time",
                            flush=True,
                        )
                with self.lock:
                    self.socket_rcvbuf_bytes = actual_rcvbuf
                    self.receiver_bound = True
                    self.kernel_timestamp_available = kernel_stamps

                print(
                    f"LiDAR UDP listening on {self.bind_ip}:{self.port} "
                    f"rcvbuf={actual_rcvbuf} bytes"
                )
                if actual_rcvbuf < LIDAR_SOCKET_RCVBUF_BYTES:
                    print(
                        f"[LiDAR WARN] rcvbuf actual={actual_rcvbuf} < "
                        f"requested={LIDAR_SOCKET_RCVBUF_BYTES}. Run: "
                        f"sudo sysctl -w net.core.rmem_max="
                        f"{LIDAR_SOCKET_RCVBUF_BYTES}"
                    )
                retry_delay = LIDAR_RECONNECT_INITIAL_SEC

                while self.running:
                    kernel_timestamp = None
                    try:
                        if kernel_stamps:
                            data, ancdata, _flags, addr = sock.recvmsg(4096, CMSG_TIMESTAMP_SPACE)
                            kernel_timestamp = _kernel_receive_time(ancdata)
                        else:
                            data, addr = sock.recvfrom(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        if not self.running:
                            break
                        raise

                    # The kernel stamps the datagram on arrival; time.time()
                    # here also carries however long this thread waited to be
                    # scheduled, which was up to 1.5 ms in measurement.
                    if kernel_timestamp is not None:
                        timestamp = kernel_timestamp
                        with self.lock:
                            self.kernel_timestamp_count += 1
                    else:
                        timestamp = time.time()
                    self.note_peer(addr)
                    try:
                        saved_len = self.ingest_packet(timestamp, data)
                    except Exception as exc:
                        message = f"{type(exc).__name__}: {exc}"
                        with self.lock:
                            self.storage_errors.append(message)
                            self.fatal_error = message
                        print(
                            f"[LiDAR FATAL] packet storage failed: {message}",
                            flush=True,
                        )
                        self.running = False
                        self.stop_event.set()
                        break

                    if timestamp - last_log >= self.log_interval_sec:
                        last_log = timestamp
                        kernel_drops = linux_udp_kernel_drops(socket_inode)
                        if kernel_drops is not None:
                            with self.lock:
                                self.kernel_drop_count = max(
                                    self.kernel_drop_count,
                                    socket_kernel_drop_base + kernel_drops,
                                )
                        if LIDAR_FOV_FILTER_ENABLED:
                            print(
                                f"[LiDAR] packets={self.packet_count} "
                                f"saved={self.saved_packet_count} "
                                f"fov_drop={self.dropped_packet_count} "
                                f"kernel_drop={self.kernel_drop_count} "
                                f"src={addr[0]}:{addr[1]} "
                                f"len={len(data)} saved_len={saved_len}"
                            )
                        else:
                            print(
                                f"[LiDAR] packets={self.packet_count} "
                                f"kernel_drop={self.kernel_drop_count} "
                                f"src={addr[0]}:{addr[1]} len={len(data)}"
                            )
            except OSError as exc:
                if not self.running:
                    break
                message = f"{type(exc).__name__}: {exc}"
                with self.lock:
                    self.receiver_bound = False
                    self.receiver_restart_count += 1
                    self.transport_errors.append(message)
                    if len(self.transport_errors) > 20:
                        del self.transport_errors[:-20]
                print(
                    f"[LiDAR WARN] UDP receiver error: {message}; "
                    f"retrying in {retry_delay:.1f}s",
                    flush=True,
                )
                if self.stop_event.wait(retry_delay):
                    break
                retry_delay = min(
                    LIDAR_RECONNECT_MAX_SEC,
                    retry_delay * 2.0,
                )
            except Exception as exc:
                message = f"{type(exc).__name__}: {exc}"
                with self.lock:
                    self.receiver_bound = False
                    self.fatal_error = message
                    self.storage_errors.append(message)
                print(
                    f"[LiDAR FATAL] receiver stopped: {message}",
                    flush=True,
                )
                self.running = False
                self.stop_event.set()
                break
            finally:
                if sock is not None:
                    kernel_drops = linux_udp_kernel_drops(socket_inode)
                    if kernel_drops is not None:
                        with self.lock:
                            self.kernel_drop_count = max(
                                self.kernel_drop_count,
                                socket_kernel_drop_base + kernel_drops,
                            )
                    try:
                        sock.close()
                    except OSError:
                        pass
                with self.lock:
                    self.receiver_bound = False


def open_capture(url: str):
    return CameraCapture(
        url,
        buffer_size=CAMERA_FRAME_BUFFER_SIZE,
        watchdog_sec=CAMERA_WATCHDOG_SEC,
        connection_timeout_sec=CAMERA_CONNECTION_TIMEOUT_SEC,
    )


def capture_frame_info(capture, fallback_timestamp):
    getter = getattr(capture, "get_last_frame_info", None)
    if callable(getter):
        info = dict(getter() or {})
    else:
        info = {}
    arrival = float(info.get("arrival_timestamp") or fallback_timestamp)
    result = {
        "source_frame_id": info.get("source_frame_id"),
        "arrival_timestamp": arrival,
        "capture_mode": info.get("capture_mode", "synchronous"),
    }
    # Keep the camera-provided PTP/RTP capture metadata together.
    for key in (
        "capture_timestamp",
        "arrival_monotonic",
        "camera_pts",
        "camera_pts_seconds",
        "camera_time_base",
        "pts_epoch_offset",
        "pts_offset_converged",
        "pts_samples_in_window",
    ):
        if key in info:
            result[key] = info[key]
    result.update(info)
    return result


def frame_reference_timestamp(frame_info: dict, fallback: float) -> float:
    """The single time the rest of the collector treats as 'when the frame is'.

    Everything downstream - LiDAR window requests, save pacing - has to agree
    on one clock, and this is where that choice is made.
    """
    capture = frame_info.get("capture_timestamp")
    if capture is None or CAMERA_MANIFEST_TIMESTAMP == "arrival":
        return float(frame_info.get("arrival_timestamp") or fallback)
    return float(capture)


def merge_capture_stats(total, capture):
    getter = getattr(capture, "stats", None)
    stats = dict(getter() or {}) if callable(getter) else {"mode": "synchronous"}
    total["sessions"] = total.get("sessions", 0) + 1
    modes = total.setdefault("modes", [])
    mode = str(stats.get("mode", "unknown"))
    if mode not in modes:
        modes.append(mode)
    for key in (
        "decoded_frames",
        "delivered_frames",
        "selected_frames",
        "skipped_frames",
        "overwritten_frames",
        "read_failures",
        "pts_missing_frames",
        "pts_settling_skipped_frames",
        "pts_settle_timeouts",
        "pts_stream_resets",
        "timestamp_backstep_dropped_frames",
    ):
        total[key] = total.get(key, 0) + int(stats.get(key, 0))
    if mode in {"pyav_pts", "gstreamer_vaapi_pts"}:
        total["pts_startup_settled"] = bool(
            total.get("pts_startup_settled", True) and stats.get("pts_startup_settled", False)
        )


def build_camera_frame_record(
    frame_index: int,
    frame_timestamp: float,
    collector_read_timestamp: float,
    frame_info: dict,
    saved_timestamp: float,
    *,
    lidar_anchor: bool = True,
    camera_interval_sec: float | None = None,
    camera_gap_detected: bool = False,
):
    frame_name = f"{int(frame_index):08d}.jpg"
    # Three different clocks, kept apart on purpose:
    #   capture_timestamp - when the camera sampled the scene (PTS-derived)
    #   arrival_timestamp - when the decoded frame reached this process
    #   saved_timestamp   - when the JPEG hit the disk
    # `timestamp` names whichever of the first two the run was configured to
    # treat as the frame's time; timestamp_source says which one it is.
    arrival = float(frame_info.get("arrival_timestamp", frame_timestamp))
    capture = frame_info.get("capture_timestamp")
    timestamp_source = (
        "camera_ptp_rtcp_utc"
        if capture is not None and CAMERA_MANIFEST_TIMESTAMP == "capture"
        else "host_arrival"
    )
    record = {
        "frame_index": int(frame_index),
        "timestamp": float(frame_timestamp),
        "path": f"frames/{frame_name}",
        "timestamp_source": timestamp_source,
        "capture_timestamp": None if capture is None else float(capture),
        "arrival_timestamp": arrival,
        "arrival_monotonic": frame_info.get("arrival_monotonic"),
        "camera_pts": frame_info.get("camera_pts"),
        "camera_pts_seconds": frame_info.get("camera_pts_seconds"),
        "camera_time_base": frame_info.get("camera_time_base"),
        "pts_epoch_offset": frame_info.get("pts_epoch_offset"),
        "pts_offset_converged": frame_info.get("pts_offset_converged"),
        "pts_samples_in_window": frame_info.get("pts_samples_in_window"),
        "collector_read_timestamp": float(collector_read_timestamp),
        "saved_timestamp": float(saved_timestamp),
        "frame_age_ms_at_read": max(
            0.0,
            (float(collector_read_timestamp) - arrival) * 1000.0,
        ),
        "source_frame_id": frame_info.get("source_frame_id"),
        "capture_mode": frame_info.get("capture_mode", "synchronous"),
        "camera_interval_sec": (
            None if camera_interval_sec is None else float(camera_interval_sec)
        ),
        "camera_gap_detected": bool(camera_gap_detected),
    }
    for key in (
        "timestamp_ns",
        "timestamp_source",
        "arrival_timestamp_ns",
        "camera_rtp_timestamp",
        "camera_ssrc",
        "ptp_grandmaster",
    ):
        record[key] = frame_info[key]
    return record


class CameraFrameWriter:
    """Encode on an ordered worker, then durably write JPEGs on another worker.

    The queue absorbs short storage stalls caused by background uploads. Files
    are renamed atomically before their JSONL record is appended, so uploaders
    and post-processors never observe a manifest entry for a partial JPEG.
    """

    def __init__(self, recorder: RunRecorder, queue_size: int):
        self.recorder = recorder
        self.jobs = queue.Queue(maxsize=max(1, int(queue_size)))
        self.lock = threading.Lock()
        self.errors = []
        self.encode_jobs = queue.Queue(maxsize=8)
        self.submitted_frames = 0
        self.encode_queue_high_watermark = 0
        self.encode_queue_block_count = 0
        self.timings = {}
        self.encoded_frames = 0
        self.written_frames = 0
        self.written_bytes = 0
        self.queue_high_watermark = 0
        self.queue_block_count = 0
        self.queue_block_seconds = 0.0
        self.closed = False
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()
        self.encoder_thread = threading.Thread(
            target=self._encode_worker, name="camera-jpeg-encoder", daemon=True
        )
        self.encoder_thread.start()
        self.pressure_stop = threading.Event()
        self.pressure_thread = threading.Thread(target=self._pressure_loop, daemon=True)
        self.pressure_thread.start()

    def _pressure_loop(self):
        # RAM-only telemetry; never fsync or wait for analysis in acquisition.
        path = Path("/dev/shm/porthole_collector_pressure.json")
        temp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
        warned = False
        while True:
            stopping = self.pressure_stop.is_set()
            record = dict(
                pid=os.getpid(),
                updated_at=time.time(),
                active=not stopping,
                queue_pending=self.jobs.unfinished_tasks + self.encode_jobs.unfinished_tasks,
                queue_capacity=self.jobs.maxsize,
                camera_writer=self.stats(),
            )
            try:
                temp.write_text(json.dumps(record))
                temp.replace(path)
            except OSError as exc:
                if not warned:
                    print(f"[Collector] queue telemetry unavailable: {exc}", flush=True)
                    warned = True
            if stopping:
                return
            self.pressure_stop.wait(0.25)

    def _raise_if_failed(self):
        with self.lock:
            errors = list(self.errors)
        if errors:
            raise RuntimeError(f"camera frame writer failed: {errors}")

    def rebind_recorder(self, recorder: RunRecorder):
        """Send later frames to a new run folder; queued ones keep their own."""
        self.recorder = recorder

    def drain(self, timeout: float = 10.0) -> bool:
        """Wait for queued JPEGs to land, so a folder is complete when sealed."""
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if self.jobs.unfinished_tasks == 0 and self.encode_jobs.unfinished_tasks == 0:
                return True
            time.sleep(0.02)
        return self.jobs.unfinished_tasks == 0 and self.encode_jobs.unfinished_tasks == 0

    def enqueue(
        self,
        frame,
        frame_index: int,
        frame_timestamp: float,
        collector_read_timestamp: float,
        frame_info: dict,
        *,
        lidar_anchor: bool = True,
        camera_interval_sec: float | None = None,
        camera_gap_detected: bool = False,
    ):
        self._raise_if_failed()
        if self.closed:
            raise RuntimeError("camera frame writer is already closed")
        job = {
            "frame_index": int(frame_index),
            "frame": frame,  # Transfer ownership; the caller must not mutate this array.
            # The destination is captured now, not read by the worker later, so
            # a folder rotation cannot move already-queued frames into the
            # next run.
            "frames_dir": self.recorder.frames_dir,
            "frames_jsonl": self.recorder.frames_jsonl,
            "frame_timestamp": float(frame_timestamp),
            "collector_read_timestamp": float(collector_read_timestamp),
            "frame_info": dict(frame_info),
            "lidar_anchor": bool(lidar_anchor),
            "camera_interval_sec": camera_interval_sec,
            "camera_gap_detected": bool(camera_gap_detected),
        }
        if self.encode_jobs.full():
            with self.lock:
                self.encode_queue_block_count += 1
        self.encode_jobs.put(job)
        with self.lock:
            self.submitted_frames += 1
            self.encode_queue_high_watermark = max(
                self.encode_queue_high_watermark, self.encode_jobs.qsize()
            )
        self._raise_if_failed()

    def _timing(self, name, started):
        elapsed = time.perf_counter() - started
        with self.lock:
            row = self.timings.setdefault(name, {"count": 0, "total_ms": 0.0, "max_ms": 0.0})
            row["count"] += 1
            row["total_ms"] += elapsed * 1000
            row["max_ms"] = max(row["max_ms"], elapsed * 1000)

    def submit_callback(self, callback):
        self._raise_if_failed()
        self.encode_jobs.put({"callback": callback})

    def _encode_worker(self):
        while True:
            job = self.encode_jobs.get()
            try:
                if job is None:
                    self.jobs.put(None)
                    return
                self._raise_if_failed()
                if "callback" not in job:
                    started = time.perf_counter()
                    ok, encoded = cv2.imencode(
                        ".jpg", job.pop("frame"), [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
                    )
                    if not ok:
                        raise RuntimeError(f"failed to encode frame {job['frame_index']}")
                    job["jpeg"] = encoded.tobytes()
                    self._timing("jpeg_encode", started)
                    with self.lock:
                        self.encoded_frames += 1
                started = time.perf_counter()
                if self.jobs.full():
                    with self.lock:
                        self.queue_block_count += 1
                self.jobs.put(job)
                with self.lock:
                    self.queue_block_seconds += time.perf_counter() - started
                    self.queue_high_watermark = max(self.queue_high_watermark, self.jobs.qsize())
            except Exception as exc:
                with self.lock:
                    if not self.errors:
                        self.errors.append(f"{type(exc).__name__}: {exc}")
            finally:
                self.encode_jobs.task_done()

    def _worker(self):
        with ThreadPoolExecutor(max_workers=4, thread_name_prefix="camera-fsync") as sync_pool:
            self._write_loop(sync_pool)

    def _write_loop(self, sync_pool):
        manifest = None
        manifest_path = None
        carry = None
        while True:
            first = carry if carry is not None else self.jobs.get()
            carry = None
            if first is None or "stop_after_batch" in first:
                if manifest is not None:
                    manifest.close()
                self.jobs.task_done()
                return
            batch = [first]
            # At most eight JPEGs / 100 ms. Callback and stop tokens are barriers.
            if "callback" not in first:
                deadline = time.monotonic() + 0.100
                while len(batch) < 8:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    try:
                        job = self.jobs.get(timeout=remaining)
                    except queue.Empty:
                        break
                    if job is None or "callback" in job:
                        # A separate flag is needed because None is a valid stop token.
                        carry = job
                        if job is None:
                            carry = {"stop_after_batch": True}
                        break
                    batch.append(job)
            try:
                self._raise_if_failed()
                if "stop_after_batch" in first:
                    if manifest is not None:
                        manifest.close()
                    return
                if "callback" in first:
                    first["callback"]()
                    continue
                records = []
                directories = set()
                # Dirty the whole batch before the first fsync. On journaling
                # filesystems this lets a shared transaction cover several JPEGs,
                # instead of serial write/fsync pairs forcing separate commits.
                with ExitStack() as handles:
                    pending_files = []
                    for job in batch:
                        frame_name = f"{job['frame_index']:08d}.jpg"
                        frames_dir = job["frames_dir"]
                        final_path = frames_dir / frame_name
                        temp_path = frames_dir / f"{frame_name}.tmp"
                        started = time.perf_counter()
                        handle = handles.enter_context(temp_path.open("wb"))
                        handle.write(job["jpeg"])
                        handle.flush()
                        self._timing("jpeg_write", started)
                        pending_files.append((job, handle, temp_path, final_path))

                    def sync_one(handle):
                        started = time.perf_counter()
                        os.fsync(handle.fileno())
                        self._timing("jpeg_fsync", started)

                    started = time.perf_counter()
                    if CAMERA_DURABLE_WRITES:
                        futures = [
                            sync_pool.submit(sync_one, handle) for _, handle, _, _ in pending_files
                        ]
                        # Wait for every fd before closing any handle, including
                        # when one fsync failed. Never publish a failed batch.
                        wait(futures)
                        for future in futures:
                            future.result()
                    self._timing("jpeg_fsync_batch", started)
                    for job, handle, temp_path, final_path in pending_files:
                        os.replace(temp_path, final_path)
                        directories.add(job["frames_dir"])
                        record = build_camera_frame_record(
                            job["frame_index"],
                            job["frame_timestamp"],
                            job["collector_read_timestamp"],
                            job["frame_info"],
                            time.time(),
                            lidar_anchor=job["lidar_anchor"],
                            camera_interval_sec=job["camera_interval_sec"],
                            camera_gap_detected=job["camera_gap_detected"],
                        )
                        records.append((job, record))
                started = time.perf_counter()
                if CAMERA_DURABLE_WRITES:
                    for directory in directories:
                        fsync_directory(directory)
                self._timing("directory_fsync_batch", started)
                started = time.perf_counter()
                # Publish records only after every referenced JPEG and directory is durable.
                for job, record in records:
                    destination = job["frames_jsonl"]
                    if destination != manifest_path:
                        if manifest is not None:
                            manifest.flush()
                            if CAMERA_DURABLE_WRITES:
                                os.fsync(manifest.fileno())
                            manifest.close()
                        manifest = destination.open("a", encoding="utf-8")
                        manifest_path = destination
                    manifest.write(json.dumps(record, ensure_ascii=False) + "\n")
                manifest.flush()
                if CAMERA_DURABLE_WRITES:
                    os.fsync(manifest.fileno())
                self._timing("manifest_write_fsync_batch", started)
                with self.lock:
                    self.written_frames += len(records)
                    self.written_bytes += sum(len(job["jpeg"]) for job, _ in records)
            except Exception as exc:
                with self.lock:
                    if not self.errors:
                        self.errors.append(f"{type(exc).__name__}: {exc}")
            finally:
                for _ in batch:
                    self.jobs.task_done()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.encode_jobs.put(None)
        self.encode_jobs.join()
        self.encoder_thread.join(timeout=10.0)
        self.jobs.join()
        self.thread.join(timeout=10.0)
        self.pressure_stop.set()
        self.pressure_thread.join(timeout=1.0)
        if self.thread.is_alive():
            with self.lock:
                self.errors.append("camera frame writer did not stop")

    def stats(self):
        with self.lock:
            return {
                "mode": "async_encode_write",
                "submitted_frames": self.submitted_frames,
                "queue_pending": self.jobs.unfinished_tasks,
                "encode_queue_pending": self.encode_jobs.unfinished_tasks,
                "encode_queue_high_watermark": self.encode_queue_high_watermark,
                "encode_queue_block_count": self.encode_queue_block_count,
                "stage_timings": {
                    k: dict(v, mean_ms=v["total_ms"] / max(1, v["count"]))
                    for k, v in self.timings.items()
                },
                "encoded_frames": self.encoded_frames,
                "written_frames": self.written_frames,
                "written_bytes": self.written_bytes,
                "queue_high_watermark": self.queue_high_watermark,
                "queue_block_count": self.queue_block_count,
                "queue_block_seconds": self.queue_block_seconds,
                "errors": list(self.errors),
            }

    def fatal_error_message(self) -> str | None:
        with self.lock:
            return None if not self.errors else self.errors[0]


def save_camera_frame_record(
    recorder: RunRecorder,
    frame,
    frame_index: int,
    frame_timestamp: float,
    collector_read_timestamp: float,
    frame_info: dict,
    *,
    lidar_anchor: bool = True,
    camera_interval_sec: float | None = None,
    camera_gap_detected: bool = False,
):
    """Save one camera frame using the collector's canonical run schema."""
    frame_name = f"{int(frame_index):08d}.jpg"
    frame_path = recorder.frames_dir / frame_name
    saved = cv2.imwrite(
        str(frame_path),
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
    )
    if not saved:
        raise RuntimeError(f"failed to save camera frame: {frame_path}")
    saved_timestamp = time.time()
    frame_record = build_camera_frame_record(
        frame_index,
        frame_timestamp,
        collector_read_timestamp,
        frame_info,
        saved_timestamp,
        lidar_anchor=lidar_anchor,
        camera_interval_sec=camera_interval_sec,
        camera_gap_detected=camera_gap_detected,
    )
    append_jsonl(
        recorder.frames_jsonl,
        frame_record,
        durable=CAMERA_DURABLE_WRITES,
    )
    return frame_record


def finalize_run_record(
    recorder: RunRecorder,
    lidar_stats: dict,
    *,
    saved_frames: int,
    dropped_bad_frames: int,
    warmup_skipped_frames: int,
    camera_capture: dict,
    camera_storage: dict | None = None,
    connection_summary: dict | None = None,
    gps_stats: dict | None = None,
    fatal_error: str | None = None,
    shutdown_signal=None,
    entrypoint: str = "collect_data.py",
):
    """Write the shared ``run_finished`` schema and print its summary."""
    stats = dict(lidar_stats or {"packet_count": 0, "byte_count": 0})
    stats.setdefault("packet_count", 0)
    stats.setdefault("byte_count", 0)
    append_jsonl(
        recorder.meta_dir / "run_meta.jsonl",
        {
            "event": "run_finished",
            "run_id": recorder.run_id,
            "timestamp": time.time(),
            "entrypoint": str(entrypoint),
            "status": "failed" if fatal_error else "completed",
            "fatal_error": fatal_error,
            "saved_frames": int(saved_frames),
            "dropped_bad_frames": int(dropped_bad_frames),
            "warmup_skipped_frames": int(warmup_skipped_frames),
            "camera_capture": dict(camera_capture or {}),
            "camera_storage": dict(camera_storage or {}),
            "connection_summary": dict(connection_summary or {}),
            "gps": dict(gps_stats or {"enabled": False}),
            "shutdown_signal": shutdown_signal,
            "lidar_packets": stats["packet_count"],
            "lidar_bytes": stats["byte_count"],
            "lidar_saved_packets": stats.get("saved_packet_count", 0),
            "lidar_saved_bytes": stats.get("saved_byte_count", 0),
            "lidar_fov_fully_filtered_datagrams": stats.get(
                "fov_fully_filtered_datagram_count", stats.get("dropped_packet_count", 0)
            ),
            "lidar_dropped_packets": stats.get("dropped_packet_count", 0),
            "lidar_kernel_dropped_packets": stats.get("kernel_drop_count", 0),
            "lidar_socket_rcvbuf_bytes": stats.get("socket_rcvbuf_bytes", 0),
            "lidar_receiver_restarts": stats.get("receiver_restart_count", 0),
            "lidar_transport_errors": stats.get("transport_errors", []),
            "lidar_frame_windows": stats.get("frame_window_count", 0),
            "lidar_empty_frame_windows": stats.get("frame_window_empty_count", 0),
            "lidar_clock_steps": stats.get("clock_step_count", 0),
            "lidar_sensor_sequence": dict(stats.get("sensor_sequence") or {}),
            "lidar_storage": {
                key: value
                for key, value in stats.items()
                if key.startswith("chunk_")
                or key.startswith("pcap_")
                or key in {"storage_format", "storage_errors", "fatal_error"}
            },
        },
        durable=True,
    )
    print(f"Saved run: {recorder.run_dir}")
    print(f"  frames        : {saved_frames}")
    print(f"  warmup skipped: {warmup_skipped_frames}")
    print(f"  bad frames    : {dropped_bad_frames}")
    print(f"  camera capture: {camera_capture}")
    if camera_storage:
        print(f"  camera storage: {camera_storage}")
    print(f"  lidar packets : {stats['packet_count']}")
    print(
        f"  lidar saved   : {stats.get('saved_packet_count', 0)} packets, "
        f"{stats.get('saved_byte_count', 0)} bytes"
    )
    print(
        "  lidar FoV cut : "
        f"{stats.get('fov_fully_filtered_datagram_count', stats.get('dropped_packet_count', 0))} "
        "fully filtered datagrams (not transport loss)"
    )
    print(f"  lidar RX drop : {stats.get('kernel_drop_count', 0)} packets")
    print(f"  lidar rcvbuf  : {stats.get('socket_rcvbuf_bytes', 0)} bytes")
    print(f"  lidar windows : {stats.get('frame_window_count', 0)}")
    print(f"  lidar storage : {stats.get('storage_format', 'unknown')}")
    if gps_stats and gps_stats.get("enabled"):
        print(
            "  GPS           : "
            f"sentences={gps_stats.get('sentence_count', 0)} "
            f"valid_fixes={gps_stats.get('valid_fix_count', 0)} "
            f"checksum_errors={gps_stats.get('checksum_error_count', 0)}"
        )


def _run_collector(on_collection_ready=None, collection_stop_event=None):
    print("=== Camera + LiDAR + GPS Collector (no inference) ===")
    print(f"CONFIG_PATH    : {CONFIG_PATH}")
    print(f"RTSP_URL       : {RTSP_URL}")
    print(f"RTSP_PROFILE   : {RTSP_PROFILE}")
    print("RTSP_OPTS      : GStreamer TCP; latency=50ms")
    print(f"CAMERA_MODE    : {CAMERA_READ_MODE}")
    print(f"LATEST_FRAME   : {CAMERA_LATEST_FRAME_ENABLED} " f"buffer={CAMERA_FRAME_BUFFER_SIZE}")
    print(
        "CAMERA_RECOVER : "
        f"timeout={CAMERA_CONNECTION_TIMEOUT_SEC:.1f}s "
        f"watchdog={CAMERA_WATCHDOG_SEC:.1f}s "
        f"backoff={CAMERA_RECONNECT_INITIAL_SEC:.1f}.."
        f"{CAMERA_RECONNECT_MAX_SEC:.1f}s"
    )
    print(f"FRAME_SAVE_FPS : {FRAME_SAVE_FPS}")
    print(
        f"CAMERA_CLOCK   : pts={CAMERA_PTS_ENABLED} "
        f"manifest_timestamp={CAMERA_MANIFEST_TIMESTAMP}"
    )
    print(
        f"CAMERA_WRITER  : "
        f"{'async' if CAMERA_ASYNC_WRITE else 'synchronous'} "
        f"queue={CAMERA_WRITE_QUEUE_SIZE}"
    )
    print(f"JPEG_QUALITY   : {JPEG_QUALITY}")
    print(f"SHOW_WINDOW    : {SHOW_WINDOW}")
    print(f"CAMERA_DECODE  : {CAMERA_DECODE_ENABLED}")
    print(f"RAW_VIDEO_COPY : {SAVE_RAW_VIDEO_COPY}")
    print(f"BBOX_FLIP_X    : {BBOX_FLIP_X}")
    print(f"DISPLAY_FLIP_X : {DISPLAY_FLIP_X}")
    print(f"BBOX_SRC_SIZE  : {BBOX_SOURCE_SIZE or '-'}")
    print(f"BBOX_LBOX_SIZE : {BBOX_LETTERBOX_SIZE or '-'}")
    print(f"DROP_BAD_FRAMES: {DROP_BAD_FRAMES}")
    print(f"LIDAR_ENABLED  : {LIDAR_ENABLED}")
    print(f"LIDAR_UDP      : {LIDAR_BIND_IP}:{LIDAR_UDP_PORT}")
    print(f"LIDAR_RCVBUF   : requested={LIDAR_SOCKET_RCVBUF_BYTES} bytes")
    print(
        "LIDAR_RECOVER  : "
        f"timeout={LIDAR_CONNECTION_TIMEOUT_SEC:.1f}s "
        f"backoff={LIDAR_RECONNECT_INITIAL_SEC:.1f}.."
        f"{LIDAR_RECONNECT_MAX_SEC:.1f}s"
    )
    print(f"LIDAR_FOV      : {LIDAR_FOV_FILTER_ENABLED}")
    print(f"LIDAR_SAVE_MODE: {LIDAR_SAVE_MODE}")
    print(f"LIDAR_STORAGE  : {LIDAR_STORAGE_FORMAT}")
    print(f"LIDAR_PCAP_SEC : {LIDAR_PCAP_SECONDS} " f"gzip={LIDAR_PCAP_GZIP}")
    if LIDAR_FOV_FILTER_ENABLED:
        print(
            "LIDAR_FOV_MODE : whole datagrams only "
            "(a part-trimmed datagram is not a valid capture)"
        )
    print(f"LIDAR_KERNEL_TS: {LIDAR_KERNEL_TIMESTAMP}")
    if LIDAR_FOV_FILTER_ENABLED:
        print(
            f"LIDAR_FOV_CFG  : forward={LIDAR_CAMERA_FORWARD_OFFSET_DEG:.1f} deg, margin=+/-{LIDAR_H_MARGIN_DEG:.1f} deg"
        )
    print(f"GPS_ENABLED    : {GPS_ENABLED}")
    if GPS_ENABLED:
        print(f"GPS_DEVICE     : {GPS_DEVICE}")
        print(f"GPS_PREFERRED  : {GPS_PREFERRED_DEVICE or '-'}")
        print(f"GPS_BAUDRATE   : {GPS_BAUDRATE}")
        print(
            "GPS_RECOVER    : "
            f"timeout={GPS_CONNECTION_TIMEOUT_SEC:.1f}s "
            f"backoff={GPS_RECONNECT_INITIAL_SEC:.1f}.."
            f"{GPS_RECONNECT_MAX_SEC:.1f}s"
        )
    print(f"RECORD_ENABLED : {RECORD_ENABLED}")
    print(f"SAVE_DIR       : {SAVE_DIR}")

    # Collect until the disk is nearly full. There is no planned-duration
    # capacity preflight: the only storage limit is the min_free_gb reserve,
    # rechecked every disk_check_interval_sec while running.
    free_gb = check_free_space_gb(RUNS_DIR)
    print(f"DISK_FREE      : {free_gb:.1f} GB")
    print(
        "RUN_CAPACITY   : "
        f"reserve={MIN_FREE_GB:.1f}GiB, "
        f"recheck every {DISK_CHECK_INTERVAL_SEC:.0f}s"
    )
    if free_gb < MIN_FREE_GB:
        raise RuntimeError(
            "Not enough disk space to start: "
            f"{free_gb:.1f} GiB available < {MIN_FREE_GB:.1f} GiB reserve"
        )
    next_disk_check_at = time.time() + DISK_CHECK_INTERVAL_SEC

    write_mibs = measure_storage_write_mibs(RUNS_DIR, STORAGE_PREFLIGHT_MIB)
    if write_mibs is None:
        print("DISK_WRITE     : not measured")
    else:
        print(f"DISK_WRITE     : {write_mibs:.1f} MiB/s")
        if write_mibs < STORAGE_MIN_WRITE_MIBS:
            print(
                "",
                "*" * 70,
                f"[STORAGE WARN] {RUNS_DIR} writes at only {write_mibs:.2f} MiB/s.",
                f"               Collection needs about {STORAGE_MIN_WRITE_MIBS:.0f} MiB/s"
                " and will drop LiDAR packets below that.",
                "               A USB disk that negotiated a slow link looks exactly",
                "               like this. Check: lsusb -t | grep -i storage",
                "               (expect 5000M/480M with driver uas, not 12M usb-storage)",
                "               Re-seating the cable has fixed this before.",
                "*" * 70,
                "",
                sep="\n",
                flush=True,
            )

    recorder = RunRecorder(RUNS_DIR) if RECORD_ENABLED else None
    run_sequence = 0
    next_run_rotation_at = next_rotation_time(time.time()) if recorder is not None else None
    if recorder is not None:
        print(f"RUN_DIR        : {recorder.run_dir}")
        if next_run_rotation_at is None:
            print("RUN_ROTATION   : off (one folder per power-on)")
        else:
            print(
                f"RUN_ROTATION   : every {RUN_ROTATION_MINUTES:g} min, "
                f"next at "
                f"{datetime.fromtimestamp(next_run_rotation_at):%Y-%m-%d %H:%M:%S}"
            )
    camera_writer = None
    if recorder is not None and SAVE_FRAMES and CAMERA_ASYNC_WRITE:
        camera_writer = CameraFrameWriter(
            recorder,
            CAMERA_WRITE_QUEUE_SIZE,
        )

    cap = None
    print("Camera reconnect supervisor enabled; opening RTSP in main loop.")

    lidar = None
    if LIDAR_ENABLED:
        lidar = LidarRawRecorder(LIDAR_BIND_IP, LIDAR_UDP_PORT, LIDAR_LOG_INTERVAL_SEC, recorder)
        lidar.start()

    gps = None
    if GPS_ENABLED:
        gps = GpsNmeaRecorder(
            GPS_DEVICE,
            GPS_BAUDRATE,
            recorder,
            GPS_PREFERRED_DEVICE,
        )
        gps.start()

    connection_states = {}
    connection_state_started_at = {}
    connection_transition_counts = {}
    connection_downtime_sec = {"CAMERA": 0.0, "LIDAR": 0.0, "GPS": 0.0}
    connection_monitor_started_at = time.time()
    next_connection_check = 0.0
    last_camera_frame_seen = 0.0
    camera_stats = {}

    def report_connection_state(sensor: str, state: str, detail: str | None = None):
        """Print only connection transitions, never one line per packet/frame."""
        normalized = str(state).upper()
        previous = connection_states.get(sensor)
        if previous == normalized:
            return
        changed_at = time.time()
        previous_started_at = connection_state_started_at.get(sensor, changed_at)
        previous_duration = max(0.0, changed_at - previous_started_at)
        if previous == "DISCONNECTED":
            connection_downtime_sec[sensor] = (
                connection_downtime_sec.get(sensor, 0.0) + previous_duration
            )
        connection_states[sensor] = normalized
        connection_state_started_at[sensor] = changed_at
        count_key = f"{sensor}:{normalized}"
        connection_transition_counts[count_key] = connection_transition_counts.get(count_key, 0) + 1
        print(
            f"[CONNECTION {time.strftime('%H:%M:%S')}] " f"{sensor}={normalized}",
            flush=True,
        )
        if recorder is not None:
            event = {
                "event": "connection_state",
                "timestamp": changed_at,
                "sensor": sensor,
                "state": normalized,
                "previous_state": previous,
                "previous_state_duration_sec": previous_duration,
            }
            if detail:
                event["detail"] = str(detail)
            try:
                append_jsonl(recorder.connection_events_jsonl, event)
            except OSError as exc:
                print(
                    f"[CONNECTION WARN] failed to record state event: {exc}",
                    flush=True,
                )

    def poll_connection_states(now: float, force: bool = False):
        nonlocal next_connection_check
        if not force and now < next_connection_check:
            return
        next_connection_check = now + 0.2

        camera_last = last_camera_frame_seen
        stats_getter = getattr(cap, "stats", None)
        if callable(stats_getter):
            try:
                capture_stats = dict(stats_getter() or {})
                camera_last = max(
                    camera_last,
                    float(capture_stats.get("last_frame_timestamp") or 0.0),
                )
            except Exception:
                pass
        if camera_last > 0.0 and now - camera_last < CAMERA_CONNECTION_TIMEOUT_SEC:
            report_connection_state("CAMERA", "CONNECTED")
        elif camera_last > 0.0 or now - connection_monitor_started_at >= CAMERA_WATCHDOG_SEC:
            report_connection_state("CAMERA", "DISCONNECTED")
        else:
            report_connection_state("CAMERA", "WAITING")

        if lidar is not None:
            try:
                lidar_last = lidar.latest_packet_timestamp()
            except Exception:
                lidar_last = 0.0
            if lidar_last > 0.0 and now - lidar_last < LIDAR_CONNECTION_TIMEOUT_SEC:
                report_connection_state("LIDAR", "CONNECTED")
            elif now - connection_monitor_started_at >= LIDAR_CONNECTION_TIMEOUT_SEC:
                report_connection_state("LIDAR", "DISCONNECTED")
            else:
                report_connection_state("LIDAR", "WAITING")
        else:
            report_connection_state("LIDAR", "DISABLED")

        if gps is not None:
            report_connection_state("GPS", "CONNECTED" if gps.is_connected() else "DISCONNECTED")
        else:
            report_connection_state("GPS", "DISABLED")

    poll_connection_states(connection_monitor_started_at, force=True)

    # ffmpeg의 fps filter가 이미 저장 목표 이하로 프레임률을 제한한다면 다시
    # wall-clock 간격으로 거르지 않는다. 두 단계의 목표 FPS가 같을 때 수신 jitter로
    # nominal interval보다 아주 조금 짧아진 프레임을 버리면 실제 저장률이 크게 떨어진다.
    # PyAV decimates on capture time inside the capture thread, which is
    # strictly better than re-filtering here on arrival time.
    frame_index = 0
    # Set when a rotation is waiting for the camera stream to reach the
    # boundary; frames taken before it keep going to the folder that closed.
    pending_camera_recorder = None
    camera_boundary_time = None
    pending_finish = None
    armed_recorder = None
    armed_boundary = None

    def persisted_frame_count(target_recorder) -> int:
        """Return the exact number of frame records after queued writes drain."""
        try:
            with target_recorder.frames_jsonl.open("rb") as handle:
                return sum(1 for line in handle if line.strip())
        except FileNotFoundError:
            return 0

    def write_pending_finish(saved_frames: int, reason: str = "run_rotation_minutes"):
        """Close the rotated-out folder once its last frame has been written."""
        nonlocal pending_finish
        if pending_finish is None:
            return
        closing = pending_finish
        pending_finish = None

        def finish_after_frames():
            actual_saved_frames = persisted_frame_count(closing["recorder"])
            if actual_saved_frames != int(saved_frames):
                print(
                    "[FRAME COUNT] corrected run_finished saved_frames "
                    f"{saved_frames} -> {actual_saved_frames}",
                    flush=True,
                )
            lidar_snapshot = dict(closing["lidar_stats_process_cumulative"] or {})
            # Writer-local fields describe whichever folder was current when the
            # snapshot happened. At a packet-time boundary that may already be the
            # next folder, which made old run metadata report false pcap counts
            # (often zero). Per-folder pcap totals are authoritative in pcaps.jsonl;
            # keep this object strictly process-cumulative.
            for key in list(lidar_snapshot):
                if key.startswith("pcap_") or key.startswith("chunk_"):
                    lidar_snapshot.pop(key, None)
            lidar_snapshot.pop("storage_format", None)
            append_jsonl(
                closing["recorder"].meta_dir / "run_meta.jsonl",
                {
                    "event": "run_finished",
                    "run_id": closing["recorder"].run_id,
                    "timestamp": time.time(),
                    "entrypoint": "collect_data.py",
                    "status": "rotated",
                    "reason": reason,
                    "run_sequence": closing["recorder"].run_sequence,
                    "next_run_id": closing["next_run_id"],
                    "saved_frames": actual_saved_frames,
                    "frame_counter_before_manifest_check": int(saved_frames),
                    "dropped_bad_frames": closing["dropped_bad_frames"],
                    "warmup_skipped_frames": closing["warmup_skipped_frames"],
                    # Receiver counters are cumulative for the collector process,
                    # not for this folder alone. Per-folder counts come from the
                    # writer manifest after its asynchronous close has completed.
                    "lidar_stats_process_cumulative": lidar_snapshot,
                    "lidar_storage": {
                        "summary_source": "lidar/pcaps.jsonl",
                        "counts_in_process_snapshot": False,
                    },
                },
                durable=True,
            )

        if camera_writer is not None and not camera_writer.closed:
            camera_writer.submit_callback(finish_after_frames)
        else:
            finish_after_frames()

    last_saved_frame_timestamp = None
    bad_frame_count = 0
    warmup_skipped = 0
    collection_ready_notified = False
    camera_reconnect_attempt = 0
    next_camera_reconnect_at = 0.0
    warmup_until = 0.0
    fatal_exception = None
    shutdown_signal_event = threading.Event()
    shutdown_signal_name = None
    previous_signal_handlers = {}
    health_started_at = time.time()
    next_health_at = health_started_at + HEALTH_LOG_INTERVAL_SEC
    health_last_at = health_started_at
    health_last_frame_count = 0
    health_last_lidar_packets = 0
    health_sample_count = 0
    system_metrics = SystemMetricsSampler(min(30.0, HEALTH_LOG_INTERVAL_SEC))
    system_metrics.start()

    def emit_health_sample(now: float, force: bool = False):
        """Persist a bounded-cost long-run health sample and terminal heartbeat."""
        nonlocal next_health_at
        nonlocal health_last_at
        nonlocal health_last_frame_count
        nonlocal health_last_lidar_packets
        nonlocal health_sample_count

        if recorder is None:
            return
        if not force and now < next_health_at:
            return

        interval = max(1e-6, now - health_last_at)
        elapsed = max(0.0, now - health_started_at)
        camera_storage_now = {} if camera_writer is None else camera_writer.stats()
        lidar_now = {} if lidar is None else lidar.stats()
        gps_now = {"enabled": False} if gps is None else gps.stats()
        lidar_packets_now = int(lidar_now.get("packet_count", 0))
        # frame_index restarts at 0 in every rotated folder, so a negative
        # delta means a rotation happened inside this interval, not a fault.
        camera_recent_fps = max(0, int(frame_index) - health_last_frame_count) / interval
        lidar_recent_packet_rate = (lidar_packets_now - health_last_lidar_packets) / interval
        disk_free_gib = check_free_space_gb(RUNS_DIR)
        try:
            fs_stats = os.statvfs(RUNS_DIR)
            inode_free = int(fs_stats.f_favail)
            inode_total = int(fs_stats.f_files)
        except (OSError, AttributeError):
            inode_free = None
            inode_total = None

        camera_disk_bytes = int(camera_storage_now.get("written_bytes", 0))
        lidar_disk_bytes = int(lidar_now.get("pcap_disk_bytes", 0))
        committed_bytes = camera_disk_bytes + lidar_disk_bytes
        observed_gib_per_hour = (
            committed_bytes / (1024**3) * 3600.0 / elapsed if elapsed >= 60.0 else None
        )
        remaining_hours = (
            max(0.0, disk_free_gib - MIN_FREE_GB) / observed_gib_per_hour
            if observed_gib_per_hour and observed_gib_per_hour > 0
            else None
        )
        os_metrics = system_metrics.snapshot()
        cpu_temperature_c = os_metrics.get("cpu_temperature_c")
        cpu_throttled = os_metrics.get("cpu_throttled_status")
        process_rss_mib = os_metrics.get("process_rss_mib")
        load_average = os_metrics.get("load_average")

        analysis_status = {}
        analysis_status_path = Path("/dev/shm/porthole_analysis_status.json")
        try:
            analysis_status = json.loads(analysis_status_path.read_text())
        except (OSError, ValueError, TypeError):
            analysis_status = {}
        analysis_queue = analysis_status.get("queue", {})
        analysis_updated_at = analysis_status.get("updated_at")
        analysis_age_sec = (
            None if analysis_updated_at is None else max(0.0, now - float(analysis_updated_at))
        )
        analysis_run = analysis_status.get("active_run")
        analysis_summary = {
            "inference_pending_frames": analysis_queue.get("inference_pending_frames"),
            "inference_fps": analysis_queue.get("inference_fps"),
            "oldest_age_seconds": analysis_queue.get("oldest_age_seconds"),
            "status_age_seconds": analysis_age_sec,
            "active_run": analysis_run,
            "active_frames": analysis_status.get("active_frames"),
        }

        record = {
            "event": "health",
            "timestamp": float(now),
            "elapsed_sec": elapsed,
            "sample_index": health_sample_count,
            "camera_saved_frames": int(frame_index),
            "camera_recent_fps": camera_recent_fps,
            "camera_writer": camera_storage_now,
            "camera_connection_state": connection_states.get("CAMERA"),
            "lidar_packets": lidar_packets_now,
            "lidar_recent_packet_rate": lidar_recent_packet_rate,
            "lidar_saved_packets": int(lidar_now.get("saved_packet_count", 0)),
            "lidar_kernel_drops": int(lidar_now.get("kernel_drop_count", 0)),
            "lidar_pcap_queue_high_watermark": int(lidar_now.get("pcap_queue_high_watermark", 0)),
            "lidar_pcap_queue_block_count": int(lidar_now.get("pcap_queue_block_count", 0)),
            "lidar_pcap_writer_errors": lidar_now.get("pcap_writer_errors", []),
            "camera_capture": cap.stats() if cap is not None and hasattr(cap, "stats") else {},
            "lidar_receiver_restarts": int(lidar_now.get("receiver_restart_count", 0)),
            "lidar_connection_state": connection_states.get("LIDAR"),
            "gps_connection_state": connection_states.get("GPS"),
            "gps_sentences": int(gps_now.get("sentence_count", 0)),
            "gps_valid_fixes": int(gps_now.get("valid_fix_count", 0)),
            "gps_checksum_errors": int(gps_now.get("checksum_error_count", 0)),
            "gps_parse_errors": int(gps_now.get("parse_error_count", 0)),
            "gps_receiver_restarts": int(gps_now.get("receiver_restart_count", 0)),
            "gps_last_sentence_age_sec": (
                None
                if gps_now.get("last_sentence_timestamp") is None
                else max(0.0, now - float(gps_now["last_sentence_timestamp"]))
            ),
            "gps_last_valid_fix_age_sec": (
                None
                if gps_now.get("last_valid_fix_timestamp") is None
                else max(0.0, now - float(gps_now["last_valid_fix_timestamp"]))
            ),
            "committed_data_bytes": committed_bytes,
            "observed_storage_gib_per_hour": observed_gib_per_hour,
            "estimated_remaining_recording_hours": remaining_hours,
            "disk_free_gib": disk_free_gib,
            "disk_reserve_gib": MIN_FREE_GB,
            "inode_free": inode_free,
            "inode_total": inode_total,
            "process_rss_mib": process_rss_mib,
            "load_average": load_average,
            "cpu_temperature_c": cpu_temperature_c,
            "cpu_throttled_status": cpu_throttled,
            "analysis": analysis_summary,
        }
        append_jsonl(recorder.health_jsonl, record)

        storage_rate_text = (
            "-" if observed_gib_per_hour is None else f"{observed_gib_per_hour:.2f}GiB/h"
        )
        temp_text = "-" if cpu_temperature_c is None else f"{cpu_temperature_c:.1f}C"
        rss_text = "-" if process_rss_mib is None else f"{process_rss_mib:.0f}MiB"
        infer_q = analysis_summary["inference_pending_frames"]
        infer_fps = analysis_summary["inference_fps"]
        infer_q_text = "-" if infer_q is None else str(int(infer_q))
        infer_fps_text = "-" if infer_fps is None else f"{float(infer_fps):.2f}"
        analysis_age_text = "-" if analysis_age_sec is None else f"{analysis_age_sec:.0f}s"
        analysis_folder_text = "-" if not analysis_run else Path(analysis_run).name
        print(
            f"[HEALTH {time.strftime('%H:%M:%S')}] "
            f"elapsed={elapsed / 3600.0:.2f}h "
            f"camera={camera_recent_fps:.2f}fps "
            f"camQ={camera_storage_now.get('queue_pending', 0)}/"
            f"{CAMERA_WRITE_QUEUE_SIZE} "
            f"lidar={lidar_recent_packet_rate:.0f}pkt/s "
            f"rxDrop={lidar_now.get('kernel_drop_count', 0)} "
            f"gpsFix={gps_now.get('valid_fix_count', 0)} "
            f"inferQ={infer_q_text} inferFPS={infer_fps_text} "
            f"inferAge={analysis_age_text} inferFolder={analysis_folder_text} "
            f"disk={disk_free_gib:.1f}GiB "
            f"rate={storage_rate_text} temp={temp_text} rss={rss_text}",
            flush=True,
        )
        if cpu_temperature_c is not None and cpu_temperature_c >= HEALTH_WARN_CPU_TEMP_C:
            print(
                f"[HEALTH WARN] CPU temperature {cpu_temperature_c:.1f}C "
                f">= {HEALTH_WARN_CPU_TEMP_C:.1f}C",
                flush=True,
            )
        if cpu_throttled and not cpu_throttled.endswith("=0x0"):
            print(
                f"[HEALTH WARN] CPU throttling reported: {cpu_throttled}",
                flush=True,
            )
        if int(lidar_now.get("kernel_drop_count", 0)) > 0:
            print(
                "[HEALTH WARN] LiDAR UDP kernel drops are non-zero: "
                f"{lidar_now.get('kernel_drop_count', 0)}",
                flush=True,
            )

        health_sample_count += 1
        health_last_at = now
        health_last_frame_count = int(frame_index)
        health_last_lidar_packets = lidar_packets_now
        next_health_at = now + HEALTH_LOG_INTERVAL_SEC

    def arm_rotation(boundary_time: float):
        """Create the next folder early and hand the boundary to each writer.

        The writers switch on their own data timestamps, so they only need to
        know the boundary before it arrives.  Arming ahead of time removes the
        main loop from the timing path entirely: however late this loop notices
        the hour, the LiDAR and GPS threads have already been told where the
        line is.
        """
        nonlocal armed_recorder, armed_boundary
        if armed_recorder is not None:
            return
        try:
            armed_recorder = RunRecorder(
                RUNS_DIR,
                run_sequence=run_sequence + 1,
                started_at=boundary_time,
            )
        except Exception as exc:
            print(
                f"[ROTATE ERROR] could not create the next run folder: {exc}",
                flush=True,
            )
            return
        armed_boundary = boundary_time
        if lidar is not None:
            lidar.schedule_rotation(armed_recorder, boundary_time)
        if gps is not None:
            gps.rebind_recorder(armed_recorder, boundary_time)

    def rotate_run(boundary_time: float):
        """Seal the current folder and continue collecting into the next one.

        Sensors are never stopped.  Each writer opens its new destination
        first, so the only gap is the microseconds between swapping pointers -
        the LiDAR socket keeps receiving throughout, and the camera decoder
        thread is untouched.

        Frame numbering restarts at 1 in the new folder so that each folder is
        a complete, self-contained run rather than a slice that only makes
        sense next to its neighbours.
        """
        nonlocal recorder, run_sequence, next_run_rotation_at
        nonlocal health_last_frame_count, warmup_skipped, bad_frame_count
        nonlocal pending_camera_recorder, camera_boundary_time, pending_finish
        nonlocal armed_recorder, armed_boundary

        previous = recorder
        lidar_snapshot = None if lidar is None else lidar.stats()

        # arm_rotation already created the folder and told LiDAR/GPS where the
        # boundary is; if it could not, fall back to creating it here.
        if armed_recorder is None or armed_boundary != boundary_time:
            arm_rotation(boundary_time)
        new_recorder = armed_recorder
        armed_recorder = None
        armed_boundary = None
        if new_recorder is None:
            print(
                "[ROTATE ERROR] no folder available; continuing in the current one.",
                flush=True,
            )
            next_run_rotation_at = next_rotation_time(time.time())
            return

        # The camera is different.  A frame's capture time runs ahead of the
        # moment it reaches this process, so a frame arriving after the
        # boundary can have been taken before it - and its LiDAR is in the
        # folder that just closed.  Hand the camera over only when a frame's
        # *capture* time actually crosses the boundary, so each folder holds
        # one contiguous slice of both streams.
        pending_camera_recorder = new_recorder
        camera_boundary_time = boundary_time

        # Emit the closing health sample while `recorder` still points at the
        # old folder and frame_index still holds its real count, so the line
        # lands in the folder it describes and reports a true frame rate.
        emit_health_sample(time.time(), force=True)

        # The closing record is written when the camera actually leaves the
        # folder, so saved_frames counts the frames that really landed in it.
        pending_finish = {
            "recorder": previous,
            "next_run_id": new_recorder.run_id,
            "dropped_bad_frames": int(bad_frame_count),
            "warmup_skipped_frames": int(warmup_skipped),
            "lidar_stats_process_cumulative": lidar_snapshot,
        }

        recorder = new_recorder
        run_sequence += 1
        health_last_frame_count = 0
        bad_frame_count = 0
        warmup_skipped = 0
        next_run_rotation_at = next_rotation_time(time.time())
        print(
            f"[ROTATE] {previous.run_id} -> {new_recorder.run_id} "
            f"(next at "
            f"{datetime.fromtimestamp(next_run_rotation_at):%H:%M:%S})",
            flush=True,
        )

    def hand_over_camera(forced: bool = False):
        """Move the camera side of a rotation to the folder already armed.

        Normally this runs on the first frame whose capture time crosses the
        boundary.  But it waits on a frame arriving, and a camera that drops
        out at the boundary never delivers one: LiDAR and GPS move on while the
        writer and the closing record stay behind on a folder nothing writes to
        any more.  The next rotation then overwrites the pending closing
        record, so that folder never gets one at all.  The forced path exists
        for that case.
        """
        nonlocal pending_camera_recorder, camera_boundary_time, frame_index
        if pending_camera_recorder is None:
            return
        if camera_writer is not None:
            camera_writer.rebind_recorder(pending_camera_recorder)
        lag_ms = (time.time() - camera_boundary_time) * 1000.0
        detail = ", forced: no frame crossed it" if forced else ""
        print(
            f"[ROTATE] camera stream handed over to "
            f"{pending_camera_recorder.run_id} ({lag_ms:.0f} ms after the "
            f"boundary{detail})",
            flush=True,
        )
        pending_camera_recorder = None
        camera_boundary_time = None
        write_pending_finish(frame_index)
        frame_index = 0

    def request_graceful_shutdown(signum, _frame):
        nonlocal shutdown_signal_name
        try:
            shutdown_signal_name = signal.Signals(signum).name
        except (ValueError, AttributeError):
            shutdown_signal_name = str(signum)
        shutdown_signal_event.set()
        if collection_stop_event is not None:
            collection_stop_event.set()

    if threading.current_thread() is threading.main_thread():
        graceful_signals = [signal.SIGTERM]
        if hasattr(signal, "SIGHUP"):
            graceful_signals.append(signal.SIGHUP)
        for handled_signal in graceful_signals:
            previous_signal_handlers[handled_signal] = signal.getsignal(handled_signal)
            signal.signal(handled_signal, request_graceful_shutdown)

    def release_camera_capture():
        nonlocal cap
        current = cap
        cap = None
        if current is None:
            return
        try:
            merge_capture_stats(camera_stats, current)
        except Exception as exc:
            print(f"[camera] stats merge failed during release: {exc}")
        try:
            current.release()
        except Exception as exc:
            print(f"[camera] release failed: {exc}")

    def schedule_camera_reconnect(reason: str):
        nonlocal camera_reconnect_attempt, next_camera_reconnect_at
        release_camera_capture()
        camera_reconnect_attempt += 1
        exponent = min(camera_reconnect_attempt - 1, 10)
        delay = min(
            CAMERA_RECONNECT_MAX_SEC,
            CAMERA_RECONNECT_INITIAL_SEC * (2**exponent),
        )
        next_camera_reconnect_at = time.time() + delay
        report_connection_state("CAMERA", "DISCONNECTED", detail=reason)
        print(
            f"[camera] {reason}; retry={camera_reconnect_attempt} " f"in {delay:.1f}s",
            flush=True,
        )

    def try_open_camera(now: float) -> bool:
        nonlocal cap, warmup_until
        if cap is not None:
            return True
        if now < next_camera_reconnect_at:
            return False
        candidate = None
        try:
            candidate = open_capture(RTSP_URL)
            if candidate is None or not candidate.isOpened():
                raise RuntimeError("RTSP capture did not open")
        except Exception as exc:
            if candidate is not None:
                try:
                    candidate.release()
                except Exception:
                    pass
            schedule_camera_reconnect(f"RTSP open failed ({type(exc).__name__}: {exc})")
            return False

        cap = candidate
        warmup_until = time.time() + CAMERA_WARMUP_SEC
        print(
            "RTSP process opened; waiting for the first decoded frame.",
            flush=True,
        )
        return True

    try:
        while True:
            if shutdown_signal_event.is_set():
                print(f"Graceful shutdown requested ({shutdown_signal_name}).")
                break
            if collection_stop_event is not None and collection_stop_event.is_set():
                print("Collection stop requested.")
                break

            loop_now = time.time()
            if (
                pending_camera_recorder is not None
                and camera_boundary_time is not None
                and loop_now >= camera_boundary_time + CAMERA_HANDOVER_GRACE_SEC
            ):
                hand_over_camera(forced=True)
            if next_run_rotation_at is not None:
                if loop_now >= next_run_rotation_at - ROTATION_ARM_LEAD_SEC:
                    arm_rotation(next_run_rotation_at)
                if loop_now >= next_run_rotation_at:
                    rotate_run(next_run_rotation_at)
            poll_connection_states(loop_now)
            if loop_now >= next_disk_check_at:
                runtime_free_gb = check_free_space_gb(RUNS_DIR)
                next_disk_check_at = loop_now + DISK_CHECK_INTERVAL_SEC
                if runtime_free_gb < MIN_FREE_GB:
                    raise RuntimeError(
                        "Local storage safety threshold reached: "
                        f"{runtime_free_gb:.2f} GB < {MIN_FREE_GB:.2f} GB"
                    )
            emit_health_sample(loop_now)

            camera_write_fatal = (
                None if camera_writer is None else camera_writer.fatal_error_message()
            )
            if camera_write_fatal:
                raise RuntimeError(f"camera frame storage failure: {camera_write_fatal}")

            lidar_fatal = None if lidar is None else lidar.fatal_error_message()
            if lidar_fatal:
                raise RuntimeError(f"LiDAR storage/receiver failure: {lidar_fatal}")

            if cap is None:
                try_open_camera(loop_now)
                time.sleep(0.01)
                continue

            # grab()은 스트림을 한 프레임 진행시킨다(색공간 변환은 하지 않음).
            frame = None
            try:
                ok, frame = cap.read()
            except Exception as exc:
                schedule_camera_reconnect(f"frame read raised {type(exc).__name__}: {exc}")
                continue
            if not ok or frame is None:
                if getattr(cap, "nonblocking_read", False) and cap.isOpened():
                    time.sleep(0.005)
                    continue
                schedule_camera_reconnect("frame read failed or watchdog expired")
                continue

            collector_read_timestamp = time.time()
            frame_info = capture_frame_info(cap, collector_read_timestamp)
            frame_timestamp = frame_reference_timestamp(frame_info, collector_read_timestamp)
            last_camera_frame_seen = frame_timestamp
            report_connection_state("CAMERA", "CONNECTED")
            camera_reconnect_attempt = 0
            next_camera_reconnect_at = 0.0

            # 저장할 프레임만 여기서 디코딩한다.

            if frame_timestamp < warmup_until:
                # 워밍업 구간: 읽기는 계속하되 저장은 건너뛴다(시작 회색 프레임 제거).
                warmup_skipped += 1
                continue

            if pending_camera_recorder is not None and frame_timestamp >= camera_boundary_time:
                hand_over_camera()

            frame_index += 1

            camera_interval_sec = (
                None
                if last_saved_frame_timestamp is None
                else frame_timestamp - last_saved_frame_timestamp
            )
            expected_frame_interval = 1.0 / FRAME_SAVE_FPS
            camera_gap_detected = bool(
                camera_interval_sec is not None
                and camera_interval_sec
                > max(expected_frame_interval * 2.5, expected_frame_interval + 0.1)
            )
            lidar_anchor = True

            if recorder is not None and SAVE_FRAMES:
                if camera_writer is not None:
                    camera_writer.enqueue(
                        frame,
                        frame_index,
                        frame_timestamp,
                        collector_read_timestamp,
                        frame_info,
                        lidar_anchor=lidar_anchor,
                        camera_interval_sec=camera_interval_sec,
                        camera_gap_detected=camera_gap_detected,
                    )
                else:
                    save_camera_frame_record(
                        recorder,
                        frame,
                        frame_index,
                        frame_timestamp,
                        collector_read_timestamp,
                        frame_info,
                        lidar_anchor=lidar_anchor,
                        camera_interval_sec=camera_interval_sec,
                        camera_gap_detected=camera_gap_detected,
                    )

            last_saved_frame_timestamp = frame_timestamp

            if not collection_ready_notified:
                collection_ready_notified = True
                if on_collection_ready is not None:
                    on_collection_ready(recorder, frame_timestamp)

    except KeyboardInterrupt:
        print("Interrupted by user (Ctrl+C).")
    except Exception as exc:
        fatal_exception = exc
        print(
            f"[COLLECTOR FATAL] {type(exc).__name__}: {exc}",
            flush=True,
        )
    finally:
        release_camera_capture()
        if camera_writer is not None:
            camera_writer.close()
        if lidar is not None:
            lidar.stop()
        if gps is not None:
            gps.stop()
        try:
            emit_health_sample(time.time(), force=True)
        except Exception as exc:
            print(f"[HEALTH WARN] final health sample failed: {exc}", flush=True)
        system_metrics.stop()

    camera_storage_stats = {} if camera_writer is None else camera_writer.stats()
    if fatal_exception is None and camera_storage_stats.get("errors"):
        fatal_exception = RuntimeError(
            f"camera frame storage failed: {camera_storage_stats['errors']}"
        )

    if fatal_exception is None and lidar is not None:
        post_stop_stats = lidar.stats()
        post_stop_error = post_stop_stats.get("fatal_error")
        storage_errors = post_stop_stats.get("storage_errors") or []
        if post_stop_error or storage_errors:
            fatal_exception = RuntimeError(
                "LiDAR storage failed: " f"{post_stop_error or storage_errors}"
            )

    if recorder is not None:
        stats = lidar.stats() if lidar is not None else {"packet_count": 0, "byte_count": 0}
        gps_stats = gps.stats() if gps is not None else {"enabled": False}
        summary_time = time.time()
        downtime = dict(connection_downtime_sec)
        for sensor, state in connection_states.items():
            if state == "DISCONNECTED":
                downtime[sensor] = downtime.get(sensor, 0.0) + max(
                    0.0,
                    summary_time - connection_state_started_at.get(sensor, summary_time),
                )
        connection_summary = {
            "final_states": dict(connection_states),
            "transition_counts": dict(connection_transition_counts),
            "disconnected_seconds": downtime,
        }
        # A rotation may still be waiting for the camera to reach the boundary.
        # Close that folder first so it is never left without a run_finished.
        if pending_finish is not None:
            write_pending_finish(frame_index, reason="collector_stopped_before_handover")
            # recorder already points to the newly armed folder. No camera
            # frame crossed the boundary, so its own count is zero.
            frame_index = 0

        final_saved_frames = persisted_frame_count(recorder)
        if final_saved_frames != int(frame_index):
            print(
                "[FRAME COUNT] corrected final saved_frames "
                f"{frame_index} -> {final_saved_frames}",
                flush=True,
            )
        finalize_run_record(
            recorder,
            stats,
            saved_frames=final_saved_frames,
            dropped_bad_frames=bad_frame_count,
            warmup_skipped_frames=warmup_skipped,
            camera_capture=camera_stats,
            camera_storage=camera_storage_stats,
            connection_summary=connection_summary,
            gps_stats=gps_stats,
            fatal_error=(
                None
                if fatal_exception is None
                else f"{type(fatal_exception).__name__}: {fatal_exception}"
            ),
            shutdown_signal=shutdown_signal_name,
            entrypoint="collect_data.py",
        )

    print("Finished.")
    for handled_signal, previous_handler in previous_signal_handlers.items():
        signal.signal(handled_signal, previous_handler)
    if fatal_exception is not None:
        raise fatal_exception
    return recorder.run_dir if recorder is not None else None


def main(on_collection_ready=None, collection_stop_event=None):
    collector_lock = acquire_collector_lock()
    try:
        return _run_collector(
            on_collection_ready=on_collection_ready,
            collection_stop_event=collection_stop_event,
        )
    finally:
        collector_lock.close()


def run(duration=0.0):
    """Own the strict PTP guard for the full lifetime of the collector."""
    if not os.path.ismount("/mnt/ssd"):
        raise RuntimeError("SSD is not mounted")
    stop = threading.Event()
    GUARD.start()

    def watchdog():
        while not stop.wait(0.5):
            if GUARD.errors:
                stop.set()
                return

    threading.Thread(target=watchdog, daemon=True).start()
    timer = threading.Timer(duration, stop.set) if duration > 0 else None
    if timer:
        timer.start()
    try:
        main(collection_stop_event=stop)
    finally:
        stop.set()
        if timer:
            timer.cancel()
        GUARD.close()
        if GUARD.errors:
            raise RuntimeError("PTP monitor failed: " + repr(GUARD.errors))
