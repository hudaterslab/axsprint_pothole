"""PTP sensor management, RTCP clocks and strict recording admission."""

from __future__ import annotations
import concurrent.futures
import fcntl
import json
import math
import os
import re
import socket
import struct
import subprocess
import threading
import time
import urllib.parse
import urllib.request
from collections import Counter
from dataclasses import dataclass

NS = 1_000_000_000

NTP_UNIX_OFFSET = 2_208_988_800


def rtp_delta(value: int, reference: int) -> int:
    """Signed difference across a 32-bit RTP wrap (within half a wrap)."""
    return ((value - reference + 0x80000000) & 0xFFFFFFFF) - 0x80000000


def sender_reports(data: bytes) -> list[dict]:
    """Parse a compound RTCP packet; reject truncated/malformed input."""
    result = []
    offset = 0
    while offset < len(data):
        if len(data) - offset < 4 or data[offset] >> 6 != 2:
            raise ValueError("Malformed RTCP header")
        first, packet_type, words = struct.unpack_from("!BBH", data, offset)
        size = (words + 1) * 4
        if size < 4 or offset + size > len(data):
            raise ValueError("Truncated RTCP packet")
        effective = size
        if first & 0x20:
            padding = data[offset + size - 1]
            if not 0 < padding <= size - 4:
                raise ValueError("Invalid RTCP padding")
            effective -= padding
        if packet_type == 200:
            if effective < 28 + 24 * (first & 0x1F):
                raise ValueError("Truncated sender report")
            ssrc, sec, fraction, rtp, packets, octets = struct.unpack_from("!6I", data, offset + 4)
            result.append(
                dict(
                    ssrc=ssrc,
                    rtp=rtp,
                    ntp_seconds=sec,
                    ntp_fraction=fraction,
                    packet_count=packets,
                    octet_count=octets,
                    sender_utc_ns=(sec - NTP_UNIX_OFFSET) * NS + ((fraction * NS) >> 32),
                )
            )
        offset += size
    return result


@dataclass
class RtpClock:
    rate: int = 90_000
    max_report_age_ns: int = 20 * NS
    max_host_difference_ns: int = 2 * NS
    max_step_ns: int = 2_000_000
    last: dict | None = None
    stable_reports: int = 0
    rejected_reports: int = 0
    last_error: str = "no_sender_report"

    def update(self, report: dict, arrival_utc_ns: int, arrival_monotonic_ns: int) -> bool:
        if abs(report["sender_utc_ns"] - arrival_utc_ns) > self.max_host_difference_ns:
            self.stable_reports = 0
            self.last = None
            self.rejected_reports += 1
            self.last_error = "sender_clock_not_utc_synchronized"
            return False
        entry = dict(report, arrival_monotonic_ns=arrival_monotonic_ns)
        if self.last is None or report["ssrc"] != self.last["ssrc"]:
            self.stable_reports = 1
        else:
            predicted = (
                self.last["sender_utc_ns"]
                + rtp_delta(report["rtp"], self.last["rtp"]) * NS // self.rate
            )
            residual = report["sender_utc_ns"] - predicted
            entry["mapping_residual_ns"] = residual
            if abs(residual) > self.max_step_ns:
                self.stable_reports = 1
                self.rejected_reports += 1
                self.last_error = "sender_clock_step"
                self.last = entry
                return False
            self.stable_reports += 1
        self.last = entry
        self.last_error = "" if self.stable_reports >= 2 else "waiting_for_second_sender_report"
        return self.stable_reports >= 2

    def timestamp(self, rtp: int, ssrc: int, now_monotonic_ns: int) -> tuple[int | None, str]:
        if self.last is None or self.stable_reports < 2:
            return None, self.last_error
        if ssrc != self.last["ssrc"]:
            return None, "ssrc_changed"
        age = now_monotonic_ns - self.last["arrival_monotonic_ns"]
        if age < 0 or age > self.max_report_age_ns:
            return None, "sender_report_stale"
        return self.last["sender_utc_ns"] + rtp_delta(rtp, self.last["rtp"]) * NS // self.rate, ""


def ptp_announce(payload: bytes) -> dict | None:
    """Read the advertised PTP/UTC relationship; never guess the leap offset."""
    if len(payload) < 64 or payload[0] & 15 != 11 or payload[1] & 15 != 2:
        return None
    size = struct.unpack_from("!H", payload, 2)[0]
    if size < 64 or len(payload) < size:
        raise ValueError("Truncated PTP Announce")
    flags = struct.unpack_from("!H", payload, 6)[0]
    return dict(
        domain=payload[4],
        current_utc_offset=struct.unpack_from("!h", payload, 44)[0],
        utc_offset_valid=bool(flags & 4),
        ptp_timescale=bool(flags & 8),
        grandmaster_identity=payload[53:61].hex(),
        source_clock_identity=payload[20:28].hex(),
    )


def native_to_utc(native_ns: int, timescale: str, announce: dict | None) -> int:
    if timescale == "utc":
        return native_ns
    if timescale != "ptp":
        raise ValueError("Native time scale must be explicitly ptp or utc")
    if not announce or not announce["utc_offset_valid"] or not announce["ptp_timescale"]:
        raise ValueError("No valid PTP-to-UTC relationship received")
    return native_ns - announce["current_utc_offset"] * NS


def camera_command(action, host="192.168.11.10", timeout=5):
    """Send the CU22 SDK's PTP-only command (the protocol has no acknowledgement)."""
    code = {"start-ptp": 0x51, "stop-ptp": 0x52}[action]
    before = time.time_ns()
    with socket.create_connection((host, 8080), timeout=timeout) as sock:
        sock.sendall(struct.pack("<BI", code, 512) + bytes(512))
    return dict(action=action, sent=True, acknowledgement_supported=False, host_utc_ns=before)


SENSORS = {"lidar": ("enp1s0", "ec9f0d.fffe.035535"), "camera": ("enp2s0", "64351c.fffe.0109b8")}

MASTER = "48210b.fffe.72dc48"

# install.sh records each unit's own clock identities (from its NIC and sensor
# MAC addresses). Without that file the first unit's identities above apply.
DEVICE_CONFIG = os.environ.get("PTP_POTHOLE_DEVICE_CONFIG", "/etc/ptp-pothole/device.json")
if os.path.exists(DEVICE_CONFIG):
    with open(DEVICE_CONFIG) as _device_file:
        _device = json.load(_device_file)
    MASTER = _device["master_identity"]
    SENSORS = {
        name: (interface, _device["sensor_identities"][name])
        for name, (interface, _) in SENSORS.items()
    }


def read_sensor(name):
    interface, identity = SENSORS[name]
    args = [
        "/usr/sbin/pmc",
        "-4",
        "-i",
        interface,
        "-b",
        "0",
        "TARGET " + identity + "-1",
        "GET CURRENT_DATA_SET",
        "GET PORT_DATA_SET",
        "GET PARENT_DATA_SET",
    ]
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=5)
        text = r.stdout
        fields = {}
        for key in ["offsetFromMaster", "meanPathDelay", "portState", "grandmasterIdentity"]:
            m = re.search(r"^\s*" + key + r"\s+(\S+)", text, re.M)
            if not m:
                raise ValueError("Missing " + key + " in sensor response")
            fields[key] = m[1]
        return dict(
            sensor=name,
            identity=identity,
            interface=interface,
            offset_ns=float(fields["offsetFromMaster"]),
            mean_path_delay_ns=float(fields["meanPathDelay"]),
            port_state=fields["portState"],
            grandmaster_identity=fields["grandmasterIdentity"],
            measured_monotonic_ns=time.monotonic_ns(),
            measured_utc_ns=time.time_ns(),
        )
    except Exception as exc:
        return dict(
            sensor=name,
            error=str(exc),
            measured_monotonic_ns=time.monotonic_ns(),
            measured_utc_ns=time.time_ns(),
        )


def read_all():
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        return dict(zip(SENSORS, pool.map(read_sensor, SENSORS)))


def qualified(snapshot, now_monotonic_ns=None, max_offset_ns=100_000, max_age_ns=8_000_000_000):
    now = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
    for name in SENSORS:
        row = snapshot.get(name, {})
        if "error" in row or "offset_ns" not in row:
            return False, name + "_ptp_unavailable"
        if not 0 <= now - row["measured_monotonic_ns"] <= max_age_ns:
            return False, name + "_ptp_status_stale"
        if row["port_state"] != "SLAVE":
            return False, name + "_not_ptp_slave"
        if row["grandmaster_identity"] != MASTER:
            return False, name + "_unexpected_master"
        if not math.isfinite(row["offset_ns"]) or abs(row["offset_ns"]) > max_offset_ns:
            return False, name + "_ptp_offset_exceeded"
    return True, ""


def enable_sensors(force_camera=False):
    result = {}
    try:
        url = "http://192.168.1.201/pandar.cgi?"
        data = json.load(urllib.request.urlopen(url + "action=get&object=lidar_config", timeout=4))
        current = data.get("Body", {}).get("ClockSource")
        if str(current) not in ("0", "1"):
            raise ValueError("Unrecognized ClockSource response")
        if str(current) != "1":
            query = urllib.parse.urlencode(
                dict(action="set", object="lidar", key="clock_source", value="1")
            )
            result["lidar_set"] = json.load(urllib.request.urlopen(url + query, timeout=4))
        result["lidar_previous_clock_source"] = current
    except Exception as exc:
        result["lidar_error"] = str(exc)
    try:
        state = read_sensor("camera")
        if force_camera or state.get("port_state") != "SLAVE":
            result["camera_start"] = camera_command("start-ptp")
        result["camera_state"] = state.get("port_state", state.get("error"))
    except Exception as exc:
        result["camera_error"] = str(exc)
    return result


def udp_payload(frame, linktype=1):
    if linktype != 1:
        raise ValueError("Ethernet PCAP required")
    if len(frame) < 14:
        return None
    offset = 14
    kind = struct.unpack_from("!H", frame, 12)[0]
    while kind in (0x8100, 0x88A8):
        if len(frame) < offset + 4:
            return None
        kind = struct.unpack_from("!H", frame, offset + 2)[0]
        offset += 4
    if kind != 0x0800 or len(frame) < offset + 20:
        return None
    ihl = (frame[offset] & 15) * 4
    if frame[offset] >> 4 != 4 or ihl < 20 or frame[offset + 9] != 17:
        return None
    if struct.unpack_from("!H", frame, offset + 6)[0] & 0x3FFF:
        return None
    total = struct.unpack_from("!H", frame, offset + 2)[0]
    if total < ihl + 8 or len(frame) < offset + total:
        return None
    udp = offset + ihl
    source, destination, length = struct.unpack_from("!HHH", frame, udp)
    if length < 8 or udp + length > offset + total:
        return None
    return source, destination, frame[udp + 8 : udp + length]


class PtpGuard:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.sensors = {}
        self.announce = None
        self.announce_at = 0
        self.runs = []
        self.errors = []
        self.rejected = Counter()
        self.master_clock = None
        self.phc_fds = []
        self.startup_ready = False
        self.qualified_since_ns = 0

    def add_run(self, recorder):
        with self.lock:
            self.runs.append(recorder)
            self.runs = self.runs[-4:]
        (recorder.meta_dir / "ptp_clock_schema.json").write_text(
            json.dumps(
                {
                    "saved_clock": "UTC",
                    "camera_source": "RTP mapped by camera RTCP sender reports after PTP lock",
                    "lidar_pcap_header": "sensor UTC (microsecond PCAP precision)",
                    "lidar_packet_payload": "unchanged native PTP/TAI; subtract advertised currentUtcOffset exactly once",
                    "gps_timestamp": "host receive UTC; GPS fix fields retain receiver data",
                    "ptp_required": True,
                    "grandmaster_identity": MASTER,
                    "max_ptp_offset_ns": 100000,
                    "max_status_age_sec": 8,
                    "startup_stable_seconds": 5,
                    "grandmaster_clock": "both PHCs follow a common host reference with rate limits; their skew is checked with hardware cross timestamps",
                    "lidar_epoch_validation": "compared with PHC0 time, independently of host NTP corrections",
                    "lidar_clock_bin": "<QQQII per accepted incoming packet: sensor UTC ns, receive UTC ns, native PTP ns, sequence uint32, UTC offset seconds uint32; before FoV filter",
                    "camera_lidar_offset_sec": 0,
                    "exposure_phase_alignment": "See motion validation report; PTP does not trigger exposures or motor phase.",
                },
                indent=2,
            )
            + "\n"
        )

    def recorder(self, utc_ns=None):
        ts = (time.time_ns() if utc_ns is None else utc_ns) / NS
        with self.lock:
            eligible = [r for r in self.runs if r.started_at <= ts]
        return max(eligible, key=lambda r: r.started_at) if eligible else None

    def log(self, name, data, utc_ns=None):
        recorder = self.recorder(utc_ns)
        if recorder:
            with (recorder.meta_dir / name).open("a") as f:
                f.write(json.dumps(data) + "\n")

    def status(self):
        with self.lock:
            return dict(self.sensors), self.announce, self.announce_at

    def qualification(self):
        state = self.current_qualification()
        now = time.monotonic_ns()
        with self.lock:
            if self.startup_ready:
                return state
            if not state[0]:
                self.qualified_since_ns = 0
                return state
            if not self.qualified_since_ns:
                self.qualified_since_ns = now
            if now - self.qualified_since_ns < 5 * NS:
                return False, "startup_settling"
            self.startup_ready = True
        return state

    def current_qualification(self):
        if self.errors:
            return False, "ptp_monitor_failed"
        sensors, announce, at = self.status()
        now = time.monotonic_ns()
        ok, reason = qualified(sensors, now)
        if not ok:
            return False, reason
        if not announce or not 0 <= now - at <= 5 * NS:
            return False, "stale_master_announce"
        if not announce["utc_offset_valid"] or not announce["ptp_timescale"]:
            return False, "invalid_utc_relationship"
        with self.lock:
            master = self.master_clock
        if not master or not 0 <= now - master["monotonic_ns"] <= 8 * NS:
            return False, "master_clock_status_stale"
        if abs(master["port_skew_ns"]) > 100000:
            return False, "master_ports_not_synchronized"
        return True, ""

    def reject(self, reason):
        self.rejected[reason] += 1

    def start(self):
        self.phc_fds = [os.open("/dev/ptp" + str(n), os.O_RDONLY | os.O_CLOEXEC) for n in (0, 1)]
        self.phc_ids = [(~fd << 3) | 3 for fd in self.phc_fds]
        self.socket = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
        self.socket.bind(("enp1s0", 0))
        self.socket.settimeout(0.5)
        self.threads = [
            threading.Thread(target=self.monitor, daemon=True),
            threading.Thread(target=self.listen, daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    def master_time_ns(self):
        with self.lock:
            sample = self.master_clock
        return (
            sample["native_ptp_ns"]
            + time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            - sample["monotonic_raw_ns"]
        )

    def cross_timestamp(self, index):
        # Linux PTP_SYS_OFFSET_PRECISE returns simultaneous hardware/host clock
        # samples. Sequential MMIO clock reads have much larger PCIe latency.
        data = bytearray(64)
        fcntl.ioctl(self.phc_fds[index], (3 << 30) | (64 << 16) | (ord("=") << 8) | 8, data, True)
        values = []
        for offset in (0, 16, 32):
            sec, nsec = struct.unpack_from("=qI", data, offset)
            values.append(sec * NS + nsec)
        return values

    def read_master_clock(self):
        first = self.cross_timestamp(0)
        second = self.cross_timestamp(1)
        skew = second[0] - first[0] - (second[2] - first[2])
        return dict(
            monotonic_ns=time.monotonic_ns(),
            native_ptp_ns=first[0],
            monotonic_raw_ns=first[2],
            host_utc_ns=first[1],
            port_skew_ns=skew,
            measurement_method="PTP_SYS_OFFSET_PRECISE",
        )

    def monitor(self):
        try:
            reported = None
            while not self.stop_event.is_set():
                sensors = read_all()
                master = self.read_master_clock()
                with self.lock:
                    self.sensors = sensors
                    self.master_clock = master
                state = self.qualification()
                if state != reported:
                    label = "READY" if state[0] else "WAITING"
                    reason = "" if state[0] else " reason=" + state[1]
                    print("[PTP " + time.strftime("%H:%M:%S") + "] " + label + reason, flush=True)
                    reported = state
                self.log(
                    "ptp_status.jsonl",
                    dict(
                        host_utc_ns=time.time_ns(),
                        sensors=sensors,
                        master_clock=master,
                        qualification=state,
                        rejected=dict(self.rejected),
                    ),
                )
                self.stop_event.wait(1)
        except Exception as exc:
            self.errors.append(repr(exc))
            self.stop_event.set()

    def listen(self):
        try:
            while not self.stop_event.is_set():
                try:
                    frame = self.socket.recv(65536)
                except socket.timeout:
                    continue
                parsed = udp_payload(frame)
                if not parsed or parsed[1] not in (319, 320):
                    continue
                announce = ptp_announce(parsed[2])
                if (
                    announce
                    and announce["domain"] == 0
                    and announce["grandmaster_identity"] == MASTER.replace(".", "")
                ):
                    with self.lock:
                        self.announce = announce
                        self.announce_at = time.monotonic_ns()
                    self.log(
                        "ptp_announce.jsonl", dict(host_utc_ns=time.time_ns(), announce=announce)
                    )
        except Exception as exc:
            self.errors.append(repr(exc))
            self.stop_event.set()

    def close(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(7)
        self.socket.close()
        for fd in self.phc_fds:
            os.close(fd)
        self.phc_fds = []


GUARD = PtpGuard()
