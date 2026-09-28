"""Two hardware LAN ports serving one PTP grandmaster with bounded UTC discipline.

No system clock writes. Both PHCs use the same CLOCK_REALTIME reference.
Requires linuxptp 3.1.1+, root, and exclusive ownership of these PTP interfaces.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import pwd
import signal
import subprocess
import time
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "app"))
from ptp import enable_sensors, camera_command


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--utc-offset", type=int, default=37)
    a = p.parse_args()
    base = Path(__file__).resolve().parent
    if os.geteuid() != 0:
        raise SystemExit("Run with sudo")
    state = base / "var"
    state.mkdir(exist_ok=True)
    # The service can recreate var before the desktop session starts. The
    # unprivileged collector must still be able to publish foreground.pid.
    collector_user = pwd.getpwnam("hudaters")
    os.chown(state, collector_user.pw_uid, collector_user.pw_gid)
    state.chmod(0o755)
    lock = (state / "ptp_service.lock").open("w")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    logdir = state / "logs"
    logdir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    children = []
    logs = []
    stop = False

    def stopping(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, stopping)
    signal.signal(signal.SIGINT, stopping)

    def start(label, argv):
        log = (logdir / (stamp + "_" + label + ".log")).open("a")
        logs.append(log)
        proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT)
        children.append(proc)
        return proc

    config = state / "ptp_hardware.conf"
    # igc can deliver a Tx timestamp later than ptp4l's 1 ms default; each such
    # timeout faulted a port for ~19 s. Wait longer for the timestamp, and
    # reset any remaining fault at once instead of after 16 s.
    config.write_text("""[global]
time_stamping hardware
network_transport UDPv4
delay_mechanism E2E
domainNumber 0
priority1 10
priority2 10
masterOnly 1
clockClass 248
boundary_clock_jbod 1
logSyncInterval -3
logAnnounceInterval 0
tx_timestamp_timeout 20
fault_reset_interval ASAP
uds_address /var/run/ptp_pothole
logging_level 6
""")
    try:
        # Use the same bounded host-clock wait as the foreground collector.
        # Advertising PTP before the first NTP correction makes both sensor
        # servos follow that correction and can interrupt admission at boot.
        clock_deadline = time.monotonic() + 60
        waiting_logged = False
        while not stop:
            try:
                check = subprocess.run(
                    ["/usr/bin/timedatectl", "show", "-p", "NTP", "-p", "NTPSynchronized"],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                clock = dict(
                    line.split("=", 1) for line in check.stdout.splitlines() if "=" in line
                )
            except (OSError, subprocess.TimeoutExpired):
                clock = {}
            if clock.get("NTPSynchronized") == "yes":
                print("PTP host clock: NTP synchronized", flush=True)
                break
            if clock.get("NTP") == "no":
                print("PTP host clock: NTP disabled; using RTC", flush=True)
                break
            if time.monotonic() >= clock_deadline:
                print("PTP host clock: NTP wait timed out after 60s; using RTC", flush=True)
                break
            if not waiting_logged:
                print(
                    "PTP host clock: waiting for initial NTP synchronization (up to 60s)",
                    flush=True,
                )
                waiting_logged = True
            time.sleep(0.5)
        if stop:
            return
        if clock.get("NTPSynchronized") == "yes" and Path("/usr/bin/chronyc").exists():
            # Startup only: finish the pending NTP correction before creating
            # the PTP timebase. Runtime corrections remain bounded by chrony.
            subprocess.run(["/usr/bin/chronyc", "makestep"], check=True, timeout=5)
        for interface, phc in [("enp1s0", "ptp0"), ("enp2s0", "ptp1")]:
            expected = Path("/sys/class/net") / interface / "device/ptp" / phc
            if not expected.exists():
                raise RuntimeError(interface + " has an unexpected PHC")
        # Stop the camera's old servo while the host hardware clocks initialize.
        camera_command("stop-ptp")
        # Both ports use the same reference and controller. This source/sink
        # combination uses the NIC's precise hardware/host cross timestamps;
        # direct PHC-to-PHC MMIO reads are too asymmetric on this device.
        for phc in ("ptp0", "ptp1"):
            start(
                phc,
                [
                    "/usr/sbin/phc2sys",
                    "-s",
                    "CLOCK_REALTIME",
                    "-c",
                    "/dev/" + phc,
                    "-O",
                    str(a.utc_offset),
                    "-P",
                    "0.1",
                    "-I",
                    "0.004",
                    "--max_frequency",
                    "200000",
                    "-m",
                    "-q",
                ],
            )
        # linuxptp PI's initial frequency estimate needs 0.016 / ki seconds.
        # Finish that estimate and the initial time step before advertising PTP.
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not stop:
            if any(c.poll() is not None for c in children):
                raise RuntimeError("A clock process exited; inspect logs")
            time.sleep(0.2)
        if stop:
            return
        start("ptp4l", ["ptp4l", "-f", str(config), "-i", "enp1s0", "-i", "enp2s0", "-m", "-q"])
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not stop:
            if any(c.poll() is not None for c in children):
                raise RuntimeError("A clock process exited; inspect logs")
            time.sleep(0.2)
        if stop:
            return
        setting = (
            "SET GRANDMASTER_SETTINGS_NP clockClass 248 clockAccuracy 0xfe offsetScaledLogVariance 0xffff "
            f"currentUtcOffset {a.utc_offset} leap61 0 leap59 0 currentUtcOffsetValid 1 "
            "ptpTimescale 1 timeTraceable 0 frequencyTraceable 0 timeSource 0x50"
        )
        result = subprocess.run(
            [
                "pmc",
                "-u",
                "-s",
                "/var/run/ptp_pothole",
                "-b",
                "0",
                setting,
                "GET TIME_PROPERTIES_DATA_SET",
                "GET PORT_DATA_SET",
            ],
            capture_output=True,
            text=True,
            timeout=8,
        )
        (logdir / (stamp + "_master_properties.txt")).write_text(result.stdout + result.stderr)
        if "currentUtcOffsetValid 1" not in result.stdout:
            raise RuntimeError("Master UTC properties were not confirmed")
        # The camera stays powered while this host reboots. Restart only its
        # vendor PTP daemon once, allowing its initial clock step to the new GM.
        # Otherwise it can take minutes to slew an old subsecond offset away.
        camera_deadline = time.monotonic() + 10
        while not stop:
            try:
                camera_command("start-ptp")
                break
            except OSError:
                if time.monotonic() >= camera_deadline:
                    raise
                time.sleep(0.5)
        (state / "ptp_runtime.json").write_text(
            json.dumps(
                dict(
                    started_utc_ns=time.time_ns(),
                    grandmaster_reference="common CLOCK_REALTIME; bounded hardware discipline",
                    utc_offset=a.utc_offset,
                    port_clock_validation="PTP_SYS_OFFSET_PRECISE",
                    max_phc_frequency_ppb=200000,
                    interfaces=["enp1s0", "enp2s0"],
                    clock_children=[c.pid for c in children],
                    log_prefix=stamp,
                ),
                indent=2,
            )
        )
        print("Common hardware PTP master ready on enp1s0 and enp2s0", flush=True)
        # Do not send START_PTP again while the just-started camera servo is
        # acquiring the master. A duplicate start can reset its acquisition.
        next_sensor_check = time.monotonic() + 60
        while not stop:
            if any(c.poll() is not None for c in children):
                raise RuntimeError("A clock process exited; inspect logs")
            if time.monotonic() >= next_sensor_check:
                print(
                    json.dumps(dict(sensor_ptp=enable_sensors(), host_utc_ns=time.time_ns())),
                    flush=True,
                )
                next_sensor_check = time.monotonic() + 60
            time.sleep(0.5)
    finally:
        for c in reversed(children):
            if c.poll() is None:
                c.terminate()
        for c in children:
            try:
                c.wait(timeout=3)
            except subprocess.TimeoutExpired:
                c.kill()
                c.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
