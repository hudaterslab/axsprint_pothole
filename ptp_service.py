"""Two hardware LAN ports serving one PTP grandmaster with bounded UTC discipline.

The system clock is not written here; chrony only finishes its NTP correction once, before
the timebase starts, and only slews afterwards. Both PHCs use the same CLOCK_REALTIME reference.
Requires linuxptp 3.1.1+, root, and exclusive ownership of these PTP interfaces.
"""

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

# TAI - UTC in seconds, advertised in every Announce; changes only with a leap second.
UTC_OFFSET = 37

# Set once the PTP timebase is set up in this boot (/run is emptied at every boot), so that a
# restart of this service neither waits for NTP again nor steps the host clock under the sensors.
TIMEBASE_MARK = Path("/run/ptp_pothole_timebase")

LOG_STARTS = 30  # var/logs keeps the logs of this many service starts (phc2sys: ~7 MB a day per port)


def disable_clock_steps():
    """No automatic system clock step by chrony from now on (chronyc's second form of makestep:
    threshold, number of future clock updates). (accepted, chronyc's answer)."""
    try:
        result = subprocess.run(["/usr/bin/chronyc", "makestep", "1", "0"],
                                capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    return result.returncode == 0, (result.stdout + result.stderr).strip()


def prune_logs(logdir, keep=LOG_STARTS):
    """Make room for one more set of logs: each start writes <YYYYmmdd_HHMMSS>_<label> files,
    and only the newest keep - 1 earlier sets stay."""
    starts = sorted({path.name[:15] for path in logdir.iterdir() if path.name[8:9] == "_"})
    for old in starts[: max(0, len(starts) - (keep - 1))]:
        for path in logdir.glob(old + "_*"):
            path.unlink(missing_ok=True)


def main():
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
    prune_logs(logdir)
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
        if TIMEBASE_MARK.exists():
            print("PTP host clock: timebase set up earlier in this boot", flush=True)
        else:
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
            if Path("/usr/bin/chronyc").exists():
                if clock.get("NTPSynchronized") == "yes":
                    # Startup only: finish the pending NTP correction before creating
                    # the PTP timebase. Runtime corrections remain bounded by chrony.
                    subprocess.run(["/usr/bin/chronyc", "makestep"], check=True, timeout=5)
                # From here on chrony may only slew the host clock. phc2sys never steps the
                # PHCs after its start, so a step now (a first NTP update arriving late) would
                # leave both sensors seconds behind the host for hours, and the camera's RTCP
                # times would be refused for being that far from the host clock.
                steps_off, message = disable_clock_steps()
                print(("PTP host clock: no clock steps from now on: " if steps_off else
                       "PTP host clock: could not turn chrony's clock steps off (retried every 60 s): ")
                      + message, flush=True)
            TIMEBASE_MARK.write_text(f"{time.time_ns()}\n")
        for interface, phc in [("enp1s0", "ptp0"), ("enp2s0", "ptp1")]:
            expected = Path("/sys/class/net") / interface / "device/ptp" / phc
            if not expected.exists():
                raise RuntimeError(interface + " has an unexpected PHC")
        # Stop the camera's old servo while the host hardware clocks initialize. A camera that
        # cannot be reached (off, still booting) must not keep the LiDAR from getting PTP.
        try:
            camera_command("stop-ptp")
        except OSError as exc:
            print(f"PTP camera stop-ptp not sent: {exc}", flush=True)
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
                    str(UTC_OFFSET),
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
            f"currentUtcOffset {UTC_OFFSET} leap61 0 leap59 0 currentUtcOffsetValid 1 "
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
            except OSError as exc:
                if time.monotonic() >= camera_deadline:
                    # Keep serving the LiDAR; enable_sensors() starts the camera's PTP later.
                    print(f"PTP camera start-ptp not sent: {exc}", flush=True)
                    break
                time.sleep(0.5)
        (state / "ptp_runtime.json").write_text(
            json.dumps(
                dict(
                    started_utc_ns=time.time_ns(),
                    grandmaster_reference="common CLOCK_REALTIME; bounded hardware discipline",
                    utc_offset=UTC_OFFSET,
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
        next_steps_check, steps_off = time.monotonic() + 5, None
        while not stop:
            if any(c.poll() is not None for c in children):
                raise RuntimeError("A clock process exited; inspect logs")
            if time.monotonic() >= next_sensor_check:
                print(
                    json.dumps(dict(sensor_ptp=enable_sensors(), host_utc_ns=time.time_ns())),
                    flush=True,
                )
                next_sensor_check = time.monotonic() + 60
            # Every 5 s: a restarted chronyd reads makestep from its config again, and its first
            # clock update comes several seconds after its start.
            if time.monotonic() >= next_steps_check and Path("/usr/bin/chronyc").exists():
                ok, message = disable_clock_steps()
                if ok != steps_off:
                    print(("PTP host clock: chrony clock steps off: " if ok else
                           "PTP host clock: could not turn chrony's clock steps off: ") + message, flush=True)
                steps_off = ok
                next_steps_check = time.monotonic() + 5
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
