#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
SUPERVISOR_MODE=0
if [[ "${1:-}" == "--supervisor" ]]; then
  if [[ $# -ne 4 ]]; then
    echo "usage: $0 --supervisor COMPONENT TERMINAL_PID LOG_FILE" >&2
    exit 2
  fi
  SUPERVISOR_MODE=1
  COMPONENT="$2"
else
  COMPONENT="${PORTHOLE_FOREGROUND_COMPONENT:-${1:-}}"
fi
CONFIG_PATH_OVERRIDE="${CONFIG_PATH:-}"
CONFIG_PATH="${CONFIG_PATH:-$SCRIPT_DIR/config.yaml}"
SAVE_MOUNT="${PORTHOLE_SAVE_MOUNT:-/mnt/ssd}"
SAVE_ROOT="${PORTHOLE_SAVE_ROOT:-$SAVE_MOUNT/porthole_runs}"
STORAGE_RESOLVER="$SCRIPT_DIR/resolve_recording_storage.sh"
STOP_TIMEOUT_SEC="${PORTHOLE_STOP_TIMEOUT_SEC:-60}"

case "$COMPONENT" in
  analysis)
    COMPONENT_LABEL="Analysis V2"
    LOCK_FILE="/tmp/porthole_analysis_foreground.lock"
    STATE_FILE="/tmp/porthole_analysis_foreground.state"
    LEGACY_SERVICE="porthole-analysis-v2.service"
    ;;
  collector)
    COMPONENT_LABEL="Collector"
    LOCK_FILE="${PORTHOLE_COLLECTOR_FOREGROUND_LOCK:-/tmp/porthole_collector_foreground.lock}"
    STATE_FILE="${PORTHOLE_COLLECTOR_FOREGROUND_STATE:-/tmp/porthole_collector_foreground.state}"
    LEGACY_SERVICE="porthole-collector.service"
    ;;
  uploader)
    COMPONENT_LABEL="Uploader"
    LOCK_FILE="${PORTHOLE_UPLOADER_FOREGROUND_LOCK:-/tmp/porthole_uploader_foreground.lock}"
    STATE_FILE="${PORTHOLE_UPLOADER_FOREGROUND_STATE:-/tmp/porthole_uploader_foreground.state}"
    LEGACY_SERVICE="porthole-uploader.service"
    ;;
  *)
    echo "usage: $0 {collector|uploader|analysis}" >&2
    exit 2
    ;;
esac

# Collection and live analysis (main_live.py) are self-contained in this
# directory. The legacy upload command keeps its original path.
LEGACY_COMPONENT_DIR="/home/hudaters/Desktop/porthole"
if [[ "$COMPONENT" == "collector" ]]; then
  CONFIG_PATH="$SCRIPT_DIR/config.yaml"
else
  CONFIG_PATH="${CONFIG_PATH_OVERRIDE:-$LEGACY_COMPONENT_DIR/config.yaml}"
fi
CHILD_PID=""
LAUNCH_PID=""
CLEANED_UP=0

process_alive() {
  local pid="${1:-}"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

remove_owned_state() {
  local state_pid=""
  if [[ -r "$STATE_FILE" ]]; then
    IFS=$'\t' read -r state_pid _ <"$STATE_FILE" || true
    if [[ "$state_pid" == "$$" ]]; then
      rm -f "$STATE_FILE"
    fi
  fi
}

write_owner_state() {
  local terminal_owner_pid="$1"
  local log_file="$2"
  local state_tmp="${STATE_FILE}.$$"
  umask 077
  printf '%s\t%s\t%s\n' "$$" "$terminal_owner_pid" "$log_file" >"$state_tmp"
  mv -f "$state_tmp" "$STATE_FILE"
}

stop_process_tree() {
  local pid="$1"
  if process_alive "$pid"; then
    echo "[$COMPONENT_LABEL] Sending SIGTERM (pid=$pid)..."
    pkill -TERM -P "$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  fi
}

component_cleanup() {
  local reason="${1:-supervisor_exit}"
  if (( CLEANED_UP )); then
    return
  fi
  CLEANED_UP=1
  trap - HUP INT TERM EXIT

  echo
  echo "[$COMPONENT_LABEL] Stopping: reason=$reason"
  stop_process_tree "$CHILD_PID"

  local timeout_sec=15
  if [[ "$COMPONENT" == "collector" ]]; then
    timeout_sec="$STOP_TIMEOUT_SEC"
  fi
  local deadline=$((SECONDS + timeout_sec))
  while process_alive "$CHILD_PID" && (( SECONDS < deadline )); do
    sleep 0.2
  done
  if process_alive "$CHILD_PID"; then
    echo "[$COMPONENT_LABEL] Stop exceeded ${timeout_sec}s; forcing exit." >&2
    pkill -KILL -P "$CHILD_PID" 2>/dev/null || true
    kill -KILL "$CHILD_PID" 2>/dev/null || true
  fi
  [[ -z "$CHILD_PID" ]] || wait "${LAUNCH_PID:-$CHILD_PID}" 2>/dev/null || true

  if [[ "$COMPONENT" == "collector" ]] && mountpoint -q "$SAVE_MOUNT"; then
    if command -v timeout >/dev/null 2>&1; then
      timeout 30 sync -f "$SAVE_ROOT" \
        || echo "[Collector] WARNING: filesystem sync did not finish in 30s." >&2
    else
      sync -f "$SAVE_ROOT" || true
    fi
  fi
  remove_owned_state
  echo "[$COMPONENT_LABEL] Stopped safely."
}

component_signal() {
  local signal_name="$1"
  component_cleanup "signal_$signal_name"
  exit 0
}

component_fail() {
  local message="$1"
  echo
  echo "[$COMPONENT_LABEL] ERROR: $message" >&2
  echo "[$COMPONENT_LABEL] The process was not started or was stopped safely." >&2
  return 1
}

wait_for_storage() {
  if [[ ! -x "$STORAGE_RESOLVER" ]]; then
    component_fail "storage resolver is missing or not executable: $STORAGE_RESOLVER"
    return 1
  fi

  local resolved
  if ! resolved="$($STORAGE_RESOLVER)"; then
    component_fail "safe external recording storage could not be resolved"
    return 1
  fi
  IFS=$'\t' read -r SAVE_MOUNT SAVE_ROOT <<<"$resolved"
  if [[ -z "$SAVE_MOUNT" || -z "$SAVE_ROOT" || ! -d "$SAVE_ROOT" || ! -w "$SAVE_ROOT" ]]; then
    component_fail "storage resolver returned an invalid save path"
    return 1
  fi

  export PORTHOLE_SAVE_MOUNT="$SAVE_MOUNT"
  export PORTHOLE_SAVE_ROOT="$SAVE_ROOT"
  export PORTHOLE_SAVE_DIR="$SAVE_ROOT"
  echo "[$COMPONENT_LABEL] STORAGE=$SAVE_MOUNT SAVE_DIR=$SAVE_ROOT"
}

wait_for_ntp_sync() {
  local terminal_owner_pid="$1"
  local max_wait="${PORTHOLE_NTP_SYNC_WAIT_SEC:-60}"
  local started_at=$SECONDS

  if [[ "$COMPONENT" != "collector" ]]; then
    return 0
  fi
  if ! command -v timedatectl >/dev/null 2>&1; then
    echo "[$COMPONENT_LABEL] NTP check unavailable; starting with the RTC clock."
    return 0
  fi
  if [[ "$(timedatectl show -p NTP --value 2>/dev/null)" != "yes" ]]; then
    echo "[$COMPONENT_LABEL] NTP is disabled; starting with the RTC clock."
    return 0
  fi
  if [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" == "yes" ]]; then
    echo "[$COMPONENT_LABEL] CLOCK=NTP_SYNCHRONIZED"
    return 0
  fi

  echo "[$COMPONENT_LABEL] CLOCK=WAITING_FOR_NTP timeout=${max_wait}s"
  while (( SECONDS - started_at < max_wait )); do
    if ! process_alive "$terminal_owner_pid"; then
      echo "[$COMPONENT_LABEL] Terminal closed while waiting for NTP; not starting."
      return 1
    fi
    if [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" == "yes" ]]; then
      echo "[$COMPONENT_LABEL] CLOCK=NTP_SYNCHRONIZED waited=$((SECONDS - started_at))s"
      return 0
    fi
    sleep 1
  done

  echo "[$COMPONENT_LABEL] CLOCK=NTP_TIMEOUT waited=${max_wait}s; starting with RTC clock."
  return 0
}

supervisor_main() {
  local terminal_owner_pid="$1"
  local log_file="$2"

  # Keep graceful shutdown independent of the terminal PTY disappearing.
  exec </dev/null >>"$log_file" 2>&1

  cat <<EOF
================================================================
 Porthole $COMPONENT_LABEL
================================================================

This terminal owns only the ${COMPONENT}.
Closing this window or pressing Ctrl+C stops only the ${COMPONENT}.

EOF

  trap 'component_signal HUP' HUP
  trap 'component_signal INT' INT
  trap 'component_signal TERM' TERM
  trap 'component_cleanup supervisor_exit' EXIT

  exec 9>"$LOCK_FILE"
  if ! flock -n 9; then
    component_fail "another $COMPONENT foreground terminal is already running"
    return 1
  fi

  if systemctl is-active --quiet "$LEGACY_SERVICE"; then
    component_fail \
      "$LEGACY_SERVICE is active; run install_foreground_autostart.sh"
    return 1
  fi

  if [[ "$COMPONENT" == "collector" ]]; then
    for required in "$CONFIG_PATH" "$SCRIPT_DIR/collect_data.py" "/usr/local/sbin/ptp-pothole-collector-foreground"; do
      if [[ ! -f "$required" ]]; then
        component_fail "required file not found: $required"
        return 1
      fi
    done
  elif [[ "$COMPONENT" == "analysis" ]]; then
    if [[ ! -f "$SCRIPT_DIR/main_live.py" ]]; then
      component_fail "required file not found: $SCRIPT_DIR/main_live.py"
      return 1
    fi
  elif [[ ! -f "$LEGACY_COMPONENT_DIR/upload_runs_to_171.sh" ]]; then
    component_fail "required file not found: $SCRIPT_DIR/upload_runs_to_171.sh"
    return 1
  fi

  wait_for_storage || return 1
  wait_for_ntp_sync "$terminal_owner_pid" || return 0
  write_owner_state "$terminal_owner_pid" "$log_file"

  export CONFIG_PATH
  export PYTHONUNBUFFERED=1

  echo "[$COMPONENT_LABEL] Starting..."
  if [[ "$COMPONENT" == "collector" ]]; then
    local ptp_pid_file="$SCRIPT_DIR/var/foreground.pid"
    rm -f -- "$ptp_pid_file"
    /usr/bin/sudo -n /usr/local/sbin/ptp-pothole-collector-foreground &
  elif [[ "$COMPONENT" == "analysis" ]]; then
    # run_main_live.sh restarts main_live.py after an error exit (e.g. an NPU reset).
    (cd "$SCRIPT_DIR" && exec /usr/bin/env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
      /usr/bin/nice -n 15 /usr/bin/ionice -c 3 "$SCRIPT_DIR/run_main_live.sh") &
  else
    "$LEGACY_COMPONENT_DIR/upload_runs_to_171.sh" --watch &
  fi
  CHILD_PID=$!
  LAUNCH_PID="$CHILD_PID"
  if [[ "$COMPONENT" == "collector" ]]; then
    local pid_deadline=$((SECONDS + 15))
    while [[ ! -s "$ptp_pid_file" ]] && (( SECONDS < pid_deadline )); do
      sleep 0.1
    done
    if [[ ! -s "$ptp_pid_file" ]]; then
      component_fail "PTP collector launcher did not publish its PID"
      return 1
    fi
    IFS= read -r CHILD_PID <"$ptp_pid_file"
    if [[ ! "$CHILD_PID" =~ ^[0-9]+$ ]]; then
      component_fail "invalid collector PID"
      return 1
    fi
  fi

  sleep 2
  if ! process_alive "$CHILD_PID"; then
    local child_status=0
    wait "${LAUNCH_PID:-$CHILD_PID}" || child_status=$?
    CHILD_PID=""
    component_fail "$COMPONENT exited during startup (status=$child_status)"
    return 1
  fi

  echo "[$COMPONENT_LABEL] Running."
  echo "[$COMPONENT_LABEL] Close this window or press Ctrl+C to stop only this process."
  echo "----------------------------------------------------------------"

  while process_alive "$terminal_owner_pid"; do
    if ! process_alive "$CHILD_PID"; then
      local child_status=0
      wait "${LAUNCH_PID:-$CHILD_PID}" || child_status=$?
      CHILD_PID=""
      if (( child_status == 0 )); then
        # A clean exit is not a failure. Saying ERROR here alarms the driver,
        # who is not the person who can tell exit 0 from a crash.
        echo "[$COMPONENT_LABEL] $COMPONENT finished on its own and saved the run."
        return 0
      fi
      component_fail "$COMPONENT exited with an error (status=$child_status)"
      return 1
    fi
    sleep 0.2
  done

  component_cleanup "terminal_window_closed"
}

terminal_signal() {
  # The detached supervisor notices this owner exit and performs cleanup after
  # qterminal has released its PTY.
  trap - HUP INT TERM
  exit 0
}

terminal_hold_error() {
  echo
  echo "[$COMPONENT_LABEL] Supervisor stopped. Close this window after reading the log." >&2
  while :; do
    sleep 60
  done
}

terminal_main() {
  printf '\033]0;Porthole %s — close window to stop only %s\007' \
    "$COMPONENT_LABEL" "$COMPONENT"
  clear
  echo "Starting Porthole $COMPONENT_LABEL foreground controller..."
  echo "Closing this window stops only the $COMPONENT."
  echo

  trap 'terminal_signal' HUP
  trap 'terminal_signal' INT
  trap 'terminal_signal' TERM

  local log_file="/tmp/porthole_${COMPONENT}_foreground_$(date +%Y%m%d_%H%M%S)_$$.log"
  : >"$log_file"

  setsid "$SCRIPT_DIR/run_component_foreground.sh" \
    --supervisor "$COMPONENT" "$$" "$log_file" \
    </dev/null >/dev/null 2>&1 &
  local supervisor_pid=$!

  tail --pid="$supervisor_pid" --sleep-interval=0.2 -n +1 -F "$log_file" &
  local tail_pid=$!
  wait "$tail_pid" 2>/dev/null || true

  local supervisor_status=0
  wait "$supervisor_pid" || supervisor_status=$?
  if (( supervisor_status != 0 )); then
    terminal_hold_error
  fi
}

if (( SUPERVISOR_MODE )); then
  supervisor_main "$3" "$4"
  exit $?
fi

terminal_main
