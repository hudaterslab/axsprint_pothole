#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
COMPONENT="${1:-}"

case "$COMPONENT" in
  analysis)
    COMPONENT_LABEL="Analysis V2"
    ;;
  collector)
    COMPONENT_LABEL="Collector"
    ;;
  uploader)
    COMPONENT_LABEL="Uploader"
    ;;
  *)
    echo "usage: $0 {collector|uploader|analysis}" >&2
    exit 2
    ;;
esac

SUPERVISOR_PID=""
LOG_FILE=""
TAIL_PID=""

process_alive() {
  local pid="${1:-}"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

read_owner_state() {
  local state_file="/tmp/porthole_${COMPONENT}_foreground.state"
  local state_pid=""
  local state_terminal_pid=""
  local state_log=""

  if [[ -r "$state_file" ]]; then
    IFS=$'\t' read -r state_pid state_terminal_pid state_log <"$state_file" || true
    if process_alive "$state_pid" \
      && [[ "$state_pid" =~ ^[0-9]+$ ]] \
      && [[ -r "$state_log" ]]; then
      SUPERVISOR_PID="$state_pid"
      LOG_FILE="$state_log"
      return 0
    fi
  fi

  # Compatibility with a supervisor that started before state-file support
  # was deployed. Read its NUL-delimited argv rather than guessing a log name.
  local pid=""
  local -a argv=()
  while IFS= read -r pid; do
    [[ -r "/proc/$pid/cmdline" ]] || continue
    mapfile -d '' -t argv <"/proc/$pid/cmdline" || true
    if (( ${#argv[@]} >= 6 )) \
      && [[ "${argv[1]}" == "$SCRIPT_DIR/run_component_foreground.sh" ]] \
      && [[ "${argv[2]}" == "--supervisor" ]] \
      && [[ "${argv[3]}" == "$COMPONENT" ]] \
      && [[ -r "${argv[5]}" ]]; then
      SUPERVISOR_PID="$pid"
      LOG_FILE="${argv[5]}"
      return 0
    fi
  done < <(pgrep -u "$UID" -f \
    "$SCRIPT_DIR/run_component_foreground.sh --supervisor $COMPONENT " || true)

  return 1
}

stop_attached_owner() {
  local signal_name="$1"
  trap - HUP INT TERM
  echo
  echo "[$COMPONENT_LABEL] Remote control terminal closing; stopping pid=$SUPERVISOR_PID..."
  if process_alive "$SUPERVISOR_PID"; then
    kill -TERM "$SUPERVISOR_PID" 2>/dev/null || true
  fi
  [[ -z "$TAIL_PID" ]] || kill "$TAIL_PID" 2>/dev/null || true
  exit 0
}

printf '\033]0;Porthole %s — attached control\007' "$COMPONENT_LABEL"
clear

if ! read_owner_state; then
  echo "No active $COMPONENT owner was found."
  echo "Starting a new foreground owner in this terminal."
  echo
  exec "$SCRIPT_DIR/run_component_foreground.sh" "$COMPONENT"
fi

cat <<EOF
================================================================
 Porthole $COMPONENT_LABEL — attached XRDP control
================================================================

The process is running in the local-console session.
This terminal shows its live log without starting a duplicate.
Closing this window or pressing Ctrl+C stops only the ${COMPONENT}.

Supervisor PID: $SUPERVISOR_PID
Log: $LOG_FILE
----------------------------------------------------------------
EOF

trap 'stop_attached_owner HUP' HUP
trap 'stop_attached_owner INT' INT
trap 'stop_attached_owner TERM' TERM

tail --pid="$SUPERVISOR_PID" --sleep-interval=0.2 -n 80 -F "$LOG_FILE" &
TAIL_PID=$!
wait "$TAIL_PID" 2>/dev/null || true

echo
echo "[$COMPONENT_LABEL] Owner stopped. This window may now be closed."
while :; do
  sleep 60
done
