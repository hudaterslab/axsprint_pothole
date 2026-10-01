#!/usr/bin/env bash
# Keep main_live.py running for the analysis terminal (run_component_foreground.sh analysis).
#
# After an NPU device reset the DEEPX runtime ends the process itself ("This application
# must exit and restart to reload models"), and main_live.py cannot start while the DEEPX
# service is restarting. So an error exit is followed by a restart after 10 s, doubling up
# to 60 s while it keeps failing. main_live.py then analyses the frames recorded from its
# new start on; the ones recorded while it was down are skipped. Exit status 0 (finished)
# or a stop signal from the analysis terminal ends the loop.
set -u
cd "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" || exit 1

child=""
stop() {
  trap - TERM HUP INT
  if [[ -n "$child" ]]; then
    kill -TERM "$child" 2>/dev/null
    wait "$child"
  fi
  exit 0
}
trap stop TERM HUP INT

delay=10
while :; do
  started=$SECONDS
  /usr/bin/python3 -u main_live.py "$@" &
  child=$!
  wait "$child"
  status=$?
  child=""
  if (( status == 0 )); then
    exit 0
  fi
  if (( SECONDS - started >= 600 )); then
    delay=10
  fi
  echo "[Analysis V2] main_live.py exited with status $status; restarting in ${delay}s (it analyses new frames from then on)"
  # Sleep in the background so a stop signal is handled at once.
  sleep "$delay" &
  wait $!
  delay=$(( delay * 2 > 60 ? 60 : delay * 2 ))
done
