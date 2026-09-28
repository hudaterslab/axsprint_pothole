#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
COMPONENT="${1:-}"

case "$COMPONENT" in
  collector|uploader|analysis)
    ;;
  *)
    echo "usage: $0 {collector|uploader|analysis}" >&2
    exit 2
    ;;
esac

# LXQt autostart is evaluated independently in every graphical login.  The
# machine has a local SDDM session on :0 and may also have one or more XRDP
# sessions on :10, :11, ... .  Starting the same autostart in XRDP would open
# duplicate terminals while the local-console collector/uploader still own
# their locks.
#
# Collection remains owned by the physical-console session.  XRDP opens an
# attached control terminal that follows the existing owner's log rather than
# attempting to acquire the lock and start a duplicate process.
if [[ "${XRDP_SESSION:-}" == "1" || "${DISPLAY:-}" != ":0" && "${DISPLAY:-}" != ":0.0" ]]; then
  if command -v logger >/dev/null 2>&1; then
    logger -t porthole-autostart \
      "attach component=$COMPONENT display=${DISPLAY:-unset} session=${XDG_SESSION_ID:-unset} xrdp=${XRDP_SESSION:-0}"
  fi
  exec /usr/bin/qterminal -e "$SCRIPT_DIR/attach_component_terminal.sh" "$COMPONENT"
fi

exec /usr/bin/env "PORTHOLE_FOREGROUND_COMPONENT=$COMPONENT" \
  /usr/bin/qterminal -e "$SCRIPT_DIR/run_component_foreground.sh"
