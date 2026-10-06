#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Start live pothole detection (live_pothole.py: camera model, every pothole checked
with the LiDAR) automatically at login, in its own
foreground terminal like the collector. Run it after install.sh:

  cd ~/Desktop/live_detection
  ./install_detection.sh --dry-run   # show what would change; changes nothing
  ./install_detection.sh             # check requirements, get models, enable autostart
  sudo reboot                        # automatic login opens the detection terminal

No sudo is needed. Closing the detection terminal stops only live_pothole.py; the
collector and the PTP master keep running. Re-running is safe; a replaced
autostart entry is first copied to ~/ptp_pothole_archive/install_detection_<time>/.

Options:
  --dry-run   report what would change without changing anything
  --disable   stop starting live_pothole.py at login (the entry is kept, Hidden=true)
EOF
}

APP_USER=hudaters
APP_HOME=/home/$APP_USER
APP_DIR=$APP_HOME/Desktop/live_detection
AUTOSTART=$APP_HOME/.config/autostart/porthole-analysis-terminal.desktop

# live_pothole.py reads these next to itself (camera: lens and camera pose; LiDAR: channel angles).
CALIBRATION=(camera_calib_best_effort.json XT32_Angle_Correction_File.csv)
UPLOAD_SETTINGS=(PORTHOLE_UPLOAD_HOST PORTHOLE_UPLOAD_USER PORTHOLE_UPLOAD_DIR PORTHOLE_UPLOAD_KEY)

SRC_DIR=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
BACKUP_DIR=$APP_HOME/ptp_pothole_archive/install_detection_$(date +%Y%m%d_%H%M%S)
DRY_RUN=0
DISABLE=0
CHANGED=()
WARNINGS=()

say() { printf '[detection] %s\n' "$*"; }
warn() {
  printf '[detection] WARNING: %s\n' "$*" >&2
  WARNINGS+=("$*")
}
die() {
  printf '[detection] ERROR: %s\n' "$*" >&2
  exit 1
}

# Run a command, or only show it with --dry-run.
act() {
  if (( DRY_RUN )); then
    printf '[dry-run] %s\n' "$*"
  else
    "$@"
  fi
}

# Files in the user's home must stay owned by the user, even under sudo.
as_user() {
  if [[ "$(id -un)" == "$APP_USER" ]]; then
    "$@"
  else
    runuser -u "$APP_USER" -- env HOME="$APP_HOME" "$@"
  fi
}

backup() {
  if (( ! DRY_RUN )); then
    as_user mkdir -p -- "$BACKUP_DIR"
    as_user cp -a -- "$1" "$BACKUP_DIR/"
  fi
}

# env_value NAME -- the value set in .env (last assignment wins, quotes removed).
env_value() {
  local value
  value=$(sed -n "s/^[[:space:]]*$1[[:space:]]*=//p" "$APP_DIR/.env" 2>/dev/null | tail -n 1 || true)
  value=${value#"${value%%[![:space:]]*}"}
  value=${value%"${value##*[![:space:]]}"}
  if [[ ${#value} -ge 2 && ( "$value" == \"*\" || "$value" == \'*\' ) ]]; then
    value=${value:1:-1}
  fi
  printf '%s' "$value"
}

preflight() {
  if (( EUID != 0 )) && [[ "$(id -un)" != "$APP_USER" ]]; then
    die "run as $APP_USER: ./install_detection.sh"
  fi
  if [[ "$SRC_DIR" != "$APP_DIR" ]]; then
    if (( DRY_RUN )); then
      warn "running from $SRC_DIR; a real install must run from $APP_DIR"
    else
      die "run from $APP_DIR; the autostart entry uses that path"
    fi
  fi
  local file
  for file in live_pothole.py run_live_pothole.sh download_models.sh launch_component_terminal.sh \
    run_component_foreground.sh deploy/porthole-analysis-terminal.desktop; do
    [[ -e "$SRC_DIR/$file" ]] || die "missing $SRC_DIR/$file"
  done
  if [[ ! -e "$APP_HOME/.config/autostart/porthole-collector-terminal.desktop" ]]; then
    warn "the collector is not installed yet; run sudo ./install.sh first (live_pothole.py analyses its recordings)"
  fi
}

# DEEPX NPU runtime and Python packages that live_pothole.py imports.
check_runtime() {
  local module
  for module in numpy cv2 simplejpeg; do
    if as_user python3 -c "import $module" >/dev/null 2>&1; then
      say "runtime: python $module ok"
    else
      warn "python module $module is missing (sudo ./install.sh installs it)"
    fi
  done
  if as_user python3 -c "import dx_engine" >/dev/null 2>&1; then
    say "runtime: DEEPX dx_engine $(as_user python3 -c 'import dx_engine; print(getattr(dx_engine, "__version__", "?"))' 2>/dev/null || true)"
  else
    warn "DEEPX runtime (python dx_engine) is not installed; install the DEEPX runtime (dx-runtime) first"
  fi
  if compgen -G "/dev/dxrt*" >/dev/null; then
    say "runtime: NPU device $(compgen -G "/dev/dxrt*" | tr '\n' ' ')"
  else
    warn "no DEEPX NPU device (/dev/dxrt*); check the NPU card and its driver"
  fi
  if [[ "$(systemctl is-active dxrt.service 2>/dev/null || true)" == active ]]; then
    say "runtime: dxrt.service active"
  else
    warn "dxrt.service is not active (DEEPX runtime service)"
  fi
}

# Models are not in git; download_models.sh fetches and verifies them.
check_models() {
  if as_user "$SRC_DIR/download_models.sh" --check; then
    return 0
  fi
  if (( DRY_RUN )); then
    printf '[dry-run] %s\n' "$SRC_DIR/download_models.sh"
    CHANGED+=("models")
  elif as_user "$SRC_DIR/download_models.sh"; then
    CHANGED+=("models")
  else
    warn "model download failed; run ./download_models.sh again or put best_seg.dxnn into $APP_DIR"
  fi
}

check_calibration() {
  local file
  for file in "${CALIBRATION[@]}"; do
    if [[ -e "$APP_DIR/$file" ]]; then
      say "calibration: $file present"
    else
      warn "calibration file $file is missing from $APP_DIR"
    fi
  done
}

# Uploads go to PORTHOLE_API_URL when it is set; otherwise over SSH, which needs
# the .env values, the SSH key and the server's host key. Without them
# live_pothole.py still analyses but does not upload.
check_upload() {
  local name missing=() host user key api
  if [[ ! -e "$APP_DIR/.env" ]]; then
    warn "no $APP_DIR/.env; copy .env.example to .env and fill in the upload settings"
    return 0
  fi
  api=$(env_value PORTHOLE_API_URL)
  if [[ -n "$api" ]]; then
    say "upload: .env sends to the API $api"
    return 0
  fi
  for name in "${UPLOAD_SETTINGS[@]}"; do
    [[ -n "$(env_value "$name")" ]] || missing+=("$name")
  done
  if (( ${#missing[@]} )); then
    warn "set PORTHOLE_API_URL, or ${missing[*]}, in $APP_DIR/.env; live_pothole.py does not upload until then"
    return 0
  fi
  host=$(env_value PORTHOLE_UPLOAD_HOST)
  user=$(env_value PORTHOLE_UPLOAD_USER)
  key=$(env_value PORTHOLE_UPLOAD_KEY)
  key=${key/#\~/$APP_HOME}
  say "upload: .env configured for $user@$host"
  if [[ ! -e "$key" ]]; then
    warn "upload SSH key $key does not exist"
  fi
  if ! as_user ssh-keygen -F "$host" >/dev/null 2>&1; then
    warn "$host is not in ~/.ssh/known_hosts; connect once as $APP_USER: ssh -i $key $user@$host true"
  fi
}

# The recordings themselves go to the server only with PORTHOLE_RAW_UPLOAD=true (default false).
check_raw_upload() {
  [[ -e "$APP_DIR/.env" ]] || return 0
  if [[ "$(env_value PORTHOLE_RAW_UPLOAD | tr '[:upper:]' '[:lower:]')" != true ]]; then
    say "raw upload: off (PORTHOLE_RAW_UPLOAD=true in .env sends the recordings)"
  elif [[ -n "$(env_value PORTHOLE_RAW_DIR)" ]]; then
    say "raw upload: on, recordings go to $(env_value PORTHOLE_RAW_DIR)"
  else
    warn "PORTHOLE_RAW_UPLOAD=true but PORTHOLE_RAW_DIR is empty in $APP_DIR/.env; recordings are not sent"
  fi
}

enable_autostart() {
  local src=$SRC_DIR/deploy/porthole-analysis-terminal.desktop
  if [[ -e "$AUTOSTART" ]] && cmp -s -- "$src" "$AUTOSTART"; then
    say "autostart: unchanged ($AUTOSTART)"
    return 0
  fi
  if [[ -e "$AUTOSTART" ]]; then
    say "autostart: replace $AUTOSTART"
    if (( DRY_RUN )); then
      diff -u -- "$AUTOSTART" "$src" | sed 's/^/    /' || true
    fi
    backup "$AUTOSTART"
  else
    say "autostart: create $AUTOSTART"
  fi
  act as_user mkdir -p -- "$(dirname -- "$AUTOSTART")"
  act as_user install -m 644 -- "$src" "$AUTOSTART"
  CHANGED+=("detection autostart")
}

disable_autostart() {
  if [[ ! -e "$AUTOSTART" ]]; then
    say "autostart: $AUTOSTART does not exist; nothing to disable"
    return 0
  fi
  if grep -qx 'Hidden=true' "$AUTOSTART"; then
    say "autostart: already disabled"
    return 0
  fi
  say "autostart: disable $AUTOSTART"
  backup "$AUTOSTART"
  act as_user sed -i -e 's/^Hidden=.*/Hidden=true/' \
    -e 's/^X-GNOME-Autostart-enabled=.*/X-GNOME-Autostart-enabled=false/' "$AUTOSTART"
  CHANGED+=("detection autostart disabled")
}

summary() {
  echo
  if (( DRY_RUN )); then
    say "dry run: nothing was changed"
  fi
  if (( ${#CHANGED[@]} )); then
    say "changed:"
    printf '  - %s\n' "${CHANGED[@]}"
  else
    say "changed: nothing"
  fi
  if [[ -d "$BACKUP_DIR" ]]; then
    say "the replaced autostart entry is backed up in $BACKUP_DIR"
  fi
  if (( ${#WARNINGS[@]} )); then
    say "warnings:"
    printf '  - %s\n' "${WARNINGS[@]}"
  fi
  if (( DISABLE )); then
    say "live_pothole.py no longer starts at login; a running detection terminal keeps running until closed"
  else
    say "next: sudo reboot; automatic login opens the collector and the detection terminals"
    say "start now from the desktop instead: $APP_DIR/launch_component_terminal.sh analysis"
  fi
}

main() {
  while (( $# )); do
    case $1 in
      --dry-run) DRY_RUN=1 ;;
      --disable) DISABLE=1 ;;
      -h | --help)
        usage
        exit 0
        ;;
      *) die "unknown option $1 (see --help)" ;;
    esac
    shift
  done
  preflight
  if (( DISABLE )); then
    disable_autostart
  else
    check_runtime
    check_models
    check_calibration
    check_upload
    check_raw_upload
    enable_autostart
  fi
  summary
}

main "$@"
