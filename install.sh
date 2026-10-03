#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Install the PTP pothole collector on a new unit.

  git clone -b live_detection https://github.com/hudaterslab/pothole.git ~/Desktop/live_detection
  cd ~/Desktop/live_detection
  sudo ./install.sh --dry-run   # show what would change; changes nothing
  sudo ./install.sh             # install, start the PTP master and verify it
  sudo reboot                   # automatic login opens the collector terminal

Needs Ubuntu 22.04 Lubuntu (LXQt), the user hudaters, and a NUC whose LiDAR
port enp1s0 owns /dev/ptp0 and camera port enp2s0 owns /dev/ptp1. Connect the
LiDAR, the camera and the "porthole" SSD before running. Re-running is safe:
unchanged items are left alone and every replaced file is first copied to
~/ptp_pothole_archive/install_<time>/.

Options:
  --dry-run        report what would change without changing anything
  --skip-packages  skip apt and pip (for example an offline re-run)
EOF
}

APP_USER=hudaters
APP_HOME=/home/$APP_USER
APP_DIR=$APP_HOME/Desktop/live_detection
SERVICE=ptp-pothole-master.service
DEVICE_CONFIG=/etc/ptp-pothole/device.json

# The same on every unit: ptp_service.py and app/ptp.py use these names.
LIDAR_IF=enp1s0 LIDAR_PHC=ptp0 LIDAR_CON=ptp-pothole-lidar
LIDAR_ADDR=192.168.1.100/32 LIDAR_IP=192.168.1.201
CAMERA_IF=enp2s0 CAMERA_PHC=ptp1 CAMERA_CON=porthole-camera
CAMERA_ADDR=192.168.11.2/24 CAMERA_IP=192.168.11.10

SSD_MOUNT=/mnt/ssd
SSD_FSTAB="LABEL=porthole $SSD_MOUNT ext4 defaults,nofail,x-systemd.automount,x-systemd.device-timeout=10,x-gvfs-show,x-gvfs-name=SanDisk_Porthole 0 2"

APT_PACKAGES=(
  linuxptp chrony ethtool iputils-ping python3-pip python3-yaml python3-gi
  gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 gstreamer1.0-plugins-base
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-vaapi
  intel-media-va-driver libgl1 qterminal
)
# The versions the first unit collects with.
PIP_PACKAGES=(numpy==2.2.6 opencv-python==4.13.0.92)

NM_FIELDS=connection.interface-name,802-3-ethernet.mac-address,ipv4.method,ipv4.addresses,ipv4.routes,ipv4.never-default,ipv6.method,connection.autoconnect,connection.autoconnect-priority

SRC_DIR=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
DEPLOY=$SRC_DIR/deploy
BACKUP_DIR=$APP_HOME/ptp_pothole_archive/install_$(date +%Y%m%d_%H%M%S)
DRY_RUN=0
SKIP_PACKAGES=0
FILE_CHANGED=0
CHANGED=()
WARNINGS=()
MASTER_ID="" LIDAR_ID="" CAMERA_ID=""

say() { printf '[install] %s\n' "$*"; }
warn() {
  printf '[install] WARNING: %s\n' "$*" >&2
  WARNINGS+=("$*")
}
die() {
  printf '[install] ERROR: %s\n' "$*" >&2
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

as_user() {
  if [[ "$(id -un)" == "$APP_USER" ]]; then
    "$@"
  else
    runuser -u "$APP_USER" -- env HOME="$APP_HOME" "$@"
  fi
}

backup() {
  if (( ! DRY_RUN )); then
    mkdir -p -- "$BACKUP_DIR$(dirname -- "$1")"
    cp -a -- "$1" "$BACKUP_DIR$1"
  fi
}

# install_file SOURCE DEST MODE OWNER:GROUP LABEL -- sets FILE_CHANGED.
install_file() {
  local src=$1 dest=$2 mode=$3 owner=$4 label=$5 have
  FILE_CHANGED=0
  if [[ -e "$dest" && ! -r "$dest" ]]; then
    say "$label: $dest is readable only with sudo; not compared"
    return 0
  fi
  if [[ -r "$dest" ]] && cmp -s -- "$src" "$dest"; then
    have=$(stat -c '%a %U:%G' -- "$dest")
    if [[ "$have" == "$mode $owner" ]]; then
      say "$label: unchanged"
    else
      say "$label: same content; permissions $have -> $mode $owner"
      act chmod "$mode" -- "$dest"
      act chown "$owner" -- "$dest"
      CHANGED+=("$label (permissions)")
    fi
    return 0
  fi
  if [[ -e "$dest" ]]; then
    say "$label: update $dest"
    if (( DRY_RUN )); then
      diff -u -- "$dest" "$src" | sed 's/^/    /' | head -n 40 || true
    fi
    backup "$dest"
  else
    say "$label: create $dest"
  fi
  act install -D -m "$mode" -o "${owner%%:*}" -g "${owner##*:}" -- "$src" "$dest"
  FILE_CHANGED=1
  CHANGED+=("$label")
}

# ec:9f:0d:03:55:35 -> ec9f0d.fffe.035535, the EUI-64 clock identity that
# ptp4l and both sensors derive from their MAC address.
identity_from_mac() {
  local mac=${1,,}
  mac=${mac//:/}
  [[ "$mac" =~ ^[0-9a-f]{12}$ ]] || return 1
  printf '%s.fffe.%s\n' "${mac:0:6}" "${mac:6:6}"
}

neighbor_mac() {  # neighbor_mac INTERFACE IP
  ping -c 2 -W 1 -I "$1" "$2" >/dev/null 2>&1 || true
  ip -4 neigh show "$2" dev "$1" 2>/dev/null \
    | awk '{for (i = 1; i < NF; i++) if ($i == "lladdr") {print $(i + 1); exit}}' || true
}

code_identity() {  # code_identity lidar|camera -- the first unit's value in app/ptp.py
  PTP_POTHOLE_DEVICE_CONFIG=/nonexistent PYTHONDONTWRITEBYTECODE=1 \
    python3 - "$SRC_DIR/app" "$1" <<'PY'
import sys

sys.path.insert(0, sys.argv[1])
import ptp

print(ptp.SENSORS[sys.argv[2]][1])
PY
}

configured_identity() {  # configured_identity lidar|camera -- the value in DEVICE_CONFIG
  if [[ -r "$DEVICE_CONFIG" ]]; then
    python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["sensor_identities"][sys.argv[2]])' \
      "$DEVICE_CONFIG" "$1" 2>/dev/null || true
  fi
}

# Prints: clock_source ptp_profile domain transport status...
lidar_config() {
  python3 - "$LIDAR_IP" <<'PY'
import json
import sys
import urllib.request

url = "http://%s/pandar.cgi?action=get&object=lidar_config" % sys.argv[1]
body = json.load(urllib.request.urlopen(url, timeout=4)).get("Body", {})
ptp = json.loads(body.get("PTPConfig") or "{}")
print(body.get("ClockSource", "?"), body.get("PTPProfile", "?"), ptp.get("Domain", "?"),
      ptp.get("Network", "?"), body.get("PTPStatus", "?"))
PY
}

# The same request ptp_service.py repeats every minute (app/ptp.py enable_sensors).
lidar_set_clock_source_ptp() {
  python3 - "$LIDAR_IP" <<'PY'
import sys
import urllib.request

url = "http://%s/pandar.cgi?action=set&object=lidar&key=clock_source&value=1" % sys.argv[1]
print(urllib.request.urlopen(url, timeout=4).read().decode(errors="replace"))
PY
}

# Prints: portState grandmasterIdentity offsetFromMaster (needs root for pmc).
sensor_ptp() {  # sensor_ptp INTERFACE IDENTITY
  timeout 5 pmc -4 -i "$1" -b 0 "TARGET $2-1" "GET PORT_DATA_SET" "GET PARENT_DATA_SET" \
    "GET CURRENT_DATA_SET" 2>/dev/null \
    | awk '$1 == "portState" {s = $2} $1 == "grandmasterIdentity" {g = $2}
           $1 == "offsetFromMaster" {o = $2} END {print s, g, o}' || true
}

preflight() {
  if (( ! DRY_RUN && EUID != 0 )); then
    die "run with sudo: sudo ./install.sh"
  fi
  id "$APP_USER" >/dev/null 2>&1 || die "user $APP_USER does not exist"
  if [[ "$SRC_DIR" != "$APP_DIR" ]]; then
    if (( DRY_RUN )); then
      warn "running from $SRC_DIR; a real install must run from $APP_DIR"
    else
      die "clone the repository to $APP_DIR; the service, launcher and autostart use that path"
    fi
  fi
  local os pair ifname phc file
  os=$(. /etc/os-release && printf '%s %s' "${ID:-}" "${VERSION_ID:-}")
  [[ "$os" == "ubuntu 22.04" ]] || warn "made for Ubuntu 22.04; this is $os"
  for pair in "$LIDAR_IF:$LIDAR_PHC" "$CAMERA_IF:$CAMERA_PHC"; do
    ifname=${pair%%:*} phc=${pair##*:}
    [[ -d /sys/class/net/$ifname ]] || die "network port $ifname not found (see: ip -br link)"
    [[ -e /sys/class/net/$ifname/device/ptp/$phc ]] \
      || die "$ifname has no hardware clock /dev/$phc (see: ethtool -T $ifname); PTP needs $LIDAR_IF=$LIDAR_PHC and $CAMERA_IF=$CAMERA_PHC"
  done
  for file in ptp_service.py collect_data.py app/ptp.py resolve_recording_storage.sh \
    launch_component_terminal.sh run_component_foreground.sh; do
    [[ -e "$SRC_DIR/$file" ]] || die "missing $SRC_DIR/$file"
  done
  say "unit $(hostname): $LIDAR_IF $(<"/sys/class/net/$LIDAR_IF/address") -> /dev/$LIDAR_PHC," \
    "$CAMERA_IF $(<"/sys/class/net/$CAMERA_IF/address") -> /dev/$CAMERA_PHC"
}

install_packages() {
  if (( SKIP_PACKAGES )); then
    say "packages: skipped (--skip-packages)"
    return 0
  fi
  local missing=() pkg name want have
  for pkg in "${APT_PACKAGES[@]}"; do
    [[ "$(dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null || true)" == "install ok installed" ]] \
      || missing+=("$pkg")
  done
  if (( ${#missing[@]} )); then
    say "packages: apt install ${missing[*]}"
    act apt-get update
    act env DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing[@]}"
    CHANGED+=("apt packages")
  else
    say "packages: apt unchanged"
  fi
  # The collector runs as $APP_USER, which sees its user site-packages.
  missing=()
  for pkg in "${PIP_PACKAGES[@]}"; do
    name=${pkg%%==*} want=${pkg##*==}
    have=$(as_user python3 -m pip show "$name" 2>/dev/null | awk '/^Version:/ {print $2}' || true)
    [[ "$have" == "$want" ]] || missing+=("$pkg")
  done
  if (( ${#missing[@]} )); then
    say "packages: pip install --user ${missing[*]}"
    act as_user python3 -m pip install --user "${missing[@]}"
    CHANGED+=("pip packages")
  else
    say "packages: pip unchanged"
  fi
}

ensure_groups() {
  local group groups missing=()
  groups=" $(id -nG "$APP_USER") "
  for group in dialout video render; do
    if ! getent group "$group" >/dev/null; then
      warn "group $group does not exist"
    elif [[ "$groups" != *" $group "* ]]; then
      missing+=("$group")
    fi
  done
  if (( ${#missing[@]} )); then
    say "groups: add $APP_USER to ${missing[*]} (GPS serial port, hardware video decoding)"
    act usermod -aG "$(IFS=,; printf '%s' "${missing[*]}")" "$APP_USER"
    CHANGED+=("user groups")
  else
    say "groups: unchanged"
  fi
}

# PTP host side. ptp_service.py (run by the service below) writes the ptp4l
# configuration and starts ptp4l on both ports plus one phc2sys per port.
install_ptp_host() {
  install_file "$DEPLOY/70-ptp-pothole-clock-read.rules" \
    /etc/udev/rules.d/70-ptp-pothole-clock-read.rules 644 root:root \
    "PTP: collector read access to /dev/$LIDAR_PHC and /dev/$CAMERA_PHC"
  if (( FILE_CHANGED )); then
    act udevadm control --reload
    act udevadm trigger --subsystem-match=ptp --action=change
  fi
  install_file "$DEPLOY/chrony.conf" /etc/chrony/chrony.conf 644 root:root \
    "PTP: chrony with bounded NTP corrections"
  if (( FILE_CHANGED )); then
    act systemctl restart chrony
  fi
  if [[ "$(systemctl is-enabled chrony 2>/dev/null || true)" != enabled ]]; then
    act systemctl enable --now chrony
    CHANGED+=("chrony enabled")
  fi
  if [[ "$(systemctl is-enabled systemd-timesyncd 2>/dev/null || true)" != masked ]]; then
    say "PTP: mask systemd-timesyncd so only chrony steers the system clock"
    act systemctl mask --now systemd-timesyncd
    CHANGED+=("systemd-timesyncd masked")
  fi
  install_file "$DEPLOY/$SERVICE" "/etc/systemd/system/$SERVICE" 644 root:root \
    "PTP: grandmaster service"
  if (( FILE_CHANGED )); then
    act systemctl daemon-reload
  fi
  if [[ "$(systemctl is-enabled "$SERVICE" 2>/dev/null || true)" != enabled ]]; then
    act systemctl enable "$SERVICE"
    CHANGED+=("PTP service enabled")
  fi
}

install_collector_host() {
  install_file "$DEPLOY/99-porthole.conf" /etc/sysctl.d/99-porthole.conf 644 root:root \
    "LiDAR socket buffer (sysctl)"
  if (( FILE_CHANGED )); then
    act sysctl -q -p /etc/sysctl.d/99-porthole.conf
  fi
  install_file "$DEPLOY/ptp-pothole-collector-foreground" \
    /usr/local/sbin/ptp-pothole-collector-foreground 755 root:root "collector launcher"
  visudo -c -q -f "$DEPLOY/ptp-pothole-foreground.sudoers" \
    || die "deploy/ptp-pothole-foreground.sudoers is not valid sudoers syntax"
  install_file "$DEPLOY/ptp-pothole-foreground.sudoers" /etc/sudoers.d/ptp-pothole-foreground \
    440 root:root "sudo rule for the launcher"
  if [[ ! -d "$APP_HOME/.config/autostart" ]]; then
    act as_user mkdir -p "$APP_HOME/.config/autostart"
  fi
  install_file "$DEPLOY/porthole-collector-terminal.desktop" \
    "$APP_HOME/.config/autostart/porthole-collector-terminal.desktop" 644 "$APP_USER:$APP_USER" \
    "collector terminal autostart"
}

# live_pothole.py reads the upload server from .env next to it (see .env.example).
ensure_env_file() {
  local env=$APP_DIR/.env
  if [[ -e "$env" ]]; then
    say "upload settings: $env exists"
  else
    say "upload settings: create $env from .env.example"
    act install -m 600 -o "$APP_USER" -g "$APP_USER" -- "$SRC_DIR/.env.example" "$env"
    CHANGED+=(".env from .env.example")
  fi
  if ! grep -qE '^(PORTHOLE_API_URL|PORTHOLE_UPLOAD_HOST)=[^[:space:]]' "$env" 2>/dev/null; then
    warn "fill in the upload settings in $env (PORTHOLE_API_URL, or PORTHOLE_UPLOAD_HOST, _USER, _DIR, _KEY); live_pothole.py does not upload until then"
  fi
}

configure_autologin() {
  local session tmp
  if [[ -e /usr/share/xsessions/Lubuntu.desktop ]]; then
    session=Lubuntu
  elif [[ -e /usr/share/xsessions/lxqt.desktop ]]; then
    session=lxqt
  else
    warn "no LXQt session is installed; the collector terminal starts only in an LXQt login"
    return 0
  fi
  if ! command -v sddm >/dev/null; then
    warn "SDDM is not installed; set up automatic login for $APP_USER yourself"
    return 0
  fi
  tmp=$(mktemp)
  python3 - /etc/sddm.conf "$tmp" "$APP_USER" "$session" <<'PY'
import configparser
import os
import sys

source, target, user, session = sys.argv[1:]
conf = configparser.ConfigParser(interpolation=None)
conf.optionxform = str
if os.path.exists(source):
    conf.read(source)
if not conf.has_section("Autologin"):
    conf.add_section("Autologin")
conf["Autologin"]["User"] = user
conf["Autologin"]["Session"] = session
with open(target, "w") as handle:
    conf.write(handle, space_around_delimiters=False)
PY
  install_file "$tmp" /etc/sddm.conf 644 root:root "automatic login ($APP_USER, $session)"
  rm -f -- "$tmp"
}

configure_storage() {
  local current
  current=$(awk -v mount="$SSD_MOUNT" '$1 !~ /^#/ && $2 == mount {$1 = $1; print}' /etc/fstab)
  if [[ -z "$current" ]]; then
    say "SSD: add $SSD_MOUNT (ext4 label porthole, mounted on first access) to /etc/fstab"
    backup /etc/fstab
    if (( ! DRY_RUN )); then
      printf '%s\n' "$SSD_FSTAB" >>/etc/fstab
    fi
    CHANGED+=("fstab $SSD_MOUNT")
    if [[ ! -d "$SSD_MOUNT" ]]; then
      act mkdir -p "$SSD_MOUNT"
    fi
    act systemctl daemon-reload
    act systemctl start "$(systemd-escape -p --suffix=automount "$SSD_MOUNT")"
  elif [[ "$current" == "$SSD_FSTAB" ]]; then
    say "SSD: fstab unchanged"
  else
    warn "/etc/fstab already has another $SSD_MOUNT entry; left as is: $current"
  fi
  if [[ "$(lsblk -rno LABEL,FSTYPE 2>/dev/null)" != *"porthole ext4"* ]]; then
    warn "no ext4 partition labelled porthole is attached; label the SSD with: sudo e2label /dev/sdX1 porthole"
  fi
}

# nm_profile NAME INTERFACE ADDRESS ROUTE PRIORITY
nm_profile() {
  local name=$1 ifname=$2 address=$3 route=$4 priority=$5 mac want current state
  mac=$(tr '[:lower:]' '[:upper:]' <"/sys/class/net/$ifname/address")
  want=$(printf '%s\n' "connection.interface-name:$ifname" "802-3-ethernet.mac-address:$mac" \
    "ipv4.method:manual" "ipv4.addresses:$address" "ipv4.routes:$route" \
    "ipv4.never-default:yes" "ipv6.method:disabled" "connection.autoconnect:yes" \
    "connection.autoconnect-priority:$priority")
  current=$(LC_ALL=C nmcli -t -f "$NM_FIELDS" connection show "$name" 2>/dev/null || true)
  state=$(LC_ALL=C nmcli -g GENERAL.STATE connection show "$name" 2>/dev/null || true)
  if [[ "$current" == "$want" && "$state" == activated ]]; then
    say "network: $name unchanged ($ifname $address)"
    return 0
  fi
  say "network: configure $name on $ifname ($address${route:+, route $route})"
  CHANGED+=("network $name")
  if (( DRY_RUN )); then
    diff -u <(printf '%s\n' "$current") <(printf '%s\n' "$want") | sed 's/^/    /' || true
    return 0
  fi
  local settings=(
    connection.interface-name "$ifname" 802-3-ethernet.mac-address "$mac"
    ipv4.method manual ipv4.addresses "$address" ipv4.routes "$route" ipv4.never-default yes
    ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority "$priority"
  )
  if [[ -z "$current" ]]; then
    nmcli connection add type ethernet con-name "$name" "${settings[@]}" >/dev/null
  else
    nmcli connection modify "$name" "${settings[@]}"
  fi
  nmcli connection up "$name" >/dev/null \
    || warn "could not activate $name on $ifname (is the cable connected?)"
}

configure_network() {
  command -v nmcli >/dev/null || die "NetworkManager (nmcli) is required"
  nm_profile "$LIDAR_CON" "$LIDAR_IF" "$LIDAR_ADDR" "$LIDAR_IP/32" 100
  nm_profile "$CAMERA_CON" "$CAMERA_IF" "$CAMERA_ADDR" "" 200
}

# The collector accepts only this unit's grandmaster and these two sensors.
configure_ptp_identities() {
  local host_mac name ifname ip mac id tmp
  local -A ids macs
  host_mac=$(<"/sys/class/net/$LIDAR_IF/address")
  MASTER_ID=$(identity_from_mac "$host_mac")
  say "PTP: grandmaster identity $MASTER_ID (from $LIDAR_IF $host_mac)"
  for name in lidar camera; do
    if [[ $name == lidar ]]; then
      ifname=$LIDAR_IF ip=$LIDAR_IP
    else
      ifname=$CAMERA_IF ip=$CAMERA_IP
    fi
    mac=$(neighbor_mac "$ifname" "$ip")
    if [[ -n "$mac" ]] && id=$(identity_from_mac "$mac"); then
      say "PTP: $name identity $id (from $ip $mac)"
    else
      id=$(configured_identity "$name")
      if [[ -z "$id" ]]; then
        id=$(code_identity "$name")
      fi
      mac=unknown
      warn "$name $ip did not answer on $ifname; keeping identity $id. Connect it and run install.sh again."
    fi
    ids[$name]=$id
    macs[$name]=$mac
  done
  LIDAR_ID=${ids[lidar]} CAMERA_ID=${ids[camera]}
  tmp=$(mktemp)
  cat >"$tmp" <<EOF
{
  "master_identity": "$MASTER_ID",
  "sensor_identities": {
    "lidar": "$LIDAR_ID",
    "camera": "$CAMERA_ID"
  },
  "mac_addresses": {
    "$LIDAR_IF": "$host_mac",
    "lidar": "${macs[lidar]}",
    "camera": "${macs[camera]}"
  }
}
EOF
  install_file "$tmp" "$DEVICE_CONFIG" 644 root:root "PTP: identities of this unit and its sensors"
  rm -f -- "$tmp"
}

# Sensor side: the LiDAR must take its clock from PTP (1588v2, UDP/IPv4,
# domain 0, as the master advertises). The camera needs no stored setting;
# ptp_service.py restarts its PTP client through the camera SDK command.
configure_lidar_ptp() {
  local line clock profile domain network status
  if ! line=$(lidar_config 2>/dev/null); then
    warn "LiDAR $LIDAR_IP web API did not answer; check later that it uses PTP (1588v2, UDP/IPv4, domain 0)"
    return 0
  fi
  read -r clock profile domain network status <<<"$line"
  say "LiDAR PTP: clock source $clock, profile $profile, domain $domain, transport $network, status $status"
  if [[ "$clock" != 1 ]]; then
    say "LiDAR PTP: switch the clock source to PTP"
    act lidar_set_clock_source_ptp
    CHANGED+=("LiDAR clock source PTP")
  fi
  if [[ "$profile" != 0 || "$domain" != 0 || "$network" != 0 ]]; then
    warn "LiDAR PTP settings differ from the master (1588v2, domain 0, UDP/IPv4); set them at http://$LIDAR_IP"
  fi
}

start_ptp() {
  if pgrep -f "python3 -u $APP_DIR/collect_data.py" >/dev/null; then
    warn "the collector is running, so the PTP master was not restarted; after collecting run: sudo systemctl restart $SERVICE"
    return 1
  fi
  if ! ping -c 1 -W 1 "$CAMERA_IP" >/dev/null 2>&1; then
    warn "camera $CAMERA_IP does not answer; $SERVICE retries every 5 s until the camera is connected"
  fi
  local since out=""
  since=$(date '+%Y-%m-%d %H:%M:%S')
  say "PTP: restart $SERVICE"
  act systemctl restart "$SERVICE"
  if (( DRY_RUN )); then
    return 1
  fi
  say "PTP: waiting for the grandmaster (it first waits up to 60 s for NTP)"
  for _ in $(seq 120); do
    out=$(journalctl -u "$SERVICE" --since "$since" -o cat --no-pager 2>/dev/null || true)
    if [[ "$out" == *"master ready"* ]]; then
      say "PTP: grandmaster ready"
      return 0
    fi
    sleep 1
  done
  printf '%s\n' "$out" | tail -n 15 >&2
  warn "$SERVICE did not report ready within 120 s (its log is above)"
  return 1
}

verify_ptp() {
  local runtime=$APP_DIR/var/ptp_runtime.json prefix log last port phc name ifname id
  local state gm offset line status
  if [[ ! -r "$runtime" ]]; then
    warn "the PTP master has not run yet ($runtime is missing)"
    return 0
  fi
  prefix=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["log_prefix"])' "$runtime")
  log=$APP_DIR/var/logs/${prefix}_ptp4l.log
  for port in 1 2; do
    last=$(grep -E "port $port: [A-Z_]+ to [A-Z_]+" -- "$log" 2>/dev/null | tail -n 1 || true)
    if [[ "$last" == *" to MASTER "* ]]; then
      say "check: ptp4l port $port is MASTER"
    else
      warn "ptp4l port $port: ${last:-no state change logged} ($log)"
    fi
  done
  for phc in "$LIDAR_PHC" "$CAMERA_PHC"; do
    last=$(tail -n 1 -- "$APP_DIR/var/logs/${prefix}_$phc.log" 2>/dev/null || true)
    if [[ "$last" =~ offset[[:space:]]+(-?[0-9]+)[[:space:]]+s2[[:space:]].*delay[[:space:]]+0$ ]]; then
      say "check: /dev/$phc follows the system clock (offset ${BASH_REMATCH[1]} ns, precise cross-timestamps)"
    else
      warn "phc2sys /dev/$phc: '$last' (expected servo state s2 and delay 0; the collector needs PTP_SYS_OFFSET_PRECISE)"
    fi
  done
  if (( EUID != 0 )); then
    say "check: sensor PTP states need sudo; skipped"
  else
    for name in lidar camera; do
      if [[ $name == lidar ]]; then
        ifname=$LIDAR_IF id=$LIDAR_ID
      else
        ifname=$CAMERA_IF id=$CAMERA_ID
      fi
      state="" gm="" offset=""
      for _ in $(seq 30); do
        read -r state gm offset < <(sensor_ptp "$ifname" "$id") || true
        if [[ "$state" == SLAVE && "$gm" == "$MASTER_ID" ]]; then
          break
        fi
        sleep 2
      done
      if [[ "$state" == SLAVE && "$gm" == "$MASTER_ID" ]]; then
        say "check: $name $id is SLAVE to this unit, offset $offset ns"
      else
        warn "$name $id: PTP state '${state:-no answer}', grandmaster '${gm:-?}' (expected SLAVE to $MASTER_ID)"
      fi
    done
  fi
  status=""
  for _ in $(seq 15); do
    if line=$(lidar_config 2>/dev/null); then
      read -r _ _ _ _ status <<<"$line"
      if [[ "$status" == Locked* ]]; then
        break
      fi
    fi
    sleep 2
  done
  if [[ "$status" == Locked* ]]; then
    say "check: LiDAR reports PTP $status"
  else
    warn "LiDAR reports PTP '${status:-no answer}' (expected Locked)"
  fi
}

verify_storage() {
  local out
  if out=$(as_user "$SRC_DIR/resolve_recording_storage.sh" 2>&1); then
    say "check: recording storage ${out//$'\t'/ -> }"
  else
    warn "recording storage: $out"
  fi
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
    chown "$APP_USER:$APP_USER" "$APP_HOME/ptp_pothole_archive"
    chown -R "$APP_USER:$APP_USER" "$BACKUP_DIR"
    say "replaced files are backed up in $BACKUP_DIR"
  fi
  if (( ${#WARNINGS[@]} )); then
    say "warnings:"
    printf '  - %s\n' "${WARNINGS[@]}"
  fi
  say "next: sudo reboot; automatic login opens the collector terminal, which records after [PTP hh:mm:ss] READY"
  say "if the LiDAR and camera are mounted differently from the first unit, set camera_forward_offset_deg in config.yaml"
}

main() {
  while (( $# )); do
    case $1 in
      --dry-run) DRY_RUN=1 ;;
      --skip-packages) SKIP_PACKAGES=1 ;;
      -h | --help)
        usage
        exit 0
        ;;
      *) die "unknown option $1 (see --help)" ;;
    esac
    shift
  done
  preflight
  install_packages
  ensure_groups
  install_ptp_host
  install_collector_host
  ensure_env_file
  configure_autologin
  configure_storage
  configure_network
  configure_ptp_identities
  configure_lidar_ptp
  start_ptp || true
  verify_ptp
  verify_storage
  summary
}

main "$@"
