#!/usr/bin/env bash
set -euo pipefail

# Resolve exactly one standardized external recording drive.  The drive may
# have a different UUID every time, but it must be an ext4 USB partition whose
# filesystem label is "porthole".  Never fall back to the internal system disk.

STORAGE_LABEL="${PORTHOLE_STORAGE_LABEL:-porthole}"
DEFAULT_MOUNT="${PORTHOLE_SAVE_MOUNT:-/mnt/ssd}"
RUNS_SUBDIR="${PORTHOLE_RUNS_SUBDIR:-porthole_runs}"
WAIT_SEC="${PORTHOLE_STORAGE_WAIT_SEC:-90}"

fail() {
  echo "[storage] ERROR: $*" >&2
  exit 1
}

candidate_is_usb() {
  local part="$1"
  local parent
  parent="$(lsblk -ndo PKNAME "$part" 2>/dev/null | head -n 1)"
  [[ -n "$parent" ]] || return 1
  [[ "$(lsblk -ndo TRAN "/dev/$parent" 2>/dev/null | tr -d ' ')" == "usb" ]]
}

find_candidates() {
  local name type fstype label
  while read -r name type fstype label; do
    [[ "$type" == "part" && "$fstype" == "ext4" && "$label" == "$STORAGE_LABEL" ]] \
      || continue
    candidate_is_usb "$name" || continue
    readlink -f -- "$name"
  done < <(lsblk -nrpo NAME,TYPE,FSTYPE,LABEL)
}

mounted_target_for() {
  local candidate="$1"
  local target fstype
  local -a targets=()
  # Query by source.  A systemd automount and its ext4 child share the same
  # target; a global findmnt listing can hide the child behind the autofs row.
  while read -r target fstype; do
    [[ "$fstype" == "ext4" ]] || continue
    targets+=("$target")
  done < <(findmnt -rn -S "$candidate" -o TARGET,FSTYPE 2>/dev/null || true)
  (( ${#targets[@]} )) || return 1
  # The drive can be mounted twice: once by fstab at $DEFAULT_MOUNT and again by
  # udisks under /media when someone opens it in the file manager.  Always
  # prefer the fstab mount -- the udisks one belongs to a desktop session and
  # disappears if that session ejects it or logs out mid-drive.
  for target in "${targets[@]}"; do
    [[ "$target" == "$DEFAULT_MOUNT" ]] && { printf '%s\n' "$target"; return 0; }
  done
  printf '%s\n' "${targets[0]}"
  return 0
}

mapfile -t CANDIDATES < <(find_candidates)
if (( ${#CANDIDATES[@]} == 0 )); then
  fail "no ext4 USB partition labelled '$STORAGE_LABEL'; refusing internal-disk fallback"
fi
if (( ${#CANDIDATES[@]} > 1 )); then
  printf '[storage] ERROR: multiple matching USB partitions; disconnect all but one:\n' >&2
  printf '  %s\n' "${CANDIDATES[@]}" >&2
  exit 1
fi

CANDIDATE="${CANDIDATES[0]}"
MOUNT_PATH="$(mounted_target_for "$CANDIDATE" || true)"

if [[ -z "$MOUNT_PATH" ]]; then
  # /mnt/ssd is provisioned through fstab with LABEL=porthole and
  # x-systemd.automount.  Touching it asks systemd to mount the real ext4 FS.
  for (( attempt=0; attempt<WAIT_SEC; attempt++ )); do
    timeout 3 stat "$DEFAULT_MOUNT" >/dev/null 2>&1 || true
    MOUNT_PATH="$(mounted_target_for "$CANDIDATE" || true)"
    [[ -n "$MOUNT_PATH" ]] && break
    sleep 1
  done
fi

[[ -n "$MOUNT_PATH" ]] \
  || fail "$CANDIDATE did not mount within ${WAIT_SEC}s (expected $DEFAULT_MOUNT)"

MOUNT_PATH="$(readlink -f -- "$MOUNT_PATH")"
SAVE_ROOT="$MOUNT_PATH/$RUNS_SUBDIR"
[[ -d "$SAVE_ROOT" ]] || mkdir -p -- "$SAVE_ROOT" \
  || fail "cannot create $SAVE_ROOT"
[[ -w "$SAVE_ROOT" ]] || fail "$SAVE_ROOT is not writable by $(id -un)"

# Emit exactly two tab-delimited fields for the foreground supervisor.
printf '%s\t%s\n' "$MOUNT_PATH" "$SAVE_ROOT"
