#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Download the model live_pothole.py runs (best_seg.dxnn) from Hugging Face into
this folder and verify its sha256. Models are not in git (.gitignore).

  ./download_models.sh           # download what is missing
  ./download_models.sh --check   # only report; exit 1 if a model is missing
  ./download_models.sh --force   # download again and replace the local copy

Set MODEL_BASE_URL=... to download from somewhere else.
EOF
}

MODEL_BASE_URL=${MODEL_BASE_URL:-https://huggingface.co/HudatersU/road_maintanance/resolve/main}
# sha256 of the published model; update it together with the file on Hugging Face.
declare -A MODELS=(
  [best_seg.dxnn]=2022ccec9c119d24fe4ea5c0352a64ee1f8afd711937bf4b0dbb552ea4ab1369
)

DIR=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
CHECK=0
FORCE=0

say() { printf '[models] %s\n' "$*"; }
die() {
  printf '[models] ERROR: %s\n' "$*" >&2
  exit 1
}

download() {  # download URL DEST
  if command -v wget >/dev/null; then
    wget -q --show-progress -O "$2" "$1"
  elif command -v curl >/dev/null; then
    curl -fL --progress-bar -o "$2" "$1"
  else
    die "wget or curl is required"
  fi
}

main() {
  while (( $# )); do
    case $1 in
      --check) CHECK=1 ;;
      --force) FORCE=1 ;;
      -h | --help)
        usage
        exit 0
        ;;
      *) die "unknown option $1 (see --help)" ;;
    esac
    shift
  done
  local name want have tmp failed=0
  for name in "${!MODELS[@]}"; do
    want=${MODELS[$name]}
    if [[ -e "$DIR/$name" ]] && (( ! FORCE )); then
      have=$(sha256sum -- "$DIR/$name" | cut -d' ' -f1)
      if [[ "$have" == "$want" ]]; then
        say "$name: up to date"
      else
        say "$name: differs from the published model; kept (--force replaces it)"
      fi
      continue
    fi
    if (( CHECK )); then
      say "$name: missing"
      failed=1
      continue
    fi
    tmp=$DIR/.$name.part
    say "$name: downloading from $MODEL_BASE_URL"
    if download "$MODEL_BASE_URL/$name" "$tmp" \
      && [[ "$(sha256sum -- "$tmp" | cut -d' ' -f1)" == "$want" ]]; then
      mv -f -- "$tmp" "$DIR/$name"
      say "$name: downloaded and verified"
    else
      rm -f -- "$tmp"
      say "$name: download failed or sha256 mismatch; the local copy is unchanged"
      failed=1
    fi
  done
  return "$failed"
}

main "$@"
