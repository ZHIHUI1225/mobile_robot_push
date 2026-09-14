#!/usr/bin/env bash
set -Eeuo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is empty; restart with ./scripts/start_gpu_dev.sh" >&2
  exit 1
fi
if [[ ! -x /opt/VirtualGL/bin/vglrun ]]; then
  echo "VirtualGL is unavailable; use --mode egl or start with the VirtualGL overlay" >&2
  exit 1
fi

exec /opt/VirtualGL/bin/vglrun -d egl python examples/view_scene.py "$@"
