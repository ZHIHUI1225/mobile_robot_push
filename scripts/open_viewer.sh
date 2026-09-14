#!/usr/bin/env bash
set -Eeuo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is empty; restart with ./scripts/start_gpu_dev.sh" >&2
  exit 1
fi
# The remote X desktop is Mesa-backed. Keep GLFW on that GLX provider while
# NVIDIA remains available to training and headless MuJoCo EGL rendering.
export LIBGL_ALWAYS_SOFTWARE=1
export __GLX_VENDOR_LIBRARY_NAME=mesa
exec python examples/view_scene.py "$@"
