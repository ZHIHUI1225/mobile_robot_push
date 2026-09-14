#!/usr/bin/env bash
set -Eeuo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(docker compose -f "$repo_dir/compose.yaml" -f "$repo_dir/compose.gpu.yaml")

command -v docker >/dev/null || { echo "docker is required" >&2; exit 1; }
command -v nvidia-smi >/dev/null || { echo "nvidia-smi is required on the host" >&2; exit 1; }
docker info >/dev/null
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader

if [[ -z "${DISPLAY:-}" ]]; then
  x_socket="$(find /tmp/.X11-unix -maxdepth 1 -type s -user "$(id -u)" -name 'X*' -print 2>/dev/null | sort -V | tail -1)"
  if [[ -n "$x_socket" ]]; then
    export DISPLAY=":${x_socket##*X}"
  fi
fi

if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null && xhost +local:docker >/dev/null 2>&1; then
  echo "X11 enabled: DISPLAY=$DISPLAY, socket=/tmp/.X11-unix"
else
  echo "X11 display is unavailable; MuJoCo EGL rendering remains enabled"
fi

"${compose[@]}" up -d --build dev

container_id="$("${compose[@]}" ps -q dev)"
test -n "$container_id" || { echo "dev container did not start" >&2; exit 1; }

mount_source="$(docker inspect "$container_id" --format '{{range .Mounts}}{{if eq .Destination "/workspace"}}{{.Source}}{{end}}{{end}}')"
test "$mount_source" = "$repo_dir" || {
  echo "unexpected /workspace mount: $mount_source (expected $repo_dir)" >&2
  exit 1
}

"${compose[@]}" exec -T dev nvidia-smi -L
"${compose[@]}" exec -T dev python scripts/check_gpu_render.py

echo "Development container is ready: $container_id"
echo "Host source: $repo_dir"
echo "Container source: /workspace"
echo "Rendering: NVIDIA GPU + EGL${DISPLAY:+ + X11 DISPLAY=$DISPLAY}"
if [[ -n "${DISPLAY:-}" ]]; then
  echo "Interactive viewer: ./scripts/open_viewer.sh --map single_room"
fi
echo "Enter with: ${compose[*]} exec dev bash"

if [[ "${1:-}" == "--shell" ]]; then
  "${compose[@]}" exec dev bash
fi
