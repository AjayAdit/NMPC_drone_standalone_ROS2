#!/bin/bash
# Shell in the Kilted image, with the host GPU (needs nvidia-container-toolkit) and the
# host X display, so the MuJoCo viewer opens on this screen. Flight results land in
# ./kilted_results on the host (mounted at /results).
#
#   docker/kilted/run.sh                          # interactive shell
#   docker/kilted/run.sh /scripts/fly_both.sh     # both 60 s flights
#
# /scripts is this folder from the HOST, so the flight script can change without a rebuild.
set -e
xhost +local:docker >/dev/null
mkdir -p "$PWD/kilted_results"
docker run --rm -it \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD/kilted_results":/results \
  -v "$(cd "$(dirname "$0")" && pwd)":/scripts:ro \
  nmpc-kilted "${@:-bash}"
