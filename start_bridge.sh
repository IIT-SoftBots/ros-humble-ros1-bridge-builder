#!/bin/bash

set -e

# Remove any existing container with same name (ignore errors)
docker rm -f "${ROBOT_NAME}_bridge" 2>/dev/null || true

# Run detached
docker run -d --name "${ROBOT_NAME}_bridge" --network host \
  --mount src="$(pwd)/config/parameter_bridge_template.yaml",target=/parameter_bridge_template.yaml,type=bind \
  --mount src=$HOME/.bashrc,target=/host_bashrc,type=bind \
  -e ROBOT_NAME="${ROBOT_NAME}" \
  ros-humble-ros1-bridge-builder:latest bash -lc "parameter_bridge" 2>&1