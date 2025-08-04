#!/bin/bash

set -e

# Avvia il container Docker con i mount e le variabili richieste
docker run -it --rm --network host --name ${ROBOT_NAME}_bridge \
  --mount src=./config/parameter_bridge_template.yaml,target=/parameter_bridge_template.yaml,type=bind \
  --mount src=$HOME/.bashrc,target=/host_bashrc,type=bind \
  -e ROBOT_NAME=$ROBOT_NAME \
  ros-humble-ros1-bridge-builder:latest bash -c "parameter_bridge"
