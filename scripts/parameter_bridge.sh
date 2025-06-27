#!/bin/bash
set -e
if [ -f /host_bashrc ]; then
  source /host_bashrc
fi

if [ -z "$ROBOT_NAME" ]; then
  echo "Errore: la variabile ROBOT_NAME non è impostata nel .bashrc dell'host."
  exit 1
fi

if [ ! -f /parameter_bridge_template.yaml ]; then
  echo "Errore: /parameter_bridge_template.yaml non trovato!"
  exit 1
fi



# Genera il file YAML parametrizzato
envsubst < /parameter_bridge_template.yaml > /parameter_bridge.yaml
echo "🔧 /parameter_bridge.yaml generato"

rosparam load /parameter_bridge.yaml
source /opt/ros/humble/setup.bash
source ros-humble-ros1-bridge/install/setup.bash
ros2 run ros1_bridge parameter_bridge