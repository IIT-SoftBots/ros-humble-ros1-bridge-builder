#!/bin/bash
set -e

# 1️⃣ Se montato, sorgiamo il .bashrc dell'host
if [ -f /host_bashrc ]; then
  source /host_bashrc
fi

# 2️⃣ Usa ROBOT_NAME passato via -e o estrai da .bashrc
if [ -z "$ROBOT_NAME" ]; then
  ROBOT_NAME=$(grep -E '^export ROBOT_NAME=' /host_bashrc \
    | sed 's/^export ROBOT_NAME=//')
fi

# 3️⃣ Verifica ROBOT_NAME
if [ -z "$ROBOT_NAME" ]; then
  echo "❌ Errore: ROBOT_NAME non definito. Passa -e ROBOT_NAME o definiscilo in .bashrc."
  exit 1
fi

echo "✅ ROBOT_NAME = $ROBOT_NAME"