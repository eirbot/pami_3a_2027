#!/bin/bash
echo "Démarrage du robot..."

# Configuration de l'environnement pour PulseAudio/PipeWire
export XDG_RUNTIME_DIR=/run/user/$(id -u)

# Dossier du code (= /home/eirbot/Documents/robot_stk sur le robot), déduit de l'emplacement du script
ROBOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

"$ROBOT_DIR/.venv/bin/python" "$ROBOT_DIR/main_robot.py"