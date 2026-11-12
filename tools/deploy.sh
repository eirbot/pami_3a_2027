#!/bin/bash
# Copie le code Rasp/ (et le firmware ESP/ dans Rasp/ESP) sur la carte du robot (rsync par ssh),
# en gardant sa config.json et ses journaux.
# Usage : tools/deploy.sh utilisateur@ip [dossier distant, défaut : ~/Rasp]
# Le driver LiDAR (LiDAR/lidar_udp) est recompilé sur la carte par main_robot.py au démarrage.
set -euo pipefail
[ $# -ge 1 ] || { echo "Usage : $0 utilisateur@ip [dossier distant]"; exit 1; }
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." >/dev/null 2>&1 && pwd )"
CIBLE="$1"
DOSSIER="${2:-Rasp}"
rsync -rc --out-format=%n \
    --exclude=.venv --exclude=__pycache__ --exclude='*.pyc' --exclude='*.log' \
    --exclude=config.json --exclude=/logs/ --exclude=LiDAR/lidar_udp \
    "$ROOT/Rasp/" "$CIBLE:$DOSSIER/"
# Firmware des ESP32 (compilé et flashé sur la carte par flash_esp.sh)
rsync -rc --out-format=%n --exclude=.pio --exclude=.vscode "$ROOT/ESP/" "$CIBLE:$DOSSIER/ESP/"
# Premier envoi : config.json du dépôt (ensuite, celle de la carte n'est jamais écrasée)
rsync --ignore-existing --out-format=%n "$ROOT/Rasp/config.json" "$CIBLE:$DOSSIER/config.json"
echo "Code copié dans $CIBLE:$DOSSIER (config.json de la carte conservée si elle existait)"
