#!/bin/bash
# Compile et flashe un firmware ESP32 depuis la carte du robot (PlatformIO).
#
#   ./flash_esp.sh                       # firmware moteurs sur /dev/esp
#   ./flash_esp.sh moteurs /dev/ttyUSB1  # autre port
#   ./flash_esp.sh actionneurs           # firmware actionneurs sur /dev/esp
#   ./flash_esp.sh moteurs --compiler    # compile seulement (vérifie sans ESP branché)
#   ./flash_esp.sh --ports               # liste les ports série et leurs identifiants USB
#
# Le code du firmware est dans ESP/ (copié par tools/deploy.sh). PlatformIO est installé la
# première fois dans ~/.pio-venv ; le premier build télécharge la chaîne de compilation ESP32
# (plusieurs centaines de Mo dans ~/.platformio, quelques minutes), les suivants sont rapides.
# main_robot.py doit être arrêté (il garde le port de l'ESP ouvert) : ./stop.sh
set -euo pipefail

ICI="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJET="$ICI/ESP"
VENV="$HOME/.pio-venv"

lister_ports() {
    for p in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$p" ] || continue
        info=$(udevadm info -q property -n "$p" 2>/dev/null)
        id=$(echo "$info" | sed -n 's/^ID_VENDOR_ID=//p; s/^ID_MODEL_ID=//p' | paste -sd:)
        serie=$(echo "$info" | sed -n 's/^ID_SERIAL_SHORT=//p')
        liens=$(for l in /dev/lidar /dev/esp; do
                    [ "$(readlink -f "$l" 2>/dev/null)" = "$p" ] && echo -n "$l "; done)
        echo "$p  usb $id  série ${serie:-?}  ${liens:+<- $liens}"
    done
}

if [ "${1:-}" = "--ports" ]; then lister_ports; exit 0; fi

CIBLE="${1:-moteurs}"
case "$CIBLE" in
    moteurs)     ENV=Motor;       PORT_DEFAUT=/dev/esp ;;
    actionneurs) ENV=Actionneurs; PORT_DEFAUT=/dev/esp ;;
    *) echo "Cible inconnue : $CIBLE (moteurs ou actionneurs)"; exit 1 ;;
esac
COMPILER_SEUL=0
PORT="$PORT_DEFAUT"
for a in "${@:2}"; do
    if [ "$a" = "--compiler" ]; then COMPILER_SEUL=1; else PORT="$a"; fi
done

[ -f "$PROJET/platformio.ini" ] || { echo "Firmware absent : $PROJET (lancer tools/deploy.sh depuis le PC)"; exit 1; }

# PlatformIO : celui du système s'il existe, sinon un venv dédié (installé une fois)
if command -v pio >/dev/null 2>&1; then
    PIO=pio
else
    if [ ! -x "$VENV/bin/pio" ]; then
        echo "Installation de PlatformIO dans $VENV (une seule fois)..."
        python3 -m venv "$VENV" || { echo "python3-venv manquant : apt install -y python3-venv"; exit 1; }
        "$VENV/bin/pip" install -q --upgrade pip platformio
    fi
    PIO="$VENV/bin/pio"
fi

if [ $COMPILER_SEUL -eq 1 ]; then
    echo "Compilation du firmware $CIBLE (env $ENV)..."
    "$PIO" run -d "$PROJET" -e "$ENV"
    echo "Compilation OK : $PROJET/.pio/build/$ENV/firmware.bin"
    exit 0
fi

if [ ! -e "$PORT" ]; then
    echo "Port $PORT introuvable. Ports série présents :"
    lister_ports
    echo "Relancer avec le port : $0 $CIBLE /dev/ttyUSBx"
    exit 1
fi
if pgrep -f "[m]ain_robot.py" >/dev/null; then
    echo "main_robot.py tourne et garde le port de l'ESP : l'arrêter d'abord (./stop.sh)"
    exit 1
fi
if [ "$(readlink -f "$PORT")" = "$(readlink -f /dev/lidar 2>/dev/null)" ]; then
    echo "$PORT est le LiDAR, pas un ESP : choisir un autre port ($0 --ports)"
    exit 1
fi

echo "Compilation et flash du firmware $CIBLE (env $ENV) sur $PORT..."
"$PIO" run -d "$PROJET" -e "$ENV" -t upload --upload-port "$PORT"
echo "Flash terminé : l'ESP a redémarré sur le nouveau firmware."
