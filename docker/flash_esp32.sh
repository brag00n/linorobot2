#!/usr/bin/env bash
# flash_esp32.sh — flash du microcode ESP32 (General Driver) DEPUIS UN CONTENEUR.
#
# POURQUOI un conteneur : l'hote dietpi n'a ni esptool ni pip, et on ne veut RIEN y
# installer. Ce script tourne donc dans l'image ROS deja presente sur le RPi (service
# compose `flash.esp32`, `extends: base`) : aucune image supplementaire n'est construite,
# et esptool est installe dans la couche EPHEMERE du conteneur (`run --rm`), qui disparait
# a la sortie. Le seul ecrit durable est la sauvegarde du binaire actuel, dans le depot.
#
# Le flash ESP32 ne demande AUCUNE manipulation physique : le CP2102N pilote EN et BOOT0
# par DTR/RTS, esptool provoque lui-meme le reset et l'entree en bootloader (contrairement
# au STM32 de la lignee Yahboom, qui exige BOOT0+RESET a la main).
#
# Usage (dans le conteneur, lance par le service compose) :
#   bash /root/linorobot2_ws/src/linorobot2/docker/flash_esp32.sh [chemin/firmware.bin]
# Variables d'environnement :
#   ESP32_PORT    port serie (defaut /dev/esp32, repli /dev/ttyUSB0)
#   ESP32_BIN     image a ecrire (defaut $HW/firmware/esp32_bamboo/flash/firmware.bin)
#   ESP32_BAUD    debit de flash (defaut 460800 ; baisser a 115200 si echecs)
#   NO_BACKUP=1   saute la sauvegarde (deconseille : un mauvais flash immobilise le robot)
#
# PREALABLE : arreter le service qui tient le port, sinon esptool ne l'ouvrira pas :
#   docker compose stop driver.real
set -uo pipefail

HW=/root/linorobot2_ws/src/linorobot2_hardware
PORT="${ESP32_PORT:-/dev/esp32}"
BIN="${1:-${ESP32_BIN:-$HW/firmware/esp32_bamboo/flash/firmware.bin}}"
BAUD="${ESP32_BAUD:-460800}"
BACKUP_DIR="$HW/firmware/esp32_bamboo/flash/backup"

die() { echo "ERREUR: $*" >&2; exit 1; }

# --- port : le symlink udev est preferable mais pas garanti (regles non consolidees) ---
[ -e "$PORT" ] || { echo "AVERTISSEMENT: $PORT absent, repli sur /dev/ttyUSB0"; PORT=/dev/ttyUSB0; }
[ -e "$PORT" ] || die "aucun port serie ($PORT) : la carte est-elle branchee ?"
[ -r "$BIN" ]  || die "image introuvable : $BIN
  Pousser le binaire depuis le poste de developpement (il n'est PAS versionne) :
    scp .pio/build/bamboov3-wirshare_bamboo_mavlink/firmware.bin \
        dietpi@<rpi>:<base>/linorobot2_hardware/firmware/esp32_bamboo/flash/"

# --- esptool : dans le conteneur uniquement, apt d'abord (pas de pip dans l'image) ---
ESPTOOL=""
for c in esptool.py esptool; do command -v "$c" >/dev/null 2>&1 && { ESPTOOL="$c"; break; }; done
if [ -z "$ESPTOOL" ]; then
    echo "--- installation d'esptool DANS LE CONTENEUR (rien sur l'hote) ---"
    apt-get update -qq && apt-get install -y -qq esptool \
        || pip3 install --no-cache-dir esptool \
        || die "installation d'esptool impossible (reseau ?)"
    for c in esptool.py esptool; do command -v "$c" >/dev/null 2>&1 && { ESPTOOL="$c"; break; }; done
    [ -n "$ESPTOOL" ] || die "esptool installe mais introuvable dans le PATH"
fi

echo "=== carte sur $PORT, image $(basename "$BIN") ($(stat -c%s "$BIN") octets) ==="
"$ESPTOOL" --port "$PORT" --baud 115200 chip_id || die "la carte ne repond pas sur $PORT"

# --- sauvegarde AVANT ecriture : c'est le seul chemin de retour arriere ---
if [ "${NO_BACKUP:-0}" != "1" ]; then
    mkdir -p "$BACKUP_DIR"
    OUT="$BACKUP_DIR/firmware_$(date +%Y%m%d_%H%M%S).bin"
    echo "=== sauvegarde de la flash actuelle -> $OUT ==="
    # 4 MiB = taille de la flash du WROOM-32 ; on relit tout, pas seulement 0x10000.
    "$ESPTOOL" --port "$PORT" --baud "$BAUD" read_flash 0 0x400000 "$OUT" \
        || die "sauvegarde echouee : on n'ecrit PAS sans filet"
fi

echo "=== ecriture ==="
"$ESPTOOL" --port "$PORT" --baud "$BAUD" write_flash -z 0x10000 "$BIN" \
    || die "ecriture echouee — reflasher la sauvegarde :
  $ESPTOOL --port $PORT --baud 115200 write_flash 0 $BACKUP_DIR/<sauvegarde>.bin"

echo "=== verification ==="
"$ESPTOOL" --port "$PORT" --baud "$BAUD" verify_flash 0x10000 "$BIN" \
    || echo "AVERTISSEMENT: verify_flash a echoue (a confirmer par la banniere STATUSTEXT)"

echo "=== fait. Relancer le driver : docker compose up -d driver.real ==="
echo "    puis verifier l'identite du microcode dans ses logs (STATUSTEXT)."
