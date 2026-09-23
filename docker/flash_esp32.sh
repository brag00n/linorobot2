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

# --- esptool : dans le conteneur uniquement, en PREFERANT une version >= 4.x -----------
# Pourquoi ce soin : l'esptool 2.8 que fournit apt (focal/jammy) ne sait pas diagnostiquer.
# Face a une carte qui n'ecoute pas, elle ne dit que "Timed out waiting for packet header",
# alors que la 4.x nomme la panne ("Download mode successfully detected, but getting no sync
# reply: The serial TX path seems to be down."). C'est ce message qui a permis d'isoler une
# coupure materielle de la voie TX ; on ne se prive pas de ce diagnostic.
# Cache persistant : le volume nomme `esptool_cache` est monte ici (cf. docker-compose.yaml).
# `run --rm` jette la couche ephemere du conteneur, donc sans ce cache CHAQUE flash
# reinstallerait pip puis esptool (1 a 2 min et du reseau). On installe avec `pip --target`
# dans le volume et on l expose par PYTHONPATH : le premier flash paie l installation, les
# suivants demarrent aussitot. Rien sur le systeme de fichiers de l hote, rien dans le depot.
ESPTOOL_CACHE="${ESPTOOL_CACHE:-/opt/esptool}"
if mkdir -p "$ESPTOOL_CACHE" 2>/dev/null; then
    export PYTHONPATH="$ESPTOOL_CACHE${PYTHONPATH:+:$PYTHONPATH}"
    export PATH="$ESPTOOL_CACHE/bin:$PATH"
else
    echo "AVERTISSEMENT: $ESPTOOL_CACHE indisponible, installation non persistante"
    ESPTOOL_CACHE=""
fi

ESPTOOL=""
resolveEsptool() {
    for c in esptool.py esptool; do command -v "$c" >/dev/null 2>&1 && { ESPTOOL="$c"; return 0; }; done
    python3 -c 'import esptool' >/dev/null 2>&1 && { ESPTOOL="python3 -m esptool"; return 0; }
    return 1
}
# la 2.8 dit "esptool.py v2.8", la 4.x "esptool.py v4.7.0" : on prend le premier numero
# de version rencontre, d'ou qu'il vienne dans la sortie.
esptoolMajor() { $ESPTOOL version 2>/dev/null | grep -oE '[0-9]+[.][0-9]+' | head -n1 | cut -d. -f1; }

resolveEsptool || true
MAJ="$(esptoolMajor)"
# pas de version lisible = outil trop vieux ou casse : on installe.
if [ -z "$ESPTOOL" ] || [ -z "$MAJ" ] || [ "$MAJ" -lt 4 ]; then
    echo "--- installation d'esptool >= 4 DANS LE CONTENEUR (rien sur l'hote) ---"
    (command -v pip3 >/dev/null 2>&1 || { apt-get update -qq && apt-get install -y -qq python3-pip; }) \
        && python3 -m pip install --no-cache-dir --upgrade \
             ${ESPTOOL_CACHE:+--target "$ESPTOOL_CACHE"} esptool >/dev/null \
        && ESPTOOL="" && resolveEsptool \
        || { echo "AVERTISSEMENT: pip indisponible, repli sur l'esptool d'apt (diagnostic pauvre)"
             [ -n "$ESPTOOL" ] || { apt-get update -qq && apt-get install -y -qq esptool; resolveEsptool; }; }
    [ -n "$ESPTOOL" ] || die "installation d'esptool impossible (reseau ?)"
fi
echo "--- esptool utilise : $($ESPTOOL version 2>/dev/null | head -n1) ---"

echo "=== carte sur $PORT, image $(basename "$BIN") ($(stat -c%s "$BIN") octets) ==="
$ESPTOOL --port "$PORT" --baud 115200 chip_id || die "la carte ne repond pas sur $PORT"

# --- sauvegarde AVANT ecriture : c'est le seul chemin de retour arriere ---
if [ "${NO_BACKUP:-0}" != "1" ]; then
    mkdir -p "$BACKUP_DIR"
    OUT="$BACKUP_DIR/firmware_$(date +%Y%m%d_%H%M%S).bin"
    echo "=== sauvegarde de la flash actuelle -> $OUT ==="
    # 4 MiB = taille de la flash du WROOM-32 ; on relit tout, pas seulement 0x10000.
    $ESPTOOL --port "$PORT" --baud "$BAUD" read_flash 0 0x400000 "$OUT" \
        || die "sauvegarde echouee : on n'ecrit PAS sans filet"
fi

echo "=== ecriture ==="
$ESPTOOL --port "$PORT" --baud "$BAUD" write_flash -z 0x10000 "$BIN" \
    || die "ecriture echouee — reflasher la sauvegarde :
  $ESPTOOL --port $PORT --baud 115200 write_flash 0 $BACKUP_DIR/<sauvegarde>.bin"

echo "=== verification ==="
$ESPTOOL --port "$PORT" --baud "$BAUD" verify_flash 0x10000 "$BIN" \
    || echo "AVERTISSEMENT: verify_flash a echoue (a confirmer par la banniere STATUSTEXT)"

echo "=== fait. Relancer le driver : docker compose up -d driver.real ==="
echo "    puis verifier l'identite du microcode dans ses logs (STATUSTEXT)."
