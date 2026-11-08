#!/usr/bin/env bash
# bamboo-camwatch.sh -- surveille la camera et la relance quand elle se fige. Cote HOTE.
#
# POURQUOI (2026-11-08) : apres ~19 h de marche, la webcam USB a repondu "Failed to set UVC probe
# control : -110" (delai depasse). gscam_node est reste VIVANT mais ne recevait plus aucune image :
# le suivi, le HUD et le Command Center sont tombes ensemble, sans rien qui se relance. Le
# diagnostic voyait la panne ("aucune trame brute") sans pouvoir agir -- et il ne doit pas : lui
# donner Docker reviendrait a donner le RPi entier a un conteneur (meme raison que
# docker/ondemand). La recuperation vit donc ICI, sur l'hote.
#
# SONDE : un instantane du flux brut servi par web_video_server (`snapshot?topic=/video/raw`).
# ~40 ms et ~60 Ko quand la camera va bien ; sans image, la requete attend puis expire. Aucune
# dependance ROS sur l'hote, et l'instantane traverse la chaine entiere (camera -> gscam ->
# web_video_server), ce que ne ferait pas un simple test "le noeud existe".
#
# ESCALADE (une etape par passage du minuteur, toutes les 30 s) :
#   - echecs 1..FAILS-1       : on attend (un a-coup, un demarrage, une charge ne sont pas une panne)
#   - FAILS echecs de suite   : niveau 1 -> redemarrer le conteneur du service (gscam repart)
#   - encore en echec apres   : niveau 2 -> reinitialiser le port USB de la camera, puis redemarrer
#                               DESACTIVE PAR DEFAUT (BAMBOO_CAMWATCH_USB=0) : la camera 1-1.1 est
#                               sur un HUB USB qui porte aussi le pont serie de la carte ESP32
#                               (CP2102N, 1-1.2). ESSAYE le 2026-11-08, robot desarme : SANS
#                               EFFET (la camera revient sans /dev/video0, UVC -110) et il fige les
#                               lectures du bus plusieurs minutes. Seul un redemarrage du RPi a
#                               repare. A LAISSER A 0 ; un echec persistant est signale, pas traite.
#   - plus de MAX_PER_H reprises en 1 h : on ARRETE (abandon) et on le dit -- une camera morte ne
#     doit pas faire redemarrer le robot en boucle.
# GARDE-FOUS : pas d'action pendant les MIN_UPTIME premieres secondes du RPi, ni si le conteneur est
# arrete (arret volontaire), ni dans les MIN_AGE secondes qui suivent son demarrage, ni si le fichier
# $DIR/pause existe (maintenance : `touch /run/bamboo-camwatch/pause`).
#
# ETAT : $DIR/state.json (tmpfs), ecrit a chaque passage -- lisible par une page ou un diagnostic.
#   journalctl -u bamboo-camwatch      cat /run/bamboo-camwatch/state.json
# Essai sans risque : BAMBOO_CAMWATCH_DRY=1 (decrit sans agir) ; sonde forcee : BAMBOO_CAMWATCH_URL.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"            # .../docker/watchdog
COMPOSE_DIR="$(dirname "$HERE")"
DIR="${BAMBOO_CAMWATCH_DIR:-/run/bamboo-camwatch}"
URL="${BAMBOO_CAMWATCH_URL:-http://127.0.0.1:8080/snapshot?topic=/video/raw&type=ros_compressed}"
SERVICE="${BAMBOO_CAMWATCH_SERVICE:-bamboo_video}"
VIDPID="${BAMBOO_CAMWATCH_VIDPID:-05a3:9331}"                   # webcam "HD Web Camera" (repli si /dev/bamboocam a disparu)
FAILS="${BAMBOO_CAMWATCH_FAILS:-3}"
MAX_PER_H="${BAMBOO_CAMWATCH_MAX_PER_H:-3}"
MIN_UPTIME="${BAMBOO_CAMWATCH_MIN_UPTIME:-240}"
MIN_AGE="${BAMBOO_CAMWATCH_MIN_AGE:-90}"
MIN_BYTES="${BAMBOO_CAMWATCH_MIN_BYTES:-5000}"
DRY="${BAMBOO_CAMWATCH_DRY:-0}"
USB="${BAMBOO_CAMWATCH_USB:-0}"                                 # 1 = autoriser le niveau 2 (reset du port USB)
PROJECT="$(grep -E '^COMPOSE_PROJECT_NAME=' "$COMPOSE_DIR/.env" 2>/dev/null | tail -1 | cut -d= -f2 | tr -d '\r[:space:]')"
CONTAINER="${PROJECT:-bamboov4_humble}-${SERVICE}-1"

log() { echo "bamboo-camwatch: $*"; }
mkdir -p "$DIR"

# ---- etat persistant entre deux passages (fichier cle=valeur simple) ----
fails=0; level=0; last_action=""; last_action_ts=0; actions=""
[ -r "$DIR/state.env" ] && . "$DIR/state.env" 2>/dev/null
now="$(date +%s)"

save() { # $1 = statut en clair
    local recent="" t n=0
    for t in $actions; do [ $((now - t)) -lt 3600 ] && { recent="$recent $t"; n=$((n + 1)); }; done
    actions="${recent# }"
    printf 'fails=%s\nlevel=%s\nlast_action="%s"\nlast_action_ts=%s\nactions="%s"\n' \
        "$fails" "$level" "$last_action" "$last_action_ts" "$actions" > "$DIR/.state.env.tmp" \
        && mv -f "$DIR/.state.env.tmp" "$DIR/state.env"
    printf '{"ts":%s,"status":"%s","fails":%s,"level":%s,"last_action":"%s","last_action_ts":%s,"actions_1h":%s,"dry":%s}\n' \
        "$now" "$1" "$fails" "$level" "$last_action" "$last_action_ts" "$n" "$([ "$DRY" = 1 ] && echo true || echo false)" \
        > "$DIR/.state.json.tmp" && mv -f "$DIR/.state.json.tmp" "$DIR/state.json"
}
count_recent() { local t n=0; for t in $actions; do [ $((now - t)) -lt 3600 ] && n=$((n + 1)); done; echo "$n"; }

# ---- peripherique USB de la camera : /dev/bamboocam -> video -> interface -> peripherique ----
usb_dev() {
    local v d
    v="$(basename "$(readlink -f /dev/bamboocam 2>/dev/null)")"
    if [ -n "$v" ] && [ -e "/sys/class/video4linux/$v/device" ]; then
        d="$(readlink -f "/sys/class/video4linux/$v/device")"; dirname "$d"; return 0
    fi
    for d in /sys/bus/usb/devices/*; do
        [ "$(cat "$d/idVendor" 2>/dev/null):$(cat "$d/idProduct" 2>/dev/null)" = "$VIDPID" ] && { readlink -f "$d"; return 0; }
    done
    return 1
}
usb_reset() {
    local dev; dev="$(usb_dev)" || { log "niveau 2 : camera introuvable sur le bus USB (VID:PID $VIDPID)"; return 1; }
    [ -w "$dev/authorized" ] || { log "niveau 2 : $dev/authorized non inscriptible (droits root requis)"; return 1; }
    log "niveau 2 : reinitialisation du port USB $(basename "$dev")"
    [ "$DRY" = 1 ] && { log "DRY : n'agit pas"; return 0; }
    echo 0 > "$dev/authorized"; sleep 2; echo 1 > "$dev/authorized"
    local i; for i in 1 2 3 4 5 6 7 8 9 10; do [ -e /dev/bamboocam ] && return 0; sleep 1; done
    log "niveau 2 : /dev/bamboocam n'est pas revenu apres 10 s"; return 1
}
restart_service() {
    log "redemarrage de $CONTAINER"
    [ "$DRY" = 1 ] && { log "DRY : n'agit pas"; return 0; }
    docker restart "$CONTAINER" >/dev/null 2>&1 || { log "docker restart a echoue"; return 1; }
}

# ---- garde-fous ----
if [ -e "$DIR/pause" ]; then save "pause"; exit 0; fi
up="$(cut -d. -f1 /proc/uptime)"
if [ "$up" -lt "$MIN_UPTIME" ]; then fails=0; save "demarrage du RPi (${up}s < ${MIN_UPTIME}s)"; exit 0; fi
running="$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || echo absent)"
if [ "$running" != "true" ]; then fails=0; level=0; save "service arrete (volontaire ?)"; exit 0; fi
started="$(docker inspect -f '{{.State.StartedAt}}' "$CONTAINER" 2>/dev/null)"
st_ts="$(date -d "$started" +%s 2>/dev/null || echo 0)"
if [ "$st_ts" -gt 0 ] && [ $((now - st_ts)) -lt "$MIN_AGE" ]; then save "service recent ($((now - st_ts))s < ${MIN_AGE}s)"; exit 0; fi

# ---- sonde ----
res="$(curl -s -m 8 -o /dev/null -w '%{http_code} %{size_download}' "$URL" 2>/dev/null || true)"
code="${res%% *}"; size="${res##* }"; size="${size:-0}"
if [ "$code" = "200" ] && [ "${size%.*}" -ge "$MIN_BYTES" ] 2>/dev/null; then
    [ "$level" != 0 ] && log "camera revenue (niveau $level) : image de $size octets"
    fails=0; level=0; save "ok"; exit 0
fi

fails=$((fails + 1))
log "pas d'image ($fails/$FAILS) : code=${code:-?} octets=${size:-0}"
if [ "$fails" -lt "$FAILS" ]; then save "image absente ($fails/$FAILS)"; exit 0; fi

if [ "$(count_recent)" -ge "$MAX_PER_H" ]; then
    save "abandon : $MAX_PER_H reprises en 1 h sans succes -- intervention humaine"; log "ABANDON : $MAX_PER_H reprises en 1 h"; exit 0
fi

if [ "$level" -eq 0 ]; then
    last_action="redemarrage de $SERVICE"; level=1
    restart_service
elif [ "$USB" = "1" ]; then
    last_action="reinitialisation USB + redemarrage de $SERVICE"; level=2
    usb_reset; restart_service
else
    # Niveau 1 deja tente sans succes et le reset USB est desactive : on ne fait PLUS que redemarrer
    # le service (sans danger pour l'ESP32), et on le DIT -- c'est ce message que verra le Command
    # Center. Les reprises restent plafonnees par MAX_PER_H.
    last_action="redemarrage de $SERVICE (niveau 1 repete ; reset USB desactive)"; level=1
    log "niveau 1 sans effet ; reset USB desactive (BAMBOO_CAMWATCH_USB=0) : camera probablement a rebrancher"
    restart_service
fi
last_action_ts="$now"; actions="$actions $now"; fails=0
save "reprise niveau $level"
