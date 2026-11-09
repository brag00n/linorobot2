#!/usr/bin/env bash
# bamboo-rbwatch.sh -- surveillance de rosbridge (2026-11-09). Minuteur systemd toutes les 30 s, COTE HOTE.
#
# POURQUOI. rosbridge est tombe TROIS FOIS en une journee, de deux facons differentes, toujours sans que rien ne le dise :
#   - boucle « WebSocketClosedError » (ecrit a l'infini vers une socket morte) ;
#   - derive : 1,16 Go de memoire et 75-88 % d'un coeur apres quelques heures, puis plus aucune reponse
#     (le CC affichait « hors ligne » alors que le conteneur etait « Up »), jusqu'a saturer tout le RPi (SSH muet).
# `restart: unless-stopped` ne sert a rien : le conteneur n'est pas MORT, il est MALADE. Un redemarrage le repare a chaque fois.
#
# CE QU'ON MESURE. Deux signes independants, pour ne pas redemarrer un rosbridge sain :
#   1. REPONSE : un appel de service leger (/rosapi/get_time) par un vrai websocket, avec delai. C'est ce que fait le CC.
#   2. MEMOIRE : au-dessus de MEM_MB (500 par defaut ; un rosbridge sain est a 110-160 Mo), c'est la derive.
# Redemarre apres FAILS passages mauvais de suite (reponse absente OU memoire excessive). Le pic de charge du demarrage du RPi
# (quand tout se lance en meme temps) ne declenche rien : MIN_UPTIME et MIN_AGE.
#
# GARDE-FOUS : jamais plus de MAX_PER_H redemarrages par heure (un rosbridge qui retombe aussitot est un autre probleme :
# on le dit au lieu de le relancer en boucle) ; rien dans les MIN_UPTIME premieres secondes du RPi, ni dans les MIN_AGE
# secondes apres le demarrage du conteneur, ni si le conteneur est arrete (volontaire), ni si $DIR/pause existe.
# ETAT : $DIR/state.json (tmpfs).  journalctl -u bamboo-rbwatch      cat /run/bamboo-rbwatch/state.json
# Essai sans risque : BAMBOO_RBWATCH_DRY=1 (decrit sans agir).
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"            # .../docker/watchdog
COMPOSE_DIR="$(dirname "$HERE")"
DIR="${BAMBOO_RBWATCH_DIR:-/run/bamboo-rbwatch}"
SERVICE="${BAMBOO_RBWATCH_SERVICE:-rosbridge}"
FAILS="${BAMBOO_RBWATCH_FAILS:-3}"
MEM_MB="${BAMBOO_RBWATCH_MEM_MB:-500}"
MAX_PER_H="${BAMBOO_RBWATCH_MAX_PER_H:-3}"
MIN_UPTIME="${BAMBOO_RBWATCH_MIN_UPTIME:-240}"
MIN_AGE="${BAMBOO_RBWATCH_MIN_AGE:-90}"
CALL_TIMEOUT="${BAMBOO_RBWATCH_TIMEOUT:-8}"
DRY="${BAMBOO_RBWATCH_DRY:-0}"
PORT="${BAMBOO_RBWATCH_PORT:-9090}"
PROJECT="$(grep -E '^COMPOSE_PROJECT_NAME=' "$COMPOSE_DIR/.env" 2>/dev/null | tail -1 | cut -d= -f2 | tr -d '\r[:space:]')"
CONTAINER="${PROJECT:-bamboov4_humble}-${SERVICE}-1"

log() { echo "bamboo-rbwatch: $*"; }
mkdir -p "$DIR"

fails=0; last_action=""; last_action_ts=0; actions=""; last_reason=""
[ -r "$DIR/state.env" ] && . "$DIR/state.env" 2>/dev/null
now="$(date +%s)"

save() { # $1 = statut en clair, $2 = memoire Mo, $3 = temps de reponse ms
    local recent="" t n=0
    for t in $actions; do [ $((now - t)) -lt 3600 ] && { recent="$recent $t"; n=$((n + 1)); }; done
    actions="${recent# }"
    printf 'fails=%s\nlast_action="%s"\nlast_action_ts=%s\nactions="%s"\nlast_reason="%s"\n' \
        "$fails" "$last_action" "$last_action_ts" "$actions" "$last_reason" > "$DIR/.state.env.tmp" && mv -f "$DIR/.state.env.tmp" "$DIR/state.env"
    printf '{"ts":%s,"status":"%s","fails":%s,"mem_mb":%s,"reply_ms":%s,"last_action":"%s","last_reason":"%s","last_action_ts":%s,"actions_1h":%s,"dry":%s}\n' \
        "$now" "$1" "$fails" "${2:-null}" "${3:-null}" "$last_action" "$last_reason" "$last_action_ts" "$n" "$([ "$DRY" = 1 ] && echo true || echo false)" \
        > "$DIR/.state.json.tmp" && mv -f "$DIR/.state.json.tmp" "$DIR/state.json"
}
count_recent() { local t n=0; for t in $actions; do [ $((now - t)) -lt 3600 ] && n=$((n + 1)); done; echo "$n"; }

# ---- garde-fous ----
up="$(cut -d. -f1 /proc/uptime 2>/dev/null || echo 0)"
if [ "$up" -lt "$MIN_UPTIME" ]; then fails=0; save "attente demarrage du RPi (${up}s)"; exit 0; fi
if [ -e "$DIR/pause" ]; then save "en pause (maintenance)"; exit 0; fi
running="$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || echo false)"
if [ "$running" != "true" ]; then fails=0; save "conteneur arrete (volontaire ?)"; exit 0; fi
started="$(docker inspect -f '{{.State.StartedAt}}' "$CONTAINER" 2>/dev/null)"
age=$(( now - $(date -d "$started" +%s 2>/dev/null || echo "$now") ))
if [ "$age" -lt "$MIN_AGE" ]; then fails=0; save "conteneur recent (${age}s)"; exit 0; fi

# ---- mesure 1 : memoire du conteneur ----
mem_mb="$(docker stats --no-stream --format '{{.MemUsage}}' "$CONTAINER" 2>/dev/null | cut -d/ -f1 | python3 -c "
import re,sys
s=sys.stdin.read().strip(); m=re.match(r'([0-9.]+)\s*([KMG]i?B)',s)
print(int(float(m.group(1))*{'KB':1/1024,'KIB':1/1024,'MB':1,'MIB':1,'GB':1024,'GIB':1024}[m.group(2).upper()]) if m else 0)" 2>/dev/null)"
mem_mb="${mem_mb:-0}"

# ---- mesure 2 : un vrai appel de service par websocket, avec delai (ce que fait le CC) ----
t0="$(date +%s%N)"
reply="$(timeout "$CALL_TIMEOUT" python3 - "$PORT" <<'PY' 2>/dev/null
import base64, json, os, socket, sys
s = socket.create_connection(("127.0.0.1", int(sys.argv[1])), 4)
s.send(("GET / HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n"
        % base64.b64encode(os.urandom(16)).decode()).encode())
s.recv(1024)
d = json.dumps({"op": "call_service", "service": "/rosapi/get_time", "id": "w", "args": {}}).encode()
mask = os.urandom(4)
s.send(bytes([0x81, 0x80 | len(d)]) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(d)))
s.settimeout(6)
print("ok" if b"service_response" in s.recv(65536) else "mauvaise-reponse")
PY
)"
reply_ms=$(( ($(date +%s%N) - t0) / 1000000 ))

# ---- decision ----
bad=""
[ "$reply" != "ok" ] && bad="pas de reponse du service /rosapi/get_time en ${CALL_TIMEOUT}s (${reply:-delai})"
[ "$mem_mb" -ge "$MEM_MB" ] && bad="${bad:+$bad ; }memoire ${mem_mb} Mo >= ${MEM_MB} Mo (derive)"

if [ -z "$bad" ]; then
    [ "$fails" -gt 0 ] && log "retabli (reponse ${reply_ms} ms, ${mem_mb} Mo)"
    fails=0; save "ok" "$mem_mb" "$reply_ms"; exit 0
fi

fails=$((fails + 1)); last_reason="$bad"
log "anomalie ${fails}/${FAILS} : $bad"
if [ "$fails" -lt "$FAILS" ]; then save "anomalie ${fails}/${FAILS}" "$mem_mb" "$reply_ms"; exit 0; fi

if [ "$(count_recent)" -ge "$MAX_PER_H" ]; then
    log "ABANDON : $(count_recent) redemarrages deja dans l'heure -- rosbridge retombe aussitot, ce n'est plus une panne passagere"
    save "abandon (plafond ${MAX_PER_H}/h atteint)" "$mem_mb" "$reply_ms"; exit 0
fi

last_action="redemarrage de $SERVICE"; last_action_ts="$now"; actions="$actions $now"; fails=0
if [ "$DRY" = 1 ]; then log "[DRY] redemarrerait $SERVICE : $bad"; save "dry: redemarrage evite" "$mem_mb" "$reply_ms"; exit 0; fi
log "REDEMARRAGE de $SERVICE : $bad"
# `docker restart <conteneur>` et NON `docker compose restart <service>` : le service est dans un profil (`boot`), que compose ne
# connait pas sans l'option --profile -> « no such service » (constate le 2026-11-09, la surveillance decidait de redemarrer mais ne faisait rien).
# Le redemarrage direct du conteneur n'a besoin d'aucun profil et garde la meme creation (volumes, ports, variables de .env).
docker restart "$CONTAINER" >/dev/null && save "redemarre" "$mem_mb" "$reply_ms" || save "echec du redemarrage" "$mem_mb" "$reply_ms"
