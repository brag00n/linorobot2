#!/usr/bin/env bash
# secure.sh -- bascule HTTP/HTTPS du robot (2026-11-08).
#   secure.sh https   liste blanche rosbridge + :9090 lie a 127.0.0.1 : seul le proxy Caddy (:8443) est joignable
#   secure.sh http    comportement historique : :9090 ouvert au reseau, aucune restriction (REPLI en cas de probleme)
#   secure.sh         affiche le mode courant
# A lancer sur le RPi, dans docker/. Redemarre rosbridge (les clients se reconnectent seuls en ~2 s).
set -euo pipefail
cd "$(dirname "$0")"
mode="${1:-}"
cur=$(grep -E '^BAMBOO_SECURE=' .env 2>/dev/null | cut -d= -f2 || true)
if [ -z "$mode" ]; then echo "mode courant : ${cur:-http}"; exit 0; fi
case "$mode" in
  https) bind=127.0.0.1 ;;
  http)  bind=0.0.0.0 ;;
  *) echo "usage : $0 [https|http]" >&2; exit 2 ;;
esac
sed -i '/^BAMBOO_SECURE=/d;/^ROSBRIDGE_BIND=/d' .env
printf 'BAMBOO_SECURE=%s\nROSBRIDGE_BIND=%s\n' "$mode" "$bind" >> .env
# Le proxy reste TOUJOURS demarre : il est inoffensif sans la liste blanche et porte les outils MCP en wss.
# Si c'est Caddy qui fait defaut : `docker compose stop proxy`, puis ouvrir http://<hote>:8000/cc/ (mode http).
docker compose up -d proxy
docker compose up -d --force-recreate rosbridge
echo "mode $mode : rosbridge ecoute sur $bind:9090$( [ "$mode" = https ] && echo ' ; acces reseau par https://<hote>:8443/' )"
