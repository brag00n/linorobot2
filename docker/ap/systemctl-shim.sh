#!/bin/bash
# Fausse commande `systemctl` du conteneur ap.rpi (pas de systemd ici).
#
# L'interface RaspAP ecrit hostapd.conf / 090_*.conf puis appelle `sudo systemctl restart
# hostapd` ou `dnsmasq`. Cette cale ne demarre rien elle-meme : elle depose un ordre dans
# /run/bamboo-ap/cmd/, que la boucle de entrypoint.sh consomme (et c'est elle qui regenere la
# config d'execution, avec le canal de wlan0). Toute autre unite est refusee.
CMD=/run/bamboo-ap/cmd

args=()
for a in "$@"; do
    case "$a" in -*) ;; *) args+=("$a") ;; esac
done
verb="${args[0]:-}"
unit="${args[1]:-}"
unit="${unit%.service}"
unit="${unit%%@*}"

case "$unit" in
    hostapd|dnsmasq) ;;
    *) echo "systemctl (cale Bamboo) : unite '${args[1]:-}' non geree" >&2; exit 5 ;;
esac

case "$verb" in
    start|stop|restart|reload)
        mkdir -p "$CMD" && touch "$CMD/$verb.$unit"
        exit $?
        ;;
    is-active|status)
        if pgrep -x "$unit" >/dev/null 2>&1; then
            echo active; exit 0
        fi
        echo inactive; exit 3
        ;;
    enable|disable|daemon-reload) exit 0 ;;
    *) echo "systemctl (cale Bamboo) : verbe '$verb' non gere" >&2; exit 1 ;;
esac
