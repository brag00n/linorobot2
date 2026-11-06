#!/bin/bash
# Point d'acces du robot : cree ap0, lance hostapd + dnsmasq (DHCP) + l'interface web RaspAP
# (lighttpd + PHP), pose le NAT vers wlan0. Pas de systemd : ce script est le superviseur.
#
# CONTRAINTE MATERIELLE (mesuree sur le RPi4, brcmfmac) : `iw phy` annonce
# {managed}<=1, {AP}<=1 avec #channels<=1 -> l'AP et le client de la box DOIVENT partager le
# MEME canal. L'AP suit donc wlan0 : le canal regle dans l'interface web est IGNORE, et un
# veilleur relance hostapd si la box change de canal. Sans box (wlan0 non associe) au demarrage,
# repli sur AP_CHANNEL_FALLBACK. 5 GHz obligatoire en pratique : la manette BT partage la puce
# avec le 2,4 GHz.
#
# CONFIGURATION : l'interface web ecrit /etc/hostapd/hostapd.conf et /etc/dnsmasq.d/090_*.conf
# (volume /state). Au PREMIER demarrage on les amorce depuis les variables ; ensuite l'interface
# fait foi (le mot de passe de ap.env n'est plus relu). A chaque (re)demarrage de hostapd on
# derive la config d'EXECUTION : celle de l'interface, avec interface/canal/mode imposes ici.
set -u

# Secrets : monte en lecture seule depuis docker/ap/secret/ (non versionne, cf. .gitignore).
if [ -f /secret/ap.env ]; then set -a; . /secret/ap.env; set +a; fi

STA_IF="${STA_IF:-wlan0}"
AP_IF="${AP_IF:-ap0}"
AP_SSID="${AP_SSID:-Bamboo}"
AP_PASS="${AP_PASS:-}"
AP_COUNTRY="${AP_COUNTRY:-FR}"
AP_CHANNEL_FALLBACK="${AP_CHANNEL_FALLBACK:-36}"   # 5 GHz non-DFS
AP_ADDR="${AP_ADDR:-10.3.141.1}"
AP_PREFIX="${AP_PREFIX:-24}"
AP_NET="${AP_NET:-10.3.141.0/24}"
AP_DHCP_RANGE="${AP_DHCP_RANGE:-10.3.141.50,10.3.141.150,255.255.255.0,12h}"
AP_DNS="${AP_DNS:-1.1.1.1,9.9.9.9}"                # servis aux clients (NAT vers la box)
AP_NAT="${AP_NAT:-1}"                              # 1 = internet des clients via wlan0
AP_WEB_PORT="${AP_WEB_PORT:-8081}"
AP_WEB_USER="${AP_WEB_USER:-admin}"
AP_WEB_PASS="${AP_WEB_PASS:-}"
STATE=/state
RUN=/run/bamboo-ap
CMD="$RUN/cmd"
HPID=""; DPID=""; WPID=""

log() { echo "[ap] $*"; }
die() { log "ERREUR : $*"; exit 1; }

mkdir -p "$RUN" "$CMD" /var/lib/misc \
    "$STATE/raspap/networking" "$STATE/raspap/backups" "$STATE/hostapd" "$STATE/dnsmasq.d"
for d in hostapd lighttpd openvpn system adblock plugins; do mkdir -p "$STATE/raspap/$d"; done
rm -f "$CMD"/*

# ---- amorcage de l'etat (premier demarrage seulement) ---------------------------------------
if [ ! -f "$STATE/hostapd/hostapd.conf" ]; then
    [ "${#AP_PASS}" -ge 8 ] || die "AP_PASS absent ou < 8 caracteres (docker/ap/secret/ap.env, cf. ap.env.example)"
    cat > "$STATE/hostapd/hostapd.conf" <<EOF
interface=$AP_IF
driver=nl80211
ctrl_interface=/var/run/hostapd
ctrl_interface_group=0
ssid=$AP_SSID
country_code=$AP_COUNTRY
hw_mode=a
channel=$AP_CHANNEL_FALLBACK
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
wpa_passphrase=$AP_PASS
EOF
    log "hostapd.conf amorce (SSID=$AP_SSID)"
fi
chown root:www-data "$STATE/hostapd/hostapd.conf"; chmod 640 "$STATE/hostapd/hostapd.conf"

if [ ! -f "$STATE/dnsmasq.d/090_$AP_IF.conf" ]; then
    cat > "$STATE/dnsmasq.d/090_$AP_IF.conf" <<EOF
interface=$AP_IF
dhcp-range=$AP_DHCP_RANGE
dhcp-option=option:router,$AP_ADDR
dhcp-option=option:dns-server,$AP_DNS
EOF
fi

[ -f "$STATE/raspap/networking/defaults.json" ] \
    || cp /var/www/html/config/defaults.json "$STATE/raspap/networking/defaults.json"

if [ ! -f "$STATE/raspap/raspap.auth" ]; then
    [ "${#AP_WEB_PASS}" -ge 8 ] || die "AP_WEB_PASS absent ou < 8 caracteres (docker/ap/secret/ap.env)"
    hash="$(php -r 'echo password_hash($argv[1], PASSWORD_BCRYPT);' -- "$AP_WEB_PASS")"
    [ -n "$hash" ] || die "calcul du hash du mot de passe web impossible (php)"
    printf '%s\n%s\n' "$AP_WEB_USER" "$hash" > "$STATE/raspap/raspap.auth"
    log "identifiants web amorces (utilisateur $AP_WEB_USER)"
fi
# Droits comme l'installeur amont : l'interface (www-data) ecrit hostapd.ini, defaults.json,
# raspap.auth ; le reste de /etc/raspap reste a root.
chown root:www-data "$STATE/raspap" "$STATE/raspap/networking"
chmod 1775 "$STATE/raspap" "$STATE/raspap/networking"
chown -R www-data:www-data "$STATE/raspap/backups"
chown www-data:www-data "$STATE/raspap/networking/defaults.json" "$STATE/raspap/raspap.auth"
chmod 600 "$STATE/raspap/raspap.auth"

# ---- helpers -------------------------------------------------------------------------------
sta_channel() {   # canal courant de wlan0, vide s'il n'est pas associe
    iw dev "$STA_IF" info 2>/dev/null | awk '/channel/ {print $2; exit}'
}

write_hostapd_conf() {   # $1 = canal. Config de l'interface web + interface/pilote/canal imposes.
    local ch="$1" mode=a
    [ "$ch" -lt 36 ] && mode=g
    {
        echo "interface=$AP_IF"; echo "driver=nl80211"; echo "hw_mode=$mode"; echo "channel=$ch"
        grep -Ev '^[[:space:]]*(interface|driver|hw_mode|channel)=' "$STATE/hostapd/hostapd.conf"
    } > "$RUN/hostapd.conf"
}

write_dnsmasq_conf() {   # DNS coupe (port=0) : DHCP seul ; le reste vient de l'interface web
    cat > "$RUN/dnsmasq.conf" <<EOF
bind-interfaces
except-interface=lo
port=0
dhcp-leasefile=/var/lib/misc/dnsmasq.leases
conf-dir=/etc/dnsmasq.d,*.conf
EOF
}

write_lighttpd_conf() {  # interface web liee a l'ADRESSE de ap0 : injoignable depuis la box
    # La config Debian definit deja server.port (ligne 14) et le REUTILISE plus loin
    # (`include_shell use-ipv6.pl + server.port`) : le retirer donne "Undefined config
    # variable", en ajouter un 2e donne "Duplicate config variable". On REMPLACE donc la
    # valeur sur place, et on retire le script use-ipv6 (inutile : on desactive l'IPv6).
    grep -Ev '^[[:space:]]*(server\.bind|server\.use-ipv6)[[:space:]]*=|use-ipv6\.pl' \
        /etc/lighttpd/lighttpd.conf.orig \
        | sed "s/^[[:space:]]*server\.port[[:space:]]*=.*/server.port = $AP_WEB_PORT/" \
        > /etc/lighttpd/lighttpd.conf
    cat >> /etc/lighttpd/lighttpd.conf <<EOF
server.use-ipv6 = "disable"
server.bind = "$AP_ADDR"
EOF
}

ipt() {   # pose une regle iptables si elle est absente (idempotent) : ipt <table> <chaine> <args...>
    local t="$1" c="$2"; shift 2
    iptables -t "$t" -C "$c" "$@" 2>/dev/null || iptables -t "$t" -I "$c" "$@"
}

nat_up() {
    [ "$AP_NAT" = "1" ] || { log "NAT desactive (AP_NAT=0)"; return; }
    [ "$(cat /proc/sys/net/ipv4/ip_forward)" = "1" ] \
        || log "ATTENTION : ip_forward=0 sur l'hote, pas de NAT (Docker l'active d'ordinaire)"
    ipt nat POSTROUTING -s "$AP_NET" -o "$STA_IF" -j MASQUERADE
    # Docker met FORWARD en DROP par defaut : on ouvre explicitement ap0 -> wlan0 et le retour.
    # Ordre d'insertion (-I = en tete) : on pose ACCEPT d'abord, puis les DROP passent devant.
    ipt filter FORWARD -i "$STA_IF" -o "$AP_IF" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
    ipt filter FORWARD -i "$AP_IF" -o "$STA_IF" -j ACCEPT
    # Les clients de l'AP ne voient QUE internet, pas le reseau local de la box. (Le trafic
    # vers le RPi LUI-MEME passe par INPUT, pas FORWARD : HUD et rosbridge restent joignables.)
    for net in 10.0.0.0/8 172.16.0.0/12 192.168.0.0/16; do
        ipt filter FORWARD -i "$AP_IF" -o "$STA_IF" -d "$net" -j DROP
    done
}

nat_down() {
    iptables -t nat -D POSTROUTING -s "$AP_NET" -o "$STA_IF" -j MASQUERADE 2>/dev/null
    iptables -D FORWARD -i "$STA_IF" -o "$AP_IF" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT 2>/dev/null
    iptables -D FORWARD -i "$AP_IF" -o "$STA_IF" -j ACCEPT 2>/dev/null
    for net in 10.0.0.0/8 172.16.0.0/12 192.168.0.0/16; do
        iptables -D FORWARD -i "$AP_IF" -o "$STA_IF" -d "$net" -j DROP 2>/dev/null
    done
}

stop_proc() {   # stop_proc <pid>
    [ -n "${1:-}" ] || return 0
    kill "$1" 2>/dev/null && wait "$1" 2>/dev/null
    return 0
}
alive() { [ -n "${1:-}" ] && kill -0 "$1" 2>/dev/null; }

start_dnsmasq() { write_dnsmasq_conf; dnsmasq -k -C "$RUN/dnsmasq.conf" & DPID=$!; }
# /run/lighttpd porte le socket FastCGI de PHP ; le paquet le cree via systemd (tmpfiles), donc
# ici il faut le faire a la main -- sans lui lighttpd meurt sur "spawning gw failed".
start_lighttpd() {
    mkdir -p /run/lighttpd && chown www-data:www-data /run/lighttpd
    write_lighttpd_conf
    lighttpd -D -f /etc/lighttpd/lighttpd.conf &
    WPID=$!
}

stop_all() {
    log "arret"
    stop_proc "$HPID"; stop_proc "$DPID"; stop_proc "$WPID"
    nat_down
    ip link set "$AP_IF" down 2>/dev/null
    iw dev "$AP_IF" del 2>/dev/null
    exit 0
}
trap stop_all TERM INT

# ---- attente de wlan0 (robot filaire ou Wi-Fi pas encore monte) ---------------------------
until ip link show "$STA_IF" >/dev/null 2>&1; do
    log "en attente de $STA_IF"; sleep 5
done

# ---- ap0 : interface virtuelle sur la meme puce que wlan0 ---------------------------------
if ! ip link show "$AP_IF" >/dev/null 2>&1; then
    iw dev "$STA_IF" interface add "$AP_IF" type __ap \
        || die "creation de $AP_IF impossible (mode AP+STA non supporte ?)"
    log "$AP_IF cree"
fi
ip addr flush dev "$AP_IF" 2>/dev/null
ip addr add "$AP_ADDR/$AP_PREFIX" dev "$AP_IF"
ip link set "$AP_IF" up

start_dnsmasq
nat_up
start_lighttpd
log "interface web : http://$AP_ADDR:$AP_WEB_PORT/ (depuis l'AP seulement)"

# ---- boucle de supervision (2 s) -----------------------------------------------------------
# Ordres de l'interface web : systemctl-shim.sh depose un fichier <verbe>.<unite> dans $CMD.
shopt -s nullglob
CUR=""; HOSTAPD_WANTED=1; FORCE_H=1; FORCE_D=0; tick=0
while true; do
    for f in "$CMD"/*; do
        order="$(basename "$f")"; rm -f "$f"
        case "$order" in
            stop.hostapd)                  HOSTAPD_WANTED=0; stop_proc "$HPID"; HPID="" ;;
            start.hostapd|restart.hostapd) HOSTAPD_WANTED=1; FORCE_H=1 ;;
            stop.dnsmasq)                  stop_proc "$DPID"; DPID="" ;;
            start.dnsmasq|restart.dnsmasq|reload.dnsmasq) FORCE_D=1 ;;
        esac
        log "ordre de l'interface web : $order"
    done

    if [ "$FORCE_D" = "1" ]; then
        stop_proc "$DPID"; start_dnsmasq; FORCE_D=0
    fi
    alive "$DPID" || { log "dnsmasq arrete, relance"; start_dnsmasq; }
    alive "$WPID" || { log "lighttpd arrete, relance"; start_lighttpd; }

    if [ "$HOSTAPD_WANTED" = "1" ]; then
        # Canal lu toutes les 10 s. wlan0 momentanement desassocie (itinerance) : on GARDE le
        # canal courant plutot que de basculer sur le repli, ce qui ferait osciller l'AP.
        if [ "$tick" -eq 0 ] || [ -z "$CUR" ]; then
            ch="$(sta_channel)"
            want="${ch:-${CUR:-$AP_CHANNEL_FALLBACK}}"
            if [ -n "$ch" ] && [ "$ch" -ge 52 ] && [ "$ch" -le 144 ]; then
                log "ATTENTION : la box est sur un canal DFS ($ch), comportement de l'AP non verifie"
            fi
        else
            want="$CUR"
        fi
        if [ "$FORCE_H" = "1" ] || [ "$want" != "$CUR" ] || ! alive "$HPID"; then
            stop_proc "$HPID"
            write_hostapd_conf "$want"
            log "hostapd : canal=$want ($([ -n "${ch:-}" ] && echo "suit $STA_IF" || echo repli))"
            hostapd "$RUN/hostapd.conf" &
            HPID=$!
            CUR="$want"; FORCE_H=0
        fi
    fi

    tick=$(( (tick + 1) % 5 ))
    sleep 2 & wait $!
done
