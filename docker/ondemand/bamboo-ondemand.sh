#!/usr/bin/env bash
# bamboo-ondemand.sh -- demarre / arrete les GROUPES de services demandes par les conteneurs.
#
# POURQUOI CE DETOUR (option B, 2026-10-11) : choisir le mode NAV a la manette doit lancer le
# lidar et la navigation, et les arreter quand on en sort. Le gestionnaire de modes vit dans
# un conteneur ; lui donner le socket Docker reviendrait a lui donner le RPi entier (root).
# Il se contente donc de DEPOSER une demande, et l'hote l'execute ici, a sa facon :
#   /run/bamboo-ondemand/<groupe>.want   `on` ou `off`, ecrit par le conteneur (atomique)
#   /run/bamboo-ondemand/<groupe>.state  `starting|running|stopping|stopped|error <raison>`
#                                        + horodatage, ecrit ici, lu par le conteneur
# Les groupes et leurs services viennent de groups.conf, liste FERMEE : rien d'autre ne peut
# etre demarre par ce chemin. /run est un tmpfs : au boot, aucune demande, donc rien ne part.
#
# Lance par bamboo-ondemand.path (systemd) a chaque changement du dossier. Une demande qui
# arrive PENDANT un `up` (~10 s) ne relancerait pas l'unite deja active : on reboucle donc
# tant que les demandes changent.
#   journalctl -u bamboo-ondemand     ls -l /run/bamboo-ondemand
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"        # .../docker/ondemand
COMPOSE_DIR="$(dirname "$HERE")"                             # .../docker
CONF="$HERE/groups.conf"
DIR="${BAMBOO_ONDEMAND_DIR:-/run/bamboo-ondemand}"
# Nom du projet compose (prefixe des conteneurs), lu du .env ; `tr -d '\r'` : .env en CRLF.
PROJECT="$(grep -E '^COMPOSE_PROJECT_NAME=' "$COMPOSE_DIR/.env" 2>/dev/null | tail -1 \
    | cut -d= -f2 | tr -d '\r[:space:]')"
PROJECT="${PROJECT:-bamboov4_humble}"

log() { echo "bamboo-ondemand: $*"; }

setState() { # $1 groupe, $2... etat
    # N'ECRIT QUE SI L'ETAT CHANGE : le fichier est dans le dossier que surveille
    # bamboo-ondemand.path, et chaque ecriture relancerait ce script -- qui reecrirait le meme
    # etat, sans fin. En regime etabli, aucune ecriture, donc aucune relance.
    local g="$1"; shift
    local cur
    cur="$(sed -E 's/ [0-9]+$//' "$DIR/$g.state" 2>/dev/null)"
    [ "$cur" = "$*" ] && return 0
    printf '%s %s\n' "$*" "$(date +%s)" > "$DIR/.$g.state.tmp" && mv -f "$DIR/.$g.state.tmp" "$DIR/$g.state"
}

running() { # le conteneur du service tourne-t-il ? (nom de conteneur : pas de profil a deviner)
    [ "$(docker inspect -f '{{.State.Running}}' "${PROJECT}-$1-1" 2>/dev/null)" = "true" ]
}

applyGroup() { # $1 groupe, $2 voulu (on|off), $3... services
    local g="$1" want="$2"; shift 2
    local s missing=() present=()
    for s in "$@"; do
        if running "$s"; then present+=("$s"); else missing+=("$s"); fi
    done
    if [ "$want" = "on" ]; then
        if [ "${#missing[@]}" -eq 0 ]; then setState "$g" running; return; fi
        setState "$g" starting
        log "$g : demarrage de ${missing[*]}"
        # Services NOMMES : nommer un service active son profil (jamais `--profile`, additif).
        # `--no-build` : une demande ne construit JAMAIS d'image.
        if (cd "$COMPOSE_DIR" && docker compose up -d --no-build "${missing[@]}"); then
            setState "$g" running
        else
            setState "$g" error "docker compose up a echoue"
        fi
    else
        if [ "${#present[@]}" -eq 0 ]; then setState "$g" stopped; return; fi
        setState "$g" stopping
        log "$g : arret de ${present[*]}"
        # `docker stop` par NOM de conteneur : `compose stop` ignore un service dont le profil
        # n'est pas actif ("no such service"). Jamais `down` ni `rm` : on garde les conteneurs.
        local names=()
        for s in "${present[@]}"; do names+=("${PROJECT}-$s-1"); done
        if docker stop "${names[@]}" >/dev/null; then
            setState "$g" stopped
        else
            setState "$g" error "docker stop a echoue"
        fi
    fi
}

pass() {
    local g svcs want
    while read -r g svcs; do
        [ -z "$g" ] || [ "${g#\#}" != "$g" ] && continue
        want="$(head -c 8 "$DIR/$g.want" 2>/dev/null | tr -d '[:space:]')"
        case "$want" in
            on|off) ;;
            "") continue ;;                               # aucune demande pour ce groupe
            *) setState "$g" error "demande inconnue : $want"; continue ;;
        esac
        # shellcheck disable=SC2086
        applyGroup "$g" "$want" $svcs
    done < <(sed -e 's/#.*//' "$CONF")
}

[ -d "$DIR" ] || { log "$DIR absent : rien a faire"; exit 0; }
[ -r "$CONF" ] || { log "$CONF illisible"; exit 1; }
for _ in 1 2 3 4 5 6 7 8 9 10; do
    before="$(cat "$DIR"/*.want 2>/dev/null | md5sum)"
    pass
    after="$(cat "$DIR"/*.want 2>/dev/null | md5sum)"
    [ "$before" = "$after" ] && break                    # rien n'a change pendant ce tour
done
