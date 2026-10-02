#!/usr/bin/env bash
# robot.sh -- choisir LE robot que ce RPi fait tourner : bambooSTM32YB (stm32) ou BambooWS (esp32).
#
# POURQUOI CE SCRIPT. Compose ne sait pas rendre un service conditionnel a la VALEUR d'une
# variable : `ROBOT=` dans .env choisissait la configuration des groupes communs, mais pas
# QUELS services demarrent, ni au boot (l'unite systemd nommait toujours driver.stm32). Basculer
# demandait donc cinq gestes a la main, dans le bon ordre. Ce script les fait, et dit ce qu'il
# va faire AVANT de le faire.
#
# LE MODELE, une seule regle : un service propre a un robot porte l'ETIQUETTE de ce robot comme
# profil compose (`stm32` ou `esp32`) ; un service commun n'en porte aucune et suit `${ROBOT}`.
# La table etiquette -> identifiant ROS vit dans docker/robots/<etiquette>.env, et nulle part
# ailleurs.
#
# Usage (sur le RPi, depuis n'importe ou) :
#   bash robot.sh status                 # robot de .env, services en marche, anomalies
#   sudo bash robot.sh use esp32         # AFFICHE ce qui serait fait, et s'arrete
#   sudo bash robot.sh use esp32 --yes   # le fait
#   bash robot.sh boot-list [etiquette]  # liste de boot d'un robot (sert a l'installateur)
#   sudo bash robot.sh install-unit      # repose bamboo4wd.service pour le robot de .env
#
# CE QUE `use` FAIT, dans cet ordre, et pourquoi cet ordre :
#   1. ARRETE (`docker stop`, jamais `down` ni `rm`) les services de l'AUTRE robot : d'abord
#      couper, pour ne jamais avoir deux drivers -- deux odometries -- en meme temps ;
#   2. ecrit ROBOT= et BAMBOO_CAM= dans .env (copie de sauvegarde .env.avant-bascule, une fois) ;
#   3. RECREE les services communs (`up -d --force-recreate`) : un `restart` ne relit PAS la
#      configuration, donc garderait l'ancien `robot:=` -- piege deja paye sur ce depot. C'est la
#      seule destruction de conteneur du script, et elle est la condition du changement : les
#      volumes de build et de visages, eux, persistent ;
#   4. demarre les services de boot du robot choisi ;
#   5. repose bamboo4wd.service avec SA liste de boot, pour qu'un reboot redemarre CE robot.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$HERE/docker-compose.yaml"
ENV_FILE="$HERE/.env"
ROBOTS_DIR="$HERE/robots"
UNIT_SRC="$HERE/systemd/bamboo4wd.service"
UNIT_DST="/etc/systemd/system/bamboo4wd.service"
TAGS="stm32 esp32"

die() { echo "ERREUR: $*" >&2; exit 2; }

# Valeur d'une variable d'un fichier KEY=VAL. `tr -d '\r'` n'est pas de la superstition : le
# .env du depot est en CRLF (edite sous Windows), et un \r final ferait echouer toute comparaison.
envget() { grep -E "^$2=" "$1" 2>/dev/null | tail -1 | cut -d= -f2- | tr -d '\r'; }

project() { local p; p="$(envget "$ENV_FILE" COMPOSE_PROJECT_NAME)"; echo "${p:-docker}"; }

# Table "service profils" lue DIRECTEMENT dans le compose, comme l'installateur le faisait : une
# cle de service est a 2 espaces, ses attributs a 4. `docker compose config --services` ne sert
# pas ici -- il masque les services profiles, ce qui est justement ce qu'on veut lister.
service_table() {
    awk '
        /^  [a-zA-Z0-9._-]+:[[:space:]]*$/ { if (svc != "") print svc "|" prof;
                                            svc = $1; sub(/:$/, "", svc); prof = ""; next }
        /^[^ #]/ { if (svc != "") print svc "|" prof; svc = ""; prof = ""; next }
        # Le commentaire de FIN DE LIGNE est retire en premier : plusieurs services en portent
        # un (profiles: ["boot"]  # cf. rosbridge), qui se colait sinon au dernier profil et
        # faisait disparaitre le service de la liste de boot. AUCUNE apostrophe ici : ce bloc
        # est entre apostrophes pour bash.
        /^    profiles:/ && svc != "" { prof = $0; sub(/#.*/, "", prof);
                                        sub(/^    profiles:[[:space:]]*/, "", prof);
                                        gsub(/[][" ]/, "", prof) }
        END { if (svc != "") print svc "|" prof }
    ' "$COMPOSE_FILE"
}

# Etiquette de robot d'un ensemble de profils ("boot,stm32" -> stm32), vide si commun.
tag_of() { local t; for t in $TAGS; do case ",$1," in *",$t,"*) echo "$t"; return;; esac; done; }

robot_services() { service_table | while IFS='|' read -r s p; do
    [ "$(tag_of "$p")" = "$1" ] && echo "$s"; done; }

# Liste de boot d'un robot : services `boot` communs + services `boot` de CE robot.
boot_list() { service_table | while IFS='|' read -r s p; do
    case ",$p," in *",boot,"*) ;; *) continue;; esac
    t="$(tag_of "$p")"; { [ -z "$t" ] || [ "$t" = "$1" ]; } && echo "$s"; done; }

# Services COMMUNS qui suivent ${ROBOT} : tout ce qui n'est pas etiquete et passe robot:=${ROBOT}.
common_robot_services() { service_table | while IFS='|' read -r s p; do
    [ -z "$(tag_of "$p")" ] || continue
    awk -v svc="$s" '
        $0 ~ "^  " svc ":[[:space:]]*$" { inside = 1; next }
        inside && /^  [a-zA-Z0-9._-]+:[[:space:]]*$/ { inside = 0 }
        inside && /robot:=\$\{ROBOT\}/ { found = 1 }
        END { exit found ? 0 : 1 }' "$COMPOSE_FILE" && echo "$s"; done; }

running_services() {
    docker ps --filter "label=com.docker.compose.project=$(project)" --filter status=running \
        --format '{{.Label "com.docker.compose.service"}}' 2>/dev/null | sort
}

container_of() {
    docker ps -a --filter "label=com.docker.compose.project=$(project)" \
        --filter "label=com.docker.compose.service=$1" --format '{{.Names}}' 2>/dev/null | head -1
}

current_tag() {
    local robot t; robot="$(envget "$ENV_FILE" ROBOT)"
    for t in $TAGS; do
        [ "$(envget "$ROBOTS_DIR/$t.env" ROBOT)" = "$robot" ] && { echo "$t"; return; }; done
}

# --------------------------------------------------------------------------- commandes
cmd_status() {
    local tag robot running
    robot="$(envget "$ENV_FILE" ROBOT)"; tag="$(current_tag)"
    running="$(running_services)"
    echo "robot de .env : ${robot:-<absent>} (${tag:-etiquette inconnue})"
    for t in $TAGS; do
        local mine on
        mine="$(robot_services "$t" | tr '\n' ' ')"
        on="$(for s in $mine; do echo "$running" | grep -qx "$s" && echo -n "$s "; done)"
        printf '  %-6s %-14s en marche : %s\n' "$t" "($(envget "$ROBOTS_DIR/$t.env" ROBOT_LABEL))" \
            "${on:-aucun}"
        if [ -n "$on" ] && [ "$t" != "$tag" ]; then
            echo "  ANOMALIE : des services de '$t' tournent alors que .env designe '$tag'" \
                 "-> sudo bash $0 use ${tag:-<etiquette>} --yes"
        fi
    done
    echo "  services communs suivant ROBOT : $(common_robot_services | tr '\n' ' ')"
    if [ -r "$UNIT_DST" ]; then
        echo "  unite de boot : $(grep -E '^ExecStart' "$UNIT_DST" | sed 's/.*--no-build //')"
    fi
}

write_unit() {
    local tag="$1" svcs label robot
    svcs="$(boot_list "$tag" | tr '\n' ' ')"; svcs="${svcs% }"
    [ -n "$svcs" ] || die "liste de boot vide pour '$tag' : unite non posee"
    label="$(envget "$ROBOTS_DIR/$tag.env" ROBOT_LABEL)"
    robot="$(envget "$ROBOTS_DIR/$tag.env" ROBOT)"
    sed -e "s#__COMPOSE_DIR__#$HERE#" -e "s#__BOOT_SERVICES__#$svcs#" \
        -e "s#__ROBOT_LABEL__#$label#" -e "s#__ROBOT__#$robot#" "$UNIT_SRC" > "$UNIT_DST" \
        || die "ecriture de $UNIT_DST impossible (sudo ?)"
    systemctl daemon-reload
    systemctl enable bamboo4wd.service >/dev/null 2>&1
    echo "  unite posee : $svcs"
}

# Remplace (ou ajoute) KEY=VAL dans .env en GARDANT le style de fin de ligne du fichier.
env_set() {
    local key="$1" val="$2" cr=""
    grep -q $'\r$' "$ENV_FILE" && cr=$'\r'
    if grep -qE "^$key=" "$ENV_FILE"; then
        sed -i -E "s#^$key=[^\r]*#$key=$val#" "$ENV_FILE"
    else
        printf '%s=%s%s\n' "$key" "$val" "$cr" >> "$ENV_FILE"
    fi
}

cmd_use() {
    local tag="${1:-}" yes="${2:-}"
    case " $TAGS " in *" $tag "*) ;; *) die "etiquette inconnue '$tag' (attendu : $TAGS)";; esac
    [ -r "$ROBOTS_DIR/$tag.env" ] || die "$ROBOTS_DIR/$tag.env absent"
    local robot cam label running others commons boots
    robot="$(envget "$ROBOTS_DIR/$tag.env" ROBOT)"
    cam="$(envget "$ROBOTS_DIR/$tag.env" BAMBOO_CAM)"
    label="$(envget "$ROBOTS_DIR/$tag.env" ROBOT_LABEL)"
    running="$(running_services)"
    others=""
    for t in $TAGS; do [ "$t" = "$tag" ] && continue
        for s in $(robot_services "$t"); do echo "$running" | grep -qx "$s" && others="$others $s"; done
    done
    # Communs a recreer : ceux qui suivent ROBOT et qui tournent, ou qui sont au boot.
    commons=""
    for s in $(common_robot_services); do
        if echo "$running" | grep -qx "$s" || boot_list "$tag" | grep -qx "$s"; then
            commons="$commons $s"; fi
    done
    boots="$(boot_list "$tag" | tr '\n' ' ')"

    echo "=== bascule vers $label ($robot) ==="
    echo "  1. arreter (sans detruire) :${others:- rien}"
    echo "  2. .env : ROBOT=$robot BAMBOO_CAM=$cam"
    echo "  3. recreer les services communs :${commons:- aucun}"
    echo "  4. demarrer la liste de boot : $boots"
    echo "  5. reposer $UNIT_DST"
    if [ "$yes" != "--yes" ]; then
        echo "(rien n'a ete fait : relancer avec --yes)"
        return 0
    fi
    [ "$(id -u)" = "0" ] || die "--yes demande les droits root (sudo)"

    local c
    for s in $others; do c="$(container_of "$s")"; [ -n "$c" ] && docker stop "$c" >/dev/null \
        && echo "  arrete : $s"; done
    [ -f "$ENV_FILE.avant-bascule" ] || cp "$ENV_FILE" "$ENV_FILE.avant-bascule"
    env_set ROBOT "$robot"
    env_set BAMBOO_CAM "$cam"
    echo "  .env : ROBOT=$(envget "$ENV_FILE" ROBOT) BAMBOO_CAM=$(envget "$ENV_FILE" BAMBOO_CAM)"
    cd "$HERE" || die "$HERE inaccessible"
    # Services NOMMES, jamais `--profile` : un profil est ADDITIF et ferait demarrer tout le
    # groupe `default` (incident du 2026-09-22). Nommer un service active son profil.
    if [ -n "$commons" ]; then
        # shellcheck disable=SC2086
        docker compose up -d --no-build --force-recreate $commons 2>&1 | grep -v DISPLAY
    fi
    # shellcheck disable=SC2086
    docker compose up -d --no-build $boots 2>&1 | grep -v DISPLAY
    write_unit "$tag"
    echo "=== $label actif ==="
}

case "${1:-}" in
    status) cmd_status ;;
    use) shift; cmd_use "${1:-}" "${2:-}" ;;
    boot-list) boot_list "${2:-$(current_tag)}" ;;
    install-unit) [ "$(id -u)" = "0" ] || die "droits root requis (sudo)"
                  t="$(current_tag)"; [ -n "$t" ] || die "ROBOT de .env ne correspond a aucun robots/*.env"
                  write_unit "$t" ;;
    *) sed -n '2,30p' "$0"; exit 1 ;;
esac
