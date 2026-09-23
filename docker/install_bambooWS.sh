#!/usr/bin/env bash
# install_bambooWS.sh — provisionnement complet du robot BambooWS (bamboo4WD_V4_WSEsp32),
# depuis un RPi fraichement installe jusqu'a un robot qui demarre seul au boot.
#
# PRINCIPE : le fichier docker-compose.yaml est la SOURCE DE VERITE et porte tout ce qui est
# declaratif -- images, montages, peripheriques, ports, profil `boot`, `restart:
# unless-stopped`. Ce script ne fait donc QUE ce que Docker ne peut structurellement pas
# faire, et rien de plus :
#   - le multiplexage des broches du SoC (/boot/config.txt), decide par le firmware au boot ;
#   - l'installation du demon Docker lui-meme, et l'appartenance au groupe `docker` ;
#   - les regles udev, qui vivent dans le noyau de l'hote ;
#   - le service Bluetooth de l'hote (l'appairage est hote et interactif) ;
#   - l'unite systemd de rattrapage.
# Tout le reste est delegue a `docker compose`. Si une etape ci-dessous pouvait etre exprimee
# dans le compose, c'est qu'elle n'a pas sa place ici.
#
# Usage :
#   bash install_bambooWS.sh                 # DIAGNOSTIC SEUL : dit ce qui manque, n'ecrit rien
#   sudo bash install_bambooWS.sh --apply    # applique, dans l'ordre, ce qui manque
#   bash install_bambooWS.sh --steps         # liste les etapes et leur ordre
#   sudo bash install_bambooWS.sh --apply --from 6   # reprend a partir d'une etape
#   sudo bash install_bambooWS.sh --apply --only 4   # une seule etape
#
# Idempotent : relancable sans effet si tout est conforme. Le diagnostic est le mode par
# defaut a dessein -- on regarde avant d'agir.
set -uo pipefail

APPLY=0; FROM=1; ONLY=0
while [ $# -gt 0 ]; do
    case "$1" in
        --apply) APPLY=1 ;;
        --from)  FROM="${2:?--from attend un numero d etape}"; shift ;;
        --only)  ONLY="${2:?--only attend un numero d etape}"; shift ;;
        --steps) STEPS_ONLY=1 ;;
        -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "argument inconnu : $1 (voir --help)" >&2; exit 2 ;;
    esac
    shift
done

HERE="$(cd "$(dirname "$0")" && pwd)"          # .../linorobot2/docker
ROS_REPO="$(dirname "$HERE")"                  # .../linorobot2
BASE_DIR="$(dirname "$ROS_REPO")"              # .../Bamboo4WD_V4
HW_REPO="$BASE_DIR/linorobot2_hardware"
# L'utilisateur reel, meme sous sudo : c'est lui qu'on met dans le groupe docker.
RUN_USER="${SUDO_USER:-$(id -un)}"

todo=0; reboot_required=0; relogin_required=0
say()  { echo "$*"; }
ok()   { echo "  [ok]       $*"; }
warn() { echo "  [A FAIRE]  $*"; todo=$((todo + 1)); }
act()  { echo "  [applique] $*"; }
skip() { echo "  [ignore]   $*"; }

STEP_NAMES=(
    "1 uart      UART de l'hote au repos (prerequis a toute ecriture ESP32)"
    "2 docker    demon Docker, plugin compose, groupe docker"
    "3 repos     les deux depots cote a cote"
    "4 udev      ports serie stables (/dev/esp32)"
    "5 bluetooth service Bluetooth de l'hote (appairage manette)"
    "6 stacks    desarmer les stacks Docker concurrentes (collision de ports)"
    "7 build     construire les images du profil boot"
    "8 systemd   unite de rattrapage bamboo4wd.service"
    "9 verify    verifications finales"
)
if [ "${STEPS_ONLY:-0}" = "1" ]; then
    echo "Etapes, dans l'ordre impose par leurs dependances :"
    printf '  %s\n' "${STEP_NAMES[@]}"
    cat <<'EOT'

Ordre, et pourquoi il n'est pas negociable :
  1 avant 2  : les deux veulent un redemarrage (brochage) ou une re-connexion (groupe) ;
               les grouper economise le seul reboot du provisionnement.
  3 avant 7  : driver.real monte ../../linorobot2_hardware ; sans le depot frere, il ne part pas.
  4 avant 7  : le driver et le flash referencent /dev/esp32, jamais /dev/ttyUSB*.
  6 avant 7  : une stack concurrente qui tient 9090/8765 fait echouer la liaison des ports.
  1 avant le flash du microcode (etape facultative, hors de ce script) : sans l'UART libere,
               l'hote ne peut rien ecrire dans la carte.
EOT
    exit 0
fi

run_step() { # $1 = numero
    [ "$ONLY" != "0" ] && { [ "$1" = "$ONLY" ] && return 0 || return 1; }
    [ "$1" -ge "$FROM" ]
}

# Le nom du projet compose, lu du .env. Le `tr -d '\r'` n'est pas de la superstition : le .env
# du depot est en CRLF (edite sous Windows), et docker compose le tolere alors qu'un shell
# non -- sans ce filtre, PROJECT portait un CR final et ne s'appariait a aucun libellé de
# conteneur, ce qui faisait passer nos propres conteneurs pour une stack concurrente.
PROJECT="$(grep -E '^COMPOSE_PROJECT_NAME=' "$HERE/.env" 2>/dev/null | cut -d= -f2 | tr -d '\r' | tr -d '[:space:]')"
PROJECT="${PROJECT:-bamboov4_humble}"

# Les services du profil `boot`, lus du compose -- donc une seule declaration a maintenir.
# Pourquoi ne pas demander a compose : `--profile boot` ne SELECTIONNE pas ces services, il
# les AJOUTE au profil `default` (tout service sans `profiles:`), et en v2.21
# `config --services` l'ignore de toute facon. D'ou la lecture directe de l'attribution :
# une cle de service est a 2 espaces, ses attributs a 4, donc un `profiles:` contenant "boot"
# appartient au dernier service vu. Cette liste est ensuite NOMMEE partout -- c'est la seule
# facon d'agir sur ces cinq services et sur eux seuls.
boot_services() {
    awk '
        /^  [a-zA-Z0-9._-]+:[[:space:]]*$/ { svc = $1; sub(/:$/, "", svc); next }
        /^    profiles:.*boot/ && svc != "" { print svc }
    ' "$HERE/docker-compose.yaml"
}

echo "=== BambooWS — provisionnement ($([ "$APPLY" = 1 ] && echo 'APPLICATION' || echo 'diagnostic seul')) ==="
echo "  depot ROS      : $ROS_REPO"
echo "  depot cartes   : $HW_REPO"
echo "  utilisateur    : $RUN_USER"
if [ "$APPLY" = "1" ] && [ "$(id -u)" != "0" ]; then
    echo "ERREUR: --apply demande les droits root (sudo)." >&2; exit 2
fi
echo ""

# ========================================================================================
run_step 1 && {
echo "=== 1. UART de l'hote ==="
# Delegue : le detail (pourquoi GPIO14 ecrase le RXD de l'ESP32, d'ou vient enable_uart=1
# sur DietPi) est documente une seule fois, dans install_host_prereq.sh.
if [ -r "$HERE/install_host_prereq.sh" ]; then
    # On lit le VERDICT du sous-script plutot que de reinspecter config.txt : `enable_uart=0`
    # deja en vigueur ne demande aucun reboot, et c'est lui seul qui sait faire la difference.
    # Une seule execution, donc un seul effet de bord en mode --apply.
    prereq_out="$(bash "$HERE/install_host_prereq.sh" $([ "$APPLY" = "1" ] && echo --apply) 2>&1)"
    printf '%s\n' "$prereq_out" | sed 's/^/  | /'
    printf '%s' "$prereq_out" | grep -q 'REBOOT REQUIS' && reboot_required=1
    printf '%s' "$prereq_out" | grep -q 'A FAIRE'       && todo=$((todo + 1))
else
    warn "install_host_prereq.sh introuvable a cote de ce script"
fi
echo ""
}

# ========================================================================================
run_step 2 && {
echo "=== 2. Docker ==="
# DietPi fournit Docker (id 162) et le plugin compose (id 134) par dietpi-software : on
# passe par lui plutot que par le depot upstream, pour rester dans l'idiome de la distro
# (mises a jour, desinstallation propre).
have_docker=0; command -v docker >/dev/null 2>&1 && have_docker=1
[ "$have_docker" = 1 ] && ok "docker present : $(docker --version)" || warn "docker absent"
if docker compose version >/dev/null 2>&1; then
    ok "plugin compose present : $(docker compose version | head -1)"
else
    warn "plugin 'docker compose' absent"
fi
if [ "$have_docker" != 1 ] || ! docker compose version >/dev/null 2>&1; then
    if [ "$APPLY" = "1" ]; then
        if [ -x /boot/dietpi/dietpi-software ]; then
            act "installation par dietpi-software (162 Docker, 134 Docker Compose)"
            /boot/dietpi/dietpi-software install 162 134
        else
            warn "hote non-DietPi : installer Docker par la procedure de la distribution"
        fi
    fi
fi
# Le groupe docker evite un `sudo` sur chaque commande -- y compris dans l'unite systemd et
# dans les scripts MCP. L'appartenance ne prend effet qu'a la prochaine ouverture de session.
if id -nG "$RUN_USER" 2>/dev/null | tr ' ' '\n' | grep -qx docker; then
    ok "$RUN_USER est dans le groupe docker"
elif [ "$APPLY" = "1" ]; then
    if getent group docker >/dev/null && usermod -aG docker "$RUN_USER"; then
        act "$RUN_USER ajoute au groupe docker"; relogin_required=1
    else
        warn "ajout au groupe docker impossible (groupe absent : Docker installe ?)"
    fi
else
    warn "$RUN_USER hors du groupe docker : chaque commande docker exige sudo"
fi
echo ""
}

# ========================================================================================
run_step 3 && {
echo "=== 3. Depots cote a cote ==="
# Contrainte venue du compose, pas d'un choix de style : driver.real monte
# `../../linorobot2_hardware`. Les deux depots doivent donc etre freres.
if [ -d "$HW_REPO/.git" ]; then
    ok "depot cartes present : $HW_REPO ($(git -C "$HW_REPO" rev-parse --abbrev-ref HEAD 2>/dev/null))"
elif [ "$APPLY" = "1" ]; then
    # On derive l'URL du depot frere de celle du depot courant : pas d'URL codee en dur, donc
    # ca suit un fork ou un miroir sans edition.
    origin="$(git -C "$ROS_REPO" remote get-url origin 2>/dev/null)"
    if [ -n "$origin" ]; then
        hw_url="$(echo "$origin" | sed -E 's#(linorobot2)(\.git)?$#\1_hardware\2#')"
        act "clonage de $hw_url"
        git clone "$hw_url" "$HW_REPO" || warn "clonage echoue : cloner $hw_url a la main dans $BASE_DIR"
    else
        warn "pas d'origin git : cloner linorobot2_hardware a la main dans $BASE_DIR"
    fi
else
    warn "depot cartes absent : $HW_REPO (driver.real ne demarrera pas)"
fi
echo ""
}

# ========================================================================================
run_step 4 && {
echo "=== 4. Regles udev ==="
SRC_RULES="$HERE/udev/99-bambooWS.rules"
DST_RULES="/etc/udev/rules.d/99-bambooWS.rules"
if [ ! -r "$SRC_RULES" ]; then
    warn "$SRC_RULES introuvable"
elif cmp -s "$SRC_RULES" "$DST_RULES" 2>/dev/null; then
    ok "$DST_RULES a jour"
elif [ "$APPLY" = "1" ]; then
    install -m 0644 "$SRC_RULES" "$DST_RULES" \
        && act "$DST_RULES pose" \
        && udevadm control --reload-rules && udevadm trigger --subsystem-match=tty \
        && act "udev recharge"
else
    warn "$DST_RULES absent ou obsolete : /dev/esp32 n'est pas garanti"
fi
# Des regles heritees se disputent les symlinks (dette connue, hors perimetre BambooWS) : on
# les signale sans y toucher -- les supprimer est une decision, pas un effet de bord.
stale="$(ls /etc/udev/rules.d/ 2>/dev/null | grep -E 'ydlidar|teensy' | tr '\n' ' ')"
[ -n "$stale" ] && skip "regles heritees presentes, non modifiees : $stale"
if [ -e /dev/esp32 ]; then
    ok "/dev/esp32 -> $(readlink -f /dev/esp32)"
else
    warn "/dev/esp32 absent : carte debranchee, ou numero de serie a relever (voir le .rules)"
fi
echo ""
}

# ========================================================================================
run_step 5 && {
echo "=== 5. Bluetooth de l'hote ==="
# L'appairage est hote : le conteneur ne voit qu'un /dev/input/js* deja appaire. Et il est
# INTERACTIF (code, confirmation) -- donc ce script prepare le terrain, il n'appaire pas.
#
# ORDRE DE CAUSALITE, et il explique un diagnostic trompeur : la RADIO est coupee au niveau
# firmware par `dtoverlay=disable-bt` (DietPi par defaut), ce qui se traite a l'etape 1
# (install_host_prereq.sh) et demande un reboot. Tant qu'elle est coupee, bluez n'a aucun
# adaptateur : le service demarre puis meurt et `hciconfig` repond "Address family not
# supported by protocol", alors que le paquet est installe et le service `enabled`. Un
# `systemctl enable --now bluetooth` ne peut donc RIEN y faire -- d'ou ce controle en tete,
# qui renvoie a la vraie cause au lieu de signaler un echec de demarrage sans explication.
if [ ! -d /sys/class/bluetooth ] || [ -z "$(ls -A /sys/class/bluetooth 2>/dev/null)" ]; then
    warn "aucun adaptateur BT : radio coupee au firmware -> etape 1 (--only 1) puis REBOOT"
elif ! command -v bluetoothctl >/dev/null 2>&1; then
    warn "bluetoothctl absent : installer bluez (DietPi : dietpi-software install 96)"
elif [ "$(systemctl is-active bluetooth 2>/dev/null)" = "active" ]; then
    ok "service bluetooth actif"
else
    if [ "$APPLY" = "1" ]; then
        systemctl enable --now bluetooth >/dev/null 2>&1 \
            && act "service bluetooth active et demarre" \
            || warn "demarrage du service bluetooth echoue"
    else
        warn "service bluetooth inactif : la manette ne pourra pas s'appairer"
    fi
fi
[ -e /dev/input/js0 ] && ok "manette vue : /dev/input/js0" \
    || skip "aucune manette appairee (etape manuelle : bluetoothctl, manette en mode XInput)"
echo ""
}

# ========================================================================================
run_step 6 && {
echo "=== 6. Stacks concurrentes ==="
# Constat du 2026-09-22 : la stack d'un robot precedent redemarrait au boot en
# `restart: always` et tenait 8765 -- le Foxglove de BambooWS ne pouvait plus se lier, et
# l'erreur ne disait rien de la cause. On DESARME (politique de restart + arret) sans jamais
# supprimer : reversible par `docker start`, et l'historique reste consultable.
others="$(docker ps -a --filter label=com.docker.compose.project --format '{{.Names}}|{{.Label "com.docker.compose.project"}}' 2>/dev/null \
          | awk -F'|' -v p="$PROJECT" '$2 != p {print $1}')"
if [ -z "$others" ]; then
    ok "aucune stack compose concurrente (projet courant : $PROJECT)"
else
    for c in $others; do
        pol="$(docker inspect -f '{{.HostConfig.RestartPolicy.Name}}' "$c" 2>/dev/null)"
        st="$(docker inspect -f '{{.State.Status}}' "$c" 2>/dev/null)"
        case "$pol" in
            always|unless-stopped|on-failure)
                if [ "$APPLY" = "1" ]; then
                    docker update --restart=no "$c" >/dev/null 2>&1
                    [ "$st" = "running" ] && docker stop "$c" >/dev/null 2>&1
                    act "$c desarme (restart=no$([ "$st" = running ] && echo ', arrete'))"
                else
                    warn "$c redemarrera au boot (restart=$pol) et peut voler les ports"
                fi ;;
            *) ok "$c inoffensif (restart=${pol:-no}, $st)" ;;
        esac
    done
fi
echo ""
}

# ========================================================================================
run_step 7 && {
echo "=== 7. Images du profil boot ==="
# Tout est declare dans le compose : on ne fait que l'appeler. Long au premier passage
# (~3,7 Go d'image ROS a construire sur RPi4).
svcs="$(boot_services | tr '\n' ' ')"
if [ -z "$svcs" ]; then
    warn "aucun service ne porte profiles: [\"boot\"] dans docker-compose.yaml"
else
    # On nomme les services explicitement : `config --images` sans argument ignore le profil
    # et listerait les images de tout le fichier (services de build compris).
    missing=""
    while read -r img; do
        [ -z "$img" ] && continue
        docker image inspect "$img" >/dev/null 2>&1 || missing="$missing $img"
    done <<< "$( cd "$HERE" && docker compose config --images $svcs 2>/dev/null | sort -u )"
    if [ -z "$missing" ]; then
        # On ne reconstruit PAS ce qui existe : un build de l'image ROS coute des dizaines de
        # minutes sur RPi4, et le provisionnement doit pouvoir etre relance sans le payer. Une
        # reconstruction deliberee reste un acte explicite :
        #   docker compose --profile boot build [--no-cache]
        ok "images presentes pour le profil boot ($svcs)"
    elif [ "$APPLY" = "1" ]; then
        act "construction des images manquantes :$missing (des dizaines de minutes sur RPi4)"
        # Services NOMMES : un `--profile boot build` construirait aussi les services de build
        # du fichier (firmware Teensy et AVR), hors perimetre et longs.
        ( cd "$HERE" && docker compose build $svcs ) || warn "build en echec (voir la sortie ci-dessus)"
    else
        for img in $missing; do warn "image absente : $img"; done
    fi
fi
# Le microcode de la carte n'est PAS installe ici : le binaire n'est pas versionne (.pio est
# ignore), donc l'etape est explicite et separee, une fois le .bin depose :
#   docker compose stop driver.real
#   docker compose --profile tools run --rm flash.esp32
echo ""
}

# ========================================================================================
run_step 8 && {
echo "=== 8. Unite systemd de rattrapage ==="
# Rappel de ce que cette unite n'est pas : ce n'est pas elle qui demarre le robot au boot,
# c'est le `restart: unless-stopped` du compose. Elle rattrape les cas ou il n'y a rien a
# relancer (premier demarrage, apres un `down`, apres modification du compose).
SRC_UNIT="$HERE/systemd/bamboo4wd.service"
DST_UNIT="/etc/systemd/system/bamboo4wd.service"
if [ ! -r "$SRC_UNIT" ]; then
    warn "$SRC_UNIT introuvable"
elif [ "$APPLY" = "1" ]; then
    # Deux substitutions : l'emplacement reel du depot, et la liste des services du profil
    # `boot` -- lue du compose a l'instant, donc l'unite posee ne peut pas deriver de lui.
    unit_svcs="$(boot_services | tr '\n' ' ')"
    unit_svcs="${unit_svcs% }"
    if [ -z "$unit_svcs" ]; then
        warn "aucun service marque profiles: [\"boot\"] : unite non posee (elle ne demarrerait rien)"
    else
        sed -e "s#__COMPOSE_DIR__#$HERE#" -e "s#__BOOT_SERVICES__#$unit_svcs#" "$SRC_UNIT" > "$DST_UNIT" \
            && act "$DST_UNIT pose (services : $unit_svcs)"
        systemctl daemon-reload
        systemctl enable bamboo4wd.service >/dev/null 2>&1 && act "bamboo4wd.service active au boot"
        systemctl restart bamboo4wd.service && act "stack de boot demarree" \
            || warn "demarrage en echec : journalctl -u bamboo4wd"
    fi
else
    if [ -r "$DST_UNIT" ]; then
        ok "unite posee ($(systemctl is-enabled bamboo4wd 2>/dev/null), $(systemctl is-active bamboo4wd 2>/dev/null))"
    else
        warn "$DST_UNIT absent : rien ne recreera les conteneurs apres un 'compose down'"
    fi
fi
echo ""
}

# ========================================================================================
run_step 9 && {
echo "=== 9. Verifications ==="
for p in 9090 8765 8554; do
    if ss -ltn 2>/dev/null | grep -q ":$p "; then ok "port $p en ecoute"
    else warn "port $p muet (rosbridge / foxglove / RTSP selon le cas)"; fi
done
expected="$(boot_services | sort | tr '\n' ' ')"
up="$( cd "$HERE" && docker compose ps --services --filter status=running 2>/dev/null | sort | tr '\n' ' ' )"
say "  attendus : ${expected:-aucun}"
if [ "$up" = "$expected" ]; then
    ok "services en marche : $up"
elif [ -n "$up" ]; then
    warn "services en marche : $up (ne correspond pas aux attendus)"
else
    warn "aucun service en marche"
fi
echo ""
}

echo "=== bilan ==="
if [ "$todo" = "0" ]; then
    echo "  robot conforme."
else
    echo "  $todo point(s) restant(s) : relancer avec 'sudo bash $0 --apply'."
fi
[ "$reboot_required" = "1" ] && echo "  REBOOT REQUIS (brochage UART)."
[ "$relogin_required" = "1" ] && echo "  RE-CONNEXION REQUISE (groupe docker) -- le reboot ci-dessus y suffit."
exit 0
