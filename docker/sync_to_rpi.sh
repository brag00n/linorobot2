#!/usr/bin/env bash
# sync_to_rpi.sh — pousse les 2 depots Windows -> RPi par rsync (boucle d'iteration).
#
# CYCLE DE VIE DU CODE SUR LE RPi (decide avec l'utilisateur) :
#   1. INIT (une seule fois, sur le RPi) : cloner les 2 depots COTE A COTE (le compose
#      bind-monte `..` et `../../linorobot2_hardware` -> ils doivent etre freres, et
#      `firmware/` doit rester a cote de `tools/` pour les imports du codec) :
#        mkdir -p "$RPI_BASE" && cd "$RPI_BASE"
#        git clone -b humble_develop_bamboov4 <url> linorobot2
#        git clone -b humble_develop_bamboov4 <url> linorobot2_hardware
#   2. ITERATION : editer sous Windows -> `bash sync_to_rpi.sh` -> tester sur le RPi.
#      Un .py de noeud (source bind-montee + install -e) : il suffit de RELANCER le noeud,
#      pas de rebuild. YAML : relancer le noeud. deps/Dockerfile : `docker compose build`.
#      Paquet C++ (ydlidar, bamboo_interfaces) : `colcon build` dans le conteneur.
#   3. FIN DE PHASE : commit + push depuis Windows, puis sur le RPi remettre le working
#      tree exactement sur le remote (les deltas rsync sont alors REDONDANTS) :
#        git fetch origin && git reset --hard origin/humble_develop_bamboov4
#      puis re-test final "propre". (Si git refuse a cause de fichiers non suivis qui
#      collisionnent : `git clean -fd` — ATTENTION supprime maps/donnees non versionnees,
#      sauvegarder d'abord.)
#
# rsync N'EXCLUT PAS par hasard : `.git/` (preserve le depot RPi pour le pull de fin de
# phase), `tools/.venv/` (venv Windows, client MCP — jamais sur ARM), `data/faces/`
# (non versionne, sensible), artefacts de build.
#
# Prerequis : rsync + ssh dans CE shell (Git Bash+rsync, WSL, ou MSYS2). Depuis PowerShell :
#   bash ./sync_to_rpi.sh    ou    wsl ./sync_to_rpi.sh
set -euo pipefail

# ---------- a adapter (ou surcharger via variables d'environnement) ----------
RPI_HOST="${RPI_HOST:-dietpi@192.168.1.164}"                        # user@ip du RPi
RPI_BASE="${RPI_BASE:-/home/dietpi/prj_robotique/Bamboo4WD_V4}"    # parent des 2 depots
BRANCH="${BRANCH:-humble_develop_bamboov4}"
# Cle SSH dediee au RPi (nom non standard : pas d'agent ni de ~/.ssh/config -> on la passe
# explicitement a ssh ET a rsync via -e, sinon la connexion echoue en non-interactif).
SSH_KEY="${SSH_KEY:-$HOME/.ssh/id_rpi_grovepi}"
SSH_CMD="ssh -i $SSH_KEY -o BatchMode=yes"
# -----------------------------------------------------------------------------

# Ce script vit dans linorobot2/docker/ ; ../.. = le dossier parent des 2 depots.
WIN_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

EXCLUDES=(
  --exclude='.git/'            # ne jamais ecraser le depot git du RPi (pull de fin de phase)
  --exclude='tools/.venv/'     # venv Windows (client MCP) : binaires win32, pas pour ARM
  --exclude='**/__pycache__/'
  --exclude='*.pyc'
  --exclude='*.egg-info/'
  --exclude='data/faces/'      # jeu de visages : non versionne, sensible
  --exclude='.pio/'            # PlatformIO
  --exclude='build/' --exclude='install/' --exclude='log/'   # colcon
  --exclude='*.stackdump'
)

# -rlt (pas -a) + no-perms/owner : evite le churn de permissions Windows->Linux.
# Pas de --delete par defaut (ne pas risquer d'effacer maps/donnees cote RPi).
RSYNC_OPTS=(-rlt --no-perms --no-owner --no-group --chmod=ugo=rwX \
            -e "$SSH_CMD" --info=stats1,progress2 "${EXCLUDES[@]}")

for repo in linorobot2 linorobot2_hardware; do
  echo "==> rsync $repo -> $RPI_HOST:$RPI_BASE/$repo"
  $SSH_CMD "$RPI_HOST" "mkdir -p '$RPI_BASE/$repo'"
  rsync "${RSYNC_OPTS[@]}" "$WIN_ROOT/$repo/" "$RPI_HOST:$RPI_BASE/$repo/"
done

echo "OK — sync termine (branche cible : $BRANCH)."
echo "Rappel : les deltas rsync ne sont PAS commites cote RPi ; en fin de phase, push"
echo "depuis Windows puis 'git fetch && git reset --hard origin/$BRANCH' sur le RPi."
