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
# Prerequis : ssh, plus rsync SI DISPONIBLE -- a defaut le script se replie sur tar (voir
# sync_repo_tar plus bas). Le Git Bash de Git for Windows n'a pas rsync : exiger rsync, c'est
# ce qui a fait qu'aucune synchro ne partait et que le RPi tournait sur du code perime.
# Depuis PowerShell :   bash ./sync_to_rpi.sh    ou    wsl ./sync_to_rpi.sh
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
  # .env porte des faits PROPRES AU RPi (chemins de peripheriques, options d'hote) et il y est
  # modifie a la main : l'ecraser depuis Windows casserait le demarrage des conteneurs.
  --exclude='docker/.env'
)

# -rlt (pas -a) + no-perms/owner : evite le churn de permissions Windows->Linux.
# Pas de --delete par defaut (ne pas risquer d'effacer maps/donnees cote RPi).
RSYNC_OPTS=(-rlt --no-perms --no-owner --no-group --chmod=ugo=rwX \
            -e "$SSH_CMD" --info=stats1,progress2 "${EXCLUDES[@]}")

# Repli tar si rsync manque, et ce n'est PAS un luxe : le Git Bash livre avec Git for
# Windows n'embarque pas rsync. Sans ce repli, le script echouait sur la PREMIERE ligne de
# la boucle, et le depot du RPi restait silencieusement en arriere -- constat du 2026-09-23 :
# le fichier canonique du lot 0 n'etait jamais arrive, le driver a demarre sur la geometrie
# d'un commit de septembre 2025. Une synchro qui echoue est visible ; une synchro jamais
# lancee ne l'est pas, d'ou l'exigence : ce script doit marcher dans le shell qu'on a.
# tar est present partout (Git Bash, WSL, MSYS2) et ne demande rien au RPi.
# Difference assumee : tar POUSSE TOUT l'arbre a chaque fois (pas de delta) et n'a pas
# --delete non plus -- plus lent, meme resultat pour un depot de cette taille.
sync_repo_tar() {
  local repo="$1"
  # On REECRIT les motifs de rsync pour tar, car les deux ne les lisent pas pareil :
  #   - tar ignore un / final ("--exclude=.git/" ne matche rien) -> on le retire ;
  #   - tar exclut en mode --no-anchored, donc un motif nu matche n'importe quel suffixe de
  #     chemin : "__pycache__" couvre deja tout l'arbre -> le prefixe "**/" de rsync devient
  #     inutile, et le garder risquerait d'exiger un / la ou il n'y en a pas.
  local tar_excl=()
  local e pat
  for e in "${EXCLUDES[@]}"; do
    pat="${e#--exclude=}"
    pat="${pat%/}"
    pat="${pat#\*\*/}"
    tar_excl+=("--exclude=$pat")
  done
  tar czf - -C "$WIN_ROOT/$repo" "${tar_excl[@]}" . \
    | $SSH_CMD "$RPI_HOST" "tar xzf - -C '$RPI_BASE/$repo'"
}

if command -v rsync >/dev/null 2>&1; then
  TRANSPORT="rsync"
else
  TRANSPORT="tar"
  echo "!! rsync absent de ce shell -> repli sur tar (arbre complet, pas de delta)."
fi

for repo in linorobot2 linorobot2_hardware; do
  echo "==> $TRANSPORT $repo -> $RPI_HOST:$RPI_BASE/$repo"
  $SSH_CMD "$RPI_HOST" "mkdir -p '$RPI_BASE/$repo'"
  if [ "$TRANSPORT" = "rsync" ]; then
    rsync "${RSYNC_OPTS[@]}" "$WIN_ROOT/$repo/" "$RPI_HOST:$RPI_BASE/$repo/"
  else
    sync_repo_tar "$repo"
  fi
done

echo "OK — sync termine (branche cible : $BRANCH)."
echo "Rappel : les deltas rsync ne sont PAS commites cote RPi ; en fin de phase, push"
echo "depuis Windows puis 'git fetch && git reset --hard origin/$BRANCH' sur le RPi."
