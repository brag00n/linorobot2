#!/usr/bin/env bash
# Compilation A LA DEMANDE de l espace de travail colcon, DANS le conteneur.
#
# POURQUOI CE SCRIPT EXISTE. Les `command:` du compose compilaient AVANT de lancer, donc tout
# `restart` repayait la compilation : ~17 min sur ce RPi4, pendant lesquelles les noeuds ne
# sont pas adressables (`ros2 param get` repond "Node not found" et on cherche une panne DDS).
# Pire, deux services relances coup sur coup lancent DEUX colcon concurrents : les 4 coeurs
# saturent au point que sshd ne repond plus a la poignee de main -- machine injoignable, alors
# que le ping reste a 25 ms. La compilation est desormais une ACTION EXPLICITE, jamais un effet
# de bord d un demarrage.
#
# Un `docker exec` ne passe PAS par l entrypoint de l image : ce script source donc ROS et pose
# FASTRTPS_DEFAULT_PROFILES_FILE lui-meme (memoire partagee DDS coupee, comme les services).
#
# Usage, depuis l hote :
#   docker exec <conteneur> /root/linorobot2_ws/src/linorobot2/docker/rosbuild.sh [paquets...]
# Sans argument : tous les paquets du depot listes dans PAQUETS_DEFAUT.
# Le depot est monte en rw dans le conteneur (compose, service `base`), donc ce script y est
# visible SANS reconstruire l image.
#
# A NE PAS FAIRE : ajouter `--cmake-args -DCMAKE_BUILD_TYPE=...` a la main. Les objets deja
# compiles le sont SANS cette variable ; la poser change la configuration CMake et invalide
# TOUT -- une recompilation complete, mesuree 17 min, au prochain demarrage. Mesure a l usage
# le 2026-09-29.
set -eo pipefail
# `set -u` volontairement ABSENT : les setup.bash de ROS 2 y sont hostiles (variables non
# definies dereferencees), le sourcing echouerait avant la moindre compilation.

WS=/root/linorobot2_ws
PAQUETS_DEFAUT="bamboo_interfaces bamboo_base bamboo_video bamboo_videotracking bamboo_control bamboo_controler_YBStm32v3 bamboo4WD_V4_YBStm32_base"
PAQUETS="${*:-$PAQUETS_DEFAUT}"

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.shm_off.xml
cd "$WS"

echo "=== colcon build : $PAQUETS"
# MAKEFLAGS=-j3 et non -j4 : un coeur reste aux autres conteneurs, et ce RPi n a pas de swap.
# --parallel-workers 1 : colcon parallelise les PAQUETS par defaut (autant que de coeurs), ce qui
# se MULTIPLIE avec MAKEFLAGS -- brider make seul ne bride rien. Un paquet a la fois, 3 coeurs
# pour lui : c est ce qui evite de reproduire le 2026-09-29, ou la saturation des 4 coeurs a
# empeche sshd de repondre a la poignee de main (machine injoignable, ping toujours a 25 ms).
MAKEFLAGS=-j3 colcon build --packages-select $PAQUETS --symlink-install --parallel-workers 1

echo "=== construit. Pour CHARGER le nouveau code, relancer le service :"
echo "===   docker compose stop <service> && docker compose start <service>"
echo "=== build/ et install/ vivent desormais dans les volumes nommes PARTAGES ws_build/ws_install"
echo "=== (voir leur declaration en tete du compose), donc une recreation de conteneur ne jette"
echo '=== PLUS la compilation. Les detruire, EUX, la jette : c est `docker volume rm` qu il faut'
echo '=== eviter, pas `up --force-recreate`.'

