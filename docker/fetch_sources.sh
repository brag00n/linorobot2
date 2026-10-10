#!/usr/bin/env bash
# fetch_sources.sh -- recupere les sources TIERCES compilees dans l'espace de travail.
#
# POURQUOI. Certains drivers n'existent pas en paquet apt pour Humble : c'est le cas du
# driver du lidar D500 (LDROBOT STL-19P, famille LD19). On ne les recopie pas dans le depot
# (ce n'est pas notre code), et on ne les installe pas dans l'image `hardware` (la
# reconstruire coute plusieurs heures sur RPi4). On les clone a un COMMIT FIGE dans
# <depot>/third_party/, ignore par git, ou colcon les trouve comme n'importe quel paquet du
# depot (le depot est monte dans src/). docker/rosbuild.sh les compile ensuite.
#
# Idempotent : un depot deja present au bon commit n'est pas retouche.
#
# Usage, dans un conteneur `hardware` (il faut git et le reseau) :
#   docker exec <conteneur> /root/linorobot2_ws/src/linorobot2/docker/fetch_sources.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/third_party"
mkdir -p "$DEST"

# nom | url | commit FIGE (pas de version publiee par le fabricant : on fige le dernier
# commit verifie, 2026-10-10)
SOURCES=(
  "ldlidar_stl_ros2|https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git|bf668a89baf722a787dadc442860dcbf33a82f5a"
)

for entry in "${SOURCES[@]}"; do
  IFS='|' read -r name url commit <<< "$entry"
  dir="$DEST/$name"
  if [ -d "$dir/.git" ] && [ "$(git -C "$dir" rev-parse HEAD)" = "$commit" ]; then
    echo "=== $name : deja au commit $commit"
    continue
  fi
  if [ ! -d "$dir/.git" ]; then
    echo "=== $name : clonage de $url"
    git clone --quiet "$url" "$dir"
  fi
  git -C "$dir" fetch --quiet origin
  git -C "$dir" checkout --quiet "$commit"
  echo "=== $name : commit $commit"
done
echo "=== sources tierces pretes dans $DEST ; compiler avec docker/rosbuild.sh <paquets>"
