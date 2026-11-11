#!/bin/bash
# Demarrage de rosbridge (une seule instance, CC + HUD + outils MCP).
#
# BAMBOO_SECURE=http  (defaut) : comportement historique, tout est permis, :9090 ouvert au reseau.
# BAMBOO_SECURE=https          : LISTE BLANCHE (topics / services) ; l'ecoute en clair n'est plus publiee
#                                que sur la boucle locale de l'hote (voir ROSBRIDGE_BIND du compose),
#                                le seul chemin reseau est donc le proxy HTTPS (wss://<hote>:8443/rosbridge).
# Bascule : docker/secure.sh https|http.
#
# LECTURE / ECRITURE SEPAREES (rosbridge >= 2.0 : topics_sub_glob / topics_pub_glob). Les clients LISENT la liste
# TOPICS mais ne PUBLIENT que PUB_TOPICS (le battement du CC). Ni /cmd_vel*, ni /cc/guard/{enable,link,stop}
# (forger `link` autoriserait la voie cc), ni /navigation/enable, /servo/cmd, /videotracking/mode_cmd ne sont
# publiables. Les ecritures utiles passent par des services (arm, mode, nav). Avec la 1.3.x, `topics_glob` seul
# ne distinguait pas (d'ou la version 2.0.8 epinglee).
test -f /root/linorobot2_ws/install/setup.bash && source /root/linorobot2_ws/install/setup.bash

TOPICS="/battery,/esp32_status,/wheel_rpm,/imu/*,/odom*,/scan,/tf,/tf_static,/map,/plan,/joint_states,/mag,\
/robot_description,/rosout,/parameter_events,/client_count,/connected_clients,/gamepad/connected,/joy,\
/diagnostics,/diagnostics_agg,/diagnostics_toplevel_state,/camera/*,/camera_info,/video/*,\
/videotracking/stats,/videotracking/track_state,/videotracking/recognition,/videotracking/enriched/compressed,\
/motion_compensation/state,/teleop/state,/bamboo/*,/bamboo_calibration/battery,/bamboo_calibration/report,\
/navigation/state,/navigation/robot_pose,/navigation/frontiers,/navigation/home,/navigation/map_preview,\
/cc/heartbeat,/cc/guard/state,/cc/mission/state,/cc/mission/list"
# Publiable depuis le reseau : le battement, et les DEUX demandes de mission (lancer, arreter). Une demande de mission
# ne commande aucun moteur : le moteur de missions refuse toute etape qui roule sans traction armee ET liaison saine.
PUB_TOPICS="/cc/heartbeat,/cc/mission/request,/cc/mission/cancel_req"

SERVICES="/rosapi/*,/bamboo/*,/cc/guard/arm,/cc/guard/arm_locked,/nav_supervisor/command,\
*/get_parameters,*/list_parameters,*/describe_parameters,*/get_parameter_types,*/set_parameters,\
/esp32_mavlink_driver/recalibrate_imu"

# rosbridge relit chaque liste comme du YAML : un motif qui commence par `*` serait pris pour un alias.
# On met donc chaque element entre apostrophes (rosbridge les retire, cf. rosbridge_websocket.py).
quote() { local IFS=,; local out=() e; for e in $1; do out+=("'$e'"); done; echo "${out[*]}"; }
TOPICS=$(quote "$TOPICS"); SERVICES=$(quote "$SERVICES"); PUB_TOPICS=$(quote "$PUB_TOPICS")

ARGS=()
if [ "${BAMBOO_SECURE:-http}" = "https" ]; then
  # Le noeud attend une CHAINE "[a,b,c]" (qu'il decoupe lui-meme) et non une liste YAML : d'ou les guillemets
  # doubles autour du crochet, que `ros2 launch` lit comme une chaine.
  ARGS+=("topics_sub_glob:=\"[${TOPICS}]\"" "topics_pub_glob:=\"[${PUB_TOPICS}]\"" "services_glob:=\"[${SERVICES}]\"")
  echo "rosbridge : mode https -> liste blanche active (lecture large, ecriture : /cc/heartbeat seul)"
else
  echo "rosbridge : mode http -> aucune restriction"
fi
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml "${ARGS[@]}" "$@"
