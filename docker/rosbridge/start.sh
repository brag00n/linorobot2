#!/bin/bash
# Demarrage de rosbridge (une seule instance, CC + HUD + outils MCP).
#
# BAMBOO_SECURE=http  (defaut) : comportement historique, tout est permis, :9090 ouvert au reseau.
# BAMBOO_SECURE=https          : LISTE BLANCHE (topics / services) ; l'ecoute en clair n'est plus publiee
#                                que sur la boucle locale de l'hote (voir ROSBRIDGE_BIND du compose),
#                                le seul chemin reseau est donc le proxy HTTPS (wss://<hote>:8443/rosbridge).
# Bascule : docker/secure.sh https|http.
#
# LIMITE ASSUMEE : rosbridge ne distingue pas lecture et ecriture dans `topics_glob`. Un topic liste est
# donc aussi publiable par tout client. On y met uniquement ce qui ne commande rien ; JAMAIS /cmd_vel*,
# /cc/guard/{enable,link,stop} (forger `link` autoriserait la voie cc), /navigation/enable,
# /servo/cmd, /videotracking/mode_cmd. Les ecritures utiles passent par des services (arm, mode, nav).
test -f /root/linorobot2_ws/install/setup.bash && source /root/linorobot2_ws/install/setup.bash

TOPICS="/battery,/esp32_status,/wheel_rpm,/imu/*,/odom*,/scan,/tf,/tf_static,/map,/plan,/joint_states,/mag,\
/robot_description,/rosout,/parameter_events,/client_count,/connected_clients,/gamepad/connected,/joy,\
/diagnostics,/diagnostics_agg,/diagnostics_toplevel_state,/camera/*,/camera_info,/video/*,\
/videotracking/stats,/videotracking/track_state,/videotracking/recognition,/videotracking/enriched/compressed,\
/motion_compensation/state,/teleop/state,/bamboo/*,/bamboo_calibration/battery,/bamboo_calibration/report,\
/navigation/state,/navigation/robot_pose,/navigation/frontiers,/navigation/home,/navigation/map_preview,\
/cc/heartbeat,/cc/guard/state"

SERVICES="/rosapi/*,/bamboo/*,/cc/guard/arm,/cc/guard/arm_locked,/nav_supervisor/command,\
*/get_parameters,*/list_parameters,*/describe_parameters,*/get_parameter_types,*/set_parameters,\
/esp32_mavlink_driver/recalibrate_imu"

# rosbridge relit chaque liste comme du YAML : un motif qui commence par `*` serait pris pour un alias.
# On met donc chaque element entre apostrophes (rosbridge les retire, cf. rosbridge_websocket.py).
quote() { local IFS=,; local out=() e; for e in $1; do out+=("'$e'"); done; echo "${out[*]}"; }
TOPICS=$(quote "$TOPICS"); SERVICES=$(quote "$SERVICES")

ARGS=()
if [ "${BAMBOO_SECURE:-http}" = "https" ]; then
  # Le noeud attend une CHAINE "[a,b,c]" (qu'il decoupe lui-meme) et non une liste YAML : d'ou les guillemets
  # doubles autour du crochet, que `ros2 launch` lit comme une chaine.
  ARGS+=("topics_glob:=\"[${TOPICS}]\"" "services_glob:=\"[${SERVICES}]\"")
  echo "rosbridge : mode https -> liste blanche active"
else
  echo "rosbridge : mode http -> aucune restriction"
fi
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml "${ARGS[@]}" "$@"
