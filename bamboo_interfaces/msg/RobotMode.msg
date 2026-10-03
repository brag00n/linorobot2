# Mode du ROBOT (bamboo_mode_manager), aux conventions de micro-ROS/system_modes sans le
# paquet : deux sujets LATCHES portent ce message,
#   /bamboo/mode_request_info   la DEMANDE, publiee AVANT d'agir (mode = cible) ;
#   /bamboo/mode                le mode APPLIQUE, publie apres les reglages d'entree.
std_msgs/Header header

string mode             # identifiant (conduite, tracking, calibration, navigation...)
string label            # texte court pour le HUD et l'OLED (CONDUITE, CALIB...)
string previous         # mode quitte
bool degraded           # un reglage d'entree n'a pas ete acquitte par son noeud
string detail           # raison du mode degrade, ou de la demande
