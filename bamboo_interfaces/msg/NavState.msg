# Etat de la navigation (bamboo_navigation/nav_supervisor), publie LATCHE sur /navigation/state
# a chaque changement et au moins a 1 Hz. Lu par le HUD (carte NAV), l'OLED, la manette et le
# MCP.
std_msgs/Header header

string submode            # idle | exploration | navigation
string map_name           # carte chargee (navigation) ou en cours (exploration)
string goal               # nom du but en cours ("" = aucun)
string goal_status        # "" | active | succeeded | aborted | canceled
float32 distance_remaining  # m (NaN si aucun but)
int32 recoveries          # reprises (spin, backup...) du but en cours
float32 localization_cov  # trace xy de la covariance AMCL (NaN hors navigation)
string[] waypoints        # points de la carte courante, dans l'ordre
string[] missing_sensors  # capteurs REQUIS absents ou muets (vide = tout va bien)
bool ready                # le sous-mode courant peut fonctionner
string detail             # message en clair (dernier evenement)
