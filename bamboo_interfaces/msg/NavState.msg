# Etat de la navigation (bamboo_navigation/nav_supervisor), publie LATCHE sur /navigation/state
# a chaque changement et au moins a 1 Hz. Lu par le HUD (carte NAV, mini-carte), l'OLED, la
# manette et le MCP.
std_msgs/Header header

string submode            # idle | exploration | navigation
string map_name           # carte COURANTE (celle qu'on complete ou sur laquelle on navigue)
string goal               # nom du but en cours ("" = aucun)
string goal_status        # "" | active | succeeded | aborted | canceled
float32 distance_remaining  # m (NaN si aucun but)
int32 recoveries          # reprises (spin, backup...) du but en cours
float32 localization_cov  # (historique, AMCL) : NaN depuis que slam_toolbox localise
string[] waypoints        # noms des DESTINATIONS de la carte courante, dans l'ordre
string[] missing_sensors  # capteurs REQUIS absents ou muets (vide = tout va bien)
bool ready                # le sous-mode courant peut fonctionner
string detail             # message en clair (dernier evenement)

# --- localisation (2026-10-13) ---
string loc_state          # inconnue | ok | douteuse | relocalisation | perdue
float32 map_match         # accord du scan avec la carte, 0..1 (NaN si non mesure)

# --- selection a la manette (L / R) : gate en exploration, destination en navigation ---
string selected_kind      # "" | gate | dest
int32 selected_n          # numero de la gate / destination selectionnee (0 = aucune)
string selected_name      # nom de la destination ("" pour une gate)

# --- objets de la carte courante, pour la mini-carte (tableaux paralleles, repere map) ---
int32[] gate_n            # GATES : zones inconnues a explorer, numerotees
float32[] gate_x
float32[] gate_y
string[] gate_status      # ouverte | en_cours | atteinte | exploree | abandonnee | inatteignable
int32[] dest_n            # DESTINATIONS
string[] dest_name
float32[] dest_x
float32[] dest_y
