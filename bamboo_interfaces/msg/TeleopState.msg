# Etat de la manette pour l'affichage (HUD du flux augmente, OLED de l'ESP32). Publie LATCHE
# par bamboo_teleop sur /teleop/state, a chaque changement et au moins a 1 Hz.
# Les trois tableaux sont PARALLELES : keys[i] declenche labels[i], allume si active[i].
std_msgs/Header header

string mode             # mode applique, tel que lu sur /bamboo/mode
string group            # groupe d'actions courant du mode (croix haut/bas)
string[] groups         # groupes du mode, dans l'ordre de la croix
string[] keys           # "A", "X long", "ZR"...
string[] labels         # "acquisition", "tracker: mil"...
bool[] active           # etat on/off de l'action (false pour une action sans etat)
bool armed              # traction armee a la manette (appui long sur B)
bool busy               # un comportement long tourne (calibration)
string busy_text        # "CALIB 3/12"
