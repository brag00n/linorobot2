# Etat de la manette pour l'affichage (HUD du flux augmente, OLED de l'ESP32). Publie LATCHE
# par bamboo_teleop sur /teleop/state, a chaque changement et au moins a 1 Hz.
# Les tableaux sont PARALLELES : keys[i] declenche labels[i], allume si active[i], en cours
# (clignotant) si pending[i].
std_msgs/Header header

string mode             # mode applique, tel que lu sur /bamboo/mode
string group            # groupe d'actions courant du mode (croix haut/bas)
string[] groups         # groupes du mode, dans l'ordre de la croix
string[] keys           # "A", "X long", "ZR"...
string[] labels         # "acquisition", "tracker: mil"...
bool[] active           # etat on/off de l'action (false pour une action sans etat)
bool[] pending          # commande envoyee, reponse pas encore recue (demarrage / arret)
bool armed              # traction armee a la manette (appui long sur B)
bool armed_latched      # armement VERROUILLE (B long + L long) : la perte de la manette ne
                        # desarme plus ; pastille ARME B clignotante (2026-10-13)
string confirm_text     # CONFIRMATION demandee ("" = aucune) : A = oui, B = non (2026-10-13)
string nav_stack        # pile NAV (A long) : off | starting | on | stopping -- pastille NAV
bool busy               # un comportement long tourne (calibration)
string busy_text        # "CALIB 3/12"
