# Telemetrie brute agregee de la carte ESP32 (diagnostic), reprise du snapshot() hote.
# Les grandeurs "propres" (odom, imu, batterie) sont publiees a part en messages standard ;
# ce message sert au debug et a l'observation directe via le MCP.
std_msgs/Header header

# Batterie (SYS_STATUS #1)
float32 battery_voltage   # V

# Attitude (ATTITUDE #30), DEGRES (reflet brut du snapshot hote ; imu/data la publie en rad)
float32 roll
float32 pitch
float32 yaw

# Vitesses chassis remontees par la carte (BAMBOO_WHEEL_STATE 42001), SI
float32 vx                # m/s
float32 vy                # m/s
float32 vz                # rad/s (lacet ; wz cote firmware)

# Comptage encodeur cumulatif par moteur (BAMBOO_ENCODERS 42003), M1..M4
int32[] encoders

# --- RELECTURE de la configuration reellement en vigueur dans la SRAM de la carte ---
# Ce n'est PAS un echo des parametres ROS : ces champs sont le resultat d'un
# PARAM_REQUEST_READ apres ecriture (idx 15-18 pour la geometrie, 0-2 pour le PID M1).
# C'est la seule preuve que la carte a pris la valeur -- l'ecriture seule ne prouve rien.
# La carte n'ayant AUCUNE persistance, le driver repousse cette config a chaque connexion.
bool board_config_ok      # false = la carte n'a pas repondu : valeurs ci-dessous non fiables
float32 board_cpr         # tics/tour (WHEEL_CPR)
float32 board_circ_mm     # circonference roue, mm BRUTS (WHEEL_CIRC)
float32 board_apb_mm      # DEMI-voie, mm BRUTS (WHEEL_APB) ; le firmware la redouble
float32 board_pid_kp      # gains du moteur M1 ; les 4 moteurs partagent un seul triplet
float32 board_pid_ki      #   sur cette carte (repliement d'index idx % 3)
float32 board_pid_kd
