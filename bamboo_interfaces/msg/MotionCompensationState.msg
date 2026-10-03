# Etat de la compensation de rotation d'un driver de carte (bamboo_base/motion_compensation).
# Publie a la cadence du driver. Sert a la CALIBRATION (vitesse roues BRUTE, jamais corrigee,
# face au gyro), au reglage et a l'analyse : il montre la consigne recue ET celle reellement
# envoyee a la carte, donc ce que la compensation a fait.
std_msgs/Header header

string variant          # variante materielle active (cle calib.* du fichier canonique)
bool identity           # true = tout a zero : comportement d'origine, rien n'est modifie
bool bypass             # true = neutralisee (calibration en cours)
bool armed              # enable_cmd_vel du driver

# Derniere consigne (corps du robot, conventions Twist) : recue puis envoyee
float32 vx_in           # m/s
float32 wz_in           # rad/s
float32 vx_out          # m/s
float32 wz_out          # rad/s
float32 slip            # s applique a cette consigne (0 = aucun)
float32 pi_term         # contribution du PI de lacet, rad/s
bool pi_active

# Mesures
float32 v_wheels        # m/s, moyenne gauche/droite des encodeurs
float32 w_wheels        # rad/s, (v_d - v_g) / ecartement : lacet VU DES ROUES, brut
float32 w_odom          # rad/s, lacet publie dans /odom (corrige si une table est chargee)
float32 gz              # rad/s, gyro z signe aux conventions ROS (NaN si absent)
float32 gz_age          # s (negatif si jamais recu)
float32 wheel_separation_m
