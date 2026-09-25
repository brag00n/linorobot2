# Telemetrie brute agregee de la carte Yahboom ROS Control Board V3.0 (STM32F103RCT6).
# Pendant de Esp32Status pour l'autre carte. Les grandeurs "propres" (odom, imu, mag,
# batterie) sont publiees a part en messages standard ; ce message sert au debug et a
# l'observation directe via le MCP.
#
# TROIS ECARTS DE FOND avec Esp32Status, tous en faveur de cette carte -- le mappage
# les EXPOSE au lieu de les niveler :
#   1. les 4 encodeurs sont REELS (l'ESP32 recopie les voies 3/4 depuis 1/2) ;
#   2. les gains PID sont PAR MOTEUR (idx / 3 cote firmware), pas un triplet commun ;
#   3. la carte sait PERSISTER en flash (PREFLIGHT_STORAGE), l'ESP32 non.
std_msgs/Header header

# Batterie (SYS_STATUS #1). Seuils du microcode : 9,6 V bas / 13,0 V haut, et une
# fenetre 6,5-8,5 V FILTREE (heritee des packs 2S) -> la bande 8,6-9,5 V est un trou
# ou la carte fait un arret LATCHANT en 2 s, recuperable seulement par reset.
float32 battery_voltage   # V

# Attitude (ATTITUDE #30), DEGRES (reflet brut du snapshot hote ; imu/data la publie
# en rad). Le MPU9250 a un MAGNETOMETRE -> ce yaw peut etre un cap ABSOLU, ce qui est
# impossible sur le MPU6050 du Teensy et de la GrovePi+.
float32 roll
float32 pitch
float32 yaw

# Vitesses chassis remontees par la carte (BAMBOO_WHEEL_STATE 42001), SI
float32 vx                # m/s
float32 vy                # m/s
float32 vz                # rad/s (lacet ; wz cote firmware)

# Comptage encodeur cumulatif, M1..M4 -- 4 VOIES REELLES sur cette carte.
int32[] encoders

# --- RELECTURE de la configuration reellement en vigueur dans la SRAM de la carte ---
# Ce n'est PAS un echo des parametres ROS : c'est le resultat d'un PARAM_REQUEST_READ
# APRES ecriture (idx 15-18 pour la geometrie, 0-11 pour les PID). C'est la seule
# preuve que la carte a pris la valeur -- l'ecriture seule ne prouve rien, et un
# COMMAND_ACK non plus (cf. servo ci-dessous).
bool board_config_ok      # false = la carte n'a pas repondu : champs suivants non fiables
float32 board_cpr         # tics/tour (WHEEL_CPR) ; 1320 = 30 x 11 x 4 sur ce robot
float32 board_circ_mm     # circonference roue, mm BRUTS (WHEEL_CIRC)
float32 board_apb_mm      # DEMI-somme voie+empattement, mm BRUTS (WHEEL_APB)

# Gains PID PAR MOTEUR, M1..M4 : 4 triplets, pas un seul comme sur l'ESP32.
float32[] board_pid_kp
float32[] board_pid_ki
float32[] board_pid_kd

# --- OMBRE D'HOTE des angles servo, S1..S4 ---
# L'angle est en ECRITURE SEULE sur cette carte : PwmServo_Get_Angle() existe
# (bsp_pwmServo.c:143-146) mais n'a AUCUN appelant, il n'y a pas de SERVO_OUTPUT_RAW
# (#36) ni d'index PARAM servo. Ces valeurs sont donc ce que l'hote a EMIS. Un decalage
# mecanique reste invisible du logiciel, et un COMMAND_ACK renvoie MAV_RESULT_ACCEPTED
# MEME quand le firmware a jete la valeur (bsp_pwmServo.c:153 : if (angle > 180) return;)
# -> un ACK ne prouve PAS que l'angle a ete applique.
float32[] servo_angle_deg
