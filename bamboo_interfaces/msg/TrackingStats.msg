# Mesure des performances du groupe videotracking (exigence "performances mesurees").
#
# Publie a 1 Hz en RELIABLE, et c'est structurel : le profil `sensor_data` est
# best-effort donc INVISIBLE a travers rosbridge, qui souscrit en reliable (meme piege
# que imu/data au chantier 1, cf. esp32_driver.yaml:38-41). Tout ce qui doit etre
# diagnosticable depuis MCP publie en reliable.

std_msgs/Header header

# --- cadences par etage du pipeline (Hz, moyennees sur la fenetre de publication) ---
float32 capture_fps      # trames recues de gscam2 (passthrough MJPG)
float32 detect_fps       # cycles de tracking_node (imdecode + YuNet ou tracker)
float32 recog_fps        # cycles de facerecog_node (SFace ; 0 si mode off)
float32 overlay_fps      # trames annotees emises par overlay_node

# --- latence BOUT EN BOUT : publication de la trame annotee - header.stamp de la
# trame brute d'origine. C'est la grandeur qui compte pour un asservissement : une
# cadence elevee avec une latence elevee fait osciller la loi pan/tilt.
float32 latency_ms
float32 latency_ms_max   # pire cas sur la fenetre ; la moyenne cache les decrochages

# --- qualite du suivi ---
float32 loss_rate        # fraction de cycles SANS verrou sur la fenetre (0..1)
string  lock_source      # off / detect / track / reanchor / recenter / redetect
string  unlock_reason    # derniere cause de deverrou : update_ko / degenerate /
                         # implausible / score / det_miss / hold_timeout / switch

# --- charge, pour distinguer "l'algorithme est lent" de "la machine est saturee" ---
float32 cpu_percent      # 0 si non mesure
uint32  dropped_frames   # trames brutes ecartees faute de temps, depuis le demarrage
