# Etat du suivi de cible, publie par tracking_node.
#
# Promu du squelette tools/robot_controlv3/docs/ros2_interfaces/msg/TrackState.msg avec
# DEUX ECARTS ASSUMES :
#  1. le squelette n'avait PAS de Header ; la latence bout en bout de TrackingStats se
#     mesure sur l'horodatage de TRAME -> il en faut un ici ;
#  2. le squelette laissait la boite englobante a une vision_msgs/Detection2DArray
#     publiee conjointement. `vision_msgs` n'est PAS dans l'image Docker et le lot V1
#     ne l'y ajoute pas -> la boite est portee ICI, en pixels image. Le jour ou
#     vision_msgs entre dans l'image, ces 4 champs deviennent redondants et non faux :
#     la Detection2DArray se publiera EN PLUS, sans casser ce message.

std_msgs/Header header    # stamp = horodatage de la TRAME, pas de la publication :
                          # c'est lui qui rend la latence bout en bout mesurable

string  mode          # none / mil / vit / auto  (tracker demande)
bool    locked        # verrou stable en cours
string  src           # detector / tracker / predicted
float32 score         # score de suivi courant

# --- cible, NORMALISEE : repere identique au prototype (RobotWebCamMotorized:341-349)
# nx = (cx - fw/2) / (fw/2), + = DROITE de l'image
# ny = (cy - fh/2) / (fh/2), + = BAS de l'image
# C'est cette convention que la loi pan/tilt consomme ; la changer inverserait
# l'asservissement sans qu'aucun test unitaire ne le voie.
float32 nx
float32 ny
float32 area_pct      # aire de la boite en % de l'image ; sert a la plausibilite

# --- boite englobante en PIXELS image (cf. ecart 2 ci-dessus) ---
int32   bbox_x
int32   bbox_y
int32   bbox_w
int32   bbox_h

# --- prediction de trajectoire (modes anticip / coast) -- touche P du prototype ---
string  pred_phase    # off / anticip / coast
float32 pred_nx       # cible anticipee, meme normalisation que nx/ny
float32 pred_ny

# --- ajouts du lot V3, requis par des CONSOMMATEURS et non par le suivi lui-meme -----
# lock_id est le NUMERO D'EPISODE de verrou : il n'incremente qu'a une prise de verrou.
# C'est lui, et rien d'autre, qui dit a facerecog_node "c'est peut-etre une autre
# personne, vide ta moyenne glissante". Le deduire de `locked` cote consommateur serait
# faux : deux episodes separes par une seule trame perdue sont indiscernables autrement,
# et l'EMA de l'ancien visage contaminerait le nouveau.
uint32  lock_id
# Age du verrou courant, en secondes. Porte ici parce que la reconnaissance n'est armee
# qu'au-dela d'un seuil (2 s dans le prototype) : le recalculer cote consommateur
# demanderait de memoriser l'instant de prise, donc de dupliquer un etat qui existe deja.
float32 lock_age_s
# Cause du DERNIER deverrouillage (update_ko / degenerate / implausible / score /
# det_miss / hold_timeout / switch), vide si jamais perdu. Alimente `unlock_reason` de
# TrackingStats : sans elle, une perte de cible se voit mais ne s'explique pas.
string  lost_reason
# Nombre de visages detectes dans la trame, verrou ou pas. Distingue "personne devant la
# camera" de "plusieurs personnes et le verrou hesite", deux pannes qui se ressemblent.
int32   n_faces
