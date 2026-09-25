# Resultat de reconnaissance du visage suivi (publie par episode / throttle).
# Promu tel quel du squelette tools/robot_controlv3/docs/ros2_interfaces/.
#
# C'est de la COMPARAISON D'EMPREINTES, pas de la classification : SFace produit un
# vecteur de 128 dimensions compare en cosinus a une galerie (gallery.npz). Un score
# eleve veut dire "proche de cette empreinte", pas "certain a X %".

std_msgs/Header header    # stamp = instant de la reconnaissance

string  status            # known / unknown / idle
int32   id_pred           # id personne, -1 si inconnu
string  name              # nom (ou "unknown")
float32 score             # cosinus LISSE (EMA 0,4), sert au badge et a l'hysteresis
float32 raw_score         # cosinus instantane non lisse
float32 stability         # taux de frames "known" sur l'episode (0..1)
string  id_lot            # lot de l'episode (acquisition)
uint32  lock_id           # identifiant d'episode
string  mode              # off / recognition / acquisition
# NB : la vignette (thumb 112x112) n'est PAS transportee ici -> topic image dedie
# si un HUD distant en a besoin.
