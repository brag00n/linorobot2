# Vitesse de rotation par moteur, deduite des encodeurs (tics/s / cpr * 60).
# La carte expose toujours 4 valeurs (M1..M4), mais leur REALITE depend du controleur :
#   - WSEsp32   : 2 voies reelles seulement -- M3 recopie M1 et M4 recopie M2 ;
#   - YBStm32v3 : 4 voies REELLES, les 4 valeurs sont independantes ;
#   - Teensy    : 4 voies reelles (cpr a confirmer).
# Ne pas deduire du nombre de champs que les 4 mesures sont independantes : c'est la
# fiche materiel du controleur qui tranche.
std_msgs/Header header
float64[] rpm
