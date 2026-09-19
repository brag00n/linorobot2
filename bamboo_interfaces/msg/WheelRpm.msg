# Vitesse de rotation par moteur, deduite des encodeurs (tics/s / cpr * 60).
# La carte expose 4 valeurs (M1..M4) ; cote encodeur physique M3 recopie M1 et M4 recopie M2
# (2 voies reelles gauche/droite seulement, cf. fiche materiel).
std_msgs/Header header
float64[] rpm
