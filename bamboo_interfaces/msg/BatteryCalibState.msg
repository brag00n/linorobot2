# Calibration de la batterie (bamboo_base/calibration_node), publiee LATCHEE sur
# /bamboo_calibration/battery une fois par seconde pendant un essai, et une fois au repos pour
# montrer le DERNIER resultat avant tout lancement. Lue par le HUD (boite BATTERIE).
std_msgs/Header header

string procedure          # batterie_plein | batterie_decharge | batterie_decharge_passive | ""
string phase              # idle | plein | decharge | calcul | fini | abandon
string activity           # activite en clair : Init... | Calib. batt. avec moteur... | Termine
float32 soc_pct           # % estime restant (NaN = inconnu)
float32 capacity_ah       # capacite mesuree du pack (NaN avant 10 % de decharge)
float32 capacity_ref_ah   # capacite du pack de bonne qualite pris comme reference
float32 quality_pct       # qualite du pack par rapport a la reference, 0..100 (NaN = inconnue)
string quality_label      # bon | moyen | faible | ?
float32 r_ohm             # resistance interne utilisee
bool r_estimated          # true : reference supposee, non mesuree (sans moteur)
float32 v_rest            # V, tension du pack
float32 current_a         # A consommes
float32 ah_used           # Ah consommes depuis le depart
float32 elapsed_s         # duree de l'essai
float32 remaining_s       # duree restante estimee, NaN = inconnue
bool remaining_firm       # false : duree PROVISOIRE (estimation encore instable)
string stop_reason        # "" tant qu'il tourne ; sinon la raison de l'arret
bool partial              # resultat PARTIEL (coupure, arret utilisateur)
string date               # date du resultat (phase idle) : "AAAA-MM-JJ HH:MM"
