# Consigne d'un axe de servomoteur -- CONTRAT ROS, PAS UNE CARTE.
#
# Le producteur (servocam_node, la manette, MCP, un futur noeud de bras) adresse un
# axe PAR SON NOM et ignore deliberement quelle carte l'execute. C'est la table
# `servo_axes` du fichier canonique du robot -- seul endroit du depot qui lie un axe
# a une carte -- qui resout `axis` en (controleur, voie materielle, bornes).
# Deplacer les servos sur une autre carte ne change donc QUE cette table.
#
# Le meme type sert de consigne (/servo/cmd) et d'etat rendu (/servo/state).
# ATTENTION /servo/state est une OMBRE D'HOTE : aucune carte du projet ne sait relire
# l'angle d'un servo PWM (pas de SERVO_OUTPUT_RAW #36, pas d'index PARAM servo), donc
# cet etat est ce que l'hote a EMIS, jamais ce que le servo a ATTEINT.

std_msgs/Header header

# Nom de l'axe : "pan", "tilt", ... Doit exister dans `servo_axes`, sinon l'executeur
# REJETTE en le journalisant -- il n'avale pas la consigne en silence.
string  axis

float32 angle_deg     # degres ; absolu par defaut, increment si `relative`
bool    relative      # true = angle_deg est un pas relatif a l'angle courant

# Voie materielle, FACULTATIVE : 0 = "laisse l'executeur resoudre par `servo_axes`",
# et c'est le cas normal. Un producteur qui remplirait ce champ a la main recreerait
# le couplage carte que ce message existe pour retirer -> ne le remplir que pour du
# diagnostic bas niveau.
uint8   id 0
