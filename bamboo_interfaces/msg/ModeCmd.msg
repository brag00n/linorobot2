# Commande de mode du groupe videotracking, emise par la manette ou par MCP.
#
# Semantique LATCHEE A SEQUENCE, reprise telle quelle du prototype roslite : une case
# de profondeur 1 n'agit qu'une fois par valeur de `seq`. Le producteur republie donc
# librement (le QoS transient_local rejoue la derniere valeur pour un abonne tardif)
# sans que l'action se repete : c'est `seq` qui declenche, pas l'arrivee du message.

std_msgs/Header header

# Cible du changement : "tracking" / "recognition" / "acquisition" / "training"
#                       / "tracker" / "detector" / "predict" / "recenter"
string  target

# Valeur visee. Vide = BASCULE (toggle) pour une cible booleenne, ou CYCLE vers la
# valeur suivante pour une cible enumeree (tracker, detector, predict) -- c'est ce que
# fait un appui de bouton. Une valeur explicite ("dnn", "coast", "on", "off") pose un
# etat absolu, ce qu'un client MCP veut pour etre idempotent.
string  value

uint32  seq           # incremente a CHAQUE appui ; une meme valeur n'agit qu'une fois
