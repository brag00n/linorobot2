"""Partie PYTHON du paquet bamboo_videotracking (hors chemin temps reel).

Le chemin chaud (decodage, detection, suivi, prediction, reconnaissance,
incrustation, loi de commande servo) est en C++ dans `src/`, compose intra-process.
Ici ne vit que l'APPRENTISSAGE : il n'est pas temps reel, il est long et annulable,
et le reecrire en C++ serait lourd pour zero gain. Il est donc recopie du prototype
`robot_control/interaction/` tel quel.

DUPLICATION ASSUMEE avec `linorobot2_hardware/tools/robot_control/interaction/`
(choix utilisateur : copier dans les paquets ROS). Une correction d'algorithme doit
donc etre portee DEUX FOIS -- c'est le prix de l'independance des deux depots.
"""
