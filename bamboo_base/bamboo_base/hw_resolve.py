#!/usr/bin/env python3
"""Resolution automatique des peripheriques par VID:PID, lecture (b) du parametrage auto.

Pourquoi ce module existe alors que les regles udev posent deja /dev/stm32 et
/dev/bamboocam : une regle udev vit sur L'HOTE. Elle est absente d'un RPi frais, absente
d'un poste de developpement Windows, et absente tant que install_bambooWS.sh n'a pas tourne
avec --apply. Un launch qui ecrirait "/dev/bamboocam" en dur ne demarrerait donc que sur une
machine deja preparee, et echouerait avec "No such file or directory" -- un message qui ne
dit RIEN de la cause reelle. Ici on prend le symlink s'il existe (c'est le chemin stable, on
ne le contourne pas) et on RETOMBE sur un balayage de /sys sinon, en nommant ce qu'on a
trouve.

Aucune dependance : stdlib seule. Ce fichier est importe par des launch ROS2, qui tournent
avant tout environnement applicatif, et l'image Docker n'a pas pip.

Ce module NE PARLE A AUCUN PERIPHERIQUE : il lit /sys et le systeme de fichiers, rien de
plus. Il peut donc tourner pendant que le driver tient le port serie -- le port serie de
cette carte est MONO-PROPRIETAIRE et une resolution qui l'ouvrirait pour verifier serait un
outil dangereux.

Usage en ligne de commande (c'est la verification V0 du plan) :
    python3 -m bamboo_base.hw_resolve
"""

import glob
import os

# --- Identifiants materiels du parc -------------------------------------------------
# Faits d'HOTE, pas de robot : ils ne vivent donc pas dans le fichier canonique
# config/robots/<robot>.yaml, qui ne porte que la verite physique du chassis.
CH340_STM32 = ("1a86", "7523")  # Yahboom ROS Control Board V3.0 (bambooSTM32YB)
CP2102_ESP32 = ("10c4", "ea60")  # WaveShare General Driver (BambooWS) -- VID:PID PARTAGE
UVC_BAMBOOCAM = ("05a3", "9331")  # webcam UVC de la camera motorisee


def _usb_ids(sysfs_path):
    """Remonte l'arborescence /sys jusqu'au peripherique USB et renvoie (vid, pid).

    Un noeud /sys/class/* pointe sur une INTERFACE USB (ex. .../1-1.2:1.0), pas sur le
    peripherique : idVendor et idProduct sont un ou plusieurs crans au-dessus. D'ou la
    remontee, bornee pour ne pas boucler si l'arbre n'est pas celui attendu.
    """
    node = os.path.realpath(sysfs_path)
    for _ in range(8):
        vid = os.path.join(node, "idVendor")
        pid = os.path.join(node, "idProduct")
        if os.path.isfile(vid) and os.path.isfile(pid):
            try:
                with open(vid) as fh:
                    v = fh.read().strip().lower()
                with open(pid) as fh:
                    p = fh.read().strip().lower()
                return (v, p)
            except OSError:
                return (None, None)
        parent = os.path.dirname(node)
        if parent == node or parent == "/":
            break
        node = parent
    return (None, None)


def _read_int(path, default=None):
    try:
        with open(path) as fh:
            return int(fh.read().strip())
    except (OSError, ValueError):
        return default


def resolve_camera(vid_pid=UVC_BAMBOOCAM, symlink="/dev/bamboocam"):
    """Renvoie le /dev/videoN de CAPTURE de la camera, ou None.

    LE PIEGE DE CETTE FONCTION, et il coute une heure a qui ne le connait pas : un
    peripherique UVC enumere PLUSIEURS noeuds /dev/videoN -- la capture ET, depuis le noyau
    4.16, un noeud de METADONNEES. Les deux portent le meme VID:PID. Prendre le premier
    trouve donne le noeud de metadonnees une fois sur deux, et l'ouverture echoue avec un
    message qui ne mentionne pas les metadonnees. On retient donc le plus petit `index`
    (0 = capture), meme critere que la regle udev, plutot que le plus petit numero de noeud.
    """
    if symlink and os.path.exists(symlink):
        return os.path.realpath(symlink)
    found = []
    for node in sorted(glob.glob("/sys/class/video4linux/video*")):
        if _usb_ids(node) != tuple(vid_pid):
            continue
        idx = _read_int(os.path.join(node, "index"), default=99)
        found.append((idx, "/dev/" + os.path.basename(node)))
    if not found:
        return None
    found.sort()
    return found[0][1]


def resolve_serial(vid_pid=CH340_STM32, symlink="/dev/stm32"):
    """Renvoie le /dev/ttyUSBn de la carte, ou None.

    A n'utiliser que comme INDICE : RobotComSerial sait deja se resoudre seul par VID:PID
    (`:38-40`, re-resolution automatique apres deconnexion `:61-64`), et c'est lui qui a
    raison en cas de desaccord -- il voit le peripherique reapparaitre sur un autre noeud,
    ce qu'une resolution faite une fois au lancement ne verrait pas.

    Ambiguite non resolue ICI, deliberement : le CP2102 10c4:ea60 est porte par plusieurs
    peripheriques du parc (ESP32 WaveShare ET adaptateur lidar), donc un appel avec ce
    VID:PID peut renvoyer le mauvais. Le CH340 etant seul de son espece sur ce banc, le cas
    par defaut est sans ambiguite. Pour le CP2102, passer par le symlink par numero de
    serie de 99-bambooWS.rules.
    """
    if symlink and os.path.exists(symlink):
        return os.path.realpath(symlink)
    for node in sorted(glob.glob("/sys/class/tty/ttyUSB*")) + sorted(
        glob.glob("/sys/class/tty/ttyACM*")
    ):
        if _usb_ids(os.path.join(node, "device")) == tuple(vid_pid):
            return "/dev/" + os.path.basename(node)
    return None


def main():
    """Impression lisible par un humain ET par un oeil de relecture de journal.

    On imprime meme ce qui est ABSENT, avec la raison probable : un resolveur muet sur un
    peripherique manquant laisse croire qu'il n'a pas tourne.
    """
    cam = resolve_camera()
    ser = resolve_serial()
    print("camera  (%s:%s) : %s" % (UVC_BAMBOOCAM + (cam or "ABSENTE",)))
    print("stm32   (%s:%s) : %s" % (CH340_STM32 + (ser or "ABSENT",)))
    if cam is None or ser is None:
        print("absent = materiel debranche, hors Linux, ou noyau sans /sys/class attendu")
    return 0 if (cam and ser) else 1


if __name__ == "__main__":
    raise SystemExit(main())
