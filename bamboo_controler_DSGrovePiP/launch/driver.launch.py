"""Controleur GrovePi+ (ATmega328P) : module DECLARATIF, sans driver ROS.

Cette carte n'est PAS un controleur de mobilite : c'est une carte CAPTEURS detournee --
aucun moteur, aucun encodeur, un MPU6050 sans magnetometre (donc roll et pitch seulement,
JAMAIS de yaw) et 4 HC-SR04 + un Sharp IR qui donnent 5 DISTANCE_SENSOR a 18 Hz. Cette
chaine est MESUREE et fonctionne sur le RPi ([[grovepi-sensor-board]]) ; ce qui manque est
le noeud ROS qui la republie.

DEUX PIEGES MATERIELS, repetes ici parce qu'un lecteur de launch ne lit pas la doc :
  1. NE JAMAIS enficher la carte sur le header du RPi -- son RESET est sur BCM8 et la
     carte reste bloquee. Liaison UART SEULEMENT, par le port Grove SERIAL.
  2. Son BAUD CAPTEURS n'est PAS le baud de controle des autres cartes. Confusion deja
     commise une fois, et elle se manifeste par un silence, pas par une erreur.

POURQUOI CE LAUNCH REFUSE DE DEMARRER : voir bamboo_controler_Teensy/launch/driver.launch.py
-- un launch qui reussit sans creer de noeud envoie le diagnostic sur une fausse piste.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

_MESSAGE = (
    "bamboo_controler_DSGrovePiP est un module DECLARATIF : il porte capabilities.yaml "
    "mais AUCUN driver ROS. La carte est une carte CAPTEURS (0 moteur, 0 encodeur, pas de "
    "magnetometre donc pas de yaw) ; son microcode MAVLink sysid 4 emet deja 5 "
    "DISTANCE_SENSOR a 18 Hz, le noeud hote reste a ecrire. Rappel materiel : UART par le "
    "port Grove SERIAL UNIQUEMENT, jamais le header RPi (RESET sur BCM8)."
)


def _refuse(context, *args, **kwargs):
    raise RuntimeError(_MESSAGE)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="",
                             description="Source unique du robot (sans .yaml)."),
        # Declare pour l'uniformite de l'interface, mais cette carte n'a AUCUN moteur :
        # un robot qui ne composerait qu'elle ne doit pas avoir de souscripteur /cmd_vel.
        DeclareLaunchArgument("enable_cmd_vel", default_value="false",
                             description="Sans objet : 0 moteur sur cette carte."),
        OpaqueFunction(function=_refuse),
    ])
