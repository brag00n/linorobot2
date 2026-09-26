"""Controleur Teensy + UGeek Motor HAT : module DECLARATIF, sans driver ROS.

Ce que ce module contient reellement : `config/capabilities.yaml`, c'est-a-dire la
declaration de ce que la carte sait faire (4 moteurs, 4 encodeurs reels, MPU6050 SANS
magnetometre -- donc aucun cap absolu -- servos differes). Ce que le microcode sait faire
est deja ecrit et compile ([[teensy-binary-frame-mode]]) ; ce qui manque, c'est un driver
ROS et surtout UN ROBOT MONTE sur lequel l'essayer.

POURQUOI CE LAUNCH REFUSE DE DEMARRER plutot que de ne rien lancer : un launch qui reussit
en ne creant aucun noeud est le pire des deux mondes -- le bringup passe, les topics
n'existent pas, et le diagnostic part chercher un probleme de decouverte DDS. On echoue
donc ici, en nommant le module et ce qui manque.

Ce refus n'est pas un garde-fou de securite, c'est un constat d'absence : le jour ou le
driver existe, il se branche ici et les robots qui incluent ce launch n'ont pas une ligne
a changer.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

_MESSAGE = (
    "bamboo_controler_Teensy est un module DECLARATIF : il porte capabilities.yaml mais "
    "AUCUN driver ROS, et aucun robot n'est monte sur cette carte. Le microcode existe "
    "(transport a trames binaires, MAVLink sysid 3) ; le noeud hote reste a ecrire. "
    "Pour lever ce refus il faut un driver, pas un contournement de launch. En attendant, "
    "lancer le robot avec enable_driver:=false, ou viser bamboo_controler_YBStm32v3."
)


def _refuse(context, *args, **kwargs):
    raise RuntimeError(_MESSAGE)


def generate_launch_description():
    return LaunchDescription([
        # Les arguments sont declares MALGRE le refus : l'interface du module est deja
        # figee (tout module de controleur prend `robot` et `enable_cmd_vel`), et un
        # appelant doit pouvoir la respecter des maintenant.
        DeclareLaunchArgument("robot", default_value="",
                             description="Source unique du robot (sans .yaml)."),
        DeclareLaunchArgument("enable_cmd_vel", default_value="false",
                             description="Actuation moteur. Sans objet tant qu'aucun "
                                         "driver n'existe."),
        OpaqueFunction(function=_refuse),
    ])
