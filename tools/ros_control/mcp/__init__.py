"""Serveurs MCP *ROS 2* de ce depot (graphe ROS et services Docker du robot).

Trois serveurs, une seule frontiere : tout passe par rosbridge ou par SSH, jamais par un
port serie -- celui-ci est mono-proprietaire et appartient au driver.

  ros2.py         ros2-analysis      LECTURE SEULE du graphe (rosbridge :9090)
  ros2_action.py  ros2-action        ACTUATION bornee (publish, services, parametres)
  orchestrator.py ros2-orchestrator  services Docker Compose du RPi (allow-list, SSH)

La telemetrie de l'app Windows (`robot-analysis`, `robot-action`) reste dans l'autre
depot : elle lit des fichiers locaux et ouvre le COM, elle n'a rien a faire ici.
"""
