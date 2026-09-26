#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""orchestrator.py - Serveur MCP (stdio) d'ORCHESTRATION de la stack ROS2 du RPi.

Quatrieme serveur MCP du projet. La ou `ros2-analysis` interroge le graphe ROS2 (via
rosbridge), l'orchestrateur DEMARRE/ARRETE les pipelines. Chez nous un pipeline n'est PAS
un node ROS isole mais un **service Docker Compose** (camera.real, camera.h264, rosbridge,
driver.real, slam.real, navigation.real...), chacun dans son conteneur. Ni le lifecycle ROS
ni les component containers ne traversent la frontiere du conteneur -> le "manager" naturel
est l'orchestration Docker : `docker compose up -d / stop / ps` lance en SSH sur le RPi.

  RPI_SSH_HOST     : cible SSH (defaut dietpi@192.168.1.164).
  RPI_SSH_KEY      : cle privee (defaut ~/.ssh/id_rpi_grovepi).
  RPI_COMPOSE_DIR  : dossier du docker-compose.yaml sur le RPi.
  RPI_SSH_TIMEOUT  : timeout connexion SSH en s (defaut 30).

GARDE-FOUS :
  - ALLOW-LIST stricte de services (SERVICES) : tout nom hors liste est refuse -> pas
    d'injection shell (les noms ne contiennent que [a-z0-9._-], valides avant tout appel).
  - PAS de backdoor d'actuation : demarrer `driver.real` le lance en LECTURE SEULE
    (enable_cmd_vel=false par defaut, cf. plan barriere L6). Ce serveur ne publie aucun
    /cmd_vel et ne modifie aucun parametre : il ne fait que up/stop/ps/logs des services.

Outils :
  services / status      -> etat des services (lecture)
  start / stop / restart -> cycle de vie d'un service (allow-list)
  logs                   -> dernieres lignes d'un service (lecture)

Lancement (via .mcp.json) :
    python -m ros_control.mcp.orchestrator
Test manuel (necessite le RPi joignable en SSH) :
    python -m ros_control.mcp.orchestrator selftest

DEMENAGE le 2026-09-26 (lot 3.0) depuis `linorobot2_hardware/tools/robot_control/mcp/` :
les services qu'il pilote sont ceux de `linorobot2/docker/docker-compose.yaml`, il vit donc
dans le depot qui porte ce fichier. Corollaire utile : l'allow-list et le compose ne peuvent
plus diverger d'un depot a l'autre sans que le meme commit les voie tous les deux.

DETTE D'IDENTITE, connue et assumee ici (lot 3.4) : la cible est declaree DEUX FOIS pour un
seul robot -- `RPI_SSH_HOST` ici et `ROS2_BRIDGE_URL` dans ros2.py/ros2_action.py -- et
AUCUN serveur MCP ne lit `bamboo_base/config/robots/<robot>.yaml`, qui est pourtant la
source unique cote ROS. Changer de machine demande donc d'editer deux variables du
`.mcp.json` en accord. A resorber quand un robot aura plus d'une adresse a porter.
"""
import os
import re
import subprocess
import sys
import time

from . import _rpc

SERVER_NAME = "ros2-orchestrator"
_LOG = _rpc.make_log("ros2-orchestrator")

SSH_HOST = os.environ.get("RPI_SSH_HOST", "dietpi@192.168.1.164")
SSH_KEY = os.path.expanduser(os.environ.get("RPI_SSH_KEY", "~/.ssh/id_rpi_grovepi"))
COMPOSE_DIR = os.environ.get(
    "RPI_COMPOSE_DIR",
    "/home/dietpi/prj_robotique/Bamboo4WD_V4/linorobot2/docker")
SSH_TIMEOUT = int(os.environ.get("RPI_SSH_TIMEOUT", "30"))

# `docker compose` a besoin de sudo -n sur le RPi (dietpi n'est pas dans le groupe docker).
COMPOSE = "sudo -n docker compose"

# ALLOW-LIST : seuls ces services (daemons durables du compose) sont pilotables. Les
# services one-shot/interactifs (savemap, teleop, bash, rviz, build.*) sont volontairement
# exclus. La description sert au libelle de l'outil et au resume `status`.
#
# Le plan (lot 3.3) demandait d'ajouter `bringup.real` ET `teleop` : seul le premier entre
# ici, et c'est un desaccord assume avec le plan, pas un oubli. `teleop` est
# `teleop_twist_keyboard` (docker-compose.yaml:193-198), un programme qui LIT STDIN : demarre
# en detache par `docker compose start`, il tourne sans clavier et ne publie jamais rien --
# un outil qui le "demarrerait" mentirait. L'equivalent DURABLE d'un teleop est le service
# manette (`drive.real`, lot 5.6 du chantier 1) est ECRIT depuis le 2026-09-26, et il est
# donc ENTRE -- c'est la promesse tenue. Il demarre en LECTURE SEULE (enable_cmd_vel=false
# par defaut, BAMBOO_ARM absent de .env) : le demarrer ne peut pas faire tourner une roue.
SERVICES = {
    "rosbridge": "pont rosbridge_suite :9090 (client MCP ros2-analysis)",
    "foxglovebridge": "pont foxglove_bridge :8765 (client Foxglove Studio)",
    "camera.real": "camera ROS : v4l2_camera -> /image_raw + web_video_server :8080 (~12fps, +metrique FPS)",
    "camera.mjpg": "camera ROS 30fps : gscam2 MJPG passthrough -> /image_raw/compressed + web_video_server :8080",
    "camera.h264": "camera H.264 distante : MediaMTX RTSP :8554 / HLS :8888 / WebRTC :8889",
    "bringup.real": "bringup AMONT linorobot2 (micro-ROS agent sur /dev/ttyACM0) -- N'EST "
                    "PAS notre pont MAVLink : EXCLUSIF avec driver.real et driver.stm32 "
                    "(un seul proprietaire du port serie)",
    "driver.real": "driver ESP32 (pont MAVLink) LECTURE SEULE (enable_cmd_vel=false)",
    "drive.real": "traction MANETTE de BambooWS : driver ESP32 + joy_linux + teleop_twist_joy "
                  "dans UN conteneur -- LECTURE SEULE (enable_cmd_vel=false ; armer demande "
                  "BAMBOO_ARM=true, hors de portee de cet outil). EXCLUSIF avec driver.real, "
                  "driver.stm32 et bringup.real (un seul proprietaire du port serie)",
    "driver.stm32": "driver STM32 Yahboom v3 (pont MAVLink sysid 1) LECTURE SEULE (enable_cmd_vel=false)",
    "bamboo_video": "groupe video : acquisition + mux + streaming externe :8080 (autonome)",
    "bamboo_videotracking": "groupe tracking : Tracking/FaceRecog/FaceTrain/ServoCam, s'enregistre sur bamboo_video",
    "bamboo_control": "groupe manette : joy_linux_node + bamboo_teleop -- modes du tracking et pas relatifs des axes servo, AUCUN moteur de traction",
    "bringup.stm32": "bringup COMPLET en un conteneur (banc) : EXCLUSIF avec bamboo_video/bamboo_videotracking/driver.stm32",
    "robotdesc": "robot_state_publisher (description URDF)",
    "slam.real": "cartographie SLAM (slam_toolbox)",
    "navigation.real": "navigation Nav2 sur carte existante",
}

# Noms de services surs : lettres, chiffres, point, tiret, underscore. Double barriere avec
# l'allow-list (ceinture + bretelles) contre toute injection dans la commande distante.
_SAFE_NAME = re.compile(r"^[A-Za-z0-9._-]+$")


class OrchestratorError(RuntimeError):
    """Erreur d'orchestration (SSH injoignable, service refuse, commande en echec)."""


# ---------------------------------------------------------------------------
# Execution SSH (sans shell local : argv liste ; la commande distante est une chaine
# unique passee a ssh, sure car construite a partir de noms deja valides)
# ---------------------------------------------------------------------------
def _ssh(remote_cmd, timeout=None):
    """Lance `cd <dir> && <remote_cmd>` sur le RPi. Renvoie (rc, stdout, stderr)."""
    argv = [
        "ssh", "-i", SSH_KEY,
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=%d" % SSH_TIMEOUT,
        "-o", "ServerAliveInterval=10",
        SSH_HOST,
        "cd %s && %s" % (COMPOSE_DIR, remote_cmd),
    ]
    try:
        p = subprocess.run(argv, capture_output=True, text=True,
                           timeout=timeout or (SSH_TIMEOUT + 30))
    except subprocess.TimeoutExpired:
        raise OrchestratorError(
            "timeout SSH (%ss) vers %s. RPi joignable ? cle %s ?"
            % (timeout or (SSH_TIMEOUT + 30), SSH_HOST, SSH_KEY))
    except FileNotFoundError:
        raise OrchestratorError("client 'ssh' introuvable dans le PATH Windows.")
    return p.returncode, p.stdout, p.stderr


def _check_service(name):
    """Valide un nom de service contre l'allow-list (+ forme sure). Leve sinon."""
    if not name:
        raise OrchestratorError("nom de service manquant.")
    if not _SAFE_NAME.match(name):
        raise OrchestratorError("nom de service invalide : %r" % name)
    if name not in SERVICES:
        raise OrchestratorError(
            "service '%s' hors allow-list. Services pilotables : %s"
            % (name, ", ".join(sorted(SERVICES))))
    return name


def _compose_state():
    """Etat courant : dict {service -> (state, status)} depuis `compose ps -a`.
    Les warnings DISPLAY de compose partent sur stderr -> ignores (on parse stdout)."""
    rc, out, err = _ssh(
        "%s ps -a --format '{{.Service}}\t{{.State}}\t{{.Status}}'" % COMPOSE)
    if rc != 0:
        raise OrchestratorError("`compose ps` a echoue (rc=%d) : %s"
                                % (rc, (err or out).strip()[:400]))
    states = {}
    for line in out.splitlines():
        parts = line.split("\t")
        if len(parts) >= 2 and parts[0]:
            states[parts[0]] = (parts[1], parts[2] if len(parts) > 2 else "")
    return states


# ---------------------------------------------------------------------------
# Outils MCP
# ---------------------------------------------------------------------------
def _render_services(states):
    lignes = []
    for svc in sorted(SERVICES):
        state, status = states.get(svc, ("absent", "non cree"))
        marque = "[on ]" if state == "running" else "[off]"
        lignes.append("%s %-16s %-9s %s" % (marque, svc, state, status))
        lignes.append("    %s" % SERVICES[svc])
    return "\n".join(lignes)


def t_services(_args):
    """Liste les services pilotables et leur etat (lecture seule)."""
    return _render_services(_compose_state())


def t_status(_args):
    """Synthese : combien de services actifs, lesquels."""
    states = _compose_state()
    up = [s for s in SERVICES if states.get(s, ("", ""))[0] == "running"]
    down = [s for s in SERVICES if s not in up]
    return ("Actifs (%d) : %s\nInactifs (%d) : %s"
            % (len(up), ", ".join(up) or "-", len(down), ", ".join(down) or "-"))


def t_start(args):
    """Demarre un service en arriere-plan (`compose up -d <service>`)."""
    svc = _check_service(args.get("service"))
    rc, out, err = _ssh("%s up -d %s" % (COMPOSE, svc), timeout=180)
    if rc != 0:
        raise OrchestratorError("demarrage %s echoue (rc=%d) : %s"
                                % (svc, rc, (err or out).strip()[:400]))
    time.sleep(2.0)
    state = _compose_state().get(svc, ("?", ""))
    return "service '%s' demarre -> etat=%s (%s)" % (svc, state[0], state[1])


def t_stop(args):
    """Arrete un service (`compose stop <service>`), conteneur conserve."""
    svc = _check_service(args.get("service"))
    rc, out, err = _ssh("%s stop %s" % (COMPOSE, svc), timeout=120)
    if rc != 0:
        raise OrchestratorError("arret %s echoue (rc=%d) : %s"
                                % (svc, rc, (err or out).strip()[:400]))
    return "service '%s' arrete." % svc


def t_restart(args):
    """Redemarre un service. stop -> pause 3s -> up : evite la course 'device busy'
    sur /dev/video0 (camera.real, v4l2_camera ne reessaie pas l'ouverture)."""
    svc = _check_service(args.get("service"))
    _ssh("%s stop %s" % (COMPOSE, svc), timeout=120)
    time.sleep(3.0)
    rc, out, err = _ssh("%s up -d %s" % (COMPOSE, svc), timeout=180)
    if rc != 0:
        raise OrchestratorError("redemarrage %s echoue (rc=%d) : %s"
                                % (svc, rc, (err or out).strip()[:400]))
    time.sleep(2.0)
    state = _compose_state().get(svc, ("?", ""))
    return "service '%s' redemarre -> etat=%s (%s)" % (svc, state[0], state[1])


def t_logs(args):
    """Dernieres lignes de log d'un service (lecture seule)."""
    svc = _check_service(args.get("service"))
    n = int(args.get("lines", 40))
    n = max(1, min(n, 400))
    rc, out, err = _ssh("%s logs --tail=%d --no-color %s" % (COMPOSE, n, svc), timeout=60)
    txt = (out or "").strip() or (err or "").strip()
    if rc != 0 and not txt:
        raise OrchestratorError("logs %s indisponibles (rc=%d)." % (svc, rc))
    return "logs %s (%d dernieres lignes) :\n%s" % (svc, n, txt[-6000:])


# ---------------------------------------------------------------------------
# Enregistrement MCP
# ---------------------------------------------------------------------------
_SVC_ENUM = sorted(SERVICES)
_SVC_DESC = "service a piloter (allow-list : %s)" % ", ".join(_SVC_ENUM)

TOOLS = [
    {"name": "services",
     "description": "Liste les pipelines pilotables (services Docker Compose) et leur etat "
                    "(running/exited/absent). Lecture seule.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "status",
     "description": "Synthese courte : quels services sont actifs / inactifs.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "start",
     "description": "Demarre un service en arriere-plan (docker compose up -d). "
                    "Demarrer driver.real le lance en LECTURE SEULE (enable_cmd_vel=false).",
     "inputSchema": {"type": "object",
                     "properties": {"service": {"type": "string", "enum": _SVC_ENUM,
                                                 "description": _SVC_DESC}},
                     "required": ["service"]}},
    {"name": "stop",
     "description": "Arrete un service (docker compose stop). Le conteneur est conserve.",
     "inputSchema": {"type": "object",
                     "properties": {"service": {"type": "string", "enum": _SVC_ENUM,
                                                 "description": _SVC_DESC}},
                     "required": ["service"]}},
    {"name": "restart",
     "description": "Redemarre un service (stop puis up, avec pause anti 'device busy').",
     "inputSchema": {"type": "object",
                     "properties": {"service": {"type": "string", "enum": _SVC_ENUM,
                                                 "description": _SVC_DESC}},
                     "required": ["service"]}},
    {"name": "logs",
     "description": "Dernieres lignes de log d'un service (lecture seule).",
     "inputSchema": {"type": "object",
                     "properties": {"service": {"type": "string", "enum": _SVC_ENUM,
                                                 "description": _SVC_DESC},
                                    "lines": {"type": "integer",
                                              "description": "nb de lignes (1..400, defaut 40)"}},
                     "required": ["service"]}},
]

HANDLERS = {
    "services": t_services,
    "status": t_status,
    "start": t_start,
    "stop": t_stop,
    "restart": t_restart,
    "logs": t_logs,
}


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        _LOG("selftest : host=%s dir=%s" % (SSH_HOST, COMPOSE_DIR))
        _LOG(t_status({}).replace("\n", " | "))
        return
    _LOG("demarrage (host=%s, dir=%s)" % (SSH_HOST, COMPOSE_DIR))
    _rpc.serve(SERVER_NAME, TOOLS, HANDLERS)


if __name__ == "__main__":
    main()
