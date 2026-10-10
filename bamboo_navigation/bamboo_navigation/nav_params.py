"""Parametres de la navigation DU ROBOT : injection du cablage et des bornes (Python pur).

Le fichier Nav2 d'un robot (<robot>_base/config/navigation.yaml) et les modeles du paquet
(config/ekf.yaml, config/slam.yaml) ne portent AUCUN nom de sujet ni aucune borne de vitesse
qui fassent foi : on les injecte ici, depuis
  - le `wiring/navigation` de layout.yaml (noms REELS des sujets) ;
  - le bloc `navigation:` du fichier canonique (max_vel_x, max_vel_theta) ;
  - le bloc `lidar:` (portee max) ;
  - use_sim_time (simulation).
Une seule source pour chaque fait : changer de robot, de lidar ou de nom de sujet ne touche
aucun fichier Nav2.
"""
import copy


def _rp(doc, *path):
    """doc[path...]['ros__parameters'], cree au besoin."""
    node = doc
    for p in path:
        node = node.setdefault(p, {})
    return node.setdefault("ros__parameters", {})


def set_sim_time(doc, use_sim_time):
    """use_sim_time sur TOUS les noeuds du document (a toute profondeur)."""
    def walk(d):
        if isinstance(d, dict):
            if "ros__parameters" in d and isinstance(d["ros__parameters"], dict):
                d["ros__parameters"]["use_sim_time"] = bool(use_sim_time)
            for v in d.values():
                walk(v)
    walk(doc)
    return doc


def patch_nav2(doc, wiring, nav, use_sim_time=False):
    """Copie du document Nav2 avec sujets, bornes et use_sim_time injectes."""
    out = copy.deepcopy(doc)
    scan, odom = wiring["scan"], wiring["odom"]
    vx = float(nav.get("max_vel_x", 0.2))
    wz = float(nav.get("max_vel_theta", 1.0))
    for cm in ("local_costmap", "global_costmap"):
        p = _rp(out, cm, cm)
        obs = p.get("obstacle_layer", {})
        for src in str(obs.get("observation_sources", "")).split():
            obs.setdefault(src, {})["topic"] = scan
    _rp(out, "amcl")["scan_topic"] = scan
    _rp(out, "bt_navigator")["odom_topic"] = odom
    vs = _rp(out, "velocity_smoother")
    vs["odom_topic"] = odom
    vs["max_velocity"] = [vx, 0.0, wz]
    vs["min_velocity"] = [-vx, 0.0, -wz]
    fp = _rp(out, "controller_server").setdefault("FollowPath", {})
    fp["desired_linear_vel"] = vx
    if "rotate_to_heading_angular_vel" in fp or fp.get("use_rotate_to_heading"):
        fp["rotate_to_heading_angular_vel"] = wz
    _rp(out, "behavior_server")["max_rotational_vel"] = wz
    return set_sim_time(out, use_sim_time)


def patch_ekf(doc, wiring, use_sim_time=False):
    out = copy.deepcopy(doc)
    p = _rp(out, "ekf_filter_node")
    p["odom0"] = wiring["odom_in"]
    p["imu0"] = wiring["imu"]
    return set_sim_time(out, use_sim_time)


def patch_slam(doc, wiring, lidar, use_sim_time=False):
    out = copy.deepcopy(doc)
    p = _rp(out, "slam_toolbox")
    p["scan_topic"] = wiring["scan"]
    if lidar.get("range_max_m"):
        p["max_laser_range"] = float(lidar["range_max_m"])
    return set_sim_time(out, use_sim_time)


def required_sensors(sensors):
    """Capteurs EXIGES par la navigation, d'apres le bloc `sensors:` du robot.

    Le lidar est indispensable (cartographie, localisation, obstacles) ; l'IMU l'est des
    qu'elle est declaree (l'EKF en tire la rotation). Un capteur absent du robot n'est
    jamais exige -- mais SANS lidar, la navigation est declaree impossible.
    """
    req = []
    if (sensors or {}).get("lidar"):
        req.append("lidar")
    if (sensors or {}).get("imu"):
        req.append("imu")
    return req
