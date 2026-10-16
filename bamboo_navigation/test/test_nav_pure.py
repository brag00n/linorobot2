"""nav_params et map_io, sans ROS."""
import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

yaml = pytest.importorskip("yaml")
from bamboo_navigation import map_io, nav_params  # noqa: E402

WIRING = {"scan": "/robot/scan", "odom_in": "/odom/unfiltered", "imu": "/imu/data",
          "odom": "/odom", "cmd_vel_nav": "/cmd_vel_nav", "map": "/map"}
HERE = os.path.dirname(__file__)


def _nav2():
    p = os.path.join(HERE, "..", "..", "bamboo4WD_V4_WSEsp32_base", "config", "navigation.yaml")
    with open(p, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def test_patch_nav2_injects_topics_limits_and_sim_time():
    out = nav_params.patch_nav2(_nav2(), WIRING, {"max_vel_x": 0.15, "max_vel_theta": 0.8}, True)
    lc = out["local_costmap"]["local_costmap"]["ros__parameters"]
    assert lc["obstacle_layer"]["scan"]["topic"] == "/robot/scan"
    assert out["amcl"]["ros__parameters"]["scan_topic"] == "/robot/scan"
    vs = out["velocity_smoother"]["ros__parameters"]
    assert vs["max_velocity"] == [0.15, 0.0, 0.8] and vs["odom_topic"] == "/odom"
    fp = out["controller_server"]["ros__parameters"]["FollowPath"]
    assert fp["desired_linear_vel"] == 0.15 and fp["rotate_to_heading_angular_vel"] == 0.8
    assert lc["use_sim_time"] is True and out["amcl"]["ros__parameters"]["use_sim_time"] is True


def test_patch_ekf_and_slam():
    with open(os.path.join(HERE, "..", "config", "ekf.yaml"), encoding="utf-8") as fh:
        e = nav_params.patch_ekf(yaml.safe_load(fh), WIRING)
    assert e["ekf_filter_node"]["ros__parameters"]["imu0"] == "/imu/data"
    with open(os.path.join(HERE, "..", "config", "slam.yaml"), encoding="utf-8") as fh:
        s = nav_params.patch_slam(yaml.safe_load(fh), WIRING, {"range_max_m": 12.0})
    assert s["slam_toolbox"]["ros__parameters"]["scan_topic"] == "/robot/scan"
    # la portee du modele (8 m) plafonne celle du lidar (12 m) pour le dessin
    assert s["slam_toolbox"]["ros__parameters"]["max_laser_range"] == 8.0


def test_required_sensors():
    assert nav_params.required_sensors({"lidar": "d500", "imu": "x"}) == ["lidar", "imu"]
    assert nav_params.required_sensors({"lidar": ""}) == []


def test_map_roundtrip_and_waypoints():
    d = tempfile.mkdtemp()
    data = [-1] * 4 + [0] * 4 + [100] * 4          # 4x3 : inconnu, libre, occupe
    y = map_io.write_map(os.path.join(d, "salon"), 4, 3, 0.05, (-1.0, -2.0, 0.0), data)
    with open(os.path.join(d, "salon.pgm"), "rb") as fh:
        raw = fh.read()
    # Image de HAUT en bas : ligne 2 (occupee, 0) d'abord, ligne 0 (inconnue, 205) en dernier.
    assert raw.startswith(b"P5\n4 3\n255\n")
    assert raw[-12:-8] == b"\x00" * 4 and raw[-8:-4] == b"\xfe" * 4 and raw[-4:] == b"\xcd" * 4
    assert map_io.list_maps(d) == ["salon"]
    assert map_io.map_stats(4, 3, data) == (8 / 12, 4 / 12)
    map_io.save_waypoint(d, "salon", "depart", 0, 0, 0)
    wps = map_io.save_waypoint(d, "salon", "porte", 1.5, -0.3, 1.57)
    assert [w["name"] for w in wps] == ["depart", "porte"]
    assert map_io.load_waypoints(d, "salon")[1]["x"] == 1.5
    assert os.path.isfile(y)


def test_frontier_log_written_next_to_the_map():
    d = tempfile.mkdtemp()
    entries = [{"n": 1, "x": 0.5, "y": -1.0, "s": "atteinte"},
               {"n": 2, "x": 2.0, "y": 3.25, "s": "inatteignable"}]
    p = map_io.save_frontier_log(d, "salon", entries, (0.0, 0.1, 0.0))
    assert os.path.basename(p) == "salon_frontieres.yaml"
    with open(p, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    assert doc["depart"] == {"x": 0.0, "y": 0.1}
    assert [f["statut"] for f in doc["frontieres"]] == ["atteinte", "inatteignable"]
