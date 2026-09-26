import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bamboo_base"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # Source unique de verite physique (un fichier par robot).
        (os.path.join("share", package_name, "config", "robots"), glob("config/robots/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Titi28011976",
    maintainer_email="titi28011976@users.noreply.github.com",
    description="Socle commun des modules de controleur bamboo : squelette de driver MAVLink, resolution materielle VID:PID, controle des capacites et source unique de verite physique (config/robots/).",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "esp32_mavlink_driver = bamboo_base.esp32_mavlink_driver:main",
            # Resolveur materiel (VID:PID -> /dev/...), utile en verification :
            #   ros2 run bamboo_base hw_resolve
            "hw_resolve = bamboo_base.hw_resolve:main",
            # Confronte servo_axes aux capacites declarees par les modules de
            # controleur, AVANT toute actuation :
            #   ros2 run bamboo_base capability_check bamboo4WD_V4_YBStm32
            "capability_check = bamboo_base.capability_check:main",
        ],
    },
)
