import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bamboo_controler_Teensy"

setup(
    name=package_name,
    version="0.1.0",
    # Liste VIDE et c'est voulu : aucun code Python ici : ce module est une declaration, pas un driver.
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # capabilities.yaml est le VRAI livrable de ce module : bamboo_base.capability_check
        # le lit au bringup pour refuser un axe de servo sur une carte qui n'en a pas.
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Titi28011976",
    maintainer_email="titi28011976@users.noreply.github.com",
    description="Capacites declarees de la carte Teensy. Aucun driver ROS : non valide sur robot.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
