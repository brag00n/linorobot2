import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bamboo4WD_V4_WSEsp32_base"

setup(
    name=package_name,
    version="0.0.0",
    # Aucun module Python : ce paquet ne porte que des launch. find_packages renvoie
    # donc une liste vide, et c'est normal.
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Titi28011976",
    maintainer_email="titi28011976@users.noreply.github.com",
    description="Bringup du robot bamboo4WD_V4_WSEsp32 (BambooWS). NON VALIDE : robot en pause.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
