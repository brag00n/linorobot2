import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bamboo_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Titi28011976",
    maintainer_email="titi28011976@users.noreply.github.com",
    description="Groupe manette du robot bamboo4WD_V4 : /joy -> modes du tracking et "
                "deplacements relatifs des axes de servo.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_node = bamboo_control.teleop_node:main",
        ],
    },
)
