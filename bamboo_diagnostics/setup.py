import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bamboo_diagnostics"

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
    description="Diagnostics du robot bamboo4WD_V4 : agregation par module et observateur "
                "des composants non instrumentables.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "health_node = bamboo_diagnostics.health_node:main",
        ],
    },
)
