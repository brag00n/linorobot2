import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bamboo_controler_YBStm32v3"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Faits d'HOTE et de transport uniquement : la verite physique est dans
        # bamboo_base/config/robots/<robot>.yaml, que le launch charge APRES celui-ci.
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Titi28011976",
    maintainer_email="titi28011976@users.noreply.github.com",
    description="Driver ROS2 de la carte Yahboom ROS Control Board V3.0 (STM32F103RCT6).",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stm32_mavlink_driver = "
            "bamboo_controler_YBStm32v3.stm32_mavlink_driver:main",
        ],
    },
)
