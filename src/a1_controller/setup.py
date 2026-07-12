from glob import glob
import os

from setuptools import find_packages, setup


package_name = "a1_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "LICENSE"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "policy"), glob("policy/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="ubuntu",
    maintainer_email="275759366@qq.com",
    description="ROS 2 simulation locomotion controller for the Unitree A1.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "a1_controller_node = a1_controller.controller_node:main",
        ],
    },
)
