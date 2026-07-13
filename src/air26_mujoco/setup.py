import os
from glob import glob

from setuptools import find_packages, setup

package_name = "air26_mujoco"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "mjcf"), glob("mjcf/*.xml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIR26 Workshop",
    maintainer_email="air26@example.com",
    description="MuJoCo physics engine for the AIR26 rover (M1 reference).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_driver = air26_mujoco.mujoco_driver:main",
        ],
    },
)
