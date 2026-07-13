import os
from glob import glob

from setuptools import find_packages, setup

package_name = "air26_behaviors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIR26 Workshop",
    maintainer_email="air26@example.com",
    description="The four working behaviors (Task 4).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "backup_node = air26_behaviors.backup_node:main",
            "swipe_node = air26_behaviors.swipe_node:main",
            "arm_pose_node = air26_behaviors.arm_pose_node:main",
            "reactive_node = air26_behaviors.reactive_node:main",
        ],
    },
)
