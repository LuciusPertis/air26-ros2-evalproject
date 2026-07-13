from setuptools import find_packages, setup

package_name = "air26_robot_info"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIR26 Workshop",
    maintainer_email="air26@example.com",
    description="Robot-info service (Task 3): working skeleton, introspection stubbed.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_info_node = air26_robot_info.robot_info_node:main",
        ],
    },
)
