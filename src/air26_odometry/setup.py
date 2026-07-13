from setuptools import find_packages, setup

package_name = "air26_odometry"

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
    description="Wheel odometry (Task 2): working scaffold, integration stubbed.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odometry_node = air26_odometry.odometry_node:main",
        ],
    },
)
