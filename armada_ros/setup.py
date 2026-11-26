from glob import glob

from setuptools import find_packages, setup

package_name = "armada_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Erwan",
    maintainer_email="erwan.martin666@gmail.com",
    description="Interface ROS 2 commune au robot ARMADA réel et simulé",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": [
            "odom_adapter = armada_ros.odom_adapter:main",
        ],
    },
)
