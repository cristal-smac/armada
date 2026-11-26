from glob import glob

from setuptools import find_packages, setup

package_name = "armada_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Erwan",
    maintainer_email="erwan.martin666@gmail.com",
    description="Simulation Gazebo multi-robot du robot ARMADA",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": [
            "cmd_vel_filter = armada_simulation.cmd_vel_filter:main",
            "scan_to_range = armada_simulation.scan_to_range:main",
        ],
    },
)
