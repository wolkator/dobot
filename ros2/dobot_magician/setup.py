import os
from glob import glob
from setuptools import find_packages, setup

package_name = "dobot_magician"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tombox",
    maintainer_email="tombox@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dobot_server = dobot_magician.dobot_server",
            "dobot_homing = dobot_magician.dobot_homing:main",
            "dobot_joint_state = dobot_magician.dobot_joint_state:main",
            "dobot_joint_ptp = dobot_magician.dobot_joint_ptp:main",
            "dobot_teleop = dobot_magician.dobot_teleop:main",
            "dobot_teleop_keyboard = dobot_magician.dobot_teleop_keyboard:main",
            "dobot_cartesian_ptp = dobot_magician.dobot_cartesian_ptp:main",
            "dobot_suction_cup = dobot_magician.dobot_suction_cup:main",
        ],
    },
)
