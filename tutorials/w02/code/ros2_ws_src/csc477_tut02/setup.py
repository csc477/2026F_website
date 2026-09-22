import os
from glob import glob

from setuptools import find_packages, setup

package_name = "csc477_tut02"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install launch files so `ros2 launch csc477_tut02 tut02.launch.py` finds them.
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Florian Shkurti",
    maintainer_email="florian@cs.toronto.edu",
    description="CSC477 Tutorial 2: minimal ROS 2 nodes.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        # Each line creates an executable: `ros2 run csc477_tut02 <name>`
        "console_scripts": [
            "minimal_publisher = csc477_tut02.minimal_publisher:main",
            "minimal_subscriber = csc477_tut02.minimal_subscriber:main",
            "fake_laser_publisher = csc477_tut02.fake_laser_publisher:main",
            "obstacle_monitor = csc477_tut02.obstacle_monitor:main",
            "param_demo = csc477_tut02.param_demo:main",
        ],
    },
)
