"""
CSC477 Tutorial 2: a Python launch file (complete).

Starts the fake laser, the (student-completed) obstacle monitor, and the
parameter demo in one go, with parameters set from launch arguments.

    ros2 launch csc477_tut02 tut02.launch.py
    ros2 launch csc477_tut02 tut02.launch.py stop_distance:=1.5
    ros2 launch csc477_tut02 tut02.launch.py --show-args

Assignment 1 uses the same machinery: `ros2 launch wall_following_assignment ...`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stop_distance = LaunchConfiguration("stop_distance")

    return LaunchDescription([
        DeclareLaunchArgument("stop_distance", default_value="1.0",
                              description="front clearance below which /obstacle_ahead is True [m]"),

        Node(
            package="csc477_tut02",
            executable="fake_laser_publisher",
            name="fake_laser_publisher",
            output="screen",
            parameters=[{"rate_hz": 10.0,
                         "obstacles": [2.5, 0.0, 0.3, 1.0, 1.5, 0.2, 3.0, -2.0, 0.5]}],
        ),
        Node(
            package="csc477_tut02",
            executable="obstacle_monitor",
            name="obstacle_monitor",
            output="screen",
            parameters=[{"stop_distance": stop_distance}],
            # Remappings are how the same node talks to a different robot:
            # remappings=[("scan", "/husky_1/scan")],
        ),
        Node(
            package="csc477_tut02",
            executable="param_demo",
            name="param_demo",
            output="screen",
            parameters=[{"gain": 1.0, "stop_distance": stop_distance}],
        ),
    ])
