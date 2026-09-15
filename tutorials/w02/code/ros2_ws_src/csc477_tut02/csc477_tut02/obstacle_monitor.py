"""
CSC477 Tutorial 2: subscribe to a LaserScan, publish derived signals (YOUR TURN).

    /scan (sensor_msgs/LaserScan)  -->  this node  -->  /front_clearance (std_msgs/Float32)
                                                   -->  /obstacle_ahead   (std_msgs/Bool)

This is the pattern of every perception node: subscribe to a sensor, reduce the
data to something small, publish it. Fill in the three TODOs, then:

  terminal 1:  ros2 run csc477_tut02 fake_laser_publisher
  terminal 2:  ros2 run csc477_tut02 obstacle_monitor
  terminal 3:  ros2 topic echo /front_clearance
               ros2 topic echo /obstacle_ahead
               ros2 param set /fake_laser_publisher obstacles "[0.8, 0.0, 0.3]"   # obstacle_ahead -> True
               ros2 param set /obstacle_monitor stop_distance 0.3                  # -> False again
               ros2 topic info -v /scan                                            # both nodes listed?
               rqt_graph

You can reuse the functions you wrote in standalone/ex1_laser_scan_geometry.py:
a LaserScan message has exactly the same field names as FakeLaserScan.

With a real robot the only change is the topic name:
  ros2 run csc477_tut02 obstacle_monitor --ros-args -r scan:=/husky_1/scan
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class ObstacleMonitor(Node):
    def __init__(self):
        super().__init__("obstacle_monitor")
        self.declare_parameter("stop_distance", 1.0)          # [m] obstacle_ahead is True below this
        self.declare_parameter("sector_half_width_deg", 15.0)  # front sector is +/- this many degrees

        # ------------------------------------------------------------------ #
        # TODO 1: create the publishers and the subscription.
        #   * publisher of Float32 on topic "front_clearance", queue depth 10
        #   * publisher of Bool    on topic "obstacle_ahead",  queue depth 10
        #   * subscription to LaserScan on topic "scan" with callback self.on_scan
        #     IMPORTANT: use qos_profile_sensor_data as the QoS, otherwise you
        #     will not receive anything from Gazebo / real drivers (best effort).
        # ------------------------------------------------------------------ #
        self.clearance_pub = None
        self.obstacle_pub = None
        self.scan_sub = None
        raise NotImplementedError("TODO 1 in ObstacleMonitor.__init__")

    def on_scan(self, scan: LaserScan):
        # ------------------------------------------------------------------ #
        # TODO 2: compute the front clearance (see ex1: front_clearance).
        #   ranges = np.asarray(scan.ranges)
        #   angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        #   keep beams that are finite and within [range_min, range_max]
        #   keep beams with |angle| < sector_half_width (convert the parameter to radians)
        #   clearance = min range among those; if there is none, use scan.range_max
        # ------------------------------------------------------------------ #
        clearance = None
        raise NotImplementedError("TODO 2 in ObstacleMonitor.on_scan")

        # ------------------------------------------------------------------ #
        # TODO 3: publish.
        #   * Float32(data=clearance)  -- data must be a Python float, not numpy.float64
        #   * Bool(data=clearance < stop_distance)  -- same story: cast with bool()
        #   Log at a low rate so the terminal stays readable, e.g.
        #   self.get_logger().info(f"clearance={clearance:.2f} m", throttle_duration_sec=1.0)
        # ------------------------------------------------------------------ #


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ObstacleMonitor()
    except NotImplementedError as e:
        print(f"\n{e}: fill in the TODOs in obstacle_monitor.py first.\n")
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except NotImplementedError as e:
        node.get_logger().error(f"{e}: fill in the TODOs in obstacle_monitor.py")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
