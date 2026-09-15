"""
CSC477 Tutorial 2: publish a synthetic sensor_msgs/LaserScan (complete, nothing to fill in).

Simulates a 2D lidar looking at a scene of circular obstacles, so you can
develop and test scan-processing nodes without Gazebo. The scene is a
parameter, so you can move obstacles around while your subscriber is running.

Run:   ros2 run csc477_tut02 fake_laser_publisher
Then:  ros2 topic hz /scan
       ros2 topic echo /scan --once
       ros2 topic info -v /scan          (note the QoS: best effort, like a real sensor)
       rviz2  -> Fixed Frame "laser", Add "LaserScan" on /scan, Reliability Policy "Best Effort"

Move the obstacle in front of the robot and watch your subscriber react:
       ros2 param set /fake_laser_publisher obstacles "[1.0, 0.0, 0.3]"
       ros2 param set /fake_laser_publisher obstacles "[3.0, 0.0, 0.3, 1.0, 1.5, 0.2]"

`obstacles` is a flat list of x, y, radius triples in the laser frame
(x forward, y left, metres).
"""

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class FakeLaserPublisher(Node):
    def __init__(self):
        super().__init__("fake_laser_publisher")

        # Parameters with defaults. Types are fixed by the default value: pass floats as 1.0, not 1.
        self.declare_parameter("obstacles", [2.5, 0.0, 0.3, 1.0, 1.5, 0.2, 3.0, -2.0, 0.5])
        self.declare_parameter("rate_hz", 10.0)        # scans per second
        self.declare_parameter("noise_std", 0.01)      # [m] Gaussian range noise
        self.declare_parameter("n_beams", 271)
        self.declare_parameter("fov_deg", 270.0)
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("frame_id", "laser")
        self.add_on_set_parameters_callback(self.on_params)

        # Sensor data uses a best-effort QoS profile, exactly like Gazebo and real lidar drivers.
        self.pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.rng = np.random.default_rng(0)
        self.timer = self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.on_timer)
        self.get_logger().info("Publishing synthetic LaserScan on /scan")

    def on_params(self, params):
        for p in params:
            if p.name == "obstacles" and len(p.value) % 3 != 0:
                return SetParametersResult(successful=False, reason="obstacles must be x, y, radius triples")
            if p.name == "rate_hz":
                if p.value <= 0:
                    return SetParametersResult(successful=False, reason="rate_hz must be > 0")
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / p.value, self.on_timer)
            if p.name == "obstacles":
                self.get_logger().info(f"obstacles <- {list(p.value)}")
        return SetParametersResult(successful=True)

    def on_timer(self):
        n = self.get_parameter("n_beams").value
        half_fov = np.radians(self.get_parameter("fov_deg").value) / 2.0
        range_min = self.get_parameter("range_min").value
        range_max = self.get_parameter("range_max").value
        noise_std = self.get_parameter("noise_std").value
        flat = list(self.get_parameter("obstacles").value)
        obstacles = [tuple(flat[i:i + 3]) for i in range(0, len(flat) - len(flat) % 3, 3)]

        angle_min, angle_max = -half_fov, half_fov
        angle_increment = (angle_max - angle_min) / (n - 1)
        angles = angle_min + np.arange(n) * angle_increment
        d = np.stack([np.cos(angles), np.sin(angles)], axis=1)

        ranges = np.full(n, np.inf)
        for cx, cy, r in obstacles:
            # ray t*d meets circle |p - c| = r  ->  t^2 - 2 t (d.c) + |c|^2 - r^2 = 0
            b = d @ np.array([cx, cy])
            disc = b * b - (cx * cx + cy * cy - r * r)
            hit = disc >= 0
            t = b[hit] - np.sqrt(disc[hit])
            ranges[hit] = np.minimum(ranges[hit], np.where(t > 0, t, np.inf))
        if noise_std > 0:
            finite = np.isfinite(ranges)
            ranges[finite] += self.rng.normal(0.0, noise_std, finite.sum())
        ranges[(ranges > range_max) | (ranges < range_min)] = np.inf

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.angle_min = float(angle_min)
        msg.angle_max = float(angle_max)
        msg.angle_increment = float(angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.get_parameter("rate_hz").value
        msg.range_min = float(range_min)
        msg.range_max = float(range_max)
        msg.ranges = ranges.astype(np.float32).tolist()  # message fields are float32 arrays
        msg.intensities = []
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
