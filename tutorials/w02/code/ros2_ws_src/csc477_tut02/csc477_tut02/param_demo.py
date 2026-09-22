"""
CSC477 Tutorial 2: runtime parameters, the ROS 2 replacement for dynamic_reconfigure (complete).

This node holds a few tunable values as parameters and reacts when they change.
It does not control anything; it shows the mechanism you need whenever you want
to tune a running node without restarting it.

Run:   ros2 run csc477_tut02 param_demo
Then:  ros2 param list /param_demo
       ros2 param get /param_demo gain
       ros2 param set /param_demo gain 2.5          # float: 2.5 works, 2 (int) is rejected
       ros2 param set /param_demo gain -1.0         # rejected by our validation callback
       ros2 param dump /param_demo                  # YAML you can load with --params-file
       ros2 run rqt_reconfigure rqt_reconfigure     # GUI sliders

Start with different values without editing code:
       ros2 run csc477_tut02 param_demo --ros-args -p gain:=0.8 -p max_speed:=0.5
"""

import rclpy
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.node import Node


class ParamDemo(Node):
    def __init__(self):
        super().__init__("param_demo")

        # A descriptor gives rqt_reconfigure a slider range and documents the parameter.
        def ranged(desc, lo, hi):
            return ParameterDescriptor(
                description=desc,
                floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)],
            )

        self.declare_parameter("gain", 1.0, ranged("some controller gain", 0.0, 10.0))
        self.declare_parameter("max_speed", 1.0, ranged("velocity limit [m/s]", 0.0, 2.0))
        self.declare_parameter("stop_distance", 0.5, ranged("emergency stop distance [m]", 0.1, 3.0))
        self.declare_parameter("verbose", False)  # a bool parameter for contrast

        # Copy parameters into plain attributes: this is what your algorithm would read.
        self.gain = self.get_parameter("gain").value
        self.max_speed = self.get_parameter("max_speed").value
        self.stop_distance = self.get_parameter("stop_distance").value
        self.verbose = self.get_parameter("verbose").value

        # Called *before* a parameter change is applied. Return successful=False to reject it.
        self.add_on_set_parameters_callback(self.on_params)

        self.create_timer(2.0, self.on_timer)
        self.get_logger().info("Try: ros2 param set /param_demo gain 2.5")

    def on_params(self, params):
        for p in params:
            if p.name in ("gain", "max_speed", "stop_distance") and p.value < 0.0:
                return SetParametersResult(successful=False, reason=f"{p.name} must be >= 0")
            if p.name in ("gain", "max_speed", "stop_distance", "verbose"):
                setattr(self, p.name, p.value)
                self.get_logger().info(f"{p.name} <- {p.value}")
                # If your algorithm has accumulated state (filters, integrators), reset it here.
        return SetParametersResult(successful=True)

    def on_timer(self):
        msg = f"gain={self.gain:.3f} max_speed={self.max_speed:.2f} stop_distance={self.stop_distance:.2f}"
        if self.verbose:
            msg += f"  (node time {self.get_clock().now().nanoseconds * 1e-9:.1f} s)"
        self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParamDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
