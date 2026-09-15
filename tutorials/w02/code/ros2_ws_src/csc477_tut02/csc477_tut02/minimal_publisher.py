"""
CSC477 Tutorial 2: the smallest useful ROS 2 publisher (complete, nothing to fill in).

Run:   ros2 run csc477_tut02 minimal_publisher
Then:  ros2 topic list
       ros2 topic echo /chatter
       ros2 topic hz /chatter
       ros2 node info /minimal_publisher
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        # The node name is what shows up in `ros2 node list` and rqt_graph.
        super().__init__("minimal_publisher")

        # create_publisher(message type, topic name, QoS). An integer QoS is a
        # shortcut for "keep the last N messages, reliable delivery".
        self.pub = self.create_publisher(String, "chatter", 10)

        # Timers are how you do periodic work in ROS 2: no while-loop + sleep.
        self.timer = self.create_timer(0.5, self.on_timer)
        self.count = 0

    def on_timer(self):
        msg = String()
        msg.data = f"Hello ROS 2: {self.count}"
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)  # runs the timer callback until Ctrl-C
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
