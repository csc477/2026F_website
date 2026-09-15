"""
CSC477 Tutorial 2: the smallest useful ROS 2 subscriber (complete, nothing to fill in).

Run in a second terminal while minimal_publisher is running:
       ros2 run csc477_tut02 minimal_subscriber
Then:  rqt_graph            (you should see publisher -> /chatter -> subscriber)

Try remapping the topic on the command line without touching the code:
       ros2 run csc477_tut02 minimal_subscriber --ros-args -r chatter:=other_topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        # create_subscription(message type, topic, callback, QoS)
        self.sub = self.create_subscription(String, "chatter", self.on_msg, 10)

    def on_msg(self, msg: String):
        # Called once per received message, but only while the node is spinning.
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
