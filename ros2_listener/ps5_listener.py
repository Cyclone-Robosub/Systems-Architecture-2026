#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PS5Listener(Node):
    def __init__(self):
        super().__init__('ps5_listener')
        self.subscription = self.create_subscription(
            Float32,
            'ps5_controller',  # topic without leading slash per ROS 2 style
            self.callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('ps5_listener started; waiting for messages on /ps5_controller')

    def callback(self, msg: Float32):
        self.get_logger().info(f'Received left stick X: {msg.data:.2f}')


def main():
    rclpy.init()
    node = PS5Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
