# https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

import rclpy
from rclpy.node import Node

from custom_interfaces.msg import Pwms
from std_msgs.msg import Bool


class CLIPublisher(Node):
	def __init__(self):
		super().__init__('pwm_cli')
		self.publisher_ = self.create_publisher(Pwms, 'pwm_cli', 10)

	def publish_pwm(self, pwms):
		msg = Pwms()
		msg.pwms = pwms
		self.publisher_.publish(msg)


class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__('heartbeat')
        self.publisher_ = self.create_publisher(Bool, 'cli_heartbeat', 10)
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
		