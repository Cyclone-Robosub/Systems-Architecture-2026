from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
        Node(
            # name of the package
            package='thrust_interface',
            # name of the elf
            executable='thrust_interface',
            # Name of the node
            name="thrust_interface_node",
            # Writes to the stdout
            output="screen"
        )
    ])
