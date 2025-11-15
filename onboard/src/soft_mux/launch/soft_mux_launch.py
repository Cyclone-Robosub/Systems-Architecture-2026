from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
        Node(
            # name of the package
            package='soft_mux',
            # name of the elf
            executable='soft_mux',
            # Name of the node
            name="softmux_node",
            # Writes to the stdout
            output="screen"
        )
    ])
