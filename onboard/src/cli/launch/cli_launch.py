from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
        Node(
            # name of the package
            package='cli',
            # name of the elf
            executable='cli',
            # Name of the node
            name="cli_node",
            # Writes to the stdout
            output="screen"
        )
    ])
