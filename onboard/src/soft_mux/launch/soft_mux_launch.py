from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      launch_ros.actions.Node(namespace= "Soft_Mux", package='soft_mux', executable='soft_mux'),
   ])