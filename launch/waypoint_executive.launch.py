from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_executive',       # package name from package.xml
            executable='waypoint_executive',   # name of the executable target in CMakeLists.txt
            name='WaypointExecutiveNode',      # ROS node name
            output='screen',                   # print logs to screen
            parameters=[{
                # example parameters (add if needed)
                # 'some_param': 'value',
            }],
            remappings=[
                # Example remap
                # ('/input_topic', '/new_topic_name'),
            ]
        )
    ])
