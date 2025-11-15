import launch
import launch_ros.actions 

def generate_launch_description():
   return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='soft_mux',
            executable='soft_mux',
            name="softmux_node"
        )
    ]),
