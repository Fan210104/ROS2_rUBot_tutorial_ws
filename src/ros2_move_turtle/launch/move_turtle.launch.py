from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # Launch your custom move_turtle node
        Node(
            package='ros2_move_turtle',
            executable='move_turtle',
            name='move_turtle'
        )
    ])
