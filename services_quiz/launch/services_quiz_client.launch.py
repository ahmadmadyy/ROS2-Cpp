from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='services_quiz',
            executable='client_quiz_node',
            output='screen'),
    ])