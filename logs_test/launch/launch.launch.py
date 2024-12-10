from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logs_test',
            namespace='logs_test_nodes',
            executable='logger_node',
            output='screen'
        ),
    ])