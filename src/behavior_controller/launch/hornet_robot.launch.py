from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_controller',
            executable='servo_controller_node',
            name='servo_controller',
            output='screen'
        ),
        Node(
            package='kinematics',
            executable='kinematics_node', 
            name='kinematics',
            output='screen'
        ),
        Node(
            package='behavior_controller',
            executable='behavior_controller_node',
            name='behavior_controller',
            output='screen'
        )
    ])
