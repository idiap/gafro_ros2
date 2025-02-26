from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os.path


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gafro_ros2',
            namespace='visualize_franka_robot',
            executable='visualize_franka_robot',
            name='visualize_franka_robot',
            output='screen'
        ),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d' + os.path.join(get_package_share_directory('gafro_ros2'), 'visualize_franka_robot', 'visualize_franka_robot.rviz')]
        )
    ])
