from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    crazyflie_pkg = get_package_share_directory('webots_ros2_crazyflie')
    crazyflie_launch = os.path.join(crazyflie_pkg, 'launch', 'robot_launch.py')

    return LaunchDescription([
        # Include another launch file (not a Node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(crazyflie_launch),
        ),

        # Your custom controller node
        Node(
            package='crazyflie_position_controller',
            executable='controller_node',
            output='screen',
            parameters=[{'Kp': 0.2, 'Kd': 0.0}]
        )
    ])

