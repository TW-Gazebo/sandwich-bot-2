import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    configured_params = [os.path.join(get_package_share_directory("sandwich_bot_2_0_control"), 'config', 'exploration_params.yaml'),
                            {'use_sim_time': use_sim_time}]

    return LaunchDescription([

        Node(
            package='sandwich_bot_2_0_control',
            executable='local_rrt',
            name='local_rrt',
            output='both',
            parameters=configured_params),

        Node(
            package='sandwich_bot_2_0_control',
            executable='global_rrt',
            name='global_rrt',
            output='both',
            parameters=configured_params),

        Node(
            package='sandwich_bot_2_0_control',
            executable='filter',
            name='filter',
            output='both',
            parameters=configured_params),
        
        Node(
            package='sandwich_bot_2_0_control',
            executable='assigner',
            name='assigner',
            output='both',
            parameters=configured_params),

    ])
