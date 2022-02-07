import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    world = LaunchConfiguration('world', default="empty")
    x = LaunchConfiguration('x', default=0.0)
    y = LaunchConfiguration('y', default=0.0)
    z = LaunchConfiguration('z', default=0.0)
    enable_gui = LaunchConfiguration('enable_gui', default="True")

    sandwich_bot_sim_package_share_dir = get_package_share_directory('sandwich_bot_2_0_ign_simulation')

    sandwich_bot_world_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sandwich_bot_sim_package_share_dir,
                    'launch', 'launch_world.py')),
         launch_arguments={'world': world, 'enable_gui': enable_gui}.items(),
    )

    sandwich_bot_spawn_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sandwich_bot_sim_package_share_dir,
                    'launch', 'spawn_bot.py')),
        launch_arguments={'world': world,
            'x': x,
            'y': y,
            'z': z,
            }.items(),
    )

    shape_deleter = Node(
          package='sandwich_bot_2_0_ign_simulation',
          executable='shape_deleter',
          name='shape_deleter',
          output='screen')
          
    return LaunchDescription([
        sandwich_bot_world_include,
        sandwich_bot_spawn_include,
        shape_deleter,
    ])