import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

     use_sim_time = LaunchConfiguration('use_sim_time', default=False)

     use_sim_time_arg = DeclareLaunchArgument(
               'use_sim_time',
               default_value=use_sim_time,
               description='If true, use simulated clock')
     
     sandiwich_bot_2_0_control_include = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(get_package_share_directory('sandwich_bot_2_0_control'), 'launch', 'box_picker_start.py')),
     )

     sandiwich_bot_2_0_ign_simulation_include = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(get_package_share_directory('sandwich_bot_2_0_ign_simulation'), 'launch', 'main_launch.py')),
               launch_arguments={
                                   'world' : 'house_modified',
                                   'x' : '-2',
                                   'y' : '-2',
                                   }.items()
     )

     rviz2 = Node(package='rviz2',
        executable='rviz2',
        name="rviz2",
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(get_package_share_directory('sandwich_bot_2_0_desktop'), 'config', 'rviz', 'config.rviz')]
        )

     return LaunchDescription([
          sandiwich_bot_2_0_control_include,
          sandiwich_bot_2_0_ign_simulation_include,
          rviz2,
          use_sim_time_arg
     ])
