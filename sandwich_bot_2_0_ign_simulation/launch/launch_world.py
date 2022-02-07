import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction

def launch_setup_gui_enabled(context, *args, **kwargs):
    sandwich_bot_2_0_ign_simulation = get_package_share_directory('sandwich_bot_2_0_ign_simulation')
    world_name = LaunchConfiguration('world', default='empty').perform(context)
    enable_gui = LaunchConfiguration('enable_gui', default="True").perform(context)
    world_file_path= os.path.join(sandwich_bot_2_0_ign_simulation, 'worlds', f'{world_name}.world')

    ign_gazebo_gui_enabled= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
            ),
        launch_arguments={
            'ign_args': f"-r {world_file_path}"+ ' --gui-config ' +
					 os.path.join(sandwich_bot_2_0_ign_simulation, 'config', 'gui.config')
        }.items(),
        condition=IfCondition(enable_gui),
    )
    return [ign_gazebo_gui_enabled]

def launch_setup_gui_disabled(context, *args, **kwargs):
    sandwich_bot_2_0_ign_simulation = get_package_share_directory('sandwich_bot_2_0_ign_simulation')
    world_name = LaunchConfiguration('world', default='empty').perform(context)
    enable_gui = LaunchConfiguration('enable_gui', default="True").perform(context)
    world_file_path= os.path.join(sandwich_bot_2_0_ign_simulation, 'worlds', f'{world_name}.world')

    ign_gazebo_gui_disnabled= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
            ),
        launch_arguments={
            'ign_args': f"-r -s {world_file_path}"+ ' --gui-config ' +
					 os.path.join(sandwich_bot_2_0_ign_simulation, 'config', 'gui.config')
        }.items(),
        condition=IfCondition(PythonExpression(['not ', enable_gui])),
    )
    return [ign_gazebo_gui_disnabled]

def generate_launch_description():
    world_models_path =  os.path.join(get_package_share_directory('sandwich_bot_2_0_ign_simulation'), 'models')

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', world_models_path),
        OpaqueFunction(function = launch_setup_gui_enabled),
        OpaqueFunction(function = launch_setup_gui_disabled),
    ])