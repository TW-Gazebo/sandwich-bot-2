import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    use_sim_time_arg = DeclareLaunchArgument(
               'use_sim_time',
               default_value=use_sim_time,
               description='If true, use simulated clock')

    rtabmap_control_include = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(get_package_share_directory('sandwich_bot_2_0_control'), 'launch', 'visual_slam_start.py')),
     )

    rtabmapviz = Node(
       package='rtabmap_ros',
       executable='rtabmapviz',
       name='rtabmapviz',
       output='screen',
       parameters=[os.path.join(get_package_share_directory('sandwich_bot_2_0_desktop'), 'config', 'rtabmapviz', 'rtabmapviz.yaml')],
       arguments=['-d', os.path.join(get_package_share_directory('sandwich_bot_2_0_desktop'), 'config', 'rtabmapviz', 'rgbd_gui.ini')],
       remappings=[
            ("left/image_rect", "/stereo_camera/left/image_rect"),
            ("right/image_rect", "/stereo_camera/right/image_rect"),
            ("left/camera_info", "/stereo_camera/left/camera_info"),
            ("right/camera_info", "/stereo_camera/right/camera_info"),
            ("odom_info", "/odom_info"),
            ("mapData", "mapData"),
            ("map", "/rtabmap_ros/map"),
        ],
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
     
    return LaunchDescription([
        rtabmap_control_include,
        rtabmapviz,
        sandiwich_bot_2_0_ign_simulation_include,
        ])

