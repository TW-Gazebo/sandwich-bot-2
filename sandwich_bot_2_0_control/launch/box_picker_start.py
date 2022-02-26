import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
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
     
     sandwich_bot_control_package_share_dir = get_package_share_directory('sandwich_bot_2_0_control')

     nav2_include = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(sandwich_bot_control_package_share_dir, 'launch', 'navigation', 'nav2_bringup_launch.py')),
               launch_arguments={'slam': "True",
                              'map': os.path.join(sandwich_bot_control_package_share_dir, 'map_data', 'image_map.yaml'),
                              'use_sim_time': use_sim_time,
                              'params_file': os.path.join(sandwich_bot_control_package_share_dir, 'config', 'nav2_params.yaml'),
                              'default_bt_xml_filename' : os.path.join(sandwich_bot_control_package_share_dir, 'behaviour_trees', 'navigate_w_replanning_time.xml'),
                              'autostart': 'True'}.items()
     )

     exploration_include = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(sandwich_bot_control_package_share_dir, 'launch', 'rrt_exploration.py')
          ),
     )

     robot_state_publisher_include = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(sandwich_bot_control_package_share_dir, 'launch', 'robot_state_publisher.py')
          )
     )

     controllers_include = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(sandwich_bot_control_package_share_dir, 'launch', 'controllers.py')
          )
     )
     
     velocity_smoother = Node(
          package="sandwich_bot_2_0_control",
          executable="velocity_smoother",
          output="both",
          remappings=[
               ("/cmd_vel_smoother", "/sandwich_bot_base_controller/cmd_vel_smoother"),
               ("/cmd_vel", "/sandwich_bot_base_controller/cmd_vel_unstamped"),
          ]
     )

     box_picker = Node(
               package='sandwich_bot_2_0_control',
               executable='box_picker',
               name='box_picker',
               output='both',
               parameters=[{'bt_xml_filename':os.path.join(get_package_share_directory("sandwich_bot_2_0_control"), 'behaviour_trees', 'pick_coloured_boxes_behaviour_tree.xml')}])

     find_coloured_box_action_server = Node(
            package='sandwich_bot_2_0_control',
            executable='find_coloured_box_action_server',
            name='find_coloured_box_action_server',
            output='both')

     pick_coloured_box_action_server = Node(
            package='sandwich_bot_2_0_control',
            executable='pick_coloured_box_action_server',
            name='pick_coloured_box_action_server',
            output='both')

     box_pickup_pose_publisher = Node(
            package='sandwich_bot_2_0_control',
            executable='box_pickup_pose_publisher',
            name='box_pickup_pose_publisher',
            output='both')

     lidar_camera_fusion = Node(
          package='sandwich_bot_2_0_control',
          executable='lidar_camera_fusion',
          name='lidar_camera_fusion',
          output='both')

     return LaunchDescription([
          nav2_include,
          exploration_include,
          robot_state_publisher_include,
          controllers_include,
          velocity_smoother,
          box_picker,
          find_coloured_box_action_server,
          pick_coloured_box_action_server,
          box_pickup_pose_publisher,
          lidar_camera_fusion,
          use_sim_time_arg
     ])

# ros2 run key_teleop key_teleop --ros-args -r key_vel:=sandwich_bot_base_controller/cmd_vel_smoother