import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

     use_sim_time = LaunchConfiguration('use_sim_time', default=False)

     xacro_file = os.path.join(
          get_package_share_directory('sandwich_bot_2_0_description'), 'urdf',
          'sandwich_bot.xacro')
     assert os.path.exists(
          xacro_file
     ), "The sandwich_bot.xacro doesnt exist in " + str(xacro_file)
     robot_description_config = xacro.process_file(xacro_file)
     robot_desc = robot_description_config.toxml()
     robot_description = {"robot_description": robot_desc}

     robot_state_pub_node = Node(
          package="robot_state_publisher",
          executable="robot_state_publisher",
          output="both",
          parameters=[robot_description, {
               'use_sim_time': use_sim_time
          }],
     )

     return LaunchDescription([
          robot_state_pub_node,
     ])
