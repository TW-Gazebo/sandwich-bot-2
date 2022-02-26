from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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

     return LaunchDescription([
          find_coloured_box_action_server,
          pick_coloured_box_action_server,
     ])
