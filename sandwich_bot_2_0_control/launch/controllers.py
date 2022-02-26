from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess

def generate_launch_description():

     load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
     )

     load_sandwich_bot_base_controller = ExecuteProcess(
          cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'sandwich_bot_base_controller'],
          output='screen'
     )

     return LaunchDescription([
          RegisterEventHandler(
               event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_sandwich_bot_base_controller],
               )
          ),
          load_joint_state_controller,
     ])
