import subprocess
import time
import xacro
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

     use_sim_time = LaunchConfiguration('use_sim_time', default=False)

     use_sim_time_arg = DeclareLaunchArgument(
          'use_sim_time',
          default_value=use_sim_time,
          description='If true, use simulated clock')

     xacro_file = os.path.join(get_package_share_directory('sandwich_bot_2_0_description'), 'urdf', 'sandwich_bot.xacro')  
     assert os.path.exists(xacro_file), "The sandwich_bot.xacro doesnt exist in "+str(xacro_file)  
     robot_description_config = xacro.process_file(xacro_file)
     robot_desc = robot_description_config.toxml()
     params = [{"robot_description": robot_desc}]

     sandwich_bot_spawner = Node(
            package="ros_ign_gazebo",
            executable="create",
            name="ros_ign_create_bot",
            parameters=params,
            output="screen",
            arguments=[
               "-world",
               LaunchConfiguration('world'),
               "-param",
               "robot_description",
               "-name",
               "sandwich_bot",
               "-allow_renaming",
               "true",
               "-x",
               LaunchConfiguration('x'),
               "-y",
               LaunchConfiguration('y'),
               "-z",
               LaunchConfiguration('z'),
            ]
        )

     base_scan_to_lidar_transform_publisher = Node(package = "tf2_ros", 
                        executable = "static_transform_publisher",
                        parameters=[{'use_sim_time': use_sim_time}],
                        arguments = ["0", "0", "0", "0", "0", "0", "base_scan", "sandwich_bot/base_footprint/gpu_lidar"])
     camera_link_left_to_camera_left_transform_publisher = Node(package = "tf2_ros", 
                         executable = "static_transform_publisher",
                         parameters=[{'use_sim_time': use_sim_time}],
                         arguments = ["0", "0", "0", "-1.57", "0", "-1.57", "camera_link_left", "sandwich_bot/base_footprint/camera_left"])
     camera_link_right_to_camera_right_transform_publisher = Node(package = "tf2_ros", 
                         executable = "static_transform_publisher",
                         parameters=[{'use_sim_time': use_sim_time}],
                         arguments = ["0", "0", "0", "-1.57", "0", "-1.57", "camera_link_right", "sandwich_bot/base_footprint/camera_right"])
     imu_link_to_imu_sensor_transform_publisher = Node(package = "tf2_ros", 
                         executable = "static_transform_publisher",
                         parameters=[{'use_sim_time': use_sim_time}],
                         arguments = ["0", "0", "0", "0", "0", "0", "imu_link", "sandwich_bot/base_footprint/imu_sensor"])
     
     left_wheel_static_transform_publisher = Node(package = "tf2_ros", 
                         executable = "static_transform_publisher",
                         parameters=[{'use_sim_time': use_sim_time}],
                         arguments = ["0", "0.08", "0.023", "0", "0", "1.57", "base_link", "wheel_left_link"])

     right_wheel_static_transform_publisher = Node(package = "tf2_ros", 
                         executable = "static_transform_publisher",
                         parameters=[{'use_sim_time': use_sim_time}],
                         arguments = ["0", "-0.08", "0.023", "0", "0", "1.57", "base_link", "wheel_right_link"])
     
     ros_ign_lidar_bridge = Node(
          package="ros_ign_bridge",
          executable="parameter_bridge",
          arguments=[
               "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
          ])

     ros_ign_odom_tf_bridge = Node(package = "ros_ign_bridge", 
                         executable = "parameter_bridge",
                         arguments = [
                              "/model/sandwich_bot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                         ],
                         remappings=[
                              ("/model/sandwich_bot/tf", "/tf"),
                         ])

     ros_ign_cmd_vel_bridge = Node(package = "ros_ign_bridge", 
                         executable = "parameter_bridge",
                         arguments = [
                              "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                         ])

     ros_ign_left_camera_bridge = Node(
          package="ros_ign_bridge",
          executable="parameter_bridge",
          arguments=[
               "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
               "/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
          ])

     ros_ign_right_camera_bridge = Node(
          package="ros_ign_bridge",
          executable="parameter_bridge",
          arguments=[
               "/stereo_camera/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
               "/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
          ])

     ros_ign_imu_bridge = Node(package="ros_ign_bridge",
                                   executable="parameter_bridge",
                                   arguments=[
                                   "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                                   ])
     
     ros_ign_odom_bridge = Node(package="ros_ign_bridge",
                                   executable="parameter_bridge",
                                   arguments=[
                                   "/model/sandwich_bot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                                   ],
                                   remappings=[
                                        ("/model/sandwich_bot/odometry", "/odom"),
                                   ])

    # why this clock bridge needs to be separated?
    # also why this bridge needs to be biway?
     ros_ign_clock_bridge = Node(
          package="ros_ign_bridge",
          executable="parameter_bridge",
          arguments=[
               "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
          ])

     return LaunchDescription([
          sandwich_bot_spawner,
          
          ros_ign_lidar_bridge,
          ros_ign_cmd_vel_bridge,
          ros_ign_left_camera_bridge,
          ros_ign_right_camera_bridge,
          ros_ign_clock_bridge,
          ros_ign_imu_bridge,
          use_sim_time_arg,
          base_scan_to_lidar_transform_publisher,
          camera_link_left_to_camera_left_transform_publisher,
          camera_link_right_to_camera_right_transform_publisher,
          imu_link_to_imu_sensor_transform_publisher,
          left_wheel_static_transform_publisher,
          right_wheel_static_transform_publisher,
          ros_ign_odom_tf_bridge,
          ros_ign_odom_bridge
     ])
