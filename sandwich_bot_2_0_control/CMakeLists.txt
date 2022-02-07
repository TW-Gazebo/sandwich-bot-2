cmake_minimum_required(VERSION 3.5)
project(sandwich_bot_2_0_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sandwich_bot_2_0_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)

add_library(find_coloured_box SHARED src/bt_plugins/find_coloured_box.cc)
list(APPEND plugin_libs find_coloured_box)

add_library(approach_coloured_box SHARED src/bt_plugins/approach_coloured_box.cc)
list(APPEND plugin_libs approach_coloured_box)

add_library(pick_coloured_box SHARED src/bt_plugins/pick_coloured_box.cc)
list(APPEND plugin_libs pick_coloured_box)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} behaviortree_cpp_v3 nav2_msgs nav2_behavior_tree sandwich_bot_2_0_interfaces)
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(box_picker src/box_picker.cc)
ament_target_dependencies(box_picker rclcpp nav2_util nav2_behavior_tree)

add_executable(find_coloured_box_action_server src/find_coloured_box_action_server.cc)
ament_target_dependencies(find_coloured_box_action_server rclcpp rclcpp_action nav2_msgs geometry_msgs sandwich_bot_2_0_interfaces)

add_executable(pick_coloured_box_action_server src/pick_coloured_box_action_server.cc)
ament_target_dependencies(pick_coloured_box_action_server rclcpp rclcpp_action sandwich_bot_2_0_interfaces)

install(TARGETS  
    box_picker 
    find_coloured_box_action_server 
    pick_coloured_box_action_server
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(velocity_smoother src/velocity_smoother.cc)
ament_target_dependencies(velocity_smoother rclcpp filters geometry_msgs)

include_directories(src/rrt_exploration/include)

add_library(utils SHARED src/rrt_exploration/utils.cc)
ament_target_dependencies(utils rclcpp)

add_library(mtrand SHARED src/rrt_exploration/mtrand.cc)

add_library(mean_shift SHARED src/rrt_exploration/mean_shift.cc)

add_library(robot SHARED src/rrt_exploration/robot.cc)
ament_target_dependencies(robot rclcpp tf2_ros geometry_msgs nav2_msgs)

add_executable(local_rrt src/rrt_exploration/local_rrt.cc)
target_link_libraries(local_rrt utils mtrand)

ament_target_dependencies(local_rrt 
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    )


add_executable(global_rrt src/rrt_exploration/global_rrt.cc)
target_link_libraries(global_rrt utils mtrand)

ament_target_dependencies(global_rrt 
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    )

add_executable(filter src/rrt_exploration/filter.cc)
target_link_libraries(filter utils mean_shift)

ament_target_dependencies(filter 
    rclcpp
    geometry_msgs
    visualization_msgs
    sandwich_bot_2_0_interfaces
    )

add_executable(assigner src/rrt_exploration/assigner.cc)
target_link_libraries(assigner utils robot)

ament_target_dependencies(assigner 
    rclcpp
    sandwich_bot_2_0_interfaces
    )

add_executable(lidar_camera_fusion src/lidar_camera_fusion.cc)

ament_target_dependencies(lidar_camera_fusion 
    rclcpp
    sensor_msgs
    tf2_ros
    laser_geometry
    geometry_msgs
    cv_bridge
    )

add_executable(box_pickup_pose_publisher src/box_pickup_pose_publisher.cc)
ament_target_dependencies(box_pickup_pose_publisher 
    rclcpp
    sensor_msgs
    tf2_ros
    geometry_msgs
    )

install(TARGETS 
  velocity_smoother
  local_rrt
  global_rrt
  filter
  assigner
  lidar_camera_fusion
  box_pickup_pose_publisher
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  behaviour_trees
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
    utils mtrand mean_shift robot
   DESTINATION lib
)

install(PROGRAMS
  scripts/boundary_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
