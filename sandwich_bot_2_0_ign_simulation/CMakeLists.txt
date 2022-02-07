cmake_minimum_required(VERSION 3.5)
project(sandwich_bot_2_0_ign_simulation)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo3 REQUIRED)
find_package(sandwich_bot_2_0_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(shape_deleter src/shape_deleter.cc)
target_link_libraries(shape_deleter ignition-gazebo3::core)
ament_target_dependencies(shape_deleter rclcpp sandwich_bot_2_0_interfaces)

install(DIRECTORY
  worlds
  models
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/shapes_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  shape_deleter
DESTINATION lib/${PROJECT_NAME})

ament_package()
