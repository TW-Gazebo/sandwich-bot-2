cmake_minimum_required(VERSION 3.5)
project(sandwich_bot_2_0_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
