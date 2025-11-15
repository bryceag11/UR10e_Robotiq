cmake_minimum_required(VERSION 3.22)
project(ur10e_robotiq_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
