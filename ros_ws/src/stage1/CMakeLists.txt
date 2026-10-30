cmake_minimum_required(VERSION 3.8)
project(stage1_stoneware_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/validate_stage1.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
