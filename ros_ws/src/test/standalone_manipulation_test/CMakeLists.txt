cmake_minimum_required(VERSION 3.8)
project(stoneware_manipulation_test)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pick_place_demo.py
    scripts/validate_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
