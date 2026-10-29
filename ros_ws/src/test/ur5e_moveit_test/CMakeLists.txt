cmake_minimum_required(VERSION 3.8)
project(ur5e_moveit_test)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pick_place_demo.py
    scripts/republish_robot_description.py
    scripts/validate_ur5e.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
