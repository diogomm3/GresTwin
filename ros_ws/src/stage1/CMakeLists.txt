cmake_minimum_required(VERSION 3.8)
project(stage1_stoneware_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/publish_stage1_planning_scene.py
    scripts/stage1_pick_place.py
    scripts/stage1_pick_place_test.py
    scripts/stage1_suction_controller.py
    scripts/validate_stage1.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/stage1_planning_scene_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
