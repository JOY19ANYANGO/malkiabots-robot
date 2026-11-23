cmake_minimum_required(VERSION 3.8)
project(cramped_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch rviz config models worlds maps docs firmware
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/base_driver.py scripts/patrol.py scripts/make_coverage_route.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
