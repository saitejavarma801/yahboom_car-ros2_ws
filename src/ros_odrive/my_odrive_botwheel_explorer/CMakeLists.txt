cmake_minimum_required(VERSION 3.8)
project(my_odrive_botwheel_explorer)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description/urdf config launch rviz map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
