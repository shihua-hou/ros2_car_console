cmake_minimum_required(VERSION 3.8)
project(wheeltec_odin1)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config udev map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
