cmake_minimum_required(VERSION 3.8)
project(wheeltec_n10plus)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config udev
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
