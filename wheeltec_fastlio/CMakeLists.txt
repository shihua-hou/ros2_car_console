cmake_minimum_required(VERSION 3.8)
project(wheeltec_fastlio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.15)
link_directories(/usr/local/lib)
set(WHEELTEC_PCL_LIBS pcl_common pcl_io pcl_filters pcl_segmentation pcl_sample_consensus pcl_search pcl_kdtree)

add_executable(pcd2pgm src/pcd2pgm.cpp)
target_include_directories(pcd2pgm PRIVATE ${PCL_INCLUDE_DIRS} /usr/include/eigen3)
target_link_libraries(pcd2pgm ${WHEELTEC_PCL_LIBS})
ament_target_dependencies(pcd2pgm rclcpp nav_msgs)

add_executable(auto_relocalize src/auto_relocalize.cpp)
ament_target_dependencies(auto_relocalize rclcpp nav_msgs sensor_msgs geometry_msgs std_srvs tf2_ros)

add_executable(depth_obstacle_filter src/depth_obstacle_filter.cpp)
ament_target_dependencies(depth_obstacle_filter rclcpp sensor_msgs geometry_msgs tf2_ros)

install(TARGETS pcd2pgm auto_relocalize depth_obstacle_filter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config map rviz
  DESTINATION share/${PROJECT_NAME})

# clean_map.py: 用实时激光把地图里"已经挪走的障碍"擦掉
# (nav2 的 static_layer 清不掉这些, 见脚本头部注释)
# blackbox.py: 1Hz fsync 的机器状态记录, 查整机卡死用(journald 断电会丢)
install(PROGRAMS scripts/clean_map.py scripts/blackbox.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
