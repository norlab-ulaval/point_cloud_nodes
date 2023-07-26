cmake_minimum_required(VERSION 3.8)
project(point_cloud_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(libpointmatcher_ros REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(point_cloud_exporter src/point_cloud_exporter.cpp)
ament_target_dependencies(point_cloud_exporter rclcpp sensor_msgs libpointmatcher_ros)
target_link_libraries(point_cloud_exporter ${libpointmatcher_LIBRARIES})

add_executable(point_clouds_exporter src/point_clouds_exporter.cpp)
ament_target_dependencies(point_clouds_exporter rclcpp rosbag2_cpp sensor_msgs libpointmatcher_ros)
target_link_libraries(point_clouds_exporter ${libpointmatcher_LIBRARIES})

install(TARGETS point_cloud_exporter point_clouds_exporter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
