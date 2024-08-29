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
find_package(rclcpp_components REQUIRED)

include_directories(${libpointmatcher_INCLUDE_DIRS})

add_executable(point_cloud_exporter src/point_cloud_exporter.cpp)
ament_target_dependencies(point_cloud_exporter rclcpp sensor_msgs libpointmatcher_ros)
target_link_libraries(point_cloud_exporter ${libpointmatcher_LIBRARIES})

add_executable(point_clouds_exporter src/point_clouds_exporter.cpp)
ament_target_dependencies(point_clouds_exporter rclcpp rosbag2_cpp sensor_msgs libpointmatcher_ros)
target_link_libraries(point_clouds_exporter ${libpointmatcher_LIBRARIES})

add_executable(nan_remover src/nan_remover.cpp)
ament_target_dependencies(nan_remover rclcpp sensor_msgs libpointmatcher_ros)
target_link_libraries(nan_remover ${libpointmatcher_LIBRARIES})

add_executable(vtk_to_csv_node src/vtk_to_csv.cpp)
target_link_libraries(vtk_to_csv_node ${libpointmatcher_LIBRARIES})

add_executable(point_cloud_downsampler src/point_cloud_downsampler.cpp)
ament_target_dependencies(point_cloud_downsampler rclcpp sensor_msgs libpointmatcher_ros)
target_link_libraries(point_cloud_downsampler ${libpointmatcher_LIBRARIES})

add_library(point_cloud_downsampler_component SHARED src/point_cloud_downsampler_component.cpp)
ament_target_dependencies(point_cloud_downsampler_component rclcpp sensor_msgs libpointmatcher_ros rclcpp_components)
target_link_libraries(point_cloud_downsampler_component ${libpointmatcher_LIBRARIES})
rclcpp_components_register_node(
  point_cloud_downsampler_component
  PLUGIN "PointCloudDownsampler"
  EXECUTABLE point_cloud_downsampler_component_executable
)

install(TARGETS point_cloud_exporter point_clouds_exporter nan_remover vtk_to_csv_node point_cloud_downsampler
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_point_cloud_downsampler_component)
install(TARGETS point_cloud_downsampler_component
  EXPORT export_point_cloud_downsampler_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
