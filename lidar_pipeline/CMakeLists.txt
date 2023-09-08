cmake_minimum_required(VERSION 3.8)
project(lidar_pipeline)
add_definitions(-std=c++17)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(message_filters)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pipeline_interfaces REQUIRED)

add_executable(perception_node src/perception_node.cpp src/lidar_processing.cpp)
target_include_directories(perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(perception_node rclcpp tf2_ros tf2 tf2_eigen sensor_msgs visualization_msgs vision_msgs geometry_msgs pcl_ros pipeline_interfaces)

add_executable(projection_node src/lidar2cam_proj.cpp)
target_include_directories(projection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(projection_node rclcpp sensor_msgs vision_msgs message_filters pcl_ros OpenCV cv_bridge)

add_executable(transform_explorer_node src/transform_explorer_node.cpp)
target_include_directories(transform_explorer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(transform_explorer_node rclcpp tf2_ros tf2 tf2_eigen sensor_msgs pcl_ros)


install(TARGETS
    perception_node
    projection_node
    transform_explorer_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    configs
    DESTINATION share/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()
endif()

ament_package()
