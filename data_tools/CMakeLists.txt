cmake_minimum_required(VERSION 3.8)
project(data_tools)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# To disable annoying PCL CMAKE policy warning
cmake_policy(SET CMP0074 NEW)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(jsoncpp REQUIRED)

add_executable(pc2_to_pcd src/convert2pcd.cpp)
target_include_directories(pc2_to_pcd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pc2_to_pcd rclcpp sensor_msgs pcl_ros geometry_msgs)

add_executable(msg_to_image src/convert2jpg.cpp)
target_include_directories(msg_to_image PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(msg_to_image rclcpp sensor_msgs geometry_msgs OpenCV cv_bridge)

add_executable(pcd2xyzi src/pcd2xyzi.cpp)
target_include_directories(pcd2xyzi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pcd2xyzi PCL pcl_ros)

add_executable(image_cloud2file src/synced_image_cloud_2file.cpp)
target_include_directories(image_cloud2file PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(image_cloud2file pcl_ros rclcpp sensor_msgs OpenCV cv_bridge)

add_executable(file2image_cloud src/synced_file2image_cloud_msgs.cpp)
target_include_directories(file2image_cloud PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(file2image_cloud pcl_ros rclcpp sensor_msgs OpenCV cv_bridge)

# add_executable(feature_extractor src/pcd2classifier_features.cpp)
# target_include_directories(feature_extractor PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(feature_extractor pcl_ros rclcpp sensor_msgs OpenCV jsoncpp)
# target_link_libraries(feature_extractor
#   jsoncpp # Link the jsoncpp library
# )

install(TARGETS
    pc2_to_pcd
    msg_to_image
    pcd2xyzi
    image_cloud2file
    file2image_cloud
    # feature_extractor
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/roi_visualizer_node.py
  ${PROJECT_NAME}/bag_inspection_tool_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
