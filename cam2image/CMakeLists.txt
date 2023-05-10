cmake_minimum_required(VERSION 3.8)
project(cam2image)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_library(cam2image SHARED
  src/cam_stream.cpp
)

target_compile_definitions(cam2image
  PRIVATE "IMAGE_TOOLS_BUILDING_DLL")

ament_target_dependencies(cam2image
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "rclcpp_components"
  "OpenCV")
rclcpp_components_register_node(cam2image PLUGIN "cam2image::Cam2Image" EXECUTABLE cam_stream)

install(TARGETS
  cam2image
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
