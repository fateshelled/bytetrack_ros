cmake_minimum_required(VERSION 3.17)
project(bytetrack_viewer)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(bboxes_ex_msgs REQUIRED)

add_compile_definitions(
  CV_BRIDGE_VERSION_MAJOR=${cv_bridge_VERSION_MAJOR}
  CV_BRIDGE_VERSION_MINOR=${cv_bridge_VERSION_MINOR}
  CV_BRIDGE_VERSION_PATCH=${cv_bridge_VERSION_PATCH}
)

# add executable
add_executable(bytetrack_viewer
  src/bytetrack_viewer.cpp
)
ament_target_dependencies(bytetrack_viewer
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  message_filters
  cv_bridge
  OpenCV
  bboxes_ex_msgs
)
target_include_directories(bytetrack_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS
bytetrack_viewer
  DESTINATION lib/${PROJECT_NAME}
)

# add component
add_library(bytetrack_viewer_component SHARED
  src/bytetrack_viewer.cpp
)
rclcpp_components_register_nodes(bytetrack_viewer_component
 "bytetrack_viewer::ByteTrackViewer")
target_compile_definitions(bytetrack_viewer_component
  PRIVATE "BYTETRACK_VIEWER_BUILDING_LIBRARY"
)
target_compile_options(bytetrack_viewer_component PUBLIC -Wall)
target_include_directories(bytetrack_viewer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bytetrack_viewer_component
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  message_filters
  cv_bridge
  OpenCV
  bboxes_ex_msgs
)
install(TARGETS bytetrack_viewer_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

# launch, param, etc...
install(DIRECTORY
  # launch
  # param
  DESTINATION share/${PROJECT_NAME}
)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
