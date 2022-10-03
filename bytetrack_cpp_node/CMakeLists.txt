cmake_minimum_required(VERSION 3.5)
project(bytetrack_cpp_node)

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
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(bboxes_ex_msgs REQUIRED)
find_package(bytetrack_cpp REQUIRED)

# add executable
add_executable(bytetrack_cpp_node
  src/bytetrack_cpp_node.cpp
)
ament_target_dependencies(bytetrack_cpp_node
  rclcpp
  rclcpp_components
  std_msgs
  bytetrack_cpp
  bboxes_ex_msgs
)
target_include_directories(bytetrack_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS
bytetrack_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# add component
add_library(bytetrack_cpp_component SHARED
  src/bytetrack_cpp_node.cpp
)
rclcpp_components_register_nodes(bytetrack_cpp_component
 "bytetrack_cpp_node::ByteTrackNode")
target_compile_definitions(bytetrack_cpp_component
  PRIVATE "BYTETRACK_CPP_ROS_BUILDING_LIBRARY"
)
target_compile_options(bytetrack_cpp_component PUBLIC -Wall)
target_include_directories(bytetrack_cpp_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bytetrack_cpp_component
  rclcpp
  rclcpp_components
  std_msgs
  bytetrack_cpp
  bboxes_ex_msgs
  )
install(TARGETS bytetrack_cpp_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

# launch, param, etc...
install(DIRECTORY
  launch
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
