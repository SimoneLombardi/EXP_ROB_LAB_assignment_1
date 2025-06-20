cmake_minimum_required(VERSION 3.8)
project(erl_ass1_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(erl_ass1_pkg_interfaces REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)

include_directories(
  include
  /opt/ros/humble/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(robot_manager src/robot_manager.cpp)
ament_target_dependencies(robot_manager
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  example_interfaces
  erl_ass1_pkg_interfaces
  ros2_aruco_interfaces
)

add_executable(routine_interface src/routine_interface.cpp)
ament_target_dependencies(routine_interface
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  example_interfaces
  erl_ass1_pkg_interfaces
  ros2_aruco_interfaces
)

install(TARGETS
  robot_manager
  routine_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch urdf config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
