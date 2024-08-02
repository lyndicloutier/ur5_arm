cmake_minimum_required(VERSION 3.8)
project(ur5_arm_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur5_arm_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

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

add_executable(ur5_robot_arm src/ur5_arm_test.cpp)
ament_target_dependencies(ur5_robot_arm rclcpp ur5_arm_msgs geometry_msgs moveit_ros_planning_interface)

add_executable(service_test src/service_test.cpp)
ament_target_dependencies(service_test rclcpp ur5_arm_msgs)

install(TARGETS
   ur5_robot_arm
   service_test
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
