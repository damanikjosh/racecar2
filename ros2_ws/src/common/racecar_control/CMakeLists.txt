cmake_minimum_required(VERSION 3.8)
project(racecar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(racecar_controller src/racecar_controller_node.cpp src/racecar_odometry.cpp)

ament_target_dependencies(racecar_controller rclcpp geometry_msgs nav_msgs ackermann_msgs realtime_tools)

install(TARGETS
  racecar_controller
  DESTINATION lib/${PROJECT_NAME})

# add_executable(cmd_vel_converter src/cmd_vel_converter.cpp)

# ament_target_dependencies(cmd_vel_converter rclcpp geometry_msgs)

# install(TARGETS cmd_vel_converter
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
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
