cmake_minimum_required(VERSION 3.5)
project(fsmt_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)

nav2_package()

include_directories(
  include
)

add_subdirectory(external/free_space_motion_tube)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

add_library(fsmt_controller SHARED
        src/fsmt_controller.cpp
        src/fsmt.cpp)

# prevent pluginlib from using boost
target_compile_definitions(fsmt_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(fsmt_controller
  free_space_motion_tube
)

ament_target_dependencies(fsmt_controller
  ${dependencies}
)

install(TARGETS fsmt_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(fsmt_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core fsmt_controller.xml)

ament_package()

