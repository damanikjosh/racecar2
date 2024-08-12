cmake_minimum_required(VERSION 3.8)
project(racecar_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(cppzmq REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Garden
else()
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden")
endif()

# Add include directories
include_directories(include)

# add_library(SpringSystem
#   SHARED
#   src/SpringSystem.cc
# )

# target_include_directories(
#   SpringSystem PRIVATE include
# )

# target_link_libraries(SpringSystem PRIVATE
#   gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

add_library(RacecarController
  SHARED
  src/RacecarController.cc
)

target_include_directories(
  RacecarController PRIVATE include
)

target_link_libraries(RacecarController PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# add_executable(shock_controller src/shock_controller.cpp)
# ament_target_dependencies(shock_controller rclcpp std_msgs sensor_msgs)


install(DIRECTORY launch worlds
    DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  # SpringSystem
  RacecarController
  # shock_controller
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

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

 # The following hooks are used to ensure that the correct environment variables 
 # will be set by executing 'source install/setup.bash' after compilation. 
 # When using this template for your project, change the filenames of the 
 # files in the 'hooks' folder, to correspond to your project name. 
 ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
 ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in") 

ament_package()
