cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME ros2_control_driver)
set(PROJECT_VERSION 1.0.0)
project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION ${PROJECT_VERSION})

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

option(BUILD_TESTING "Build tests" OFF)

find_package(unified_robot_interface REQUIRED)
find_package(mc_rtc REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(fmt REQUIRED)

add_subdirectory(src)

install(DIRECTORY include/ DESTINATION include)
