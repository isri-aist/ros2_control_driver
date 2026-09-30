add_library(RobotDriverROS2Control SHARED RobotDriverROS2Control.cpp)

target_include_directories(
  RobotDriverROS2Control
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(RobotDriverROS2Control
                      PUBLIC unified_robot_interface::robot_driver_api)
target_link_libraries(RobotDriverROS2Control PRIVATE fmt::fmt
                                                     mc_rtc::mc_rtc_utils)

ament_target_dependencies(
  RobotDriverROS2Control
  PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  controller_manager_msgs)

add_robot_driver(RobotDriverROS2Control)

install(
  TARGETS RobotDriverROS2Control
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/robot_interface
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/robot_interface)
