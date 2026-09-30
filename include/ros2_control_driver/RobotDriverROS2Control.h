#pragma once

#include <robot_interface/RobotDriverTemplate.h>
#include <robot_interface/driver/GripperInfo.h>

#include <mc_rtc/Configuration.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace ros2_control_driver
{

class RobotDriverROS2Control : public mc_robot_interface::RobotDriver
{
public:
  RobotDriverROS2Control(const std::string & name,
                         const std::string & ip,
                         uint16_t port,
                         const std::string & config_path,
                         const std::vector<mc_robot_interface::GripperInfo> & grippers);

  ~RobotDriverROS2Control() override;

  void sync() override;

  void setDataRead() override {}

  std::vector<double> getActualQ() override;
  std::vector<double> getActualQd() override;
  std::vector<double> getJointTorques() override;

  void servoJ(const std::vector<double> & q) override;
  void speedJ(const std::vector<double> & alpha) override;
  void tauJ(const std::vector<double> & tau) override;

private:
  void checkControllersActive(const std::string & controller_manager_node,
                              const std::vector<std::string> & required_controllers);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishTrajectoryCommand(const std::vector<double> & q);

  std::vector<size_t> resolveJointIndices(const std::vector<std::string> & joints, const std::string & topic) const;

  // One entry per controller an interface's commands are split across (e.g.
  // one per arm of a bimanual platform). indices map this group's joints to
  // positions in a full-robot vector (indexed per joint_names_).
  struct CommandGroup
  {
    std::vector<std::string> joints;
    std::vector<size_t> indices;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;
  };

  std::vector<CommandGroup> parseCommandGroups(const mc_rtc::Configuration & config, const std::string & key);
  void publishCommandGroups(const std::vector<CommandGroup> & groups,
                            const char * interface_name,
                            const std::vector<double> & values);

  std::vector<CommandGroup> position_groups_;
  std::vector<CommandGroup> velocity_groups_;
  std::vector<CommandGroup> effort_groups_;

  struct TrajectoryGroup
  {
    std::vector<std::string> joints;
    std::vector<size_t> indices;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub;
  };
  std::vector<TrajectoryGroup> trajectory_groups_;

  std::vector<std::string> joint_names_;
  // joint_names_[i] -> index in the latest /joint_states message; resolved
  // from the first message since JointState doesn't guarantee order.
  std::vector<size_t> joint_state_index_;
  bool joint_state_index_resolved_ = false;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_cmd_pub_;

  std::mutex state_mutex_;
  std::condition_variable state_cv_;
  std::vector<double> position_, velocity_, effort_;
  bool got_new_state_ = false;
};

} // namespace ros2_control_driver

// robot_interface plugin symbols
#include <robot_interface/driver/api.h>

extern "C"
{
  MC_ROBOT_DRIVER_DLLAPI void MC_RTC_ROBOT_DRIVER(std::vector<std::string> & classes);

  MC_ROBOT_DRIVER_DLLAPI mc_robot_interface::RobotDriver * create(
      const std::string & name,
      const std::string & ip,
      const uint16_t & port,
      const std::string & config_path,
      const std::vector<mc_robot_interface::GripperInfo> & grippers);

  MC_ROBOT_DRIVER_DLLAPI void destroy(mc_robot_interface::RobotDriver * ptr);
}
