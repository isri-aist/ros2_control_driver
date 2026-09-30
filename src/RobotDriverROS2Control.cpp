#include <ros2_control_driver/RobotDriverROS2Control.h>

#include <algorithm>
#include <chrono>
#include <fmt/core.h>
#include <iterator>
#include <numeric>
#include <stdexcept>

#include <mc_rtc/Configuration.h>

namespace ros2_control_driver
{

RobotDriverROS2Control::RobotDriverROS2Control(const std::string & name,
                                               const std::string & /*ip*/,
                                               uint16_t /*port*/,
                                               const std::string & config_path,
                                               const std::vector<mc_robot_interface::GripperInfo> & grippers)
{
  auto config = mc_rtc::Configuration(config_path);

  joint_names_ = config("joint_names", std::vector<std::string>{});
  if(joint_names_.empty())
  {
    throw std::runtime_error("[RobotDriverROS2Control] config must list non-empty 'joint_names'");
  }

  // Grippers come from the RobotModule (via the manager's init query), not
  // from this static config, so their joints may not be listed above yet:
  // append any that aren't, otherwise servoJ()'s incoming q (sized for the
  // full robot, grippers included) silently overruns joint_names_.
  for(const auto & gripper : grippers)
  {
    for(const auto & j : gripper.joints)
    {
      if(std::find(joint_names_.begin(), joint_names_.end(), j) == joint_names_.end())
      {
        joint_names_.push_back(j);
      }
    }
  }

  joint_state_index_.assign(joint_names_.size(), 0);
  position_.assign(joint_names_.size(), 0.0);
  velocity_.assign(joint_names_.size(), 0.0);
  effort_.assign(joint_names_.size(), 0.0);

  const std::string joint_states_topic = config("joint_states_topic", std::string{"/joint_states"});
  const std::string controller_manager_node = config("controller_manager_node", std::string{"/controller_manager"});
  const auto required_controllers = config("required_controllers", std::vector<std::string>{});

  if(!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  node_ = std::make_shared<rclcpp::Node>("mc_rtc_ros2_control_driver_" + name);

  if(!required_controllers.empty())
  {
    checkControllersActive(controller_manager_node, required_controllers);
  }

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });

  position_groups_ = parseCommandGroups(config, "position_command_topic");
  velocity_groups_ = parseCommandGroups(config, "velocity_command_topic");
  effort_groups_ = parseCommandGroups(config, "effort_command_topic");

  if(config.has("joint_trajectory_topics"))
  {
    for(const auto & g : config("joint_trajectory_topics"))
    {
      TrajectoryGroup group;
      const std::string topic = g("topic");
      group.joints = g("joints", std::vector<std::string>{});
      if(group.joints.empty())
      {
        throw std::runtime_error(
            fmt::format("[RobotDriverROS2Control] joint_trajectory_topics entry for '{}' has empty 'joints'", topic));
      }
      group.indices = resolveJointIndices(group.joints, topic);
      group.pub = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(topic, 1);
      trajectory_groups_.push_back(std::move(group));
    }
  }

  // Maps a gripper (by name, as declared on the RobotModule and forwarded
  // via `grippers` above) to the ROS2 topic that actually serves it -- e.g.
  // a JointTrajectoryController distinct from the arm's ForwardCommandController.
  // See etc/ros2_control/openarm_bimanual.yaml.
  if(config.has("gripper_command_topics"))
  {
    const auto gripper_topics = config("gripper_command_topics");
    for(const auto & gripper : grippers)
    {
      if(!gripper_topics.has(gripper.name)) continue;
      const std::string topic = gripper_topics(gripper.name);
      TrajectoryGroup group;
      group.joints = gripper.joints;
      group.indices = resolveJointIndices(group.joints, topic);
      group.pub = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(topic, 1);
      trajectory_groups_.push_back(std::move(group));
    }
  }

  if(position_groups_.empty() && velocity_groups_.empty() && effort_groups_.empty() && trajectory_groups_.empty())
  {
    throw std::runtime_error("[RobotDriverROS2Control] config must set at least one of "
                             "position_command_topic(s) / velocity_command_topic(s) / effort_command_topic(s) / "
                             "joint_trajectory_topics");
  }

  executor_->add_node(node_);
  spin_thread_ = std::make_unique<std::thread>([this] { executor_->spin(); });

  fmt::print(
      "[RobotDriverROS2Control] '{}' bridging {} joints: /joint_states <- '{}', commands -> controller_manager '{}'\n",
      node_->get_name(), joint_names_.size(), joint_states_topic, controller_manager_node);
}

RobotDriverROS2Control::~RobotDriverROS2Control()
{
  if(executor_) executor_->cancel();
  if(spin_thread_ && spin_thread_->joinable()) spin_thread_->join();
}

void RobotDriverROS2Control::checkControllersActive(const std::string & controller_manager_node,
                                                    const std::vector<std::string> & required_controllers)
{
  auto client = node_->create_client<controller_manager_msgs::srv::ListControllers>(controller_manager_node
                                                                                    + "/list_controllers");
  if(!client->wait_for_service(std::chrono::seconds(5)))
  {
    throw std::runtime_error(
        fmt::format("[RobotDriverROS2Control] '{}/list_controllers' service unavailable — is controller_manager "
                    "running?",
                    controller_manager_node));
  }

  std::string last_error;
  for(int attempt = 0; attempt < 5; ++attempt)
  {
    if(attempt > 0) std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto future = client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      last_error = "list_controllers call timed out";
      continue;
    }

    const auto & controllers = future.get()->controller;
    last_error.clear();
    for(const auto & name : required_controllers)
    {
      auto it = std::find_if(controllers.begin(), controllers.end(), [&](const auto & c) { return c.name == name; });
      if(it == controllers.end())
      {
        last_error = fmt::format("required controller '{}' not found on '{}' ({} controllers seen)", name,
                                 controller_manager_node, controllers.size());
        break;
      }
      if(it->state != "active")
      {
        last_error = fmt::format("required controller '{}' is '{}', not active", name, it->state);
        break;
      }
    }
    if(last_error.empty()) return;
  }
  throw std::runtime_error(fmt::format("[RobotDriverROS2Control] {}", last_error));
}

std::vector<size_t> RobotDriverROS2Control::resolveJointIndices(const std::vector<std::string> & joints,
                                                                const std::string & topic) const
{
  std::vector<size_t> indices;
  indices.reserve(joints.size());
  for(const auto & j : joints)
  {
    auto it = std::find(joint_names_.begin(), joint_names_.end(), j);
    if(it == joint_names_.end())
    {
      throw std::runtime_error(fmt::format(
          "[RobotDriverROS2Control] '{}' references joint '{}' which is not in this robot's joint_names", topic, j));
    }
    indices.push_back(static_cast<size_t>(std::distance(joint_names_.begin(), it)));
  }
  return indices;
}

std::vector<RobotDriverROS2Control::CommandGroup> RobotDriverROS2Control::parseCommandGroups(
    const mc_rtc::Configuration & config,
    const std::string & key)
{
  std::vector<CommandGroup> groups;
  const std::string plural_key = key + "s";
  if(config.has(plural_key))
  {
    for(const auto & g : config(plural_key))
    {
      CommandGroup group;
      const std::string topic = g("topic");
      group.joints = g("joints", std::vector<std::string>{});
      if(group.joints.empty())
      {
        throw std::runtime_error(
            fmt::format("[RobotDriverROS2Control] '{}' entry for '{}' has empty 'joints'", plural_key, topic));
      }
      group.indices = resolveJointIndices(group.joints, topic);
      group.pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 1);
      groups.push_back(std::move(group));
    }
  }
  else if(config.has(key))
  {
    // Singular form: one topic covering every joint, in joint_names_ order.
    CommandGroup group;
    const std::string topic = config(key);
    group.joints = joint_names_;
    group.indices.resize(joint_names_.size());
    std::iota(group.indices.begin(), group.indices.end(), 0);
    group.pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 1);
    groups.push_back(std::move(group));
  }
  return groups;
}

void RobotDriverROS2Control::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if(!joint_state_index_resolved_)
  {
    for(size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if(it == msg->name.end())
      {
        fmt::print("[RobotDriverROS2Control] Warning: joint '{}' not found in /joint_states yet\n", joint_names_[i]);
        return;
      }
      joint_state_index_[i] = static_cast<size_t>(std::distance(msg->name.begin(), it));
    }
    joint_state_index_resolved_ = true;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  for(size_t i = 0; i < joint_names_.size(); ++i)
  {
    const size_t j = joint_state_index_[i];
    if(j < msg->position.size()) position_[i] = msg->position[j];
    if(j < msg->velocity.size()) velocity_[i] = msg->velocity[j];
    if(j < msg->effort.size()) effort_[i] = msg->effort[j];
  }
  got_new_state_ = true;
  state_cv_.notify_one();
}

void RobotDriverROS2Control::sync()
{
  std::unique_lock<std::mutex> lock(state_mutex_);
  if(!state_cv_.wait_for(lock, std::chrono::milliseconds(50), [this] { return got_new_state_; }))
  {
    fmt::print("[RobotDriverROS2Control] Warning: no new /joint_states message within timeout\n");
  }
  got_new_state_ = false;
}

std::vector<double> RobotDriverROS2Control::getActualQ()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return position_;
}

std::vector<double> RobotDriverROS2Control::getActualQd()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return velocity_;
}

std::vector<double> RobotDriverROS2Control::getJointTorques()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return effort_;
}

void RobotDriverROS2Control::publishCommandGroups(const std::vector<CommandGroup> & groups,
                                                  const char * interface_name,
                                                  const std::vector<double> & values)
{
  if(groups.empty())
  {
    fmt::print("[RobotDriverROS2Control] Warning: {} command received but no {}_command_topic(s) configured\n",
               interface_name, interface_name);
    return;
  }
  for(const auto & group : groups)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(group.indices.size());
    for(size_t idx : group.indices) msg.data.push_back(values[idx]);
    group.pub->publish(msg);
  }
}

void RobotDriverROS2Control::publishTrajectoryCommand(const std::vector<double> & q)
{
  for(const auto & group : trajectory_groups_)
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = group.joints;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(group.indices.size());
    for(size_t idx : group.indices) point.positions.push_back(q[idx]);
    // time_from_start=0: immediate target, requires interpolation_method: none.
    point.time_from_start = rclcpp::Duration(0, 0);
    msg.points.push_back(point);
    group.pub->publish(msg);
  }
}

void RobotDriverROS2Control::servoJ(const std::vector<double> & q)
{
  if(!position_groups_.empty()) publishCommandGroups(position_groups_, "position", q);
  if(!trajectory_groups_.empty()) publishTrajectoryCommand(q);
  if(position_groups_.empty() && trajectory_groups_.empty())
  {
    fmt::print("[RobotDriverROS2Control] Warning: position command received but neither "
               "position_command_topic(s) nor joint_trajectory_topics configured\n");
  }
}

void RobotDriverROS2Control::speedJ(const std::vector<double> & alpha)
{
  publishCommandGroups(velocity_groups_, "velocity", alpha);
}

void RobotDriverROS2Control::tauJ(const std::vector<double> & tau)
{
  publishCommandGroups(effort_groups_, "effort", tau);
}

} // namespace ros2_control_driver

// Lets robot_interface refuse this plugin (instead of crashing) once the
// RobotDriver interface changes and the plugin needs a rebuild.
MC_ROBOT_DRIVER_EXPORT_ABI_VERSION()

extern "C"
{
  void LOAD_GLOBAL() {}

  void MC_RTC_ROBOT_DRIVER(std::vector<std::string> & classes)
  {
    classes.push_back("RobotDriverROS2Control");
  }

  mc_robot_interface::RobotDriver * create(const std::string & name,
                                           const std::string & ip,
                                           const uint16_t & port,
                                           const std::string & config_path,
                                           const std::vector<mc_robot_interface::GripperInfo> & grippers)
  {
    return new ros2_control_driver::RobotDriverROS2Control(name, ip, port, config_path, grippers);
  }

  void destroy(mc_robot_interface::RobotDriver * ptr)
  {
    delete ptr;
  }
}
