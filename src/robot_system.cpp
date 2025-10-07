#include "robot_hardware_interfaces/robot_system.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>

#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robot_hardware_interfaces
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  for (auto& j : info_.joints)
  {
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Joint '%s' found", j.name.c_str());

    pos_state_[j.name] = 0.0;
    vel_state_[j.name] = 0.0;
    vel_commands_[j.name] = 0.0;
  }

  connection_timeout_ms_ = std::stoul(info_.hardware_parameters["connection_timeout_ms"]);
  connection_check_period_ms_ = std::stoul(info_.hardware_parameters["connection_check_period_ms"]);

  // Read wheel parameters for kinematics (with defaults matching robot_xl)
  wheel_radius_ = info_.hardware_parameters.count("wheel_radius") > 0 ? std::stod(info_.hardware_parameters["wheel_"
                                                                                                            "radius"]) :
                                                                        0.047;  // 47mm default
  wheel_base_ = info_.hardware_parameters.count("wheel_base") > 0 ? std::stod(info_.hardware_parameters["wheel_base"]) :
                                                                    0.220;  // Average of x and y separation

  std::string velocity_command_joint_order_raw = info_.hardware_parameters["velocity_command_joint_order"];
  // remove whitespaces
  velocity_command_joint_order_raw.erase(
      std::remove_if(velocity_command_joint_order_raw.begin(), velocity_command_joint_order_raw.end(),
                     [](char c) { return std::isspace(static_cast<unsigned char>(c)); }),
      velocity_command_joint_order_raw.end());
  std::stringstream velocity_command_joint_order_stream(velocity_command_joint_order_raw);
  std::string joint_name;
  while (getline(velocity_command_joint_order_stream, joint_name, ','))
  {
    velocity_command_joint_order_.push_back(joint_name);
  }

  if (velocity_command_joint_order_.size() != info_.joints.size())
  {
    RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Joint order size is invalid");
    return CallbackReturn::ERROR;
  }

  for (auto& j : info_.joints)
  {
    if (std::find(velocity_command_joint_order_.begin(), velocity_command_joint_order_.end(), j.name) ==
        velocity_command_joint_order_.end())
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Joint '%s' missing from velocity command joint order",
                   j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  node_ = std::make_shared<rclcpp::Node>("robot_system_node");
  executor_.add_node(node_);
  executor_thread_ =
      std::make_unique<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Configuring");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Activating");

  for (const auto& x : pos_state_)
  {
    pos_state_[x.first] = 0.0;
    vel_state_[x.first] = 0.0;
    vel_commands_[x.first] = 0.0;
  }

  // Create Twist publisher for velocity commands to firmware
  cmd_vel_publisher_ = node_->create_publisher<Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
  realtime_cmd_vel_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<Twist>>(cmd_vel_publisher_);

  // Subscribe to /joint_states directly (standardized topic from firmware)
  motor_state_subscriber_ = node_->create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(), std::bind(&RobotSystem::motor_state_cb, this, std::placeholders::_1));

  // Initialize last command time
  last_command_time_ = node_->get_clock()->now();

  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"),
              "Waiting for first joint state message from firmware (timeout: %u ms)...", connection_timeout_ms_);

  // Wait for first joint state message
  auto start_time = node_->get_clock()->now();
  std::shared_ptr<JointState> initial_state;
  double timeout_seconds = connection_timeout_ms_ / 1000.0;

  while (rclcpp::ok())
  {
    received_motor_state_msg_ptr_.get(initial_state);

    if (initial_state)
    {
      RCLCPP_INFO(rclcpp::get_logger("RobotSystem"),
                  "Successfully activated with real hardware feedback. "
                  "Received joint states for %zu joints. "
                  "Parameters: wheel_radius=%.3f m, wheel_base=%.3f m",
                  initial_state->name.size(), wheel_radius_, wheel_base_);
      return CallbackReturn::SUCCESS;
    }

    if ((node_->get_clock()->now() - start_time).seconds() > timeout_seconds)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"),
                   "Timeout (%.1f s) waiting for joint states from firmware. "
                   "Ensure micro-ROS agent is running and firmware is publishing to /joint_states",
                   timeout_seconds);
      return CallbackReturn::ERROR;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(connection_check_period_ms_));
  }

  return CallbackReturn::ERROR;
}

CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Deactivating");

  // Publish final zero velocity command for safety
  if (realtime_cmd_vel_publisher_ && realtime_cmd_vel_publisher_->trylock())
  {
    auto& cmd_vel_msg = realtime_cmd_vel_publisher_->msg_;
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;
    realtime_cmd_vel_publisher_->unlockAndPublish();
  }

  cleanup_node();
  received_motor_state_msg_ptr_.set(nullptr);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Shutting down");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_error(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Handling error");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_state_[info_.joints[i].name]));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_state_[info_.joints[i].name]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[info_.joints[i].name]));
  }

  return command_interfaces;
}

void RobotSystem::cleanup_node()
{
  motor_state_subscriber_.reset();
  realtime_cmd_vel_publisher_.reset();
  cmd_vel_publisher_.reset();

  // Stop executor thread before destruction
  if (executor_thread_ && executor_thread_->joinable())
  {
    executor_.cancel();
    executor_thread_->join();
  }
}

void RobotSystem::motor_state_cb(const std::shared_ptr<JointState> msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "Received motors response");
  received_motor_state_msg_ptr_.set(std::move(msg));
}

return_type RobotSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  std::shared_ptr<JointState> motor_state;
  received_motor_state_msg_ptr_.get(motor_state);

  RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"), "Reading motors state");

  // No mock mode - return ERROR if no data received from firmware
  if (!motor_state)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RobotSystem"), *node_->get_clock(), 1000,
                          "No joint states received from firmware");
    return return_type::ERROR;
  }

  // Verify and map joint names from firmware
  // Expected names: front_left_wheel_joint, front_right_wheel_joint,
  //                 rear_left_wheel_joint, rear_right_wheel_joint
  for (auto i = 0u; i < motor_state->name.size(); i++)
  {
    if (pos_state_.find(motor_state->name[i]) == pos_state_.end() ||
        vel_state_.find(motor_state->name[i]) == vel_state_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"),
                   "Joint name mismatch: received '%s' but not found in configured joints. "
                   "Expected: front_left_wheel_joint, front_right_wheel_joint, "
                   "rear_left_wheel_joint, rear_right_wheel_joint",
                   motor_state->name[i].c_str());
      return return_type::ERROR;
    }

    // Update position and velocity state interfaces
    pos_state_[motor_state->name[i]] = motor_state->position[i];
    vel_state_[motor_state->name[i]] = motor_state->velocity[i];

    RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"), "Joint '%s' - Position: %.3f rad, Velocity: %.3f rad/s",
                 motor_state->name[i].c_str(), pos_state_[motor_state->name[i]], vel_state_[motor_state->name[i]]);
  }
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time& time, const rclcpp::Duration&)
{
  if (!realtime_cmd_vel_publisher_)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RobotSystem"), *node_->get_clock(), 1000,
                          "Realtime publisher not initialized");
    return return_type::ERROR;
  }

  // Check for command timeout (500ms)
  const double timeout_seconds = 0.5;
  double time_since_last_command = (time - last_command_time_).seconds();

  if (realtime_cmd_vel_publisher_->trylock())
  {
    auto& cmd_vel_msg = realtime_cmd_vel_publisher_->msg_;

    // Safety timeout: publish zero velocity if no commands received for 500ms
    if (time_since_last_command > timeout_seconds)
    {
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.linear.y = 0.0;
      cmd_vel_msg.angular.z = 0.0;

      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RobotSystem"), *node_->get_clock(), 1000,
                           "Velocity command timeout (%.2fs since last command), publishing zero velocity",
                           time_since_last_command);
    }
    else
    {
      // Apply mecanum drive forward kinematics: wheel velocities → robot velocity
      // For mecanum drive:
      // vx = (vfl + vfr + vrl + vrr) / 4 * r
      // vy = (-vfl + vfr + vrl - vrr) / 4 * r
      // ω = (-vfl + vfr - vrl + vrr) / 4 * r / L

      double vfl = vel_commands_["front_left_wheel_joint"];
      double vfr = vel_commands_["front_right_wheel_joint"];
      double vrl = vel_commands_["rear_left_wheel_joint"];
      double vrr = vel_commands_["rear_right_wheel_joint"];

      // Calculate normalized velocities (before applying wheel radius)
      double vx_normalized = (vfl + vfr + vrl + vrr) / 4.0;
      double vy_normalized = (-vfl + vfr + vrl - vrr) / 4.0;
      double omega_normalized = (-vfl + vfr - vrl + vrr) / 4.0;

      // Apply wheel radius and wheel base
      cmd_vel_msg.linear.x = vx_normalized * wheel_radius_;
      cmd_vel_msg.linear.y = vy_normalized * wheel_radius_;
      cmd_vel_msg.angular.z = omega_normalized * wheel_radius_ / wheel_base_;

      RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"),
                   "Publishing cmd_vel: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f", cmd_vel_msg.linear.x,
                   cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);

      last_command_time_ = time;
    }

    realtime_cmd_vel_publisher_->unlockAndPublish();
  }

  return return_type::OK;
}

}  // namespace robot_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_hardware_interfaces::RobotSystem, hardware_interface::SystemInterface)
