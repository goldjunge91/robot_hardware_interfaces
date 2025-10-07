#include "robot_hardware_interfaces/robot_system.hpp"

#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robot_hardware_interfaces
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotSystem"), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotSystem"), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotSystem"), "Joint '%s' has %zu state interface. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotSystem"), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotSystem"), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  for (auto & j : info_.joints) {
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Joint '%s' found", j.name.c_str());

    pos_state_[j.name] = 0.0;
    vel_state_[j.name] = 0.0;
    vel_commands_[j.name] = 0.0;
  }

  connection_timeout_ms_ = std::stoul(info_.hardware_parameters["connection_timeout_ms"]);
  connection_check_period_ms_ = std::stoul(info_.hardware_parameters["connection_check_period_ms"]);

  std::string velocity_command_joint_order_raw =
    info_.hardware_parameters["velocity_command_joint_order"];
  // remove whitespaces
  velocity_command_joint_order_raw.erase(
    std::remove_if(
      velocity_command_joint_order_raw.begin(), velocity_command_joint_order_raw.end(),
      [](char c) {return std::isspace(static_cast<unsigned char>(c));}),
    velocity_command_joint_order_raw.end());
  std::stringstream velocity_command_joint_order_stream(velocity_command_joint_order_raw);
  std::string joint_name;
  while (getline(velocity_command_joint_order_stream, joint_name, ',')) {
    velocity_command_joint_order_.push_back(joint_name);
  }

  if (velocity_command_joint_order_.size() != info_.joints.size()) {
    RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Joint order size is invalid");
    return CallbackReturn::ERROR;
  }

  for (auto & j : info_.joints) {
    if (std::find(
        velocity_command_joint_order_.begin(), velocity_command_joint_order_.end(),
        j.name) ==
      velocity_command_joint_order_.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotSystem"), "Joint '%s' missing from velocity command joint order",
        j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  node_ = std::make_shared<rclcpp::Node>("robot_system_node");
  executor_.add_node(node_);
  executor_thread_ =
    std::make_unique<std::thread>(
    std::bind(
      &rclcpp::executors::MultiThreadedExecutor::spin,
      &executor_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Configuring");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Activating");

  for (const auto & x : pos_state_) {
    pos_state_[x.first] = 0.0;
    vel_state_[x.first] = 0.0;
    vel_commands_[x.first] = 0.0;
  }

  // Note: We don't publish motors_cmd anymore since the firmware expects cmd_vel directly
  // The mecanum_drive_controller will publish cmd_vel which goes directly to the Pico
  // motor_command_publisher_ = node_->create_publisher<Float32MultiArray>(
  //   "~/motors_cmd",
  //   rclcpp::SensorDataQoS());
  // realtime_motor_command_publisher_ =
  //   std::make_shared<realtime_tools::RealtimePublisher<Float32MultiArray>>(motor_command_publisher_);

  motor_state_subscriber_ =
    node_->create_subscription<JointState>(
    "~/motors_response", rclcpp::SensorDataQoS(),
    std::bind(&RobotSystem::motor_state_cb, this, std::placeholders::_1));

  RCLCPP_WARN(
    rclcpp::get_logger("RobotSystem"),
    "Activating without waiting for motor feedback (mock mode enabled).");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Deactivating");
  cleanup_node();
  received_motor_state_msg_ptr_.set(nullptr);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Shutting down");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Handling error");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &pos_state_[info_.joints[i].name]));
    state_interfaces.emplace_back(
      StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_state_[info_.joints[i].name]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[info_.joints[i].name]));
  }

  return command_interfaces;
}

void RobotSystem::cleanup_node()
{
  motor_state_subscriber_.reset();
  // realtime_motor_command_publisher_.reset();
  // motor_command_publisher_.reset();
}

void RobotSystem::motor_state_cb(const std::shared_ptr<JointState> msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "Received motors response");
  received_motor_state_msg_ptr_.set(std::move(msg));
}

return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::shared_ptr<JointState> motor_state;
  received_motor_state_msg_ptr_.get(motor_state);

  RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"), "Reading motors state");

  if (!motor_state) {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger("RobotSystem"),
      *node_->get_clock(), 10000,
      "No feedback from motors, using mock values");
    
    // Mock behavior: integrate velocity commands to position
    for (const auto & joint_name : velocity_command_joint_order_) {
      pos_state_[joint_name] += vel_commands_[joint_name] * period.seconds();
      vel_state_[joint_name] = vel_commands_[joint_name];
    }
    return return_type::OK;
  }

  for (auto i = 0u; i < motor_state->name.size(); i++) {
    if (pos_state_.find(motor_state->name[i]) == pos_state_.end() ||
      vel_state_.find(motor_state->name[i]) == vel_state_.end())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotSystem"), "Position or velocity feedback not found for joint %s",
        motor_state->name[i].c_str());
      return return_type::ERROR;
    }

    pos_state_[motor_state->name[i]] = motor_state->position[i];
    vel_state_[motor_state->name[i]] = motor_state->velocity[i];

    RCLCPP_DEBUG(
      rclcpp::get_logger("RobotSystem"), "Position feedback: %f, velocity feedback: %f",
      pos_state_[motor_state->name[i]], vel_state_[motor_state->name[i]]);
  }
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Note: We don't publish motor commands here anymore since the mecanum_drive_controller
  // publishes cmd_vel directly to the Pico firmware. The hardware interface just tracks
  // the commanded velocities for state feedback.
  
  RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"), "Hardware interface write - commands tracked");
  
  return return_type::OK;
}

}  // namespace robot_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interfaces::RobotSystem,
  hardware_interface::SystemInterface)
