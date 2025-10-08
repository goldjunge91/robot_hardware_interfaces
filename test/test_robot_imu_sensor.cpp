#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>

#include "robot_hardware_interfaces/robot_imu_sensor.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using robot_hardware_interfaces::RobotImuSensor;
using hardware_interface::HardwareInfo;

class RobotImuSensorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    
    // Create hardware info for IMU sensor
    hardware_info_.name = "imu_sensor";
    hardware_info_.type = "sensor";
    
    // Add sensor configuration
    hardware_interface::ComponentInfo sensor_info;
    sensor_info.name = "imu_sensor";
    sensor_info.type = "sensor";
    
    // Add 10 state interfaces (orientation quaternion, angular velocity, linear acceleration)
    hardware_interface::InterfaceInfo orientation_x;
    orientation_x.name = "orientation.x";
    sensor_info.state_interfaces.push_back(orientation_x);
    
    hardware_interface::InterfaceInfo orientation_y;
    orientation_y.name = "orientation.y";
    sensor_info.state_interfaces.push_back(orientation_y);
    
    hardware_interface::InterfaceInfo orientation_z;
    orientation_z.name = "orientation.z";
    sensor_info.state_interfaces.push_back(orientation_z);
    
    hardware_interface::InterfaceInfo orientation_w;
    orientation_w.name = "orientation.w";
    sensor_info.state_interfaces.push_back(orientation_w);
    
    hardware_interface::InterfaceInfo angular_velocity_x;
    angular_velocity_x.name = "angular_velocity.x";
    sensor_info.state_interfaces.push_back(angular_velocity_x);
    
    hardware_interface::InterfaceInfo angular_velocity_y;
    angular_velocity_y.name = "angular_velocity.y";
    sensor_info.state_interfaces.push_back(angular_velocity_y);
    
    hardware_interface::InterfaceInfo angular_velocity_z;
    angular_velocity_z.name = "angular_velocity.z";
    sensor_info.state_interfaces.push_back(angular_velocity_z);
    
    hardware_interface::InterfaceInfo linear_acceleration_x;
    linear_acceleration_x.name = "linear_acceleration.x";
    sensor_info.state_interfaces.push_back(linear_acceleration_x);
    
    hardware_interface::InterfaceInfo linear_acceleration_y;
    linear_acceleration_y.name = "linear_acceleration.y";
    sensor_info.state_interfaces.push_back(linear_acceleration_y);
    
    hardware_interface::InterfaceInfo linear_acceleration_z;
    linear_acceleration_z.name = "linear_acceleration.z";
    sensor_info.state_interfaces.push_back(linear_acceleration_z);
    
    hardware_info_.sensors.push_back(sensor_info);
    
    // Add hardware parameters
    hardware_info_.hardware_parameters["connection_timeout_ms"] = "5000";
    hardware_info_.hardware_parameters["connection_check_period_ms"] = "100";
  }

  void TearDown() override
  {
    // Clean up any sensor instances
    if (imu_sensor_) {
      rclcpp_lifecycle::State state;
      imu_sensor_->on_shutdown(state);
      imu_sensor_.reset();
    }
  }

  HardwareInfo hardware_info_;
  std::unique_ptr<RobotImuSensor> imu_sensor_;
};

TEST_F(RobotImuSensorTest, InitializationSucceeds)
{
  imu_sensor_ = std::make_unique<RobotImuSensor>();
  auto result = imu_sensor_->on_init(hardware_info_);
  EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(RobotImuSensorTest, ConfigurationSucceeds)
{
  imu_sensor_ = std::make_unique<RobotImuSensor>();
  imu_sensor_->on_init(hardware_info_);
  
  rclcpp_lifecycle::State state;
  auto result = imu_sensor_->on_configure(state);
  EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(RobotImuSensorTest, ExportsCorrectNumberOfStateInterfaces)
{
  imu_sensor_ = std::make_unique<RobotImuSensor>();
  imu_sensor_->on_init(hardware_info_);
  
  auto state_interfaces = imu_sensor_->export_state_interfaces();
  EXPECT_EQ(state_interfaces.size(), 10u);  // 4 orientation + 3 angular velocity + 3 linear acceleration
}

TEST_F(RobotImuSensorTest, StateInterfaceNamesAreCorrect)
{
  imu_sensor_ = std::make_unique<RobotImuSensor>();
  imu_sensor_->on_init(hardware_info_);
  
  auto state_interfaces = imu_sensor_->export_state_interfaces();
  
  EXPECT_EQ(state_interfaces[0].get_interface_name(), "orientation.x");
  EXPECT_EQ(state_interfaces[1].get_interface_name(), "orientation.y");
  EXPECT_EQ(state_interfaces[2].get_interface_name(), "orientation.z");
  EXPECT_EQ(state_interfaces[3].get_interface_name(), "orientation.w");
  EXPECT_EQ(state_interfaces[4].get_interface_name(), "angular_velocity.x");
  EXPECT_EQ(state_interfaces[5].get_interface_name(), "angular_velocity.y");
  EXPECT_EQ(state_interfaces[6].get_interface_name(), "angular_velocity.z");
  EXPECT_EQ(state_interfaces[7].get_interface_name(), "linear_acceleration.x");
  EXPECT_EQ(state_interfaces[8].get_interface_name(), "linear_acceleration.y");
  EXPECT_EQ(state_interfaces[9].get_interface_name(), "linear_acceleration.z");
}

TEST_F(RobotImuSensorTest, ReadReturnsErrorWhenNoDataReceived)
{
  imu_sensor_ = std::make_unique<RobotImuSensor>();
  imu_sensor_->on_init(hardware_info_);
  
  rclcpp::Time time;
  rclcpp::Duration period(0, 0);
  
  // Read should return ERROR when no IMU data is available
  auto result = imu_sensor_->read(time, period);
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(RobotImuSensorTest, DeactivationSucceeds)
{
  imu_sensor_ = std::make_unique<RobotImuSensor>();
  imu_sensor_->on_init(hardware_info_);
  
  rclcpp_lifecycle::State state;
  auto result = imu_sensor_->on_deactivate(state);
  EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(RobotImuSensorTest, ShutdownSucceeds)
{
  imu_sensor_ = std::make_unique<RobotImuSensor>();
  imu_sensor_->on_init(hardware_info_);
  
  rclcpp_lifecycle::State state;
  auto result = imu_sensor_->on_shutdown(state);
  EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
