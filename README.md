# robot_hardware_interfaces

The package uses [ros2_control](https://github.com/ros-controls/ros2_control) to communicate with the RObot's 2R, RObot's 2 PRO and RObot's XL hardware.
RObots contain [micro-ros](https://micro.ros.org/) firmware which provides the encoders and the imu measurements and the motors speed commander.

### Subscribes

- `/cmd_vel` (*geometry_msgs/Twist*, **/robot_base_controller**)
- `/_motors_response` (*sensor_msgs/msg/JointState*, **/robot_system_node**)
- `/_imu/data_raw` (*sensor_msgs/msg/Imu*, **/imu_sensor_node**)

### Publishes
- `/imu_broadcaster/imu` (*sensor_msgs/Imu*, **/imu_broadcaster**)
- `/robot_base_controller/odom` (*nav_msgs/Odometry*, **/robot_base_controller**)
- `/_motors_cmd` (*std_msgs/msg/Float32MultiArray*, **/robot_system_node**)

# Usage
1. Add this repository to ros2 workspace
```bash
cd src/
git clone https://github.com/husarion/robot_hardware_interfaces
```

2. Add to the Robot description following lines:
```xml
<xacro:include filename="$(find robot_hardware_interfaces)/urdf/ros2_control.urdf.xacro" />
<xacro:ros2_control_system/>
```

3. Configure and launch the controller with the example usage.
```bash
ros2 launch robot_hardware_interfaces example_diff_drive.launch.py
```