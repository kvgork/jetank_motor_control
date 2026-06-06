# jetank_motor_control

`ros2_control` hardware interface + diff-drive motor driver for the JeTank tracked
base (PCA9685 over I2C / libgpiod). Owns the controller configuration that
`controller_manager` spawns (diff-drive, arm, gripper).

## ROS 2 API

`jetank_motor_control` provides the JeTank motor control and `ros2_control` hardware-interface layer. It contains **one runtime node** (`robot_controller`, the real-hardware diff-drive PWM driver), a **`ros2_control` `SystemInterface` plugin** (for real serial-bus servos), and the **controller configuration** that `controller_manager` spawns. The arm/gripper/diff-drive controllers below are standard `ros2_control` controllers configured by this package's `config/jetank_controllers.yaml` but spawned by an external `controller_manager` (e.g. in sim bringup), not by a node in this package.

### Nodes

| Node | Executable | Role |
|---|---|---|
| `robot_controller` | `robot_controller` | Real-hardware diff-drive driver: maps `Twist` `cmd_vel` to per-motor PWM via PCA9685 (I2C + libgpiod). Applies linear/angular clamps, per-wheel mixing with `track_width`, and a 1 s cmd_vel-timeout safety stop. |

### Subscribed topics

| Topic | Type | Node |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `robot_controller` |

### Published topics

| Topic | Type | Node |
|---|---|---|
| `robot_status` | `std_msgs/msg/String` | `robot_controller` (1 Hz human-readable status string) |

### Actions

These are exposed by `ros2_control` controllers defined in `config/jetank_controllers.yaml` and spawned by `controller_manager`:

| Action | Type | Role |
|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Server — `joint_trajectory_controller/JointTrajectoryController` over arm joints `S1_joint, S2_joint, S3_joint, S5_joint`. |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | Server — `position_controllers/GripperActionController` driving `gripper_left_joint` (sim). |

### `robot_controller` parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `left_motor` | int | `0` | PCA9685 motor channel index (left). |
| `right_motor` | int | `1` | PCA9685 motor channel index (right). |
| `left_motor_alpha` | double | `1.0` | Left motor gain calibration. |
| `right_motor_alpha` | double | `1.0` | Right motor gain calibration. |
| `left_motor_beta` | double | `0.0` | Left motor offset calibration. |
| `right_motor_beta` | double | `0.0` | Right motor offset calibration. |
| `track_width` | double | `0.11` | Wheel/track separation (m) used for diff-drive mixing. |
| `max_linear_velocity` | double | `1.0` | Linear velocity clamp / normalization (m/s). |
| `max_angular_velocity` | double | `2.0` | Angular velocity clamp (rad/s). |

### ros2_control hardware interface (plugin)

Exported via `jetank_hardware.xml` for real hardware:

- **Plugin class:** `jetank_motor_control/JetankSerialHardware` (base `hardware_interface::SystemInterface`) — JeTank ST/SC serial-bus servos over UART. Selected in `config/ros2_control.xacro` when `use_sim:=false` (params `serial_port=/dev/ttyTHS1`, `baud_rate=1000000`). In sim (`use_sim:=true`) the `ign_ros2_control/IgnitionSystem` plugin is used instead.

  > Note: this is a **stub** — `jetank_hardware.xml` declares the class (namespace consistent with `config/ros2_control.xacro`), but there is no C++ implementation yet and `CMakeLists.txt` neither builds the library nor calls `pluginlib_export_plugin_description_file`. The real robot currently drives via the `robot_controller` node, not this interface. See the comment in `jetank_hardware.xml` for how to enable it.

### Controllers configured (`config/jetank_controllers.yaml`)

`controller_manager` runs at 50 Hz and manages:

| Controller | Type | Joints / target |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | publishes `/joint_states` |
| `arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | `S1_joint, S2_joint, S3_joint, S5_joint` |
| `gripper_controller` | `position_controllers/GripperActionController` | `gripper_left_joint` |
| `gripper_right_mimic_controller` | `forward_command_controller/ForwardCommandController` | `gripper_right_joint` (sim-only) |
| `diff_drive_controller` | `diff_drive_controller/DiffDriveController` | `front_left/right`, `rear_left/right` wheel joints; `wheel_separation=0.14`, `wheel_radius=0.03`, frames `base_footprint`/`odom` |

### Launch entrypoints

| File | Status |
|---|---|
| `launch/test_urdf.launch.py` | Loads `jetank_description` URDF (xacro `use_sim:=true use_ros2_control:=true`) into `robot_state_publisher` + `joint_state_publisher_gui` + RViz. Does **not** start `robot_controller` or `controller_manager`. |
| `launch/gazebo_sim.launch.py` | **Deprecated** (Gazebo Classic). Emits a deprecation notice only; use `jetank_simulation` launches instead. |
