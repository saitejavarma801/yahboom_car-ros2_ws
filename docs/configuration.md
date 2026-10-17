# Configuration Reference

All robot configuration lives in `src/ros_odrive/my_odrive_botwheel_explorer/`, apart from a few constants inside the Python and C++ nodes (listed at the end).

## `config/diffbot_controllers.yaml`: ros2_control

Loaded by `ros2_control_node`. It defines the controller manager and the drive controller.

| Key | Value | Notes |
|---|---|---|
| `controller_manager.update_rate` | 10 Hz | Read/write loop to the ODrives. Low; consider 50–100 Hz |
| `joint_state_broadcaster` | `JointStateBroadcaster` | Publishes `/joint_states` |
| `botwheel_explorer` | `DiffDriveController` | Controller **name** sets its topics: `/botwheel_explorer/cmd_vel`, `/botwheel_explorer/odom` |
| `left_wheel_names` | `front_left_wheel_joint`, `rear_left_wheel_joint` | Must match URDF / ros2_control joint names |
| `right_wheel_names` | `front_right_wheel_joint`, `rear_right_wheel_joint` | |
| `wheel_separation` | 0.552 m | Measured track width (see [Hardware](hardware.md#mechanical-layout)) |
| `wheel_radius` | 0.0855 m | |
| `left_wheel_radius_multiplier` | **−1.0** | Left motors are mounted mirrored |
| `right_wheel_radius_multiplier` | 1.0 | |
| `odom_frame_id` / `base_frame_id` | `odom` / `base_link` | |
| `open_loop` | true | Odometry from commands, not encoders. Set to `false` to use wheel feedback |
| `enable_odom_tf` | true | Publishes `odom→base_link` |
| `cmd_vel_timeout` | 0.5 s | Wheels stop if commands stop |
| `publish_rate` | 50 Hz | Capped by `update_rate` in practice |
| Pose/twist covariance | 0.001 (x, y, z, roll, pitch), 0.01 (yaw) | |
| `linear.x` limits | ±1.0 m/s, 1.0 m/s² | |
| `angular.z` limits | ±1.0 rad/s, ±1.0 rad/s² | |

Skid-steer note: with four wheels in contact, real yaw rate is lower than the ideal differential-drive model predicts. If odometry under-rotates, increase `wheel_separation_multiplier` (currently 1.0) until a commanded 360° turn reads as 360°.

## `config/twist_mux.yaml`: velocity arbitration

| Input | Topic | Timeout | Priority |
|---|---|---|---|
| `joystick` | `cmd_vel_joy` | 0.5 s | **100** (wins) |
| `navigation` | `cmd_vel` | 0.5 s | 10 |
| `tracker` | `cmd_vel_tracker` | 0.5 s | 5 |

Output is remapped to `/cmd_vel_mux` in the launch files. No lock topics (e-stop locks) are configured. Adding one, for example a `std_msgs/Bool` from a hardware e-stop with priority 255, is a cheap safety gain.

## `config/joystick.yaml`: gamepad

| Node | Key | Value |
|---|---|---|
| `joy_node` | `device_id` | 0 (`/dev/input/js0`) |
| | `deadzone` | 0.2 |
| | `autorepeat_rate` | 20 Hz (keeps publishing while held, so `twist_mux` does not time out) |
| `teleop_node` | `axis_linear.x` | 1 |
| | `scale_linear.x` / turbo | 0.5 / 1.0 m/s |
| | `axis_angular.yaw` | 0 |
| | `scale_angular.yaw` / turbo | 0.5 / 1.0 rad/s |
| | `enable_button` / `enable_turbo_button` | 6 / 7 |
| | `require_enable_button` | true (deadman switch) |

## `config/mapper_params_online_async.yaml`: slam_toolbox (mapping)

| Key | Value |
|---|---|
| `mode` | `mapping` |
| `odom_frame` / `map_frame` / `base_frame` | `odom` / `map` / `base_footprint` |
| `scan_topic` | `/scan` |
| `resolution` | 0.05 m |
| `max_laser_range` | 20.0 m |
| `minimum_travel_distance` / `heading` | 0.5 m / 0.5 rad |
| `map_update_interval` | 5.0 s |
| `transform_publish_period` | 0.02 s (50 Hz `map→odom`) |
| `solver_plugin` | Ceres, `SPARSE_NORMAL_CHOLESKY`, Levenberg–Marquardt |
| `use_map_saver` | true |
| `enable_interactive_mode` | true |

## `config/localization_params_online_async.yaml`: slam_toolbox (localization)

This is the same as the mapping file except for `mode: localization`, `map_file_name: map_4`, and `map_start_at_dock: true`. `robot_localization_launch.launch.py` loads this file, but **the slam_toolbox node is not in its node list**. The supported localization path is AMCL (`localization_launch.py`). To use slam_toolbox localization instead, add `slam_node` back and set `map_file_name` to an absolute path to a serialized `.posegraph/.data` pair (for example `.../map/map_2_serial`).

## `config/nav2_params.yaml`: Nav2

This is the stock Nav2 Humble parameter set with these robot-relevant values:

| Component | Key values |
|---|---|
| `amcl` | `base_frame_id: base_footprint`, `odom_frame_id: odom`, `global_frame_id: map`, `scan_topic: scan`, likelihood-field model, alphas 0.2 |
| `bt_navigator` | `global_frame: map`, `robot_base_frame: base_link`, `odom_topic: /odom` |
| `controller_server` | 20 Hz; goal checker `xy_goal_tolerance: 0.25`, `yaw_goal_tolerance: 0.1`; **DWB** with `max_vel_x: 0.26`, `min_vel_x: 0.0` (no reversing), `max_vel_theta: 1.0` |
| `local_costmap` | rolling window in `odom`, 0.05 m, `robot_radius: 0.22`, voxel layer on `/scan`, inflation 0.55 m |
| `global_costmap` | `map` frame, 0.05 m, `robot_radius: 0.22`, static + obstacle (`/scan`) + inflation 0.55 m |
| `planner_server` | `NavfnPlanner` |
| `smoother_server` | `SimpleSmoother` |
| `behavior_server` | spin, backup, drive_on_heading, wait, assisted_teleop |
| `velocity_smoother` | 20 Hz, `OPEN_LOOP`, max `[0.26, 0, 1.0]`, accel `[2.5, 0, 3.2]` |
| all | `use_sim_time: True`, overridden by the launch argument `use_sim_time:=false` |

**Footprint:** the chassis is 0.658 × 0.488 m, which needs a circumscribed radius of ≈ 0.41 m, but `robot_radius` is 0.22 m. The planner can therefore route the robot closer to obstacles than its body allows. Consider a polygon footprint, for example `footprint: "[[0.33,0.25],[0.33,-0.25],[-0.33,-0.25],[-0.33,0.25]]"` adjusted for where `base_link` actually sits on the chassis.

## Maps

| File | Size | Resolution | Origin | Installed? | Notes |
|---|---|---|---|---|---|
| `map/map_3.yaml` + `map_3.pgm` | 312 × 131 px (15.6 × 6.55 m) | 0.05 m | (−3.13, −3.98, 0) | yes | Covers the bin route (bins up to x = 15 m) |
| `map.yaml` + `map.pgm` (package root) | 212 × 79 px (10.6 × 3.95 m) | 0.05 m | (−7.95, −0.307, 0) | **no** | Earlier, smaller map |
| `map/map_2_serial.data` + `.posegraph` | — | — | — | yes | slam_toolbox serialized graph |

`localization_launch.py` defaults to `map_5.yaml`, and the localization slam config refers to `map_4`. **Neither is in the repository**, so always pass `map:=<absolute path>`.

## URDF / xacro

| File | Purpose | Arguments |
|---|---|---|
| `rashwan_bot.urdf.xacro` | **Real robot** top-level model | `prefix` (""), `use_mock_hardware` (false) |
| `rashwan_bot_desc.urdf.xacro` | Links and joints, dimensions | — |
| `rashwan_lidar.xacro`, `rashwan_camera.xacro` | Sensor frames + Gazebo sensors | — |
| `rashwan_inertial.xacro` | Inertia macros | — |
| `rashwan_gazebo.xacro` | Gazebo Classic diff-drive plugin (not matched to the real joints) | — |
| `diffbot.ros2_control.xacro` | ros2_control hardware block: ODrive plugin, `can0`, node IDs 0–3 | `name`, `prefix`, `use_mock_hardware` |
| `diffbot.urdf.xacro`, `diffbot_description.urdf.xacro` | Placeholder base from the BotWheel example | — |
| `my_bot.urdf.xacro` | Unused draft | — |

Render the real model for inspection:

```bash
xacro $(ros2 pkg prefix my_odrive_botwheel_explorer)/share/my_odrive_botwheel_explorer/urdf/rashwan_bot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

## Launch-file arguments

| Launch file | Argument | Default | Effect |
|---|---|---|---|
| `robot_mapping_launch`, `rashwan`, `my_botwheel_explorer` | `gui` | `true` | Start RViz |
| (same) | `use_mock_hardware` | `true` | **No effect**: not forwarded to xacro (hardware is always real) |
| `robot_mapping_launch`, `robot_localization_launch`, `rashwan`, `new` | `serial_port`, `serial_baudrate`, `frame_id`, `inverted`, `angle_compensate`, `channel_type` | see [Hardware → LiDAR](hardware.md#lidar) | LiDAR driver params, overridable with `name:=value`. Only `new.launch.py` declares them, so they do not show up in `ros2 launch --show-args` for the others |
| `localization_launch.py` | `map`, `use_sim_time`, `params_file`, `autostart`, `namespace`, `use_composition`, `use_respawn`, `log_level` | `map_5.yaml`, `false`, `config/nav2_params.yaml`, `true`, `""`, `False`, `False`, `info` | Standard Nav2 localization args |
| `navigation_launch.py` | `use_sim_time`, `params_file`, `autostart`, `namespace`, `use_composition`, `use_respawn`, `log_level` | `false`, `config/nav2_params.yaml`, … | Standard Nav2 navigation args. Remaps the controller output to `cmd_vel_nav` and the smoother output to `cmd_vel` |
| `ball_tracker.launch.py` | `params_file`, `detect_only`, `follow_only`, `tune_detection`, `use_sim_time`, `image_topic`, `cmd_vel_topic`, `enable_3d_tracker` | example params, `false`, `false`, `false`, `false`, `/camera/image_raw`, `/cmd_vel`, `false` | |

Camera parameters (in the launch files): `image_size: [640, 480]`, `camera_frame_id: camera_link_optical`, `time_per_frame: [1, 6]`. The Foxglove bridge in `robot_mapping_launch` / `robot_localization_launch` uses `send_buffer_limit: 100000000`, a 100 MB send buffer for streaming images and maps over Wi-Fi.

## Constants in code

These values are not ROS parameters. Change them in the source and rebuild:

| File | Constant | Value |
|---|---|---|
| `robot_nav/robot_nav/robot_nav.py` | `bins_list` | 11 (x, y) goals in `map`, index 0 = dock |
| | ID offset | 51 |
| `manipulator_control/manipulator_control/manipulator_control.py` | `sleep_time` | 5.0 s |
| | `grip_servo` / `ungrip_servo` | `[170, 20]` / `[10, 190]` |
| | arm angles | 180° home, 8° down, 85° drain |
| | DYNAMIXEL IDs | `[1, 2]` |
| | drain wait, grip wait | 15 s, 10 s |
| `servo_control/servo_control/servo_controller.py` | `SERVO_PIN_1` / `SERVO_PIN_2` | 33 / 15 (BOARD) |
| | PWM frequency, move delay | 50 Hz, 1 s |
| `DynamixelSDK/dynamixel_sdk_examples/src/read_write_node.cpp` | `DEVICE_NAME`, `BAUDRATE`, `PROTOCOL_VERSION` | `/dev/ttyUSB0`, 57600, 2.0 |
| | profile velocity / acceleration | 500 / 700 |
| | `degreeToPosition()` | ID 1: 180°→1587, 0°→3784; ID 2: 0°→1845, 180°→4030 |
| `tag_follower_pkg/tag_follower_pkg/tag_follower.py` | centring dead-bands | 5 px (centre), 10 px (margins) |
| | post-trigger block | 120 s |
| | AprilTag family | `apriltag.Detector()` default (tag36h11) |
| `robot_localization_launch.launch.py` | micro-ROS agent | `udp4 -p 8888 -v6` |
