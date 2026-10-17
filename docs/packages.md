# Package Reference

This page covers each robot-specific package and node: what it does, its ROS interface, its parameters, and the hardware behind it. For the vendored upstream packages, see the README inside each package.

- [my_odrive_botwheel_explorer](#my_odrive_botwheel_explorer): bring-up, description, configs, launch
- [robot_nav](#robot_nav): bin ID → navigation goal
- [tag_follower_pkg](#tag_follower_pkg): AprilTag final approach
- [manipulator_control](#manipulator_control): pick-and-tip sequence
- [servo_control](#servo_control): gripper PWM
- [DynamixelSDK (modified)](#dynamixelsdk-modified): arm servo driver and messages
- [ball_tracker](#ball_tracker): experimental ball follower
- [odrive_ros2_control](#odrive_ros2_control): wheel hardware plugin (upstream)
- [rplidar_ros](#rplidar_ros) and [micro_ros_agent](#micro_ros_agent) (upstream)

---

## my_odrive_botwheel_explorer

**Path:** `src/ros_odrive/my_odrive_botwheel_explorer` · **Build type:** ament_cmake (installs files only; there is no compiled code)

This is the robot's bring-up package. It started as a fork of ODrive's `odrive_botwheel_explorer` example and grew to hold the full system configuration.

`CMakeLists.txt` installs these directories into `share/my_odrive_botwheel_explorer/`:

| Source dir | Installed as | Contents |
|---|---|---|
| `description/urdf/` | `urdf/` | xacro robot models (see [Hardware → Robot model](hardware.md#robot-model-and-tf-frames)) |
| `config/` | `config/` | controllers, joystick, twist_mux, Nav2, slam_toolbox parameters |
| `launch/` | `launch/` | all bring-up launch files |
| `rviz/` | `rviz/` | `mybot.rviz` (robot model, `/map`, `/scan`) |
| `map/` | `map/` | `map_3.yaml/.pgm` (15.6 × 6.55 m, the bin route), `map_2_serial.data/.posegraph` (slam_toolbox serialized pose graph) |

`map.yaml` and `map.pgm` in the package root (10.6 × 3.95 m) are **not** installed. Pass their full source path if you need them.

**Declared exec dependencies:** `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster`, `ros2controlcli`, `xacro`, `odrive_ros2_control`. The launch files also need `robot_state_publisher`, `rplidar_ros`, `v4l2_camera`, `slam_toolbox`, `joy`, `teleop_twist_joy`, `twist_mux`, `twist_stamper`, `foxglove_bridge`, `micro_ros_agent`, `dynamixel_sdk_examples`, `servo_control`, `robot_nav`, `tag_follower_pkg`, `rviz2`, and `nav2_*`. These are **not** declared in `package.xml`, so `rosdep` will not install them for you.

Every launch file and config is described in [Configuration](configuration.md) and [Operation](operation.md).

---

## robot_nav

**Path:** `src/robot_nav` · **Executable:** `robot_nav` · **Node name:** `robot_nav`

This node turns a bin's "I'm full" message into a Nav2 navigation goal.

### Interface

| Direction | Topic | Type | QoS |
|---|---|---|---|
| Subscribes | `/bin_full_id` | `std_msgs/Int32` | best-effort, depth 10 |
| Publishes | `/goal_pose` | `geometry_msgs/PoseStamped` | reliable, depth 10 |

No parameters.

### Behaviour

1. It computes `idx = msg.data − 51`. The offset of 51 is fixed by the bin firmware's ID scheme.
2. It looks up `bins_list[idx]`, a hard-coded table of (x, y) positions in the `map` frame:

   | idx | x [m] | y [m] | Meaning |
   |---|---|---|---|
   | 0 | 0.0 | 0.0 | Dock / home |
   | 1 | 2.0 | −0.004 | Bin 1 |
   | 2 | 3.5 | 0.004 | Bin 2 |
   | 3 | 5.9 | −0.004 | Bin 3 |
   | 4 | 4.7 | −0.5 | Bin 4 |
   | 5 | 9.0 | 0.8 | Bin 5 |
   | 6 | 9.0 | −0.8 | Bin 6 |
   | 7 | 12.0 | 0.8 | Bin 7 |
   | 8 | 12.0 | −0.3 | Bin 8 |
   | 9 | 15.0 | 0.3 | Bin 9 |
   | 10 | 15.0 | −0.3 | Bin 10 |

3. For `idx > 0`, it sets the goal orientation to a ±90° yaw: −90° (`z=-0.707, w=0.707`, facing −y) if the entry's y is positive, +90° (facing +y) otherwise. Either way the robot ends up perpendicular to the route, which runs along the map's x axis. It then publishes the goal. The table entries are the robot's *goal* positions, so they must be chosen so that the bin's AprilTag is in front of the camera once the goal is reached. Check this on site whenever you move a bin.
4. For `idx == 0` (dock), it fills in the goal but **never publishes it**: the publish line is commented out. The robot therefore does not go home on its own.
5. For a negative `idx`, it prints "Invalid Goal Index". An `idx` past the end of the table (> 10) **raises an IndexError and crashes the node**.

`euler_to_quaternion()` is computed and printed but not used. Its argument order is also wrong (it is called with roll = ±π/2). The hard-coded quaternions are what actually gets published.

### Hardware link

The bin coordinates only make sense in the frame of the map used for localization (`map/map_3.yaml`, origin (−3.13, −3.98)). If you re-map the site, measure the bins again in the new map frame and update `bins_list`.

---

## tag_follower_pkg

**Path:** `src/tag_follower_pkg` · **Executable:** `tag_follower` · **Node name:** `tag_follower`

This node handles the camera-guided final approach. It centres the robot on the AprilTag mounted on the bin, drives forward until the tag looks big enough (meaning the robot is at gripping distance), and then starts the manipulator.

### Interface

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/camera/image_raw` | `sensor_msgs/Image` (converted to `mono8`) |
| Publishes | `/cmd_vel_tracker` | `geometry_msgs/Twist` |
| Publishes | `/manipulator_start_flag` | `std_msgs/Int32` (1 = start) |

### Parameters

| Name | Default | Meaning |
|---|---|---|
| `linear_velocity` | 0.1 m/s | Forward speed while the tag is still too small |
| `angular_velocity` | 0.2 rad/s | Declared but **not used** |
| `size_threshold` | 187.0 px | Average tag edge length at which the robot stops. This sets the stopping distance: a bigger value stops closer |
| `proportional_gain` | 0.005 rad/s per px | Gain on horizontal pixel error |

### Algorithm (runs on every frame)

1. Detect tags with `apriltag.Detector()`. The default family is **tag36h11**. The first detection is the one it follows.
2. If no tag is visible, it publishes a zero Twist and returns.
3. `tag_size` = mean of two adjacent corner-to-corner edge lengths, in pixels.
4. Linear: `linear_velocity` if `tag_size < size_threshold`, else 0 and `goal_reached = True`.
5. Angular, in this order:
   - if |centre_x − width/2| > 5 px: `ω = −gain × centre_error` (turn toward the tag);
   - else if |left_margin − right_margin| > 10 px: `ω = gain × (left − right)` (fine symmetric alignment);
   - else `ω = 0` and `centered = True`.
6. Once `goal_reached` and `centered` are both true, it publishes `1` on `/manipulator_start_flag`. It then **sleeps 120 s inside the callback**, which blocks the node so that the arm can work undisturbed.

### Hardware link

- It depends on the camera mounting. Because the camera is on the robot's centreline (y = 0), "tag centred in the image" means "robot centred on the bin".
- `size_threshold` was tuned for this camera (640×480, about 62° HFOV) and this tag's printed size. If you change either one, tune it again: drive up to the correct gripping position by hand and log the `Tag Size` value the node prints.
- The node is **not** started by any launch file. Its entry in `robot_localization_launch.launch.py` is commented out. Run it by hand (see [Operation](operation.md#4-autonomous-bin-collection-mission)).

---

## manipulator_control

**Path:** `src/manipulator_control` · **Executable:** `manipulator_control` · **Node name:** `manipulator_control`

This node runs the fixed, timed sequence that picks up a bin, empties it into the robot, and puts it back.

### Interface

| Direction | Topic | Type | Used? |
|---|---|---|---|
| Subscribes | `/manipulator_start_flag` | `std_msgs/Int32` | yes; `data == 1` starts the sequence |
| Publishes | `/set_position` | `dynamixel_sdk_custom_interfaces/SetMultiplePositions` | yes; arm angle for IDs 1 and 2 |
| Publishes | `/servo_angles` | `std_msgs/Float64MultiArray` | yes; gripper |
| Publishes | `/cmd_vel` | `geometry_msgs/Twist` | yes; backs away and holds still |
| Publishes | `/bin_full_id` | `std_msgs/Int32` | yes; sends 51 ("dock") when finished |
| Publishes | `/cu_cmd` | `std_msgs/Int32` | publisher exists, calls are commented out |
| Publishes | `/goal_pose` | `geometry_msgs/PoseStamped` | publisher exists, never used |

There are no parameters. The timings and angles are attributes: `sleep_time = 5.0` s, `grip_servo = [170, 20]`, `ungrip_servo = [10, 190]`.

### Sequence

See [Mission workflow](mission-workflow.md#4-pick-tip-and-return) for the step-by-step timeline. The sequence takes about 70 s in total:

| Step | Command | Wait |
|---|---|---|
| 1 | Gripper open; arm → 180° (home) | 5 s |
| 2 | Arm → 8° (down) | 5 s |
| 3 | Gripper close | 10 s |
| 4 | Arm → 85° (tip / drain) | 5 s + 15 s |
| 5 | Arm → 8° (put bin down) | 5 s |
| 6 | Gripper open (sent 4 times) | 10 s |
| 7 | Arm → 180° (home) | 5 s |
| 8 | `/cmd_vel` linear.x = −0.1 (back away) | 5 s |
| 9 | `/cmd_vel` zero; `/bin_full_id` = 51 | 5 s |

All waits are `time.sleep()` calls inside the subscription callback. The node is single-threaded, so it handles nothing else until the sequence ends. Because of the twist_mux and controller timeouts (0.5 s), the single −0.1 m/s message in step 8 moves the robot for about 0.5 s (≈ 5 cm), not 5 s.

A 10 Hz timer (`publish_cmd_vel`) keeps publishing a **zero** Twist on `/cmd_vel` once the first sequence has run (`flag == 1`). This holds the robot still, but it also competes with Nav2 on the same `twist_mux` input. See [Known issues](troubleshooting.md#known-issues).

### Hardware link

- Arm angles are in degrees and use the calibrated mapping in `read_write_node`. 180° = stowed, 8° = gripping height, 85° = tipping angle over the hopper.
- The fixed waits assume the arm's profile velocity (500) and acceleration (700) set in `read_write_node`. If you speed the arm up or slow it down, re-time the waits.

---

## servo_control

**Path:** `src/servo_control` · **Executable:** `servo_control` · **Node name:** `servo_controller` · **Module:** `servo_control/servo_controller.py`

This node drives the two gripper servos with software-timed PWM on the Jetson's 40-pin header.

### Interface

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `servo_angles` (resolves to `/servo_angles`) | `std_msgs/Float64MultiArray`; exactly two elements `[angle1, angle2]` in degrees |

### Behaviour

- At start-up it calls `GPIO.setmode(GPIO.BOARD)` and sets pins 33 and 15 as outputs.
- For each message it computes `duty = angle/18 + 2` (%), creates a 50 Hz PWM on each pin, applies the duty cycle, **sleeps 1 s** for the servo to move, and then sets duty to 0 so the servo stops twitching (no holding pulses).
- If the message does not have exactly two values, it logs an error and ignores it.

### Hardware link

- Pin 33 → servo 1, pin 15 → servo 2 (BOARD numbering, physical pin numbers on the header).
- Needs `Jetson.GPIO` (which provides the `RPi.GPIO` import), and the user must be in the `gpio` group. On Jetson dev kits the pins also need PWM enabled with `jetson-io`. See [Setup](setup-and-build.md#6-gripper-pwm-pins).
- Since no pulses are sent after the 1 s move, the servos are **not actively holding**. Grip force while the arm is moving comes only from the servo gearing and the gripper's mechanics.
- The cleanup code (`pwm.stop()` / `GPIO.cleanup()`) is commented out, so the pins are not released on exit.

---

## DynamixelSDK (modified)

**Path:** `src/DynamixelSDK` · ROBOTIS SDK v3 with two packages changed for this robot.

### dynamixel_sdk_custom_interfaces

| Interface | Definition |
|---|---|
| `msg/SetPosition` | `uint8 id`, `int32 position` (upstream) |
| `msg/SetMultiplePositions` | `int32[] ids`, `float32 position` (**added**: one angle in degrees for several servos) |
| `srv/GetPosition` | request `uint8 id` → response `int32 position` (upstream) |

### dynamixel_sdk_examples → `read_write_node`

**Executable:** `read_write_node` · **Node name:** `read_write_node`

| Direction | Name | Type |
|---|---|---|
| Subscribes | `set_position` (`/set_position`) | `SetMultiplePositions`; reliable, depth `qos_depth` (default 10) |
| Service | `get_position` (`/get_position`) | `GetPosition`; returns raw present position (ticks) |

Parameter: `qos_depth` (int, default 10).

**Start-up** (before ROS init): it opens `/dev/ttyUSB0` at 57600 baud, and then over the broadcast ID sets *Operating Mode = 3 (position)*, *Profile Velocity = 500*, *Profile Acceleration = 700*, and *Torque Enable = 1*. If it cannot open the port, it exits with `-1`. The log messages say "5000" and "7000", but the values actually written are 500 and 700.

**On each `/set_position` message:** for every ID in `ids`, it clamps `position` to [0, 180]°, converts it to ticks with `degreeToPosition()`, and writes *Goal Position* (address 116). Unknown IDs are skipped with a warning.

**Calibration (`degreeToPosition`):**

| ID | 0° | 180° | Direction |
|---|---|---|---|
| 1 | 3784 | 1587 | reversed |
| 2 | 1845 | 4030 | normal |

The two servos are mounted facing each other on the same axis, so equal angles produce the same physical motion. To recalibrate: with torque off, move the arm by hand to the stowed (180°) and fully-lowered (0°) positions, read the ticks with `ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "{id: 1}"`, and update the constants.

**Shutdown:** after `rclcpp::shutdown()` it disables torque on all servos. The arm will then fall under gravity, so support it before stopping the node.

---

## ball_tracker

**Path:** `src/ball_tracker` · Third-party (Articulated Robotics). It is **not part of the mission**. It is kept for testing the camera and drive pipeline with a simple target. Parts of the source are commented-out experiments in adapting it to AprilTags.

| Executable | Node | In → Out |
|---|---|---|
| `detect_ball` | `detect_ball` | `/image_in` (Image) → `/detected_ball` (Point: x, y normalised to [−1, 1], z = normalised size), `/image_out`, `/image_tuning` |
| `detect_ball_3d` | `detect_ball_3d` | `/detected_ball` → `/detected_ball_3d` (Point in camera frame, metres), `/ball_3d_marker` |
| `follow_ball` | `follow_ball` | `/detected_ball` → `/cmd_vel` (Twist) |

Detection pipeline (`process_image.find_circles`): blur → HSV threshold (`h/s/v_min/max`) → dilate/erode → crop to the search window (`x/y_min/max`, % of the image) → `SimpleBlobDetector` → filter by size (`sz_min/max`, % of width).

`follow_ball` turns at `-angular_chase_multiplier × x`, drives at `forward_chase_speed` while the ball is smaller than `max_size_thresh`, and spins at `search_angular_speed` when the target has been lost for longer than `rcv_timeout_secs`.

To use it on this robot, run it with the robot's camera topic and the low-priority mux input:

```bash
ros2 launch ball_tracker ball_tracker.launch.py image_topic:=/camera/image_raw cmd_vel_topic:=/cmd_vel_tracker tune_detection:=true
```

`tune_detection:=true` opens an OpenCV trackbar window (needs a display) for setting the HSV values. `launch/example_launch_include.launch.py` is a template that refers to a placeholder package `my_package` and will not run as-is.

---

## odrive_ros2_control

**Path:** `src/ros_odrive/odrive_ros2_control` · Upstream plugin `odrive_ros2_control_plugin/ODriveHardwareInterface` (a `hardware_interface::SystemInterface`).

| Aspect | Behaviour |
|---|---|
| Hardware parameter | `can`: SocketCAN interface name (`can0`) |
| Joint parameter | `node_id`: ODrive CAN node ID |
| Command interfaces per joint | `position` [rad], `velocity` [rad/s], `effort` [Nm] |
| State interfaces per joint | `position`, `velocity`, `effort` (`effort` = torque estimate) |
| `on_configure` | Opens the SocketCAN interface. If that fails, the log shows `Failed to initialize SocketCAN on can0` |
| `perform_command_mode_switch` | Picks the ODrive control mode from the claimed interfaces (position > velocity > torque), clears errors, then requests `CLOSED_LOOP_CONTROL`, or `IDLE` if nothing is claimed |
| `write` | Sends `Set_Input_Pos`, `Set_Input_Vel`, or `Set_Input_Torque` with rad → turns conversion |
| `read` | Handles CAN frames as they arrive: `Get_Encoder_Estimates` → position/velocity, `Get_Torques` → effort |

`diff_drive_controller` only claims `velocity`, so the wheels always run in velocity control.

## rplidar_ros

Upstream Slamtec driver, version 2.1.4. The bring-up passes `channel_type`, `serial_port`, `serial_baudrate`, `frame_id`, `inverted`, and `angle_compensate`. Some launch files declare a `scan_mode` but **never pass it to the node**, so the sensor's default mode is used. The package's own `launch/*.py` files are useful for testing the LiDAR on its own.

## micro_ros_agent

Upstream micro-ROS Agent 3.0.6. It is started with `udp4 -p 8888 -v6`: UDP over IPv4 on port 8888, verbosity 6. Microcontroller clients must be set to the Jetson's IP address and port 8888, and use the same `ROS_DOMAIN_ID`. When it is started by hand as `micro_ros_agent udp4 --port 8888 --dev <ip>`, the `--dev` option is ignored: it only applies to the serial and CAN transports, and the UDP agent always listens on all interfaces. See [Operation §5](operation.md#5-field-bring-up-the-five-terminal-sequence). Building it pulls Micro-XRCE-DDS from the internet through a CMake superbuild. See [Setup](setup-and-build.md).
