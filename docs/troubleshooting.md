# Troubleshooting & Known Issues

## Troubleshooting

### Drive base

| Symptom | Likely cause | Check / fix |
|---|---|---|
| `Failed to initialize SocketCAN on can0`, then repeated `command interface not in available list` warnings | `can0` missing or down | `ip link show can0`; bring it up (see [Setup §5](setup-and-build.md#5-can-bus-for-the-odrives)) |
| `/joint_states` shows `.nan` for a wheel | That ODrive is not sending feedback: wrong `node_id`, CAN wiring, bit-rate mismatch, or encoder/torque message rates set to 0 | `candump can0` and look for frames with `(can_id >> 5) == node_id` |
| Controllers load but the wheels do not move | Axis did not enter closed loop (an ODrive error, e.g. undervoltage, or not calibrated) | Check errors in the ODrive GUI / `odrivetool`. The plugin does **not** report ODrive errors to ROS |
| The robot spins when told to go straight | Wheel direction sign wrong | Flip `left_wheel_radius_multiplier` or `right_wheel_radius_multiplier` |
| The robot moves for 0.5 s and stops | Commands not arriving continuously | `twist_mux` and the controller both time out after 0.5 s; publish at ≥ 10 Hz |
| Jerky motion, odometry lags | `update_rate: 10` Hz | Raise `controller_manager.update_rate` |
| Odometry drifts in rotation | Skid-steer slip + `open_loop: true` | Tune `wheel_separation_multiplier`; consider `open_loop: false` and/or fusing an IMU |

### Sensors

| Symptom | Likely cause | Check / fix |
|---|---|---|
| `rplidar_node` cannot open the port / times out | Wrong port, or the DYNAMIXEL adapter took `ttyUSB0` | `ls -l /dev/serial/by-path/ /dev/ttyUSB*`; use udev symlinks; pass `serial_port:=` |
| `read_write_node`: `Failed to open the port!` | `/dev/ttyUSB0` missing, busy (LiDAR on it), or permissions | `sudo usermod -aG dialout $USER`, re-login; udev |
| DYNAMIXEL `[TxRxResult] There is no status packet!` | Wrong baud rate or ID, servos unpowered | The servos need their own supply (typically 11–12 V for X-series); confirm the ID and baud rate with DYNAMIXEL Wizard |
| No `/camera/image_raw` | Camera not `/dev/video0`, or the format is not supported | `v4l2-ctl --list-devices`, `v4l2-ctl --list-formats-ext` |
| `tag_follower` logs `No tags detected` all the time | Wrong tag family (must be **tag36h11**), tag too small or far away, poor lighting, low frame rate | View `/camera/image_raw` in Foxglove; print a bigger tag |
| `tag_follower` errors on `imgmsg_to_cv2` | Camera publishes a format cv_bridge cannot convert to mono8 | Set the camera's `output_encoding` to `rgb8` or `mono8` |

### Gripper

| Symptom | Likely cause | Check / fix |
|---|---|---|
| `RuntimeError: Not running on a RPi!` / import error for `RPi.GPIO` | `Jetson.GPIO` not installed | `pip3 install Jetson.GPIO` |
| Permission error on GPIO | User not in the `gpio` group | `sudo usermod -aG gpio $USER` |
| No movement on pin 15/33 | PWM not enabled on the header | `jetson-io.py` → enable PWM on those pins, reboot |
| Servos twitch or reset the Jetson | Powered from the Jetson's 5 V | Use a separate supply with a common ground |

### Navigation / localization

| Symptom | Likely cause | Check / fix |
|---|---|---|
| Nav2 nodes stuck waiting, `/clock` warnings | `use_sim_time` is true | Launch with `use_sim_time:=false` |
| `map_server` fails to load | Default `map_5.yaml` does not exist | Pass `map:=/abs/path/map_3.yaml` |
| `Timed out waiting for transform map → base_link` | No initial pose set, or AMCL not receiving `/scan` | Set "2D Pose Estimate"; `ros2 topic hz /scan` |
| Robot reaches the goal but faces the wrong way for the tag | Wrong `bins_list` entry or yaw convention | Re-measure the goal in the current map; see [robot_nav](packages.md#robot_nav) |
| Robot cuts corners and clips obstacles | `robot_radius: 0.22` is smaller than the chassis | Use a polygon `footprint` (see [Configuration](configuration.md#confignav2_paramsyaml-nav2)) |
| `twist_mux` passes nothing from Nav2 | Nav2 publishing to the wrong topic | `ros2 topic info /cmd_vel -v`; `navigation_launch.py` remaps the smoother output to `cmd_vel` |

### micro-ROS

| Symptom | Likely cause | Check / fix |
|---|---|---|
| Bins connect but `/bin_full_id` never appears | `ROS_DOMAIN_ID` mismatch, or the client is set to the wrong agent IP/port | Agent log at `-v6` shows sessions and topics; client must target `<jetson-ip>:8888` |
| `micro_ros_agent` fails to build | No internet during the superbuild, or `micro_ros_msgs` missing | `sudo apt install ros-humble-micro-ros-msgs`; build once with network access |

## Known issues

These are defects or gaps in the current code that affect how the robot behaves. Each is listed with its location and a suggested fix.

### Mission logic

1. **The robot never returns to the dock.** `robot_nav.py`, branch `idx == 0`: the `goal_pose_publisher.publish` call is commented out, so the "51" message sent by `manipulator_control` does nothing. *Fix:* uncomment it.
2. **`manipulator_control` keeps publishing zero velocity on `/cmd_vel` at 10 Hz forever** once the first pick has run (`publish_cmd_vel` timer, `flag` is never reset). Nav2 publishes on the same `/cmd_vel` input of `twist_mux`, so the two sources fight on later missions and the robot stalls or stutters. *Fix:* reset `flag = 0` at the end of the sequence, or publish the hold commands on a separate higher-priority mux input.
3. **The back-off is about 0.5 s, not 5 s.** A single `linear.x = −0.1` message is followed by `sleep(5)`, but `twist_mux` and the diff-drive controller time out after 0.5 s. *Fix:* publish at ≥ 10 Hz for the intended duration.
4. **Blocking callbacks.** `manipulator_control` (about 70 s) and `tag_follower` (120 s after the trigger) run `time.sleep()` inside callbacks on single-threaded executors. While they block they handle no messages, and images queue up. *Fix:* use a state machine driven by timers, or ROS 2 actions.
5. **No completion feedback.** Nav2's goal result, the arm's position, and the gripper's state are never checked. Every stage is timed open loop. *Fix:* use the `NavigateToPose` action client (already imported in `tag_follower.py`) and the `/get_position` service.
6. **`tag_follower` is not started by any launch file.** The node entry is commented out in `robot_localization_launch.launch.py`.
7. **Index out of range.** `/bin_full_id` values above 61 crash `robot_nav` with an `IndexError`.
8. **The collection-unit commands (`/cu_cmd`) are commented out**, so the hopper is never opened or closed.
9. `robot_nav.euler_to_quaternion()` is called with the wrong argument order and its result is ignored. It is harmless, but misleading.

### Hardware interface and drivers

10. **Duplicate nodes in the five-terminal field sequence.** `robot_localization_launch.launch.py` also starts `read_write_node` and `micro_ros_agent`. If they were already started by hand, the second agent fails to bind UDP 8888 (harmless). The second `read_write_node` shares `/dev/ttyUSB0` with the first, so commands are written twice, packets collide, and either copy cuts arm torque when it exits. *Fix:* start each node only once (see [Operation §5](operation.md#-duplicates-with-the-launch-file)).
11. **Shared `/dev/ttyUSB0`.** `read_write_node` hard-codes `/dev/ttyUSB0`, which `rashwan.launch.py` and `new.launch.py` also use for the LiDAR. *Fix:* udev symlinks, and make `DEVICE_NAME` a ROS parameter.
12. **Disabling torque on exit drops the arm.** `read_write_node` disables torque at shutdown.
13. **Wrong log messages.** `read_write_node` logs "Profile Velocity to 5000 / Acceleration to 7000", but writes 500 / 700.
14. **`servo_controller` re-creates `GPIO.PWM` objects on every message** and never calls `stop()`/`cleanup()`. Depending on the GPIO library version, repeated commands can raise "PWM object already exists" or leak channels. *Fix:* create one PWM object per pin in `__init__` and reuse it.
15. **`use_mock_hardware` launch argument has no effect.** It is not forwarded to xacro.
16. **The ODrive plugin does not report errors.** If an axis disarms (for example on undervoltage), ROS is not told. This is an upstream limitation.

### Configuration

17. `nav2_params.yaml`: `robot_radius: 0.22` is smaller than the 0.658 × 0.488 m chassis.
18. `nav2_params.yaml` `bt_navigator.odom_topic: /odom`, but the controller publishes `/botwheel_explorer/odom`. Only TF is used for core navigation, but BT nodes that read odometry will see nothing. *Fix:* set it to `/botwheel_explorer/odom`.
19. `controller_manager.update_rate: 10` Hz limits drive responsiveness and odometry rate.
20. `localization_launch.py` default map `map_5.yaml` and the slam localization `map_file_name: map_4` are not in the repository.
21. `scan_mode` is declared in several launch files but never passed to `rplidar_node`.
22. `rashwan_gazebo.xacro` refers to `left_wheel_joint` / `right_wheel_joint`, which do not exist in the Rashwan URDF, so the Gazebo simulation will not drive.
23. `robot_state_publisher` is given a `cmd_vel` remap, which does nothing (it was copied from the upstream example).
24. Package metadata (`package.xml` descriptions, licences, and the dependencies of the Python packages) is still at the `TODO` defaults, so `rosdep` cannot install their runtime dependencies (`apriltag`, `cv_bridge`, `Jetson.GPIO`, `nav2_msgs`, …).

### Repository

25. The repository name says "yahboom_car", but the code is for the custom ODrive-based "Rashwan" robot.
26. Large third-party trees (`ros-controls/`, `rplidar_ros/sdk`, `DynamixelSDK`) are vendored as plain source, not as submodules or a `.repos` file. That makes upstream updates and diffs against upstream hard. Consider a `vcs import` file plus a small set of patches for the modified DynamixelSDK files.
