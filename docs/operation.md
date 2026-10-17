# Operation

All commands assume the workspace is built and sourced (`source ~/yahboom_car-ros2_ws/install/setup.bash`) and that `can0` is up (see [Setup](setup-and-build.md#5-can-bus-for-the-odrives)). Every machine taking part, including the robot, a laptop running RViz or Foxglove, and the micro-ROS bins, must use the same `ROS_DOMAIN_ID`.

## Pre-flight checklist

- [ ] Battery charged; the ODrives powered and showing no errors.
- [ ] The emergency stop or main power switch can be reached.
- [ ] `candump can0` shows traffic from node IDs 0–3.
- [ ] The DYNAMIXEL adapter is `/dev/ttyUSB0` (or your udev name) and the LiDAR is on the port the launch file expects.
- [ ] The arm is in the stowed position (180°) and nothing is near the gripper.
- [ ] The gamepad is connected. **Holding the enable button (button 6) is the manual override**: joystick commands have the top `twist_mux` priority.

## 1. Teleoperation only

The simplest way to drive the robot by hand is `rashwan.launch.py`. It starts the drive base, LiDAR, camera, joystick + `twist_mux`, Foxglove, and `slam_toolbox`; SLAM running in the background does no harm.

```bash
ros2 launch my_odrive_botwheel_explorer rashwan.launch.py gui:=false
```

Driving: hold **button 6** (enable) and use **axis 1** (left stick, vertical) for forward/back and **axis 0** (left stick, horizontal) for turning. Hold **button 7** for turbo (1.0 m/s and 1.0 rad/s instead of 0.5). Button and axis numbers depend on the controller model: check them with `ros2 topic echo /joy`.

For a quick driving test without a joystick:

```bash
ros2 topic pub --rate 10 /cmd_vel_joy geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

## 2. Building a map (SLAM)

```bash
ros2 launch my_odrive_botwheel_explorer robot_mapping_launch.launch.py
```

This starts the drive base, LiDAR (`by-path` port), camera, `slam_toolbox` in **mapping** mode, joystick + `twist_mux`, Foxglove bridge, arm and gripper drivers, and RViz (`gui:=false` turns RViz off).

1. Drive slowly through the whole area with the joystick, closing loops where you can. SLAM only adds a scan after the robot has moved 0.5 m or turned 0.5 rad.
2. Save the map in both formats:

   ```bash
   # Occupancy grid for Nav2 / AMCL
   ros2 run nav2_map_server map_saver_cli -f ~/yahboom_car-ros2_ws/src/ros_odrive/my_odrive_botwheel_explorer/map/map_4
   # Serialized pose graph for slam_toolbox localization mode
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/$USER/yahboom_car-ros2_ws/src/ros_odrive/my_odrive_botwheel_explorer/map/map_4'}"
   ```

   The "Save Map" and "Serialize Map" buttons in RViz's SlamToolbox panel do the same thing.

3. Remember where you started: the SLAM origin (0, 0) becomes the **dock** position in `robot_nav`. Measure each bin's approach pose in this map, for example with RViz's "Publish Point" tool, and update `bins_list` in `src/robot_nav/robot_nav/robot_nav.py`.

## 3. Localization + navigation

This takes three launches: the hardware bring-up, AMCL with the map server, and the Nav2 servers.

```bash
# Terminal 1: hardware, sensors, teleop, arm/gripper, robot_nav, micro-ROS agent
ros2 launch my_odrive_botwheel_explorer robot_localization_launch.launch.py

# Terminal 2: map_server + AMCL
ros2 launch my_odrive_botwheel_explorer localization_launch.py \
  map:=$HOME/yahboom_car-ros2_ws/src/ros_odrive/my_odrive_botwheel_explorer/map/map_3.yaml \
  use_sim_time:=false

# Terminal 3: planner, controller, BT navigator, behaviours, velocity smoother
ros2 launch my_odrive_botwheel_explorer navigation_launch.py use_sim_time:=false
```

- **Always pass `use_sim_time:=false`.** The params file has `use_sim_time: True`, and the launch default (`false`) rewrites it. If you supply your own params file without that rewrite, the nodes will wait forever for `/clock`.
- **Always pass `map:=`** with an absolute path. The default `map_5.yaml` is not in the repository.
- Set the initial pose: in RViz or Foxglove, use "2D Pose Estimate" at the robot's real position. If the robot starts at the dock where mapping began, that is (0, 0, 0).
- Send a test goal with RViz "2D Goal Pose", or publish to `/goal_pose`.

RViz is commented out in the localization bring-up. Run it on a laptop instead:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix my_odrive_botwheel_explorer)/share/my_odrive_botwheel_explorer/rviz/mybot.rviz
```

Or connect **Foxglove Studio** to `ws://<jetson-ip>:8765`.

## 4. Autonomous bin-collection mission

With section 3 running, start the two mission nodes that are not in any launch file:

```bash
# Terminal 4: AprilTag final approach
ros2 run tag_follower_pkg tag_follower

# Terminal 5: arm/gripper sequence
ros2 run manipulator_control manipulator_control
```

The mission now runs on its own whenever a bin publishes on `/bin_full_id` (see [Mission workflow](mission-workflow.md)). To trigger it by hand:

```bash
ros2 topic pub --once /bin_full_id std_msgs/msg/Int32 "{data: 53}"   # go to bin 2
```

To test the manipulator alone, with the robot parked in front of a bin:

```bash
ros2 topic pub --once /manipulator_start_flag std_msgs/msg/Int32 "{data: 1}"
```

## 5. Other launch files

| Launch file | Use |
|---|---|
| `rashwan.launch.py` | The robot plus SLAM mapping, joystick, and Foxglove, with the LiDAR on `/dev/ttyUSB0` and no arm. An older form of `robot_mapping_launch` |
| `new.launch.py` | Drive base with the placeholder `diffbot` URDF, plus the LiDAR and RViz. For bench testing the LiDAR and wheels |
| `my_botwheel_explorer.launch.py` | Drive base with the placeholder URDF. Its RViz config comes from `ros2_control_demo_description` |
| `joy_stick.launch.py` | Joystick → `/cmd_vel_joy` → stamper → `/botwheel_explorer/cmd_vel`, bypassing `twist_mux`. Run it **instead of** the joystick nodes in the other launches, not alongside them |
| `ball_tracker/ball_tracker.launch.py` | Colour-ball following demo (see [packages](packages.md#ball_tracker)) |

## 6. Stopping safely

1. Release the joystick enable button and stop the mission nodes (Ctrl-C on `tag_follower`, `manipulator_control`).
2. Make sure the arm is stowed or supported. **Stopping `read_write_node` disables DYNAMIXEL torque**, and the arm will drop.
3. Ctrl-C the bring-up. The diff-drive controller stops sending commands, the hardware interface moves the ODrives to `IDLE` on deactivation, and the ODrive watchdog (if enabled) disarms them.
4. Power down the motors before handling the robot.

## Useful introspection

```bash
ros2 control list_controllers                     # botwheel_explorer + joint_state_broadcaster active?
ros2 control list_hardware_interfaces             # velocity interfaces claimed?
ros2 topic echo /joint_states                     # wheel feedback; NaN = that ODrive is not talking
ros2 topic hz /scan /camera/image_raw
ros2 topic echo /cmd_vel_mux                      # what actually reaches the base
ros2 run tf2_tools view_frames                    # TF tree PDF
ros2 run rqt_graph rqt_graph                      # live node graph
```
