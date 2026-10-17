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

## 5. Field bring-up: the five-terminal sequence

On the robot, the system is often started by hand, one command per terminal, in this order:

```bash
ros2 run dynamixel_sdk_examples read_write_node                                  # T1: arm driver
ros2 run manipulator_control manipulator_control                                 # T2: pick-and-tip sequencer
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --dev 192.168.8.138    # T3: bridge to the bin controllers
ros2 run tag_follower_pkg tag_follower                                           # T4: AprilTag final approach
ros2 launch my_odrive_botwheel_explorer robot_localization_launch.launch.py      # T5: robot hardware bring-up
```

Each command gets its own terminal so its log can be watched and it can be restarted on its own. Source the workspace in every terminal (`source ~/yahboom_car-ros2_ws/install/setup.bash`) and use the same `ROS_DOMAIN_ID` in all of them.

### What each command does

**T1: `read_write_node` (arm hardware).**
- Opens the DYNAMIXEL USB-TTL adapter at `/dev/ttyUSB0`, 57600 baud, Protocol 2.0.
- Puts both arm servos (IDs 1 and 2) in position-control mode, sets profile velocity and acceleration to 500 and 700, and **turns torque on**, so the arm holds its current pose from this point.
- Waits for angle commands on `/set_position` and serves `/get_position`.
- If it prints `Failed to open the port!`, it exits straight away. Check that the adapter is plugged in, that it really is `ttyUSB0`, and that you are in the `dialout` group.
- It is started first so that it claims `/dev/ttyUSB0` before anything else and the arm is under control before any node can send it commands. See [packages → DynamixelSDK](packages.md#dynamixelsdk-modified).

**T2: `manipulator_control` (pick sequencer).**
- Subscribes to `/manipulator_start_flag` and sits idle until it receives `1`.
- It then runs the timed sequence (about 70 s): open, lower, grip, tip, lower, release, stow, back off.
- To do that it publishes on `/set_position` (to T1), `/servo_angles` (to `servo_controller`, started by T5), `/cmd_vel` (to `twist_mux`, started by T5), and finally `/bin_full_id = 51`.
- It must be running **before** the tag follower can trigger it. A start flag published while it is not running is lost. See [Mission workflow → pick, tip, and return](mission-workflow.md#4-pick-tip-and-return).

**T3: `micro_ros_agent udp4 --port 8888 --dev 192.168.8.138` (network bridge).**
- Starts a micro-ROS agent with the UDP/IPv4 transport on port 8888.
- The bin and collection-unit microcontrollers connect to it over Wi-Fi. Their topics (for example `/bin_full_id`) then show up in the ROS 2 graph, and topics they subscribe to (for example `/cu_cmd`) reach them.
- `--port 8888` (same as `-p 8888`) is the UDP port the agent listens on. Every client's firmware must be built with this agent port.
- **`--dev 192.168.8.138` has no effect with `udp4`.** In the Micro-XRCE-DDS agent, `--dev` is only an option for the `serial`, `multiserial`, `pseudoterminal`, and `canfd` transports. The UDP transport accepts only `--port`, and the parser ignores options it does not look for. The agent always binds to **all** network interfaces (`INADDR_ANY`).
- The address is still worth recording. `192.168.8.138` is presumably the Jetson's IP address on the robot's network, and it is the **agent IP each micro-ROS client must be set to**. Check it with `hostname -I`. If DHCP gives the Jetson a different address, the clients will not find the agent. Give the Jetson a static IP or a DHCP reservation.
- Add `-v6` for verbose output that shows each client session and topic as it is created. This is the quickest way to confirm the bins are connected.

**T4: `tag_follower` (final approach).**
- Subscribes to `/camera/image_raw` and publishes on `/cmd_vel_tracker` and `/manipulator_start_flag`.
- Until T5 starts the camera, it receives no images and does nothing.
- Once images arrive, it runs on every frame:
  - **No tag visible:** it publishes zero velocity.
  - **A tag36h11 tag visible:** it centres on the tag and creeps forward at 0.1 m/s. When the tag is 187 px wide and centred, it publishes `/manipulator_start_flag = 1` (handled by T2) and blocks for 120 s.
- This node is started by hand because its entry in the launch file is commented out. See [packages → tag_follower_pkg](packages.md#tag_follower_pkg).

**T5: `robot_localization_launch.launch.py` (robot bring-up).** This starts everything else on the robot:

| Started node | Hardware | Role |
|---|---|---|
| `ros2_control_node` + `joint_state_broadcaster` + `botwheel_explorer` (DiffDriveController) | 4 ODrives on `can0` | wheel drive and odometry (`odom→base_link`) |
| `robot_state_publisher` | — | URDF and static TF (`rashwan_bot.urdf.xacro`) |
| `camera/v4l2_camera_node` | USB camera | `/camera/image_raw` (wakes up T4) |
| `rplidar_node` | RPLidar on its `by-path` USB port | `/scan` |
| `joy_node` + `teleop_node` | gamepad | `/cmd_vel_joy` (manual override, priority 100) |
| `twist_mux` + `twist_stamper` | — | merge joystick, Nav2/manipulator (`/cmd_vel`), and tracker (`/cmd_vel_tracker`) → `/botwheel_explorer/cmd_vel` |
| `foxglove_bridge` | network | remote view on `ws://<jetson-ip>:8765` |
| `servo_controller` | gripper servos on header pins 33/15 | executes `/servo_angles` from T2 |
| `robot_nav` | — | `/bin_full_id` → `/goal_pose` |
| `read_write_node` | DYNAMIXEL bus | **duplicate of T1** (see below) |
| `micro_ros_agent udp4 -p 8888 -v6` | network | **duplicate of T3** (see below) |

It is started last so that the drive base, camera, and velocity mux come up only once the arm driver, the sequencer, the micro-ROS bridge, and the tag follower are already waiting. The robot cannot move until this step, and the first camera frame finds the whole pipeline ready.

### ⚠️ Duplicates with the launch file

`robot_localization_launch.launch.py` already contains a `read_write_node` and a `micro_ros_agent`. With T1 and T3 running, T5 starts **a second copy of each**:

- **Second `micro_ros_agent`:** it cannot bind UDP port 8888, which T3 already holds, so its transport fails to start. This only adds error noise; the agent in T3 keeps working.
- **Second `read_write_node`:** this one is **not harmless**.
  - Linux allows two processes to open `/dev/ttyUSB0` at once, and the DynamixelSDK takes no exclusive lock. Both nodes subscribe to `/set_position` and write every command to the same bus, so packets can collide. You will see `There is no status packet!` / `[TxRxResult]` errors and arm moves can be missed.
  - ROS also warns about two nodes named `/read_write_node`.
  - When either copy shuts down, it sends *torque off* to **all** servos (broadcast ID), so the arm can drop even though the other copy is still running.

Pick one way of starting these two nodes:

- **Keep the five-terminal sequence** (recommended while debugging, since the arm and micro-ROS logs stay separate). Comment out `dynamixel_node` and `micro_ros_agent_Launch` in the `nodes = [...]` list of `robot_localization_launch.launch.py`, then rebuild.
- **Or drop T1 and T3** and let the launch file start both (in that case the agent runs with `-v6`).

### What this sequence does *not* start

The sequence gives you the robot's hardware, teleop, the tag approach, and the arm. It does **not** start localization or Nav2:

- With no AMCL there is no `map→odom` transform, and with no Nav2 nothing consumes `/goal_pose`. A bin publishing `/bin_full_id` makes `robot_nav` publish a goal, but **the robot does not drive to it**.
- In this mode the robot is driven to a bin with the joystick. Once you release the enable button and the tag is in view, `tag_follower` (priority 5) takes over, centres, approaches, and triggers the pick.

For fully autonomous runs, also start the two Nav2 launches from [section 3](#3-localization--navigation) after T5:

```bash
ros2 launch my_odrive_botwheel_explorer localization_launch.py map:=<abs path>/map_3.yaml use_sim_time:=false   # T6
ros2 launch my_odrive_botwheel_explorer navigation_launch.py use_sim_time:=false                                # T7
```

Then set the initial pose ("2D Pose Estimate") before the first bin call.

### Safety notes for this sequence

- Once T5 is up, **the robot can move with nobody touching the joystick**: `tag_follower` drives toward any tag36h11 tag the camera sees. Keep the gamepad in hand. Holding the enable button (button 6) overrides every other source.
- From T1 onward the arm is under torque and holds its pose. Stopping T1 (or the duplicate in T5) releases torque, so support the arm first.
- Shut down in reverse order: T7/T6 → T5 → T4 → T3 → T2 → T1 (arm supported).

### Quick checks after start-up

```bash
ros2 node list | sort | uniq -d                    # should print nothing; any line is a duplicate node
ros2 topic info /set_position                      # Subscription count: 1 (2 means the duplicate is running)
ros2 topic echo /bin_full_id                       # shows messages when a bin reports full
ros2 topic hz /camera/image_raw                    # about 6 Hz: tag_follower is getting images
ros2 control list_controllers                      # botwheel_explorer and joint_state_broadcaster active
```

## 6. Other launch files

| Launch file | Use |
|---|---|
| `rashwan.launch.py` | The robot plus SLAM mapping, joystick, and Foxglove, with the LiDAR on `/dev/ttyUSB0` and no arm. An older form of `robot_mapping_launch` |
| `new.launch.py` | Drive base with the placeholder `diffbot` URDF, plus the LiDAR and RViz. For bench testing the LiDAR and wheels |
| `my_botwheel_explorer.launch.py` | Drive base with the placeholder URDF. Its RViz config comes from `ros2_control_demo_description` |
| `joy_stick.launch.py` | Joystick → `/cmd_vel_joy` → stamper → `/botwheel_explorer/cmd_vel`, bypassing `twist_mux`. Run it **instead of** the joystick nodes in the other launches, not alongside them |
| `ball_tracker/ball_tracker.launch.py` | Colour-ball following demo (see [packages](packages.md#ball_tracker)) |

## 7. Stopping safely

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
