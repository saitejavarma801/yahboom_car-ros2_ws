# Setup & Build

These steps take a fresh robot computer to a built workspace. They assume an **NVIDIA Jetson running JetPack 6 (Ubuntu 22.04)**. Any Ubuntu 22.04 machine works for development, but without the GPIO, CAN, and serial hardware.

## 1. Install ROS 2 Humble

Follow the official guide (<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>) and install `ros-humble-ros-base` (or `ros-humble-desktop` if you want RViz on the robot). Then:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pip git
sudo rosdep init && rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 2. Install system dependencies

`package.xml` files in this workspace do not declare everything the launch files use, so `rosdep` alone is not enough. Install these explicitly:

```bash
sudo apt install \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-twist-mux ros-humble-twist-stamper \
  ros-humble-joy ros-humble-teleop-twist-joy \
  ros-humble-v4l2-camera ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-foxglove-bridge \
  ros-humble-micro-ros-msgs \
  ros-humble-rviz2 \
  can-utils v4l-utils

pip3 install apriltag numpy opencv-python   # apriltag builds a C extension; it needs cmake + build-essential
# Jetson.GPIO ships with JetPack. On other systems: pip3 install Jetson.GPIO
```

Then let rosdep install whatever else is declared:

```bash
cd ~/yahboom_car-ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 3. Get the workspace

```bash
git clone https://github.com/saitejavarma801/yahboom_car-ros2_ws.git ~/yahboom_car-ros2_ws
```

## 4. Serial device names (udev)

The DYNAMIXEL node opens **`/dev/ttyUSB0`** (hard-coded). The LiDAR launch files use either `/dev/ttyUSB0` or a `by-path` name. To make this deterministic, give each adapter a fixed name:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head
```

Create `/etc/udev/rules.d/99-robot-serial.rules`. The IDs below are typical: the U2D2 is an FTDI part and the RPLidar is a CP210x part. Use the values you read from your own hardware:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="dynamixel", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar",   MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER
```

After that, change `DEVICE_NAME` in `read_write_node.cpp` to `/dev/dynamixel`, and launch with `serial_port:=/dev/rplidar`. Until then, plug the DYNAMIXEL adapter in **first** so that it becomes `ttyUSB0`.

For the U2D2, lowering the FTDI latency timer from 16 ms to 1 ms makes the arm noticeably more responsive:

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## 5. CAN bus for the ODrives

### 5.1 ODrive configuration (once, with the ODrive GUI or `odrivetool`)

For each of the four ODrives:

| Setting | Value |
|---|---|
| `can.config.protocol` | CAN-simple |
| `can.config.baud_rate` | the same as `can0` (250000 in ODrive's guides and below) |
| `axis0.config.can.node_id` | 0 = front-left, 1 = front-right, 2 = rear-left, 3 = rear-right |
| Encoder-estimate and torque message rates | non-zero (for example 10–20 ms), so feedback reaches `/joint_states` |
| Motor/encoder calibration | done and saved (`odrive_ros2_control` assumes calibrated axes) |
| Watchdog (recommended) | enabled, so the motors stop if the Jetson or the CAN link dies |

Check that the bus has exactly one 120 Ω termination at each end.

### 5.2 Bring up `can0` on the Jetson

Jetson's built-in CAN controller (`mttcan`, with an external transceiver):

```bash
sudo modprobe can && sudo modprobe can_raw && sudo modprobe mttcan
sudo ip link set can0 up type can bitrate 250000
ip -details link show can0        # should be UP, state ERROR-ACTIVE
candump can0                      # with the ODrives powered, periodic frames from node IDs 0-3
```

With a USB-CAN adapter (gs_usb / candleLight / slcan), the `ip link` command is the same once the adapter shows up as `can0`.

To make it persistent, create `/etc/systemd/network/80-can.network` and enable `systemd-networkd`:

```ini
[Match]
Name=can0

[CAN]
BitRate=250000
```

## 6. Gripper PWM pins

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py      # Configure 40-pin header → enable the PWM functions on pins 15 and 33 → save, reboot
sudo groupadd -f -r gpio && sudo usermod -aG gpio $USER
```

Wire each servo's signal lead to header pin 33 (servo 1) or pin 15 (servo 2). Power the servos from a separate 5–6 V supply, and connect its ground to a Jetson GND pin.

## 7. Camera

```bash
v4l2-ctl --list-devices          # the camera should be /dev/video0 (the v4l2_camera default)
```

If it enumerates as a different device, add `'video_device': '/dev/videoN'` to the camera node's parameters in the launch file.

## 8. Build

Building the whole `src/` tree compiles the vendored ros2_control stack and its Gazebo demos, which is slow on a Jetson and needs Gazebo. Skip the packages the robot does not use:

```bash
cd ~/yahboom_car-ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-skip-regex 'ign_ros2_control.*|gz_ros2_control_tests|ros2_control_demo.*|example_[0-9]+|rqt_.*|odrive_node|odrive_botwheel_explorer' \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
echo "source ~/yahboom_car-ros2_ws/install/setup.bash" >> ~/.bashrc
```

Or build only what the robot needs and their in-workspace dependencies:

```bash
colcon build --symlink-install --packages-up-to \
  my_odrive_botwheel_explorer odrive_ros2_control rplidar_ros micro_ros_agent \
  dynamixel_sdk_examples servo_control manipulator_control robot_nav tag_follower_pkg
```

Notes:

- **Vendored ros2_control vs. apt.** If `ros-humble-ros2-control` / `ros-humble-ros2-controllers` are also installed from apt, the workspace copies **override** them once you source `install/setup.bash`. Pick one source and use it everywhere. If you use the apt packages, add `COLCON_IGNORE` files to `src/ros-controls/`.
- **micro_ros_agent** downloads and builds Micro-XRCE-DDS-Agent during its CMake step, so the first build needs internet access.
- **`ament_python` packages and `--symlink-install`.** The Python sources are symlinked, but launch and config files are only refreshed when you rebuild the package (`ball_tracker` and the other Python packages install their data with `data_files`). `my_odrive_botwheel_explorer` is CMake-installed, so with `--symlink-install` its config and launch files are symlinked and edits take effect immediately.
- The new console scripts are only on your `PATH` after you `source install/setup.bash`.

## 9. Verify each subsystem

Bring each part up on its own before running the full system:

| Subsystem | Command | Expected |
|---|---|---|
| CAN / ODrives | `candump can0` | periodic frames from IDs 0–3 |
| Drive base | `ros2 launch my_odrive_botwheel_explorer rashwan.launch.py gui:=false` then `ros2 control list_hardware_interfaces` | 4 joints, `velocity` claimed |
| LiDAR | `ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/rplidar` then `ros2 topic hz /scan` | about 7–10 Hz |
| Camera | `ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/camera` then `ros2 topic hz /camera/image_raw` | images at the configured rate |
| Arm | `ros2 run dynamixel_sdk_examples read_write_node`, then `ros2 topic pub --once /set_position dynamixel_sdk_custom_interfaces/msg/SetMultiplePositions "{ids: [1,2], position: 180.0}"` | arm moves to stowed |
| Gripper | `ros2 run servo_control servo_control`, then `ros2 topic pub --once /servo_angles std_msgs/msg/Float64MultiArray "{data: [10.0, 190.0]}"` | gripper opens |
| micro-ROS | `ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 -v6` | client sessions logged when the bins connect |

⚠️ Before the first powered drive test, lift the robot so its wheels are off the ground. Check that a positive `linear.x` turns all four wheels forward. If one side turns backwards, flip the sign of that side's `*_wheel_radius_multiplier`.
