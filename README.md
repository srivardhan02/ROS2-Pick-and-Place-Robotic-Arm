# ROS 2 Pick-and-Place Robotic Arm Using Waypoints and RViz

A ROS 2 project for controlling and visualizing a 6-DOF servo-based robotic arm. The arm can be visualized in RViz, moved using joint controls, and operated through a waypoint-based pick-and-place sequence.

The repository is organized as a ROS 2 workspace source tree: each top-level package is an independent ROS 2 package.

## Project features

- **3D robot model:** URDF description and STL meshes for the robotic arm and gripper.
- **RViz visualization:** View the robot model and adjust its joints using the joint-state GUI.
- **Waypoint teaching:** Save the current arm pose as `PICK`, `PLACE1`, `PLACE2`, and `PLACE3`.
- **Pick-and-place sequence:** Execute the saved poses in order using joint-space interpolation.
- **Servo control:** ROS 2 nodes and ESP32 firmware for sending servo commands to the physical arm.
- **Optional camera / QR workflow:** Receive ESP32-CAM video over UDP and optionally trigger the gripper from QR detections.

> **Important:** The waypoint sequence is joint-space interpolation, not collision-aware motion planning. Check the complete motion path for collisions and verify joint limits before operating the physical arm.

## Repository structure

```text
.
├── esp32_cam_udp/             # ESP32-CAM UDP image receiver and optional QR/gripper logic
├── pick_place_waypoints/      # Waypoint GUI and pick-and-place sequence
├── qr_decoder/                # QR decoding / tracking ROS 2 package
├── robot_arm_controller/      # Arm controller, GUI, parameters and ESP32 firmware
├── robotic_arm_description/   # URDF, STL meshes, RViz config and display launch
└── rviz_arm_sync/              # RViz joint-state to arm-command synchronization
```

## Software requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- `colcon`
- RViz 2
- Python 3
- Tkinter
- For physical servo control: the compatible ESP32 arm firmware and a correctly wired servo power supply
- Optional camera features: ESP32-CAM, OpenCV and the required ROS image message packages

Install common ROS tools and dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-tk \
  ros-jazzy-rviz2 \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rosbridge-server \
  ros-jazzy-rqt-image-view \
  python3-serial \
  python3-opencv
```

If `rosdep` has not been initialized on this computer:

```bash
sudo rosdep init
rosdep update
```

## 1. Create the ROS 2 workspace

Clone this repository into the `src` directory of a workspace. For example:

```bash
mkdir -p ~/arm_ws/src
cd ~/arm_ws/src
git clone <YOUR_GITHUB_REPOSITORY_URL> .
```

The repository root itself contains the ROS 2 packages, so cloning into `src` places each package at the expected level.

Alternatively, download the repository ZIP, extract it, and copy the six package folders into `~/arm_ws/src/`.

## 2. Install dependencies and build

```bash
cd ~/arm_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build
source install/setup.bash
```

If you only want the core arm and waypoint functionality, you can build the required packages:

```bash
colcon build --packages-up-to pick_place_waypoints
source install/setup.bash
```

If `rosdep` reports a dependency that is unavailable, install the corresponding ROS 2 package or Python dependency manually and rebuild.

## 3. Visualize the robotic arm in RViz

Open a terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arm_ws/install/setup.bash
ros2 launch robotic_arm_description display.launch.py
```

RViz and the joint-state publisher GUI should open. Move the sliders to inspect the arm's joint motion.

## 4. Teach and execute pick-and-place waypoints

The waypoint package saves four poses:

1. `PICK` – the pose where the gripper picks up the object.
2. `PLACE1` – first placement pose.
3. `PLACE2` – second placement pose.
4. `PLACE3` – third placement pose.

The waypoint GUI is launched with:

```bash
ros2 run pick_place_waypoints waypoint_gui
```

Use the GUI to move the arm to each required pose and save it. Then press **START SEQUENCE** to run:

```text
PICK → PLACE1 → PLACE2 → PLACE3
```

The saved waypoint file is stored at:

```text
~/pick_place_waypoints.json
```

The waypoint package publishes arm joint commands on `/arm/joint_command`. The `rviz_arm_sync` package includes a bridge node that converts these commands to `/servo_angles` for the ESP32-side interface.

### Start the command bridge

In a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arm_ws/install/setup.bash
ros2 run rviz_arm_sync servo_wifi_bridge
```

For the waypoint GUI's documented workflow, also start rosbridge if the GUI is using its WebSocket connection:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

The RViz synchronization launch can be used to connect RViz joint-state changes to the arm command path:

```bash
ros2 launch rviz_arm_sync rviz_sync.launch.py port:='' gui:=false
```

Review the launch configuration and hardware connection method in your setup before using this with a powered arm. Do not start multiple controllers that command the same servos.

## 5. Optional ESP32-CAM and QR features

The `esp32_cam_udp` package receives JPEG frames sent by the ESP32-CAM over UDP and publishes a compressed ROS image topic. Its optional QR/gripper node can react to QR detections.

The camera firmware is located at:

```text
esp32_cam_udp/firmware/esp32_cam_udp/esp32_cam_udp.ino
```

Configure the Wi-Fi SSID, password and receiver laptop IP in the firmware before flashing. Build the package, then launch it:

```bash
ros2 launch esp32_cam_udp udp_camera.launch.py show:=true detect_qr:=true
```

View the image topic:

```bash
ros2 run rqt_image_view rqt_image_view
```

See `esp32_cam_udp/README.md` for firmware setup, UDP testing, launch arguments and QR-to-gripper configuration.

## ROS 2 interfaces

| Interface | Purpose |
|---|---|
| `/joint_states` | Current joint states from the visualization controls |
| `/arm/joint_command` | Joint commands sent by the waypoint / synchronization system |
| `/servo_angles` | Servo-angle array produced by the ESP32 bridge |
| `/camera/image/compressed` | Compressed image stream from ESP32-CAM |
| `/qr/data` | Optional decoded QR text |

Topic availability depends on which nodes are running.

## Safety and calibration

- Test the URDF and GUI without connecting servo power first.
- Keep the arm mechanically supported during initial tests.
- Confirm the servo order, center angles, direction, and mechanical limits in `rviz_arm_sync/config/sync_map.yaml`.
- Confirm the gripper's open and closed angles in `pick_place_waypoints` and the relevant ESP32 firmware.
- Use a current-limited, appropriately rated servo supply. Do not power multiple servos from the computer's USB port.
- Keep hands clear of the arm while it is moving.
- Test each waypoint at low speed and with the gripper unloaded before attempting to pick an object.
- Add collision checking / MoveIt 2 planning before relying on the arm for complex or unattended motions.

## Packages

| Package | Description |
|---|---|
| `robotic_arm_description` | URDF model, STL meshes, RViz configuration and display launch |
| `robot_arm_controller` | Arm control nodes, GUI, configuration and ESP32 firmware |
| `rviz_arm_sync` | Maps RViz joint states to servo-angle commands |
| `pick_place_waypoints` | Saves poses and executes the waypoint sequence |
| `esp32_cam_udp` | ESP32-CAM UDP receiver and optional QR/gripper workflow |
| `qr_decoder` | QR decoding and tracking nodes |

## License

Package manifests currently declare MIT for most packages, but some package metadata still contains placeholder maintainer/license information. Confirm authorship and licensing for all included source files and CAD meshes before publishing the repository publicly.
