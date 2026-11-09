# pick_place_waypoints (ROS 2 Jazzy)

A separate package for saving PICK, PLACE1, PLACE2, PLACE3 from the current ROS arm pose and executing a one-pass pick-and-place sequence.

## Install into the existing workspace

Copy the `pick_place_waypoints` directory from this archive into:
`~/Documents/arm_ws/src/`

Then build:

```bash
cd ~/Documents/arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pick_place_waypoints
source install/setup.bash
```

## Run prerequisites

Start the existing RViz/slider system and the `servo_wifi_bridge`. For Wi-Fi-only testing, launch the existing RViz system with an empty serial port so the old controller does not try to open the ESP32 USB port:

```bash
ros2 launch rviz_arm_sync rviz_sync.launch.py port:=''
```

In another terminal, run rosbridge if it is not already running:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Start the existing bridge if not already running:

```bash
ros2 run rviz_arm_sync servo_wifi_bridge
```

Then launch the waypoint GUI:

```bash
ros2 run pick_place_waypoints waypoint_gui
```

## Use

1. Move the robot in the RViz joint slider window.
2. Click SAVE PICK.
3. Set the arm to each destination pose in turn and click SAVE PLACE1, SAVE PLACE2, SAVE PLACE3.
4. Press START SEQUENCE. The sequence runs once: PICK → PLACE1 → PLACE2 → PLACE3.
5. STOP requests the sequence to stop between interpolation steps.

The saved poses are stored at `~/pick_place_waypoints.json`.

## Important behavior and safety

- Angles are servo degrees in order BASE, SHOULDER, ELBOW, WRIST1, WRIST2, GRIPPER.
- Default gripper angles are OPEN=140 and CLOSED=40. Verify these values and the direction/limits for your particular gripper before enabling servo power.
- The new package publishes `/arm/joint_command`, which the existing `servo_wifi_bridge` converts to `/servo_angles`.
- Keep external servo power disconnected during first tests.
- This sequence is waypoint joint interpolation, not collision-aware path planning. Ensure the route between poses is physically clear before running the real arm.
- Do not run a second controller that sends competing commands to the same servos.
