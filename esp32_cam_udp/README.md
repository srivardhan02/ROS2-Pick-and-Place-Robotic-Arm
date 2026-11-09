# esp32_cam_udp

ROS 2 (ament_python) package that receives UDP JPEG video from an ESP32-CAM.

Publishes
- `/camera/image/compressed` (sensor_msgs/CompressedImage)
- `/qr/data` (std_msgs/String) when `detect_qr:=true`

## 1. Flash the ESP32-CAM
Open `firmware/esp32_cam_udp/esp32_cam_udp.ino`, set WIFI_SSID, WIFI_PASS and HOST_IP
(your laptop IP from `hostname -I`), select board "AI Thinker ESP32-CAM", upload.

## 2. Build
    cp -r esp32_cam_udp ~/Documents/arm_ws/src/
    cd ~/Documents/arm_ws
    rosdep install --from-paths src --ignore-src -y   # or: sudo apt install python3-opencv
    colcon build --packages-select esp32_cam_udp
    source install/setup.bash

## 3. Run
    sudo ufw allow 5005/udp          # if ufw is enabled
    ros2 launch esp32_cam_udp udp_camera.launch.py show:=true detect_qr:=true
    ros2 topic hz /camera/image/compressed
    ros2 run rqt_image_view rqt_image_view

## Test without hardware
    ros2 launch esp32_cam_udp udp_camera.launch.py show:=true
    ros2 run esp32_cam_udp udp_test_sender --host 127.0.0.1      # or --webcam 0

## Tuning
Config: `config/udp_camera.yaml`. If frames drop, keep QVGA, raise jpeg_quality
number to 15-20, and raise the pacing `delay(1)` to `delay(2)` in the firmware.

## QR -> gripper
Plug the gripper ESP32 (flashed with the arm firmware, `GRIPPER:<angle>` protocol) into the laptop.
Find its port with `ls /dev/ttyUSB* /dev/ttyACM*` (or `ls /dev/serial/by-id/`) and run:

    ros2 launch esp32_cam_udp udp_camera.launch.py show:=true gripper_port:=/dev/ttyACM0

Options: `trigger_text:=PICK` (only that QR), `gripper_angle:=0`, `gripper:=false` (camera only).
Manual test: `ros2 topic pub --once /gripper/angle std_msgs/Int32 "{data: 0}"`

Behaviour: QR in view -> GRIPPER:0 (closed); QR gone for `release_timeout` s -> GRIPPER:90 (open).
Image: `rotate_180:=true` (default) rotates the preview; `rotate_published` also rotates the topic.
