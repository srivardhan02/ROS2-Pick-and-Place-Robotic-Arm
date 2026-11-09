# robotic_arm_description (v0.2)

URDF generated from `Robotic_Arm_3D_Model.STEP`.

Controllable joints: waist, shoulder, elbow, wrist_roll, wrist_pitch, gripper_joint
(5 arm joints + 1 gripper joint). The other gripper joints are mimic joints that follow `gripper_joint`.

Gripper (v0.2): the real parallelogram linkage is modelled with mimic joints, so the link pins stay
connected while it opens and closes (pin gap < 0.5 mm over the whole range). The gear pair moves
oppositely, each jaw stays parallel, each link pair follows its gear.
gripper_joint: negative = close (jaws touch at about -0.55 rad), positive = open (about +0.8 rad).

Notes
* Zero pose = the pose stored in the STEP file (gripper half open), not the servo mid-position.
* Arm joint limits (+-90 deg) are defaults; set them to what your printed arm really allows.
* Joint positive directions are by convention (waist CCW, shoulder forward, elbow forward/down).
* Masses are estimates (servo datasheet masses + printed parts at ~30% infill).
* `gripper_tip` is an estimated tool point between the fingertips.

Run (ROS 2):
    cp -r robotic_arm_description ~/ros2_ws/src/
    cd ~/ros2_ws && colcon build --packages-select robotic_arm_description
    source install/setup.bash
    ros2 launch robotic_arm_description display.launch.py
