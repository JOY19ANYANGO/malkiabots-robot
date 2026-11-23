# cramped_robot — physical implementation

The simulation and the real robot share the same URDF, map format, SLAM and Nav2 configuration.
Only the bottom layer changes: Gazebo's DiffDrive/lidar/IMU plugins are replaced by a motor
microcontroller, a lidar driver and (optionally) an IMU driver. Everything above `/cmd_vel`, `/odom`,
`/scan`, `/imu` and TF is identical.

```
             Gazebo                          Real robot
  /cmd_vel -> gz DiffDrive        /cmd_vel -> base_driver.py -> USB serial -> base_firmware.ino -> motors
  /odom, odom->base_footprint     <- base_driver.py (encoders)        [or robot_localization EKF]
  /scan   <- gz gpu_lidar         <- rplidar_ros / ldlidar_stl_ros2 (frame lidar_link)
  /imu    <- gz IMU               <- IMU node (frame imu_link)
```

## 1. Hardware

| Item | Spec / note |
|---|---|
| Compute | Raspberry Pi 4/5 (4 GB+) with Ubuntu 24.04 + ROS 2 Jazzy. A laptop on the robot also works. |
| Motor MCU | Arduino Nano/Uno/Mega or ESP32 running `firmware/base_firmware.ino` |
| Motor driver | 2 channels, ≥ 3 A each (TB6612FNG for small 37D motors, L298N or BTS7960 for stall > 2 A). One channel per **side**; the two motors of a side in parallel. |
| Motors | 4 × 37 mm gearmotors (as in the CAD), ideally with hall encoders; at least one encoder per side |
| Lidar | 2-D 360° lidar on the cabin roof: RPLIDAR A1/A2 or LDROBOT LD19/LD06. Mount it where `lidar_link` is: roof centre, scan plane ≈ 0.22 m above the ground, cable/0° mark facing **forward** |
| IMU (optional) | MPU-6050/9250, BNO055 or similar, mounted flat on the bottom plate at `imu_link` |
| Power | 3S LiPo (11.1 V) → motor driver; 5 V/3 A buck converter → Pi; separate 5 V for the lidar if it draws > 0.5 A. Common ground everywhere. |
| Safety | Inline fuse on the battery, a physical kill switch on the motor supply |

Keep the motor wiring symmetrical: left pair on channel L, right pair on channel R, both motors of a side
turning the same way under the same command.

## 2. Software on the robot

```bash
# Ubuntu 24.04 on the Pi
sudo apt install ros-jazzy-ros-base ros-jazzy-robot-state-publisher ros-jazzy-xacro \
     ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
     ros-jazzy-slam-toolbox ros-jazzy-robot-localization ros-jazzy-rplidar-ros \
     ros-jazzy-teleop-twist-keyboard python3-serial
# LD19/LD06 instead of RPLIDAR: clone ldrobot's ldlidar_stl_ros2 into the workspace and build it
mkdir -p ~/cramped_robot_ws/src && cp -r cramped_robot_description ~/cramped_robot_ws/src/
cd ~/cramped_robot_ws && colcon build --symlink-install && source install/setup.bash
sudo usermod -aG dialout $USER     # serial port access, then log out/in
```

Flash `firmware/base_firmware.ino` with the Arduino IDE after setting the pin table at the top of the file.

## 3. Bring-up, one layer at a time

**3.1 Motors and encoders (no ROS yet).** Open the Arduino serial monitor at 115200 baud and send
`v 3 3`. Both sides must turn forward at ~3 rad/s and the `e …` lines must show both tick counts
increasing. Fix sign problems with `L_INVERT` / `R_INVERT` (motors) or by swapping encoder A/B
(ticks counting the wrong way). Send `v 0 0` to stop.

**3.2 Base driver.**
```bash
ros2 launch cramped_robot_description robot.launch.py lidar:=none port:=/dev/ttyUSB0
ros2 run teleop_twist_keyboard teleop_twist_keyboard          # drive it; i/j/l/k
ros2 topic echo /odom --field pose.pose                        # must track the motion
```

**3.3 Odometry calibration** (do this before mapping — it decides SLAM/Nav2 quality):
1. *Ticks per wheel revolution*: rotate a wheel by hand exactly 10 turns, read the tick delta, divide by 10 →
   `ticks_per_rev` in `base_driver.py` **and** `TICKS_PER_REV` in the firmware.
2. *Wheel radius*: drive straight 2.0 m (tape measure); if `/odom` reports 1.9 m, multiply `wheel_radius` by 2.0/1.9.
3. *Effective track (skid-steer slip)*: rotate in place 5 full turns; if `/odom` yaw shows 4.5 turns, multiply
   `skid_factor` by 5/4.5. Typical skid_factor is 1.2–1.5 on rubber tyres.
Put the final values in `launch/robot.launch.py` or pass them as `-p` parameters.

**3.4 Lidar.**
```bash
ros2 launch cramped_robot_description robot.launch.py lidar:=rplidar lidar_port:=/dev/ttyUSB1
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix cramped_robot_description)/share/cramped_robot_description/rviz/sim.rviz
```
Hold a box 1 m in front of the robot: the red points must appear at +x in RViz. If they appear rotated,
correct the lidar yaw in the URDF (`lidar_joint` rpy) rather than in the driver.
If the mast/cabin edges show up in the scan, crop them (`angle_compensate` / angle-crop parameters) or
raise the lidar — the mid-plate was trimmed in the model for exactly this reason; do the same physically.

**3.5 IMU (optional).** Run your IMU node with `frame_id: imu_link`, topic `/imu`, then launch with
`use_ekf:=true`. The EKF takes over the `odom → base_footprint` transform (base_driver stops publishing it).

## 4. Map the real arena

```bash
ros2 launch cramped_robot_description robot.launch.py                                   # T1 (on the robot)
ros2 launch cramped_robot_description mapping.launch.py use_sim_time:=false             # T2 (laptop, same ROS_DOMAIN_ID)
ros2 run teleop_twist_keyboard teleop_twist_keyboard                                    # T3
ros2 run nav2_map_server map_saver_cli -f ~/maps/gamefield_real                         # when the map is closed
```
Drive slowly (≤ 0.2 m/s), turn gently, revisit the start for loop closure. The simulated map
(`maps/gamefield.yaml`) is the CAD ground truth; use it to sanity-check the real one — wall positions
should agree to within a few cm. You can also navigate with the CAD map directly if the real arena is
built exactly to the drawing.

## 5. Navigate autonomously

```bash
ros2 launch cramped_robot_description robot.launch.py                                   # T1
ros2 launch cramped_robot_description navigation.launch.py use_sim_time:=false map:=$HOME/maps/gamefield_real.yaml   # T2
```
In RViz click **2D Pose Estimate** on the robot's real position (the yaml's `set_initial_pose` is the Gazebo
spawn pad), then **2D Goal Pose**, or run the patrol:
```bash
ros2 run cramped_robot_description patrol.py --no-initial-pose
```
Edit the `WAYPOINTS` list in `scripts/patrol.py` for your mission.

## 6. Tuning checklist (real robot)

* `nav2_params.yaml` → `FollowPath.max_vel_x` / `max_vel_theta`: start at 0.15 m/s and 1.0 rad/s, raise once stable.
* Skid-steer turns in place with high friction: if the robot stalls while spinning, raise
  `behavior_server.min_rotational_vel` and the controller's `min_speed_theta`, and lower `RotateToGoal.scale`.
* `inflation_radius` 0.30 m suits the 0.5 m-wide corridors; shrink to 0.25 if paths are refused.
* AMCL jitter → raise `max_particles` to 3000 and `laser_likelihood_max_dist` to 3.0.
* If the robot drifts after a turn, redo step 3.3 (track calibration) — it is the usual culprit.
* Lidar `max_laser_range` in `mapper_params_online_async.yaml` and `laser_max_range` in AMCL must match
  your sensor (RPLIDAR A1: 12 m, LD19: 12 m, LD06: 8 m).

## 7. Safety

The firmware stops the motors 0.5 s after the last command (`CMD_TIMEOUT_MS`), and `base_driver.py`
sends zero velocity if `/cmd_vel` goes quiet; keep the hardware kill switch reachable anyway. Test every
Nav2 run first at low speed with someone holding the kill switch.
