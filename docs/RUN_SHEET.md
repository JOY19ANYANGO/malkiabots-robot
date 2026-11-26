# Run sheet

Pi address `192.168.5.245` (check your router if it changed). One command per box.

## 0. Every Pi terminal

```bash
ssh malkia@192.168.5.245
```
```bash
export ROS_DOMAIN_ID=7 && source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

## 1. Pre-flight (one Pi terminal, right after boot)

Change the date and time on the first line to now, then paste the whole box.

```bash
sudo date -s "2026-11-24 10:30"
sudo systemctl restart waste-detector && ros2 daemon stop
sleep 15
systemctl is-active waste-detector && ros2 service list | grep waste
ros2 service call /waste/enable std_srvs/srv/SetBool "{data: false}"
```
Expect: `active`, `/waste/enable`, `/waste/reset`, `success=True`.

Never start `rpicam-vid` or the detector by hand: the `waste-detector` service runs both.

## 2. Start the robot (one Pi terminal each)

T1, robot base:
```bash
ros2 launch malkia_base bringup.launch.py
```
T2, navigation (wait until the startup lines stop):
```bash
ros2 launch cramped_robot_description navigation.launch.py use_sim_time:=false rviz:=false map:=$HOME/maps/gamefield_nopillars.yaml
```
T3, gate check and loading timer:
```bash
python3 ~/real_inputs.py
```
Expect: `real inputs: gate from lidar, loading wait 5.0 s`.

## 3. Start pose (laptop, no SSH)

```bash
export ROS_DOMAIN_ID=7 && source /opt/ros/jazzy/setup.bash && rviz2
```
Fixed Frame `map`; add `/map` (Map) and `/scan` (LaserScan). Robot on the start pad, **2D Pose Estimate**,
drag the way it faces. The red dots must sit on the walls. Then check on the Pi:
```bash
ros2 topic echo /amcl_pose --once
```
x and y must match the real spot (not about 6.93, -4.76).

## 4. Mission (T4)

```bash
ros2 launch malkia_mission mission.launch.py waste_type_default:=metal
```
Startup must show `NavVia/GoToLoading` and **no** `GoToFinish`.

Expected order: `GoToScreen` -> `AlignAtScreen: aligned` -> `detector ON` -> `waste_detected <type>` ->
`DockAtLoading: docked` -> `loading_completed` -> `junction_reached` -> `gate_decision path2_open | path1_divert`
-> `DockAtBin: docked` -> gate opens -> `mission_complete` (under 180 s).

## 5. Watch (laptop)

```bash
export ROS_DOMAIN_ID=7 && source /opt/ros/jazzy/setup.bash && ros2 topic echo /mission/events
```
```bash
export ROS_DOMAIN_ID=7 && source /opt/ros/jazzy/setup.bash && ros2 run rqt_image_view rqt_image_view /waste/image/compressed
```

## 6. Another run, stop, results

- Another run: Ctrl+C in T4, robot back on the start pad, redo step 3, start T4 again. T1-T3 stay up.
- Emergency stop: Ctrl+C in T4, then in T2.
- Results: `tail -40 ~/maps/mission_events.csv`

## 7. Camera test (robot still)

```bash
ros2 service call /waste/enable std_srvs/srv/SetBool "{data: true}"
```
```bash
PYTHONUNBUFFERED=1 timeout -s INT 8 ros2 topic echo /waste_type std_msgs/msg/String --field data | grep -v -- --- | sort | uniq -c
```
```bash
ros2 service call /waste/enable std_srvs/srv/SetBool "{data: false}"
```
