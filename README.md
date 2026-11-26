# Malkiabots robot (Robotics Dojo 2026)

A 4-wheel skid-steer waste-sorting robot. It reads the waste type from a screen with a camera, collects the
item at the matching loading chute, picks a route at the gate, and drops the item in the matching bin, all
autonomously, in under 3 minutes.

| | |
|---|---|
| Computer | Raspberry Pi 5 (Ubuntu 24.04) + Hailo-8L AI HAT |
| Micro-controller | Arduino Nano (wheel encoders, motor driver, MPU6050, servo gate) |
| Sensors | RPLIDAR A1, Pi camera (IMX219) |
| Software | ROS 2 Jazzy, Nav2, slam_toolbox, py_trees / py_trees_ros, YOLO (Hailo) |

## Repository layout

This repository is the `src/` folder of the ROS 2 workspace (`~/ros2_ws/src`).

| Folder | What it is |
|---|---|
| `malkia_base/` | Base driver. `base_node` turns encoder ticks into `/odom` + TF `odom -> base_footprint`, turns `/cmd_vel` into motor PWM and `/gate` into servo open/close. `launch/bringup.launch.py` starts it with the lidar and the robot model. Settings: `config/base.yaml` (track width 0.36, wheel 0.065 m, 1980 ticks/rev). |
| `cramped_robot_description/` | Robot model (URDF), Nav2 settings (`config/nav2_params.yaml`), SLAM settings, the `navigation`, `mapping` and simulation launch files and the Gazebo field. (Its `firmware/` folder is a legacy sketch: not the robot's.) |
| `malkia_mission/` | The mission behaviour tree (`malkia_mission/mission_tree.py`), the Nav2 trees it uses (`bt/`) and the mission poses (`config/mission.yaml`). |
| `firmware/Nano_code/` | **Arduino Nano firmware** running on the robot: closed-loop motor speed control, 4 encoders, MPU6050, gate servo, serial protocol to `base_node`. See [firmware/README.md](firmware/README.md). |
| `extras/waste_detector/` | Hailo YOLO waste detector and the systemd service that starts the camera stream and the detector at boot. |
| `extras/scripts/real_inputs.py` | Lidar gate check (`/gate_open`) and loading timer. |
| `extras/maps/` | Navigation map `gamefield_nopillars` and the copy of `mission.yaml` the robot runs from (`~/maps/mission.yaml`). |
| `extras/sync_extras.sh` | Copies the files that run from the home folder into this repo. Run it before every commit on the Pi. |
| `docs/` | [Architecture](docs/ARCHITECTURE.md), [run sheet](docs/RUN_SHEET.md) ([PDF](docs/run_sheet.pdf)), [tuning notes](docs/TUNING.md), [troubleshooting](docs/TROUBLESHOOTING.md). |

`firmware/`, `extras/` and `docs/` carry a `COLCON_IGNORE`, so `colcon build` skips them. The lidar driver
`sllidar_ros2` is not stored here: it is cloned from Slamtec (`dependencies.repos`).

## Install on the Pi

```bash
# 1. workspace + this repo + lidar driver
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/<your-user>/<this-repo>.git src
git clone https://github.com/Slamtec/sllidar_ros2.git src/sllidar_ros2
rosdep install --from-paths src --ignore-src -y
colcon build && source install/setup.bash

# 2. files that run from the home folder
mkdir -p ~/waste ~/maps
cp src/extras/scripts/real_inputs.py ~/
cp src/extras/waste_detector/* ~/waste/
cp src/extras/maps/* ~/maps/

# 3. camera + detector service (starts at every boot)
sudo cp ~/waste/waste-detector.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now waste-detector
```

Flash the Nano from `firmware/Nano_code/` (see [firmware/README.md](firmware/README.md)).

Not in git (copy them by hand): the Hailo model `~/waste/waste.hef` (YOLO26n, 2026-11-19 build, compiled for
Hailo-8L) and the HailoRT 4.23.0 Python environment `~/hailo_venv`.

## Run a mission

Full version with checks: [docs/RUN_SHEET.md](docs/RUN_SHEET.md). Every Pi terminal first runs
`export ROS_DOMAIN_ID=7 && source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash`.

```bash
# pre-flight (the Pi has no clock battery: set the clock FIRST, then restart the detector)
sudo date -s "YYYY-MM-DD HH:MM"
sudo systemctl restart waste-detector && ros2 daemon stop
ros2 service call /waste/enable std_srvs/srv/SetBool "{data: false}"

ros2 launch malkia_base bringup.launch.py                                   # T1
ros2 launch cramped_robot_description navigation.launch.py use_sim_time:=false rviz:=false \
    map:=$HOME/maps/gamefield_nopillars.yaml                                # T2
python3 ~/real_inputs.py                                                    # T3
# laptop: rviz2 -> 2D Pose Estimate on the start pad
ros2 launch malkia_mission mission.launch.py waste_type_default:=metal      # T4
```

## What the mission does

```
Screen -> square up -> read waste type (camera vote) -> loading chute -> dock (tail 75 cm from wall) -> wait 5 s
  -> junction -> lidar reads the gate (up to 5 s) -> path2 (gate open) or path1 over the grass (gate closed)
  -> bins -> matching bin -> dock -> servo gate opens 2.5 s -> done
```

| Waste | Station colour | Loading chute | Bin |
|---|---|---|---|
| metal | red | nearest the start | metal |
| organic | green | middle | organic |
| plastic | blue | top | plastic |

Every step is logged with a timestamp to `~/maps/mission_events.csv` and `/mission/events`.
Details: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Working on the code

On the Pi, after changing anything:

```bash
~/ros2_ws/src/extras/sync_extras.sh          # copy real_inputs.py, the detector, the map and mission.yaml in
cd ~/ros2_ws && colcon build --packages-select malkia_mission   # (or the package you changed)
cd ~/ros2_ws/src && git add -A && git commit -m "what changed" && git push
```

`malkia_mission/config/mission.yaml` (used by `mission.launch.py`) and `extras/maps/mission.yaml`
(`~/maps/mission.yaml`, used by `ros2 run ... -p config:=`) must stay identical. Change both, or copy one
over the other.

## Known issues

See [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md). The ones that bite most often:

- **Clock**: the Pi boots about a month behind. Programs started before `date -s` and programs started after it
  cannot see each other. Set the clock first, then restart everything. A Pi 5 RTC battery fixes this for good.
- **AMCL start pose**: `nav2_params.yaml` still starts AMCL at the Gazebo spawn point (6.93, -4.76), which is off
  this field. Always do the 2D Pose Estimate, and check it with `ros2 topic echo /amcl_pose --once`.
- **Detector class order**: `CLASSES` in `hailo_waste_detector.py` must match the order the model was trained
  with. If organic reads as metal, fix that list.
