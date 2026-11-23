# cramped_robot_description

ROS 2 URDF/xacro description of the 4-wheel skid-steer platform in `Main_Assembly.step`
(Onshape export, 2026-09-22). Geometry, mass properties and colours were extracted from the
STEP solids by `scripts/build_from_step.py`.

```
urdf/cramped_robot.urdf.xacro           main model
urdf/cramped_robot.gazebo.xacro         gz-sim: 4-wheel DiffDrive, joint states, IMU, 2-D lidar
urdf/cramped_robot.ros2_control.xacro   velocity interfaces for the 4 wheel joints
urdf/cramped_robot.urdf                 pre-expanded plain URDF (defaults)
meshes/visual/base_link.stl, wheel.stl  metres, link-local frames
config/controllers.yaml                 diff_drive_controller + joint_state_broadcaster (ros2_control)
config/gz_bridge.yaml                   ros_gz_bridge topic map
config/mapper_params_online_async.yaml  slam_toolbox parameters
launch/display.launch.py                RViz + joint_state_publisher_gui
launch/launch_sim.launch.py             Gazebo Harmonic + arena + robot spawn + bridge (+ rviz:=true)
launch/mapping.launch.py                slam_toolbox online_async + RViz
launch/navigation.launch.py             Nav2: map_server + AMCL + planner/controller/behaviors/bt_navigator + RViz
launch/robot.launch.py                  PHYSICAL robot: base driver + lidar (+ EKF) + robot_state_publisher
config/nav2_params.yaml                 Nav2 parameters (footprint, DWB limits, costmaps, AMCL)
config/ekf.yaml                         robot_localization (wheel odom + IMU), real robot
maps/gamefield.{pgm,yaml}               arena map (CAD ground truth at lidar height, 2 cm cells)
config/coverage_waypoints.yaml          80-waypoint route covering the whole reachable field
config/patrol_waypoints.yaml            short 8-point lap
scripts/patrol.py                       autonomous route follower (nav2_simple_commander)
scripts/make_coverage_route.py          generates a coverage route from any map
scripts/base_driver.py                  real-robot motor/odometry node (serial to the MCU)
scripts/make_map_from_arena.py          regenerates maps/gamefield.* from the arena mesh
firmware/base_firmware.ino              Arduino/ESP32 motor + encoder firmware
docs/PHYSICAL_IMPLEMENTATION.md         step-by-step hardware bring-up
worlds/gamefield.world, models/         the game-field arena
```

## Kinematic tree (8 links, 7 joints)

```
base_footprint ─fixed─ base_link ─┬─continuous (axis y)─ front_left / front_right / rear_left / rear_right _wheel_link
                                  ├─fixed─ lidar_link   (centre of the front-cabin roof, origin on the scan plane)
                                  └─fixed─ imu_link     (floor of the lower cabin, centred)
```

`base_link` is one rigid body: all chassis plates, the 10 brackets, and the four motor mounts +
gearmotors. Only the wheels move.

| Parameter | Value | Source |
|---|---|---|
| Wheel radius / width | 40.6 mm / 38.4 mm | tyre solid |
| Wheelbase / track | 130.0 mm / 216.0 mm | motor axes / wheel centres |
| Axle height | 19.8 mm below base_link | shaft axis |
| base_link above ground | 60.4 mm | tyre bottom |
| Lidar | roof centre (x = 0.085 m), scan plane 0.220 m above ground | roof top face + 25 mm |
| IMU | x = y = 0, 5 mm above base_link | bottom-plate top face |
| Mass (base / wheel / total) | 1.58 / 0.069 / 2.00 kg | user estimate 2.0 kg, split by CAD volume × density |

Frames follow REP-103 (x forward, y left, z up). The CAD front is −Y, so
`x_urdf = −y_cad, y_urdf = x_cad`. `base_link` is at the CAD origin (centre of the bottom face
of the bottom plate).

## Notes

* The red/green/blue axis triads seen in viewers are the TF frames, one per link. This model keeps
  them to the minimum (8); the RViz config ships with the TF display off.
* Design change vs. the STEP: the inclined mid-plate is cut off at roof height (z = 0.135 m in `base_link`)
  so nothing rises into the lidar's 360° field of view (scan plane 25 mm above the roof). Trim the physical
  plate the same way, or set `TRIM_MIDDLE_PLANE_TO_ROOF = False` in `scripts/build_from_step.py` to restore it.
* Mass: `total_mass` (2.0 kg, user estimate) is spread over the links in proportion to the CAD-derived
  masses (acrylic plates, aluminium brackets/mounts, ~0.19 kg gearmotors, ~0.09 kg wheels, 0.17 kg lidar,
  0.01 kg IMU placeholders). Edit `total_mass` or the `*_mass_cad` values; inertia tensors follow.
* Motor limits (1 N·m, 10.5 rad/s) are typical 37D 12 V values, not from the model.
* The CAD wheels sit 0.4 mm off the motor axis (mate tolerance); the URDF puts them on the axis.

## Use

```bash
# deps (ROS 2 Jazzy / Gazebo Harmonic)
sudo apt install ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-nav2-map-server
cd ~/cramped_robot_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch cramped_robot_description display.launch.py            # RViz only
ros2 launch cramped_robot_description launch_sim.launch.py         # Gazebo + arena + robot + bridge
ros2 launch cramped_robot_description launch_sim.launch.py x:=-2.6 y:=1.15 yaw:=0.0   # other spawn pose
```

### Mapping (SLAM) — build your own map

```bash
ros2 launch cramped_robot_description launch_sim.launch.py                       # terminal 1
ros2 launch cramped_robot_description mapping.launch.py                          # terminal 2 (slam_toolbox + RViz)
ros2 run teleop_twist_keyboard teleop_twist_keyboard                             # terminal 3 — drive around the arena
ros2 run nav2_map_server map_saver_cli -f ~/maps/gamefield --ros-args -p use_sim_time:=true   # when done
```

### Autonomous navigation (Nav2)

`maps/gamefield.yaml` is already provided (exact CAD slice of the arena at the lidar's scan height), so
navigation works without mapping first:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander
ros2 launch cramped_robot_description launch_sim.launch.py        # terminal 1: Gazebo
ros2 launch cramped_robot_description navigation.launch.py        # terminal 2: Nav2 + RViz (AMCL starts at the spawn pad)
ros2 run cramped_robot_description patrol.py                      # terminal 3: drives the WHOLE reachable field
```

`patrol.py` follows `config/coverage_waypoints.yaml`: 78 waypoints generated from the map by
`scripts/make_coverage_route.py` (free space eroded by the robot's clearance, one waypoint per 0.45 m cell,
ordered by shortest path through the maze, each facing the next) — 98 % of the reachable floor passes within
0.35 m of the route, 28 m per lap; every waypoint is >= 0.22 m from any wall. `--loop` runs the short 8-point lap, `--through` drives the route
continuously (NavigateThroughPoses), `--once` stops after one lap, `--file x.yaml` uses your own route.
Regenerate the route for a different map or start pose:
`python3 scripts/make_coverage_route.py maps/gamefield.yaml config/coverage_waypoints.yaml --start -0.54 1.15`.

One area is deliberately excluded: the pocket along the west wall (x < −2.3, including the room behind the
small block at (−2, −1.1)) is only entered through 0.15–0.20 m slots — narrower than the robot (0.255 m).
Or click **2D Goal Pose** in RViz for single goals.

**Obstacle avoidance, three layers.** (1) Global planner on the CAD map (walls + pillars solid, unknown space
forbidden). (2) DWB with `ObstacleFootprint` on a local costmap that carries the static map *and* the live
scan, so the whole footprint is checked against known and seen obstacles. (3) `nav2_collision_monitor` as the
last stage before `/cmd_vel`: a laser point inside the red stop polygon (footprint + 3 cm) halts the robot
whatever Nav2 commanded, the orange slowdown polygon (0.38 m ahead) cuts speed to 40 %. Both polygons are
shown in RViz. The simulated lidar's minimum range is 0.05 m so there is no blind zone at the bumper.

Frames: `map` → `odom` (AMCL or slam_toolbox) → `base_footprint` (Gazebo DiffDrive / base_driver / EKF) → `base_link` → …
Velocity chain: Nav2 → `/cmd_vel_nav` → velocity_smoother → `/cmd_vel` → Gazebo bridge or `base_driver.py`.

### Troubleshooting (sim)

* `GridBased plugin failed to plan` + `Collision Ahead - Exiting Spin`: the robot is pinned against a wall, its footprint is
  inside lethal cost. Teleop it clear, re-set the pose with **2D Pose Estimate**, resend the goal.
* A ring of laser points around the robot itself: the lidar sensor origin is inside a visual. The puck visual in the URDF now
  ends 3 mm below the scan plane — keep it that way if you change the lidar.
* Robot under-rotates in Gazebo (turns in RViz but not in Gazebo): wheel lateral friction. The wheels use `mu1 = 1.0`,
  `mu2 = 0.15`; raise `mu2` only if the robot slides on straights.
* `Waiting for service velocity_smoother/get_state` forever: the smoother is off by default; enable with
  `navigation.launch.py velocity_smoother:=true` once `ros2 run nav2_velocity_smoother velocity_smoother` starts cleanly.
* Robot stops and won't move, `ros2 topic echo /collision_monitor_state` shows `PolygonStop`: a laser point is inside
  the stop zone (robot nudged against something, or a stray return). Teleop it 5 cm clear and it resumes.
* Using a SLAM-built map (`map:=...`): its origin is where mapping started, so the preset AMCL pose is wrong — click
  **2D Pose Estimate** before the first goal.

### Physical robot

See `docs/PHYSICAL_IMPLEMENTATION.md`. Short version: flash `firmware/base_firmware.ino`, then
`ros2 launch cramped_robot_description robot.launch.py`, map with `mapping.launch.py use_sim_time:=false`,
navigate with `navigation.launch.py use_sim_time:=false map:=<your map>`.

### Arena

`worlds/gamefield.world` loads `models/gamefield_arena` (6.0 × 3.0 m, x ∈ [−3, 3], y ∈ [−1.53, 1.53]).
Floor top is at z = 0.05, the two raised start pads at z = 0.08 (centres (−0.54, 1.15) and (0.92, −0.49)),
walls 0.5 m. The default spawn is the centre of the first pad; anything outside the arena rectangle lands on
the ground plane at z = 0.
