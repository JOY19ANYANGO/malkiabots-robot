# malkia_mission — behaviour trees for Robotics Dojo 2026

Three layers, each a tree:

| Layer | File | Engine | Job |
|---|---|---|---|
| Mission | `bt/mission_tree.xml` (view in Groot2) ↔ `malkia_mission/mission_tree.py` (runs) | py_trees_ros | *What* to do: the rulebook sequence, decisions, retries, timestamps |
| Navigation | `bt/navigate_to_pose_no_spin.xml` | Nav2 bt_navigator | *How* to drive between poses; dynamic obstacles → wait / back up / replan |
| Terrain | `bt/navigate_through_poses_terrain.xml` | Nav2 bt_navigator | Path 1 (ramps, gravel, sand): slower replanning, wait-only recovery, never reverse on a slope |

## Mission tree vs. the rulebook (§5 autonomous phase, §7a scoring, §8 required data)

```
Mission (Parallel)
├─ ObstacleMonitor ............................ logs "obstacle_detected" on the first collision-monitor stop   [§8]
└─ AutonomousPhase (Sequence)
   ├─ Log autonomous_start                                                                                 [§8]
   ├─ WasteDetection: GoToScreen → (Retry×3 Timeout15s DetectWasteType | UseAssignedType) → log            [15 pts]
   ├─ Loading:        SelectPose(loading[type]) → GoToLoading → Retry×2 Timeout20s LoadItem → log          [10 pts]
   ├─ RouteDecision:  GoToJunction → Fallback                                                               [10 pts]
   │     ├─ gate open now        → log decision → Path2
   │     ├─ wait ≤ gate_wait_s   → log waiting time + decision → Path2
   │     └─ divert               → log decision → speed cap → Path1 (terrain tree) → cap off
   │   → log terrain_completed                                                                              [15 pts]
   ├─ Depositing:     GoToBins → SelectPose(bin[type]) → GoToBin → Retry×2 Offload → log                   [15 pts]
   ├─ GoToFinish → log finish                                                                              [+time]
```

Why these choices:
- **Detection fallback.** A missed screen reading must not end the run: after 3 × 15 s the tree uses the type
  assigned at the start (`waste_type_default:=…`), accepts the lower detection score, and continues.
- **Gate logic = the scoring rule.** Open → path 2; opens within the standard wait → path 2 (waiting time logged);
  otherwise path 1. Set `gate_wait_s` to the announced standard waiting time — it decides the 10 points.
- **Bins by rule, camera as confirmation.** Blue/white/black ↔ plastic/organic/metal is fixed, so the bin pose comes from
  the map; a `/bin_colour` detector can be added as a check without changing the sequence.
- **Dynamic obstacles** are handled inside Nav2 (costmap + collision monitor) — the mission tree only timestamps them.
  The no-spin tree waits 5 s first because the obstacles are "workers" that move.
- **No reversing on terrain**, no spinning anywhere (skid-steer).

## Interfaces (the contract with the inference and mechanism work)

| Topic | Type | Direction | Produced by |
|---|---|---|---|
| `/waste_type` | `std_msgs/String` `plastic\|metal\|organic` | → tree | camera + classifier at the screen (publish every frame; the tree majority-votes) |
| `/gate_open` | `std_msgs/Bool` | → tree | gate detector: camera, or lidar (free range through the gate > 1 m) |
| `/bin_colour` | `std_msgs/String` `blue\|white\|black` | → tree (optional) | camera at the bins |
| `/loading/request` | `std_msgs/String` (waste type) | tree → | — |
| `/loading/done` | `std_msgs/Bool` | → tree | loading-station protocol node (the "specified communication protocol", §6.4) |
| `/gate` | `std_msgs/Bool` true=open | tree → | `base_node` → servo (exists) |
| `/speed_limit` | `nav2_msgs/SpeedLimit` | tree → Nav2 | — |
| `/mission/events` | `std_msgs/String` JSON | tree → | telemetry uplink to the judges' monitor (§6.7, §8); also `log_file` CSV |

Until the inference nodes exist, `fake_inputs` stands in for the camera, the gate detector and the loading station:
```bash
ros2 run malkia_mission fake_inputs                                     # metal, gate open, loading takes 3 s
ros2 run malkia_mission fake_inputs --ros-args -p gate:=closed          # -> path 1 (ramps)
ros2 run malkia_mission fake_inputs --ros-args -p gate:=6.0             # opens 6 s after loading -> path 2 after waiting
ros2 run malkia_mission fake_inputs --ros-args -p waste_type:=none      # screen unreadable -> waste_type_default
```
The gate detector must publish `/gate_open` continuously (≥ 1 Hz): a reading older than 2 s doesn't count as "open".

## Run

```bash
sudo apt install ros-jazzy-py-trees ros-jazzy-py-trees-ros ros-jazzy-nav2-msgs
cd ~/ros2_ws && colcon build --symlink-install --packages-select malkia_mission && source install/setup.bash
```

**1. Dry run in Gazebo** (no robot needed; `config/mission.yaml` ships with TEST poses on the CAD map):
```bash
ros2 launch cramped_robot_description launch_sim.launch.py                     # T1 Gazebo, robot on the start pad
ros2 launch cramped_robot_description navigation.launch.py                     # T2 Nav2 (sim time, CAD map)
ros2 run malkia_mission fake_inputs                                            # T3 fake camera / gate / station
ros2 launch malkia_mission mission.launch.py                                   # T4 the mission — starts the clock
ros2 topic echo /mission/events                                                # T5 watch the events
```
Expected: screen → loading station → junction → path 2 → bins → finish, then `mission_complete`, and the tree
stops (it does **not** restart; relaunch T4 for another run). Repeat with `gate:=closed` and `waste_type:=none`.
`bt_files` and `log_file` work on any machine (bare BT file names resolve to this package's `bt/` folder,
`~` to your home), and `log_file`'s folder is created if missing.

**2. Real robot** (CAD map, robot on the start pad facing +x):
```bash
ros2 launch malkia_base bringup.launch.py                                                                    # T1
ros2 launch cramped_robot_description navigation.launch.py use_sim_time:=false rviz:=false \
     map:=$HOME/maps/gamefield.yaml x:=-0.54 y:=1.15 yaw:=0.0                                                # T2
ros2 run malkia_mission fake_inputs          # T3 — only until the real camera / gate / station nodes exist
ros2 launch malkia_mission mission.launch.py waste_type_default:=metal                                       # T4
```
**Which map?** `mission.yaml` poses are in world (CAD / Gazebo) coordinates. On the CAD map nothing else is needed.
On a SLAM map whose mapping started on the start pad (e.g. `map_1`), add
`map_origin_x:=-0.54 map_origin_y:=1.15 map_origin_yaw:=0.0` to the mission launch and the tree converts every pose.

The tree checks `mission.yaml` at start-up and refuses to run if a pose lies outside `field_bounds`
(a forgotten placeholder) or a `bt_files` path doesn't exist.

## Before competition day
1. Replace every TEST pose in `config/mission.yaml` with the real one: RViz **Publish Point** on the spot, read `ros2 topic echo /clicked_point` (CAD map frame).
2. Put the two Nav2 XML paths in `bt_files` (install paths on the robot).
3. Set `gate_wait_s` to the announced standard waiting time; `terrain_speed` after a ramp test.
4. Dry-run with the fake publishers above, then with the real detector; check `mission_events.csv` has every §8 event.
5. Tick the 3-minute budget: detection ≤ 20 s, loading ≤ 30 s, route ≤ 60 s, depositing ≤ 40 s, finish ≤ 20 s.
