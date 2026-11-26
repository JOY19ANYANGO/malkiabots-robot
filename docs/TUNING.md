# Tuning notes

All poses are on the `gamefield_nopillars` map (CAD frame, origin (-3.315, -1.831), 2 cm cells), in metres
and radians. Yaw 0 faces +x, 1.57 faces +y, 3.14 faces -x, -1.57 faces -y.

## Field and robot

| | |
|---|---|
| Field | 6 m x 3 m (inner walls about x -2.975 .. 3.0, y -1.50 .. 1.50) |
| Lanes | 60-62 cm (most), 66 cm (start/screen lane, far-right lane), bin bays 52 cm |
| Open gate | lies along the inner wall, pointing left, sticks out 10 cm: the bottom lane narrows to about 50 cm |
| Robot (Nav2 footprint) | 28 cm long x 26 cm wide, turning circle about 38 cm |
| Grass stretch | 143.5 cm (path1), driven at 0.12 m/s |

## Mission poses (`mission.yaml`)

| Pose | x | y | yaw | Why |
|---|---|---|---|---|
| screen | -2.485 | -1.20 | 3.14 | front 30 cm from the screen, centred on it (screen 22 cm from the bottom wall, 17 cm wide) |
| loading plastic | -2.085 | 1.189 | 0.0 | top chute; tail 75 cm from the wall (wall face x -2.975, tail 0.14 m behind centre) |
| loading organic | -2.085 | 0.619 | 0.0 | middle chute |
| loading metal | -2.085 | 0.049 | 0.0 | chute nearest the start |
| junction | 2.00 | -0.95 | 3.14 | 0.5 m from the gate, inside the 1.5 m lidar range |
| bins | -0.90 | -0.50 | -1.57 | end of both routes, already facing the bins |
| bin plastic | -1.83 | -0.55 | -1.57 | 5 cm deeper into the offloading zone |
| bin organic | -1.47 | -1.18 | -1.57 | |
| bin metal | -0.89 | -1.18 | -1.57 | |

Routes: **path2** (flat, gate open) drops to the corridor centre (y -1.22) before the open gate, then
(0.00, -1.15) -> (-0.15, -0.30) -> (-0.15, 0.35) -> (-0.90, 0.35) -> bins. **path1** (grass) starts at
(0.90, -0.50) and joins the same end. Corner checkpoints for each leg and waste type are under `via:`.

## Timings and thresholds

| Setting | Value | Where |
|---|---|---|
| Detection vote | 10 votes, 70 %, 2 tries x 8 s | `mission.yaml` `min_votes`, `min_share`; tree `Detect8s` |
| Loading wait | 5 s fixed | `load_wait_s` |
| Gate wait | up to 5 s | `gate_wait_s` |
| Servo open | 2.5 s | `offload_open_s` |
| Square-up tolerance | 3 deg (screen, bins), 1.5 deg (loading) | `screen_align_tol_deg`, `bin_align_tol_deg`, `loading_align_tol_deg` |
| Docking distance tolerance | 2 cm, at 0.08 m/s | `DockAt(tol_m, speed)` in the tree |
| Detector confidence | 0.25 | `extras/waste_detector/start_waste_detector.sh` `CONF` |

Last full run (23 Nov, camera off, gate closed): **151 s**. With detection working (about 2 s instead of 16 s)
and path2 open it should be well under that.

## Base

`malkia_base/config/base.yaml`: `track_width: 0.36` is the **effective** skid-steer width (the measured
22 cm made every turn wrong and bent the map), wheel diameter 0.065 m, 1980 ticks per wheel turn,
`base_frame: base_footprint`.

## Ideas not applied yet

- RPP: `min_approach_linear_velocity` 0.05 -> 0.12, `approach_velocity_scaling_dist` 0.4 -> 0.25 (less crawling
  near goals).
- AMCL after the grass: `z_hit` 0.7, `z_rand` 0.3, `laser_max_range` 4.0.
- Tighter corners: move the corner checkpoints inward first; `inflation_radius` 0.20 -> 0.15 only if needed.
- A relocalise step after path1 (the grass), before the bins.
- Set AMCL's `initial_pose` to the real start pad so it is right even without a 2D Pose Estimate.
