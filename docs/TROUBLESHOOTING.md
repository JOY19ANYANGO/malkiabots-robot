# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `/waste/enable not available`, service list empty, rqt shows a grey gradient, although `systemctl is-active waste-detector` says `active` | the clock was changed after the detector started: programs on either side of a time jump can't discover each other | `sudo systemctl restart waste-detector && ros2 daemon stop`; restart anything else that was running. Always set the clock first |
| Logs and file dates say October | the Pi has no clock battery | `sudo date -s "..."` before anything else, or fit a Pi 5 RTC battery |
| Mission uses old poses (`GoToFinish` in the startup lines, robot heads for the junction first) | `mission.launch.py` loads `malkia_mission/config/mission.yaml`, which was out of date | copy `~/maps/mission.yaml` over it and rebuild `malkia_mission` |
| `waste_detected` always equals `waste_type_default`, about 16 s after `detector ON` | no votes reached the tree: old detector without the `/waste_type` publisher, or nothing recognised | `grep -c vote_pub ~/waste/hailo_waste_detector.py` must be 2; test with the camera test in the run sheet |
| Organic reads as metal (or any fixed swap) | `CLASSES` order differs from the model's training order | set `CLASSES` in `hailo_waste_detector.py` to the training `names:` order, restart the service |
| `No module named hailo_platform` | running the detector with the system Python | the service uses `~/hailo_venv/bin/python3`; don't start it by hand |
| `rpicam-still`: `Pipeline handler in use by another process` | the service already holds the camera | normal; stop the service first if you need a still |
| Robot drives the wrong way from the start | AMCL never got the 2D Pose Estimate (it starts at 6.93, -4.76) or the arrow was dragged backwards | redo the pose estimate; `ros2 topic echo /amcl_pose --once` |
| Scans don't sit on the walls when the robot is at an angle | wrong `track_width` (turns mis-measured) | keep `track_width: 0.36` |
| Scans slide off after the grass | wheel slip on grass, AMCL loses confidence | see the AMCL ideas in TUNING.md; a relocalise step after path1 is not in the tree yet |
| Spins for a long time at the end of a route | RPP approach speed too low near the goal, or the goal faces a different way to the arrival | routes now end facing the bins; RPP idea in TUNING.md |
| Stuck at the open gate | the open gate narrows the lane; the old path hugged it | path2 now drops to y -1.22 before the gate |
| Servo buzzes or stalls | gate too stiff, or the servo is powered from the Nano 5 V pin | free the hinge; separate 5-6 V supply with common ground |
| SLAM map stops drawing after the ramp / clock jump | `transform_timeout`, clock jump | set the clock first; slam_toolbox `transform_timeout 0.5`, `tf_buffer_duration 30` |
| `ros2 run malkia_mission mission_tree`: `No package metadata was found` | workspace not sourced, or a half-finished build | `source ~/ros2_ws/install/setup.bash`; else `rm -rf build/malkia_mission install/malkia_mission` and rebuild |
