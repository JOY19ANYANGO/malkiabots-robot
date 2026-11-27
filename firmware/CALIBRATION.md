# Motor recalibration

The robot runs the closed-loop firmware in firmware/Nano_code/. This file explains what changed from the
original sketch (firmware/legacy/malkia_nano/) and why it made navigation smooth.

## The problem
With the original firmware, the command "M <left> <right>" from the Raspberry Pi set the motor power
directly and switched the speed controller off. Nothing checked whether the wheels reached the speed Nav2
asked for. On the field this caused three problems:
- The robot drifted to the left on the grass, stopped near a wall and took time to recover.
- It turned back and forth on the spot at the end of goals.
- It veered to the right when going down the ramp.

## What we changed
- Closed loop for every command: M now sets a target speed for each side (a fraction of maxTps, 4500
  ticks/s at PWM 255) and a PI controller holds it, updating every 20 ms.
- Gains: Kp 0.01, Ki 0.05 (were 0.03 and 0.15), integral limit 100, and the integral only grows while the
  output is not at its limit (anti-windup). Gentler gains, so the speed settles without hunting.
- Feed-forward: maxTps changed from 1600 to 4500 ticks/s to match the motors.
- Side speed: half the average of the two wheels plus half the slower wheel, smoothed. A slipping wheel
  can no longer make a side look faster than it is.
- Minimum speed: targets below 1300 ticks/s (about 0.13 m/s) are raised to it, keeping the left/right
  ratio. Small commands now always move the robot instead of stalling.
- Soft start: the target speed changes by at most 4000 ticks/s per second.
- No braking by reversing: a side moving forward is never driven backwards to slow down.
- Gyro heading hold (H command): on straight commands, the MPU6050 gyro trims the two sides to keep the
  heading. The IMU now uses the MPU6050_light library, with the level calibration saved in EEPROM.
- Encoders: the two left encoders are flipped (ENC_FL_REVERSE and ENC_RL_REVERSE = true) so that all four
  count forward when the robot drives forward.
- Wiring: both left motors share the PWM pin D5 (the front driver's left enable is wired to D5 as well),
  right motors on D6, gate servo on D11.
- Unchanged: maximum PWM 200, power ramp of 12 per 20 ms, stall cut-off after 600 ms, and motors stop if no
  command arrives for 500 ms.

## Results on the field
- Grass: the robot still drifts slightly but corrects itself straight away, without going near the wall.
- End of goals: the robot turns once and stops, without spinning.
- Ramp: the robot comes down straight, without veering or tipping.
- Navigation became smooth without any change to the Nav2 settings.
