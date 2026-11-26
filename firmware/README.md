# Arduino Nano firmware

`Nano_code/Nano_code.ino` is the sketch running on the robot's Arduino Nano. It drives the motors with a
closed-loop speed controller, counts the four wheel encoders, reads the MPU6050, moves the gate servo and
streams telemetry to `malkia_base/base_node.py` on the Pi over USB at 115200 baud.

The source cannot be read back off the Nano (the chip only holds the compiled program): this folder is the
only copy. Commit every change.

## Upload

1. Arduino IDE (or `arduino-cli`), board **Arduino Nano**, processor **ATmega328P** (try
   "ATmega328P (Old Bootloader)" if the upload times out).
2. Libraries: **MPU6050_light** (by rfetick, Library Manager). `Servo`, `Wire` and `EEPROM` come with the IDE.
3. Open `Nano_code/Nano_code.ino` (the folder name must match the file name) and upload.
4. Stop `bringup` on the Pi first: only one program can hold the serial port.
5. After a reset the Nano calibrates the gyro for about a second: keep the robot still.

## Wiring

| Function | Pins |
|---|---|
| Left motors | PWM D5 (both left motors, front driver's left ENA wired here too), direction D9 / D10 |
| Right motors | PWM D6, direction A0 / A1 |
| Encoder front-left | A = D2 (INT0), B = D4 |
| Encoder front-right | A = D3 (INT1), B = D7 |
| Encoder rear-left | A = D8 (pin-change, PB0), B = A2 |
| Encoder rear-right | A = D12 (pin-change, PB4), B = A3 |
| Gate servo | D11 (open 90 deg, closed 0 deg) |
| MPU6050 | SDA = A4, SCL = A5, address 0x68 |

Motor driver: Handsontec 7A/160W dual H-bridge, inputs active high. The `Servo` library takes Timer1, so D9 and
D10 cannot do PWM; they are only used as direction pins, which is fine. Power the servo from its own 5-6 V supply
with a common ground if it stalls or resets the Nano.

## Serial protocol (newline-terminated)

Pi to Nano:

| Command | Meaning |
|---|---|
| `M <l> <r>` | speed, -255..255 per side as a fraction of the top speed (closed loop); `M 0 0` stops at once |
| `X` | stop |
| `V <l> <r>` | speed in encoder ticks per second (closed loop) |
| `O` / `C` | open / close the gate |
| `A <deg>` | gate servo to an angle (0-180) |
| `H <gain>` | heading hold while driving straight (0 = off); `base_node` sends it at start-up |
| `P <kp*1000> <ki*1000>` | speed-loop gains |
| `K <ticks/s/s>` | soft-start rate (0 = jump) |
| `U <ticks/s>` | smallest speed target |
| `N <pwm>` | smallest power while moving |
| `W <ticks/s>` | top speed at PWM 255 (feed-forward) |
| `Z` | zero encoders and yaw |
| `T <ms>` | telemetry period (0 = off) |
| `L` / `R` | save "level" for the accelerometer in EEPROM / forget it |
| `F <0/1>` | accepted for compatibility (front-left always shares D5) |
| `Q` | print the controller state |

Nano to Pi, every 50 ms:

```
D,<ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<pitch>,<roll>,<yaw>,<enc_fl>,<enc_fr>,<enc_rl>,<enc_rr>
```
Accelerations in g, rates in deg/s, angles in deg, encoders as cumulative ticks. All-zero accelerations mean
the MPU6050 was not found. Replies: `OK <cmd>`, `ERR ...`, `ERR stall left|right`.

## Safety built in

- Motors stop if no `M` command arrives for 500 ms (`CMD_TIMEOUT_MS`), matching `cmd_timeout: 0.5` in
  `malkia_base/config/base.yaml`.
- Power never exceeds PWM 200 (`MAX_PWM`) and ramps by at most 12 per 20 ms.
- A side is cut if it is driven above PWM 90 but its wheels do not turn for 600 ms (`ERR stall`). A new,
  different command clears it.

## Main constants

| Constant | Value | Notes |
|---|---|---|
| `pidKp`, `pidKi` | 0.01, 0.05 | speed loop, PWM per tick/s |
| `maxTps` | 4500 | ticks/s at PWM 255 |
| `minTps` | 1300 | smallest speed target |
| `accelTps` | 4000 | soft start |
| `hdgGain` | 15 | heading hold (overwritten by `H` from `base_node`) |
| `INVERT_LEFT` / `INVERT_RIGHT` | true / false | flip if a side runs backwards |
| `ENC_*_REVERSE` | FL, RL true | flip if an encoder counts backwards |
