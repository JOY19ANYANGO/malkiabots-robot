# Legacy firmware - do not upload to the robot

`base_firmware.ino` is the early, simulation-era sketch that pairs with `scripts/base_driver.py`
(`v` / `r` / `e` protocol, two encoders, no IMU, no servo). The robot does **not** run it.

The firmware on the robot's Arduino Nano is [`firmware/Nano_code/Nano_code.ino`](../../firmware/Nano_code/Nano_code.ino)
at the top of this repository; it talks to `malkia_base/base_node.py`.

(This folder stays here because `CMakeLists.txt` installs it.)
