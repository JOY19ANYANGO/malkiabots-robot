"""Pretend Nano for testing without hardware (Linux only).

Creates a pseudo serial port, linked at /tmp/ttyFAKE, speaks the same protocol
as the real firmware and moves simulated wheels when it receives M commands.
"""
import math
import os
import pty
import time

MAX_TICKS_PER_S = 1600.0     # encoder counts per second at PWM 255
TRACK = 0.20                 # m, only used to fake a gyro reading
MAX_WHEEL_SPEED = 0.50       # m/s at PWM 255
WATCHDOG_S = 0.5

LINK = '/tmp/ttyFAKE'


def main():
    master, slave = pty.openpty()
    if os.path.islink(LINK) or os.path.exists(LINK):
        os.remove(LINK)
    os.symlink(os.ttyname(slave), LINK)
    os.set_blocking(master, False)
    print(f'fake Nano on {LINK} -> {os.ttyname(slave)}', flush=True)

    left = right = 0
    period_ms = 50
    enc = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
    yaw = 0.0
    last_cmd = time.time()
    last_tel = 0.0
    last = time.time()
    t0 = last
    buf = ''

    def send(text):
        try:
            os.write(master, (text + '\r\n').encode())
        except OSError:
            pass

    while True:
        now = time.time()
        dt = now - last
        last = now

        try:
            buf += os.read(master, 256).decode(errors='replace')
        except (BlockingIOError, OSError):
            pass
        while '\n' in buf or '\r' in buf:
            for sep in ('\n', '\r'):
                if sep in buf:
                    line, buf = buf.split(sep, 1)
                    break
            parts = line.strip().split()
            if not parts:
                continue
            c = parts[0].upper()
            if c == 'M' and len(parts) == 3:
                left, right = int(parts[1]), int(parts[2])
                last_cmd = now
                send('OK M')
            elif c == 'X':
                left = right = 0
                last_cmd = now
                send('OK X')
            elif c == 'Z':
                enc = {k: 0.0 for k in enc}
                yaw = 0.0
                send('OK Z')
            elif c == 'T' and len(parts) == 2:
                period_ms = int(parts[1])
                send('OK T')
            elif c in ('O', 'C', 'A'):
                send('OK')
            else:
                send('ERR unknown command')

        if now - last_cmd > WATCHDOG_S:
            left = right = 0

        enc['fl'] += left / 255.0 * MAX_TICKS_PER_S * dt
        enc['rl'] += left / 255.0 * MAX_TICKS_PER_S * dt
        enc['fr'] += right / 255.0 * MAX_TICKS_PER_S * dt
        enc['rr'] += right / 255.0 * MAX_TICKS_PER_S * dt
        vl = left / 255.0 * MAX_WHEEL_SPEED
        vr = right / 255.0 * MAX_WHEEL_SPEED
        wz = (vr - vl) / TRACK                       # rad/s
        yaw += math.degrees(wz) * dt

        if period_ms > 0 and now - last_tel >= period_ms / 1000.0:
            last_tel = now
            ms = int((now - t0) * 1000)
            send(f"D,{ms},0.000,0.000,1.000,0.00,0.00,{math.degrees(wz):.2f},"
                 f"0.00,0.00,{yaw:.2f},{int(enc['fl'])},{int(enc['fr'])},"
                 f"{int(enc['rl'])},{int(enc['rr'])}")
        time.sleep(0.005)


if __name__ == '__main__':
    main()
