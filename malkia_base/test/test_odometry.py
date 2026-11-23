import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from malkia_base.odometry import WheelOdometry, twist_to_pwm   # noqa: E402
from malkia_base.protocol import imu_healthy, parse_telemetry  # noqa: E402


def make():
    return WheelOdometry(ticks_per_rev=600.0, wheel_diameter=0.1, track_width=0.2)


def test_straight():
    o = make()
    o.update(0, 0, 0, 0, 0.0)
    o.update(600, 600, 600, 600, 1.0)            # one wheel turn
    assert abs(o.x - math.pi * 0.1) < 1e-9 and abs(o.y) < 1e-9
    assert abs(o.theta) < 1e-9


def test_spin_in_place():
    o = make()
    o.update(0, 0, 0, 0, 0.0)
    ticks = int(600 * (math.pi * 0.2 / 2) / (math.pi * 0.1))   # half-circle of the track
    o.update(-ticks, ticks, -ticks, ticks, 1.0)
    assert abs(abs(o.theta) - math.pi / 2) < 0.02 or abs(abs(o.theta) - math.pi) < 0.05
    assert abs(o.x) < 1e-9


def test_pwm():
    assert twist_to_pwm(0.5, 0.0, 0.2, 0.5) == (255, 255)
    assert twist_to_pwm(0.0, 0.0, 0.2, 0.5) == (0, 0)
    l, r = twist_to_pwm(0.0, 1.0, 0.2, 0.5)
    assert l < 0 < r


def test_pwm_cap():
    assert twist_to_pwm(2.0, 0.0, 0.2, 1.4, 130) == (130, 130)
    assert twist_to_pwm(-2.0, 0.0, 0.2, 1.4, 130) == (-130, -130)


def test_pwm_keeps_steering_at_limit():
    l, r = twist_to_pwm(0.5, -0.6, 0.22, 0.47, 150)
    assert l == 150 and r < 120          # correction survives even at full speed


def test_parse():
    t = parse_telemetry('D,1000,0.01,0.02,0.98,0.1,0.2,0.3,1.5,-0.5,10.2,100,-100,98,-97')
    assert t.enc_fl == 100 and t.enc_rr == -97
    assert parse_telemetry(',0.00,0.00,0.00,0,0,0,0') is None


def test_imu_health():
    dead = parse_telemetry('D,5,0.000,0.000,0.000,0.00,0.00,0.00,0.00,0.00,0.00,0,0,0,0')
    live = parse_telemetry('D,5,0.010,0.020,0.980,0.10,0.20,0.30,1.50,-0.50,0.00,0,0,0,0')
    assert not imu_healthy(dead) and imu_healthy(live)


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
