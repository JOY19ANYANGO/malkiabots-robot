import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from malkia_base.heading import DriftMonitor, GyroBias, HeadingHold  # noqa: E402

DT = 0.05
TRACK = 0.22


def drive(seconds, bias_dps, left_strength, use_hold, calibrate, sign=1.0):
    """Simulated robot driving 'straight'. Returns the real heading change in degrees."""
    g = GyroBias(3.0)
    h = HeadingHold(2.5, dt=DT)
    true_yaw = 0.0
    est_yaw = 0.0
    if calibrate:
        for _ in range(int(3.5 / DT)):
            g.update(sign * bias_dps, DT, still=True)
    for _ in range(int(seconds / DT)):
        w = h.step(0.25, 0.0, est_yaw) if use_hold else 0.0
        vl = (0.25 - w * TRACK / 2) * left_strength
        vr = 0.25 + w * TRACK / 2
        true_w = (vr - vl) / TRACK
        true_yaw += true_w * DT
        gz = sign * (math.degrees(true_w) + bias_dps)
        est_yaw += math.radians(gz - g.bias) * DT
    return math.degrees(true_yaw)


def test_calibration_learns_bias():
    g = GyroBias(3.0)
    for _ in range(int(3.1 / DT)):
        g.update(-1.2, DT, still=True)
    assert g.ready and abs(g.bias + 1.2) < 1e-6


def test_calibration_restarts_if_moved():
    g = GyroBias(3.0)
    for _ in range(int(2.0 / DT)):
        g.update(-1.0, DT, still=True)
    g.update(30.0, DT, still=False)
    assert not g.ready
    for _ in range(int(3.1 / DT)):
        g.update(-1.0, DT, still=True)
    assert abs(g.bias + 1.0) < 1e-6


def test_weak_left_side_is_fixed():
    off = drive(6, 0.0, 0.93, use_hold=False, calibrate=True)
    on = drive(6, 0.0, 0.93, use_hold=True, calibrate=True)
    assert off > 10            # weak left side -> real turn to the left
    assert abs(on) < 1


def test_gyro_bias_no_longer_steers_robot():
    uncal = drive(6, -1.5, 1.0, use_hold=True, calibrate=False)
    cal = drive(6, -1.5, 1.0, use_hold=True, calibrate=True)
    assert uncal > 5           # uncalibrated hold follows the false rotation -> pulls left
    assert abs(cal) < 1


def test_monitor_reports_reversed_sign():
    m = DriftMonitor(TRACK, 0.0001)
    for _ in range(int(2 / DT)):          # robot turns left, gyro says right
        m.update('turn', -0.5, -20, 20, -20, 20, DT)
    rep = m.update(None, 0, 0, 0, 0, 0, DT)
    assert rep and 'REVERSED' in rep[1]


def test_monitor_straight_report():
    m = DriftMonitor(TRACK, 0.0001)
    for _ in range(int(3 / DT)):
        m.update('straight', 0.05, 120, 130, 120, 130, DT)
    rep = m.update(None, 0, 0, 0, 0, 0, DT)
    assert rep[0] == 'straight' and 'LEFT' in rep[1]


if __name__ == '__main__':
    print("weak left, no hold :", round(drive(6, 0.0, 0.93, False, True), 1), "deg")
    print("weak left, new hold:", round(drive(6, 0.0, 0.93, True, True), 1), "deg")
    print("gyro bias -1.5, hold WITHOUT calibration (old):", round(drive(6, -1.5, 1.0, True, False), 1), "deg")
    print("gyro bias -1.5, hold WITH calibration (new)   :", round(drive(6, -1.5, 1.0, True, True), 1), "deg")
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
