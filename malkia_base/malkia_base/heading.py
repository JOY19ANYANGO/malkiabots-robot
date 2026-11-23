"""Gyro calibration, heading hold and drift diagnosis (pure Python, no ROS)."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GyroBias:
    """Learns the gyro Z offset while the robot is standing still.

    First it needs `calib_time` seconds of stillness (startup calibration).
    After that it keeps slowly re-learning the offset every time the robot
    stands still, because the MPU6050 offset changes as it warms up.
    """

    def __init__(self, calib_time=3.0, slow_time=20.0):
        self.calib_time = calib_time
        self.slow_time = slow_time
        self.bias = 0.0            # deg/s
        self.ready = False
        self._sum = 0.0
        self._t = 0.0

    def update(self, gz, dt, still):
        if dt <= 0:
            return False
        if not self.ready:
            if not still:              # moved during calibration: start again
                self._sum = self._t = 0.0
                return False
            self._sum += gz * dt
            self._t += dt
            if self._t >= self.calib_time:
                self.bias = self._sum / self._t
                self.ready = True
                return True            # just finished
            return False
        if still:
            a = min(1.0, dt / self.slow_time)
            self.bias += a * (gz - self.bias)
        return False


class HeadingHold:
    """When driving straight (v != 0, w == 0), steer to keep the heading
    the robot had when the straight drive started."""

    def __init__(self, kp=2.5, ki=1.5, max_w=0.6, dt=0.1):
        self.kp = kp
        self.ki = ki            # removes the small error left by a weaker motor side
        self.max_w = max_w
        self.dt = dt            # how often step() is called (send_motors timer)
        self.target = None
        self.integral = 0.0

    def step(self, v, w, yaw):
        if abs(v) > 0.02 and abs(w) < 0.02:
            if self.target is None:
                self.target = yaw
                self.integral = 0.0
            err = wrap(self.target - yaw)
            self.integral += err * self.dt
            lim = self.max_w / self.ki if self.ki > 0 else 0.0
            self.integral = max(-lim, min(lim, self.integral))   # anti-windup
            out = self.kp * err + self.ki * self.integral
            return max(-self.max_w, min(self.max_w, out))
        self.target = None
        return w


class DriftMonitor:
    """Watches teleop driving and explains drift in plain words.

    A segment is a stretch of time with the same kind of command
    (straight or turning). When a segment ends, `finish()` returns a report.
    """

    def __init__(self, track, m_per_tick):
        self.track = track
        self.m_per_tick = m_per_tick
        self._kind = None
        self._reset()

    def _reset(self):
        self.t = 0.0
        self.gyro_turn = 0.0       # rad
        self.wheel_turn = 0.0      # rad
        self.dl = 0.0              # m
        self.dr = 0.0

    def update(self, kind, gz_rad, d_fl, d_fr, d_rl, d_rr, dt):
        """kind: 'straight', 'turn' or None (stopped). Returns a report or None."""
        report = None
        if kind != self._kind:
            report = self.finish()
            self._kind = kind
            self._reset()
        if kind is None or dt <= 0:
            return report
        dl = (d_fl + d_rl) / 2.0 * self.m_per_tick
        dr = (d_fr + d_rr) / 2.0 * self.m_per_tick
        self.t += dt
        self.dl += dl
        self.dr += dr
        self.gyro_turn += gz_rad * dt
        self.wheel_turn += (dr - dl) / self.track
        return report

    def finish(self):
        if self._kind is None or self.t < 1.0:
            return None
        g = math.degrees(self.gyro_turn)
        wh = math.degrees(self.wheel_turn)
        if self._kind == 'turn':
            if abs(g) > 15 and abs(wh) > 15 and (g > 0) != (wh > 0):
                return ('turn',
                        f"TURN CHECK: gyro says {g:+.0f} deg but wheels say {wh:+.0f} deg. "
                        "Opposite signs: the gyro Z sign is REVERSED -> set gyro_z_sign: -1.0")
            if abs(g) > 15:
                return ('turn', f"turn check OK: gyro {g:+.0f} deg, wheels {wh:+.0f} deg (same direction)")
            return None
        dist = (self.dl + self.dr) / 2.0
        if abs(dist) < 0.2:
            return None
        lines = [f"STRAIGHT {self.t:.1f} s, {dist:.2f} m: measured turn {g:+.1f} deg, "
                 f"wheel turn {wh:+.1f} deg (+ = left)"]
        if self.dl != 0:
            ratio = self.dr / self.dl
            lines.append(f"  left wheels {self.dl:.2f} m, right wheels {self.dr:.2f} m "
                         f"(right/left = {ratio:.3f})")
        if abs(g) <= 3:
            lines.append("  -> robot held its heading (straight)")
        else:
            side = "LEFT" if g > 0 else "RIGHT"
            lines.append(f"  -> robot really turned {side} by {abs(g):.0f} deg")
        return ('straight', "\n".join(lines))
