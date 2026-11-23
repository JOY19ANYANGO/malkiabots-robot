import math


class WheelOdometry:
    """Differential-drive odometry for a 4WD skid-steer robot.

    Left speed is the mean of FL and RL, right speed is the mean of FR and RR.
    """

    def __init__(self, ticks_per_rev, wheel_diameter, track_width):
        self.m_per_tick = math.pi * wheel_diameter / ticks_per_rev
        self.track = track_width
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self._prev = None

    def update(self, fl, fr, rl, rr, dt):
        if self._prev is None:
            self._prev = (fl, fr, rl, rr)
            return
        pfl, pfr, prl, prr = self._prev
        self._prev = (fl, fr, rl, rr)

        dl = ((fl - pfl) + (rl - prl)) / 2.0 * self.m_per_tick
        dr = ((fr - pfr) + (rr - prr)) / 2.0 * self.m_per_tick
        ds = (dl + dr) / 2.0
        dth = (dr - dl) / self.track

        mid = self.theta + dth / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.theta = math.atan2(math.sin(self.theta + dth), math.cos(self.theta + dth))

        if dt > 0:
            self.v = ds / dt
            self.w = dth / dt

    def reset_counts(self):
        """Call when the Nano encoder counters were zeroed."""
        self._prev = None


def twist_to_pwm(v, w, track_width, max_wheel_speed, max_pwm=255):
    """cmd_vel (m/s, rad/s) -> (left, right) PWM, limited to +-max_pwm."""
    left = v - w * track_width / 2.0
    right = v + w * track_width / 2.0
    scale = 255.0 / max_wheel_speed
    left, right = left * scale, right * scale
    # If one side is over the limit, slow BOTH sides by the same factor, so the
    # difference between them (the steering) is kept instead of being cut off.
    big = max(abs(left), abs(right))
    if big > max_pwm:
        left, right = left * max_pwm / big, right * max_pwm / big
    clamp = lambda x: int(max(-max_pwm, min(max_pwm, round(x))))
    return clamp(left), clamp(right)
