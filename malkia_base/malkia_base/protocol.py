from dataclasses import dataclass


@dataclass
class Telemetry:
    t_ms: int
    ax: float; ay: float; az: float          # g
    gx: float; gy: float; gz: float          # deg/s
    pitch: float; roll: float; yaw: float    # deg
    enc_fl: int; enc_fr: int; enc_rl: int; enc_rr: int


def parse_telemetry(line):
    """'D,ms,ax,ay,az,gx,gy,gz,pitch,roll,yaw,fl,fr,rl,rr' -> Telemetry or None."""
    p = line.split(",")
    if len(p) != 15 or p[0] != "D":
        return None
    try:
        f = [float(v) for v in p[2:11]]
        e = [int(v) for v in p[11:15]]
        return Telemetry(int(p[1]), *f, *e)
    except ValueError:
        return None


def imu_healthy(t):
    """The Nano reports exact zeros when it cannot find the MPU6050.
    A working sensor always shows gravity on at least one accel axis."""
    return not (t.ax == 0.0 and t.ay == 0.0 and t.az == 0.0)
