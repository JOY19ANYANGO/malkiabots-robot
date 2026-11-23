import math
import time

import rclpy
import serial
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int16
from tf2_ros import TransformBroadcaster

from .heading import DriftMonitor, GyroBias, HeadingHold
from .odometry import WheelOdometry, twist_to_pwm
from .protocol import imu_healthy, parse_telemetry

G = 9.80665


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class BaseNode(Node):
    def __init__(self):
        super().__init__('base_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_diameter', 0.065)
        self.declare_parameter('track_width', 0.22)
        self.declare_parameter('ticks_per_rev', 1980.0)
        self.declare_parameter('max_wheel_speed', 0.47)
        self.declare_parameter('max_pwm', 150)
        self.declare_parameter('heading_gain', 0.0)     # OLD heading hold on the Nano, 0 = off
        self.declare_parameter('ros_heading_hold', True)  # NEW heading hold here, with gyro calibration
        self.declare_parameter('heading_kp', 2.5)       # rad/s of correction per rad of heading error
        self.declare_parameter('heading_ki', 1.5)       # removes the leftover error of a weaker side
        self.declare_parameter('gyro_calib_time', 3.0)  # s the robot must stand still at startup
        self.declare_parameter('gyro_z_sign', 1.0)      # set -1.0 if the gyro turns the wrong way
        self.declare_parameter('left_trim', 1.0)        # PWM multiplier for the left motors
        self.declare_parameter('right_trim', 1.0)       # PWM multiplier for the right motors
        self.declare_parameter('nano_split_fl', False)  # False = one speed loop per side (sends F 0)
        self.declare_parameter('drift_report', True)    # print a drift report after each teleop move
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')

        p = self.get_parameter
        self.track = p('track_width').value
        self.max_speed = p('max_wheel_speed').value
        self.max_pwm = int(p('max_pwm').value)
        self.heading_gain = float(p('heading_gain').value)
        self.ros_hold = bool(p('ros_heading_hold').value)
        self.gyro_sign = float(p('gyro_z_sign').value)
        self.left_trim = float(p('left_trim').value)
        self.right_trim = float(p('right_trim').value)
        self.drift_report = bool(p('drift_report').value)
        self.split_fl = bool(p('nano_split_fl').value)
        if self.ros_hold and self.heading_gain > 0:
            self.get_logger().warn('ros_heading_hold is on, so the Nano heading hold is switched off '
                                   '(two heading holds would fight each other)')
            self.heading_gain = 0.0
        self.cmd_timeout = p('cmd_timeout').value
        self.publish_imu = p('publish_imu').value
        self.publish_tf = p('publish_tf').value
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value
        self.imu_frame = p('imu_frame').value

        self.odom = WheelOdometry(p('ticks_per_rev').value,
                                  p('wheel_diameter').value, self.track)
        self.gyro = GyroBias(float(p('gyro_calib_time').value))
        self.hold = HeadingHold(float(p('heading_kp').value), float(p('heading_ki').value))
        self.monitor = DriftMonitor(self.track, self.odom.m_per_tick)
        self.yaw_imu = 0.0            # rad, from the calibrated gyro
        self._prev_enc = None
        self._calib_msg_t = 0.0
        self._imu_last = None          # last raw IMU values, to spot a frozen IMU
        self._imu_same_t = 0.0
        self._imu_frozen = False
        self._last_tel_wall = None
        self._status_t = 0.0
        self._last_pwm = (0, 0)
        self._hold_src = None          # 'gyro' or 'wheels': what the heading hold steers by

        self.port = p('port').value
        self.baud = p('baud').value
        self.ser = None
        self._buf = b''
        self._open_port()
        self._last_t_ms = None
        self._cmd = (0.0, 0.0)
        self._imu_ok = None
        self._last_cmd_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Bool, 'gate', self.on_gate, 10)            # true = open, false = close
        self.create_subscription(Int16, 'gate_angle', self.on_gate_angle, 10)  # 0..180 degrees

        self.create_timer(0.01, self.read_serial)
        self.create_timer(0.1, self.send_motors)
        self.create_timer(1.0, self.reconnect)
        self.get_logger().info(f"Bridge running on {self.port}")

    # ---- serial port handling ----
    def _open_port(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn(f"cannot open {self.port}: {e}")
            return
        time.sleep(2.0)                      # the Nano resets when the port opens
        self.ser.reset_input_buffer()
        self._buf = b''
        self._last_t_ms = None
        self.odom.reset_counts()             # the Nano counters restart from zero
        self._send_line(f"H {self.heading_gain}")   # the Nano forgets this at every reset
        # One speed loop for the whole left side, like the right side. With two loops
        # (F 1) the front-left and rear-left loops fight and the front-left motor is
        # slowly switched off.
        self._send_line(f"F {1 if self.split_fl else 0}")
        self.get_logger().info(f"Nano heading hold {self.heading_gain}, ROS heading hold "
                               f"{'ON' if self.ros_hold else 'OFF'}, trim L {self.left_trim} R {self.right_trim}, "
                               f"left side speed loops: {'2 (F 1)' if self.split_fl else '1 (F 0)'}")
        self._prev_enc = None

    def _lost(self, err):
        if self.ser is not None:
            self.get_logger().error(f"serial lost ({err}); will try to reconnect")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def reconnect(self):
        if self.ser is None:
            self._open_port()
            if self.ser is not None:
                self.get_logger().info(f"reconnected to {self.port}")

    # ---- commands to the Nano ----
    def on_cmd_vel(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z)
        self._last_cmd_time = self.get_clock().now()

    def _send_line(self, text):
        if self.ser is None:
            return
        try:
            self.ser.write((text + "\n").encode())
        except (serial.SerialException, OSError) as e:
            self._lost(e)

    def on_gate(self, msg):
        self._send_line("O" if msg.data else "C")
        self.get_logger().info("gate " + ("open" if msg.data else "closed"))

    def on_gate_angle(self, msg):
        angle = max(0, min(180, int(msg.data)))
        self._send_line(f"A {angle}")
        self.get_logger().info(f"gate angle {angle}")

    def send_motors(self):
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        v, w = self._cmd if age < self.cmd_timeout else (0.0, 0.0)
        if self._last_tel_wall is not None and time.time() - self._last_tel_wall > 0.5 \
                and (v != 0.0 or w != 0.0):
            self.get_logger().warn('NO DATA from the Nano for '
                                   f'{time.time() - self._last_tel_wall:.1f} s (Nano frozen or USB problem)',
                                   throttle_duration_sec=1.0)
        if self.ros_hold and self._last_tel_wall is not None:
            yaw, src = self._heading()
            if src != self._hold_src:
                self.hold.target = None          # restart the hold on the new heading source
                self._hold_src = src
                if src == 'gyro':
                    self.get_logger().info('Heading hold uses the GYRO')
                else:
                    self.get_logger().warn('Heading hold uses the WHEEL ENCODERS (IMU not working). '
                                           'Keeps the sides equal but cannot see wheel slip.')
            w = self.hold.step(v, w, yaw)
        left, right = twist_to_pwm(v, w, self.track, self.max_speed, self.max_pwm)
        lim = lambda x: int(max(-self.max_pwm, min(self.max_pwm, round(x))))
        left, right = lim(left * self.left_trim), lim(right * self.right_trim)
        self._last_pwm = (left, right)
        line = "X\n" if left == 0 and right == 0 else f"M {left} {right}\n"
        if self.ser is None:
            return
        try:
            self.ser.write(line.encode())
        except (serial.SerialException, OSError) as e:
            self._lost(e)

    def _heading(self):
        """Heading for the heading hold: the gyro when it works, otherwise the
        wheel encoders (keeps left and right distance equal; cannot see slip)."""
        if self._imu_ok and self.gyro.ready and not self._imu_frozen:
            return self.yaw_imu, 'gyro'
        return self.odom.theta, 'wheels'

    # ---- telemetry from the Nano ----
    def read_serial(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self._lost(e)
            return
        if not data:
            return
        self._buf += data
        while b'\n' in self._buf:
            raw, self._buf = self._buf.split(b'\n', 1)
            text = raw.decode(errors='replace').strip()
            tel = parse_telemetry(text)
            if tel:
                self.on_telemetry(tel)
            elif text and not text.startswith('OK') and not text.startswith('D,'):
                # Messages from the Nano itself, e.g. 'ERR stall left' or 'MPU6050 OK'
                if text.startswith('ERR'):
                    self.get_logger().warn(f"[Nano] {text}")
                else:
                    if text != getattr(self, "_last_nano_msg", None):   # print repeats only once
                        self.get_logger().info(f"[Nano] {text}")
                    self._last_nano_msg = text

    def on_telemetry(self, t):
        now = self.get_clock().now().to_msg()
        dt = 0.0 if self._last_t_ms is None else (t.t_ms - self._last_t_ms) * 1e-3
        if dt < 0:                           # the Nano restarted
            self.odom.reset_counts()
            dt = 0.0
        self._last_t_ms = t.t_ms
        self._last_tel_wall = time.time()

        self.odom.update(t.enc_fl, t.enc_fr, t.enc_rl, t.enc_rr, dt)
        o = self.odom

        # ---- gyro calibration + drift diagnosis ----
        enc = (t.enc_fl, t.enc_fr, t.enc_rl, t.enc_rr)
        d = (0, 0, 0, 0) if self._prev_enc is None or dt == 0 else \
            tuple(a - b for a, b in zip(enc, self._prev_enc))
        self._prev_enc = enc
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        v_cmd, w_cmd = self._cmd if age < self.cmd_timeout else (0.0, 0.0)
        still = d == (0, 0, 0, 0) and v_cmd == 0.0 and w_cmd == 0.0
        gz = self.gyro_sign * t.gz                               # deg/s

        # A frozen IMU (I2C hang from motor noise) keeps sending exactly the same numbers.
        raw = (t.ax, t.ay, t.az, t.gx, t.gy, t.gz)
        moving = d != (0, 0, 0, 0)
        if raw == self._imu_last and moving and imu_healthy(t):   # all zeros = no IMU, not frozen
            self._imu_same_t += dt
        else:
            self._imu_same_t = 0.0
        self._imu_last = raw
        frozen = self._imu_same_t > 1.0
        if frozen != self._imu_frozen:
            self._imu_frozen = frozen
            if frozen:
                self.get_logger().error('IMU FROZEN: the gyro values stopped changing while driving '
                                        '(I2C hang, usually motor noise). Heading hold paused.')
            else:
                self.get_logger().info('IMU values changing again, heading hold back on')
        if self.gyro.update(gz, dt, still):
            self.get_logger().info(f"Gyro calibrated: offset {self.gyro.bias:+.3f} deg/s "
                                   f"(the old code would have drifted {self.gyro.bias * 60:+.0f} deg per minute)")
        elif not self.gyro.ready and imu_healthy(t) and time.time() - self._calib_msg_t > 2.0:
            self._calib_msg_t = time.time()
            self.get_logger().info('Calibrating gyro: keep the robot STILL...')
        gz_c = gz - self.gyro.bias if self.gyro.ready else 0.0  # deg/s, calibrated
        self.yaw_imu = math.atan2(math.sin(self.yaw_imu + math.radians(gz_c) * dt),
                                  math.cos(self.yaw_imu + math.radians(gz_c) * dt))
        if self.drift_report and self.gyro.ready:
            kind = 'straight' if abs(v_cmd) > 0.02 and abs(w_cmd) < 0.02 else \
                   'turn' if abs(w_cmd) > 0.2 else None
            hyaw, hsrc = self._heading()
            if kind == 'straight' and time.time() - self._status_t > 1.0:
                self._status_t = time.time()
                err = 0.0 if self.hold.target is None else \
                    math.degrees(math.atan2(math.sin(self.hold.target - hyaw),
                                            math.cos(self.hold.target - hyaw)))
                self.get_logger().info(
                    f"live [{hsrc}]: heading error {err:+5.1f} deg | gyro {gz_c:+6.2f} deg/s | "
                    f"PWM L {self._last_pwm[0]:4d} R {self._last_pwm[1]:4d} | "
                    f"ticks/msg L {(d[0] + d[2]) / 2:+5.1f} R {(d[1] + d[3]) / 2:+5.1f}")
            if hsrc == 'wheels' and kind == 'turn':
                kind = None                      # no gyro: nothing to compare a turn with
            rate = math.radians(gz_c) if hsrc == 'gyro' else self.odom.w
            rep = self.monitor.update(kind, rate, *d, dt)
            if rep:
                if 'REVERSED' in rep[1]:
                    self.get_logger().warn(rep[1])
                else:
                    self.get_logger().info(rep[1])

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = o.x
        msg.pose.pose.position.y = o.y
        msg.pose.pose.orientation = yaw_to_quat(o.theta)
        msg.twist.twist.linear.x = o.v
        msg.twist.twist.angular.z = o.w
        # Wheel odometry drifts, especially in yaw while skid-steering.
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.1
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 0.1
        self.odom_pub.publish(msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = o.x
            tf.transform.translation.y = o.y
            tf.transform.rotation = yaw_to_quat(o.theta)
            self.tf_pub.sendTransform(tf)

        if self.publish_imu:
            ok = imu_healthy(t)
            if ok != self._imu_ok:
                self._imu_ok = ok
                if ok:
                    self.get_logger().info('IMU detected, publishing /imu/data')
                else:
                    self.get_logger().warn('IMU not detected (all zeros), not publishing /imu/data')
        if self.publish_imu and self._imu_ok and self.gyro.ready:
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = self.imu_frame
            imu.orientation = rpy_to_quat(math.radians(t.roll),
                                          math.radians(t.pitch),
                                          self.yaw_imu)
            imu.angular_velocity.x = math.radians(t.gx)
            imu.angular_velocity.y = math.radians(t.gy)
            imu.angular_velocity.z = math.radians(gz_c)
            imu.linear_acceleration.x = t.ax * G
            imu.linear_acceleration.y = t.ay * G
            imu.linear_acceleration.z = t.az * G
            imu.orientation_covariance[0] = 0.02
            imu.orientation_covariance[4] = 0.02
            imu.orientation_covariance[8] = 0.05
            imu.angular_velocity_covariance[0] = 0.01
            imu.angular_velocity_covariance[4] = 0.01
            imu.angular_velocity_covariance[8] = 0.01
            imu.linear_acceleration_covariance[0] = 0.1
            imu.linear_acceleration_covariance[4] = 0.1
            imu.linear_acceleration_covariance[8] = 0.1
            self.imu_pub.publish(imu)

    def destroy_node(self):
        try:
            self.ser.write(b"X\n")
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
