#!/usr/bin/env python3
"""cramped_robot base driver — the ROS 2 side of the physical robot.

Talks to the motor microcontroller (firmware/base_firmware.ino) over USB serial:
    PC -> MCU : "v <left_rad_s> <right_rad_s>\n"        wheel angular-velocity targets
    MCU -> PC : "e <lt> <rt> <lv> <rv>\n"  @ 50 Hz       encoder ticks (cumulative) and measured rad/s
Subscribes /cmd_vel (geometry_msgs/Twist), publishes /odom, /joint_states and (optionally) odom->base_footprint TF.
Skid-steer is driven as a differential pair: both left wheels get the left speed, both right wheels the right speed.

Parameters: port, baud, wheel_radius, wheel_separation, skid_factor, ticks_per_rev, max_wheel_rad_s, publish_tf, cmd_timeout
"""
import math, threading, time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:                                   # sudo apt install python3-serial
    serial = None


class BaseDriver(Node):
    def __init__(self):
        super().__init__("base_driver")
        p = self.declare_parameters("", [
            ("port", "/dev/ttyUSB0"), ("baud", 115200),
            ("wheel_radius", 0.0406),                 # m   (from CAD)
            ("wheel_separation", 0.216),              # m   (track, from CAD)
            ("skid_factor", 1.25),                    # effective track = wheel_separation * skid_factor (skid-steer slip); calibrate!
            ("ticks_per_rev", 1320.0),                # encoder ticks per WHEEL revolution (encoder CPR x gear ratio); calibrate!
            ("max_wheel_rad_s", 10.5),                # ~100 rpm gearmotor
            ("publish_tf", True),                     # false when robot_localization (EKF) publishes odom->base_footprint
            ("cmd_timeout", 0.5),
            ("odom_frame", "odom"), ("base_frame", "base_footprint"),
        ])
        g = lambda n: self.get_parameter(n).value
        self.r, self.sep = g("wheel_radius"), g("wheel_separation") * g("skid_factor")
        self.tpr, self.vmax = g("ticks_per_rev"), g("max_wheel_rad_s")
        self.publish_tf, self.cmd_timeout = g("publish_tf"), g("cmd_timeout")
        self.odom_frame, self.base_frame = g("odom_frame"), g("base_frame")

        self.ser = None
        if serial is not None:
            try:
                self.ser = serial.Serial(g("port"), g("baud"), timeout=0.05)
                self.get_logger().info(f"serial open on {g('port')}")
            except Exception as e:                    # keep running so you can test the ROS side without hardware
                self.get_logger().error(f"serial open failed ({e}); running without hardware")

        self.x = self.y = self.yaw = 0.0
        self.last_ticks = None
        self.wheel_pos = [0.0, 0.0]                   # left, right (rad)
        self.wheel_vel = [0.0, 0.0]
        self.last_cmd_time = time.time()
        self.target = (0.0, 0.0)

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.js_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_timer(0.02, self.send_cmd)        # 50 Hz command / watchdog
        threading.Thread(target=self.read_loop, daemon=True).start()

    # ---------------- cmd_vel -> wheel speeds ----------------
    def on_cmd(self, msg: Twist):
        v, w = msg.linear.x, msg.angular.z
        left = (v - w * self.sep / 2.0) / self.r
        right = (v + w * self.sep / 2.0) / self.r
        scale = max(1.0, abs(left) / self.vmax, abs(right) / self.vmax)
        self.target = (left / scale, right / scale)
        self.last_cmd_time = time.time()

    def send_cmd(self):
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.target = (0.0, 0.0)                  # watchdog: stop if cmd_vel goes silent
        if self.ser:
            try:
                self.ser.write(f"v {self.target[0]:.3f} {self.target[1]:.3f}\n".encode())
            except Exception as e:
                self.get_logger().warn(f"serial write failed: {e}", throttle_duration_sec=5.0)

    # ---------------- encoders -> odometry ----------------
    def read_loop(self):
        while rclpy.ok():
            if not self.ser:
                time.sleep(0.1); continue
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            if not line.startswith("e "):
                continue
            try:
                _, lt, rt, lv, rv = line.split()
                self.integrate(int(lt), int(rt), float(lv), float(rv))
            except ValueError:
                pass

    def integrate(self, lt, rt, lv, rv):
        now = self.get_clock().now()
        if self.last_ticks is None:
            self.last_ticks = (lt, rt, now); return
        dl = (lt - self.last_ticks[0]) / self.tpr * 2 * math.pi * self.r
        dr = (rt - self.last_ticks[1]) / self.tpr * 2 * math.pi * self.r
        dt = (now - self.last_ticks[2]).nanoseconds * 1e-9
        self.last_ticks = (lt, rt, now)
        if dt <= 0: return
        ds, dyaw = (dl + dr) / 2.0, (dr - dl) / self.sep
        self.x += ds * math.cos(self.yaw + dyaw / 2.0)
        self.y += ds * math.sin(self.yaw + dyaw / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        self.wheel_pos = [self.wheel_pos[0] + dl / self.r, self.wheel_pos[1] + dr / self.r]
        self.wheel_vel = [lv, rv]
        self.publish(now, ds / dt, dyaw / dt)

    def publish(self, stamp, v, w):
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        od = Odometry()
        od.header.stamp = stamp.to_msg(); od.header.frame_id = self.odom_frame; od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.angular.z = v, w
        od.pose.covariance[0] = od.pose.covariance[7] = 0.01; od.pose.covariance[35] = 0.05
        od.twist.covariance[0] = 0.01; od.twist.covariance[35] = 0.05
        self.odom_pub.publish(od)
        js = JointState(); js.header.stamp = od.header.stamp
        js.name = ["front_left_wheel_joint", "rear_left_wheel_joint", "front_right_wheel_joint", "rear_right_wheel_joint"]
        js.position = [self.wheel_pos[0], self.wheel_pos[0], self.wheel_pos[1], self.wheel_pos[1]]
        js.velocity = [self.wheel_vel[0], self.wheel_vel[0], self.wheel_vel[1], self.wheel_vel[1]]
        self.js_pub.publish(js)
        if self.publish_tf:
            t = TransformStamped(); t.header = od.header; t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf_pub.sendTransform(t)


def main():
    rclpy.init()
    node = BaseDriver()
    try:
        rclpy.spin(node)
    finally:
        if node.ser:
            node.ser.write(b"v 0 0\n")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
