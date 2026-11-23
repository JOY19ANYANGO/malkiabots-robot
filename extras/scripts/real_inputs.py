#!/usr/bin/env python3
"""Real inputs for the mission (replaces fake_inputs). Run ON THE PI:
    python3 ~/real_inputs.py                       # gate from the lidar, loading = fixed 5 s wait
    python3 ~/real_inputs.py --load-wait 8         # other loading time

What it does
  /gate_open (Bool, 5 Hz): TRUE when the lidar sees the path-2 gate opening EMPTY. The lidar points are put on
      the map (AMCL position) and counted inside the gate box. Checked only when the robot is within 1.5 m of
      the gate, so a far-away or hidden gate never counts as "open".
  /loading/done (Bool): sent load_wait seconds after the mission asks for loading (/loading/request).
  The waste type is NOT faked any more: it comes from the camera detector (/waste_type).
"""
import argparse, math, threading, time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

# gate opening on gamefield_nopillars (from the field CAD: gate x 1.534..1.578, y -1.427..-0.866), slightly shrunk
# so the side walls never fall inside
GATE_X = (1.46, 1.65)
GATE_Y = (-1.37, -0.92)
GATE_C = (1.556, -1.146)


class RealInputs(Node):
    def __init__(self, load_wait, closed_points, view_dist):
        super().__init__("real_inputs")
        self.load_wait, self.closed_points, self.view_dist = load_wait, closed_points, view_dist
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.gate_pub = self.create_publisher(Bool, "/gate_open", 10)
        self.done_pub = self.create_publisher(Bool, "/loading/done", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(String, "/loading/request", self.on_request, 10)
        self.gate_open, self.last_state, self.history = False, None, []
        self.create_timer(0.2, lambda: self.gate_pub.publish(Bool(data=self.gate_open)))
        self.get_logger().info(f"real inputs: gate from lidar, loading wait {load_wait:.1f} s")

    def on_request(self, msg):
        self.get_logger().info(f"loading request for '{msg.data}' -> done in {self.load_wait:.1f} s")
        def later():
            time.sleep(self.load_wait)
            self.done_pub.publish(Bool(data=True)); self.get_logger().info("loading done")
        threading.Thread(target=later, daemon=True).start()

    def on_scan(self, scan):
        try:
            t = self.tf.lookup_transform("map", scan.header.frame_id, Time())
        except Exception:
            return                                            # no map position yet (AMCL not ready)
        tx, ty = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if math.hypot(tx - GATE_C[0], ty - GATE_C[1]) > self.view_dist:
            self.gate_open, self.history = False, []         # too far to judge: treat as closed
            return
        c, s = math.cos(yaw), math.sin(yaw)
        n = 0
        a = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                lx, ly = r * math.cos(a), r * math.sin(a)
                x, y = tx + c * lx - s * ly, ty + s * lx + c * ly
                if GATE_X[0] <= x <= GATE_X[1] and GATE_Y[0] <= y <= GATE_Y[1]:
                    n += 1
            a += scan.angle_increment
        self.history = (self.history + [n])[-3:]              # last 3 scans (~0.5 s)
        if len(self.history) == 3:
            if all(k < self.closed_points for k in self.history):
                self.gate_open = True
            elif all(k >= self.closed_points for k in self.history):
                self.gate_open = False
        if self.gate_open != self.last_state:
            self.last_state = self.gate_open
            self.get_logger().info(f"gate {'OPEN' if self.gate_open else 'CLOSED'} ({n} lidar points in the gate)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--load-wait", type=float, default=5.0, help="seconds to wait at the loading station")
    ap.add_argument("--closed-points", type=int, default=6, help="lidar points in the gate box that mean CLOSED")
    ap.add_argument("--view-dist", type=float, default=1.5, help="only judge the gate within this distance (m)")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = RealInputs(a.load_wait, a.closed_points, a.view_dist)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
