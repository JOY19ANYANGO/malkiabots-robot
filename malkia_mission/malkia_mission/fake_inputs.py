#!/usr/bin/env python3
"""Stand-in for the camera, gate detector and loading station, so the mission tree can run end to end
in Gazebo (or on the real robot before those nodes exist).

    ros2 run malkia_mission fake_inputs                                          # metal, gate open, load takes 3 s
    ros2 run malkia_mission fake_inputs --ros-args -p waste_type:=plastic -p gate:=closed
    ros2 run malkia_mission fake_inputs --ros-args -p gate:=2.0                  # gate opens 2 s after the robot
                                                                                 #   reaches the junction
    ros2 run malkia_mission fake_inputs --ros-args -p waste_type:=none           # screen unreadable -> default type

gate: "open" | "closed" | <seconds after the robot reaches the junction>
      (the mission logs `junction_reached` on /mission/events; the timer starts there).
noise: share of wrong /waste_type labels (0..1).
"""
import json
import random
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

TYPES = ["plastic", "metal", "organic"]
EVENTS_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=100)


class FakeInputs(Node):
    def __init__(self):
        super().__init__("fake_inputs")
        self.waste = self.declare_parameter("waste_type", "metal").value
        self.gate = str(self.declare_parameter("gate", "open", ParameterDescriptor(dynamic_typing=True)).value)
        self.load_s = float(self.declare_parameter("load_delay_s", 3.0).value)
        self.noise = float(self.declare_parameter("noise", 0.1).value)
        self.waste_pub = self.create_publisher(String, "/waste_type", 10)
        self.gate_pub = self.create_publisher(Bool, "/gate_open", 10)
        self.done_pub = self.create_publisher(Bool, "/loading/done", 10)
        self.create_subscription(String, "/loading/request", self.on_request, 10)
        self.create_subscription(String, "/mission/events", self.on_event, EVENTS_QOS)
        self.at_junction = None
        self.pending_load = None
        self.gate_announced = False
        self.create_timer(0.1, self.tick)                 # camera at 10 Hz, gate detector at 10 Hz
        self.get_logger().info(f"fake inputs: waste={self.waste} gate={self.gate} load={self.load_s}s noise={self.noise}")

    def on_request(self, msg):
        self.get_logger().info(f"loading request for '{msg.data}' -> done in {self.load_s:.1f} s")
        self.pending_load = time.time() + self.load_s

    def on_event(self, msg):
        try:
            ev = json.loads(msg.data).get("event")
        except (ValueError, AttributeError):
            return
        if ev == "autonomous_start":                     # a new mission run: reset the gate timer
            self.at_junction, self.gate_announced = None, False
        elif ev == "junction_reached" and self.at_junction is None:
            self.at_junction = time.time()
            if self.gate not in ("open", "closed"):
                self.get_logger().info(f"robot at the junction - gate opens in {self.gate} s")

    def gate_is_open(self):
        if self.gate == "open":
            return True
        if self.gate == "closed":
            return False
        try:
            delay = float(self.gate)
        except ValueError:
            return False
        is_open = self.at_junction is not None and time.time() - self.at_junction >= delay
        if is_open and not self.gate_announced:
            self.gate_announced = True
            self.get_logger().info("gate OPEN")
        return is_open

    def tick(self):
        if self.waste in TYPES:
            label = self.waste if random.random() >= self.noise else random.choice([t for t in TYPES if t != self.waste])
            self.waste_pub.publish(String(data=label))
        self.gate_pub.publish(Bool(data=self.gate_is_open()))
        if self.pending_load and time.time() >= self.pending_load:
            self.pending_load = None
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info("loading done")


def main():
    rclpy.init()
    node = FakeInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
