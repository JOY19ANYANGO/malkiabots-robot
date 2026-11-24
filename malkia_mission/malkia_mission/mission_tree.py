#!/usr/bin/env python3
"""Robotics Dojo 2026 autonomous-phase mission tree (py_trees_ros, structure in bt/mission_tree.xml).

    ros2 launch malkia_mission mission.launch.py                 # after bringup + navigation (AMCL localised)
    ros2 run malkia_mission mission_tree --ros-args -p config:=... -p waste_type_default:=metal
    ros2 run malkia_mission fake_inputs                          # simulation / dry run: fakes the sensors + station

Interfaces the tree relies on (other teams' nodes):
    /waste_type        std_msgs/String   "plastic" | "metal" | "organic"   <- camera/AI inference node (screen)
    /gate_open         std_msgs/Bool     true while the path-2 gate is open <- gate detector (camera or lidar)
    /loading/request   std_msgs/String   waste type, published by the tree  -> loading-station protocol node
    /loading/done      std_msgs/Bool     true when the item is on board     <- loading-station protocol node
    /gate              std_msgs/Bool     true=open the payload gate         -> base_node (servo)
    /speed_limit       nav2_msgs/SpeedLimit                                 -> Nav2 (slow on terrain)
    /mission/events    std_msgs/String   JSON timestamped events            -> telemetry uplink (reliable, transient local)
Nav2 actions: navigate_to_pose, navigate_through_poses. The tree never publishes /cmd_vel itself.

The mission runs ONCE: when the root finishes (success or failure) the tree logs mission_complete /
mission_failed and stops ticking. Restart the node for another run.
"""
import json
import math
import os
import time

import py_trees
import py_trees_ros
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateThroughPoses, NavigateToPose, Spin
from rclpy.action import ActionClient
from nav2_msgs.msg import CollisionMonitorState, SpeedLimit
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

BB = py_trees.blackboard.Blackboard
WASTE_TYPES = ("plastic", "metal", "organic")
EVENTS_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=100)


# ----------------------------------------------------------------------------- helpers
def world_to_map(x, y, yaw, origin):
    """mission.yaml poses are in WORLD (CAD / Gazebo) coordinates. A SLAM map's (0, 0) is wherever mapping
    started; `origin` is that start pose in world coordinates ((0, 0, 0) for the CAD map)."""
    ox, oy, oyaw = origin
    dx, dy = x - ox, y - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    return c * dx - s * dy, s * dx + c * dy, math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw))


def pose_from(d, node):
    x, y, yaw = world_to_map(float(d["x"]), float(d["y"]), float(d.get("yaw", 0.0)), Ctx.map_origin)
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class Ctx:
    """Shared node, config and the mission clock."""
    node = None
    cfg = None
    t0 = None
    events_pub = None
    robot_id = "MB01"
    map_origin = (0.0, 0.0, 0.0)     # world pose of the map's (0, 0, 0); CAD map = 0 0 0


def log_event(event, detail=None):
    """Required data (rulebook §8): timestamped event, published as JSON and appended to a CSV."""
    t = time.time()
    if event == "autonomous_start":
        Ctx.t0 = t
    rec = {"robot_id": Ctx.robot_id, "event": event, "t_rel": round(t - (Ctx.t0 or t), 1),
           "t_abs": round(t, 3), "detail": detail}
    Ctx.events_pub.publish(String(data=json.dumps(rec)))
    path = Ctx.cfg["log_file"]
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        new = not os.path.exists(path)
        with open(path, "a") as f:
            if new:
                f.write("robot_id,t_abs,t_rel,event,detail\n")
            f.write(f'{Ctx.robot_id},{rec["t_abs"]},{rec["t_rel"]},{event},{"" if detail is None else detail}\n')
    except OSError as e:                                   # a full disk must never stop the robot
        Ctx.node.get_logger().error(f"cannot write {path}: {e}")
    Ctx.node.get_logger().info(f"EVENT {event} @ {rec['t_rel']:.1f}s {'' if detail is None else detail}")


# ----------------------------------------------------------------------------- leaves
class LogEvent(py_trees.behaviour.Behaviour):
    def __init__(self, event, detail_key=None):
        super().__init__(name=f"Log[{event}]")
        self.event, self.detail_key = event, detail_key

    def update(self):
        detail = BB.get(self.detail_key) if self.detail_key and BB.exists(self.detail_key) else None
        log_event(self.event, detail)
        return py_trees.common.Status.SUCCESS


class SelectPose(py_trees.behaviour.Behaviour):
    """Pick a pose from a table keyed by the waste type (loading stations, bins)."""
    def __init__(self, table_key, key_key, out_key):
        super().__init__(name=f"SelectPose[{table_key}]")
        self.table_key, self.key_key, self.out_key = table_key, key_key, out_key

    def update(self):
        key = BB.get(self.key_key)
        table = Ctx.cfg[self.table_key]
        if key not in table:
            self.feedback_message = f"no pose for {key}"
            return py_trees.common.Status.FAILURE
        BB.set(self.out_key, table[key])
        return py_trees.common.Status.SUCCESS


class NavToPose(py_trees_ros.action_clients.FromBlackboard):
    """Nav2 NavigateToPose with the goal read from the blackboard (a dict x/y/yaw) or the config."""
    def __init__(self, name, pose_key=None, cfg_key=None, bt=None):
        super().__init__(name=name, action_type=NavigateToPose, action_name="navigate_to_pose",
                         key="_nav_goal", generate_feedback_message=lambda m: f"{m.feedback.distance_remaining:.2f} m")
        self.pose_key, self.cfg_key, self.bt = pose_key, cfg_key, bt

    def initialise(self):
        d = BB.get(self.pose_key) if self.pose_key else Ctx.cfg["poses"][self.cfg_key]
        goal = NavigateToPose.Goal()
        goal.pose = pose_from(d, Ctx.node)
        if self.bt:
            goal.behavior_tree = Ctx.cfg["bt_files"][self.bt]
        BB.set("_nav_goal", goal)
        super().initialise()


class NavThroughPoses(py_trees_ros.action_clients.FromBlackboard):
    def __init__(self, name, cfg_key, bt=None):
        super().__init__(name=name, action_type=NavigateThroughPoses, action_name="navigate_through_poses",
                         key="_nav_goals", generate_feedback_message=lambda m: f"{m.feedback.number_of_poses_remaining} left")
        self.cfg_key, self.bt = cfg_key, bt

    def initialise(self):
        goal = NavigateThroughPoses.Goal()
        goal.poses = [pose_from(d, Ctx.node) for d in Ctx.cfg["routes"][self.cfg_key]]
        if self.bt:
            # a route needs a through-poses tree ({goals}); bt_files.through, else the terrain tree
            bts = Ctx.cfg["bt_files"]
            goal.behavior_tree = bts.get(self.bt) or bts["terrain"]
        BB.set("_nav_goals", goal)
        super().initialise()


class TopicVote(py_trees.behaviour.Behaviour):
    """Majority vote over an inference stream: SUCCESS once one label has >= min_votes AND >= min_share of
    all votes since this behaviour started (a short burst of misclassifications can't win).

    enable_service (optional, std_srvs/SetBool): the detector is switched ON when voting starts and OFF
    when it ends (success, timeout or abort), so detection only runs while the robot is at the screen."""
    def __init__(self, name, topic, out_key, allowed, min_votes=10, min_share=0.7, enable_service=None):
        super().__init__(name=name)
        self.topic, self.out_key, self.allowed = topic, out_key, allowed
        self.min_votes, self.min_share = min_votes, min_share
        self.enable_service = enable_service
        self.enable_client = None
        self.votes = {}
        self.listening = False

    def setup(self, **kw):
        Ctx.node.create_subscription(String, self.topic, self._cb, 10)
        if self.enable_service:
            self.enable_client = Ctx.node.create_client(SetBool, self.enable_service)

    def _set_detector(self, on):
        """Ask the detector to start/stop. Non-blocking: the tree keeps ticking while the call is in flight."""
        if self.enable_client is None:
            return
        if not self.enable_client.service_is_ready():
            Ctx.node.get_logger().warn(
                f"{self.enable_service} not available - is the waste detector running on the Pi?")
            return
        self.enable_client.call_async(SetBool.Request(data=on))
        Ctx.node.get_logger().info(f"detector {'ON' if on else 'OFF'}")

    def _cb(self, msg):
        if not self.listening:                         # ignore readings from before the robot reached the screen
            return
        v = msg.data.strip().lower()
        if v in self.allowed:
            self.votes[v] = self.votes.get(v, 0) + 1

    def initialise(self):
        self.votes = {}
        self.listening = True
        self._set_detector(True)

    def update(self):
        total = sum(self.votes.values())
        if not total:
            return py_trees.common.Status.RUNNING
        best = max(self.votes, key=self.votes.get)
        n = self.votes[best]
        self.feedback_message = f"{best} {n}/{total}"
        if n >= self.min_votes and n / total >= self.min_share:
            BB.set(self.out_key, best)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.listening:
            self._set_detector(False)
        self.listening = False


class WaitForBool(py_trees.behaviour.Behaviour):
    """SUCCESS as soon as the Bool topic reads True (wrap in a Timeout for a deadline).
    Writes how long it waited (s) to the blackboard, and `<wait_key>_on_arrival` = True if it was
    already True at the first check (open on arrival)."""
    def __init__(self, name, topic, wait_key):
        super().__init__(name=name)
        self.topic, self.wait_key = topic, wait_key
        self.value, self.stamp, self.t_start = None, 0.0, None

    def setup(self, **kw):
        Ctx.node.create_subscription(Bool, self.topic, self._cb, 10)

    def _cb(self, m):
        self.value, self.stamp = m.data, time.time()

    def initialise(self):
        self.t_start = time.time()
        self.first_check = True

    def update(self):
        fresh = time.time() - self.stamp < 2.0           # a detector that went silent must not count as "open"
        if self.value is True and fresh:
            on_arrival = self.first_check
            BB.set(self.wait_key, 0.0 if on_arrival else round(time.time() - self.t_start, 1))
            BB.set(self.wait_key + "_on_arrival", on_arrival)
            return py_trees.common.Status.SUCCESS
        self.first_check = False
        return py_trees.common.Status.RUNNING


class GateRoute(py_trees.behaviour.Behaviour):
    """Turns the measured wait into the logged decision: open on arrival vs. opened within the standard wait."""
    def __init__(self):
        super().__init__(name="GateRoute")

    def update(self):
        BB.set("gate_route", "path2_open" if BB.get("gate_wait_on_arrival") else "path2_after_wait")
        return py_trees.common.Status.SUCCESS


class LoadItem(py_trees.behaviour.Behaviour):
    """Loading station: announce the waste type, then wait a fixed time (mission.yaml load_wait_s).
    No signal comes from the station, so the tree just waits."""
    def __init__(self, wait_s=5.0):
        super().__init__(name="LoadItem")
        self.wait_s = float(wait_s)
        self.t = None

    def setup(self, **kw):
        self.pub = Ctx.node.create_publisher(String, "/loading/request", 10)

    def initialise(self):
        self.pub.publish(String(data=BB.get("waste_type")))
        self.t = time.time()
        Ctx.node.get_logger().info(f"loading: waiting {self.wait_s:.1f} s")

    def update(self):
        if time.time() - self.t < self.wait_s:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS


class Offload(py_trees.behaviour.Behaviour):
    """Offloading mechanism: open the payload gate, hold, close."""
    def __init__(self, open_s=2.5):
        super().__init__(name="Offload")
        self.open_s = open_s
        self.t = None

    def setup(self, **kw):
        self.pub = Ctx.node.create_publisher(Bool, "/gate", 10)

    def initialise(self):
        self.pub.publish(Bool(data=True))
        self.t = time.time()

    def update(self):
        if time.time() - self.t < self.open_s:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.pub.publish(Bool(data=False))          # always close, also when interrupted


class SpeedCapped(py_trees.decorators.Decorator):
    """Nav2 speed cap (m/s) while the child runs; cleared however the child ends (success, failure, abort)."""
    def __init__(self, name, speed, child):
        super().__init__(name=name, child=child)
        self.speed = float(speed)

    def setup(self, **kw):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = Ctx.node.create_publisher(SpeedLimit, "/speed_limit", qos)

    def _send(self, speed):
        m = SpeedLimit()
        m.percentage = False
        m.speed_limit = speed                        # 0.0 = no limit
        self.pub.publish(m)

    def initialise(self):
        self._send(self.speed)

    def update(self):
        return self.decorated.status

    def terminate(self, new_status):
        self._send(0.0)


class ObstacleMonitor(py_trees.behaviour.Behaviour):
    """Runs in parallel with the mission: logs the first collision-monitor stop (dynamic obstacle).
    Needs collision_monitor.state_topic set (nav2_params.yaml: collision_monitor_state)."""
    def __init__(self):
        super().__init__(name="ObstacleMonitor")
        self.logged = False
        self.state = 0

    def setup(self, **kw):
        Ctx.node.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                     lambda m: setattr(self, "state", m.action_type), 10)

    def update(self):
        if self.state == CollisionMonitorState.STOP and not self.logged:
            self.logged = True
            log_event("obstacle_detected")
        return py_trees.common.Status.RUNNING


class AlignYaw(py_trees.behaviour.Behaviour):
    """Turn on the spot until the robot faces the pose's yaw within `tol_deg` (Nav2 'spin' behaviour).
    Nav2's goal checker accepts up to yaw_goal_tolerance (0.25 rad = 14 deg); this squares the robot up,
    e.g. in front of the screen so the camera looks straight at it. It never fails the mission: if TF or
    the spin server is unavailable it logs a warning and carries on."""
    def __init__(self, name, cfg_key=None, pose_key=None, tol_deg=3.0, tries=2):
        super().__init__(name=name)
        self.cfg_key, self.pose_key = cfg_key, pose_key
        self.tol, self.tries = math.radians(tol_deg), tries

    def setup(self, **kw):
        if getattr(Ctx, "tf", None) is None:
            from tf2_ros import Buffer, TransformListener
            Ctx.tf = Buffer()
            Ctx.tf_listener = TransformListener(Ctx.tf, Ctx.node)
        self.client = ActionClient(Ctx.node, Spin, "spin")

    def _robot_yaw(self):
        t = Ctx.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        q = t.transform.rotation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _target_yaw(self):
        d = BB.get(self.pose_key) if self.pose_key else Ctx.cfg["poses"][self.cfg_key]
        return world_to_map(float(d["x"]), float(d["y"]), float(d.get("yaw", 0.0)), Ctx.map_origin)[2]

    def initialise(self):
        self.left, self.goal_future, self.result_future, self.t_sent = self.tries, None, None, 0.0

    def update(self):
        log = Ctx.node.get_logger()
        if self.result_future is not None:                 # a spin is running
            if not self.result_future.done():
                if time.time() - self.t_sent > 8.0:
                    log.warn(f"{self.name}: spin took too long - carrying on")
                    return py_trees.common.Status.SUCCESS
                return py_trees.common.Status.RUNNING
            self.result_future = None
        if self.goal_future is not None:                   # waiting for the spin to be accepted
            if not self.goal_future.done():
                return py_trees.common.Status.RUNNING
            handle, self.goal_future = self.goal_future.result(), None
            if not handle.accepted:
                log.warn(f"{self.name}: spin rejected - carrying on")
                return py_trees.common.Status.SUCCESS
            self.result_future, self.t_sent = handle.get_result_async(), time.time()
            return py_trees.common.Status.RUNNING
        try:
            err = self._target_yaw() - self._robot_yaw()
            err = math.atan2(math.sin(err), math.cos(err))
        except Exception as e:
            log.warn(f"{self.name}: no robot heading ({e}) - carrying on")
            return py_trees.common.Status.SUCCESS
        if abs(err) <= self.tol:
            log.info(f"{self.name}: aligned ({math.degrees(err):+.1f} deg)")
            return py_trees.common.Status.SUCCESS
        if self.left <= 0 or not self.client.server_is_ready():
            log.warn(f"{self.name}: still {math.degrees(err):+.1f} deg off - carrying on")
            return py_trees.common.Status.SUCCESS
        self.left -= 1
        goal = Spin.Goal()
        goal.target_yaw = float(err)
        goal.time_allowance = Duration(sec=5)
        self.goal_future = self.client.send_goal_async(goal)
        log.info(f"{self.name}: turning {math.degrees(err):+.1f} deg to face straight")
        return py_trees.common.Status.RUNNING


class DockAt(AlignYaw):
    """Dock exactly on a pose: square up to its yaw (spin), then drive straight forward or back so the
    robot sits on the pose along its heading (DriveOnHeading / BackUp), then square up once more.
    Nav2 stops anywhere within xy_goal_tolerance (12 cm) and 14 deg; this brings it to ~2 cm and ~3 deg.
    Sideways error cannot be fixed by a skid-steer without re-approaching, so it is only logged.
    Like AlignYaw it never fails the mission."""
    def __init__(self, name, cfg_key=None, pose_key=None, tol_deg=3.0, tol_m=0.02, speed=0.08):
        super().__init__(name, cfg_key=cfg_key, pose_key=pose_key, tol_deg=tol_deg, tries=1)
        self.tol_m, self.speed = tol_m, speed

    def setup(self, **kw):
        super().setup(**kw)
        self.drive = ActionClient(Ctx.node, DriveOnHeading, "drive_on_heading")
        self.backup = ActionClient(Ctx.node, BackUp, "backup")

    def _robot_pose(self):
        t = Ctx.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _target(self):
        d = BB.get(self.pose_key) if self.pose_key else Ctx.cfg["poses"][self.cfg_key]
        return world_to_map(float(d["x"]), float(d["y"]), float(d.get("yaw", 0.0)), Ctx.map_origin)

    def initialise(self):
        self.steps, self.future, self.result, self.t_sent = ["spin", "drive", "spin"], None, None, 0.0

    def update(self):
        log = Ctx.node.get_logger()
        if self.result is not None:                        # a manoeuvre is running
            if not self.result.done():
                if time.time() - self.t_sent > 10.0:
                    log.warn(f"{self.name}: manoeuvre took too long - carrying on")
                    return py_trees.common.Status.SUCCESS
                return py_trees.common.Status.RUNNING
            self.result = None
        if self.future is not None:                        # waiting for it to be accepted
            if not self.future.done():
                return py_trees.common.Status.RUNNING
            handle, self.future = self.future.result(), None
            if handle.accepted:
                self.result, self.t_sent = handle.get_result_async(), time.time()
                return py_trees.common.Status.RUNNING
            log.warn(f"{self.name}: manoeuvre rejected - skipping it")
        while self.steps:
            step = self.steps.pop(0)
            try:
                x, y, yaw = self._robot_pose()
                tx, ty, tyaw = self._target()
            except Exception as e:
                log.warn(f"{self.name}: no robot pose ({e}) - carrying on")
                return py_trees.common.Status.SUCCESS
            if step == "spin":
                err = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
                if abs(err) <= self.tol or not self.client.server_is_ready():
                    continue
                goal = Spin.Goal()
                goal.target_yaw = float(err)
                goal.time_allowance = Duration(sec=5)
                log.info(f"{self.name}: turning {math.degrees(err):+.1f} deg to square up")
                self.future = self.client.send_goal_async(goal)
                return py_trees.common.Status.RUNNING
            along = (tx - x) * math.cos(tyaw) + (ty - y) * math.sin(tyaw)
            if abs(along) <= self.tol_m:
                continue
            if along > 0:
                client, goal = self.drive, DriveOnHeading.Goal()
            else:
                client, goal = self.backup, BackUp.Goal()
            if not client.server_is_ready():
                log.warn(f"{self.name}: drive/backup server not ready - skipping distance fix")
                continue
            goal.target.x = abs(float(along))
            goal.speed = float(self.speed)
            goal.time_allowance = Duration(sec=8)
            log.info(f"{self.name}: {'forward' if along > 0 else 'back'} {abs(along) * 100:.1f} cm to the docking spot")
            self.future = client.send_goal_async(goal)
            return py_trees.common.Status.RUNNING
        try:
            x, y, yaw = self._robot_pose()
            tx, ty, tyaw = self._target()
            along = (tx - x) * math.cos(tyaw) + (ty - y) * math.sin(tyaw)
            side = -(tx - x) * math.sin(tyaw) + (ty - y) * math.cos(tyaw)
            err = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
            log.info(f"{self.name}: docked - {along * 100:+.1f} cm along, {side * 100:+.1f} cm sideways, {math.degrees(err):+.1f} deg")
        except Exception:
            pass
        return py_trees.common.Status.SUCCESS


class NavVia(NavThroughPoses):
    """A leg that drives through corner checkpoints (mission.yaml `via:`) and ends at the leg's goal."""
    def __init__(self, name, via, pose_key=None, cfg_key=None, bt=None):
        super().__init__(name, "_via_" + name, bt=bt)
        self.via, self.goal_pose_key, self.goal_cfg_key = via, pose_key, cfg_key

    def initialise(self):
        final = BB.get(self.goal_pose_key) if self.goal_pose_key else Ctx.cfg["poses"][self.goal_cfg_key]
        via = self.via
        if isinstance(via, dict):                     # checkpoints per waste type
            via = via.get(str(BB.get("waste_type")), [])
        Ctx.cfg.setdefault("routes", {})["_via_" + self.name] = list(via) + [final]
        super().initialise()


def Leg(name, pose_key=None, cfg_key=None, bt=None):
    """NavVia when mission.yaml lists checkpoints for this leg, else a plain NavToPose."""
    via = (Ctx.cfg.get("via") or {}).get(name)
    if via:
        bts = Ctx.cfg.get("bt_files") or {}
        return NavVia(name, via, pose_key=pose_key, cfg_key=cfg_key,
                      bt="through" if "through" in bts else "terrain")
    return NavToPose(name, pose_key=pose_key, cfg_key=cfg_key, bt=bt)


# ----------------------------------------------------------------------------- tree
def build_tree():
    cfg = Ctx.cfg
    S, F = py_trees.composites.Sequence, py_trees.composites.Selector
    timeout = lambda name, s, child: py_trees.decorators.Timeout(name=name, duration=s, child=child)
    retry = lambda name, n, child: py_trees.decorators.Retry(name=name, num_failures=n, child=child)

    detect = S("WasteDetection", memory=True, children=[
        Leg("GoToScreen", cfg_key="screen", bt="nav"),
        AlignYaw("AlignAtScreen", cfg_key="screen", tol_deg=cfg.get("screen_align_tol_deg", 3.0)),
        F("DetectOrDefault", memory=True, children=[
            retry("Retry2", 2, timeout("Detect8s", 8.0,
                  TopicVote("DetectWasteType", "/waste_type", "waste_type", WASTE_TYPES,
                            cfg.get("min_votes", 10), cfg.get("min_share", 0.7),
                            enable_service=cfg.get("detector_enable_service", "/waste/enable")))),
            py_trees.behaviours.SetBlackboardVariable("UseAssignedType", "waste_type", cfg["waste_type_default"], True),
        ]),
        LogEvent("waste_detected", "waste_type"),
    ])

    loading = S("Loading", memory=True, children=[
        SelectPose("poses_loading", "waste_type", "pose_loading"),
        Leg("GoToLoading", pose_key="pose_loading", bt="nav"),
        DockAt("DockAtLoading", pose_key="pose_loading", tol_deg=cfg.get("loading_align_tol_deg", 3.0)),
        LoadItem(cfg.get("load_wait_s", 5.0)),
        LogEvent("loading_completed"),
    ])

    # Gate rule: open on arrival -> path 2; opens within the standard wait -> path 2 (waiting time logged);
    # otherwise path 1. One timer measures the whole wait, so the total never exceeds gate_wait_s.
    route = S("RouteDecision", memory=True, children=[
        Leg("GoToJunction", cfg_key="junction", bt="nav"),
        LogEvent("junction_reached"),
        F("ChooseRoute", memory=True, children=[
            S("Path2", memory=True, children=[
                timeout("GateWait", cfg["gate_wait_s"], WaitForBool("GateOpen?", "/gate_open", "gate_wait")),
                GateRoute(),
                LogEvent("gate_waiting_time", "gate_wait"),
                LogEvent("gate_decision", "gate_route"),
                NavThroughPoses("Path2Flat", "path2", bt="through")]),
            S("Path1", memory=True, children=[
                py_trees.behaviours.SetBlackboardVariable("Route1", "gate_route", "path1_divert", True),
                LogEvent("gate_decision", "gate_route"),
                SpeedCapped("TerrainSpeed", cfg["terrain_speed"],
                            NavThroughPoses("Path1Terrain", "path1", bt="terrain"))]),
        ]),
        LogEvent("terrain_completed"),
    ])

    deposit = S("Depositing", memory=True, children=[
        Leg("GoToBins", cfg_key="bins", bt="nav"),
        SelectPose("poses_bins", "waste_type", "pose_bin"),        # colour is fixed by rule: blue/white/black
        Leg("GoToBin", pose_key="pose_bin", bt="nav"),
        DockAt("DockAtBin", pose_key="pose_bin", tol_deg=cfg.get("bin_align_tol_deg", 3.0)),
        Offload(cfg["offload_open_s"]),
        LogEvent("depositing_completed"),
    ])

    # Testing aid: mission.yaml `skip_stages: [detection, loading, route, depositing]` replaces those stages
    # with a log entry ("skipped"). Detection skipped -> the assigned waste_type_default is used.
    skip = set(cfg.get("skip_stages") or [])
    if "detection" in skip:
        detect = S("DetectionSkipped", memory=True, children=[
            py_trees.behaviours.SetBlackboardVariable("UseAssignedType", "waste_type", cfg["waste_type_default"], True),
            LogEvent("waste_detected", "waste_type")])
    skipped = lambda name, event: S(name, memory=True, children=[
        py_trees.behaviours.SetBlackboardVariable(name + "Note", "_skipped", "skipped", True), LogEvent(event, "_skipped")])
    if "loading" in skip:
        loading = skipped("LoadingSkipped", "loading_completed")
    if "route" in skip:
        route = skipped("RouteSkipped", "terrain_completed")
    if "depositing" in skip:
        deposit = skipped("DepositingSkipped", "depositing_completed")
    if skip:
        Ctx.node.get_logger().warn(f"TEST MODE - skipping: {', '.join(sorted(skip))}")

    mission = S("AutonomousPhase", memory=True, children=[
        LogEvent("autonomous_start"), detect, loading, route, deposit,
        # mission.yaml `drive_to_finish: false` -> the run ends at the bin (no drive to poses.finish)
        *([Leg("GoToFinish", cfg_key="finish", bt="nav")] if cfg.get("drive_to_finish", True) else []),
        LogEvent("finish"),
    ])
    return py_trees.composites.Parallel(
        "Mission", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([mission]),
        children=[ObstacleMonitor(), mission])


def check_config(cfg, log):
    """Catch placeholder poses before the robot drives into a wall."""
    (x0, x1), (y0, y1) = cfg.get("field_bounds", [[-3.0, 3.0], [-1.53, 1.53]])
    poses = [(f"poses.{k}", v) for k, v in cfg["poses"].items()]
    poses += [(f"poses_loading.{k}", v) for k, v in cfg["poses_loading"].items()]
    poses += [(f"poses_bins.{k}", v) for k, v in cfg["poses_bins"].items()]
    poses += [(f"routes.{r}[{i}]", v) for r, lst in cfg["routes"].items() for i, v in enumerate(lst)]
    bad = [n for n, p in poses if not (x0 <= p["x"] <= x1 and y0 <= p["y"] <= y1)]
    for n in bad:
        log.error(f"{n} is outside the field {cfg.get('field_bounds')} - still a placeholder?")
    for k in WASTE_TYPES:
        for table in ("poses_loading", "poses_bins"):
            if k not in cfg[table]:
                log.error(f"{table} has no entry for '{k}'")
    for name, path in cfg["bt_files"].items():
        if not os.path.exists(path):
            log.error(f"bt_files.{name}: {path} does not exist")
    return not bad


def main():
    rclpy.init()
    node = rclpy.create_node("mission_tree")
    node.declare_parameter("config", "")
    node.declare_parameter("waste_type_default", "plastic")
    node.declare_parameter("robot_id", "MB01")
    node.declare_parameter("start_even_if_config_bad", False)
    for k in ("map_origin_x", "map_origin_y", "map_origin_yaw"):
        node.declare_parameter(k, 0.0)
    with open(node.get_parameter("config").value) as f:
        cfg = yaml.safe_load(f)
    cfg["waste_type_default"] = node.get_parameter("waste_type_default").value
    # machine-independent paths: '~' expands to the user's home; bare BT file names live in this package's bt/
    cfg["log_file"] = os.path.expanduser(cfg["log_file"])
    bt_dir = os.path.join(get_package_share_directory("malkia_mission"), "bt")
    cfg["bt_files"] = {k: os.path.expanduser(v) if os.path.isabs(os.path.expanduser(v)) else os.path.join(bt_dir, v)
                       for k, v in cfg["bt_files"].items()}
    Ctx.map_origin = tuple(float(node.get_parameter(k).value) for k in ("map_origin_x", "map_origin_y", "map_origin_yaw"))
    node.get_logger().info(f"map origin (world pose of map 0,0,0): {Ctx.map_origin}")
    Ctx.node, Ctx.cfg, Ctx.robot_id = node, cfg, node.get_parameter("robot_id").value
    Ctx.events_pub = node.create_publisher(String, "/mission/events", EVENTS_QOS)

    if not check_config(cfg, node.get_logger()) and not node.get_parameter("start_even_if_config_bad").value:
        node.get_logger().fatal("fix mission.yaml (or -p start_even_if_config_bad:=true)")
        rclpy.try_shutdown()
        return

    tree = py_trees_ros.trees.BehaviourTree(root=build_tree(), unicode_tree_debug=False)
    tree.setup(node=node, timeout=15.0)

    def stop_when_done(t):
        status = t.root.status
        if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE) and not getattr(t, "_done", False):
            t._done = True
            t.timer.cancel()                       # a finished tree would otherwise restart from the top
            tip = t.root.tip()
            log_event("mission_complete" if status == py_trees.common.Status.SUCCESS else "mission_failed",
                      None if status == py_trees.common.Status.SUCCESS else (tip.name if tip else None))
            node.get_logger().info("mission finished - tree stopped; restart the node for another run")

    node.get_logger().info("mission tree ready - ticking at 5 Hz")
    tree.tick_tock(period_ms=200, post_tick_handler=stop_when_done)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
