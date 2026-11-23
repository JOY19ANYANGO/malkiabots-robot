"""Gazebo (gz-sim Harmonic) simulation of cramped_robot in the gamefield arena.

    ros2 launch cramped_robot_description launch_sim.launch.py
    ros2 launch cramped_robot_description launch_sim.launch.py x:=-2.6 y:=1.15 yaw:=0.0 rviz:=true

Fixes vs. the previous version:
  * GZ_SIM_RESOURCE_PATH now includes <install>/share so that the URDF's
    package://cramped_robot_description/meshes/... URIs resolve inside Gazebo
    (without it the robot is spawned invisible).
  * Default spawn pose moved onto the arena's start pad (-0.54, 1.15). The old
    pose (-1.18, 2.44) is 0.9 m outside the arena's north wall where there is no
    floor, so the robot fell out of the world.
  * ros_gz_bridge added: /clock, /cmd_vel, /odom, /tf, /joint_states, /scan, /imu.
"""
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            AppendEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "cramped_robot_description"
    share = get_package_share_directory(pkg_name)
    pkg = FindPackageShare(pkg_name)

    # ---------------- arguments ----------------
    world = LaunchConfiguration("world")
    x, y, z, yaw = (LaunchConfiguration(k) for k in ("x", "y", "z", "yaw"))
    rviz = LaunchConfiguration("rviz")
    args = [
        DeclareLaunchArgument("world", default_value=os.path.join(share, "worlds", "gamefield.world")),
        # centre of the raised start pad (top face z = 0.08); base_footprint spawns 3 cm above it and drops
        DeclareLaunchArgument("x",   default_value="-0.54"),
        DeclareLaunchArgument("y",   default_value="1.15"),
        DeclareLaunchArgument("z",   default_value="0.11"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="false"),
    ]

    # ---------------- Gazebo resource paths ----------------
    # <prefix>/share   -> resolves package://cramped_robot_description/...  (robot meshes)
    # <share>/models   -> resolves model://gamefield_arena                  (arena)
    gz_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join([os.path.join(get_package_prefix(pkg_name), "share"),
                         os.path.join(share, "models")]),
    )

    # ---------------- robot description ----------------
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ",
                 PathJoinSubstitution([pkg, "urdf", "cramped_robot.urdf.xacro"]),
                 " use_sim:=true"]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # ---------------- Gazebo ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -v 3 ", world], "on_exit_shutdown": "true"}.items(),
    )

    # ---------------- spawn ----------------
    spawn_robot = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "cramped_robot",
                   "-x", x, "-y", y, "-z", z, "-Y", yaw],
    )
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    # ---------------- ROS <-> Gazebo bridge ----------------
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": os.path.join(share, "config", "gz_bridge.yaml"),
                     "use_sim_time": True}],
    )

    # ---------------- optional RViz ----------------
    rviz_node = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", os.path.join(share, "rviz", "sim.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(args + [gz_resources, gazebo, robot_state_publisher, bridge, delayed_spawn, rviz_node])
