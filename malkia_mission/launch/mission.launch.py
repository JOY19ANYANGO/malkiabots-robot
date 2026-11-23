"""Autonomous-phase mission tree. Run after bringup + navigation (robot localised).
    ros2 launch malkia_mission mission.launch.py waste_type_default:=metal                       # CAD map
    ros2 launch malkia_mission mission.launch.py map_origin_x:=-0.54 map_origin_y:=1.15         # SLAM map from the start pad
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("malkia_mission")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "mission.yaml")),
        DeclareLaunchArgument("waste_type_default", default_value="plastic",
                              description="load assigned at the start; used if the screen cannot be read"),
        DeclareLaunchArgument("robot_id", default_value="MB01"),
        # World pose of the map's (0, 0, 0). CAD map: 0 0 0. SLAM map started on the start pad: -0.54 1.15 0.0
        DeclareLaunchArgument("map_origin_x", default_value="0.0"),
        DeclareLaunchArgument("map_origin_y", default_value="0.0"),
        DeclareLaunchArgument("map_origin_yaw", default_value="0.0"),
        Node(package="malkia_mission", executable="mission_tree", name="mission_tree", output="screen",
             parameters=[{"config": LaunchConfiguration("config"),
                          "waste_type_default": LaunchConfiguration("waste_type_default"),
                          "robot_id": LaunchConfiguration("robot_id"),
                          "map_origin_x": ParameterValue(LaunchConfiguration("map_origin_x"), value_type=float),
                          "map_origin_y": ParameterValue(LaunchConfiguration("map_origin_y"), value_type=float),
                          "map_origin_yaw": ParameterValue(LaunchConfiguration("map_origin_yaw"), value_type=float)}]),
    ])
