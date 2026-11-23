"""Nav2 bring-up for cramped_robot: map_server + AMCL (localization) and the navigation servers.

Simulation (after launch_sim.launch.py):
    ros2 launch cramped_robot_description navigation.launch.py
Real robot (after robot.launch.py):
    ros2 launch cramped_robot_description navigation.launch.py use_sim_time:=false map:=/path/to/your_map.yaml

The collision monitor (stop/slowdown polygons fed by /scan) is always the last stage before /cmd_vel.
Optional: velocity_smoother:=true inserts nav2_velocity_smoother before it.
Send goals with RViz "2D Goal Pose", or run the autonomous patrol:
    ros2 run cramped_robot_description patrol.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("cramped_robot_description")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")

    # use_sim_time is passed AFTER the params file so it overrides the yaml (true in Gazebo, false on hardware)
    sim = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    # Velocity chain: controller/behaviors -> cmd_vel_nav -> [velocity_smoother -> cmd_vel_smoothed] -> collision_monitor -> cmd_vel
    # The collision monitor is always the last stage: it owns /cmd_vel and can stop the robot regardless of Nav2.
    use_vs = LaunchConfiguration("velocity_smoother")
    nav_remaps = [("cmd_vel", "cmd_vel_nav")]
    smoother_remaps = [("cmd_vel", "cmd_vel_nav")]                       # publishes cmd_vel_smoothed
    cm_input = PythonExpression(["'cmd_vel_smoothed' if '", use_vs, "'.lower() == 'true' else 'cmd_vel_nav'"])

    localization_nodes = ["map_server", "amcl"]
    navigation_nodes = PythonExpression(
        ["[n for n in ['controller_server', 'smoother_server', 'planner_server', 'behavior_server', "
         "'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower'] if n != 'velocity_smoother' or '",
         use_vs, "'.lower() == 'true']"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "gamefield.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("velocity_smoother", default_value="false"),

        # ---- localization ----
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[params, {"yaml_filename": map_yaml}, sim]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[params, sim]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
             output="screen", parameters=[{"autostart": autostart, "node_names": localization_nodes}, sim]),

        # ---- navigation ----
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[params, sim], remappings=nav_remaps),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", output="screen",
             parameters=[params, sim]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[params, sim]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
             parameters=[params, sim], remappings=nav_remaps),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="screen", parameters=[params, sim], remappings=smoother_remaps,
             condition=IfCondition(use_vs)),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             output="screen", parameters=[params, {"cmd_vel_in_topic": cm_input}, sim]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[params, sim]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower",
             output="screen", parameters=[params, sim]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"autostart": autostart, "node_names": navigation_nodes}, sim]),

        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "nav.rviz")], parameters=[sim],
             condition=IfCondition(rviz)),
    ])
