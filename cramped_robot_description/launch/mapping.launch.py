"""SLAM (slam_toolbox online_async) + optional RViz.
    ros2 launch cramped_robot_description mapping.launch.py                    # simulation
    ros2 launch cramped_robot_description mapping.launch.py use_sim_time:=false rviz:=false   # real robot
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("cramped_robot_description")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "mapper_params_online_async.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        # slam_toolbox's own launch configures + activates the lifecycle node (autostart)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": params, "use_sim_time": use_sim_time, "autostart": "true"}.items()),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "mapping.rviz")],
             parameters=[{"use_sim_time": use_sim_time}], condition=IfCondition(rviz)),
    ])
