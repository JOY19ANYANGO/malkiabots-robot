"""Physical cramped_robot bring-up (no Gazebo).

    ros2 launch cramped_robot_description robot.launch.py                      # base + lidar + state publisher
    ros2 launch cramped_robot_description robot.launch.py lidar:=ldlidar port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
    ros2 launch cramped_robot_description robot.launch.py use_ekf:=true       # with an IMU publishing /imu

Then, in other terminals:
    ros2 launch cramped_robot_description mapping.launch.py use_sim_time:=false          # build the real map
    ros2 launch cramped_robot_description navigation.launch.py use_sim_time:=false map:=~/maps/gamefield_real.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "cramped_robot_description"
    share = get_package_share_directory(pkg_name)
    pkg = FindPackageShare(pkg_name)
    port, lidar, lidar_port, use_ekf = (LaunchConfiguration(k) for k in ("port", "lidar", "lidar_port", "use_ekf"))

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ",
                 PathJoinSubstitution([pkg, "urdf", "cramped_robot.urdf.xacro"]), " use_sim:=false"]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0", description="motor MCU serial port"),
        DeclareLaunchArgument("lidar", default_value="rplidar", description="rplidar | ldlidar | none"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("use_ekf", default_value="false", description="fuse /odom + /imu with robot_localization"),

        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description, "use_sim_time": False}]),

        # motor controller + wheel odometry (publishes odom->base_footprint unless the EKF does)
        Node(package=pkg_name, executable="base_driver.py", name="base_driver", output="screen",
             parameters=[{"port": port, "use_sim_time": False,
                          "publish_tf": ParameterValue(NotSubstitution(use_ekf), value_type=bool)}]),

        # 2-D lidar -> /scan in frame lidar_link
        Node(package="rplidar_ros", executable="rplidar_node", name="rplidar", output="screen",
             parameters=[{"serial_port": lidar_port, "serial_baudrate": 115200, "frame_id": "lidar_link",
                          "angle_compensate": True, "scan_mode": "Standard"}],
             condition=LaunchConfigurationEquals("lidar", "rplidar")),
        Node(package="ldlidar_stl_ros2", executable="ldlidar_stl_ros2_node", name="ldlidar", output="screen",
             parameters=[{"product_name": "LDLiDAR_LD19", "topic_name": "scan", "frame_id": "lidar_link",
                          "port_name": lidar_port, "port_baudrate": 230400, "laser_scan_dir": True,
                          "enable_angle_crop_func": False}],
             condition=LaunchConfigurationEquals("lidar", "ldlidar")),

        # optional: EKF (needs an IMU node publishing sensor_msgs/Imu on /imu, frame imu_link)
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
             parameters=[os.path.join(share, "config", "ekf.yaml")],
             condition=IfCondition(use_ekf)),
    ])
