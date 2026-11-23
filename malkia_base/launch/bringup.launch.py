"""malkia_base bring-up: Nano bridge (+ EKF) + RPLIDAR + robot model.

    ros2 launch malkia_base bringup.launch.py                                   # everything, /dev/mcu + /dev/rplidar
    ros2 launch malkia_base bringup.launch.py lidar:=none                       # motors/odometry only (as before)
    ros2 launch malkia_base bringup.launch.py use_ekf:=true                     # EKF owns odom->base_footprint (after the IMU axis test)
    ros2 launch malkia_base bringup.launch.py lidar_baud:=256000                # RPLIDAR A2M12 / A3 / S1

TF tree:  odom -> base_footprint (EKF, or base_node if use_ekf:=false)
                   -> base_link -> lidar_link, imu_link, 4 wheels   (robot_state_publisher, URDF from cramped_robot_description)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('malkia_base')
    base_cfg = os.path.join(share, 'config', 'base.yaml')
    ekf_cfg = os.path.join(share, 'config', 'ekf.yaml')

    use_ekf, port, use_urdf = (LaunchConfiguration(k) for k in ('use_ekf', 'port', 'urdf'))
    lidar, lidar_port, lidar_baud = (LaunchConfiguration(k) for k in ('lidar', 'lidar_port', 'lidar_baud'))

    # With the EKF running, the EKF owns the odom -> base_footprint transform.
    base_publishes_tf = ParameterValue(PythonExpression(["'", use_ekf, "' != 'true'"]), value_type=bool)

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 PathJoinSubstitution([FindPackageShare('cramped_robot_description'), 'urdf', 'cramped_robot.urdf.xacro']),
                 ' use_sim:=false']),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_ekf', default_value='false',
                              description='fuse wheel odometry with the gyro; enable only after the IMU axis test'),
        DeclareLaunchArgument('port', default_value='/dev/mcu'),
        DeclareLaunchArgument('urdf', default_value='true', description='publish the robot model/TF from the URDF'),
        DeclareLaunchArgument('lidar', default_value='rplidar', description='rplidar | none'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('lidar_baud', default_value='115200', description='A1/A2M8 115200, A2M12/A3/S1 256000, S2 1000000'),

        # ---- Nano bridge: /cmd_vel -> motors, encoders -> /odom (+ /imu/data when the IMU works)
        Node(package='malkia_base', executable='base_node', name='base_node', output='screen',
             parameters=[base_cfg, {'port': port, 'publish_tf': base_publishes_tf}]),

        # ---- robot model: base_footprint -> base_link -> lidar_link / imu_link / wheels
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
             condition=IfCondition(use_urdf)),
        # fallback without the URDF: just an IMU frame on base_link (old behaviour)
        Node(package='tf2_ros', executable='static_transform_publisher', name='imu_tf',
             arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'imu_link'],
             condition=UnlessCondition(use_urdf)),

        # ---- RPLIDAR -> /scan in frame lidar_link
        Node(package='sllidar_ros2', executable='sllidar_node', name='rplidar', output='screen', respawn=True, respawn_delay=3.0,
             parameters=[{'serial_port': lidar_port,
                          'serial_baudrate': ParameterValue(lidar_baud, value_type=int),
                          'frame_id': 'lidar_link', 'inverted': False,
                          'angle_compensate': True, 'scan_frequency': 6.0}],
             condition=LaunchConfigurationEquals('lidar', 'rplidar')),

        # ---- EKF: wheel odometry (+ gyro yaw rate when available) -> odom -> base_footprint
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[ekf_cfg], condition=IfCondition(use_ekf)),
    ])
