"""Abre todo para teleoperar y ver el robot en RViz2.

  ros2 launch minisumo_ros visualizacion.launch.py              # robot + RViz2
  ros2 launch minisumo_ros visualizacion.launch.py guante:=true # además el guante

Argumentos: guante, v_max, w_max, swap_axes, invert_pitch, invert_roll, dojo_cx, dojo_cy.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('minisumo_ros'), 'rviz', 'minisumo.rviz')
    arg = LaunchConfiguration
    args = [
        DeclareLaunchArgument('guante', default_value='false'),
        DeclareLaunchArgument('v_max', default_value='0.6'),
        DeclareLaunchArgument('w_max', default_value='4.0'),
        DeclareLaunchArgument('swap_axes', default_value='true'),
        DeclareLaunchArgument('invert_pitch', default_value='true'),
        DeclareLaunchArgument('invert_roll', default_value='true'),
        DeclareLaunchArgument('dojo_cx', default_value='0.0'),
        DeclareLaunchArgument('dojo_cy', default_value='0.0'),
    ]
    nodes = [
        Node(package='minisumo_ros', executable='robot_bridge', output='screen',
             parameters=[{'cmd_timeout': 1.0}]),
        Node(package='minisumo_ros', executable='diff_kinematics', output='screen',
             parameters=[{'omega_max': 30.0}]),
        Node(package='minisumo_ros', executable='visualizer', output='screen',
             parameters=[{'dojo_cx': arg('dojo_cx'), 'dojo_cy': arg('dojo_cy')}]),
        Node(package='minisumo_ros', executable='glove_teleop', output='screen',
             condition=IfCondition(arg('guante')),
             parameters=[{'v_max': arg('v_max'), 'w_max': arg('w_max'), 'max_deg': 30.0,
                          'swap_axes': arg('swap_axes'), 'invert_pitch': arg('invert_pitch'),
                          'invert_roll': arg('invert_roll')}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg], output='log'),
    ]
    return LaunchDescription(args + nodes)
