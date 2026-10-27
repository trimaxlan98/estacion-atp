"""Servidor Monte Carlo. Lanzar una campana:
  ros2 action send_goal --feedback /atp/run_campaign atp_interfaces/action/RunCampaign \\
    "{mode: lazo, n: 300}"
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='atp_ros', executable='campaign_server')])
