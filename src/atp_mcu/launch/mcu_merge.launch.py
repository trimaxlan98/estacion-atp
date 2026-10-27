"""Fusiona los ESP32 de cada eje en /joint_states.

El agente micro-ROS NO se lanza aqui: es un contenedor aparte
(firmware/esp32_eje/agente.sh). Con el agente y los ESP32 en marcha:

  ros2 launch atp_mcu mcu_merge.launch.py                     # dos ESP32 (az y el)
  ros2 launch atp_mcu mcu_merge.launch.py simulated:=el       # un solo ESP32 (az); el = referencia
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulated', default_value=''),
        DeclareLaunchArgument('stale_s', default_value='0.5'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='atp_mcu', executable='mcu_merge', parameters=[{
            'simulated': LC('simulated'),
            'stale_s': LC('stale_s'),
            'use_sim_time': LC('use_sim_time')}]),
    ])
