"""Gemelo digital: lazo ATP + robot_state_publisher (TF) y, opcional, RViz.

  ros2 launch atp_ros atp_twin.launch.py rviz:=true
Azimut: el URDF gira antihorario (+Z); el acimut astronomico es horario desde
el norte, asi que se publica tal cual y se documenta aqui, no se oculta.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('atp_ros')
    with open(os.path.join(share, 'urdf', 'antena.urdf')) as f:
        urdf = f.read()
    lazo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'atp_loop.launch.py')),
        launch_arguments={'exit_on_done': 'false'}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': urdf, 'use_sim_time': True}])
    rviz = Node(package='rviz2', executable='rviz2', condition=IfCondition(LC('rviz')),
                arguments=['-d', os.path.join(share, 'rviz', 'atp.rviz')],
                parameters=[{'use_sim_time': True}])
    return LaunchDescription([DeclareLaunchArgument('rviz', default_value='false'),
                              lazo, rsp, rviz])
