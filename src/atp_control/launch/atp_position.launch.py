"""Modo de POSICION de la montura: joint_trajectory_controller sobre AtpMountSystem.

  ros2 launch atp_control atp_position.launch.py                (comprueba y termina)
  ros2 launch atp_control atp_position.launch.py backend:=serial

El ros2_control_node estandar (reloj de pared, 100 Hz) con el plugin de hardware en
`sim` o `serial` (MCU falso) y el controlador de trayectorias de ros2_controllers:
las dos juntas reciben la consigna de POSICION y el hardware cierra el servo.
"""
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context):
    c = lambda k: LC(k).perform(context)   # noqa: E731
    share = get_package_share_directory('atp_control')
    with open(os.path.join(share, 'config', 'atp_controllers.yaml')) as fh:
        cfg = yaml.safe_load(fh)
    cfg['controller_manager']['ros__parameters']['update_rate'] = 100
    fd, ruta = tempfile.mkstemp(prefix='atp_pos_', suffix='.yaml')
    with os.fdopen(fd, 'w') as fh:
        yaml.safe_dump(cfg, fh)
    urdf = xacro.process_file(
        os.path.join(get_package_share_directory('atp_description'), 'urdf', 'antena.urdf.xacro'),
        mappings={'hw': c('backend'), 'az0': '0.0', 'el0': '0.5236'}).toxml()
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': ParameterValue(urdf, value_type=str)}])
    cm = Node(package='controller_manager', executable='ros2_control_node', output='screen',
              parameters=[ruta])
    sp = lambda n: Node(package='controller_manager', executable='spawner', output='screen',   # noqa: E731
                        arguments=[n, '--controller-manager', '/controller_manager',
                                   '--controller-manager-timeout', '60'])
    jsb, jtc = sp('joint_state_broadcaster'), sp('joint_trajectory_controller')
    chk = Node(package='atp_control', executable='position_check.py', output='screen')
    return [rsp, cm, jsb,
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jtc])),
            RegisterEventHandler(OnProcessExit(target_action=jtc, on_exit=[chk])),
            RegisterEventHandler(OnProcessExit(target_action=chk, on_exit=[EmitEvent(event=Shutdown())]))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('backend', default_value='sim'),
                              OpaqueFunction(function=_setup)])
