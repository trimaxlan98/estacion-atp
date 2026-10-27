"""Identificacion de la planta de la montura con un escalon de par (J y b).

  ros2 launch atp_control atp_identify.launch.py backend:=sim axis:=0   # AtpMountSystem con ros2_control_node
  ros2 launch atp_control atp_identify.launch.py backend:=gz  axis:=0   # Gazebo (gz_ros2_control)
  ros2 launch atp_control atp_identify.launch.py backend:=gz  axis:=1 torque_nm:=0.05 t_push_s:=12

Misma descripcion, mismo controller_manager y mismo identify_plant.py para los dos:
solo cambia el hardware. backend:=sim usa el `ros2_control_node` ESTANDAR (reloj de
pared a 100 Hz, sin lockstep): prueba que el plugin funciona sin nuestro lanzador.
"""
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = (('backend', 'sim'), ('axis', '0'), ('torque_nm', '1.0'), ('t_hold_s', '2.0'),
        ('t_push_s', '24.0'), ('t_coast_s', '20.0'), ('cm_rate', '100'), ('out_file', ''),
        ('headless', 'true'), ('csv_file', ''))


def _setup(context):
    c = lambda k: LC(k).perform(context)   # noqa: E731
    gz_mode = c('backend') == 'gz'
    share_desc = get_package_share_directory('atp_description')
    base = os.path.join(get_package_share_directory('atp_control'), 'config',
                        'atp_controllers.yaml')
    with open(base) as fh:
        cfg = yaml.safe_load(fh)
    cfg['controller_manager']['ros__parameters'].update(
        {'update_rate': int(c('cm_rate')), 'use_sim_time': gz_mode})
    fd, ctrl_yaml = tempfile.mkstemp(prefix='atp_ident_', suffix='.yaml')
    with os.fdopen(fd, 'w') as fh:
        yaml.safe_dump(cfg, fh)
    urdf = xacro.process_file(os.path.join(share_desc, 'urdf', 'antena.urdf.xacro'), mappings={
        'hw': 'gz' if gz_mode else 'sim', 'az0': '0.0', 'el0': '0.5236',
        'controllers_file': ctrl_yaml}).toxml()
    sim = {'use_sim_time': gz_mode}
    descr = {'robot_description': ParameterValue(urdf, value_type=str)}

    spawn_jsb = Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=['joint_state_broadcaster', '--controller-manager',
                                '/controller_manager', '--controller-manager-timeout', '120'])
    spawn_eff = Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=['effort_controller', '--controller-manager',
                                '/controller_manager', '--controller-manager-timeout', '120'])
    ident = Node(package='atp_control', executable='identify_plant.py', output='screen',
                 parameters=[sim, {'axis': int(c('axis')), 'torque_nm': float(c('torque_nm')),
                                   't_hold_s': float(c('t_hold_s')), 't_push_s': float(c('t_push_s')),
                                   't_coast_s': float(c('t_coast_s')), 'out_file': c('out_file'),
                                   'csv_file': c('csv_file')}])
    fin = RegisterEventHandler(OnProcessExit(target_action=ident, on_exit=[EmitEvent(event=Shutdown())]))
    if not gz_mode:
        # Jazzy: el controller_manager recibe el URDF por el topico /robot_description
        # (lo publica robot_state_publisher), no como parametro
        rsp0 = Node(package='robot_state_publisher', executable='robot_state_publisher',
                    parameters=[sim, descr])
        cm = Node(package='controller_manager', executable='ros2_control_node', output='screen',
                  parameters=[ctrl_yaml])
        return [rsp0, cm, RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_eff])),
                spawn_jsb, RegisterEventHandler(OnProcessExit(target_action=spawn_eff, on_exit=[ident])), fin]
    particion = f'atp_id_{os.getpid()}'
    world = os.path.join(share_desc, 'worlds', 'atp.sdf')
    gz_cmd = ['gz', 'sim', '-r', '-v', '2', world]
    if c('headless') == 'true':
        gz_cmd.insert(2, '-s')
    gz = ExecuteProcess(cmd=gz_cmd, output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[sim, descr])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'atp_mount', '-world', 'atp'])
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(share_desc)),
            SetEnvironmentVariable('GZ_PARTITION', particion),
            SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
            gz, rsp, bridge, spawn,
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawn_jsb])),
            RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_eff])),
            RegisterEventHandler(OnProcessExit(target_action=spawn_eff, on_exit=[ident])), fin]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS] + [OpaqueFunction(function=_setup)])
