"""Pase N1/N2 con la montura en ros2_control (sin Gazebo), en lockstep determinista.

  ros2 launch atp_control atp_ros2_control.launch.py el_max:=85 mask:=70 time_scale:=0
  ros2 launch atp_control atp_ros2_control.launch.py controller:=lqr estimator:=kalman
  ros2 launch atp_control atp_ros2_control.launch.py backend:=serial          (MCU falso)
  ros2 launch atp_control atp_ros2_control.launch.py backend:=serial serial_port:=/dev/ttyUSB0

Es el lazo de atp_loop.launch.py con la montura y el controlador cambiados por
la via estandar: en lugar de mount_sim + pid_controller/lqr_controller (nodos
con topicos), un controller_manager con el plugin AtpMountSystem y el
AtpTrackingController. El generador de referencia, el reloj, el enlace y el
monitor de error son LOS MISMOS de atp_ros. `lockstep_node` mueve el
controller_manager con /clock (CONTRATO 8); con mode:=free no espera ni confirma.

Argumentos como los de atp_loop (el_max, mask, controller, estimator, planta
realista, record_dir...), mas:
  backend:=sim|serial   hardware (hw:= del xacro); serial usa un MCU falso en proceso
  mode:=lockstep|free|realtime   lockstep (determinista), reloj libre a time_scale (sim_clock
                        sin confirmaciones) o ros2_control_node estandar con reloj de pared a 100 Hz
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

ARGS = (
    ('profile', 'pass'), ('el_max', '85.0'), ('mask', '70.0'),
    ('ramp_deg_s', '5.0'), ('ramp_duration', '20.0'), ('h_km', '550.0'),
    ('az_culmination', '140.0'),
    ('u_max', '8.0'), ('vel_max', '0.0'),
    ('kp', '25.0'), ('ki', '2.0'), ('kd', '8.0'),
    ('band', 'S'), ('eirp_dbw', '9.0'),
    ('time_scale', '0.0'), ('exit_on_done', 'true'), ('record_dir', ''),
    ('controller', 'pid'), ('estimator', 'none'),
    ('q_theta', '10000.0'), ('q_omega', '1.0'), ('q_int', '1000.0'), ('r_u', '1.0'),
    ('kf_q_acc', '0.0001'), ('kf_q_par', '0.01'), ('noise_deg', '0.0'),
    # planta realista (todo apagado, igual que atp_loop)
    ('backlash_deg', '0.0'), ('coulomb_nm', '0.0'), ('stribeck_nm', '0.0'),
    ('unbalance_nm', '0.0'), ('encoder_bits', '0'),
    # hardware y modo
    ('backend', 'sim'), ('serial_port', 'fake'), ('serial_baud', '921600'),
    ('mode', 'lockstep'), ('cm_rate', '100'), ('step_s', '0.05'), ('min_step_wall_s', '0.01'), ('trace_file', ''), ('eval_file', ''),
)


def _inicio(profile, h_km, el_max, mask, az_cul, ramp_az=10.0, ramp_el=30.0):
    """Posicion (rad) de la referencia en t = 0: donde arranca la montura (enganche en AOS)."""
    import math

    if profile == 'ramp':
        return math.radians(ramp_az), math.radians(ramp_el)
    from atp_ros import model
    p = model.perfil_pase(h_km, el_max, mask, az_cul)
    return math.radians(float(p['az_continuo'][0])), math.radians(float(p['el'][0]))


def _setup(context):
    c = lambda k: LC(k).perform(context)   # noqa: E731
    f = lambda k: float(c(k))              # noqa: E731
    if c('profile') not in ('pass', 'ramp'):
        raise RuntimeError('profile debe ser pass o ramp (el perfil tle no esta cableado aqui)')
    modo = c('mode')
    if modo not in ('lockstep', 'free', 'realtime'):
        raise RuntimeError('mode debe ser lockstep, free o realtime')
    lockstep = modo == 'lockstep'
    realtime = modo == 'realtime'
    az0, el0 = _inicio(c('profile'), f('h_km'), f('el_max'), f('mask'), f('az_culmination'))

    # --- robot_description desde el xacro: aqui se elige el hardware (hw:=)
    xacro_path = os.path.join(get_package_share_directory('atp_description'),
                              'urdf', 'antena.urdf.xacro')
    urdf = xacro.process_file(xacro_path, mappings={
        'hw': c('backend'), 'az0': repr(az0), 'el0': repr(el0), 'u_max': c('u_max'),
        'vel_max_deg_s': c('vel_max'), 'coulomb_nm': c('coulomb_nm'),
        'stribeck_nm': c('stribeck_nm'), 'backlash_deg': c('backlash_deg'),
        'unbalance_nm': c('unbalance_nm'), 'encoder_bits': c('encoder_bits'),
        'serial_port': c('serial_port'), 'serial_baud': c('serial_baud')}).toxml()

    # --- parametros de los controladores: el yaml base + lo que pide el usuario
    base = os.path.join(get_package_share_directory('atp_control'), 'config',
                        'atp_controllers.yaml')
    with open(base) as fh:
        cfg = yaml.safe_load(fh)
    cfg['controller_manager']['ros__parameters']['update_rate'] = (
        int(c('cm_rate')) if realtime else int(round(1.0 / f('step_s'))))
    t = cfg['atp_tracking_controller']['ros__parameters']
    t.update({'law': c('controller'), 'estimator': c('estimator'), 'u_max': f('u_max'),
              'kp': f('kp'), 'ki': f('ki'), 'kd': f('kd'),
              'q_theta': f('q_theta'), 'q_omega': f('q_omega'), 'q_int': f('q_int'),
              'r_u': f('r_u'), 'kf_q_acc': f('kf_q_acc'), 'kf_q_par': f('kf_q_par'),
              'enc_sigma_deg': f('noise_deg'), 'enc_bits': int(c('encoder_bits')),
              'rate_hz': 1.0 / f('step_s'), 'sync_publish': lockstep, 'trace_file': c('trace_file')})
    fd, ruta = tempfile.mkstemp(prefix='atp_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as fh:
        yaml.safe_dump(cfg, fh)

    sim = {'use_sim_time': not realtime}
    grabando = c('record_dir') != ''
    nodos_espera = ['link_budget', 'error_monitor', 'pointing_eval']
    if not realtime:
        nodos_espera = ['sim_clock', 'lockstep_runner'] + nodos_espera
    if grabando:
        nodos_espera.append('recorder')
    clock = Node(package='atp_ros', executable='sim_clock', parameters=[{
        'time_scale': f('time_scale'), 'lockstep': lockstep, 'step_s': f('step_s')}])
    ref = Node(package='atp_ros', executable='reference_generator', parameters=[sim, {
        'profile': c('profile'), 'el_max_deg': f('el_max'), 'mask_deg': f('mask'),
        'h_km': f('h_km'), 'az_culmination_deg': f('az_culmination'),
        'ramp_deg_s': f('ramp_deg_s'), 'ramp_duration_s': f('ramp_duration'),
        'wait_subscribers': 4 if grabando else 3,
        'wait_nodes': ','.join(nodos_espera)}])
    if realtime:
        # ros2_control_node ESTANDAR con reloj de pared: el mismo plugin y controlador
        # sin nuestro lockstep (lo que correria en un banco real); el URDF le llega
        # por /robot_description (robot_state_publisher), como en Jazzy
        rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
                   parameters=[sim, {'robot_description': ParameterValue(urdf, value_type=str)}])
        runner = [rsp, Node(package='controller_manager', executable='ros2_control_node',
                            output='screen', parameters=[ruta, sim])]
    else:
        runner = [Node(package='atp_control', executable='lockstep_node', output='screen',
                       parameters=[ruta, sim, {
                           'robot_description': ParameterValue(urdf, value_type=str),
                           'lockstep': lockstep, 'min_step_wall_s': f('min_step_wall_s'),
                           'expected_controllers': ','.join(['joint_state_broadcaster',
                                                            'atp_tracking_controller'])}])]
    # Primero el controlador y al final el joint_state_broadcaster: mientras el
    # arranque gira update() sin reloj, un broadcaster activo publicaria cientos de
    # /joint_states sin pareja y el monitor pasaria a emparejar con "la ultima".
    spawn_trk = Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=['atp_tracking_controller', '--controller-manager',
                                '/controller_manager', '--controller-manager-timeout', '60'])
    spawn_jsb = Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=['joint_state_broadcaster', '--controller-manager',
                                '/controller_manager', '--controller-manager-timeout', '60'])
    en_cadena = RegisterEventHandler(OnProcessExit(target_action=spawn_trk, on_exit=[spawn_jsb]))
    link = Node(package='atp_ros', executable='link_budget', parameters=[sim, {
        'band': c('band'), 'eirp_dbw': f('eirp_dbw')}])
    mon = Node(package='atp_ros', executable='error_monitor', parameters=[sim, {
        'exit_on_done': c('exit_on_done') == 'true' and not grabando}])
    evalu = Node(package='atp_control', executable='pointing_eval.py',
                 parameters=[sim, {'out_file': c('eval_file')}])
    acciones = ([] if realtime else [clock]) + [ref] + runner + [spawn_trk, en_cadena, link, mon, evalu]
    if grabando:
        rec = Node(package='atp_ros', executable='recorder',
                   parameters=[sim, {'output_dir': c('record_dir')}])
        acciones += [rec, RegisterEventHandler(OnProcessExit(
            target_action=rec, on_exit=[EmitEvent(event=Shutdown())]))]
    else:
        acciones.append(RegisterEventHandler(OnProcessExit(
            target_action=mon, on_exit=[EmitEvent(event=Shutdown())])))
    return acciones


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS] + [OpaqueFunction(function=_setup)])
