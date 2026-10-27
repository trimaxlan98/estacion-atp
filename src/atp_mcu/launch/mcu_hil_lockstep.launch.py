"""Banco HIL en LOCKSTEP: el firmware (o `eje_host` de Linux) en lugar de la montura simulada.

Ensayo general del primer encendido: cuando llegue la placa, cambia el transporte
(agente UDP/serie) y nada mas. Los nodos de la Estacion son los de siempre
(`sim_clock` en lockstep, `reference_generator`, `error_monitor`); `mcu_lockstep`
ocupa el lugar de `mount_sim` + `pid_controller` (ver su docstring).

El agente micro-ROS y el MCU se arrancan aparte, con el MISMO ROS_DOMAIN_ID
(firmware/esp32_eje/scripts/prueba_hil_lockstep.sh lo hace todo con eje_host).

  ros2 launch atp_mcu mcu_hil_lockstep.launch.py el_max:=85 mask:=70 salida:=/tmp/hil
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in (
        ('profile', 'pass'), ('el_max', '85.0'), ('mask', '70.0'),
        ('ramp_deg_s', '5.0'), ('ramp_duration', '20.0'),
        ('usar_eco', 'true'), ('origen', 'cmd'), ('salida', ''),
        ('exit_on_done', 'true'))]
    sim = {'use_sim_time': True}
    # time_scale 0: el ritmo lo marca la confirmacion de mcu_lockstep (tiempo real del MCU)
    clock = Node(package='atp_ros', executable='sim_clock',
                 parameters=[{'time_scale': 0.0, 'lockstep': True}])
    ref = Node(package='atp_ros', executable='reference_generator', parameters=[sim, {
        'profile': LC('profile'), 'el_max_deg': LC('el_max'), 'mask_deg': LC('mask'),
        'ramp_deg_s': LC('ramp_deg_s'), 'ramp_duration_s': LC('ramp_duration'),
        'wait_subscribers': 1,
        'wait_nodes': 'sim_clock,mcu_lockstep,error_monitor'}])
    puente = Node(package='atp_mcu', executable='mcu_lockstep', parameters=[sim, {
        'usar_eco': ParameterValue(LC('usar_eco'), value_type=bool),
        'origen': ParameterValue(LC('origen'), value_type=str),
        'salida': ParameterValue(LC('salida'), value_type=str)}])
    mon = Node(package='atp_ros', executable='error_monitor', parameters=[sim, {
        'exit_on_done': LC('exit_on_done')}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=mon, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [clock, ref, puente, mon, stop])
