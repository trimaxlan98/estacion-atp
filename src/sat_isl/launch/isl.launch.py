"""Enlace optico entre dos satelites vecinos de una constelacion Walker, en lockstep.

    ros2 launch sat_isl isl.launch.py time_scale:=0
    ros2 launch sat_isl isl.launch.py time_scale:=0 sat_b:=18 start_s:=400 dur_s:=40     # par de planos distintos
    ros2 launch sat_isl isl.launch.py time_scale:=0 detector:=apd                        # APD de deteccion directa
    ros2 launch sat_isl isl.launch.py time_scale:=0 sigma_pos_km:=0.036                  # efemerides del OD (docs/OD.md)

Constelacion Walker `n_sats`/`planos`/`fase` (24/3/1 a 800 km y 55 deg por omision),
propagada por `atp_ros.propagador` (J2). El par es (`sat_a`, `sat_b`), indices
plano-mayor (8 por plano: 0 = P0S0, 1 = P0S1, 18 = P2S2). `start_s` es el segundo de la
propagacion en que empieza la corrida y `dur_s` su duracion. `sim_clock` es el de
atp_ros (CONTRATO sec. 8): `step_s` es el periodo del lockstep (0.05 s); los lazos finos
de 10 kHz corren dentro de la planta. Al final el monitor imprime el RESUMEN y apaga el
lanzamiento. El resto de `sat_isl.misiones.Config` se fija con `--ros-args -p` en cada nodo.
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
        ('n_sats', '24'), ('planos', '3'), ('fase', '1'), ('h_km', '800.0'), ('inc_deg', '55.0'),
        ('sat_a', '0'), ('sat_b', '1'), ('start_s', '600.0'), ('dur_s', '60.0'),
        ('step_s', '0.05'), ('seed', '7'), ('time_scale', '1.0'),
        ('detector', 'coherente'), ('d_m', '0.05'), ('p_tx_w', '1.0'),
        ('theta_baliza_urad', '300.0'), ('sigma_pos_km', '2.1'), ('sigma_actitud_urad', '145.0'))]
    p = {
        'n_sats': LC('n_sats'), 'planos': LC('planos'), 'fase': LC('fase'),
        'h_km': LC('h_km'), 'inc_deg': LC('inc_deg'),
        'sat_a': LC('sat_a'), 'sat_b': LC('sat_b'), 'inicio_s': LC('start_s'),
        'dur_s': LC('dur_s'), 'paso_s': LC('step_s'), 'semilla': LC('seed'),
        'detector': ParameterValue(LC('detector'), value_type=str),
        'd_m': LC('d_m'), 'p_tx_w': LC('p_tx_w'), 'theta_baliza_urad': LC('theta_baliza_urad'),
        'sigma_pos_km': LC('sigma_pos_km'), 'sigma_actitud_urad': LC('sigma_actitud_urad'),
    }
    sim = {'use_sim_time': True}
    espera = {'wait_subscribers': 2}       # planta: controlador + monitor; controlador: planta + monitor
    clock = Node(package='atp_ros', executable='sim_clock',
                 parameters=[{'time_scale': LC('time_scale'), 'step_s': LC('step_s')}])
    planta = Node(package='sat_isl', executable='isl_sim', parameters=[sim, p, espera])
    ctrl = Node(package='sat_isl', executable='isl_controller', parameters=[sim, p, espera])
    mon = Node(package='sat_isl', executable='isl_monitor', parameters=[sim, p, {'exit_on_done': True}])
    stop = RegisterEventHandler(OnProcessExit(target_action=mon, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [clock, planta, ctrl, mon, stop])
