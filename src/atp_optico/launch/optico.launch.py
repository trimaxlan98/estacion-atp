"""ATP optico de la estacion en lockstep: la montura gruesa de atp_ros + el FSM, la turbulencia y el enlace de bajada.

    ros2 launch atp_optico optico.launch.py time_scale:=0
    ros2 launch atp_optico optico.launch.py time_scale:=0 sat:=ISS station:=cdmx pass_index:=3 D_m:=0.6 salida:=/ws/runs/optico
    ros2 launch atp_optico optico.launch.py time_scale:=0 campo_urad:=4 modem:=ppm

Arranca el lazo de la montura que ya existe (`sim_clock`, `reference_generator` con el perfil SGP4 de `sat` sobre
`station`, `mount_sim` con `publish_truth`, `pid_controller` y `error_monitor` con `use_truth`) y el nodo pasivo
`optico_atp`, que empareja por sello el error de la montura con el estado del satelite y corre los 10 kHz del FSM,
el tilt atmosferico con semilla y el enlace. El monitor imprime `RESUMEN OPTICO`, escribe `salida`/optico.json y
optico_serie.jsonl (si `salida` no esta vacio) y apaga el lanzamiento. Cada campo de
`atp_optico.escenario.ConfigOptico` (p_tx_w, theta_h_urad, D_m, campo_urad, modem, cn2_suelo, viento_rms, semilla...)
es un argumento. `start_utc` fija el pase (sin el, 'ahora').
"""
import dataclasses

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from atp_optico.escenario import ConfigOptico


def generate_launch_description():
    base = [('time_scale', '1.0'), ('sat', 'ISS'), ('pass_index', '0'), ('start_utc', '2026-10-02T15:00:00'),
            ('tle_mask', '10.0'), ('tle_search_hours', '72.0'), ('stations_file', ''), ('exit_on_done', 'true')]
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in base]
    cfg_campos = {}
    for f in dataclasses.fields(ConfigOptico):
        args.append(DeclareLaunchArgument(f.name, default_value=str(f.default)))
        tipo = type(f.default)
        cfg_campos[f.name] = ParameterValue(LC(f.name), value_type=tipo)
    sim = {'use_sim_time': True}
    clock = Node(package='atp_ros', executable='sim_clock', parameters=[{'time_scale': LC('time_scale')}])
    ref = Node(package='atp_ros', executable='reference_generator', parameters=[sim, {
        'profile': 'tle',
        'tle_name': ParameterValue(LC('sat'), value_type=str),
        'station': ParameterValue(LC('station'), value_type=str),
        'start_utc': ParameterValue(LC('start_utc'), value_type=str),
        'pass_index': LC('pass_index'), 'tle_mask_deg': LC('tle_mask'),
        'tle_search_hours': LC('tle_search_hours'),
        'stations_file': ParameterValue(LC('stations_file'), value_type=str),
        # error_monitor + optico_atp suscritos a /atp/sat_state; todos los nodos del lazo antes de arrancar
        'wait_subscribers': 2,
        'wait_nodes': 'sim_clock,mount_sim,pid_controller,error_monitor,optico_atp'}])
    mount = Node(package='atp_ros', executable='mount_sim', parameters=[sim, {'publish_truth': True}])
    pid = Node(package='atp_ros', executable='pid_controller', parameters=[sim])
    mon = Node(package='atp_ros', executable='error_monitor', parameters=[sim, {
        'use_truth': True, 'exit_on_done': False}])
    optico = Node(package='atp_optico', executable='optico_atp', parameters=[
        sim, cfg_campos, {'exit_on_done': ParameterValue(LC('exit_on_done'), value_type=bool)}])
    stop = RegisterEventHandler(OnProcessExit(target_action=optico, on_exit=[EmitEvent(event=Shutdown())]))
    # si el generador muere (p. ej. `pass_index` sin pase) no hay nada que esperar: se apaga todo
    stop_ref = RegisterEventHandler(OnProcessExit(target_action=ref, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [clock, ref, mount, pid, mon, optico, stop, stop_ref])
