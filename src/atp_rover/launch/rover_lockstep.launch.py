"""Rover en LOCKSTEP (CONTRATO sec. 8 y 18; docs/ROVER.md «Resultados R9»): sim_clock + rover_dinamica + rover_navegador
(+ rover_enlace si la escena tiene enlace). Misma escena con y sin ROS: mismos bytes.

    ros2 launch atp_rover rover_lockstep.launch.py time_scale:=0 salida:=/tmp/rover id:=l1 \\
        cfg:='{"planeta":"luna","terreno":"rocoso","semilla":7,"ruta":[[4,0],[4,3]],"duracion_s":20,"navegacion":"completa"}'
    ros2 launch atp_rover rover_lockstep.launch.py escena:=experiments/rover-lockstep-luna.yaml time_scale:=2

Argumentos: `escena` (YAML de experiments/ con `args`, opcional), `cfg` (JSON que pisa a la escena: claves de
`atp_rover.cli.DEFECTOS` y `atp_rover.lockstep.DEFECTOS_LOCK`: `enlace` = ninguno|ideal|retardo|ventanas, `operador`
= navegador|guion, `guion` = [[t, v, w], ...], `rele`, `fecha`, `t0_s`, ...), `time_scale` (0 = lo mas rapido posible;
2 = el doble del tiempo real), `salida` e `id` (escribe <salida>/<id>/telemetry.jsonl, metrics.json, mandos.jsonl).
`sim_clock` avanza con step_s = 0.1 s (el intervalo de decision y de muestreo; la dinamica integra a 0.01 s dentro de cada tick).
Al terminar `rover_dinamica` imprime el RESUMEN y el lanzamiento se apaga.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodos(contexto):
    import json
    from atp_rover import lockstep
    cfg = lockstep.cargar_cfg(LC('escena').perform(contexto) or None, LC('cfg').perform(contexto) or '{}')
    cfg_txt = json.dumps(cfg, separators=(',', ':'))
    pcfg = ParameterValue(cfg_txt, value_type=str)
    sim = {'use_sim_time': True, 'cfg': pcfg, 'wait_subscribers': 1}
    reloj = Node(package='atp_ros', executable='sim_clock',
                 parameters=[{'time_scale': LC('time_scale'), 'step_s': lockstep.PASO_S, 'ack_timeout_s': 3600.0}])
    planta = Node(package='atp_rover', executable='rover_dinamica', name='rover_dinamica', output='screen',
                  parameters=[{**sim, 'modo': 'lockstep', 'salida': ParameterValue(LC('salida'), value_type=str),
                               'id': ParameterValue(LC('id'), value_type=str)}])
    nav = Node(package='atp_rover', executable='rover_navegador', name='rover_navegador', output='screen',
               parameters=[{**sim, 'modo': 'lockstep'}])
    nodos = [reloj, planta, nav]
    if cfg['enlace'] != 'ninguno':
        nodos.append(Node(package='atp_rover', executable='rover_enlace', name='rover_enlace', output='screen',
                          parameters=[{**sim, 'modo_nodo': 'lockstep'}]))
    parada = RegisterEventHandler(OnProcessExit(target_action=planta, on_exit=[EmitEvent(event=Shutdown())]))
    return nodos + [parada]


def generate_launch_description():
    args = [DeclareLaunchArgument('escena', default_value=''), DeclareLaunchArgument('cfg', default_value='{}'),
            DeclareLaunchArgument('time_scale', default_value='0'), DeclareLaunchArgument('salida', default_value='/tmp/rover'),
            DeclareLaunchArgument('id', default_value='rover')]
    return LaunchDescription(args + [OpaqueFunction(function=_nodos)])
