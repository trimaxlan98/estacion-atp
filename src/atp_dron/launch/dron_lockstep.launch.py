"""Dron en LOCKSTEP (CONTRATO sec. 8 y 21; docs/DRON.md «D5»): sim_clock + dron_dinamica + dron_navegador. Misma escena con y sin
ROS: mismos bytes.

    ros2 launch atp_dron dron_lockstep.launch.py time_scale:=0 salida:=/tmp/dron id:=l1 \\
        cfg:='{"ruta":[[10,0,5],[10,10,5]],"duracion_s":30,"viento":[2,0,0],"rafaga_sigma":1.0,"semilla":3}'
    ros2 launch atp_dron dron_lockstep.launch.py escena:=experiments/dron-lockstep-ruta.yaml time_scale:=2

Argumentos: `escena` (YAML de experiments/ con `args`, opcional), `cfg` (JSON que pisa a la escena: claves de
`atp_dron.lazo.DEFECTOS`), `time_scale` (0 = lo mas rapido posible), `salida` e `id`. `sim_clock` avanza con step_s = 0.02 s (el
lazo externo de la cascada; la dinamica integra a 4 ms dentro de cada tick). Al terminar `dron_dinamica` imprime el RESUMEN y el
lanzamiento se apaga.
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
    from atp_dron import lockstep
    cfg = lockstep.cargar_cfg(LC('escena').perform(contexto) or None, LC('cfg').perform(contexto) or '{}')
    pcfg = ParameterValue(json.dumps(cfg, separators=(',', ':')), value_type=str)
    sim = {'use_sim_time': True, 'cfg': pcfg, 'wait_subscribers': 1}
    reloj = Node(package='atp_ros', executable='sim_clock',
                 parameters=[{'time_scale': LC('time_scale'), 'step_s': lockstep.PASO_S, 'ack_timeout_s': 3600.0}])
    planta = Node(package='atp_dron', executable='dron_dinamica', name='dron_dinamica', output='screen',
                  parameters=[{**sim, 'salida': ParameterValue(LC('salida'), value_type=str),
                               'id': ParameterValue(LC('id'), value_type=str)}])
    nav = Node(package='atp_dron', executable='dron_navegador', name='dron_navegador', output='screen', parameters=[sim])
    parada = RegisterEventHandler(OnProcessExit(target_action=planta, on_exit=[EmitEvent(event=Shutdown())]))
    return [reloj, planta, nav, parada]


def generate_launch_description():
    args = [DeclareLaunchArgument('escena', default_value=''), DeclareLaunchArgument('cfg', default_value='{}'),
            DeclareLaunchArgument('time_scale', default_value='0'), DeclareLaunchArgument('salida', default_value='/tmp/dron'),
            DeclareLaunchArgument('id', default_value='dron')]
    return LaunchDescription(args + [OpaqueFunction(function=_nodos)])
