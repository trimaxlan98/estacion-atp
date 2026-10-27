"""Escenario ADCS completo DETUMBLE -> NADIR -> TRACK_GS en lockstep.

    ros2 launch sat_adcs adcs.launch.py time_scale:=0
    ros2 launch sat_adcs adcs.launch.py time_scale:=0 start_utc:=2026-10-02T08:30:00 lead_s:=3600
    ros2 launch sat_adcs adcs.launch.py time_scale:=0 model_degree:=1     # dipolo a bordo
    ros2 launch sat_adcs adcs.launch.py time_scale:=0 band:=S d_sat:=0.1 w0_deg_s:=5

`sim_clock` es el de atp_ros (lockstep, CONTRATO sec. 8): `step_s` es el
periodo del lazo ADCS (1 s por omision) y `time_scale:=0` corre lo mas rapido
posible. El escenario sigue el pase real `pass_index` de `sat` sobre `station`
a partir de `start_utc` (SGP4 con data/tle, estaciones de data/estaciones.yaml).
`record_dir:=<carpeta>` (Estacion) agrega el grabador `adcs_recorder`, que escribe
telemetry.jsonl y metrics.json (CONTRATO 2.1 y 3.1) y, al cerrar, apaga el
lanzamiento; sin el todo queda como estaba (el monitor apaga el lanzamiento).
`model_degree` es el grado del campo magnetico de a bordo (1 = dipolo inclinado,
13 = IGRF completo, la verdad de la simulacion). El resto de los parametros de
`sat_adcs.misiones.Config` se fija con `--ros-args -p` en cada nodo.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in (
        ('sat', 'FUNCUBE-1'), ('station', 'cdmx'),
        ('start_utc', '2026-10-02T20:30:00'), ('pass_index', '0'),
        ('el_max_min', '20.0'), ('lead_s', '3600.0'),
        ('seed', '7'), ('w0_deg_s', '8.0'), ('step_s', '1.0'), ('time_scale', '1.0'),
        ('model_degree', '4'), ('band', 'X'), ('d_sat', '0.3'),
        # Estacion: carpeta de la corrida para el grabador ('' = sin grabar)
        ('record_dir', ''))]
    p = {
        'sat': ParameterValue(LC('sat'), value_type=str),
        'estacion': ParameterValue(LC('station'), value_type=str),
        'inicio_utc': ParameterValue(LC('start_utc'), value_type=str),
        'banda': ParameterValue(LC('band'), value_type=str),
        'pase_idx': LC('pass_index'), 'el_max_min_deg': LC('el_max_min'),
        'anticipo_min_s': LC('lead_s'), 'semilla': LC('seed'),
        'w0_deg_s': LC('w0_deg_s'), 'paso_s': LC('step_s'),
        'modelo_bordo_nmax': LC('model_degree'), 'd_sat_m': LC('d_sat'),
    }
    sim = {'use_sim_time': True}
    grabando = PythonExpression(["'", LC('record_dir'), "' != ''"])
    # la planta (suscriptores del estado) y el controlador (suscriptores del mando)
    # esperan al grabador ademas de al monitor: CONTRATO 8, regla 4
    espera = {'wait_subscribers': PythonExpression(["3 if '", LC('record_dir'), "' else 2"])}
    clock = Node(package='atp_ros', executable='sim_clock',
                 parameters=[{'time_scale': LC('time_scale'), 'step_s': LC('step_s')}])
    planta = Node(package='sat_adcs', executable='adcs_sim', parameters=[sim, p, espera])
    ctrl = Node(package='sat_adcs', executable='adcs_controller', parameters=[sim, p, espera])
    # Con grabador, quien apaga el lanzamiento es el grabador al escribir metrics.json;
    # el monitor imprime el RESUMEN pero no sale antes (sin carrera con la ultima escritura)
    mon = Node(package='sat_adcs', executable='adcs_monitor', parameters=[sim, p, {
        'exit_on_done': PythonExpression(["'", LC('record_dir'), "' == ''"])}])
    rec = Node(package='sat_adcs', executable='adcs_recorder', condition=IfCondition(grabando),
               parameters=[sim, p, {'output_dir': ParameterValue(LC('record_dir'), value_type=str)}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=mon, on_exit=[EmitEvent(event=Shutdown(), condition=UnlessCondition(grabando))]))
    stop_rec = RegisterEventHandler(OnProcessExit(target_action=rec,
                                                  on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [clock, planta, ctrl, mon, rec, stop, stop_rec])
