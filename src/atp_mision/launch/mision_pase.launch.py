"""Un contacto de la mision integrada: un pase REAL (SGP4) en lockstep con el
satelite VERDADERO distinto de lo que el operador predice (docs/MISION.md).

  ros2 launch atp_mision mision_pase.launch.py sat:=ISS station:=cdmx \\
      start_utc:=2026-10-02T15:51:30 time_scale:=0 band:=S record_dir:=/tmp/p1 \\
      truth_file:=/tmp/mundo/ISS.npz [profile:=od od_file:=od.json] \\
      [autotrack:=true] [sdr:=true]

Lo que lo distingue de `atp_loop.launch.py` / `atp_autotrack.launch.py`:
  * `reference_generator` predice el pase con el TLE (`profile:=tle`) o con la
    orbita estimada (`profile:=od od_file:=...`) y publica esa prediccion como
    referencia; el satelite que ve la estacion es el de `truth_file` (sat_state
    y /atp/truth). El monitor mide el apuntamiento contra la VERDAD y el enlace
    usa el error y el rango de la verdad.
  * `autotrack:=true` pone el program track en /atp/program y deja que el
    conical scan (rf_beacon con verdad externa + autotrack) lo corrija: la
    montura sigue al satelite verdadero si el error de efemeride cae dentro del
    cono de captura; si no, no engancha y queda en program track.
  * `sdr:=true` suma el receptor IQ con la compuerta del reloj (como
    `atp_sdr/sdr_loop.launch.py`).
  * Siempre graba (`record_dir`): el grabador cierra el lanzamiento.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = (
    ('sat', 'ISS'), ('station', ''), ('pass_index', '0'), ('start_utc', ''),
    ('tle_mask', '-1.0'), ('tle_search_hours', '0.5'),
    ('profile', 'tle'), ('od_file', ''), ('truth_file', ''),
    ('band', 'S'), ('eirp_dbw', '9.0'),
    ('autotrack', 'false'), ('max_corr_deg', '1.0'), ('noise_db', '0.2'),
    ('sdr', 'false'), ('sdr_seed', '1'),
    ('vel_max', '6.0'), ('u_max', '8.0'), ('kp', '25.0'), ('kd', '8.0'),
    ('time_scale', '0.0'), ('record_dir', ''),
)


def _lanzar(context, *_):
    v = {k: LC(k).perform(context) for k, _ in ARGS}
    auto = v['autotrack'] == 'true'
    sdr = v['sdr'] == 'true'
    if not v['record_dir']:
        raise RuntimeError('mision_pase: falta record_dir')
    sim = {'use_sim_time': True}
    texto = lambda x: ParameterValue(x, value_type=str)    # noqa: E731

    esperan = ['sim_clock', 'mount_sim', 'pid_controller', 'link_budget',
               'error_monitor', 'recorder']
    if auto:
        esperan += ['rf_beacon', 'autotrack']
    if sdr:
        esperan += ['sdr_receiver', 'sdr_gate']
    ref = Node(
        package='atp_ros', executable='reference_generator',
        # con autotrack la prediccion va a /atp/program y el conical scan la corrige
        remappings=[('/atp/reference', '/atp/program')] if auto else [],
        parameters=[sim, {
            'profile': texto(v['profile']),
            'tle_name': texto(v['sat']), 'station': texto(v['station']),
            'start_utc': texto(v['start_utc']), 'od_file': texto(v['od_file']),
            'truth_file': texto(v['truth_file']),
            'pass_index': int(v['pass_index']), 'tle_mask_deg': float(v['tle_mask']),
            'tle_search_hours': float(v['tle_search_hours']),
            # monitor + enlace + grabador (+ receptor SDR) suscritos a /atp/sat_state
            'wait_subscribers': 3 + (1 if sdr else 0),
            'wait_nodes': ','.join(esperan)}])
    nodos = [
        Node(package='atp_ros', executable='sim_clock',
             parameters=[{'time_scale': float(v['time_scale'])}]),
        ref,
        Node(package='atp_ros', executable='mount_sim',
             # con el receptor, la confirmacion pasa por la compuerta del reloj
             remappings=[('/atp/step_ack', '/atp/step_ack_raw')] if sdr else [],
             parameters=[sim, {'u_max': float(v['u_max']),
                               'vel_max_deg_s': float(v['vel_max'])}]),
        Node(package='atp_ros', executable='pid_controller',
             parameters=[sim, {'kp': float(v['kp']), 'kd': float(v['kd']),
                               'u_max': float(v['u_max'])}]),
        Node(package='atp_ros', executable='link_budget',
             parameters=[sim, {'band': texto(v['band']),
                               'eirp_dbw': float(v['eirp_dbw'])}]),
        # el monitor mide contra la VERDAD, no contra la referencia que sigue la montura
        Node(package='atp_ros', executable='error_monitor',
             remappings=[('/atp/reference', '/atp/truth')],
             parameters=[sim, {'exit_on_done': False}]),
    ]
    if auto:
        nodos += [
            Node(package='atp_ros', executable='rf_beacon', parameters=[sim, {
                'band': texto(v['band']), 'external_truth': True,
                'noise_db': float(v['noise_db'])}]),
            Node(package='atp_ros', executable='autotrack', parameters=[sim, {
                'enabled': True, 'band': texto(v['band']),
                'max_corr_deg': float(v['max_corr_deg'])}]),
        ]
    if sdr:
        nodos += [
            Node(package='atp_sdr', executable='sdr_gate', parameters=[sim]),
            # no cierra el lanzamiento (exit_on_done false): lo cierra el grabador,
            # que escribe metrics.json despues del RESUMEN SDR
            Node(package='atp_sdr', executable='sdr_receiver', parameters=[sim, {
                'band': texto(v['band']), 'mode': 'portadora',
                'seed': int(v['sdr_seed']), 'eirp_dbw': float(v['eirp_dbw']),
                'exit_on_done': False}]),
        ]
    rec = Node(package='atp_ros', executable='recorder',
               parameters=[sim, {'output_dir': texto(v['record_dir'])}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=rec, on_exit=[EmitEvent(event=Shutdown())]))
    return nodos + [rec, stop]


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=d) for k, d in ARGS]
    return LaunchDescription(args + [OpaqueFunction(function=_lanzar)])
