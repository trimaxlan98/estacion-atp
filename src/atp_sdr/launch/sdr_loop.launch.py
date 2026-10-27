"""Lazo ATP con el receptor SDR a nivel IQ, en tiempo simulado y lockstep.

N2 (pase cenital, banda S, Doppler de la geometria esferica):
  ros2 launch atp_sdr sdr_loop.launch.py el_max:=85 mask:=70 time_scale:=0
N4 (pase oblicuo completo):
  ros2 launch atp_sdr sdr_loop.launch.py el_max:=30 mask:=5 time_scale:=0 band:=Ka
S4 (pase REAL de la ISS sobre CDMX con SGP4):
  ros2 launch atp_sdr sdr_loop.launch.py profile:=tle sat:=ISS station:=cdmx \\
      pass_index:=0 start_utc:=2026-10-02T15:00:00 time_scale:=0 band:=S
Con BPSK y defectos del oscilador:
  ros2 launch atp_sdr sdr_loop.launch.py mode:=bpsk offset_lo_hz:=2200 lw_lo_hz:=0.5
Enlace degradado (el receptor trabaja cerca de su umbral): delta_cn0_db:=-45

Es el lazo de atp_loop.launch.py (reloj, referencia, montura, PID, enlace,
monitor) mas `sdr_receiver` y, por defecto (`gate:=true`), la compuerta
`sdr_gate`: el reloj espera al receptor en cada paso, asi la corrida es
determinista aunque el receptor sea mas lento que la montura. `gate:=false` lo
deja pasivo (CONTRATO 8.3): a maxima velocidad el reloj corre mas que el receptor
y, con la maquina cargada, DDS puede perder pasos (el resumen cuenta los huecos).

No se reutiliza atp_loop con IncludeLaunchDescription porque el saludo de
arranque (CONTRATO 8) exige que `sdr_receiver` y `sdr_gate` figuren en
`wait_nodes` del generador de referencia, y porque el lanzamiento lo cierra el
propio receptor tras su resumen.
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
        ('profile', 'pass'), ('el_max', '85.0'), ('mask', '70.0'),
        ('u_max', '8.0'), ('kp', '25.0'), ('kd', '8.0'),
        ('band', 'S'), ('eirp_dbw', '9.0'), ('time_scale', '0.0'),
        # perfil tle (Sprint 4)
        ('sat', 'ISS'), ('station', ''), ('pass_index', '0'),
        ('start_utc', ''), ('tle_mask', '-1.0'), ('h_km', '550.0'),
        # receptor SDR
        ('mode', 'portadora'), ('seed', '1'), ('order', '3'), ('bn_hz', '0.0'),
        ('bn_fll_hz', '10.0'), ('offset_lo_hz', '0.0'), ('lw_lo_hz', '0.0'),
        ('delta_cn0_db', '0.0'), ('symbol_rate', '12500.0'), ('gate', 'true'))]
    sim = {'use_sim_time': True}
    clock = Node(package='atp_ros', executable='sim_clock',
                 parameters=[{'time_scale': LC('time_scale')}])
    ref = Node(package='atp_ros', executable='reference_generator', parameters=[sim, {
        'profile': LC('profile'),
        'el_max_deg': LC('el_max'), 'mask_deg': LC('mask'),
        'tle_name': ParameterValue(LC('sat'), value_type=str),
        'station': ParameterValue(LC('station'), value_type=str),
        'start_utc': ParameterValue(LC('start_utc'), value_type=str),
        'pass_index': LC('pass_index'), 'tle_mask_deg': LC('tle_mask'),
        # monitor + enlace + receptor SDR suscritos a /atp/sat_state
        'wait_subscribers': 3,
        'wait_nodes': PythonExpression([
            "'sim_clock,mount_sim,pid_controller,link_budget,error_monitor,sdr_receiver'"
            " + (',sdr_gate' if '", LC('gate'), "' == 'true' else '')"]),
    }])
    # con la compuerta, la confirmacion de la montura va a /atp/step_ack_raw y
    # sdr_gate la reemite como /atp/step_ack cuando el receptor termino el paso
    con_gate = IfCondition(PythonExpression(["'", LC('gate'), "' == 'true'"]))
    sin_gate = UnlessCondition(PythonExpression(["'", LC('gate'), "' == 'true'"]))
    mpar = [sim, {'u_max': LC('u_max'), 'vel_max_deg_s': 0.0}]
    mount = Node(package='atp_ros', executable='mount_sim', parameters=mpar,
                 condition=con_gate, remappings=[('/atp/step_ack', '/atp/step_ack_raw')])
    mount_libre = Node(package='atp_ros', executable='mount_sim', parameters=mpar,
                       condition=sin_gate)
    gate = Node(package='atp_sdr', executable='sdr_gate', condition=con_gate,
                parameters=[sim])
    pid = Node(package='atp_ros', executable='pid_controller', parameters=[
        sim, {'kp': LC('kp'), 'kd': LC('kd'), 'u_max': LC('u_max')}])
    link = Node(package='atp_ros', executable='link_budget', parameters=[sim, {
        'band': LC('band'), 'eirp_dbw': LC('eirp_dbw')}])
    mon = Node(package='atp_ros', executable='error_monitor',
               parameters=[sim, {'exit_on_done': False}])
    sdr = Node(package='atp_sdr', executable='sdr_receiver', parameters=[sim, {
        'band': ParameterValue(LC('band'), value_type=str),
        'mode': ParameterValue(LC('mode'), value_type=str),
        'seed': LC('seed'), 'order': LC('order'), 'bn_hz': LC('bn_hz'),
        'bn_fll_hz': LC('bn_fll_hz'), 'offset_lo_hz': LC('offset_lo_hz'),
        'lw_lo_hz': LC('lw_lo_hz'), 'delta_cn0_db': LC('delta_cn0_db'),
        'symbol_rate': LC('symbol_rate'), 'h_km': LC('h_km'),
        'eirp_dbw': LC('eirp_dbw'), 'exit_on_done': True}])
    # el lanzamiento lo cierra el receptor, al terminar de procesar el pase
    stop = RegisterEventHandler(OnProcessExit(
        target_action=sdr, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [clock, ref, mount, mount_libre, gate, pid, link,
                                     mon, sdr, stop])
