"""Autotrack por conical scan con la potencia medida sobre la AMPLITUD IQ.

  ros2 launch atp_sdr sdr_autotrack.launch.py beacon:=sdr   # amplitud del bloque IQ
  ros2 launch atp_sdr sdr_autotrack.launch.py beacon:=rf    # potencia ideal en dB + ruido (rf_beacon)
  ros2 launch atp_sdr sdr_autotrack.launch.py beacon:=sdr autotrack:=false   # program track puro
  ros2 launch atp_sdr sdr_autotrack.launch.py beacon:=sdr delta_cn0_db:=-35  # enlace degradado

Es atp_autotrack.launch.py con `sdr_beacon` (o `rf_beacon`) a eleccion. El resto
del lazo es el mismo (autotrack de atp_ros SIN CAMBIOS): el discriminador del
conical scan recibe /atp/beacon con position = [P_dB, fase_scan]; lo unico que
cambia es de donde sale P_dB. Misma verdad (efemeride 0.1 s atrasada y sesgo
mecanico) y mismo escenario N5, asi que las cifras son comparables.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in (
        ('beacon', 'sdr'), ('autotrack', 'true'), ('band', 'Ka'), ('el_max', '60.0'),
        ('mask', '20.0'), ('ephem_dt_s', '0.1'), ('bias_xel_deg', '0.10'),
        ('bias_el_deg', '-0.08'), ('noise_db', '0.2'), ('time_scale', '0.0'),
        ('delta_cn0_db', '0.0'), ('seed', '5'))]
    sim = {'use_sim_time': True}
    to_program = [('/atp/reference', '/atp/program')]
    to_truth = [('/atp/reference', '/atp/truth')]
    es_sdr = PythonExpression(["'", LC('beacon'), "' == 'sdr'"])
    es_rf = PythonExpression(["'", LC('beacon'), "' == 'rf'"])
    nodes = [
        Node(package='atp_ros', executable='sim_clock',
             parameters=[{'time_scale': LC('time_scale')}]),
        Node(package='atp_ros', executable='reference_generator', remappings=to_program,
             parameters=[sim, {'profile': 'pass', 'el_max_deg': LC('el_max'),
                               'mask_deg': LC('mask'),
                               'wait_subscribers': PythonExpression([
                                   "3 if '", LC('beacon'), "' == 'sdr' else 2"]),
                               'wait_nodes': PythonExpression([
                                   "'sim_clock,' + ('sdr_beacon' if '", LC('beacon'),
                                   "' == 'sdr' else 'rf_beacon') + ',autotrack,mount_sim,"
                                   "pid_controller,link_budget,error_monitor'"])}]),
        Node(package='atp_ros', executable='rf_beacon', condition=IfCondition(es_rf),
             parameters=[sim, {'band': LC('band'), 'ephem_dt_s': LC('ephem_dt_s'),
                               'bias_xel_deg': LC('bias_xel_deg'),
                               'bias_el_deg': LC('bias_el_deg'),
                               'noise_db': LC('noise_db')}]),
        Node(package='atp_sdr', executable='sdr_beacon', condition=IfCondition(es_sdr),
             parameters=[sim, {'band': LC('band'), 'ephem_dt_s': LC('ephem_dt_s'),
                               'bias_xel_deg': LC('bias_xel_deg'),
                               'bias_el_deg': LC('bias_el_deg'),
                               'delta_cn0_db': LC('delta_cn0_db'), 'seed': LC('seed')}]),
        Node(package='atp_ros', executable='autotrack', parameters=[sim, {
            'enabled': LC('autotrack'), 'band': LC('band')}]),
        Node(package='atp_ros', executable='mount_sim', parameters=[sim]),
        Node(package='atp_ros', executable='pid_controller', parameters=[sim]),
        Node(package='atp_ros', executable='link_budget',
             parameters=[sim, {'band': LC('band')}]),
    ]
    mon = Node(package='atp_ros', executable='error_monitor', remappings=to_truth,
               parameters=[sim, {'exit_on_done': True}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=mon, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + nodes + [mon, stop])
