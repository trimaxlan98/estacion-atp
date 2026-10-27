"""Program track contra autotrack (conical scan) sobre el mismo cielo.

  ros2 launch atp_ros atp_autotrack.launch.py autotrack:=false   # solo efemeride
  ros2 launch atp_ros atp_autotrack.launch.py autotrack:=true    # cierra con la RF

Flujo: reference_generator -> /atp/program -> autotrack -> /atp/reference
-> PID -> montura. rf_beacon fabrica la verdad (/atp/truth) y la senal; el
monitor y el enlace miden contra la VERDAD, no contra la efemeride.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in (
        ('autotrack', 'true'), ('band', 'Ka'), ('el_max', '60.0'), ('mask', '20.0'),
        ('ephem_dt_s', '0.1'), ('bias_xel_deg', '0.10'), ('bias_el_deg', '-0.08'),
        ('noise_db', '0.2'), ('time_scale', '0.0'), ('exit_on_done', 'true'),
        ('record_dir', ''))]
    sim = {'use_sim_time': True}
    to_program = [('/atp/reference', '/atp/program')]
    to_truth = [('/atp/reference', '/atp/truth')]
    nodes = [
        Node(package='atp_ros', executable='sim_clock',
             parameters=[{'time_scale': LC('time_scale')}]),
        Node(package='atp_ros', executable='reference_generator', remappings=to_program,
             parameters=[sim, {'profile': 'pass', 'el_max_deg': LC('el_max'),
                               'mask_deg': LC('mask'),
                               'wait_subscribers': PythonExpression(
                                   ["3 if '", LC('record_dir'), "' else 2"]),
                               'wait_nodes': PythonExpression([
                                   "'sim_clock,rf_beacon,autotrack,mount_sim,pid_controller,"
                                   "link_budget,error_monitor' + (',recorder' if '",
                                   LC('record_dir'), "' else '')"])}]),
        Node(package='atp_ros', executable='rf_beacon', parameters=[sim, {
            'band': LC('band'), 'ephem_dt_s': LC('ephem_dt_s'),
            'bias_xel_deg': LC('bias_xel_deg'), 'bias_el_deg': LC('bias_el_deg'),
            'noise_db': LC('noise_db')}]),
        Node(package='atp_ros', executable='autotrack', parameters=[sim, {
            'enabled': LC('autotrack'), 'band': LC('band')}]),
        Node(package='atp_ros', executable='mount_sim', parameters=[sim]),
        Node(package='atp_ros', executable='pid_controller', parameters=[sim]),
        Node(package='atp_ros', executable='link_budget',
             parameters=[sim, {'band': LC('band')}]),
    ]
    # igual que atp_loop: con grabador, el lanzamiento lo cierra el grabador
    grabando = PythonExpression(["'", LC('record_dir'), "' != ''"])
    mon = Node(package='atp_ros', executable='error_monitor', remappings=to_truth,
               parameters=[sim, {'exit_on_done': PythonExpression([
                   "'", LC('exit_on_done'), "' == 'true' and '", LC('record_dir'), "' == ''"])}])
    rec = Node(package='atp_ros', executable='recorder', condition=IfCondition(grabando),
               parameters=[sim, {'output_dir': LC('record_dir')}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=mon, on_exit=[EmitEvent(event=Shutdown(),
                                              condition=UnlessCondition(grabando))]))
    stop_rec = RegisterEventHandler(OnProcessExit(
        target_action=rec, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + nodes + [mon, rec, stop, stop_rec])
