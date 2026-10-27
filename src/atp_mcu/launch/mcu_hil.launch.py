"""Un eje real (ESP32 o el nodo micro-ROS de Linux) contra el generador y el monitor de atp_ros.

Como `atp_hw/hil.launch.py`, pero el lazo de posicion lo cierra el
microcontrolador (NO hay mount_sim ni pid_controller): el generador publica
/atp/reference, el ESP32 (via agente) publica /atp/mcu/az/joint_states y
`mcu_merge` lo une en /joint_states para error_monitor.

El agente y el ESP32 (o `eje_host`) se arrancan aparte, con el MISMO dominio:
  firmware/esp32_eje/agente.sh udp         (o serie)
  ros2 topic pub --once /atp/mcu/az/cmd/enable std_msgs/msg/Bool "{data: true}"

TIEMPO REAL: time_scale = 1 y reloj libre (sin lockstep), como en HIL.
El eje `el` es simulado (toma la referencia): solo `az` es hardware.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in (
        ('ramp_deg_s', '5.0'), ('ramp_duration_s', '40.0'),
        ('simulated', 'el'), ('exit_on_done', 'true'))]
    sim = {'use_sim_time': True}
    clock = Node(package='atp_ros', executable='sim_clock',
                 parameters=[{'time_scale': 1.0, 'lockstep': False}])
    ref = Node(package='atp_ros', executable='reference_generator', parameters=[sim, {
        'profile': 'ramp', 'ramp_deg_s': LC('ramp_deg_s'),
        'ramp_duration_s': LC('ramp_duration_s')}])
    mon = Node(package='atp_ros', executable='error_monitor', parameters=[sim, {
        'exit_on_done': LC('exit_on_done')}])
    merge = Node(package='atp_mcu', executable='mcu_merge', parameters=[sim, {
        'simulated': LC('simulated')}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=mon, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [clock, ref, mon, merge, stop])
