"""Hardware en el lazo: el pase de atp_ros contra un rotor (real o simulado).

Se lanzan sim_clock, reference_generator, link_budget y error_monitor de
atp_ros + rotctld_bridge. NO hay mount_sim ni pid_controller: el lazo de
posicion lo cierra el controlador del rotor y /joint_states sale de leerlo.

TIEMPO REAL: el rotor no se acelera, asi que aqui time_scale es SIEMPRE 1
(no es un argumento; el puente ademas vigila el reloj y avisa si no lo es).

  rotor:=fake   rotor falso de atp_hw (dinamica de G-5500: 6/3 deg/s) [defecto]
  rotor:=dummy  `rotctld -m 1` de Hamlib (dummy, ~6 deg/s en ambos ejes)
  rotor:=none   rotctld ya corriendo en host:port (hardware real, p. ej.
                `rotctld -m 603 -r /dev/ttyUSB0 -s 9600 -t 4533`)

N2 contra el rotor falso:
  ros2 launch atp_hw hil.launch.py el_max:=85 mask:=70
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in (
        # pase (atp_ros)
        ('el_max', '85.0'), ('mask', '70.0'), ('band', 'S'), ('eirp_dbw', '9.0'),
        ('exit_on_done', 'true'),
        # rotor
        ('rotor', 'fake'), ('host', '127.0.0.1'), ('port', '4690'),
        ('rot_az_min', '0.0'), ('rot_az_max', '360.0'),
        ('rot_el_min', '0.0'), ('rot_el_max', '90.0'),
        ('rot_az_res', '0.1'), ('rot_el_res', '0.1'),
        ('cmd_rate_hz', '5.0'), ('read_rate_hz', '10.0'),
        ('az_prefer', 'low'), ('preposition', 'true'),
        # rotor falso
        ('fake_az_speed', '6.0'), ('fake_el_speed', '3.0'),
        ('fake_az0', '0.0'), ('fake_el0', '0.0'),
        ('fake_az_res', '0.0'), ('fake_el_res', '0.0'), ('fake_delay', '0.0'))]
    sim = {'use_sim_time': True}
    clock = Node(package='atp_ros', executable='sim_clock',
                 # fijo: tiempo real y reloj libre (un rotor real no confirma pasos)
                 parameters=[{'time_scale': 1.0, 'lockstep': False}])
    ref = Node(package='atp_ros', executable='reference_generator', parameters=[sim, {
        'profile': 'pass', 'el_max_deg': LC('el_max'), 'mask_deg': LC('mask')}])
    link = Node(package='atp_ros', executable='link_budget', parameters=[sim, {
        'band': LC('band'), 'eirp_dbw': LC('eirp_dbw')}])
    mon = Node(package='atp_ros', executable='error_monitor', parameters=[sim, {
        'exit_on_done': LC('exit_on_done')}])
    bridge = Node(package='atp_hw', executable='rotctld_bridge', parameters=[sim, {
        'host': LC('host'), 'port': LC('port'),
        'az_min': LC('rot_az_min'), 'az_max': LC('rot_az_max'),
        'el_min': LC('rot_el_min'), 'el_max': LC('rot_el_max'),
        'az_res_deg': LC('rot_az_res'), 'el_res_deg': LC('rot_el_res'),
        'cmd_rate_hz': LC('cmd_rate_hz'), 'read_rate_hz': LC('read_rate_hz'),
        'az_prefer': LC('az_prefer'), 'preposition': LC('preposition')}])
    fake = ExecuteProcess(
        cmd=['python3', '-m', 'atp_hw.fake_rotctld',
             '--host', LC('host'), '--port', LC('port'),
             '--az-speed', LC('fake_az_speed'), '--el-speed', LC('fake_el_speed'),
             '--az-min', LC('rot_az_min'), '--az-max', LC('rot_az_max'),
             '--el-min', LC('rot_el_min'), '--el-max', LC('rot_el_max'),
             '--az0', LC('fake_az0'), '--el0', LC('fake_el0'),
             '--az-res', LC('fake_az_res'), '--el-res', LC('fake_el_res'),
             '--delay', LC('fake_delay')],
        condition=IfCondition(PythonExpression(["'", LC('rotor'), "' == 'fake'"])),
        output='screen')
    dummy = ExecuteProcess(
        cmd=['rotctld', '-m', '1', '-T', LC('host'), '-t', LC('port')],
        condition=IfCondition(PythonExpression(["'", LC('rotor'), "' == 'dummy'"])),
        output='screen')
    stop = RegisterEventHandler(OnProcessExit(
        target_action=mon, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [fake, dummy, clock, ref, link, mon, bridge, stop])
