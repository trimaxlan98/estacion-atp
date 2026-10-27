"""Dinamica del rover (CONTRATO sec. 18).

    ros2 launch atp_rover rover.launch.py planeta:=marte terreno:=dunas semilla:=3
    ros2 topic pub -r 10 /rover/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

Con `use_sim_time` el nodo avanza con /clock (hace falta un `sim_clock`); `reloj:=pared` lo avanza a tiempo real.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defs = {'planeta': 'luna', 'terreno': 'llano', 'semilla': '0', 'terreno_params': '{}', 'rover': '{}',
            'x0': '0.0', 'y0': '0.0', 'rumbo_deg': '0.0', 'reloj': 'pared'}
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defs.items()]
    tipos = {'semilla': int, 'x0': float, 'y0': float, 'rumbo_deg': float}
    par = {k: ParameterValue(LC(k), value_type=tipos.get(k, str)) for k in defs}
    return LaunchDescription(args + [Node(package='atp_rover', executable='rover_dinamica', name='rover_dinamica',
                                           output='screen', parameters=[par])])
