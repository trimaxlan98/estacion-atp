"""Servidor + cliente de una campana Monte Carlo; termina cuando el cliente termina.

  ros2 launch atp_ros campaign_run.launch.py mode:=lazo n:=300 record_dir:=/tmp/c

Los argumentos son los campos del goal de RunCampaign (0 = valor por defecto
del servidor). record_dir recibe campaign.json.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    campos = (('mode', 'analitico'), ('n', '500'), ('seed', '0'),
              ('sigma_wind_nm', '0.0'), ('sigma_noise_deg', '0.0'),
              ('bias_max_deg', '0.0'), ('latency_max_s', '0.0'), ('kp', '0.0'),
              ('mask_deg', '0.0'), ('vel_max_deg_s', '0.0'))
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in campos]
    args.append(DeclareLaunchArgument('record_dir', default_value=''))
    server = Node(package='atp_ros', executable='campaign_server')
    client = Node(package='atp_ros', executable='campaign_client', parameters=[
        {k: LC(k) for k, _ in campos}, {'output_dir': LC('record_dir')}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=client, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [server, client, stop])
