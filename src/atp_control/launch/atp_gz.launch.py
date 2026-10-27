"""Pase N2 con la montura en Gazebo Harmonic (gz sim) y gz_ros2_control.

  ros2 launch atp_control atp_gz.launch.py headless:=true el_max:=85 mask:=70
  ros2 launch atp_control atp_gz.launch.py headless:=false          (con GUI, necesita X11)
  ros2 launch atp_control atp_gz.launch.py controller:=lqr estimator:=kalman

La MISMA descripcion (atp_description, hw:=gz), el MISMO controlador
(AtpTrackingController) y los MISMOS nodos del lazo de atp_ros (generador de
referencia, enlace, monitor de error) que en atp_ros2_control.launch.py; cambia el
hardware: en lugar de AtpMountSystem hay un robot fisico en Gazebo (J, b de la
planta como inercia y amortiguamiento de las juntas) y el controller_manager vive
dentro del proceso de gz sim (plugin gz_ros2_control).

El reloj es el de Gazebo (/clock por ros_gz_bridge), a tiempo real: NO es
determinista como el lockstep. Para correr paso a paso ver docs/ROS2_CONTROL.md
y scripts/atp_gz_stepper.py (stepped:=true).
"""
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = (
    ('headless', 'true'), ('world', 'atp.sdf'),
    ('profile', 'pass'), ('el_max', '85.0'), ('mask', '70.0'), ('h_km', '550.0'),
    ('az_culmination', '140.0'),
    ('u_max', '8.0'), ('kp', '25.0'), ('ki', '2.0'), ('kd', '8.0'),
    ('band', 'S'), ('eirp_dbw', '9.0'), ('exit_on_done', 'true'), ('record_dir', ''),
    ('controller', 'pid'), ('estimator', 'none'),
    ('q_theta', '10000.0'), ('q_omega', '1.0'), ('q_int', '1000.0'), ('r_u', '1.0'),
    ('cm_rate', '100'), ('extrapolate_ref', 'true'), ('trace_file', ''),
    ('stepped', 'false'), ('step_s', '0.05'), ('settle_s', '0.004'), ('partition', ''),
    ('eval_file', ''),
    ('escena', 'false'), ('radio_escena', '10.0'), ('gui_res', '1920x1080'),
)


# Paleta Co.De Aerospace (codeaerospace_contenido/marca/paleta.json). Gazebo toma los colores en
# espacio LINEAL (no aplica gamma): se convierten desde el hex sRGB de la marca.
def _lineal(hex_, a=1.0, k=1.0):
    c = [int(hex_[i:i + 2], 16) / 255.0 for i in (1, 3, 5)]
    lin = [((v + 0.055) / 1.055) ** 2.4 if v > 0.04045 else v / 12.92 for v in c]
    return ' '.join(f'{v * k:.4f}' for v in lin) + f' {a}'


PALETA = {
    'fondo': _lineal('#080F15'), 'marino': _lineal('#0A0E27'), 'suelo': _lineal('#0F1A24'),
    'plata': _lineal('#F2F3F4'), 'tenue': _lineal('#8E99A6'),
    'cian': _lineal('#00D9FF'), 'cian_vivo': _lineal('#00D9FF', k=0.6), 'ambar': _lineal('#F59E0B'),
    'ambar_vivo': _lineal('#F59E0B', k=0.5), 'cian_tenue': _lineal('#00D9FF', k=0.22), 'panel': _lineal('#0B3A66'),
}


def _mat(color, emisivo=None, transparencia=None):
    e = f'<emissive>{PALETA[emisivo]}</emissive>' if emisivo else ''
    t = f'<transparency>{transparencia}</transparency>' if transparencia is not None else ''
    return (f'<material><ambient>{PALETA[color]}</ambient><diffuse>{PALETA[color]}</diffuse>{e}</material>'
            f'{t}')


def _mundo_con_escena(world, p, az_cul, radio, gui_res, zc=1.9):
    """Copia del mundo con la paleta Co.De, el satelite visible, el arco del pase, el haz de la
    antena y la camara de la GUI.

    El satelite (esfera con paneles, sin dinamica) y el haz (cilindro delgado a lo largo del eje
    REAL de la antena, de /joint_states) los mueve `atp_gz_escena.py`; el arco son puntos
    estaticos sobre la trayectoria (az, el) del pase a `radio` m del cruce de las juntas."""
    import math
    with open(world) as fh:
        sdf = fh.read()
    # suelo y base de concreto con la paleta (el mundo base no cambia en disco)
    sdf = sdf.replace('0.35 0.45 0.35 1', PALETA['suelo']).replace('0.6 0.6 0.6 1', PALETA['tenue'])
    n = len(p['t'])
    marcas = []
    for k in range(0, n, max(1, n // 100)):
        az, el = math.radians(float(p['az_continuo'][k])), math.radians(float(p['el'][k]))
        x = radio * math.cos(el) * math.cos(az)
        y = radio * math.cos(el) * math.sin(az)
        z = zc + radio * math.sin(el)
        marcas.append(f'<visual name="m{k}"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><geometry>'
                      f'<sphere><radius>0.055</radius></sphere></geometry>{_mat("cian_vivo", "cian")}</visual>')
    # huella del arco en el suelo (tenue) y estrellas (esferas emisivas lejanas, semilla fija)
    huella = []
    for k in range(0, n, max(1, n // 100)):
        az, el = math.radians(float(p['az_continuo'][k])), math.radians(float(p['el'][k]))
        huella.append(f'<visual name="h{k}"><pose>{radio * math.cos(el) * math.cos(az):.3f} '
                      f'{radio * math.cos(el) * math.sin(az):.3f} 0.02 0 0 0</pose><geometry><sphere>'
                      f'<radius>0.05</radius></sphere></geometry>{_mat("cian_tenue", "cian_tenue")}</visual>')
    import random
    azr = random.Random(7)
    estrellas = []
    for k in range(260):
        a = math.radians(az_cul) + math.radians(azr.uniform(-75, 75))
        e = math.radians(azr.uniform(4, 62))
        rr = 90.0
        r_est = azr.choice((0.07, 0.09, 0.12))
        col = 'plata' if azr.random() < 0.8 else 'cian'
        estrellas.append(f'<visual name="e{k}"><cast_shadows>false</cast_shadows><pose>{rr * math.cos(e) * math.cos(a):.2f} '
                         f'{rr * math.cos(e) * math.sin(a):.2f} {rr * math.sin(e):.2f} 0 0 0</pose><geometry><sphere>'
                         f'<radius>{r_est}</radius></sphere></geometry>{_mat(col, col)}</visual>')
    camara_az = math.radians(az_cul)
    # la camara mira desde el lado contrario al del pase
    dist = 1.6 * radio
    cx, cy = -dist * math.cos(camara_az), -dist * math.sin(camara_az)
    extra = f"""
    <scene>
      <ambient>0.40 0.43 0.50 1</ambient>
      <background>{PALETA['marino']}</background>
      <grid>false</grid><shadows>true</shadows>
    </scene>
    <model name="arco_del_pase"><static>true</static><link name="l">{''.join(marcas)}{''.join(huella)}</link></model>
    <model name="estrellas"><static>true</static><link name="l">{''.join(estrellas)}</link></model>
    <model name="satelite">
      <pose>0 0 {zc + radio:.3f} 0 0 0</pose>
      <link name="l">
        <gravity>false</gravity>
        <inertial><mass>1</mass></inertial>
        <visual name="cuerpo"><geometry><box><size>0.55 0.55 0.55</size></box></geometry>
          {_mat('ambar', 'ambar_vivo')}</visual>
        <visual name="antena"><pose>0 0 -0.4 0 0 0</pose><geometry><cylinder><radius>0.18</radius>
          <length>0.06</length></cylinder></geometry>{_mat('plata')}</visual>
        <visual name="panel_a"><pose>0 0.95 0 0 0 0</pose><geometry><box><size>0.7 1.2 0.03</size></box></geometry>
          {_mat('panel', 'cian_vivo')}</visual>
        <visual name="panel_b"><pose>0 -0.95 0 0 0 0</pose><geometry><box><size>0.7 1.2 0.03</size></box></geometry>
          {_mat('panel', 'cian_vivo')}</visual>
      </link>
    </model>
    <model name="haz">
      <pose>0 0 {zc:.3f} 0 0 0</pose>
      <link name="l">
        <gravity>false</gravity>
        <inertial><mass>0.1</mass></inertial>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><cylinder><radius>0.03</radius>
          <length>{radio:.3f}</length></cylinder></geometry>{_mat('cian', 'cian', 0.45)}</visual>
      </link>
    </model>
"""
    sdf = sdf.replace('</world>', extra + '  </world>')
    # el simbolo verde de la luz direccional de la GUI: se pone detras de la camara, fuera de cuadro
    sdf = sdf.replace('<pose>0 0 10 0 0 0</pose>', f'<pose>{2 * cx:.1f} {2 * cy:.1f} 12 0 0 0</pose>', 1)
    fd, ruta = tempfile.mkstemp(prefix='atp_escena_', suffix='.sdf')
    with os.fdopen(fd, 'w') as fh:
        fh.write(sdf)
    return ruta, _gui_config(cx, cy, camara_az, gui_res)


def _gui_config(cx, cy, camara_az, res):
    """Configuracion de la GUI (--gui-config): tamano de ventana (sin gestor de ventanas Gazebo
    ignora `fullscreen`), tema de la marca en la barra y la escena con la camara fija."""
    w, h = res.split('x')
    cfg = f"""<?xml version="1.0"?>
<window><width>{w}</width><height>{h}</height>
  <style material_theme="Dark" material_primary="#080F15" material_accent="#00D9FF"
         toolBarColorLight="#080F15" toolBarColorDark="#080F15" toolBarTextLight="#F2F3F4"
         toolBarTextDark="#F2F3F4" plugin_toolbar_color_light="#080F15" plugin_toolbar_color_dark="#080F15"
         plugin_toolbar_text_light="#F2F3F4" plugin_toolbar_text_dark="#F2F3F4"/>
</window>
<plugin filename="MinimalScene" name="3D View"><gz-gui><title>Estacion ATP</title>
  <property type="bool" key="showTitleBar">false</property></gz-gui>
  <engine>ogre2</engine><scene>scene</scene><ambient_light>0.4 0.43 0.5</ambient_light>
  <background_color>{PALETA['marino']}</background_color>
  <camera_pose>{cx:.2f} {cy:.2f} 2.0 0 -0.27 {camara_az:.3f}</camera_pose></plugin>
"""
    for filtro, nombre in (('GzSceneManager', 'Scene Manager'),
                           ('InteractiveViewControl', 'Interactive view control')):
        cfg += f"""<plugin filename="{filtro}" name="{nombre}"><gz-gui>
  <anchors target="3D View"><line own="right" target="right"/><line own="top" target="top"/></anchors>
  <property key="resizable" type="bool">false</property>
  <property key="width" type="double">5</property><property key="height" type="double">5</property>
  <property key="state" type="string">floating</property>
  <property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
"""
    fd, ruta = tempfile.mkstemp(prefix='atp_gui_', suffix='.config')
    with os.fdopen(fd, 'w') as fh:
        fh.write(cfg)
    return ruta


def _setup(context):
    c = lambda k: LC(k).perform(context)   # noqa: E731
    f = lambda k: float(c(k))              # noqa: E731
    if c('profile') != 'pass':
        raise RuntimeError('solo profile:=pass')
    import math
    from atp_ros import model
    p = model.perfil_pase(f('h_km'), f('el_max'), f('mask'), f('az_culmination'))
    az0, el0 = math.radians(float(p['az_continuo'][0])), math.radians(float(p['el'][0]))

    share_desc = get_package_share_directory('atp_description')
    base = os.path.join(get_package_share_directory('atp_control'), 'config',
                        'atp_controllers.yaml')
    with open(base) as fh:
        cfg = yaml.safe_load(fh)
    cfg['controller_manager']['ros__parameters'].update(
        {'update_rate': int(c('cm_rate')), 'use_sim_time': True})
    t = cfg['atp_tracking_controller']['ros__parameters']
    t.update({'law': c('controller'), 'estimator': c('estimator'), 'u_max': f('u_max'),
              'kp': f('kp'), 'ki': f('ki'), 'kd': f('kd'), 'q_theta': f('q_theta'),
              'q_omega': f('q_omega'), 'q_int': f('q_int'), 'r_u': f('r_u'),
              'rate_hz': 20.0, 'sync_publish': False,
              'extrapolate_ref': c('extrapolate_ref') == 'true', 'trace_file': c('trace_file')})
    fd, ctrl_yaml = tempfile.mkstemp(prefix='atp_gz_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as fh:
        yaml.safe_dump(cfg, fh)

    urdf = xacro.process_file(os.path.join(share_desc, 'urdf', 'antena.urdf.xacro'), mappings={
        'hw': 'gz', 'az0': repr(az0), 'el0': repr(el0), 'u_max': c('u_max'),
        'tema': 'code' if c('escena') == 'true' else 'base',
        'controllers_file': ctrl_yaml}).toxml()

    # los meshes se resuelven como package://atp_description/...: gz busca en esta ruta
    resource = os.path.dirname(share_desc)
    # aisla el transporte de gz de otras instancias en la misma maquina (red host)
    particion = c('partition') or f'atp_gz_{os.getpid()}'
    # GZ_IP=127.0.0.1: con la red del contenedor (host) el descubrimiento de gz-transport
    # por la interfaz por defecto no funciona (ni servicios ni topicos): por loopback si
    env = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource),
           SetEnvironmentVariable('GZ_PARTITION', particion),
           SetEnvironmentVariable('GZ_IP', '127.0.0.1')]

    sim = {'use_sim_time': True}
    grabando = c('record_dir') != ''
    world = c('world') if os.path.isabs(c('world')) else os.path.join(share_desc, 'worlds', c('world'))
    escena = c('escena') == 'true'
    gz_cmd = ['gz', 'sim', '-r', '-v', '2', world]
    if escena:
        world, gui_cfg = _mundo_con_escena(world, p, f('az_culmination'), f('radio_escena'),
                                           c('gui_res'))
        gz_cmd = ['gz', 'sim', '-r', '-v', '2', '--gui-config', gui_cfg, world]
    if c('headless') == 'true':
        gz_cmd.insert(2, '-s')
    if c('stepped') == 'true':
        gz_cmd.remove('-r')      # arranca en pausa: lo avanza atp_gz_stepper.py
    gz = ExecuteProcess(cmd=gz_cmd, output='screen')

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[sim, {'robot_description': ParameterValue(urdf, value_type=str)}])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'atp_mount',
                            '-world', 'atp', '-z', '0.0'])
    spawn_jsb = Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=['joint_state_broadcaster', '--controller-manager',
                                '/controller_manager', '--controller-manager-timeout', '120'])
    spawn_trk = Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=['atp_tracking_controller', '--controller-manager',
                                '/controller_manager', '--controller-manager-timeout', '120'])
    cadena = [RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawn_jsb])),
              RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_trk]))]

    # lazo de atp_ros: sin sim_clock (el reloj es el de Gazebo) ni lockstep
    nodos_espera = ['link_budget', 'error_monitor', 'pointing_eval'] + (['recorder'] if grabando else [])
    ref = Node(package='atp_ros', executable='reference_generator', parameters=[sim, {
        'profile': 'pass', 'el_max_deg': f('el_max'), 'mask_deg': f('mask'),
        'h_km': f('h_km'), 'az_culmination_deg': f('az_culmination'),
        'wait_subscribers': 4 if grabando else 3, 'wait_nodes': ','.join(nodos_espera)}])
    link = Node(package='atp_ros', executable='link_budget', parameters=[sim, {
        'band': c('band'), 'eirp_dbw': f('eirp_dbw')}])
    mon = Node(package='atp_ros', executable='error_monitor', parameters=[sim, {
        'exit_on_done': c('exit_on_done') == 'true' and not grabando}])
    evalu = Node(package='atp_control', executable='pointing_eval.py',
                 parameters=[sim, {'out_file': c('eval_file')}])
    acciones = env + [gz, rsp, bridge, spawn] + cadena + [ref, link, mon, evalu]
    if escena:
        acciones.append(Node(package='atp_control', executable='atp_gz_escena.py', output='screen',
                             parameters=[{'radio': f('radio_escena')}]))
    if c('stepped') == 'true':
        # gz arranca en pausa: este nodo lo avanza paso a paso (ver su docstring)
        acciones.append(Node(package='atp_control', executable='atp_gz_stepper.py',
                             output='screen', parameters=[{
                                 'step_s': f('step_s'), 'settle_s': f('settle_s')}]))
    if grabando:
        rec = Node(package='atp_ros', executable='recorder',
                   parameters=[sim, {'output_dir': c('record_dir')}])
        acciones += [rec, RegisterEventHandler(OnProcessExit(
            target_action=rec, on_exit=[EmitEvent(event=Shutdown())]))]
    else:
        acciones.append(RegisterEventHandler(OnProcessExit(
            target_action=mon, on_exit=[EmitEvent(event=Shutdown())])))
    return acciones


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS] + [OpaqueFunction(function=_setup)])
