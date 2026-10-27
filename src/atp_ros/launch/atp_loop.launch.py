"""Lazo ATP completo con Doppler y enlace, en tiempo simulado.

N1:  ros2 launch atp_ros atp_loop.launch.py profile:=ramp
N2:  ros2 launch atp_ros atp_loop.launch.py profile:=pass el_max:=85
N4:  ros2 launch atp_ros atp_loop.launch.py el_max:=30 mask:=5 time_scale:=10 band:=Ka
S4:  ros2 launch atp_ros atp_loop.launch.py profile:=tle sat:=ISS station:=cdmx pass_index:=0 time_scale:=10 band:=S
     (pase REAL con SGP4; start_utc:=2026-10-02T15:00:00 lo fija en el tiempo,
      tle_mask:=<deg> sustituye la mascara de la estacion)
Estacion: agregar record_dir:=<carpeta> para grabar telemetry.jsonl y metrics.json.
Control avanzado (equipo H, ver docs/CONTROL.md):
  controller:=pid|lqr (defecto pid), estimator:=none|kalman (defecto none)
  planta realista (todo apagado por defecto): backlash_deg, coulomb_nm,
  stribeck_nm, unbalance_nm, encoder_bits, encoder_side:=load|motor,
  wind_sigma_nm + wind_model:=dryden|davenport + wind_tau_s
  truth:=true mide el apuntamiento contra el angulo REAL de la carga
  rec_vel:=true (velocidad real en telemetry.jsonl), error_esferico:=true (error en la esfera exacta).
  profile:=handover: re-apuntamiento A->B (ho_t, ho_daz, ho_del, ho_rate_az/el, ho_dur).
N2 con LQR:  ros2 launch atp_ros atp_loop.launch.py el_max:=85 mask:=70 time_scale:=0 controller:=lqr
Keyhole: keyhole:=ingenua|anticipada|sobre_cenit (defecto ingenua),
  vel_max:=6 (tope del rotor), acel_plan, el_montura_max:=180 (sobre_cenit), pass_n y
  tle_dt (paso del perfil: 12000 / 0.05 cerca del cenit), stations_file, tle_search_hours.
  meas_delay:=<s>  retardo de la lectura del encoder (gemelo vs real; defecto 0).
Determinacion de orbita (equipo N, docs/OD.md; todo apagado por defecto):
  od:=true agrega el nodo od_estimator, que con las mediciones selladas del pase
  (angulos de la montura; od_range / od_doppler:=true suman rango / Doppler
  simulados) ajusta la orbita y la escribe en <od_out>/od.json.
  profile:=od od_file:=<od.json> usa esa orbita estimada como efemeride del pase.
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
        ('ramp_deg_s', '5.0'), ('ramp_duration', '20.0'),
        ('u_max', '8.0'), ('vel_max', '0.0'), ('noise_deg', '0.0'),
        ('gust_nm', '0.0'), ('gust_start', '10.0'),
        ('kp', '25.0'), ('kd', '8.0'),
        ('band', 'S'), ('eirp_dbw', '9.0'),
        ('time_scale', '1.0'), ('exit_on_done', 'true'),
        # perfil tle (Sprint 4): satelite, estacion ('' = la por defecto del YAML)
        ('sat', 'ISS'), ('station', ''), ('pass_index', '0'),
        ('start_utc', ''), ('tle_mask', '-1.0'),
        # Estacion (Sprint 2): carpeta de la corrida para el grabador
        ('record_dir', ''),
        # control avanzado (aditivo: los defectos dejan el lazo como estaba)
        ('controller', 'pid'), ('estimator', 'none'),
        ('q_theta', '10000.0'), ('q_omega', '1.0'), ('q_int', '1000.0'), ('r_u', '1.0'),
        ('kf_q_acc', '0.0001'), ('kf_q_par', '0.01'),
        # planta realista (todo apagado)
        ('backlash_deg', '0.0'), ('coulomb_nm', '0.0'), ('stribeck_nm', '0.0'),
        ('unbalance_nm', '0.0'), ('encoder_bits', '0'), ('encoder_side', 'load'),
        ('wind_sigma_nm', '0.0'), ('wind_model', 'dryden'), ('wind_tau_s', '5.0'),
        ('truth', 'false'),
        # determinacion de orbita (aditivo, apagado por defecto)
        ('od', 'false'), ('od_out', ''), ('od_file', ''), ('od_mode', 'lotes'),
        ('od_range', 'false'), ('od_doppler', 'false'), ('od_prior', 'tle'),
        # keyhole (defectos = de siempre)
        ('keyhole', 'ingenua'), ('acel_plan', '20.0'), ('el_montura_max', '90.0'),
        ('pass_n', '1200'), ('tle_dt', '0.5'), ('tle_search_hours', '72.0'),
        ('stations_file', ''), ('meas_delay', '0.0'),
        ('rec_vel', 'false'), ('error_esferico', 'false'),
        ('ho_t', '20.0'), ('ho_dur', '60.0'), ('ho_az0', '100.0'), ('ho_el0', '30.0'),
        ('ho_daz', '135.0'), ('ho_del', '15.0'), ('ho_rate_az', '0.5'), ('ho_rate_el', '0.0'))]
    sim = {'use_sim_time': True}
    clock = Node(package='atp_ros', executable='sim_clock',
                 parameters=[{'time_scale': LC('time_scale')}])
    ref = Node(package='atp_ros', executable='reference_generator', parameters=[sim, {
        'profile': LC('profile'),
        'el_max_deg': LC('el_max'), 'mask_deg': LC('mask'),
        'ramp_deg_s': LC('ramp_deg_s'), 'ramp_duration_s': LC('ramp_duration'),
        # str explicito: sin el, el YAML de launch_ros convierte '25544' en int
        # y una fecha ISO en datetime
        'tle_name': ParameterValue(LC('sat'), value_type=str),
        'station': ParameterValue(LC('station'), value_type=str),
        'start_utc': ParameterValue(LC('start_utc'), value_type=str),
        'pass_index': LC('pass_index'), 'tle_mask_deg': LC('tle_mask'),
        'od_file': ParameterValue(LC('od_file'), value_type=str),
        'keyhole_strategy': ParameterValue(LC('keyhole'), value_type=str),
        'vel_plan_deg_s': PythonExpression(["float('", LC('vel_max'), "') or 6.0"]),
        'acel_plan_deg_s2': LC('acel_plan'), 'el_montura_max_deg': LC('el_montura_max'),
        'pass_n': LC('pass_n'), 'tle_dt_s': LC('tle_dt'),
        'tle_search_hours': LC('tle_search_hours'),
        'stations_file': ParameterValue(LC('stations_file'), value_type=str),
        # monitor + enlace (+ grabador) (+ od_estimator): nadie debe llegar
        # tarde al pase
        'wait_subscribers': PythonExpression([
            "(3 if '", LC('record_dir'), "' else 2) + (1 if '", LC('od'), "' == 'true' else 0)"]),
        # todos los nodos del lazo antes de arrancar (el controlador segun cual)
        'ho_t_s': LC('ho_t'), 'ho_dur_s': LC('ho_dur'), 'ho_az0_deg': LC('ho_az0'),
        'ho_el0_deg': LC('ho_el0'), 'ho_daz_deg': LC('ho_daz'), 'ho_del_deg': LC('ho_del'),
        'ho_rate_az': LC('ho_rate_az'), 'ho_rate_el': LC('ho_rate_el'),
        'wait_nodes': PythonExpression([
            "'sim_clock,mount_sim,link_budget,error_monitor,' + ('lqr_controller' if '",
            LC('controller'), "' == 'lqr' else 'pid_controller') + (',recorder' if '",
            LC('record_dir'), "' else '') + (',od_estimator' if '", LC('od'),
            "' == 'true' else '')"]),
    }])
    mount = Node(package='atp_ros', executable='mount_sim', parameters=[sim, {
        'u_max': LC('u_max'), 'vel_max_deg_s': LC('vel_max'),
        'encoder_noise_deg': LC('noise_deg'),
        'gust_start_s': LC('gust_start'), 'gust_duration_s': 1.0,
        'gust_torque_nm': LC('gust_nm'),
        'backlash_deg': LC('backlash_deg'), 'coulomb_nm': LC('coulomb_nm'),
        'stribeck_nm': LC('stribeck_nm'), 'unbalance_nm': LC('unbalance_nm'),
        'encoder_bits': LC('encoder_bits'), 'meas_delay_s': LC('meas_delay'),
        'encoder_side': ParameterValue(LC('encoder_side'), value_type=str),
        'wind_sigma_nm': LC('wind_sigma_nm'),
        'wind_model': ParameterValue(LC('wind_model'), value_type=str),
        'wind_tau_s': LC('wind_tau_s'),
        'publish_truth': ParameterValue(LC('truth'), value_type=bool),
    }])
    # lo que el filtro sabe del sensor: el ruido y los bits del encoder
    estim = {'enc_sigma_deg': LC('noise_deg'), 'enc_bits': LC('encoder_bits'),
             'estimator': ParameterValue(LC('estimator'), value_type=str),
             'kf_q_acc': LC('kf_q_acc'), 'kf_q_par': LC('kf_q_par')}
    pid = Node(package='atp_ros', executable='pid_controller',
               condition=IfCondition(PythonExpression(["'", LC('controller'), "' == 'pid'"])),
               parameters=[sim, {'kp': LC('kp'), 'kd': LC('kd'), 'u_max': LC('u_max')}, estim])
    lqr = Node(package='atp_ros', executable='lqr_controller',
               condition=IfCondition(PythonExpression(["'", LC('controller'), "' == 'lqr'"])),
               parameters=[sim, {'u_max': LC('u_max'), 'q_theta': LC('q_theta'),
                                 'q_omega': LC('q_omega'), 'q_int': LC('q_int'),
                                 'r_u': LC('r_u')}, estim])
    link = Node(package='atp_ros', executable='link_budget', parameters=[sim, {
        'band': LC('band'), 'eirp_dbw': LC('eirp_dbw')}])
    # Con grabador (record_dir no vacio), quien termina el lazo es el grabador
    # al escribir metrics.json; el monitor no sale antes, asi no hay carrera
    # entre el cierre del lanzamiento y la ultima escritura.
    grabando = PythonExpression(["'", LC('record_dir'), "' != ''"])
    con_keyhole = PythonExpression(["'", LC('keyhole'), "' != 'ingenua' or '",
                                    LC('error_esferico'), "' == 'true'"])
    mon = Node(package='atp_ros', executable='error_monitor', parameters=[sim, {
        'use_truth': ParameterValue(LC('truth'), value_type=bool),
        # con una estrategia de keyhole se mide contra la VERDAD (/atp/truth)
        'ref_topic': PythonExpression([
            "'/atp/truth' if '", LC('keyhole'), "' != 'ingenua' else '/atp/reference'"]),
        'esferico': con_keyhole,
        'montura_sobre_cenit': PythonExpression(["'", LC('keyhole'), "' == 'sobre_cenit'"]),
        'exit_on_done': PythonExpression([
            "'", LC('exit_on_done'), "' == 'true' and '", LC('record_dir'), "' == ''"])}])
    rec = Node(package='atp_ros', executable='recorder', condition=IfCondition(grabando),
               parameters=[sim, {'output_dir': LC('record_dir'),
                                 'record_velocity': ParameterValue(LC('rec_vel'), value_type=bool)}])
    # determinacion de orbita: mismos parametros de pase que el generador, para
    # que el nodo ponga fecha a las mediciones y elija el a priori
    odn = Node(package='atp_ros', executable='od_estimator',
               condition=IfCondition(PythonExpression(["'", LC('od'), "' == 'true'"])),
               parameters=[sim, {
                   'profile': ParameterValue(LC('od_prior'), value_type=str),
                   'tle_name': ParameterValue(LC('sat'), value_type=str),
                   'station': ParameterValue(LC('station'), value_type=str),
                   'start_utc': ParameterValue(LC('start_utc'), value_type=str),
                   'pass_index': LC('pass_index'), 'tle_mask_deg': LC('tle_mask'),
                   'od_file': ParameterValue(LC('od_file'), value_type=str),
                   'output_dir': ParameterValue(LC('od_out'), value_type=str),
                   'mode': ParameterValue(LC('od_mode'), value_type=str),
                   'use_range': ParameterValue(LC('od_range'), value_type=bool),
                   'use_doppler': ParameterValue(LC('od_doppler'), value_type=bool)}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=mon, on_exit=[EmitEvent(event=Shutdown(),
                                              condition=UnlessCondition(grabando))]))
    stop_rec = RegisterEventHandler(OnProcessExit(
        target_action=rec, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [clock, ref, mount, pid, lqr, link, mon, rec,
                                     stop, stop_rec, odn])
