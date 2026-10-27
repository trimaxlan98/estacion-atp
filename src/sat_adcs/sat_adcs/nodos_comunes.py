"""Piezas comunes de los nodos ADCS: parametros y conversion de mensajes."""
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor

from sat_adcs.misiones import Config, config_campos


def declarar_config(node):
    """Declara cada campo de `Config` como parametro ROS (mismo nombre y
    defecto) y devuelve el `Config` leido. Los numericos aceptan int o float
    desde la linea de comandos (como `atp_ros.params.declare_num`)."""
    valores = {}
    for nombre, tipo, defecto in config_campos():
        if tipo is str:
            node.declare_parameter(nombre, defecto)
        else:
            node.declare_parameter(nombre, defecto, ParameterDescriptor(dynamic_typing=True))
        v = node.get_parameter(nombre).value
        valores[nombre] = tipo(v) if tipo in (int, float) else v
    return Config(**valores)


def sello_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def sens_de_msg(m):
    """AdcsState -> dict de sensores y verdad (el formato de `PlantaAdcs.medir`)."""
    return {'t': m.t_sim_s, 'mag': np.array(m.mag_body_t), 'gyro': np.array(m.gyro_rad_s),
            'sol': np.array(m.sun_body), 'sol_valido': bool(m.sun_valid),
            'rueda_h': np.array(m.wheel_h_nms), 'q': np.array(m.q_true),
            'w': np.array(m.w_true_rad_s), 'sesgo': np.array(m.gyro_bias_true),
            'eclipse': bool(m.eclipse)}


def msg_de_sens(msg, s, stamp):
    """Llena un AdcsState desde el dict `s` de la planta."""
    msg.header.stamp = stamp
    msg.t_sim_s = float(s['t'])
    msg.mag_body_t = [float(x) for x in s['mag']]
    msg.gyro_rad_s = [float(x) for x in s['gyro']]
    msg.sun_body = [float(x) for x in s['sol']]
    msg.sun_valid = bool(s['sol_valido'])
    msg.wheel_h_nms = [float(x) for x in s['rueda_h']]
    msg.q_true = [float(x) for x in s['q']]
    msg.w_true_rad_s = [float(x) for x in s['w']]
    msg.gyro_bias_true = [float(x) for x in s['sesgo']]
    msg.eclipse = bool(s['eclipse'])
    return msg


def msg_de_cmd(msg, c, t_sim, stamp):
    """Llena un AdcsCmd desde el dict del controlador."""
    msg.header.stamp = stamp
    msg.t_sim_s = float(t_sim)
    msg.mode = int(c['modo'])
    msg.wheel_torque_nm = [float(x) for x in c['u_w']]
    msg.mtq_dipole_am2 = [float(x) for x in c['m']]
    msg.est_valid = bool(c['est_valido'])
    msg.q_est = [float(x) for x in c['q_est']]
    msg.bias_est_rad_s = [float(x) for x in c['sesgo']]
    msg.sigma_att_rad = [float(x) for x in c['sigma_att']]
    return msg


def cmd_de_msg(m):
    return {'modo': int(m.mode), 'u_w': np.array(m.wheel_torque_nm),
            'm': np.array(m.mtq_dipole_am2), 'est_valido': bool(m.est_valid),
            'q_est': np.array(m.q_est), 'sesgo': np.array(m.bias_est_rad_s),
            'sigma_att': np.array(m.sigma_att_rad)}
