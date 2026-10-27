"""Piezas comunes de los nodos ISL: parametros y conversion de mensajes."""
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor

from sat_isl.misiones import Config, config_campos


def declarar_config(node):
    """Declara cada campo de `Config` como parametro ROS (mismo nombre y defecto)
    y devuelve el `Config` leido. Los numericos aceptan int o float desde la
    linea de comandos (como `atp_ros.params.declare_num`)."""
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


_ESCALARES_EST = ('modo', 'visible', 'rango_km', 'tasa_los_urad_s', 'adelanto_urad', 'theta_h_urad',
                  'det_a', 'det_b', 'qpd_ok', 'pr_a_dbm', 'pr_b_dbm', 'ber', 'tasa_bps', 'cerrado',
                  'perdida_db', 'err_a_ms', 'err_b_ms', 'fsm_y_max_urad', 'nea_urad', 'busq_k',
                  'barridos', 'enganchado')
_TIPOS_EST = {'modo': int, 'busq_k': int, 'barridos': int, 'visible': bool, 'det_a': bool,
              'det_b': bool, 'qpd_ok': bool, 'cerrado': bool, 'enganchado': bool}


def msg_de_est(msg, s, stamp):
    """Llena un IslState desde el dict de `PlantaIsl.medir`."""
    msg.header.stamp = stamp
    msg.t_sim_s = float(s['t'])
    for k in _ESCALARES_EST:
        setattr(msg, k, _TIPOS_EST.get(k, float)(s[k]))
    msg.cen_a_urad = [float(x) for x in s['cen_a']]
    msg.cen_b_urad = [float(x) for x in s['cen_b']]
    msg.err_a_urad = [float(x) for x in s['err_a_urad']]
    msg.err_b_urad = [float(x) for x in s['err_b_urad']]
    return msg


def est_de_msg(m):
    d = {k: getattr(m, k) for k in _ESCALARES_EST}
    d['t'] = m.t_sim_s
    d['cen_a'] = np.array(m.cen_a_urad)
    d['cen_b'] = np.array(m.cen_b_urad)
    d['err_a_urad'] = np.array(m.err_a_urad)
    d['err_b_urad'] = np.array(m.err_b_urad)
    return d


def msg_de_cmd(msg, c, t_sim, stamp):
    msg.header.stamp = stamp
    msg.t_sim_s = float(t_sim)
    msg.modo = int(c['modo'])
    for k in ('dir_a', 'dir_b', 'steer_a', 'steer_b', 'pa_a', 'pa_b'):
        setattr(msg, k, [float(x) for x in c[k]])
    for k in ('pitch_urad', 'rmax_urad', 'dwell_s', 'tasa_bps', 'sigma_u_urad'):
        setattr(msg, k, float(c[k]))
    return msg


def cmd_de_msg(m):
    d = {k: np.array(getattr(m, k)) for k in ('dir_a', 'dir_b', 'steer_a', 'steer_b', 'pa_a', 'pa_b')}
    d.update({k: getattr(m, k) for k in ('pitch_urad', 'rmax_urad', 'dwell_s', 'tasa_bps', 'sigma_u_urad')})
    d['modo'] = int(m.modo)
    return d
