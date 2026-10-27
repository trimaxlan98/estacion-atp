"""Lazo de una corrida del dron sin ROS: ruta de metas [[x, y, z], ...], escribe telemetry.jsonl y metrics.json.

Mismo escenario y semilla -> mismos bytes (sin sellos de reloj de pared). Telemetria a `hz_telemetria` (10 Hz).
"""
DEFECTOS = {'cuerpo': 'tierra', 'semilla': 0, 'ruta': [[0, 0, 5]], 'duracion_s': 40.0, 'x0': 0.0, 'y0': 0.0,
            'yaw_deg': 0.0, 'v_max': 5.0, 'radio_meta_m': 0.5, 'viento': [0.0, 0.0, 0.0], 'rafaga_sigma': 0.0,
            'rafaga_tau_s': 2.0, 'dron': {}, 'hz_telemetria': 10.0, 'z_despegue_m': 1.5, 'parar_al_llegar': True,
            'sensores': False, 'sensores_params': {}}


def correr(cfg, carpeta=None):
    """Una corrida del dron: la cadena en lockstep (`atp_dron.lockstep`) en un solo proceso. Devuelve (metricas, filas)."""
    from atp_dron.lockstep import correr_cadena
    met, filas, _ = correr_cadena(cfg, carpeta)
    return met, filas
