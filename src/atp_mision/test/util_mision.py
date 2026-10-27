"""Ejecutor SIMULADO de contactos: emula `mision_pase.launch.py` sin ROS.

Para las pruebas de la orquestacion: produce el `telemetry.jsonl` y el `metrics.json`
que grabaria la corrida real, con la geometria real (prediccion TLE u orbita estimada
contra la verdad del mundo) y un modelo SENCILLO del seguimiento:

  * program track: la montura sigue la prediccion con un error constante `err_montura`;
  * autotrack: si el error de efemeride en AOS cae dentro del cono de captura
    (0.913 theta3) la montura sigue a la verdad con `ruido_autotrack` (1.3 % de
    theta3); si no, queda en program track.

El enlace es el de `model.presupuesto_enlace` con el error de ese instante. No es el
lazo ROS (eso lo prueba la integracion real): sirve para ejercitar plan, papeles,
dependencias, OD con telemetria registrada, contabilidad, gemelos e informe.
"""
import json
import os
import threading

import numpy as np

from atp_ros import medidas as M
from atp_ros import model, od, orbit


class EjecutorSimulado:
    def __init__(self, fallos=None, err_montura_deg=0.0):
        self.llamadas = []            # (args, carpeta)
        self.fallos = fallos or {}    # nombre de carpeta -> n de intentos que fallan
        self.err_montura = err_montura_deg
        self._intentos = {}
        self._lock = threading.Lock()

    def __call__(self, args, carpeta):
        nombre = os.path.basename(carpeta)
        self.llamadas.append((dict(args), carpeta))
        self._intentos[nombre] = self._intentos.get(nombre, 0) + 1
        with self._lock:
            if 'primero' in self.fallos and 'primero_nombre' not in self.__dict__:
                self.primero_nombre = nombre          # la primera corrida que se lance
                self.fallos[nombre] = self.fallos.pop('primero')
        if self._intentos[nombre] <= self.fallos.get(nombre, 0):
            return {'estado': 'fallo', 'rc': 1, 'dominio': 200, 'pared_s': 0.0}
        os.makedirs(carpeta, exist_ok=True)
        sat, est = args['sat'], args['station']
        if args['profile'] == 'od':
            perfil, pase, _, estacion = od.perfil_od(args['od_file'], est, 0, args['start_utc'],
                                                     0.5, None, 0.0)
        else:
            perfil, pase, _, estacion = orbit.perfil_tle(sat, est, 0, args['start_utc'], 0.5,
                                                         None, 0.0)
        ef, reloj = od.cargar_efemeride(args['truth_file'])
        dur = float(pase['duracion_s'])
        pv = od.perfil_de_efemeride(ef, reloj, M.sitio(estacion),
                                    {'aos': reloj.segundos(pase['aos']), 'duracion_s': dur})
        t = np.arange(0.0, dur, 0.1)
        rad = np.radians
        # acimut desenroscado (como el generador): interpolar el acimut en [0, 360) rompe
        # el cruce por el norte
        k = round((perfil['az_continuo'][0] - pv['az_continuo'][0]) / 360.0)
        az_p = np.interp(t, perfil['t'], perfil['az_continuo'])
        el_p = np.interp(t, perfil['t'], perfil['el'])
        az_v = np.interp(t, pv['t'], pv['az_continuo'] + 360.0 * k)
        el_v = np.interp(t, pv['t'], pv['el'])
        d = np.interp(t, pv['t'], pv['d'])
        rr = np.interp(t, pv['t'], pv['vr'])
        f0 = model.BANDAS[args['band']]
        th3 = model.ancho_haz(model.DIAMETRO_PLATO_M, f0)
        e_efem = M.separacion_deg(rad(az_p), rad(el_p), rad(az_v), rad(el_v))
        rng = np.random.default_rng(7)
        engancha = bool(args['autotrack']) and e_efem[0] < 0.913 * th3
        if engancha:
            sig = 0.013 * th3
            az_m = az_v + rng.normal(0, sig, len(t)) / np.cos(rad(el_v))
            el_m = el_v + rng.normal(0, sig, len(t))
        else:
            az_m = az_p + self.err_montura
            el_m = el_p
        e_pt = M.separacion_deg(rad(az_m), rad(el_m), rad(az_v), rad(el_v))
        lb = model.presupuesto_enlace(d, e_pt, f_hz=f0, eirp_dbw=args['eirp_dbw'])
        fd = model.doppler_hz(rr, f0)
        with open(os.path.join(carpeta, 'telemetry.jsonl'), 'w') as f:
            for i in range(len(t)):
                if i == 0:
                    continue
                f.write(json.dumps({
                    't': round(float(t[i]), 3), 'az': round(float(az_m[i]), 6),
                    'el': round(float(el_m[i]), 6), 'e_pt': round(float(e_pt[i]), 6),
                    'range_km': round(float(d[i]), 3), 'doppler_hz': round(float(fd[i]), 2),
                    'l_point_db': round(float(lb['l_point_db'][i]), 6),
                    'cn0_dbhz': round(float(lb['cn0_dbhz'][i]), 3),
                    'link_closed': bool(lb['margen_db'][i] >= 0)}) + '\n')
        with open(os.path.join(carpeta, 'metrics.json'), 'w') as f:
            json.dump({'n': len(t), 'duration_s': dur, 'verdict': 'CUMPLE'}, f)
        with open(os.path.join(carpeta, 'launch.log'), 'w') as f:
            if engancha:
                f.write('[autotrack-8] [INFO] [1.0] [autotrack]: ENGANCHE\n')
            if args['sdr']:
                f.write('[sdr_receiver-10] [INFO] [1.0] [sdr_receiver]: RESUMEN SDR banda=S '
                        'modo=portadora fs=150 kHz | bloques=100 huecos=0 | Doppler rms=0.024 Hz '
                        'sesgo=-0.001 Hz max=1.24 Hz | C/N0 medido-presupuesto sesgo=+0.002 dB '
                        'rms=0.172 dB | enganchado 100.0% (adquisiciones=1 perdidas=0)\n')
        return {'estado': 'ok', 'rc': 0, 'dominio': 200, 'pared_s': 0.0}
