"""Grabador ADCS de la Estacion: guarda la actitud del CubeSat en disco
(docs/CONTRATO.md 2.1 y 3.1).

- telemetry.jsonl: UNA fila por paso del lazo (paso_s, 1 s de tiempo simulado) con la
  verdad de la planta y la salida del controlador del MISMO instante. Un campo que
  no tiene dato se omite. Con `flush` por linea para que el backend la siga en vivo.
- metrics.json: se escribe al cerrar la cuenta tras /atp/adcs_done y el nodo sale
  (el lanzamiento se apaga con el: ver adcs.launch.py, `record_dir`).

Las cifras son las de `misiones.Medidor`, las mismas del RESUMEN del monitor y de la
corrida offline: el grabador no recalcula nada propio, solo las vuelca.

Lockstep (CONTRATO 8): nodo PASIVO. Cola profunda (400), empareja estado y mando por
SELLO (nunca "el ultimo que llego"), no frena el reloj y cierra la cuenta con un
temporizador de tiempo simulado tras /atp/adcs_done. Anuncia en /atp/listo cuando
sus suscripciones tienen publicador (`anunciar_listo`); la planta y el controlador
esperan un suscriptor mas cuando se graba (wait_subscribers = 3 en el launch).
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64

from atp_ros.nodo import anunciar_listo
from sat_adcs.escenario import cargar_base
from sat_adcs.misiones import NOMBRE_MODO, Medidor
from sat_adcs.nodos_comunes import cmd_de_msg, declarar_config, sello_ns, sens_de_msg
from sat_interfaces.msg import AdcsCmd, AdcsState


def _r(x, nd=6):
    return round(float(x), nd)


def _lista(v, nd=6):
    return [_r(x, nd) for x in v]


def _limpio(x):
    """NaN/inf -> None (json.dump con allow_nan=False): un metrics.json valido
    para cualquier lector, el navegador incluido."""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {k: _limpio(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_limpio(v) for v in x]
    return x


def fila_de(t, modo, s, cmd, med, g):
    """Fila de telemetria (CONTRATO 2.1) de un instante ya pasado por `med.muestra`.

    `med` guarda en sus listas el valor de ESTE instante en la ultima posicion;
    `g` es la geometria (elevacion, rango) de la estacion en t."""
    row = {'t': _r(t, 3), 'mode': int(modo), 'w_deg_s': _r(med.w_deg[-1]),
           'q': _lista(s['q'], 8), 'h_nms': _lista(s['rueda_h'], 8),
           'eclipse': bool(s['eclipse']), 'el_deg': _r(g['el_deg'], 4),
           'range_km': _r(g['rango_km'], 3), 'in_pass': bool(med.en_pase[-1]),
           'u_w_nm': _lista(cmd['u_w'], 8), 'm_am2': _lista(cmd['m'], 6)}
    err = med.err_boresight[-1]
    if math.isfinite(err):                      # en DETUMBLE no hay objetivo de apuntamiento
        row['err_deg'] = _r(err)
    if cmd['est_valido']:
        row['q_est'] = _lista(cmd['q_est'], 8)
        row['know_deg'] = _r(med.conoc[-1])
        row['sigma_deg'] = _r(med.sigma[-1])
    return row


def segmentos_de_modo(t, modo):
    """[{mode, name, t0, t1}] de los tramos de modo consecutivos (t1 = ultima
    muestra del tramo)."""
    out = []
    for ti, mi in zip(t, modo):
        if out and out[-1]['mode'] == mi:
            out[-1]['t1'] = _r(ti, 3)
        else:
            out.append({'mode': int(mi), 'name': NOMBRE_MODO[int(mi)],
                        't0': _r(ti, 3), 't1': _r(ti, 3)})
    return out


def metricas_de(med, cfg):
    """metrics.json de una corrida ADCS (CONTRATO 3.1) a partir de las cifras del Medidor."""
    c = med.cifras()
    return _limpio({
        'kind': 'adcs', 'n': c['n'], 'duration_s': c['duracion_s'],
        'paso_s': cfg.paso_s,
        'modes': segmentos_de_modo(med.t, med.modo),
        'detumble': {'w0_deg_s': c['w0_deg_s'], 'w_threshold_deg_s': math.degrees(med.w_umbral),
                     't_detumble_s': c['t_detumble_s'], 't_nadir_s': c['t_nadir_s']},
        'nadir': {'n': c['nadir_n'], 'rms': c['nadir_rms'], 'p95': c['nadir_p95'],
                  'max': c['nadir_max'], 'settle_s': c['t_asentamiento_s']},
        'track': {'n': c['track_n'], 'rms': c['track_rms'], 'p95': c['track_p95'],
                  'max': c['track_max'], 'threshold_deg': c['umbral_deg'],
                  'beamwidth_deg': c['theta3_sat_deg'], 'mask_deg': med.mask},
        'knowledge': {'rms_deg': c['conoc_rms'], 'sigma_rms_deg': c['sigma_rms']},
        'wheels': {'h_max_nms': c['h_max_nms'], 'capacity_nms': med.sat.h_max,
                   'frac': c['h_max_frac']},
        'link': {'band': cfg.banda, 'antenna_m': cfg.d_sat_m,
                 'l_sat_rms_db': c['l_sat_rms_db'], 'l_sat_max_db': c['l_sat_max_db'],
                 'margin_min_db': c['margen_min_db'], 'closed_pct': c['cerrado_pct']},
        'eclipse_in_pass': c['eclipse_en_pase'],
        'verdict': 'CUMPLE' if c['cumple'] else 'NO CUMPLE',
    })


class AdcsRecorder(Node):
    def __init__(self):
        super().__init__('adcs_recorder')
        cfg = declarar_config(self)
        self.declare_parameter('output_dir', '')
        out = str(self.get_parameter('output_dir').value)
        if not out:
            raise ValueError('adcs_recorder: falta el parametro output_dir')
        self.cfg, self.out = cfg, out
        os.makedirs(out, exist_ok=True)
        sat, _, estacion, _, efem = cargar_base(cfg)
        self.med = Medidor(cfg, sat, efem, estacion)
        self.path = os.path.join(out, 'telemetry.jsonl')
        self.tele = open(self.path, 'w', encoding='utf-8')
        self.estados, self.cmds = {}, {}
        self.t_ultimo = -1.0
        self.t_fin = None
        self.cierre = None
        self.finished = False
        q = 400
        self.create_subscription(AdcsState, '/atp/adcs_state', self.on_state, q)
        self.create_subscription(AdcsCmd, '/atp/adcs_cmd', self.on_cmd, q)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64, '/atp/adcs_done', self.on_done, latched)
        self.get_logger().info(f'grabando en {out}')

    def intentar(self, s):
        if s not in self.estados or s not in self.cmds or self.finished:
            return
        st, cmd = self.estados.pop(s), self.cmds.pop(s)
        if st['t'] <= self.t_ultimo:                    # el estado inicial se repite al arrancar
            return
        self.t_ultimo = st['t']
        self.med.muestra(st, cmd)
        g = self.med.geo.en(st['t'])
        self.tele.write(json.dumps(fila_de(st['t'], cmd['modo'], st, cmd, self.med, g)) + '\n')
        self.tele.flush()

    def on_state(self, msg):
        s = sello_ns(msg.header.stamp)
        self.estados[s] = sens_de_msg(msg)
        self.intentar(s)

    def on_cmd(self, msg):
        s = sello_ns(msg.header.stamp)
        self.cmds[s] = cmd_de_msg(msg)
        self.intentar(s)

    def on_done(self, msg):
        if self.cierre is not None or self.finished:
            return
        self.t_fin = float(msg.data)
        # igual que el monitor: los ultimos instantes pueden llegar despues del
        # aviso (otro topico, sin orden entre ellos): 3 s simulados de margen
        self.cierre = self.create_timer(3.0, self.cerrar)

    def _recortar_archivo(self):
        """Quita las filas posteriores a t_fin (la planta sigue paso a paso tras
        el fin del escenario; el monitor tambien las descarta)."""
        self.tele.close()
        with open(self.path, 'r', encoding='utf-8') as f:
            filas = [ln for ln in f if ln.strip()]
        buenas = [ln for ln in filas if json.loads(ln)['t'] <= self.t_fin + 1e-9]
        if len(buenas) != len(filas):
            with open(self.path + '.tmp', 'w', encoding='utf-8') as f:
                f.writelines(buenas)
            os.replace(self.path + '.tmp', self.path)

    def cerrar(self):
        self.cierre.cancel()
        if self.finished:
            return
        self.finished = True
        self._recortar_archivo()
        self.med.recortar(self.t_fin)
        if not self.med.t:
            self.get_logger().error('sin muestras')
            raise SystemExit(1)
        path = os.path.join(self.out, 'metrics.json')
        with open(path + '.tmp', 'w', encoding='utf-8') as f:
            json.dump(metricas_de(self.med, self.cfg), f, indent=1, allow_nan=False)
        os.replace(path + '.tmp', path)
        self.get_logger().info(f'metrics.json escrito en {self.out}')
        raise SystemExit(0)


def main():
    rclpy.init()
    node = AdcsRecorder()
    anunciar_listo(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except RuntimeError as e:
        # misma carrera de cierre que trata atp_ros.nodo.correr
        if rclpy.ok() and 'Unable to convert call argument' not in str(e):
            raise
    finally:
        if not node.tele.closed:
            node.tele.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

