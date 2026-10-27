"""Grabador de la Estacion: guarda cada pase en disco (docs/CONTRATO.md 2 y 3).

- telemetry.jsonl: una muestra cada 0.1 s de tiempo simulado (10 Hz), con
  flush por linea para que el backend la pueda seguir en vivo. Un campo que
  aun no tiene dato se omite.
- metrics.json: se escribe al recibir /atp/pass_done (latched) y el nodo sale.

Las metricas se calculan con las muestras de /atp/pointing_error (20 Hz, las
mismas que ve error_monitor) y /atp/link, no con el archivo submuestreado.

Lockstep (CONTRATO 8): cada fila es UN instante (sello) con todas sus piezas
del mismo instante (referencia, medida, satelite y, dentro del pase, error y
enlace), y las filas salen en orden de sello: la misma corrida produce el
mismo archivo. Sin lockstep (hardware real) se graba con lo ultimo que llego.
"""
import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from atp_interfaces.msg import LinkStatus, PointingError, SatState
from atp_ros import model
from atp_ros.params import declare_num
from atp_ros.nodo import anunciar_listo
from atp_ros.qos import MEDICION


def _r(x, nd=6):
    return round(float(x), nd)


def _sello(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        self.declare_parameter('output_dir', '')
        declare_num(self, 'objective_deg', model.OBJETIVO_DEG)
        declare_num(self, 'sample_hz', 10.0)
        # velocidad real de la carga (deg/s) en cada fila: para contar los pasos en que la planta topo
        # su velocidad (estudio del MPC); apagado por defecto = el archivo de siempre
        self.declare_parameter('record_velocity', False)
        self.rec_vel = bool(self.get_parameter('record_velocity').value)
        out = str(self.get_parameter('output_dir').value)
        if not out:
            raise ValueError('recorder: falta el parametro output_dir')
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.obj = float(self.get_parameter('objective_deg').value)
        self.tele = open(os.path.join(out, 'telemetry.jsonl'), 'w', encoding='utf-8')

        self.paso_ns = int(round(1e9 / float(self.get_parameter('sample_hz').value)))
        # piezas por sello: 'ref', 'meas' (+ mando), 'sat', 'err', 'link'
        self.piezas = {}
        self.errs = {}         # sello -> (e_az, e_el, e_pt): base de las metricas
        self.links = {}        # sello -> (margen, fd, tasa, l_point, cerrado)
        self.band = ''
        self.last_t = None     # ultimo t escrito (la espera en AOS repite t=0)
        self.sat_ultimo = None
        self.finished = False
        self.cierre = None
        self.vistos = 0
        self.filas = 0
        self.modo_ultimo = False
        self.ultimo = {}

        q = 200
        self.create_subscription(JointState, '/atp/reference', self.on_ref, q)
        self.create_subscription(JointState, '/joint_states', self.on_joint, q)
        self.create_subscription(PointingError, '/atp/pointing_error', self.on_err, MEDICION)
        self.create_subscription(SatState, '/atp/sat_state', self.on_sat, q)
        self.create_subscription(LinkStatus, '/atp/link', self.on_link, MEDICION)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/atp/pass_done', self.on_done, latched)
        self.get_logger().info(f'grabando en {out}')

    def _poner(self, m, clave, valor):
        s = _sello(m)
        self.ultimo[clave] = valor
        if s % self.paso_ns == 0:      # solo los instantes de la rejilla de 10 Hz
            self.piezas.setdefault(s, {})[clave] = valor
        self._emitir()

    def on_ref(self, m):
        self._poner(m, 'ref', (math.degrees(m.position[0]), math.degrees(m.position[1])))

    def on_joint(self, m):
        u = (m.effort[0], m.effort[1]) if len(m.effort) >= 2 else None
        w = ((math.degrees(m.velocity[0]), math.degrees(m.velocity[1]))
             if self.rec_vel and len(m.velocity) >= 2 else None)
        self._poner(m, 'meas', (math.degrees(m.position[0]),
                                math.degrees(m.position[1]), u, w))

    def on_err(self, m):
        self.errs[_sello(m)] = (m.e_az_deg, m.e_el_deg, m.e_pt_deg)
        self._poner(m, 'err', (m.e_az_deg, m.e_el_deg, m.e_pt_deg))

    def on_link(self, m):
        self.band = m.band
        self.links[_sello(m)] = (m.margin_db, m.doppler_hz, m.doppler_rate_hz_s,
                                 m.l_point_db, bool(m.link_closed))
        self._poner(m, 'link', m)

    def on_sat(self, m):
        self.sat_ultimo = m
        self.vistos += 1
        if not self.modo_ultimo and self.vistos > 200 and self.filas == 0:
            self.modo_ultimo = True
            self.get_logger().warn('los sellos no coinciden (sin lockstep): '
                                   'se graba con lo ultimo que llego')
        if self.modo_ultimo:
            if _sello(m) % self.paso_ns == 0:
                self._fila(m, self.ultimo)
            return
        self._poner(m, 'sat', m)

    def _completa(self, p):
        if not all(k in p for k in ('ref', 'meas', 'sat')):
            return False
        sat = p['sat']
        en_pase = 0.0 < sat.t_since_aos_s < sat.pass_duration_s
        return not en_pase or ('err' in p and 'link' in p)

    def _emitir(self):
        # en orden de sello: el mas viejo pendiente sale primero; si lleva 5 s
        # simulados incompleto ya no se completara y se descarta
        while self.piezas:
            s = min(self.piezas)
            p = self.piezas[s]
            if self._completa(p):
                del self.piezas[s]
                if s % self.paso_ns == 0:
                    self._fila(p['sat'], p)
                continue
            if max(self.piezas) - s > 5_000_000_000:
                del self.piezas[s]
                continue
            return

    def _fila(self, sat, p):
        if self.finished:
            return
        t = float(sat.t_since_aos_s)
        if self.last_t is not None and t <= self.last_t:
            return
        self.last_t = t
        self.filas += 1
        row = {'t': _r(t, 3)}
        if p.get('ref'):
            row['ref_az'], row['ref_el'] = _r(p['ref'][0]), _r(p['ref'][1])
        meas = p.get('meas')
        if meas:
            row['az'], row['el'] = _r(meas[0]), _r(meas[1])
        en_pase = 0.0 < t < sat.pass_duration_s
        if en_pase and p.get('err'):
            row['e_az'], row['e_el'], row['e_pt'] = (_r(x) for x in p['err'])
        row['range_km'] = _r(sat.range_km, 3)
        row['range_rate_km_s'] = _r(sat.range_rate_km_s, 4)
        k = p.get('link') if en_pase else None
        if k:
            row.update(doppler_hz=_r(k.doppler_hz, 2),
                       doppler_rate_hz_s=_r(k.doppler_rate_hz_s, 2),
                       l_point_db=_r(k.l_point_db), cn0_dbhz=_r(k.cn0_dbhz, 3),
                       ebn0_db=_r(k.ebn0_db, 3), margin_db=_r(k.margin_db, 3),
                       link_closed=bool(k.link_closed))
        if meas and meas[2]:
            row['u_az'], row['u_el'] = _r(meas[2][0]), _r(meas[2][1])
        if meas and meas[3]:
            row['w_az'], row['w_el'] = _r(meas[3][0], 4), _r(meas[3][1], 4)
        self.tele.write(json.dumps(row) + '\n')
        self.tele.flush()

    def metrics(self):
        e = np.array([self.errs[k] for k in sorted(self.errs)],
                     dtype=float).reshape(-1, 3)
        n = len(e)
        s = self.sat_ultimo
        dur = float(s.pass_duration_s) if s is not None else 0.0
        if n == 0:
            return {'n': 0, 'duration_s': dur, 'verdict': 'SIN DATOS'}
        pt = e[:, 2]
        p95 = float(np.percentile(pt, 95))
        rms = lambda x: float(np.sqrt(np.mean(x ** 2)))  # noqa: E731
        out = {
            'n': n, 'duration_s': round(dur, 3),
            'pointing': {
                'rms': rms(pt), 'p95': p95, 'max': float(np.max(pt)),
                'within_objective_pct': float(np.mean(pt <= self.obj)) * 100.0,
                'objective_deg': self.obj},
            'axes': {
                'az': {'rms': rms(e[:, 0]), 'max': float(np.max(np.abs(e[:, 0])))},
                'el': {'rms': rms(e[:, 1]), 'max': float(np.max(np.abs(e[:, 1])))}},
            # mismo criterio que la campana Monte Carlo: p95 < objetivo
            'verdict': 'CUMPLE' if p95 < self.obj else 'NO CUMPLE',
        }
        if self.links:
            h = np.array([self.links[k] for k in sorted(self.links)], dtype=float)
            out['link'] = {
                'band': self.band,
                'margin_min_db': float(h[:, 0].min()),
                'margin_max_db': float(h[:, 0].max()),
                'closed_pct': float(np.mean(h[:, 4])) * 100.0,
                'doppler_max_hz': float(h[:, 1].max()),
                'doppler_min_hz': float(h[:, 1].min()),
                # la primera tasa es 0 por construccion (no hay muestra previa)
                'doppler_rate_max_hz_s': float(np.abs(h[1:, 2]).max()) if len(h) > 1 else 0.0,
                'l_point_max_db': float(h[:, 3].max())}
        return out

    def on_done(self, msg):
        if not msg.data or self.finished or self.cierre is not None:
            return
        # el monitor cierra su cuenta 1 s simulado despues de LOS (le llegan
        # los ultimos instantes); el grabador espera un poco mas para tener
        # exactamente las mismas muestras
        self.cierre = self.create_timer(1.5, self.cerrar)

    def cerrar(self):
        self.cierre.cancel()
        if self.finished:
            return
        self.finished = True
        self.tele.close()
        path = os.path.join(self.out, 'metrics.json')
        with open(path + '.tmp', 'w', encoding='utf-8') as f:
            json.dump(self.metrics(), f, indent=1)
        os.replace(path + '.tmp', path)
        self.get_logger().info(f'metrics.json escrito en {self.out}')
        raise SystemExit(0)


def main():
    rclpy.init()
    node = Recorder()
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
