"""Nodo `od_estimator`: determina la orbita con las mediciones de un pase.

Cierra el lazo de la orbita en el simulador (docs/OD.md): durante el pase
recoge, en lockstep (CONTRATO 8), las mediciones SELLADAS de la estacion y, al
recibir /atp/pass_done, ajusta el estado orbital y lo publica con su
covarianza para que `reference_generator` (profile:=od) lo use en el pase
siguiente.

Entradas (emparejadas por SELLO, nunca por "el ultimo que llego"):
  /joint_states   angulos de la montura (acimut, elevacion): lo que la antena
                  apunta de verdad; con el autotrack enganchado es la
                  direccion del satelite (0.004 deg rms en N5).
  /atp/sat_state  del generador: hace de VERDAD mientras no exista receptor.
                  De aqui salen el rango (`use_range`) y la tasa de rango
                  (`use_doppler`), a los que el nodo SUMA ruido gaussiano de
                  sigma `sigma_rng_km` / `sigma_rr_km_s` con una semilla
                  fija (`noise_seed`): corrida repetible. Los angulos NO se
                  perturban: ya traen el ruido y el error de la propia
                  montura.
  /atp/pass_done  fin del pase (latched).
Salidas:
  /atp/od_result  std_msgs/String (JSON, latched): estado TEME en la epoca del
                  primer dato, covarianza, residuos y modelo. Mismo formato
                  que `od.resultado_json` y que lee `reference_generator`.
  <output_dir>/od.json  el mismo JSON, si `output_dir` no esta vacio.

Dos fuentes del a priori: profile:=tle (el TLE de data/tle con el prior de
su edad) o profile:=od (la orbita de `od_file` propagada al pase con su
covarianza: encadena pases). Los parametros del pase (tle_name, station,
start_utc, pass_index...) son LOS MISMOS que los del generador: el nodo
calcula con ellos el AOS para poner fecha a las mediciones.

El ajuste corre en un hilo para no bloquear al ejecutor, y al cerrar el nodo
espera a que termine (el lanzamiento envia SIGINT en cuanto el grabador
escribe metrics.json). Modo 'lotes' (defecto) o 'ekf' (secuencial: el estado
ya esta listo al llegar /atp/pass_done).
"""
import json
import math
import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from atp_interfaces.msg import SatState

from atp_ros import medidas as M
from atp_ros import od, orbit
from atp_ros import propagador as P
from atp_ros.nodo import correr
from atp_ros.params import declare_num


def _sello(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class OdEstimator(Node):
    def __init__(self):
        super().__init__('od_estimator')
        d = self.declare_parameter
        d('profile', 'tle')               # tle | od: de donde sale el a priori
        d('tle_name', 'ISS')
        d('station', '')
        d('start_utc', '')
        d('od_file', '')
        d('output_dir', '')
        d('mode', 'lotes')                # lotes | ekf
        d('use_range', False)
        d('use_doppler', False)
        declare_num(self, 'pass_index', 0)
        declare_num(self, 'tle_mask_deg', -1.0)
        declare_num(self, 'tle_min_el_max_deg', 0.0)
        declare_num(self, 'tle_search_hours', 72.0)
        declare_num(self, 'decim_s', 5.0)            # una medida cada 5 s simulados
        declare_num(self, 'el_min_deg', 5.0)
        declare_num(self, 'el_max_deg', 85.0)        # cerca del cenit el acimut no sirve
        declare_num(self, 'sigma_az_deg', math.degrees(M.SIGMAS_DEFECTO['az']))
        declare_num(self, 'sigma_el_deg', math.degrees(M.SIGMAS_DEFECTO['el']))
        declare_num(self, 'sigma_rng_km', M.SIGMAS_DEFECTO['rng'])
        declare_num(self, 'sigma_rr_km_s', M.SIGMAS_DEFECTO['rr'])
        declare_num(self, 'noise_seed', 1)
        declare_num(self, 'cd_a_m', 0.0105 * 0.406)   # ISS efectivo (docs/OD.md)
        d('estimate_drag', False)
        g = lambda k: self.get_parameter(k).value    # noqa: E731
        self.g = g
        self.modo = str(g('mode'))
        self.canales = ['az', 'el'] + (['rng'] if g('use_range') else []) \
            + (['rr'] if g('use_doppler') else [])
        self.sg = {'az': math.radians(float(g('sigma_az_deg'))),
                   'el': math.radians(float(g('sigma_el_deg'))),
                   'rng': float(g('sigma_rng_km')), 'rr': float(g('sigma_rr_km_s'))}
        self.rng = np.random.default_rng(int(g('noise_seed')))
        self.modelo = P.Modelo(grado=2, cd_a_m=float(g('cd_a_m')))

        # pase que vuela el generador (mismos parametros): da el AOS y el a priori
        mask = float(g('tle_mask_deg'))
        if str(g('profile')) == 'od':
            self.perfil, pase, self.orbita_previa, self.est = od.perfil_od(
                str(g('od_file')), str(g('station')) or None, int(g('pass_index')),
                str(g('start_utc')) or None, float(g('tle_search_hours')),
                mask if mask >= 0.0 else None, float(g('tle_min_el_max_deg')))
            self.tle = None
        else:
            self.perfil, pase, self.tle, self.est = orbit.perfil_tle(
                str(g('tle_name')), str(g('station')) or None, int(g('pass_index')),
                str(g('start_utc')) or None, float(g('tle_search_hours')),
                mask if mask >= 0.0 else None, float(g('tle_min_el_max_deg')))
            self.orbita_previa = None
        self.aos = pase['aos']
        self.reloj = M.Reloj(self.aos)
        self.sitio = M.sitio(self.est)
        self.get_logger().info(
            f'od_estimator: pase {self.aos:%Y-%m-%d %H:%M:%S} UTC sobre {self.est.nombre}, '
            f'canales {"+".join(self.canales)}, modo {self.modo}, a priori '
            f'{"TLE" if self.tle else "orbita " + str(g("od_file"))}')

        self.joint = {}            # sello -> (az_rad, el_rad)
        self.sat = {}              # sello -> (t_aos, el_deg, rng_km, rr_km_s, duracion)
        self.filas = []            # (t, az, el, rng, rr) ya muestreadas
        self.t_ultima = -1e9
        self.hecho = False
        self.hilo = None
        self.resultado = None
        q = 200   # nodo pasivo: el reloj no lo espera; colas profundas
        self.create_subscription(JointState, '/joint_states', self.on_joint, q)
        self.create_subscription(SatState, '/atp/sat_state', self.on_sat, q)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/atp/pass_done', self.on_done, latched)
        self.pub = self.create_publisher(String, '/atp/od_result', latched)

    # ------------------------------------------------ recoleccion (emparejada por sello)
    def on_joint(self, msg):
        s = _sello(msg)
        self.joint[s] = (msg.position[0], msg.position[1])
        self._acotar(self.joint)
        self._intentar(s)

    def on_sat(self, msg):
        s = _sello(msg)
        self.sat[s] = (msg.t_since_aos_s, msg.el_deg, msg.range_km, msg.range_rate_km_s,
                       msg.pass_duration_s)
        self._acotar(self.sat)
        self._intentar(s)

    @staticmethod
    def _acotar(d):
        if len(d) > 400:
            d.pop(min(d))

    def _intentar(self, s):
        if s not in self.joint or s not in self.sat or self.hecho:
            return
        az, el = self.joint.pop(s)
        t, el_sat, rng, rr, dur = self.sat.pop(s)
        if not 0.0 < t < dur:
            return                       # fuera del pase
        if not float(self.g('el_min_deg')) <= el_sat <= float(self.g('el_max_deg')):
            return
        if t - self.t_ultima < float(self.g('decim_s')) - 1e-9:
            return
        self.t_ultima = t
        # el "receptor": rango y tasa de rango de la verdad + ruido de la semilla
        n_rng = self.rng.normal(0.0, self.sg['rng'])
        n_rr = self.rng.normal(0.0, self.sg['rr'])
        self.filas.append((t, az % (2 * math.pi), el, rng + n_rng, rr + n_rr))

    # ------------------------------------------------ ajuste
    def on_done(self, msg):
        if not msg.data or self.hecho:
            return
        self.hecho = True
        self.hilo = threading.Thread(target=self.ajustar, daemon=False)
        self.hilo.start()

    def _medidas(self):
        a = np.array(self.filas)
        n = len(a)
        return M.Medidas(self.reloj, [self.sitio], a[:, 0], np.zeros(n, int),
                         {'az': a[:, 1], 'el': a[:, 2], 'rng': a[:, 3], 'rr': a[:, 4]}
                         ).solo(self.canales)

    def ajustar(self):
        try:
            if len(self.filas) < 10:
                self.get_logger().error(f'OD: solo {len(self.filas)} medidas: no se ajusta')
                return
            med = self._medidas()
            t0 = float(med.t[0])
            estp = bool(self.g('estimate_drag'))
            if self.tle is not None:
                jd, fr = self.reloj.jdfr(t0)
                r, v = orbit.propagar_teme(self.tle, jd, fr)
                x0 = np.r_[r[0], v[0]]
                edad = self.tle.edad_dias(self.reloj.utc(t0))
                xb, Pb = od.prior_de_tle(x0, edad, estima_p=estp)
                fuente = f'TLE {self.tle.nombre} ({edad:.2f} dias de edad)'
            else:
                xb, Pb, estp_prev = od.prior_de_orbita(self.orbita_previa, self.reloj, t0)
                if estp and not estp_prev:      # empezar a estimar p con prior laxo (50 %)
                    Pn = np.zeros((7, 7))
                    Pn[:6, :6] = Pb
                    Pn[6, 6] = 0.25
                    xb, Pb = np.r_[xb, 1.0], Pn
                estp = estp or estp_prev
                fuente = f'orbita previa {self.g("od_file")}'
            if self.modo == 'ekf':
                res = od.ekf(med, xb, Pb, t0, self.modelo, self.sg, estp)
            else:
                res = od.ajuste_lotes(med, xb[:7 if estp else 6], t0, self.modelo, self.sg,
                                      estp, (xb, Pb))
            norad = self.tle.norad if self.tle else self.orbita_previa.get('norad')
            nombre = self.tle.nombre if self.tle else self.orbita_previa.get('satelite', '')
            info = od.resultado_json(
                res, self.reloj, nombre, norad, [self.est.nombre],
                {'fuente_a_priori': fuente, 'modo': self.modo, 'canales': self.canales,
                 'n_medidas': len(self.filas)})
            self.resultado = info
            texto = json.dumps(info)
            out = str(self.g('output_dir'))
            if out:
                os.makedirs(out, exist_ok=True)
                with open(os.path.join(out, 'od.json'), 'w') as f:
                    f.write(texto)
            chi2 = info.get('chi2_dof')
            self.get_logger().info(
                f'OD {self.modo}: {len(self.filas)} medidas ({"+".join(self.canales)}), '
                f'sigma_pos formal {res.sigma_pos_km():.3f} km'
                + (f', chi2/dof {chi2:.2f}, {info["iteraciones"]} iter' if chi2 is not None else '')
                + f'; a priori: {fuente}')
            try:
                if rclpy.ok():
                    self.pub.publish(String(data=texto))
            except Exception:       # el contexto ya cerro: el archivo basta
                pass
        except Exception as e:      # el hilo no debe morir en silencio
            self.get_logger().error(f'OD fallo: {e!r}')

    def destroy_node(self):
        if self.hilo is not None and self.hilo.is_alive():
            self.hilo.join(timeout=120.0)
        return super().destroy_node()


def main():
    correr(OdEstimator)
