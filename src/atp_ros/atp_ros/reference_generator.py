"""Publica la referencia Az/El que la antena debe seguir.

profile='ramp': rampa de acimut a velocidad constante (N1, lazo de 1 eje).
profile='pass': pase LEO completo con geometria esferica (N2, keyhole).
profile='tle': pase REAL de un satelite (SGP4 + TLE) sobre una estacion de
data/estaciones.yaml; el reloj arranca en el AOS del pase `pass_index`.
profile='handover': re-apuntamiento entre dos satelites: la referencia sigue al
satelite A y, en `ho_t_s`, salta al satelite B (otro acimut/elevacion): un salto de `ho_daz_deg` /
`ho_del_deg`; ambos se mueven a la tasa `ho_rate_*`.
profile='od': como 'tle' pero el pase se predice con la ORBITA ESTIMADA por el
nodo od_estimator (`od_file`, JSON de docs/OD.md) y el propagador numerico.

`truth_file` (opcional, perfiles tle/od; mision integrada, docs/MISION.md): un .npz
de `od.guardar_efemeride` con la orbita VERDADERA del satelite. La referencia sigue
siendo la prediccion (TLE u orbita estimada: lo que el operador sabe), pero el
satelite que ve la estacion es el verdadero: `/atp/sat_state` lleva SU direccion,
su rango y su tasa, y `/atp/truth` (JointState, rad) su direccion, contra la que
el monitor mide el apuntamiento. Sin `truth_file` todo es como siempre (la
prediccion hace de verdad).
"""
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from atp_interfaces.msg import SatState

from atp_ros import keyhole, model
from atp_ros.nodo import LISTO, correr
from atp_ros.params import declare_num


class ReferenceGenerator(Node):
    def __init__(self):
        super().__init__('reference_generator')
        self.declare_parameter('profile', 'pass')
        declare_num(self, 'rate_hz', 20.0)
        declare_num(self, 'start_delay_s', 1.5)
        declare_num(self, 'ramp_deg_s', 5.0)
        declare_num(self, 'ramp_duration_s', 20.0)
        declare_num(self, 'h_km', model.H_LEO_KM)
        declare_num(self, 'el_max_deg', 85.0)
        declare_num(self, 'mask_deg', 70.0)
        declare_num(self, 'az_culmination_deg', 140.0)
        # cuantos nodos deben estar suscritos a /atp/sat_state (monitor,
        # enlace, grabador...) antes de arrancar: si uno llega tarde se
        # perderia el principio del pase y la corrida no seria repetible
        declare_num(self, 'wait_subscribers', 0)
        # y, ya presentes, un margen de PARED para que DDS termine de emparejar
        # todos los topicos entre ellos (en lockstep a maxima velocidad, 1.5 s
        # simulados duran milisegundos de pared)
        declare_num(self, 'wait_wall_s', 0.2)
        # nodos que deben existir antes de arrancar (separados por comas): cada
        # nodo crea todos sus topicos en el constructor, asi que si existe,
        # tras el margen de pared sus topicos ya estan emparejados. Sin esto,
        # la primera referencia se pierde si p. ej. el autotrack aun no
        # descubrio /atp/program, y con la compuerta el lazo se cuelga.
        self.declare_parameter('wait_nodes', '')
        self.listo_desde = None
        self.esperados = {n.strip() for n in
                          str(self.get_parameter('wait_nodes').value).split(',') if n.strip()}
        self.listos = set()
        self.create_subscription(String, '/atp/listo',
                                 lambda m: self.listos.add(m.data), LISTO)
        self.listo = (int(self.get_parameter('wait_subscribers').value) <= 0
                      and not self.esperados)
        if not self.listo:
            # de PARED: con la compuerta el reloj simulado esta parado
            self.create_timer(0.05, self.vigilar_grafo,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))
        # perfil 'tle' (Sprint 4)
        self.declare_parameter('tle_name', 'ISS')
        self.declare_parameter('station', '')       # vacio: la por_defecto del YAML
        self.declare_parameter('start_utc', '')     # vacio: ahora
        declare_num(self, 'pass_index', 0)
        declare_num(self, 'tle_mask_deg', -1.0)     # <0: la mascara de la estacion
        declare_num(self, 'tle_min_el_max_deg', 0.0)
        declare_num(self, 'tle_search_hours', 72.0)
        self.declare_parameter('od_file', '')       # perfil 'od': JSON del od_estimator
        # keyhole: defectos = comportamiento de siempre
        self.declare_parameter('keyhole_strategy', 'ingenua')   # ingenua|anticipada|sobre_cenit
        declare_num(self, 'vel_plan_deg_s', 6.0)     # tope de velocidad de cada articulacion
        declare_num(self, 'acel_plan_deg_s2', 20.0)  # tope de aceleracion (planificador)
        declare_num(self, 'el_montura_max_deg', 90.0)  # 180: la montura admite sobre_cenit
        declare_num(self, 'pass_n', 1200)            # muestras del perfil sintetico
        declare_num(self, 'tle_dt_s', 0.5)           # paso del perfil SGP4 (0.05-0.1 en keyhole)
        self.declare_parameter('stations_file', '')  # YAML de estaciones (vacio: data/)
        self.declare_parameter('truth_file', '')    # .npz con la orbita verdadera (opcional)
        # perfil handover
        declare_num(self, 'ho_t_s', 20.0)
        declare_num(self, 'ho_dur_s', 60.0)
        declare_num(self, 'ho_az0_deg', 100.0)
        declare_num(self, 'ho_el0_deg', 30.0)
        declare_num(self, 'ho_daz_deg', 135.0)
        declare_num(self, 'ho_del_deg', 15.0)
        declare_num(self, 'ho_rate_az', 0.5)
        declare_num(self, 'ho_rate_el', 0.0)

        p = self.get_parameter
        self.profile = p('profile').value
        self.verdad = None           # perfil del satelite verdadero (truth_file), si lo hay
        self.v_prof = None           # velocidad explicita de la referencia (perfil handover)
        if self.profile == 'ramp':
            v = float(p('ramp_deg_s').value)
            dur = float(p('ramp_duration_s').value)
            self.t = np.linspace(0.0, dur, 400)
            self.az = 10.0 + v * self.t
            self.el = np.full_like(self.t, 30.0)
            self.d = np.full_like(self.t, 1000.0)
            self.vr = np.zeros_like(self.t)
        elif self.profile == 'pass':
            perfil = model.perfil_pase(
                p('h_km').value, p('el_max_deg').value, p('mask_deg').value,
                p('az_culmination_deg').value, n=int(p('pass_n').value))
            self.t, self.az, self.el = (perfil['t'], perfil['az_continuo'],
                                        perfil['el'])
            self.d, self.vr = perfil['d'], perfil['vr']
            self.get_logger().info(
                f"pase: duracion {perfil['duracion']:.1f} s, "
                f"az_pto max {np.max(np.abs(perfil['az_pto'])):.2f} deg/s, "
                f"el max {np.max(self.el):.1f} deg")
        elif self.profile == 'handover':
            self.init_handover()
        elif self.profile == 'tle':
            self.init_tle()
        elif self.profile == 'od':
            self.init_od()
        else:
            raise ValueError(f'profile desconocido: {self.profile}')

        # estrategia de keyhole: la orden a las articulaciones difiere de la
        # direccion del satelite (self.az/self.el pasan a ser la VERDAD)
        self.estrategia = str(p('keyhole_strategy').value)
        self.cmd_az = self.cmd_el = None
        if self.estrategia != 'ingenua':
            self.init_keyhole()

        self.pub = self.create_publisher(JointState, '/atp/reference', 10)
        self.sat_pub = self.create_publisher(SatState, '/atp/sat_state', 10)
        self.truth_pub = (self.create_publisher(JointState, '/atp/truth', 10)
                          if (self.verdad is not None or self.cmd_az is not None) else None)
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.done_pub = self.create_publisher(Bool, '/atp/pass_done', latched)
        # La referencia se congela en AOS hasta ver el primer mando del PID:
        # con el reloj acelerado, arrancar por tiempo dejaba al satelite
        # moverse antes de que el lazo estuviera cerrado.
        self.t_ini = None
        self.create_subscription(JointState, '/atp/torque',
                                 self.on_torque, 10)
        self.done = False
        self.create_timer(1.0 / float(p('rate_hz').value), self.tick)
        self.delay = float(p('start_delay_s').value)

    def init_handover(self):
        """Re-apuntamiento A -> B: salto de la referencia en ho_t_s (el reloj del lockstep cae en
        la rejilla de 0.05 s y el salto es entre dos ticks)."""
        p = self.get_parameter
        dt = 0.05
        self.t = np.arange(0.0, float(p('ho_dur_s').value) + 1e-9, dt)
        despues = self.t >= float(p('ho_t_s').value) - 1e-9
        self.az = (float(p('ho_az0_deg').value) + float(p('ho_rate_az').value) * self.t
                   + np.where(despues, float(p('ho_daz_deg').value), 0.0))
        self.el = (float(p('ho_el0_deg').value) + float(p('ho_rate_el').value) * self.t
                   + np.where(despues, float(p('ho_del_deg').value), 0.0))
        self.d = np.full_like(self.t, 1000.0)
        self.vr = np.zeros_like(self.t)
        # velocidad de la referencia: la de cada satelite (el salto no es una velocidad)
        self.v_prof = (np.full_like(self.t, float(p('ho_rate_az').value)),
                       np.full_like(self.t, float(p('ho_rate_el').value)))
        self.get_logger().info(
            f"handover: salto de ({float(p('ho_daz_deg').value):.0f}, "
            f"{float(p('ho_del_deg').value):.0f}) deg en t={float(p('ho_t_s').value):.1f} s")

    def init_tle(self):
        """Pase real: SGP4 sobre el TLE de data/tle, ver orbit.perfil_tle."""
        from atp_ros import orbit
        p = self.get_parameter
        mask = float(p('tle_mask_deg').value)
        perfil, pase, tle, est = orbit.perfil_tle(
            p('tle_name').value, p('station').value or None,
            int(p('pass_index').value), p('start_utc').value or None,
            float(p('tle_search_hours').value), mask if mask >= 0.0 else None,
            float(p('tle_min_el_max_deg').value), dt_s=float(p('tle_dt_s').value),
            archivo_estaciones=p('stations_file').value or None)
        self.t, self.az, self.el = (perfil['t'], perfil['az_continuo'],
                                    perfil['el'])
        self.d, self.vr = perfil['d'], perfil['vr']
        edad = tle.edad_dias(pase['aos'])
        self.get_logger().info(
            f"pase real {tle.nombre} (NORAD {tle.norad}) sobre {est.nombre}: "
            f"AOS {pase['aos']:%Y-%m-%d %H:%M:%S} UTC, duracion "
            f"{perfil['duracion']:.1f} s, el max {pase['el_max_deg']:.1f} deg, "
            f"az {pase['az_aos_deg']:.0f}->{pase['az_los_deg']:.0f} deg, "
            f"rango min {pase['rango_min_km']:.0f} km, "
            f"az_pto max {np.max(np.abs(perfil['az_pto'])):.2f} deg/s")
        if abs(edad) > 7.0:
            self.get_logger().warn(
                f'el TLE tiene {edad:.1f} dias de edad en el pase: el error de '
                f'SGP4 crece ~1-3 km/dia; refrescar con scripts/actualizar_tle.sh')
        if pase['aos_truncado']:
            self.get_logger().warn('el pase ya estaba en curso al inicio de la '
                                   'ventana: AOS es el instante inicial')
        self.init_verdad(pase, est)

    def init_keyhole(self):
        """Orden de articulaciones del pase (keyhole.planificar). No aplica a la
        rampa; si la estrategia no puede usarse (sobre_cenit con una montura
        limitada) el nodo se niega a arrancar: no hay sustituto silencioso."""
        p = self.get_parameter
        if self.profile == 'ramp':
            raise ValueError('keyhole_strategy no aplica al perfil ramp')
        plan = keyhole.planificar(
            self.t, self.az, self.el, self.estrategia, float(p('vel_plan_deg_s').value),
            float(p('acel_plan_deg_s2').value), float(p('el_montura_max_deg').value))
        if not plan['aplica']:
            raise ValueError(f"keyhole_strategy={self.estrategia}: {plan['motivo']}")
        self.cmd_az, self.cmd_el = plan['az_cmd'], plan['el_cmd']
        self.get_logger().info(
            f"keyhole {self.estrategia}: modo {plan['modo']}, error planificado "
            f"(cota inferior) {plan['eps_deg']:.3f} deg, el' max {np.max(self.cmd_el):.1f} deg")

    def init_od(self):
        """Pase predicho con la orbita estimada (od.perfil_od)."""
        from atp_ros import od, orbit
        p = self.get_parameter
        mask = float(p('tle_mask_deg').value)
        perfil, pase, orbita, est = od.perfil_od(
            p('od_file').value, p('station').value or None,
            int(p('pass_index').value), p('start_utc').value or None,
            float(p('tle_search_hours').value), mask if mask >= 0.0 else None,
            float(p('tle_min_el_max_deg').value))
        self.t, self.az, self.el = (perfil['t'], perfil['az_continuo'],
                                    perfil['el'])
        self.d, self.vr = perfil['d'], perfil['vr']
        edad = (pase['aos'] - orbit.a_utc(orbita['epoca_utc'])).total_seconds() / 86400.0
        self.get_logger().info(
            f"pase por ORBITA ESTIMADA ({orbita.get('satelite', '')}, epoca "
            f"{orbita['epoca_utc']}, sigma_pos {orbita.get('sigma_pos_km', float('nan')):.3f} km) "
            f"sobre {est.nombre}: AOS {pase['aos']:%Y-%m-%d %H:%M:%S} UTC, duracion "
            f"{perfil['duracion']:.1f} s, el max {pase['el_max_deg']:.1f} deg, "
            f"az {pase['az_aos_deg']:.0f}->{pase['az_los_deg']:.0f} deg")
        if edad > 3.0:
            self.get_logger().warn(f'la orbita estimada tiene {edad:.1f} dias en el pase: el error '
                                   f'de prediccion crece; repetir la OD con un pase reciente')
        self.init_verdad(pase, est)

    def init_verdad(self, pase, est):
        """Con `truth_file`: el pase del satelite VERDADERO muestreado en la misma
        rejilla de tiempo (t desde el AOS predicho) que la prediccion."""
        ruta = str(self.get_parameter('truth_file').value)
        if not ruta:
            return
        from atp_ros import medidas as M
        from atp_ros import od
        ef, reloj = od.cargar_efemeride(ruta)
        pv = od.perfil_de_efemeride(
            ef, reloj, M.sitio(est),
            {'aos': reloj.segundos(pase['aos']), 'duracion_s': pase['duracion_s']})
        # el acimut desenroscado de cada perfil parte de su primera muestra: si el
        # pase cruza el norte en AOS pueden diferir en 360 grados
        k = round((float(self.az[0]) - float(pv['az_continuo'][0])) / 360.0)
        pv['az_continuo'] = pv['az_continuo'] + 360.0 * k
        self.verdad = pv
        sep = float(M.separacion_deg(math.radians(self.az[0]), math.radians(self.el[0]),
                                     math.radians(pv['az_continuo'][0]),
                                     math.radians(pv['el'][0])))
        self.get_logger().info(
            f"satelite VERDADERO ({ruta}): en AOS a {sep:.3f} deg de la prediccion, "
            f"rango {pv['d'][0]:.1f} km (prediccion {self.d[0]:.1f})")

    def vigilar_grafo(self):
        if self.listo:
            return
        if self.get_clock().now().nanoseconds == 0:
            return            # aun sin /clock: la referencia saldria con sello 0
        # no basta con que existan: cada uno debe haber anunciado que sus
        # suscripciones estan emparejadas (saludo de arranque, nodo.py)
        faltan = self.esperados - self.listos
        if faltan or self.sat_pub.get_subscription_count() < int(
                self.get_parameter('wait_subscribers').value):
            self.listo_desde = None
            return
        ahora = time.monotonic()
        if self.listo_desde is None:
            self.listo_desde = ahora
        if ahora - self.listo_desde >= float(self.get_parameter('wait_wall_s').value):
            self.listo = True
            self.get_logger().info('grafo listo: se publica la referencia')
            self.tick()

    def on_torque(self, _msg):
        if self.t_ini is None:
            self.t_ini = self.get_clock().now()
            self.get_logger().info('lazo cerrado: arranca el pase')

    def tick(self):
        if not self.listo:
            return
        now = self.get_clock().now()
        if self.t_ini is None:
            t = 0.0
        else:
            t = (now - self.t_ini).nanoseconds * 1e-9 - self.delay
        t_raw = t if self.t_ini is not None else -self.delay
        if t < 0.0:
            t = 0.0   # mantiene la referencia en AOS mientras arrancan los demas
        if t > self.t[-1]:
            # LOS: se anuncia una vez y se sostiene la ultima posicion, para
            # que el lazo en lockstep siga confirmando pasos hasta el cierre
            t = float(self.t[-1])
            if not self.done:
                self.done = True
                self.done_pub.publish(Bool(data=True))
                self.get_logger().info('fin del pase')
        az = float(np.interp(t, self.t, self.az))
        el = float(np.interp(t, self.t, self.el))
        msg = JointState()
        msg.header.stamp = now.to_msg()   # el mismo instante con que se calculo t
        msg.name = ['az', 'el']
        if self.cmd_az is None:
            msg.position = [math.radians(az), math.radians(el)]
        else:     # keyhole: la referencia es la ORDEN; az/el siguen siendo la verdad
            msg.position = [math.radians(float(np.interp(t, self.t, self.cmd_az))),
                            math.radians(float(np.interp(t, self.t, self.cmd_el)))]
        self.pub.publish(msg)
        sat = SatState()
        sat.header = msg.header
        sat.az_deg, sat.el_deg = az, el
        sat.range_km = float(np.interp(t, self.t, self.d))
        sat.range_rate_km_s = float(np.interp(t, self.t, self.vr))
        if self.verdad is not None:
            # el satelite que ve la estacion es el verdadero, no la prediccion
            pv = self.verdad
            sat.az_deg = float(np.interp(t, pv['t'], pv['az_continuo']))
            sat.el_deg = float(np.interp(t, pv['t'], pv['el']))
            sat.range_km = float(np.interp(t, pv['t'], pv['d']))
            sat.range_rate_km_s = float(np.interp(t, pv['t'], pv['vr']))
            tr = JointState()
            tr.header.stamp = msg.header.stamp
            tr.name = ['az', 'el']
            tr.position = [math.radians(sat.az_deg), math.radians(sat.el_deg)]
            self.truth_pub.publish(tr)
        elif self.truth_pub is not None:      # keyhole sin truth_file: la verdad es el perfil
            tr = JointState()
            tr.header.stamp = msg.header.stamp
            tr.name = ['az', 'el']
            tr.position = [math.radians(az), math.radians(el)]
            self.truth_pub.publish(tr)
        sat.t_since_aos_s = float(t)
        sat.pass_duration_s = float(self.t[-1])
        self.sat_pub.publish(sat)


def main():
    correr(ReferenceGenerator)
