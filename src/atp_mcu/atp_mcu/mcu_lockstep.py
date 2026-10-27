"""Banco HIL en lockstep: el firmware (o su nodo de Linux) EN LUGAR de la montura simulada.

Sustituye a `mount_sim` + `pid_controller` del lazo de la Estacion (CONTRATO.md sec. 8):
el lazo de posicion lo cierra el microcontrolador, que recibe /atp/reference por micro-ROS
y publica /atp/mcu/<eje>/joint_states. Este nodo es el puente con el reloj en lockstep:

  /clock t_k ──▶ el generador publica /atp/reference(t_k) ──▶ agente ──▶ MCU
  MCU ──▶ eco /atp/mcu/eco (sello de la referencia) ──▶ aqui: latencia de ida y vuelta
  este nodo publica /joint_states(t_k) = estado del MCU en el instante de pared T_k,
  y confirma /atp/step_ack(t_k) cuando (a) llego el eco de t_k (el MCU YA tiene la
  referencia) y (b) la pared alcanzo T_{k+1}.

Por que se marca el ritmo aqui: el MCU gira en TIEMPO REAL (1 kHz de pared). El lockstep
garantiza que cada estado y cada referencia llevan el mismo sello (nada de emparejar
instantes distintos), pero ya NO que el resultado sea independiente de la carga de la
maquina: si el host se atrasa, el paso se alarga (se registra), no se falsea. Se usa
sim_clock con time_scale 0: el ritmo lo da la confirmacion de este nodo.
Relacion simulado-pared: T_k = T_0 + (t_k - t_0), x1.

Al terminar el pase (/atp/pass_done) calcula, con los mismos datos, el gemelo digital
(`atp_mcu.gemelo`: PlantaMontura + PID de atp_ros a 20 Hz) y escribe `salida`.json y .csv:
latencia de ida y vuelta, diagnostico del lazo del MCU y deriva contra el gemelo.

Parametros:
  axes [az, el]         ejes del MCU
  step_s 0.05           paso del reloj (el de sim_clock)
  usar_eco true         el MCU publica /atp/mcu/eco (ATP_ECO_LATENCIA / eje_host --eco)
  esperar_eco_s 0.3     si el eco no llega, el paso se confirma igual y se cuenta como perdido
  origen cmd|none       cmd: manda cmd/origen = referencia inicial y cmd/enable (como con
                        una placa real); none: el MCU ya esta alineado y armado (eje_host --armar --theta0)
  salida ''             prefijo de los archivos de resultado ('' = no escribe)
  kp ki kd tau u_max    del PID del gemelo
"""
import collections
import csv
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, Float64, Int64

from atp_mcu import gemelo
from atp_mcu.mcu_core import decodificar_diag
from atp_ros.nodo import correr
from atp_ros.params import declare_num


def _ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class McuLockstep(Node):
    def __init__(self):
        super().__init__('mcu_lockstep')
        self.declare_parameter('axes', ['az', 'el'])
        declare_num(self, 'step_s', 0.05)
        self.declare_parameter('usar_eco', True)
        declare_num(self, 'esperar_eco_s', 0.3)
        self.declare_parameter('origen', 'cmd')
        self.declare_parameter('salida', '')
        declare_num(self, 'timeout_arranque_s', 20.0)
        for k, v in (('kp', 25.0), ('ki', 2.0), ('kd', 8.0), ('tau', 0.08), ('u_max', 8.0)):
            declare_num(self, k, v)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.ejes = list(g('axes'))
        self.step_ns = int(round(float(g('step_s')) * 1e9))
        self.usar_eco = bool(g('usar_eco'))
        self.esperar_eco = float(g('esperar_eco_s'))
        self.origen_cmd = str(g('origen')) == 'cmd'
        self.salida = str(g('salida'))

        # ---- estado del puente
        self.muestras = {e: collections.deque(maxlen=4000) for e in self.ejes}  # (t_mcu_ns, pos, vel, eff)
        self.estado_mcu = {}
        self.diag_ult = {}
        self.diag_max = {e: {'t_lazo_max_us': 0.0, 'jitter_max_us': 0.0, 'ticks_perdidos': 0.0}
                         for e in self.ejes}
        self.pasos = {}                   # k -> dict (ver _paso)
        self.k_primero = None
        self.k_sig = None                 # proximo paso por cerrar
        self.t_primero_ns = None          # t_k (sim) del primer paso
        self.pared_primero_ns = None      # T_k de pared del primer paso
        self.armado = False
        self.t_armado = None
        self.t_origen_ok = None
        self.anunciado = False
        self.fin = False
        self.filas = []                   # filas del resultado, una por paso cerrado
        self.n_eco_perdidos = 0
        self.n_tarde = 0
        self.adelanto_ns = 0              # cuanto se adelanta el ack (se adapta, ver _cerrar)
        self.t_arranque = time.monotonic()
        self.avisado_arranque = False

        # ---- ROS
        q = 200
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(JointState, '/atp/reference', self.on_ref, q)
        for e in self.ejes:
            self.create_subscription(JointState, f'/atp/mcu/{e}/joint_states',
                                     lambda m, e=e: self.on_medida(e, m), q)
            self.create_subscription(Float32MultiArray, f'/atp/mcu/{e}/diag',
                                     lambda m, e=e: self.on_diag(e, m), qos_profile_sensor_data)
        if self.usar_eco:
            self.create_subscription(Int64, '/atp/mcu/eco', self.on_eco, q)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/atp/pass_done', self.on_done, latched)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 50)
        self.pub_ack = self.create_publisher(Clock, '/atp/step_ack', 10)
        self.pub_lazo = self.create_publisher(JointState, '/atp/torque', 10)
        self.pub_en = {e: self.create_publisher(Bool, f'/atp/mcu/{e}/cmd/enable', 10) for e in self.ejes}
        self.pub_or = {e: self.create_publisher(Float64, f'/atp/mcu/{e}/cmd/origen', 10) for e in self.ejes}
        # temporizador de PARED (el reloj simulado esta parado mientras se espera)
        from rclpy.clock import Clock as RClock, ClockType
        self.create_timer(0.001, self.avanzar, clock=RClock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'puente HIL en lockstep: ejes {self.ejes}, eco={"si" if self.usar_eco else "no"}, '
            f'origen={"cmd" if self.origen_cmd else "ninguno"}')

    # ------------------------------------------------------------ utilidades
    @staticmethod
    def _pared_ns():
        return time.time_ns()          # REALTIME: el MCU sella con la hora del agente (el mismo reloj)

    def _paso(self, k):
        return self.pasos.setdefault(k, {'clock_pared': None, 'ref': None, 'ref_pared': None,
                                         'eco_pared': None, 'cerrado': False})

    def _k(self, t_ns):
        return int(round(t_ns / self.step_ns))

    # ------------------------------------------------------------ callbacks
    def on_clock(self, msg):
        t = _ns(msg.clock)
        if t <= 0:
            return
        p = self._paso(self._k(t))
        if p['clock_pared'] is None:
            p['clock_pared'] = self._pared_ns()

    def on_ref(self, msg):
        t = _ns(msg.header.stamp)
        if t <= 0 or len(msg.position) < len(self.ejes):
            return
        k = self._k(t)
        p = self._paso(k)
        if p['ref'] is None:
            p['ref'] = np.array(msg.position[:len(self.ejes)], dtype=float)
            p['ref_pared'] = self._pared_ns()
        if self.k_primero is None:
            self.k_primero = self.k_sig = k
            self.t_primero_ns = t
            if self.origen_cmd:
                for i, e in enumerate(self.ejes):
                    self.pub_or[e].publish(Float64(data=float(msg.position[i])))
                self.t_origen_pub = time.monotonic()
                self.get_logger().info('primera referencia: se declara el origen de los ejes (cmd/origen)')

    def on_medida(self, eje, msg):
        if not msg.position:
            return
        self.muestras[eje].append((_ns(msg.header.stamp), float(msg.position[0]),
                                   float(msg.velocity[0]) if msg.velocity else 0.0,
                                   float(msg.effort[0]) if msg.effort else 0.0))

    def on_diag(self, eje, msg):
        d = decodificar_diag(msg.data)
        if d is None:
            return
        self.estado_mcu[eje] = d['estado_nombre']
        self.diag_ult[eje] = d
        m = self.diag_max[eje]
        m['t_lazo_max_us'] = max(m['t_lazo_max_us'], d['t_lazo_max_us'])
        m['jitter_max_us'] = max(m['jitter_max_us'], d['jitter_max_us'])
        m['ticks_perdidos'] = d['ticks_perdidos']

    def on_eco(self, msg):
        k = self._k(int(msg.data))
        p = self.pasos.get(k)
        if p is not None and p['eco_pared'] is None:
            p['eco_pared'] = self._pared_ns()

    def on_done(self, msg):
        if msg.data and not self.fin:
            self.fin = True
            self.finalizar()

    # ------------------------------------------------------------ interpolacion del estado del MCU
    def _estado_en(self, pared_ns):
        """Estado de cada eje en el instante de pared dado, interpolando entre las dos
        muestras que lo rodean. None si aun no llegaron muestras posteriores."""
        pos, vel, eff = [], [], []
        for e in self.ejes:
            m = self.muestras[e]
            if len(m) < 2 or m[-1][0] < pared_ns:
                return None
            t = [x[0] for x in m]
            y = np.array([x[1:] for x in m])
            r = gemelo.interpolar(pared_ns, t, y)
            if r is None:                      # pared anterior a la primera muestra: la primera
                r = y[0]
            pos.append(r[0]); vel.append(r[1]); eff.append(r[2])
        return pos, vel, eff

    # ------------------------------------------------------------ bucle de pared
    def avanzar(self):
        if self.k_sig is None:
            if not self.avisado_arranque and \
                    time.monotonic() - self.t_arranque > float(self.get_parameter('timeout_arranque_s').value):
                self.avisado_arranque = True
                self.get_logger().warn('sin /atp/reference tras el tiempo de arranque')
            return
        ahora = self._pared_ns()

        # ---- armado: origen declarado -> cmd/enable -> el estado del MCU ya sigue al origen
        if not self.armado:
            if self.origen_cmd and time.monotonic() - self.t_origen_pub < 0.15:
                return                            # dejar que cmd/origen llegue antes que cmd/enable
            if self.t_armado is None:
                if self.origen_cmd:
                    for e in self.ejes:
                        self.pub_en[e].publish(Bool(data=True))
                    self.get_logger().info('cmd/enable=true')
                self.t_armado = time.monotonic()
            ref0 = self.pasos[self.k_primero]['ref']
            est = self._estado_en(ahora - 10_000_000)       # una muestra de hace 10 ms o mas reciente
            ok = est is not None and all(abs(est[0][i] - ref0[i]) < 0.01 for i in range(len(self.ejes)))
            if not ok:
                if time.monotonic() - self.t_armado > 5.0 and not self.avisado_arranque:
                    self.avisado_arranque = True
                    self.get_logger().error(
                        f'el MCU no sigue el origen declarado tras 5 s: estado={est}, ref0={ref0}')
                return
            self.armado = True
            self.pared_primero_ns = ahora
            self.get_logger().info('MCU alineado con la referencia inicial: arranca el lockstep')

        # ---- cierra los pasos pendientes, en orden
        while True:
            k = self.k_sig
            p = self.pasos.get(k)
            if p is None or p['ref'] is None:
                return                            # la referencia de este paso aun no llego
            T_k = self.pared_primero_ns + (k - self.k_primero) * self.step_ns
            if ahora < T_k:
                return
            est = self._estado_en(T_k)
            if est is None:
                if ahora - T_k > 250_000_000:      # el MCU no entrega datos: se avisa y se sigue
                    self.n_tarde += 1
                    est = self._estado_en(min(T_k, self._ultimo_mcu_ns()))
                    if est is None:
                        return
                else:
                    return
            # estado publicado con el sello t_k del paso
            self._publicar_estado(k, est)
            p['estado'] = est
            p['T_pared'] = T_k
            # ---- confirmacion: el eco de este paso y la pared en T_{k+1}
            T_sig = T_k + self.step_ns
            if self.usar_eco and p['eco_pared'] is None:
                if (ahora - p['ref_pared']) < self.esperar_eco * 1e9:
                    return                         # el MCU aun no confirma que recibio la referencia
                self.n_eco_perdidos += 1
                p['eco_perdido'] = True
            if ahora < T_sig - self.adelanto_ns:
                return
            self._cerrar(k, p, T_sig)
            self.k_sig = k + 1
            ahora = self._pared_ns()
            if not self.anunciado:
                self._revisar_lazo_cerrado()

    def _ultimo_mcu_ns(self):
        return min(self.muestras[e][-1][0] for e in self.ejes if self.muestras[e]) \
            if all(self.muestras[e] for e in self.ejes) else 0

    def _publicar_estado(self, k, est):
        # el estado se publica una sola vez por paso (los reintentos no lo duplican)
        p = self.pasos[k]
        if p.get('publicado'):
            return
        p['publicado'] = True
        m = JointState()
        t = k * self.step_ns
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(t, 1_000_000_000)
        m.name = list(self.ejes)
        m.position, m.velocity, m.effort = [list(map(float, x)) for x in est]
        self.pub_js.publish(m)

    def _cerrar(self, k, p, T_sig):
        t = k * self.step_ns
        ack = Clock()
        ack.clock.sec, ack.clock.nanosec = divmod(t, 1_000_000_000)
        self.pub_ack.publish(ack)
        retraso = self._pared_ns() - T_sig          # ns; >0: el ack salio despues de T_{k+1}
        p['cerrado'] = True
        rtt = None
        if p['eco_pared'] is not None and p['ref_pared'] is not None:
            rtt = (p['eco_pared'] - p['ref_pared']) * 1e-6          # ms
        self.filas.append({
            'k': k, 't_s': t * 1e-9, 'ref': p['ref'], 'meas': np.array(p['estado'][0]),
            'rtt_ms': rtt, 'eco_perdido': bool(p.get('eco_perdido')),
            'retraso_cierre_ms': retraso * 1e-6,
        })
        # el ack llega a sim_clock, que publica t_{k+1} y el generador la referencia: todo eso tarda
        # (ejecutor de Python bajo carga). Un control proporcional adelanta el ack lo que haga falta
        # para que la referencia salga en T_{k+1} y no despues (la referencia llegaria al MCU tarde)
        self.adelanto_ns = int(min(max(self.adelanto_ns + 0.3 * retraso, 0.0), 40e6))
        # memoria: los pasos viejos ya no hacen falta
        self.pasos.pop(k - 50, None)

    def _revisar_lazo_cerrado(self):
        """Como mcu_merge: cuando todos los ejes estan ACTIVOS se anuncia (/atp/torque) y el
        generador arranca el pase (hasta entonces mantiene la referencia en AOS)."""
        if all(self.estado_mcu.get(e) == 'ACTIVO' for e in self.ejes):
            self.anunciado = True
            a = JointState()
            a.header.stamp = self.get_clock().now().to_msg()
            a.name = list(self.ejes)
            a.effort = [0.0] * len(self.ejes)
            self.pub_lazo.publish(a)
            self.get_logger().info('MCU ACTIVO en todos los ejes: lazo cerrado, arranca el pase')

    # ------------------------------------------------------------ resultados
    def finalizar(self):
        if not self.filas:
            self.get_logger().warn('fin del pase sin pasos cerrados: no hay resultado')
            return
        # solo desde que el pase arranco (la espera previa en AOS es el calentamiento del lazo)
        filas = self.filas
        refs = np.array([f['ref'] for f in filas])
        meas = np.array([f['meas'] for f in filas])
        g = lambda k: float(self.get_parameter(k).value)  # noqa: E731
        twin, _ = gemelo.gemelo_pid(refs, dt=self.step_ns * 1e-9, kp=g('kp'), ki=g('ki'), kd=g('kd'),
                                    tau=g('tau'), u_max=g('u_max'))
        rtt = [f['rtt_ms'] for f in filas if f['rtt_ms'] is not None]
        ciclo = [f['retraso_cierre_ms'] for f in filas]
        res = {
            'pasos': len(filas), 'duracion_s': filas[-1]['t_s'] - filas[0]['t_s'],
            'step_s': self.step_ns * 1e-9, 'ejes': self.ejes,
            # receptor de la referencia en este nodo -> eco del MCU; los negativos son ruido del ejecutor
            # de Python (el eco se proceso antes que la referencia): se cuentan, no se ocultan
            'rtt_ms': gemelo.percentiles(rtt, (50, 95, 99)),
            'rtt_negativos': int(sum(1 for x in rtt if x < 0.0)),
            'eco_perdidos': self.n_eco_perdidos, 'pasos_con_mcu_tarde': self.n_tarde,
            'retraso_de_cierre_ms': gemelo.percentiles(ciclo, (50, 95, 99)),
            'adelanto_final_del_ack_ms': self.adelanto_ns * 1e-6,
            'diag_mcu': {e: {'t_lazo_max_us': self.diag_max[e]['t_lazo_max_us'],
                             'jitter_max_us': self.diag_max[e]['jitter_max_us'],
                             'ticks_perdidos': self.diag_max[e]['ticks_perdidos']} for e in self.ejes},
            'deriva_vs_gemelo': gemelo.deriva(meas, twin, refs, dt=self.step_ns * 1e-9),
        }
        self.get_logger().info('RESULTADO HIL: ' + json.dumps(res, default=float))
        if self.salida:
            with open(self.salida + '.json', 'w') as f:
                json.dump(res, f, indent=1, default=float)
            with open(self.salida + '.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t_s'] + [f'ref_{e}' for e in self.ejes] + [f'hil_{e}' for e in self.ejes]
                           + [f'gemelo_{e}' for e in self.ejes] + ['rtt_ms'])
                for i, fl in enumerate(filas):
                    w.writerow([f'{fl["t_s"]:.3f}'] + [f'{x:.7f}' for x in fl['ref']]
                               + [f'{x:.7f}' for x in fl['meas']] + [f'{x:.7f}' for x in twin[i]]
                               + [f'{fl["rtt_ms"]:.3f}' if fl['rtt_ms'] is not None else ''])


def main():
    correr(McuLockstep)
