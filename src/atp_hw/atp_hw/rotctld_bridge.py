"""Puente ROS 2 <-> rotor real (o simulado) via Hamlib `rotctld`.

  /atp/reference (JointState rad, az continuo) --> `P az el` al rotor
  `p` al rotor --------------------------------> /joint_states (JointState rad)

Asi el mismo `error_monitor` y `link_budget` miden el rotor REAL. El lazo de
posicion lo cierra el controlador del rotor; aqui no hay PID ni mount_sim.

Dos detalles que el lazo simulado no tenia:
  * /joint_states usa el marco CONTINUO de la referencia (el acimut del rotor
    se reubica +-360 para quedar junto a la referencia), de modo que
    ref - meas es el error fisico y no un +-360 espurio.
  * reference_generator congela el pase en AOS hasta ver /atp/torque. Sin PID,
    este nodo la publica (valor 0) cuando el rotor llego al punto de AOS
    (preposicion), y no publica /joint_states antes de eso para que el
    error_monitor no cuente el viaje del rotor desde su posicion de reposo.
"""
import math
import signal
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from atp_ros.params import declare_num

from atp_hw import rotor_logic as rl
from atp_hw.bridge_core import BridgeConfig, BridgeCore


class RotctldBridge(Node):
    def __init__(self):
        super().__init__('rotctld_bridge')
        d = BridgeConfig()
        self.declare_parameter('host', d.host)
        self.declare_parameter('port', d.port)
        self.declare_parameter('az_prefer', d.az_prefer)
        for k in ('preposition', 'allow_flip', 'query_limits', 'stop_on_exit'):
            self.declare_parameter(k, getattr(d, k))
        for k in ('timeout_s', 'az_min', 'az_max', 'el_min', 'el_max',
                  'az_res_deg', 'el_res_deg', 'cmd_rate_hz', 'read_rate_hz',
                  'backoff_min_s', 'backoff_max_s', 'ref_timeout_s',
                  'ready_tol_deg', 'ready_hold_s', 'preposition_timeout_s'):
            declare_num(self, k, getattr(d, k))
        self.declare_parameter('hold_joint_states_until_ready', True)
        self.declare_parameter('publish_ready_signal', True)
        self.declare_parameter('require_realtime', True)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        kw = {}
        for k, v in vars(d).items():
            kw[k] = type(v)(g(k)) if not isinstance(v, bool) else bool(g(k))
        cfg = BridgeConfig(**kw)
        if cfg.az_max - cfg.az_min < 360.0:
            self.get_logger().warn(
                f'rango de acimut {cfg.az_min:g}..{cfg.az_max:g} < 360: hay zona '
                'que el rotor no cubre')
        self.hold = bool(g('hold_joint_states_until_ready'))
        self.signal = bool(g('publish_ready_signal'))
        self.require_rt = bool(g('require_realtime'))

        self.core = BridgeCore(cfg, self._log)
        self.ref_az_c = None          # ultimo acimut continuo de la referencia
        self.ref_el = None
        self.t_track0 = None
        self.err = {'n': 0, 'max': 0.0, 'gt1': 0, 'first_gt1': None}
        self.last_ready_pub = 0.0
        self.clock_chk = None
        self.clock_bad = False

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.raw_pub = self.create_publisher(JointState, '/atp/rotor/raw', 10)
        # /atp/torque es JointState sellado desde el lockstep (atp_ros N5)
        self.torque_pub = self.create_publisher(JointState, '/atp/torque', 10)
        self.create_subscription(JointState, '/atp/reference', self.on_ref, 10)
        self.create_subscription(Bool, '/atp/rotor/halt', self.on_halt, 10)
        # temporizador de PARED: el rotor vive en tiempo real aunque el reloj
        # de la simulacion se pare o se acelere
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.tick, clock=wall)
        self.create_timer(5.0, self.check_clock, clock=wall)
        self.core.start()
        self.get_logger().info(
            f'puente a rotctld {cfg.host}:{cfg.port} | az [{cfg.az_min:g},{cfg.az_max:g}] '
            f'el [{cfg.el_min:g},{cfg.el_max:g}] | banda muerta '
            f'{cfg.az_res_deg:g}/{cfg.el_res_deg:g} deg | P {cfg.cmd_rate_hz:g} Hz, '
            f'p {cfg.read_rate_hz:g} Hz')

    def _log(self, lvl, msg):
        if not rclpy.ok():           # contexto ya cerrado por SIGINT: rosout no existe
            print(f'[{lvl.upper()}] [rotctld_bridge]: {msg}', file=sys.stderr,
                  flush=True)
            return
        # una linea por severidad: rclpy prohibe cambiar la severidad en un
        # mismo punto de llamada (ValueError) y mataria el hilo de E/S
        lg = self.get_logger()
        if lvl == 'warn':
            lg.warn(msg)
        elif lvl == 'error':
            lg.error(msg)
        else:
            lg.info(msg)

    # ---- entradas ----------------------------------------------------------
    def on_ref(self, msg):
        az, el = math.degrees(msg.position[0]), math.degrees(msg.position[1])
        self.ref_az_c, self.ref_el = az, el
        self.core.set_reference(az, el)

    def on_halt(self, msg):
        self.core.set_halt(msg.data)

    # ---- salidas -----------------------------------------------------------
    def tick(self):
        readings = self.core.pop_readings()
        tracking = self.core.state == 'TRACKING'
        if tracking and self.t_track0 is None:
            self.t_track0 = time.monotonic()
        for t_mono, az, el in readings:
            now = self.get_clock().now().to_msg()
            raw = JointState()
            raw.header.stamp = now
            raw.name = ['az', 'el']
            raw.position = [math.radians(az), math.radians(el)]
            self.raw_pub.publish(raw)
            if self.hold and not tracking:
                continue
            ref_az = self.ref_az_c if self.ref_az_c is not None else az
            az_c = rl.meas_in_ref_frame(az, ref_az)
            msg = JointState()
            msg.header.stamp = now
            msg.name = ['az', 'el']
            msg.position = [math.radians(az_c), math.radians(el)]
            self.pub.publish(msg)
            if tracking and self.ref_el is not None:
                self.account_error(t_mono, az_c, el)
        if tracking and self.signal and time.monotonic() - self.last_ready_pub >= 1.0:
            # "lazo cerrado": reference_generator arranca el pase al verla
            self.last_ready_pub = time.monotonic()
            arranque = JointState()
            arranque.header.stamp = self.get_clock().now().to_msg()
            arranque.name = ['az', 'el']
            arranque.effort = [0.0, 0.0]
            self.torque_pub.publish(arranque)

    def account_error(self, t_mono, az_c, el):
        e = math.hypot((self.ref_az_c - az_c) * math.cos(math.radians(self.ref_el)),
                       self.ref_el - el)
        s = self.err
        s['n'] += 1
        s['max'] = max(s['max'], e)
        if e > 1.0:
            s['gt1'] += 1
            if s['first_gt1'] is None:
                s['first_gt1'] = t_mono - self.t_track0

    def check_clock(self):
        """El rotor real no se acelera: con time_scale != 1 el pase correria
        mas rapido que el rotor y la medicion no significaria nada. Se mide el
        cociente ACUMULADO sim/pared (no por ventana: bajo carga de CPU el
        /clock llega a rafagas y una ventana de 5 s oscila 0.75..1.2)."""
        sim = self.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        if sim <= 0.0:
            return
        if self.clock_chk is None:
            self.clock_chk = (sim, wall)
            return
        ds, dw = sim - self.clock_chk[0], wall - self.clock_chk[1]
        if dw >= 15.0 and abs(ds / dw - 1.0) > 0.15 and not self.clock_bad:
            self.clock_bad = True
            self._log('error',
                      f'EL RELOJ NO ES TIEMPO REAL (x{ds / dw:.2f}): con hardware '
                      f'en el lazo time_scale debe ser 1. Resultados NO validos.')
            if self.require_rt:
                self.core.set_halt(True)

    def report(self):
        s, rate = self.err, float(self.core.cfg.read_rate_hz)
        self._log('info', f'PUENTE {self.core.summary()}')
        if s['n']:
            first = ('nunca' if s['first_gt1'] is None
                     else f'a los {s["first_gt1"]:.1f} s de TRACKING')
            self._log('info',
                f'ROTOR error de apuntamiento (lecturas de rotctld, {s["n"]} '
                f'muestras): max={s["max"]:.2f} deg | >1 deg durante '
                f'~{s["gt1"] / rate:.1f} s | primera vez >1 deg: {first}')


def main():
    rclpy.init()
    node = RotctldBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():               # un fallo real; si no, es la carrera del SIGINT
            raise
    finally:
        # un segundo SIGINT (launch + grupo de procesos) no debe interrumpir el
        # cierre: es lo que envia `S` al rotor y escribe el resumen
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.core.stop()
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
