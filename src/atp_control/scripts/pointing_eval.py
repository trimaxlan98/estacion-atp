#!/usr/bin/env python3
"""Evaluador del error de apuntamiento que NO exige sellos identicos.

error_monitor empareja medida y referencia por sello EXACTO (lockstep, CONTRATO 8).
Con un reloj que no es nuestro (Gazebo: /clock por puente, sellos de 10 ms del
controller_manager contra sellos de la referencia que dependen de cuando el
generador vio el reloj) solo coinciden algunos instantes. Este nodo mide en TODOS
los instantes de /joint_states: interpola linealmente la referencia (20 Hz,
suave: el error de la interpolacion es a Ts^2/8 ~ 1e-5 grados a 9 deg/s) en el
sello de cada medida y calcula el mismo error que error_monitor,
sqrt((e_az cos el)^2 + e_el^2), solo entre AOS y LOS (t_since_aos de /atp/sat_state).

Con sellos identicos (lockstep) da las mismas cifras que error_monitor: es la
comprobacion cruzada del evaluador. Parametros: out_file (JSON opcional).
"""
import json
import math

import numpy as np
import rclpy
from atp_interfaces.msg import SatState
from atp_ros.nodo import anunciar_listo
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

OBJETIVO_DEG = 0.1


def _t(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class PointingEval(Node):
    def __init__(self):
        super().__init__('pointing_eval')
        self.declare_parameter('out_file', '')
        q = 5000
        self.js, self.ref, self.sat = [], [], []
        self.create_subscription(JointState, '/joint_states', self.on_js, q)
        self.create_subscription(JointState, '/atp/reference', self.on_ref, q)
        self.create_subscription(SatState, '/atp/sat_state', self.on_sat, q)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/atp/pass_done', self.on_done, latched)
        self.cierre = None
        self.fin = False
        self.parcial = False    # True si se evalua porque el nodo se apago antes de acabar el pase

    def on_js(self, m):
        if len(m.position) >= 2:
            self.js.append((_t(m), m.position[0], m.position[1]))

    def on_ref(self, m):
        self.ref.append((_t(m), m.position[0], m.position[1]))

    def on_sat(self, m):
        self.sat.append((_t(m), m.t_since_aos_s, m.pass_duration_s))

    def on_done(self, m):
        if m.data and self.cierre is None:
            self.cierre = self.create_timer(1.0, self.evaluar)   # 1 s simulado: ultimas medidas

    def evaluar(self):
        if self.cierre is not None:
            self.cierre.cancel()
        if self.fin:
            return
        self.fin = True
        js, ref, sat = (np.array(sorted(set(x))) for x in (self.js, self.ref, self.sat))
        if len(js) == 0 or len(ref) < 3 or len(sat) < 3:
            self.get_logger().error('EVAL sin datos')
            return
        t = js[:, 0]
        dentro = (t >= ref[0, 0]) & (t <= ref[-1, 0])
        t_since = np.interp(t, sat[:, 0], sat[:, 1])
        dur = float(sat[-1, 2])
        # un instante cuenta si el satelite esta en pase en ese sello (como error_monitor)
        dentro &= (t_since > 0.0) & (t_since < dur)
        r_az = np.interp(t, ref[:, 0], ref[:, 1])
        r_el = np.interp(t, ref[:, 0], ref[:, 2])
        e_az = np.degrees(r_az - js[:, 1])[dentro]
        e_el = np.degrees(r_el - js[:, 2])[dentro]
        e_pt = np.hypot(e_az * np.cos(r_el[dentro]), e_el)
        if len(e_pt) == 0:
            self.get_logger().error('EVAL: ninguna medida dentro del pase')
            return
        res = {
            'n': int(len(e_pt)),
            'az_rms': float(np.sqrt(np.mean(e_az ** 2))), 'az_max': float(np.max(np.abs(e_az))),
            'el_rms': float(np.sqrt(np.mean(e_el ** 2))), 'el_max': float(np.max(np.abs(e_el))),
            'rms': float(np.sqrt(np.mean(e_pt ** 2))), 'p95': float(np.percentile(e_pt, 95)),
            'max': float(np.max(e_pt)),
            'dentro_pct': float(np.mean(e_pt <= OBJETIVO_DEG) * 100.0),
            'parcial': self.parcial,
        }
        dt = np.diff(t)
        res['periodo_medio_ms'] = float(np.mean(dt) * 1e3) if len(dt) else 0.0
        self.get_logger().info(
            f"EVAL{' (PARCIAL: el pase no termino)' if self.parcial else ''} n={res['n']} | az: rms={res['az_rms']:.4f} max={res['az_max']:.4f} | el: "
            f"rms={res['el_rms']:.4f} max={res['el_max']:.4f} | apuntamiento: rms={res['rms']:.4f} "
            f"p95={res['p95']:.4f} max={res['max']:.4f} deg | dentro de {OBJETIVO_DEG} deg: "
            f"{res['dentro_pct']:.1f}% | muestras a {res['periodo_medio_ms']:.1f} ms")
        out = self.get_parameter('out_file').value
        if out:
            with open(out, 'w') as f:
                json.dump(res, f, indent=1)
        raise SystemExit(0)


def main():
    rclpy.init()
    n = PointingEval()
    anunciar_listo(n)      # saludo de arranque (CONTRATO 8.4)
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        # apagado antes de que acabe el pase (p. ej. el vigilante del host): deja lo medido
        n.parcial = True
        try:
            n.evaluar()
        except SystemExit:
            pass
    finally:
        try:
            n.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:    # carrera conocida de rclpy al cerrar
            pass


if __name__ == '__main__':
    main()
