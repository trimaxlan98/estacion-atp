"""Autotrack por conical scan: corrige el program track con la senal RF.

Entre /atp/program (efemeride) y /atp/reference (lo que sigue el PID).
Cada vuelta del scan estima el desapuntamiento con el discriminador
(`model.discriminador_conscan`) y lo integra en una correccion. Como el
discriminador mide blanco-menos-montura, la correccion absorbe a la vez el
error de efemeride, el sesgo mecanico y el retraso del servo.

En lockstep (CONTRATO 8) la referencia del instante t_k solo sale cuando ya
se proceso la muestra del beacon de t_{k-1}: la correccion entra siempre en
el mismo paso y la corrida es repetible.

Si la potencia media cae por debajo de `lock_db` (blanco fuera del haz), no
corrige: congela la correccion y lo declara (sin enganche no hay medida).
`enabled:=false` lo convierte en un paso directo (program track puro).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _sello(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

from atp_ros import model
from atp_ros.nodo import correr
from atp_ros.params import declare_num


class Autotrack(Node):
    def __init__(self):
        super().__init__('autotrack')
        self.declare_parameter('enabled', True)
        self.declare_parameter('band', 'Ka')
        declare_num(self, 'dish_m', model.DIAMETRO_PLATO_M)
        declare_num(self, 'scan_hz', 2.5)        # 8 muestras por vuelta a 20 Hz
        declare_num(self, 'scan_frac', 0.2)
        declare_num(self, 'beacon_hz', 20.0)
        declare_num(self, 'gain', 0.25)              # fraccion corregida por vuelta
        declare_num(self, 'max_corr_deg', 1.0)
        declare_num(self, 'lock_db', -10.0)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.enabled = bool(g('enabled'))
        th3 = model.ancho_haz(float(g('dish_m')), model.BANDAS[str(g('band'))])
        self.th3, self.r = th3, float(g('scan_frac')) * th3
        # una vuelta EXACTA del scan: el discriminador solo es exacto sobre
        # un numero entero de vueltas
        self.n_vuelta = max(4, int(round(float(g('beacon_hz')) / float(g('scan_hz')))))
        self.gain = float(g('gain'))
        self.max_c = float(g('max_corr_deg'))
        self.lock_db = float(g('lock_db'))
        self.c = np.zeros(2)            # correccion (xel, el) grados
        self.buf = []                   # (t, P, fase)
        self.locked = False
        self.prog_pend = []             # programas esperando su muestra previa
        self.prog_prev = None           # sello del programa anterior
        self.beacon_ultimo = None       # sello de la ultima muestra procesada
        self.c_hist = {}                # sello de muestra -> correccion tras ella
        self.create_subscription(JointState, '/atp/program', self.on_program, 200)
        self.create_subscription(JointState, '/atp/beacon', self.on_beacon, 200)
        self.pub = self.create_publisher(JointState, '/atp/reference', 10)
        self.state_pub = self.create_publisher(
            Float64MultiArray, '/atp/autotrack_state', 10)
        self.get_logger().info('autotrack ' + ('ACTIVO' if self.enabled
                                               else 'apagado: program track puro'))

    def on_beacon(self, msg):
        self.beacon_ultimo = _sello(msg)
        self._integrar(msg)
        self.c_hist[self.beacon_ultimo] = self.c.copy()
        if len(self.c_hist) > 400:
            self.c_hist.pop(min(self.c_hist))
        self._despachar()

    def _integrar(self, msg):
        if not self.enabled:
            return
        self.buf.append((_sello(msg) * 1e-9, msg.position[0], msg.position[1]))
        if len(self.buf) < self.n_vuelta:
            return
        b = np.array(self.buf)
        self.buf = []
        p_media = float(b[:, 1].mean())
        locked = p_media > self.lock_db
        if locked != self.locked:
            self.locked = locked
            self.get_logger().info('ENGANCHE' if locked else 'perdida de enganche')
        ex, ey = model.discriminador_conscan(b[:, 1], b[:, 2], self.r, self.th3)
        if locked:
            self.c = np.clip(self.c + self.gain * np.array([ex, ey]),
                             -self.max_c, self.max_c)
        self.state_pub.publish(Float64MultiArray(
            data=[ex, ey, float(self.c[0]), float(self.c[1]), p_media,
                  1.0 if locked else 0.0]))

    def on_program(self, msg):
        s = _sello(msg)
        # se necesita la muestra del instante ANTERIOR (si ya hay beacon)
        self.prog_pend.append((s, self.prog_prev, msg))
        self.prog_prev = s
        self._despachar()

    def _despachar(self):
        while self.prog_pend:
            s, previo, msg = self.prog_pend[0]
            if (self.beacon_ultimo is not None and previo is not None
                    and self.beacon_ultimo < previo):
                return            # falta la muestra de t_{k-1}: esperar
            self.prog_pend.pop(0)
            # la correccion EXACTAMENTE como quedo tras la muestra de t_{k-1},
            # aunque la de t_k ya haya llegado (llega por otro camino)
            c = self.c_hist.get(previo, self.c) if previo is not None else self.c
            self._publicar(msg, c)

    def _publicar(self, msg, c):
        az, el = msg.position[:2]
        cos_el = max(math.cos(el), 1e-3)
        out = JointState()
        out.header = msg.header
        out.name = ['az', 'el']
        out.position = [az + math.radians(c[0]) / cos_el,
                        el + math.radians(c[1])]
        self.pub.publish(out)


def main():
    correr(Autotrack)
