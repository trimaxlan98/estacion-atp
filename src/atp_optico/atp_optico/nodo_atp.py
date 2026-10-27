"""Nodo `optico_atp`: el ATP optico de la estacion en LOCKSTEP (CONTRATO 17, que cumple el 8).

Es un nodo PASIVO (regla 3): no frena el reloj. Se cuelga del lazo de la montura que YA existe
(`reference_generator` + `pid_controller` + `mount_sim` con `publish_truth`, el MISMO de `atp_loop`) y
usa lo que este publica:

    /atp/pointing_error  (PointingError: error de la montura contra la VERDAD, sellado, solo AOS-LOS)
    /atp/sat_state       (SatState: az/el continuos, rango, sellado)

Empareja por SELLO (nunca "el ultimo que llego"), alimenta con cada par al motor `atp.AtpOptico`
(el MISMO de la corrida sin ROS: los 10 kHz del FSM, el tilt atmosferico con semilla, el ruido del cuadrante y el
enlace de bajada) y, 1 s simulado despues de `/atp/pass_done`, imprime el RESUMEN y escribe
`optico.json` y `optico_serie.jsonl` (byte a byte iguales entre corridas: la semilla, el orden de los sorteos y
el sello fijan todo). Pide `error_monitor` con `use_truth:=true` (es quien publica /atp/pointing_error).
"""
import math
import queue
import threading

import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Bool

from atp_interfaces.msg import PointingError, SatState
from atp_optico import atp as P
from atp_optico import escenario as E
from atp_ros.nodo import correr


def _sello(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def declarar_config(node):
    valores = {}
    for nombre, tipo, defecto in E.config_campos():
        if tipo is str:
            node.declare_parameter(nombre, defecto)
        else:
            node.declare_parameter(nombre, defecto, ParameterDescriptor(dynamic_typing=True))
        v = node.get_parameter(nombre).value
        valores[nombre] = tipo(v) if tipo in (int, float) else v
    return E.ConfigOptico(**valores)


class OpticoAtp(Node):
    def __init__(self):
        super().__init__('optico_atp')
        self.cfg = declarar_config(self)
        self.declare_parameter('exit_on_done', True)
        self.exit_on_done = bool(self.get_parameter('exit_on_done').value)
        sat, est, atm, catp = E.construir(self.cfg)
        self.est = est
        self.motor = P.AtpOptico(catp, sat, est, atm)
        self.med = P.Medidor(catp, est)
        self.filas = []
        self.pe, self.ss = {}, {}
        self.cola = queue.Queue()      # pares emparejados por sello, en orden, esperando al motor
        self.error = None
        self.t_ultimo = -1.0
        self.dur_pase = None
        self.reintentos = 0
        self.cierre = None
        self.finished = False
        # pasivo: el reloj no lo espera, que no pierda nada. Colas MUY profundas: a maxima velocidad el lockstep publica un
        # paso cada ~3 ms y el pase mas la espera de arranque y el rabo tras LOS pasan de 2000 mensajes si el nodo va retrasado.
        q = 50000
        qos_pe = QoSProfile(depth=q, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointingError, '/atp/pointing_error', self.on_pe, qos_pe)
        self.create_subscription(SatState, '/atp/sat_state', self.on_sat, q)
        # El motor tarda mas por paso (~10 ms de pared) que el lockstep en generarlos: las llamadas solo EMPAREJAN y
        # encolan (el ejecutor las atiende de inmediato: nunca pierde un mensaje) y un HILO consume la cola en orden.
        self.hilo = threading.Thread(target=self.trabajar, daemon=True)
        self.hilo.start()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/atp/pass_done', self.on_done, latched)
        self.get_logger().info(
            f'ATP optico: telescopio {est.D_m:.2f} m, receptor {est.receptor}, campo {est.campo_urad:.1f} urad, '
            f'modem {est.modem.nombre}, FSM {catp.lazo.f_cruce_hz:.0f} Hz a {catp.lazo.fs_hz / 1e3:.0f} kHz, semilla {catp.semilla}')

    def intentar(self, s):
        if s in self.pe and s in self.ss:
            pe, ss = self.pe.pop(s), self.ss.pop(s)
            if ss.t_since_aos_s <= self.t_ultimo:      # estado repetido (arranque): ya contado
                return
            self.t_ultimo = ss.t_since_aos_s
            self.cola.put((pe, ss))

    def trabajar(self):
        while True:
            item = self.cola.get()
            if item is None:
                return
            try:
                pe, ss = item
                el = ss.el_deg
                err = np.array([1e6 * math.radians(pe.e_az_deg) * math.cos(math.radians(el)),
                                1e6 * math.radians(pe.e_el_deg)])
                r = self.motor.paso(ss.t_since_aos_s, ss.az_deg, el, ss.range_km, err)
                self.med.muestra(r)
                self.filas.append(r)
            except Exception as e:           # no tumbar el hilo en silencio: se avisa en el resumen
                self.error = e
                return

    def on_pe(self, msg):
        s = _sello(msg)
        self.pe[s] = msg
        self.intentar(s)

    def on_sat(self, msg):
        s = _sello(msg)
        self.ss[s] = msg
        self.dur_pase = msg.pass_duration_s
        if len(self.ss) > 4000:
            self.ss.pop(min(self.ss))
        self.intentar(s)

    def on_done(self, msg):
        if not msg.data or self.cierre is not None or self.finished:
            return
        # el aviso de LOS puede adelantarse a los ultimos instantes (otro topico): 1 s simulado de margen
        self.cierre = self.create_timer(1.0, self.resumen)

    def resumen(self):
        if self.finished:
            return
        # el aviso de LOS y los ultimos pasos viajan por topicos distintos: no se cierra hasta haber emparejado el ultimo
        # paso del pase (el generador publica pointing_error mientras t < duracion; ~0.05 s antes de LOS)
        if self.dur_pase and self.t_ultimo < self.dur_pase - 0.0501 and self.reintentos < 30:
            self.reintentos += 1
            return                                      # el temporizador es periodico: se reintenta 1 s simulado despues
        self.cierre.cancel()
        self.finished = True
        self.cola.put(None)                             # vacia lo que falte y para el hilo
        self.hilo.join()
        if self.error is not None:
            self.get_logger().error(f'el motor fallo: {self.error!r}')
        if not self.filas:
            self.get_logger().error('sin muestras')
            return
        for linea in self.med.resumen():
            self.get_logger().info(linea)
        if self.cfg.salida:
            E.escribir_salida(self.cfg.salida, self.med, self.filas)
        if self.exit_on_done:
            raise SystemExit(0)


def main():
    correr(OpticoAtp)
