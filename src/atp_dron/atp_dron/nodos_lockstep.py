"""Nodos `dron_dinamica` y `dron_navegador` en LOCKSTEP (CONTRATO sec. 8 y 21; D5). Solo tuberia DDS: la logica esta en
`atp_dron.lockstep` (Python puro, probada sin ROS). Cadena de un tick k (t_k = k * 0.02 s; el /clock lo da `sim_clock` con
step_s = 0.02):

    /clock -> dron_dinamica publica /dron/estado (e_k, sello k) y /dron/pose (PoseStamped, sello t_k)
    dron_navegador -> /dron/setpoint {k, t, v, yaw, guia}   (UN mensaje: no hay nada que emparejar entre topicos)
    dron_dinamica: con el setpoint de k integra [t_k, t_{k+1}) con el y confirma en /atp/step_ack el sello del /clock.

Reglas de CONTRATO sec. 8: se sella con el instante del calculo; se empareja por sello (un setpoint rancio se descarta y nadie
responde dos veces al mismo k); el arranque espera a los suscriptores esperados (`wait_subscribers`) y la planta repite el estado
inicial con cada /clock hasta recibir el primer setpoint; los nodos no usan el reloj de pared salvo para reintentar (STEADY).

Contrato de `/dron/setpoint` (para sustituir al navegador por otro nodo): String JSON
`{"k": int, "t": float, "v": [vx, vy, vz] (m/s, ENU), "yaw": rad, "guia": {"terminado": bool, "dest": [x,y,z]|null,
"seg0": [x,y,z], "i": int, "meta": int, "n": int, "t_llegada": float|null}}`; `guia` solo lo usan las metricas.
"""
import json

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock as RelojRclpy, ClockType
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from atp_dron import lockstep as L


def _ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _sello(ns):
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _reintento(nodo, f, s=0.02):
    """Temporizador de PARED (el reloj simulado esta parado mientras se espera): reintenta `f` hasta que haya trabajo."""
    return nodo.create_timer(s, f, clock=RelojRclpy(clock_type=ClockType.STEADY_TIME))


def _pose(pl, ns):
    q, p = pl.d.q, pl.d.p                              # q = (w, x, y, z) cuerpo -> mundo; ROS usa (x, y, z, w)
    m = PoseStamped()
    m.header.stamp = _sello(ns)
    m.header.frame_id = 'world'
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p[0]), float(p[1]), float(p[2])
    m.pose.orientation.x, m.pose.orientation.y = float(q[1]), float(q[2])
    m.pose.orientation.z, m.pose.orientation.w = float(q[3]), float(q[0])
    return m


class PlantaROS:
    def __init__(self, nodo, cfg, salida, id_, wait):
        self.nodo, self.wait = nodo, int(wait)
        self.pl = L.PlantaLock(L.cargar_cfg(None, cfg))
        self.carpeta = f'{salida}/{id_}'
        self.step_ns = int(round(L.PASO_S * 1e9))
        self.epoca = self.step_ns        # el primer /clock que el lazo atiende es k = 1 (la compuerta de sim_clock lo fija)
        self.empezo = self.pend = False
        self.s_act = self.sig = None
        self.pub = nodo.create_publisher(String, '/dron/estado', 50)
        self.pub_pose = nodo.create_publisher(PoseStamped, '/dron/pose', 50)
        self.ack = nodo.create_publisher(Clock, '/atp/step_ack', 10)
        nodo.create_subscription(String, '/dron/setpoint', lambda m: self._sp(m), 50)
        nodo.create_subscription(Clock, '/clock', lambda m: self.on_clock(m), 10)
        c = self.pl.c
        nodo.get_logger().info(f"dron_dinamica LOCKSTEP: {c['cuerpo']}, {len(c['ruta'])} metas, sensores {c['sensores']}, "
                               f"{self.pl.n} pasos de {L.DT:g} s ({L.PASO_S:g} s por tick)")

    def on_clock(self, msg):
        s = _ns(msg.clock)
        if self.pl.fin:
            return
        if not self.empezo:
            # arranque: el instante inicial es el /clock de k = 1; se repite el estado con cada /clock hasta el primer setpoint
            if s != self.epoca or self.pub.get_subscription_count() < self.wait:
                return
        elif s != self.sig or self.pend:
            return
        self.s_act, self.pend = s, True
        self.pub.publish(String(data=self.pl.estado_txt()))
        self.pub_pose.publish(_pose(self.pl, s))

    def _sp(self, m):
        sp = json.loads(m.data)
        if not self.pend or sp['k'] != self.pl.k:
            return                                           # rancio: de otro instante
        self.pl.aplicar(sp)
        self.empezo, self.pend = True, False
        self.sig = self.s_act + self.step_ns
        self.ack.publish(Clock(clock=_sello(self.s_act)))    # confirma el instante PARA EL QUE se calculo
        if self.pl.fin:
            self.terminar()

    def terminar(self):
        met = self.pl.cerrar(self.carpeta)
        self.nodo.get_logger().info('RESUMEN ' + json.dumps(met))
        raise SystemExit(0)


class GuiadoROS:
    """Computadora de mision: /dron/estado -> /dron/setpoint (un setpoint por sello k)."""

    def __init__(self, nodo, cfg, wait):
        self.nodo, self.wait = nodo, int(wait)
        self.gu = L.Guiado(L.cargar_cfg(None, cfg))
        self.pub = nodo.create_publisher(String, '/dron/setpoint', 50)
        self.est = []
        nodo.create_subscription(String, '/dron/estado', lambda m: self._estado(m), 50)
        _reintento(nodo, self._intentar)
        nodo.get_logger().info(f"dron_navegador LOCKSTEP: {len(self.gu.ruta)} metas, v_max {self.gu.c['v_max']:g} m/s")

    def _estado(self, m):
        self.est.append(m.data)
        self._intentar()

    def _intentar(self):
        if self.pub.get_subscription_count() < self.wait:
            return                  # la planta repite la observacion inicial hasta recibir el primer setpoint
        while self.est:
            sp = self.gu.decidir(json.loads(self.est.pop(0)))
            if sp is not None:      # None = duplicado: nadie responde dos veces al mismo sello
                self.pub.publish(String(data=json.dumps(sp, separators=(',', ':'))))
