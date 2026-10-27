"""Modo `lockstep` de los nodos `rover_dinamica`, `rover_navegador` y `rover_enlace` (CONTRATO sec. 8 y 18; R9).

La logica esta en `atp_rover.lockstep` (Python puro, probada sin ROS); aqui solo hay tuberia DDS. Cadena de un tick k
(t_k = k * 0.1 s de escena; el /clock lo da `sim_clock` con step_s = 0.1):

    /clock -> rover_dinamica publica /rover/estado (e_k, sello t_k)
    sin enlace: rover_navegador -> /rover/cmd_vel (Twist) + ficha /rover/cmd_vel_listo {"t": t_k, "n": 1, "info": {...}}
    con enlace: rover_enlace recibe e_k -> entrega las ordenes vencidas en /rover/cmd_vel + ficha /rover/cmd_vel_listo,
                y los estados vencidos en /rover/estado_tierra + ficha /rover/estado_tierra_listo;
                el operador (rover_navegador con `fuente` en Tierra, o un guion) responde a esa ficha con
                /rover/cmd_vel_op + ficha /rover/cmd_vel_op_listo (la usa el enlace en el tick siguiente)
    rover_dinamica: con la ficha de t_k integra [t_k, t_{k+1}) con el mando y confirma en /atp/step_ack el sello del /clock.

Reglas de CONTRATO sec. 8: se sella con el instante del calculo (el `t` del estado viaja en la ficha); se empareja por sello
(fichas rancias se descartan; nadie responde dos veces al mismo sello); el arranque espera a los suscriptores esperados
(`wait_subscribers`) y la planta repite el estado inicial con cada /clock hasta recibir la primera ficha; los nodos de la
cadena no usan el reloj de pared salvo para reintentar la apertura (temporizador STEADY).
"""
import json

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.clock import Clock as RelojRclpy, ClockType
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from atp_rover import lockstep as L


def _ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _sello(ns):
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _twist(v, w):
    m = Twist()
    m.linear.x, m.angular.z = float(v), float(w)
    return m


def _ficha(t, n, info=None):
    d = {'t': t, 'n': n}
    if info:
        d['info'] = info
    return String(data=json.dumps(d, separators=(',', ':')))


def _reintento(nodo, f, s=0.02):
    """Temporizador de PARED (el reloj simulado esta parado mientras se espera): reintenta `f` hasta que haya trabajo."""
    return nodo.create_timer(s, f, clock=RelojRclpy(clock_type=ClockType.STEADY_TIME))


class PlantaROS:
    def __init__(self, nodo, cfg, salida, id_, wait):
        self.nodo, self.wait = nodo, int(wait)
        self.pl = L.crear_planta(cfg)
        self.carpeta = f'{salida}/{id_}'
        self.step_ns = int(round(L.PASO_S * 1e9))
        self.epoca = self.step_ns        # el primer /clock que el lazo atiende es k = 1 (la compuerta de sim_clock lo fija)
        self.empezo = self.pend = False
        self.s_act = self.sig = None
        self.buzon = L.Buzon()
        self.pub = nodo.create_publisher(String, '/rover/estado', 50)
        self.ack = nodo.create_publisher(Clock, '/atp/step_ack', 10)
        nodo.create_subscription(Twist, '/rover/cmd_vel', lambda m: self._cmd(m), 50)
        nodo.create_subscription(String, '/rover/cmd_vel_listo', lambda m: self._ficha(m), 50)
        nodo.create_subscription(Clock, '/clock', lambda m: self.on_clock(m), 10)
        nodo.get_logger().info(f"rover_dinamica LOCKSTEP: {cfg['planeta']} / {cfg['terreno']} semilla {cfg['semilla']}, "
                               f"{cfg['navegacion']}, enlace {cfg['enlace']}, {self.pl.n} pasos de {L.DT:g} s")

    def on_clock(self, msg):
        s = _ns(msg.clock)
        if self.pl.fin:
            return
        if not self.empezo:
            # arranque: el instante inicial es el /clock de k = 1; se repite el estado con cada /clock hasta la primera ficha
            if s != self.epoca or self.pub.get_subscription_count() < self.wait:
                return
        elif s != self.sig or self.pend:
            return
        self.s_act, self.pend = s, True
        self.pub.publish(String(data=self.pl.estado_txt()))

    def _cmd(self, m):
        self.buzon.meter((m.linear.x, m.angular.z))
        self._intentar()

    def _ficha(self, m):
        self.buzon.ficha(json.loads(m.data))
        self._intentar()

    def _intentar(self):
        while True:
            r = self.buzon.sacar()
            if r is None:
                return
            f, cmds = r
            if not self.pend or f['t'] != self.pl.e['t']:
                continue                                     # rancia: de otro instante
            self.pl.aplicar(cmds, f.get('info'))
            self.empezo, self.pend = True, False
            self.sig = self.s_act + self.step_ns
            self.ack.publish(Clock(clock=_sello(self.s_act)))   # confirma el instante PARA EL QUE se calculo
            if self.pl.fin:
                self.terminar()
            return

    def terminar(self):
        met = self.pl.cerrar(self.carpeta)
        self.nodo.get_logger().info('RESUMEN ' + json.dumps(met))
        raise SystemExit(0)


class NavegadorROS:
    """Sin enlace: controlador a bordo (estado -> mando). Con enlace: operador en Tierra (estado_tierra -> cmd_vel_op)."""

    def __init__(self, nodo, cfg, wait):
        self.nodo, self.wait = nodo, int(wait)
        self.tierra = cfg['enlace'] != 'ninguno'
        ent, sal = ('/rover/estado_tierra', '/rover/cmd_vel_op') if self.tierra else ('/rover/estado', '/rover/cmd_vel')
        self.pub = nodo.create_publisher(Twist, sal, 50)
        self.pub_f = nodo.create_publisher(String, sal + '_listo', 50)
        self.buzon = L.Buzon()
        self.est = []
        self.ultimo = None
        if self.tierra:
            self.op = L.crear_operador(cfg)
            nodo.create_subscription(String, ent + '_listo', lambda m: self._ficha(m), 50)
        else:
            self.nav = L.Navegacion(cfg)
        nodo.create_subscription(String, ent, lambda m: self._estado(m), 50)
        self.timer = _reintento(nodo, self._intentar)
        nodo.get_logger().info(f"rover_navegador LOCKSTEP: {'operador en Tierra (' + cfg['operador'] + ')' if self.tierra else 'a bordo'}, "
                               f"{cfg['navegacion']}, {sal}")

    def _listo(self):
        return self.pub.get_subscription_count() >= self.wait and self.pub_f.get_subscription_count() >= self.wait

    def _estado(self, m):
        if self.tierra:
            self.buzon.meter(m.data)
        else:
            self.est.append(m.data)
        self._intentar()

    def _ficha(self, m):
        self.buzon.ficha(json.loads(m.data))
        self._intentar()

    def _intentar(self):
        if not self._listo():
            return                  # la planta/enlace repite la observacion inicial hasta recibir la primera respuesta
        if not self.tierra:
            while self.est:
                d = json.loads(self.est.pop(0))
                if self.ultimo is not None and d['t'] <= self.ultimo:
                    continue        # duplicado: nadie responde dos veces al mismo sello
                self.ultimo = d['t']
                vw, info = self.nav.decidir(d)
                self.pub.publish(_twist(*vw))
                self.pub_f.publish(_ficha(d['t'], 1, {'nav': info}))
            return
        while True:
            r = self.buzon.sacar()
            if r is None:
                return
            f, ests = r
            cmds, info = self.op.responder(f['t'], ests)
            for vw in cmds:
                self.pub.publish(_twist(*vw))
            self.pub_f.publish(_ficha(f['t'], len(cmds), info))


class EnlaceROS:
    """Recibe el estado de la planta (sello t_k) y las ordenes del operador; ver el docstring del modulo."""

    def __init__(self, nodo, cfg, wait):
        self.nodo, self.wait = nodo, int(wait)
        self.e = L.EnlaceLock(cfg)
        self.pub_cmd = nodo.create_publisher(Twist, '/rover/cmd_vel', 50)
        self.pub_cmd_f = nodo.create_publisher(String, '/rover/cmd_vel_listo', 50)
        self.pub_est = nodo.create_publisher(String, '/rover/estado_tierra', 50)
        self.pub_est_f = nodo.create_publisher(String, '/rover/estado_tierra_listo', 50)
        self.pub_enl = nodo.create_publisher(String, '/rover/enlace', 50)
        self.buzon = L.Buzon()
        self.est = []
        self.ultimo = None
        nodo.create_subscription(String, '/rover/estado', lambda m: self._estado(m), 50)
        nodo.create_subscription(Twist, '/rover/cmd_vel_op', lambda m: self._cmd(m), 50)
        nodo.create_subscription(String, '/rover/cmd_vel_op_listo', lambda m: self._ficha(m), 50)
        self.timer = _reintento(nodo, self._avanzar)
        nodo.get_logger().info(f"rover_enlace LOCKSTEP: {cfg['planeta']}/{self.e.enlace.rele_nombre} modo {cfg['enlace']}")

    def _listo(self):
        return all(p.get_subscription_count() >= self.wait for p in (self.pub_cmd, self.pub_cmd_f, self.pub_est_f))

    def _estado(self, m):
        self.est.append(m.data)
        self._avanzar()

    def _cmd(self, m):
        self.buzon.meter((m.linear.x, m.angular.z))
        self._avanzar()

    def _ficha(self, m):
        self.buzon.ficha(json.loads(m.data))
        self._avanzar()

    def _avanzar(self):
        while self.est:
            txt = self.est[0]
            t = json.loads(txt)['t']
            if self.ultimo is not None and t <= self.ultimo:
                self.est.pop(0)                  # duplicado de la observacion inicial
                continue
            if not self._listo():
                return
            prev, info_prev = [], None
            if self.ultimo is not None:          # hace falta la respuesta del operador al tick anterior
                r = self.buzon.sacar()
                if r is None:
                    return
                f, prev = r
                info_prev = f.get('info')
            self.est.pop(0)
            cmds, ests, status, info = self.e.tick(t, txt, prev)
            self.ultimo = t
            for vw in cmds:
                self.pub_cmd.publish(_twist(*vw))
            for s in ests:
                self.pub_est.publish(String(data=s))
            if status:
                self.pub_enl.publish(String(data=json.dumps(status)))
            self.pub_est_f.publish(_ficha(t, len(ests)))
            self.pub_cmd_f.publish(_ficha(t, len(cmds), {'enlace': info, **(info_prev or {})}))
