"""Un `rclpy` en memoria para ejecutar los nodos REALES (`atp_dron.nodos_lockstep`, `atp_ros.sim_clock`) sin ROS ni DDS.

Reproduce de DDS lo que importa para el lockstep (CONTRATO sec. 8): cada suscripcion tiene su cola FIFO, pero el orden de entrega
ENTRE suscripciones/topicos es aleatorio (semilla); un suscriptor solo recibe lo publicado despues de que `descubrimiento`
iteraciones lo emparejaron (QoS volatil); los temporizadores de pared se disparan en momentos aleatorios y el reloj de pared de
`sim_clock` es una cuenta de iteraciones. NO es DDS: no prueba QoS reales, serializacion ni el ejecutor de rclpy.
"""
import random
import sys
import types
from collections import deque


# ----------------------------------------------------------------------------------------------------------- mensajes
class Time:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


class Clock:
    def __init__(self, clock=None):
        self.clock = clock if clock is not None else Time()


class String:
    def __init__(self, data=''):
        self.data = data


class _P:
    def __init__(self, **k):
        self.__dict__.update(k)


class PoseStamped:
    def __init__(self):
        self.header = _P(stamp=Time(), frame_id='')
        self.pose = _P(position=_P(x=0.0, y=0.0, z=0.0), orientation=_P(x=0.0, y=0.0, z=0.0, w=1.0))


def self_it_ok(bus, sub):
    return bus.it - sub.t0 >= bus.descub + bus.desfase.get(sub.topic, 0)


# ------------------------------------------------------------------------------------------------------------- el bus
class Bus:
    actual = None

    def __init__(self, semilla=0, descubrimiento=0, p_timer=0.3, desfase_datos=None):
        self.rng = random.Random(semilla)
        self.it, self.descub, self.p_timer = 0, descubrimiento, p_timer
        self.desfase = desfase_datos or {}   # {topico: iteraciones}: el emparejamiento se anuncia antes de que fluyan los datos
        self.subs, self.timers, self.nodos = [], [], []
        self.publicados = {}

    def pared(self):
        return 1000.0 + self.it * 0.0005             # medio ms de pared por iteracion

    def publicar(self, topic, msg):
        self.publicados[topic] = self.publicados.get(topic, 0) + 1
        for s in self.subs:
            if s.topic == topic and s.activa() and self_it_ok(self, s):
                s.cola.append(msg)

    def correr(self, fin=lambda: False, max_it=3_000_000):
        """Entrega mensajes/dispara temporizadores hasta `fin()` o hasta que un nodo haga SystemExit. Devuelve True si termino."""
        try:
            while self.it < max_it:
                self.it += 1
                listos = [s for s in self.subs if s.cola]
                if listos and self.rng.random() >= self.p_timer:
                    s = self.rng.choice(listos)
                    s.cb(s.cola.popleft())
                else:
                    vivos = [t for t in self.timers if not t.cancelado]
                    if vivos:
                        self.rng.choice(vivos).cb()
                if fin():
                    return True
        except SystemExit:
            return True
        return False


class _Pub:
    def __init__(self, bus, topic):
        self.bus, self.topic = bus, topic

    def publish(self, msg):
        self.bus.publicar(self.topic, msg)

    def get_subscription_count(self):
        return sum(1 for s in self.bus.subs if s.topic == self.topic and s.activa())


class _Sub:
    def __init__(self, bus, topic, cb):
        self.bus, self.topic, self.cb = bus, topic, cb
        self.topic_name, self.cola, self.t0 = topic, deque(), bus.it

    def activa(self):
        return self.bus.it - self.t0 >= self.bus.descub

    def get_publisher_count(self):
        return 1


class _Timer:
    def __init__(self, cb):
        self.cb, self.cancelado = cb, False

    def cancel(self):
        self.cancelado = True


class _Log:
    def __init__(self):
        self.lineas = []

    def info(self, m):
        self.lineas.append(('info', m))

    warn = warning = error = debug = info


class Node:
    parametros = {}                                   # {nombre_nodo: {param: valor}}

    def __init__(self, nombre):
        self.bus = Bus.actual
        self._nombre, self._params, self._log = nombre, {}, _Log()
        self.subscriptions = []
        self.bus.nodos.append(self)

    def get_name(self):
        return self._nombre

    def get_logger(self):
        return self._log

    def declare_parameter(self, nombre, valor=None, descriptor=None):
        self._params[nombre] = Node.parametros.get(self._nombre, {}).get(nombre, valor)

    def get_parameter(self, nombre):
        return types.SimpleNamespace(value=self._params[nombre])

    def create_publisher(self, tipo, topic, qos=None):
        return _Pub(self.bus, topic)

    def create_subscription(self, tipo, topic, cb, qos=None):
        s = _Sub(self.bus, topic, cb)
        self.bus.subs.append(s)
        self.subscriptions.append(s)
        return s

    def create_timer(self, periodo, cb, clock=None):
        t = _Timer(cb)
        self.bus.timers.append(t)
        return t


# --------------------------------------------------------------------------------------------- modulos falsos en sys.modules
def _mod(nombre, **attrs):
    m = types.ModuleType(nombre)
    m.__dict__.update(attrs)
    return m


def modulos():
    class ClockType:
        STEADY_TIME = 'steady'

    class DurabilityPolicy:
        TRANSIENT_LOCAL = 'tl'

    class QoSProfile:
        def __init__(self, **k):
            self.__dict__.update(k)

    class ExternalShutdownException(Exception):
        pass

    rclpy = _mod('rclpy', ok=lambda: True, init=lambda *a, **k: None, shutdown=lambda: None, spin=lambda n: None)
    return {
        'rclpy': rclpy,
        'rclpy.node': _mod('rclpy.node', Node=Node),
        'rclpy.clock': _mod('rclpy.clock', Clock=lambda **k: None, ClockType=ClockType),
        'rclpy.qos': _mod('rclpy.qos', QoSProfile=QoSProfile, DurabilityPolicy=DurabilityPolicy),
        'rclpy.executors': _mod('rclpy.executors', ExternalShutdownException=ExternalShutdownException),
        'rcl_interfaces': _mod('rcl_interfaces'),
        'rcl_interfaces.msg': _mod('rcl_interfaces.msg', ParameterDescriptor=lambda **k: None),
        'builtin_interfaces': _mod('builtin_interfaces'),
        'builtin_interfaces.msg': _mod('builtin_interfaces.msg', Time=Time),
        'rosgraph_msgs': _mod('rosgraph_msgs'),
        'rosgraph_msgs.msg': _mod('rosgraph_msgs.msg', Clock=Clock),
        'std_msgs': _mod('std_msgs'),
        'std_msgs.msg': _mod('std_msgs.msg', String=String),
        'geometry_msgs': _mod('geometry_msgs'),
        'geometry_msgs.msg': _mod('geometry_msgs.msg', PoseStamped=PoseStamped),
    }


DEPENDIENTES = ('atp_dron.nodos_lockstep', 'atp_dron.nodo_dinamica', 'atp_dron.nodo_navegador', 'atp_ros.sim_clock', 'atp_ros.nodo',
                'atp_ros.params')


def instalar():
    """Pone los modulos falsos; devuelve una funcion que restaura sys.modules (y olvida los que los importaron)."""
    previos = {k: sys.modules.get(k) for k in list(modulos()) + list(DEPENDIENTES)}
    for k in DEPENDIENTES:
        sys.modules.pop(k, None)
    sys.modules.update(modulos())

    def restaurar():
        for k, v in previos.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return restaurar
