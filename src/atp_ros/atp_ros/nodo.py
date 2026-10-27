"""Arranque y cierre comun de los nodos de la Estacion."""
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

# Saludo de arranque (CONTRATO 8): cada nodo anuncia su nombre en /atp/listo
# cuando TODAS sus suscripciones tienen ya su publicador emparejado. Que un
# nodo exista no basta: con la maquina cargada DDS puede tardar segundos en
# emparejar, y en lockstep a maxima velocidad eso son decenas de segundos
# simulados perdidos (el beacon llego 15 s tarde en una corrida medida).
LISTO = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL)
_IGNORAR = ('/clock', '/parameter_events', '/rosout')


def anunciar_listo(node):
    """Publica el nombre del nodo en /atp/listo cuando cada suscripcion suya
    (salvo reloj y parametros) tiene al menos un publicador emparejado."""
    pub = node.create_publisher(String, '/atp/listo', LISTO)
    estado = {'hecho': False}

    def revisar():
        if estado['hecho']:
            return
        for sub in node.subscriptions:
            if sub.topic_name.startswith(_IGNORAR):
                continue
            if sub.get_publisher_count() < 1:
                return
        estado['hecho'] = True
        pub.publish(String(data=node.get_name()))
        timer.cancel()

    # de PARED: con la compuerta de arranque el reloj simulado esta parado
    timer = node.create_timer(0.05, revisar, clock=Clock(clock_type=ClockType.STEADY_TIME))

# Carrera conocida de rclpy al cerrar: el SIGINT del launch invalida el
# contexto mientras el ejecutor entrega un ultimo mensaje.
_CARRERA_CIERRE = 'Unable to convert call argument'


def _es_cierre(e):
    return not rclpy.ok() or _CARRERA_CIERRE in str(e)


def correr(clase):
    """Crea el nodo, gira hasta Ctrl-C, SystemExit o el cierre del launch,
    y apaga sin trazas espurias. No toca los manejadores de senales: rclpy
    los necesita para despertar al ejecutor bloqueado en C."""
    rclpy.init()
    node = clase()
    anunciar_listo(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    except Exception as e:
        if not _es_cierre(e):
            raise
    finally:
        # destruir el nodo con el contexto ya cerrado puede lanzar lo mismo
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            if not _es_cierre(e):
                raise
