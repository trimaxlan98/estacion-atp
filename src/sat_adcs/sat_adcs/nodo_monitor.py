"""Monitor ADCS: mide con la VERDAD de la planta y al final imprime el RESUMEN.

Es un nodo pasivo (CONTRATO sec. 8, regla 3): colas profundas, empareja
estado y mando por sello (nunca "el ultimo que llego") y cierra la cuenta con
un temporizador de tiempo simulado de 3 s tras /atp/adcs_done. Las cifras
las calcula `misiones.Medidor`, el mismo que usa la corrida offline.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64

from sat_adcs.escenario import cargar_base
from sat_adcs.misiones import Medidor
from sat_adcs.nodos_comunes import cmd_de_msg, declarar_config, sello_ns, sens_de_msg
from sat_interfaces.msg import AdcsCmd, AdcsState
from atp_ros.nodo import correr


class AdcsMonitor(Node):
    def __init__(self):
        super().__init__('adcs_monitor')
        cfg = declarar_config(self)
        self.declare_parameter('exit_on_done', True)
        self.exit_on_done = bool(self.get_parameter('exit_on_done').value)
        sat, _, estacion, _, efem = cargar_base(cfg)
        self.med = Medidor(cfg, sat, efem, estacion)
        self.estados, self.cmds = {}, {}
        self.t_ultimo = -1.0
        self.t_fin = None
        self.cierre = None
        self.finished = False
        q = 400
        self.create_subscription(AdcsState, '/atp/adcs_state', self.on_state, q)
        self.create_subscription(AdcsCmd, '/atp/adcs_cmd', self.on_cmd, q)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64, '/atp/adcs_done', self.on_done, latched)

    def intentar(self, s):
        if s in self.estados and s in self.cmds:
            st, cmd = self.estados.pop(s), self.cmds.pop(s)
            if st['t'] > self.t_ultimo:                 # el estado inicial se repite al arrancar
                self.t_ultimo = st['t']
                self.med.muestra(st, cmd)

    def on_state(self, msg):
        s = sello_ns(msg.header.stamp)
        self.estados[s] = sens_de_msg(msg)
        self.intentar(s)

    def on_cmd(self, msg):
        s = sello_ns(msg.header.stamp)
        self.cmds[s] = cmd_de_msg(msg)
        self.intentar(s)

    def on_done(self, msg):
        if self.cierre is not None or self.finished:
            return
        self.t_fin = float(msg.data)
        # los ultimos instantes pueden llegar DESPUES del aviso (son otro topico, sin
        # orden entre ellos): 3 s simulados de margen (la planta sigue paso a paso
        # tras el fin) y `recortar` descarta lo que pase de t_fin
        self.cierre = self.create_timer(3.0, self.resumen)

    def resumen(self):
        self.cierre.cancel()
        if self.finished:
            return
        self.finished = True
        self.med.recortar(self.t_fin)
        if not self.med.t:
            self.get_logger().error('sin muestras')
            return
        for linea in self.med.resumen():
            self.get_logger().info(linea)
        if self.exit_on_done:
            raise SystemExit(0)


def main():
    correr(AdcsMonitor)
