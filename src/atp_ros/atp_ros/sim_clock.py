"""Reloj simulado en LOCKSTEP: el tiempo solo avanza cuando el lazo cerro el paso.

Cadena de un paso k (t_k = k * step_s):
  /clock t_k -> mount_sim integra hasta t_k y publica /joint_states(t_k)
             -> reference_generator publica /atp/reference(t_k)
             -> pid_controller (estado y referencia con el MISMO sello) manda
             -> mount_sim recibe el mando y confirma en /atp/step_ack(t_k)
             -> /clock t_{k+1}
Asi el resultado no depende de la carga de la CPU: misma semilla, mismos
numeros. `time_scale` solo limita la velocidad (<=0: tan rapido como se
pueda).

Compuerta de arranque: con `lockstep:=true` el reloj NO pasa del primer paso
hasta la primera confirmacion. El generador no publica nada hasta que todo
el grafo esta emparejado, asi que la linea de tiempo absoluta es la misma en
cada corrida por mucho que tarde el descubrimiento de DDS. Con
`lockstep:=false` (bancos sin montura simulada, como el HIL con un rotor
real) nadie confirma y el reloj corre libre a `time_scale` (<=0 se toma x1).
"""
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from atp_ros.nodo import correr
from atp_ros.params import declare_num


class SimClock(Node):
    def __init__(self):
        super().__init__('sim_clock')
        declare_num(self, 'time_scale', 1.0)
        declare_num(self, 'step_s', 0.05)
        declare_num(self, 'ack_timeout_s', 10.0)     # de pared; solo red de seguridad
        self.declare_parameter('lockstep', True)
        g = lambda k: float(self.get_parameter(k).value)  # noqa: E731
        self.scale = g('time_scale')
        self.step_ns = int(round(g('step_s') * 1e9))
        self.timeout = g('ack_timeout_s')
        self.modo_lockstep = bool(self.get_parameter('lockstep').value)
        self.k = 0
        self.acked = -1               # ultimo paso confirmado
        self.lockstep = False         # se activa con la primera confirmacion
        self.stalls = 0
        self.t_pub = 0.0
        self.t_latido = 0.0
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.create_subscription(Clock, '/atp/step_ack', self.on_ack, 10)
        self.wall0 = time.monotonic()
        self.publish()
        self.create_timer(0.0005, self.tick)        # temporizador de pared
        modo = 'maxima velocidad' if self.scale <= 0 else f'x{self.scale:g}'
        tipo = 'lockstep' if self.modo_lockstep else 'libre'
        self.get_logger().info(f'reloj simulado {tipo}, paso {g("step_s"):g} s, {modo}')

    def on_ack(self, msg):
        k = (msg.clock.sec * 1_000_000_000 + msg.clock.nanosec) // self.step_ns
        self.acked = max(self.acked, int(k))
        if not self.lockstep:
            self.lockstep = True
            self.get_logger().info('lazo cerrado: avance en lockstep')

    def publish(self):
        ns = self.k * self.step_ns
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(ns, 1_000_000_000)
        self.pub.publish(msg)
        self.t_pub = time.monotonic()

    def tick(self):
        now = time.monotonic()
        # latido: mientras se espera, se repite el instante actual cada 0.2 s
        # de pared para quien termino de descubrir /clock tarde (un /clock
        # perdido dejaria a ese nodo en t=0 y al lazo esperando)
        esperando = (self.lockstep and self.acked < self.k) or \
            (self.modo_lockstep and not self.lockstep and self.k >= 1)
        if esperando and now - self.t_pub > 0.2 and now - self.t_latido > 0.2:
            self.t_latido = now
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = divmod(self.k * self.step_ns, 1_000_000_000)
            self.pub.publish(msg)
        if self.lockstep and self.acked < self.k:
            if now - self.t_pub < self.timeout:
                return
            self.stalls += 1
            if self.stalls in (1, 10, 100) or self.stalls % 1000 == 0:
                self.get_logger().warn(f'paso {self.k} sin confirmar ({self.stalls} veces)')
        # compuerta: en lockstep no se pasa del primer paso sin confirmacion
        if self.modo_lockstep and not self.lockstep and self.k >= 1:
            return
        # ritmo: time_scale; sin limite (<=0) solo en lockstep
        escala = self.scale if self.scale > 0 else (0.0 if self.lockstep else 1.0)
        if escala > 0:
            t_sig = (self.k + 1) * self.step_ns * 1e-9
            if (now - self.wall0) * escala < t_sig:
                return
        self.k += 1
        self.publish()


def main():
    correr(SimClock)
