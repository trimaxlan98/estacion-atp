"""Nodo `rover_dinamica` (CONTRATO sec. 18 y 8): integra el rover en tiempo simulado y publica `/rover/estado`.

Suscribe `/rover/cmd_vel` (geometry_msgs/Twist: linear.x m/s, angular.z rad/s) y `/clock`. Publica `/rover/estado`
(std_msgs/String, JSON del contrato) cada 0.1 s de tiempo simulado, sellado por el contador de pasos (`t`, sin reloj de pared).

Dos modos (`modo`):
  * `pasivo` (defecto; uso interactivo, p. ej. `ros2 topic pub /rover/cmd_vel ...`, y el visor de Gazebo): con cada /clock
    integra (RK4, dt = 0.01 s) hasta el instante del reloj con el ULTIMO mando recibido. No confirma pasos
    (`/atp/step_ack`) ni frena el reloj, asi que el mando que rige un tramo depende del orden de llegada respecto al
    /clock en el DDS (no controlado): NO es reproducible bit a bit con un controlador en el lazo. Si `reloj:=pared` (sin
    /clock) avanza con un temporizador de 0.1 s a tiempo real.
  * `lockstep` (CONTRATO sec. 8 y 18, R9; `atp_rover.nodos_lockstep.PlantaROS`): publica e_k con el sello t_k, espera la ficha
    `/rover/cmd_vel_listo` de t_k y su mando, integra el intervalo [t_k, t_{k+1}) COMPLETO con ese mando, confirma el
    /clock en `/atp/step_ack` y no pasa al tick siguiente hasta que el reloj llegue. Escribe telemetry.jsonl, metrics.json
    y mandos.jsonl en `<salida>/<id>/` y sale al terminar. Misma corrida con o sin ROS: mismos bytes (docs/ROVER.md, R9).

Parametros: planeta, terreno, semilla, terreno_params (JSON), rover (JSON), x0, y0, rumbo_deg; en lockstep: cfg (JSON de la
escena completa, el que arma el launch), salida, id, wait_subscribers.
"""
import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from atp_rover.cli import DEFECTOS, construir
from atp_rover.rover import DT


class RoverDinamica(Node):
    def __init__(self):
        super().__init__('rover_dinamica')
        self.declare_parameter('modo', 'pasivo')
        if self.get_parameter('modo').value == 'lockstep':
            from atp_rover.nodos_lockstep import PlantaROS
            for k, d in (('cfg', '{}'), ('salida', '/tmp/rover'), ('id', 'rover')):
                self.declare_parameter(k, d)
            self.declare_parameter('wait_subscribers', 1)
            g = lambda k: self.get_parameter(k).value
            self.lock = PlantaROS(self, json.loads(g('cfg')), g('salida'), g('id'), g('wait_subscribers'))
            return
        for k in ('planeta', 'terreno'):
            self.declare_parameter(k, DEFECTOS[k])
        self.declare_parameter('semilla', 0)
        self.declare_parameter('terreno_params', '{}')
        self.declare_parameter('rover', '{}')
        for k in ('x0', 'y0', 'rumbo_deg'):
            self.declare_parameter(k, float(DEFECTOS[k]))
        self.declare_parameter('reloj', 'clock')
        g = lambda k: self.get_parameter(k).value
        cfg = {**DEFECTOS, 'planeta': g('planeta'), 'terreno': g('terreno'), 'semilla': int(g('semilla')),
               'terreno_params': json.loads(g('terreno_params') or '{}'), 'rover': json.loads(g('rover') or '{}'),
               'x0': g('x0'), 'y0': g('y0'), 'rumbo_deg': g('rumbo_deg')}
        self.rov, self.ter, self.pl = construir(cfg)
        self.rov.asentar(2.0)
        self.cmd = (0.0, 0.0)
        self.pub = self.create_publisher(String, '/rover/estado', 50)
        self.create_subscription(Twist, '/rover/cmd_vel', self.on_cmd, 10)
        if g('reloj') == 'pared':
            self.create_timer(0.1, self._tick_pared)
        else:
            self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.get_logger().info(f"rover_dinamica: {cfg['planeta']} / {cfg['terreno']} semilla {cfg['semilla']}")

    def on_cmd(self, m):
        self.cmd = (float(m.linear.x), float(m.angular.z))

    def _avanzar_hasta(self, t_obj):
        n_obj = int(round(t_obj / DT))
        while self.rov.n_pasos < n_obj:
            self.rov.paso(*self.cmd)
            if self.rov.n_pasos % 10 == 0:
                self.pub.publish(String(data=json.dumps(self.rov.estado())))

    def on_clock(self, m):
        self._avanzar_hasta(m.clock.sec + m.clock.nanosec * 1e-9)

    def _tick_pared(self):
        self._avanzar_hasta(self.rov.t + 0.1)


def main(args=None):
    from atp_ros.nodo import correr
    correr(RoverDinamica)


if __name__ == '__main__':
    main()
