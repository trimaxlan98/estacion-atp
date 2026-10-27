"""Nodo ROS 2 `rover_enlace` (R7, CONTRATO sec. 19): el enlace Tierra-rover entre el operador y la dinamica.

    operador --/rover/cmd_vel_op (Twist)--> [retardo + ventanas + perdida] --/rover/cmd_vel (Twist)--> rover_dinamica
    rover_dinamica --/rover/estado (String JSON)--> [retardo + DTN + perdida] --/rover/estado_tierra (String JSON)--> operador
    /rover/enlace (String JSON, 1 Hz sim): visibilidad, elevacion del rele, tasa, pendientes en cola, contadores.

Todo el calculo esta en `comunicacion.FiltroEnlace` y `Enlace` (Python puro). Los estados entregados llevan
`t_tierra` (instante de recepcion) y conservan su `t` original.

Dos modos de nodo (`modo_nodo`):
  * `pasivo` (defecto): el tiempo es el del ultimo /clock recibido (con `reloj:=pared` un temporizador de 0.1 s desde 0); una
    orden o un estado se sella con el /clock vigente CUANDO LLEGA el mensaje, asi que el instante de envio depende del
    orden de llegada en el DDS (no reproducible bit a bit). No confirma pasos.
  * `lockstep` (R9; `atp_rover.nodos_lockstep.EnlaceROS`): el tiempo es el sello `t` del estado de la planta. En cada tick
    k entrega al rover (`/rover/cmd_vel` + ficha `/rover/cmd_vel_listo`) las ordenes cuyo instante de entrega es <= t_k y
    a Tierra (`/rover/estado_tierra` + ficha) los estados vencidos; las ordenes del operador de t_{k-1} (ficha
    `/rover/cmd_vel_op_listo`) entran con SU sello de envio. Retardo en tiempo simulado, cuantizado al paso de 0.1 s y de
    un paso como minimo. `cfg` (JSON de la escena) trae `enlace` (= modo de FiltroEnlace), `rele`, `fecha`, `t0_s`, ...
Parametros: planeta, rele, lat_deg, lon_deg, terreno, semilla, antena, el_min_deg, fecha, modo (ideal|retardo|ventanas),
p_perdida_cmd, p_perdida_estado, horizonte_dias.
"""
import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from atp_rover.comunicacion import Enlace, FiltroEnlace, jd_de_iso


def _terreno(tipo, semilla):
    if tipo == 'llano':
        return None
    from atp_rover.terreno import Terreno
    return Terreno(tipo, semilla)


class RoverEnlace(Node):
    def __init__(self):
        super().__init__('rover_enlace')
        self.declare_parameter('modo_nodo', 'pasivo')
        if self.get_parameter('modo_nodo').value == 'lockstep':
            from atp_rover.nodos_lockstep import EnlaceROS
            self.declare_parameter('cfg', '{}')
            self.declare_parameter('wait_subscribers', 1)
            self.lock = EnlaceROS(self, json.loads(self.get_parameter('cfg').value),
                                  self.get_parameter('wait_subscribers').value)
            return
        for k, d in (('planeta', 'marte'), ('rele', 'mro'), ('lat_deg', -4.6), ('lon_deg', 137.4), ('terreno', 'llano'),
                     ('semilla', 0), ('antena', 'omni_helice'), ('el_min_deg', 10.0), ('fecha', '2026-10-07'),
                     ('modo', 'ventanas'), ('p_perdida_cmd', 0.0), ('p_perdida_estado', 0.0), ('horizonte_dias', 10.0),
                     ('reloj', 'clock')):
            self.declare_parameter(k, d)
        g = lambda k: self.get_parameter(k).value
        self.enlace = Enlace(g('planeta'), g('rele'), g('lat_deg'), g('lon_deg'), _terreno(g('terreno'), int(g('semilla'))),
                             g('antena'), el_min_deg=g('el_min_deg'), jd0=jd_de_iso(g('fecha')))
        self.filtro = FiltroEnlace(self.enlace, g('modo'), g('horizonte_dias') * self.enlace.cuerpo['dia_s'],
                                   g('p_perdida_cmd'), g('p_perdida_estado'), int(g('semilla')))
        self.t = 0.0
        self.t_enlace = -1.0
        self.pub_cmd = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.pub_est = self.create_publisher(String, '/rover/estado_tierra', 50)
        self.pub_enl = self.create_publisher(String, '/rover/enlace', 10)
        self.create_subscription(Twist, '/rover/cmd_vel_op', self.on_cmd, 10)
        self.create_subscription(String, '/rover/estado', self.on_estado, 50)
        if g('reloj') == 'pared':
            self.create_timer(0.1, self._tick_pared)
        else:
            self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.get_logger().info(f"rover_enlace: {g('planeta')}/{g('rele')} modo {g('modo')}")

    def on_cmd(self, m):
        self.filtro.cmd(self.t, (float(m.linear.x), float(m.angular.z)))

    def on_estado(self, m):
        self.filtro.estado(self.t, m.data)

    def _avanzar(self, t):
        self.t = t
        cmds, ests = self.filtro.listos(t)
        for _, (v, w) in cmds:
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
            self.pub_cmd.publish(msg)
        for t_ent, txt in ests:
            d = json.loads(txt)
            d['t_tierra'] = round(t_ent, 3)
            self.pub_est.publish(String(data=json.dumps(d)))
        if t - self.t_enlace >= 1.0:
            self.t_enlace = t
            m = self.enlace.muestras([t])
            pc, pe = self.filtro.pendientes()
            self.pub_enl.publish(String(data=json.dumps({
                't': round(t, 3), 'visible': bool(m['visible'][0]), 'el_deg': round(float(m['el'][0]), 2),
                'tasa_bps': float(m['tasa_bps'][0]), 'cmd_pendientes': pc, 'estados_pendientes': pe,
                **self.filtro.stats})))

    def on_clock(self, m):
        self._avanzar(m.clock.sec + m.clock.nanosec * 1e-9)

    def _tick_pared(self):
        self._avanzar(self.t + 0.1)


def main(args=None):
    from atp_ros.nodo import correr
    correr(RoverEnlace)


if __name__ == '__main__':
    main()
