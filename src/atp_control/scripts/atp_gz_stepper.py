#!/usr/bin/env python3
"""Avanza Gazebo PASO A PASO y espera a que el lazo cierre cada paso (modo `stepped`).

Con `gz sim` en pausa (atp_gz.launch.py stepped:=true) este nodo manda `multi_step`
por el servicio /world/atp/control y solo da el siguiente paso cuando el lazo de
control ya vio el anterior. Hace en Gazebo lo que `lockstep_node` y `sim_clock` hacen
en la simulacion sin Gazebo (CONTRATO 8), pero con un protocolo mas debil:

  fase A (arranque): pasos de `step_s` sin esperar a nadie, hasta ver el primer mando
                     de /atp/torque (el controller_manager solo procesa activar un
                     controlador mientras la simulacion avanza)
  fase B (lazo)    : tras cada paso espera, en este orden, a que
                       1. /clock llegue al instante del paso (el puente de Gazebo),
                       2. /joint_states traiga el estado de ese instante (broadcaster),
                       3. /atp/reference traiga la referencia de ese instante (generador),
                     y deja `settle_s` de pared para que llegue a los demas nodos.

Gazebo resuelve cada paso de 1 ms de forma determinista; lo que NO se vuelve
determinista por si solo es el orden de llegada de los mensajes de ROS 2 entre
procesos. El protocolo lo acota, no lo elimina (el comando de par del paso k se
calcula en la 1.a actualizacion del controller_manager del paso k+1 con la
referencia extrapolada, y la referencia de k puede llegar un instante antes o
despues): ver docs/ROS2_CONTROL.md para la variabilidad medida entre corridas.
"""
import time

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


def _t(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class Stepper(Node):
    def __init__(self):
        super().__init__('atp_gz_stepper')
        self.declare_parameter('world', 'atp')
        self.declare_parameter('step_s', 0.05)
        self.declare_parameter('physics_dt_s', 0.001)
        self.declare_parameter('settle_s', 0.004)
        self.declare_parameter('step_timeout_s', 10.0)
        g = lambda k: self.get_parameter(k).value     # noqa: E731
        self.service = f"/world/{g('world')}/control"
        self.n = int(round(float(g('step_s')) / float(g('physics_dt_s'))))
        self.step_s = float(g('step_s'))
        self.settle = float(g('settle_s'))
        self.timeout = float(g('step_timeout_s'))
        self.gz = GzNode()
        self.clock = 0.0
        self.js = -1.0
        self.ref = -1.0
        self.torque = False
        self.create_subscription(Clock, '/clock', self.on_clock, 100)
        self.create_subscription(JointState, '/joint_states', self.on_js, 100)
        self.create_subscription(JointState, '/atp/reference', self.on_ref, 100)
        self.create_subscription(JointState, '/atp/torque', self.on_torque, 100)

    def on_clock(self, m):
        self.clock = max(self.clock, m.clock.sec + m.clock.nanosec * 1e-9)

    def on_js(self, m):
        self.js = max(self.js, _t(m))

    def on_ref(self, m):
        self.ref = max(self.ref, _t(m))

    def on_torque(self, _m):
        self.torque = True

    def paso(self):
        req = WorldControl()
        req.multi_step = self.n
        ok, rep = self.gz.request(self.service, req, WorldControl, Boolean, 2000)
        return ok and rep.data

    def esperar(self, cond, limite):
        t0 = time.monotonic()
        while rclpy.ok() and not cond():
            rclpy.spin_once(self, timeout_sec=0.001)
            if time.monotonic() - t0 > limite:
                return False
        return True

    def run(self):
        self.get_logger().info(f'esperando {self.service}')
        while rclpy.ok() and not self.gz.service_list():
            time.sleep(0.2)
            rclpy.spin_once(self, timeout_sec=0.0)
        objetivo = 0.0
        pasos = 0
        eps = 1e-6
        while rclpy.ok():
            if not self.paso():
                self.get_logger().warn('multi_step no aceptado; reintento')
                time.sleep(0.05)
                continue
            objetivo += self.step_s
            pasos += 1
            if not self.esperar(lambda: self.clock >= objetivo - eps, self.timeout):
                self.get_logger().warn(f'el reloj no llego a t={objetivo:.3f}')
            if self.torque:
                # fase B: cierre del lazo
                if not self.esperar(lambda: self.js >= objetivo - eps, self.timeout):
                    self.get_logger().warn(f'/joint_states no llego a t={objetivo:.3f}')
                if not self.esperar(lambda: self.ref >= objetivo - eps, 2.0):
                    pass    # antes del pase el generador espera al grafo: no es un error
            else:
                time.sleep(0.01)
            t0 = time.monotonic()
            while time.monotonic() - t0 < self.settle:
                rclpy.spin_once(self, timeout_sec=0.0005)
            if pasos % 200 == 0:
                self.get_logger().info(f'paso {pasos}: t={self.clock:.2f} s')


def main():
    rclpy.init()
    n = Stepper()
    try:
        n.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
