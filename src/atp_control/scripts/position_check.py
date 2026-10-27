#!/usr/bin/env python3
"""Comprueba el modo de POSICION de AtpMountSystem con joint_trajectory_controller.

El controlador estandar reclama la interfaz `position` (no `effort`); el plugin cierra
entonces un servo PD interno (servo_kp, servo_kd) que convierte la consigna en par.
Envia una trayectoria de az 0 -> 40 deg y el 30 -> 55 deg en 8 s por el topico del
controlador, espera 6 s mas y verifica que las dos juntas quedaron a menos de
`tol_deg` de la consigna final y a velocidad ~0.

  ros2 launch atp_control atp_position.launch.py
"""
import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Check(Node):
    def __init__(self):
        super().__init__('position_check')
        self.declare_parameter('tol_deg', 0.1)
        self.declare_parameter('target_az_deg', 40.0)
        self.declare_parameter('target_el_deg', 55.0)
        self.pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.js = None
        self.t0 = None
        self.create_timer(0.1, self.tick)
        self.sent = False

    def on_js(self, m):
        self.js = dict(zip(m.name, zip(m.position, m.velocity)))

    def tick(self):
        if self.js is None or self.pub.get_subscription_count() == 0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.sent:
            self.sent = True
            self.t0 = now
            tr = JointTrajectory()
            tr.joint_names = ['az', 'el']
            tgt = [math.radians(self.get_parameter('target_az_deg').value),
                   math.radians(self.get_parameter('target_el_deg').value)]
            p = JointTrajectoryPoint()
            p.positions = tgt
            p.velocities = [0.0, 0.0]
            p.time_from_start = Duration(sec=8)
            tr.points = [p]
            self.pub.publish(tr)
            self.get_logger().info('trayectoria enviada')
            return
        if now - self.t0 < 14.0:
            return
        tol = float(self.get_parameter('tol_deg').value)
        e = []
        for j, tgt in (('az', self.get_parameter('target_az_deg').value),
                       ('el', self.get_parameter('target_el_deg').value)):
            pos, vel = self.js[j]
            e.append((j, math.degrees(pos) - tgt, vel))
        ok = all(abs(x) < tol and abs(v) < 1e-2 for _, x, v in e)
        self.get_logger().info(
            'POSICION ' + ('OK' if ok else 'FALLA') + ' | ' +
            ' | '.join(f'{j}: error={x:+.4f} deg vel={v:+.5f} rad/s' for j, x, v in e) +
            f' (tolerancia {tol} deg)')
        raise SystemExit(0 if ok else 1)


def main():
    rclpy.init()
    n = Check()
    code = 0
    try:
        rclpy.spin(n)
    except SystemExit as e:
        code = int(e.code or 0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(code)


if __name__ == '__main__':
    main()
