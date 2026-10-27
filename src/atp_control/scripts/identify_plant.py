#!/usr/bin/env python3
"""Identifica J y b de la montura (cualquier hardware de ros2_control) con un escalon de par.

Aplica `u` N m al eje (por /effort_controller/commands), mide la velocidad por
/joint_states y ajusta el modelo J w' + b w = u:

  * subida:  w(t) = (u/b) (1 - exp(-t/tau)),  tau = J/b   (ajuste lineal de ln(1 - w/w_inf))
  * bajada:  w(t) = w0 exp(-t/tau)           (al quitar el par; independiente de u)

Da b = u / w_inf y J = tau b. Con la planta de planta.py (J = 2, b = 0.5, tau = 4 s)
debe salir exactamente eso; con Gazebo da la inercia y el amortiguamiento FISICOS
que hay detras de las juntas del URDF. Parametros: axis (0 = az, 1 = el), torque_nm,
t_hold_s, t_push_s, t_coast_s (en tiempo del nodo: simulado si use_sim_time).

  ros2 run atp_control identify_plant.py --ros-args -p axis:=0 -p torque_nm:=1.0
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class Identify(Node):
    def __init__(self):
        super().__init__('identify_plant')
        self.declare_parameter('axis', 0)
        self.declare_parameter('torque_nm', 1.0)
        self.declare_parameter('t_hold_s', 2.0)
        self.declare_parameter('t_push_s', 24.0)     # 6 tau: w llega al 99.8 %
        self.declare_parameter('t_coast_s', 20.0)
        self.declare_parameter('out_file', '')
        self.declare_parameter('csv_file', '')
        g = lambda k: self.get_parameter(k).value    # noqa: E731
        self.axis, self.u = int(g('axis')), float(g('torque_nm'))
        self.t_hold, self.t_push, self.t_coast = float(g('t_hold_s')), float(g('t_push_s')), float(g('t_coast_s'))
        self.pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 5000)
        self.rows = []          # (t, theta, omega)
        self.t0 = None
        self.fase = 'espera'
        self.create_timer(0.02, self.tick)
        self.get_logger().info('identify_plant: esperando /joint_states')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_js(self, m):
        if 'az' in m.name:
            i = m.name.index('az' if self.axis == 0 else 'el')
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            w = m.velocity[i] if len(m.velocity) > i else float('nan')
            self.rows.append((t, m.position[i], w))
            if self.t0 is None:
                self.t0 = self.now_s()

    def comando(self, u):
        d = [0.0, 0.0]
        d[self.axis] = u
        self.pub.publish(Float64MultiArray(data=d))

    def tick(self):
        if self.t0 is None:
            return
        t = self.now_s() - self.t0
        if t < self.t_hold:
            self.comando(0.0)
        elif t < self.t_hold + self.t_push:
            self.fase = 'empuje'
            self.comando(self.u)
        elif t < self.t_hold + self.t_push + self.t_coast:
            self.fase = 'inercia'
            self.comando(0.0)
        else:
            self.comando(0.0)
            self.resultado()

    def resultado(self):
        a = np.array(self.rows)
        csv = self.get_parameter('csv_file').value
        if csv:
            np.savetxt(csv, a, delimiter=',', header='t,theta,omega')
        t = a[:, 0] - a[0, 0]
        w = a[:, 2]
        t1, t2 = self.t_hold, self.t_hold + self.t_push
        t3 = t2 + self.t_coast
        # deriva en reposo (planta equilibrada: ningun par sin mando)
        reposo = w[t < self.t_hold - 0.2]
        deriva = float(np.max(np.abs(reposo))) if len(reposo) else float('nan')
        # subida: minimos cuadrados de w(t) = w_inf (1 - exp(-t/tau)); para cada tau el w_inf
        # optimo es lineal, asi que basta barrer tau (sin sesgo aunque no llegue al estacionario)
        s = (t > t1) & (t <= t2)
        x = t[s] - t1
        y = w[s]
        mejor = (float('inf'), 0.0, 0.0, 0.0)
        # tambien se ajusta el instante en que el par llega a la planta (latencia del
        # controlador y del bus): ts en [0, 0.5] s
        for ts in np.arange(0.0, 0.5, 0.01):
            xs = np.clip(x - ts, 0.0, None)
            for tau in np.arange(0.2, 40.0, 0.01):
                f = 1.0 - np.exp(-xs / tau)
                winf = float(np.dot(f, y) / np.dot(f, f))
                r = float(np.sum((y - winf * f) ** 2))
                if r < mejor[0]:
                    mejor = (r, tau, winf, ts)
        _, tau_up, w_inf, ts_mejor = mejor
        # bajada: ln(w/w0)
        s2 = (t > t2) & (t <= t3)
        x2 = t[s2] - t2
        y2 = w[s2] / w[s2][0] if np.any(s2) else np.array([])
        k2 = (y2 > 0.05) & (y2 < 0.90)
        tau_dn = float('nan')
        if np.count_nonzero(k2) > 5:
            tau_dn = -1.0 / np.polyfit(x2[k2], np.log(y2[k2]), 1)[0]
        b = self.u / w_inf
        res = {'eje': 'az' if self.axis == 0 else 'el', 'par_nm': self.u, 'w_inf_rad_s': w_inf,
               'b_nm_s_rad': b, 'tau_subida_s': float(tau_up), 'tau_bajada_s': float(tau_dn),
               'J_kg_m2': float(tau_up * b), 'J_de_bajada_kg_m2': float(tau_dn * b),
               'deriva_en_reposo_rad_s': deriva, 'latencia_par_s': float(ts_mejor), 'muestras': int(len(t))}
        self.get_logger().info('IDENT ' + json.dumps(res))
        out = self.get_parameter('out_file').value
        if out:
            with open(out, 'w') as f:
                json.dump(res, f, indent=1)
        raise SystemExit(0)


def main():
    rclpy.init()
    n = Identify()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
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
