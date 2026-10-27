"""Nodo: une `/atp/mcu/<eje>/joint_states` de cada ESP32 en `/joint_states`.

Es el lado PC de la interfaz del firmware (firmware/esp32_eje/README.md):
los ESP32 hablan por un agente micro-ROS y publican por eje; la Estacion
espera un unico /joint_states con name=[az, el] (CONTRATO.md, seccion 7).
Tambien registra en el log los cambios de estado y de fallas de cada eje
(`~/diag`) para que un paro o un atasco no pase en silencio.

Parametros:
  axes          orden de salida                       [az, el]
  simulated     ejes sin ESP32 (valor = referencia), separados por coma   ''
  stale_s       caducidad del dato de un eje real     0.5
  rate_hz       frecuencia de /joint_states           20
  announce_loop_closed  publica /atp/torque (0) cuando todos los ejes reales
                estan ACTIVOS: reference_generator congela el pase en AOS hasta
                verlo (igual que rotctld_bridge, atp_hw)          true
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from atp_mcu.mcu_core import Fusion, decodificar_diag


class McuMerge(Node):
    def __init__(self):
        super().__init__('mcu_merge')
        self.declare_parameter('axes', ['az', 'el'])
        self.declare_parameter('simulated', '')
        self.declare_parameter('stale_s', 0.5)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('announce_loop_closed', True)
        ejes = list(self.get_parameter('axes').value)
        sim = [x.strip() for x in str(self.get_parameter('simulated').value).split(',') if x.strip()]
        self.fusion = Fusion(ejes, sim, float(self.get_parameter('stale_s').value))
        self.estado_previo = {}
        self.estados = {}
        self.reales = [e for e in ejes if e not in sim]
        self.anuncia = bool(self.get_parameter('announce_loop_closed').value)
        self.t_anuncio = 0.0
        self.avisado = False

        reales = [e for e in ejes if e not in sim]
        for e in reales:
            self.create_subscription(
                JointState, f'/atp/mcu/{e}/joint_states',
                lambda m, e=e: self.on_medida(e, m), 50)
            self.create_subscription(
                Float32MultiArray, f'/atp/mcu/{e}/diag',
                lambda m, e=e: self.on_diag(e, m), qos_profile_sensor_data)
        # la referencia solo se usa para los ejes simulados
        if sim:
            self.create_subscription(JointState, '/atp/reference', self.on_ref, 50)
        self.pub = self.create_publisher(JointState, '/joint_states', 50)
        self.pub_arranque = self.create_publisher(JointState, '/atp/torque', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.publicar)

    def _ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_medida(self, eje, msg):
        if not msg.position:
            return
        vel = msg.velocity[0] if msg.velocity else 0.0
        eff = msg.effort[0] if msg.effort else 0.0
        self.fusion.medida(eje, msg.position[0], vel, eff, self._ahora())

    def on_ref(self, msg):
        for i, n in enumerate(msg.name):
            if n in self.fusion.simulados and i < len(msg.position):
                self.fusion.referencia(n, msg.position[i], self._ahora())

    def on_diag(self, eje, msg):
        d = decodificar_diag(msg.data)
        if d is None:
            return
        self.estados[eje] = d['estado_nombre']
        clave = (d['estado_nombre'], tuple(d['fallas_nombres']))
        if self.estado_previo.get(eje) != clave:
            self.estado_previo[eje] = clave
            nivel = self.get_logger().error if d['estado_nombre'] == 'FALLA' else self.get_logger().info
            nivel(f"eje {eje}: {d['estado_nombre']}"
                  + (f" fallas={d['fallas_nombres']}" if d['fallas_nombres'] else '')
                  + f" | lazo max {d['t_lazo_max_us']:.0f} us, jitter max {d['jitter_max_us']:.0f} us,"
                    f" ticks perdidos {d['ticks_perdidos']:.0f}")

    def publicar(self):
        r = self.fusion.combinar(self._ahora())
        if r is None:
            if not self.avisado:
                self.get_logger().warn(
                    f'sin dato fresco de los ejes reales {self.fusion.faltan(self._ahora())} (o sin referencia para los simulados): no se publica /joint_states')
                self.avisado = True
            return
        if self.avisado:
            self.get_logger().info('datos de los ejes recuperados')
            self.avisado = False
        nombres, pos, vel, eff = r
        if self.anuncia and all(self.estados.get(e) == 'ACTIVO' for e in self.reales) \
                and self._ahora() - self.t_anuncio >= 1.0:
            # "lazo cerrado": reference_generator arranca el pase al verlo
            self.t_anuncio = self._ahora()
            a = JointState()
            a.header.stamp = self.get_clock().now().to_msg()
            a.name = list(nombres)
            a.effort = [0.0] * len(nombres)
            self.pub_arranque.publish(a)
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name, m.position, m.velocity, m.effort = nombres, pos, vel, eff
        self.pub.publish(m)


def main():
    rclpy.init()
    node = McuMerge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
