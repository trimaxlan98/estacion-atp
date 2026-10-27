"""Controlador ADCS: estimador (TRIAD + MEKF) y modos DETUMBLE/NADIR/TRACK_GS.

En lockstep responde a CADA estado de la planta con un mando sellado con el
MISMO instante (/atp/adcs_cmd), que es lo que la planta confirma para que el
reloj avance. Solo usa los sensores del mensaje (nunca la verdad) y la
efemeride del TLE (orbita conocida a bordo).
"""
import rclpy
from rclpy.node import Node

from sat_adcs.escenario import cargar_base
from sat_adcs.misiones import NOMBRE_MODO, ControladorAdcs
from sat_adcs.nodos_comunes import declarar_config, msg_de_cmd, sens_de_msg
from sat_interfaces.msg import AdcsCmd, AdcsState
from atp_ros.nodo import correr
from atp_ros.params import declare_num


class AdcsController(Node):
    def __init__(self):
        super().__init__('adcs_controller')
        cfg = declarar_config(self)
        # cuantos suscriptores del mando (planta, monitor...) deben estar conectados antes
        # de responder: un mando que el monitor no recibe por llegar tarde el descubrimiento
        # DDS le quitaria el principio de la corrida (CONTRATO sec. 8, regla 4)
        declare_num(self, 'wait_subscribers', 2)
        self.wait = int(self.get_parameter('wait_subscribers').value)
        self.listo = False
        sat, _, estacion, _, efem = cargar_base(cfg)
        self.ctrl = ControladorAdcs(cfg, sat, efem, estacion)
        self.modo = None
        m = self.ctrl.modelo
        self.get_logger().info(
            f'campo de a bordo: IGRF grado {cfg.modelo_bordo_nmax}; error contra IGRF-13 '
            f'completo en la orbita: {100 * m["rms_rel"]:.1f} % rms ({m["rms_deg"]:.1f} deg rms de '
            f'direccion); umbral de detumbling |w| < {self.ctrl.w_umbral * 57.2958:.3f} deg/s')
        self.pub = self.create_publisher(AdcsCmd, '/atp/adcs_cmd', 50)
        self.create_subscription(AdcsState, '/atp/adcs_state', self.on_state, 50)

    def on_state(self, msg):
        if not self.listo:
            if self.pub.get_subscription_count() < self.wait:
                return          # la planta repite el estado inicial hasta recibir el primer mando
            self.listo = True
        s = sens_de_msg(msg)
        c = self.ctrl.paso(s['t'], s)
        if c['modo'] != self.modo:
            self.modo = c['modo']
            self.get_logger().info(f"t={s['t']:.0f} s: modo {NOMBRE_MODO[c['modo']]}")
        # mismo sello que el estado al que responde (CONTRATO sec. 8, regla 1)
        self.pub.publish(msg_de_cmd(AdcsCmd(), c, s['t'], msg.header.stamp))


def main():
    correr(AdcsController)
