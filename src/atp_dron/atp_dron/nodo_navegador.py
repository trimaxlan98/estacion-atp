"""Nodo `dron_navegador` (CONTRATO sec. 8 y 21): la computadora de mision del dron, en lockstep.

Suscribe `/dron/estado` y publica `/dron/setpoint` (uno por sello). Parametros: cfg (JSON de la escena), wait_subscribers.
Se puede sustituir por otro nodo que cumpla el contrato de `/dron/setpoint` (ver `nodos_lockstep`).
"""
import rclpy  # noqa: F401
from rclpy.node import Node


class DronNavegador(Node):
    def __init__(self):
        super().__init__('dron_navegador')
        import json
        from atp_dron.nodos_lockstep import GuiadoROS
        self.declare_parameter('cfg', '{}')
        self.declare_parameter('wait_subscribers', 1)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.lock = GuiadoROS(self, json.loads(g('cfg')), g('wait_subscribers'))


def main(args=None):
    from atp_ros.nodo import correr
    correr(DronNavegador)


if __name__ == '__main__':
    main()
