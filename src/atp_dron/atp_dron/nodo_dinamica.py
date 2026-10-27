"""Nodo `dron_dinamica` (CONTRATO sec. 8 y 21): el dron con su controlador de vuelo, en lockstep.

Suscribe `/dron/setpoint` y `/clock`; publica `/dron/estado` (String JSON), `/dron/pose` (PoseStamped, marco `world`, ENU) y confirma
cada tick en `/atp/step_ack`. Escribe telemetry.jsonl, metrics.json y setpoints.jsonl en `<salida>/<id>/` y sale al terminar.
Parametros: cfg (JSON de la escena completa, el que arma el launch), salida, id, wait_subscribers.
"""
import rclpy  # noqa: F401  (el entorno ROS debe estar cargado)
from rclpy.node import Node


class DronDinamica(Node):
    def __init__(self):
        super().__init__('dron_dinamica')
        import json
        from atp_dron.nodos_lockstep import PlantaROS
        for k, d in (('cfg', '{}'), ('salida', '/tmp/dron'), ('id', 'dron')):
            self.declare_parameter(k, d)
        self.declare_parameter('wait_subscribers', 1)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.lock = PlantaROS(self, json.loads(g('cfg')), g('salida'), g('id'), g('wait_subscribers'))


def main(args=None):
    from atp_ros.nodo import correr
    correr(DronDinamica)


if __name__ == '__main__':
    main()
