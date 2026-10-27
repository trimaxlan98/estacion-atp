"""Predice los proximos pases de un satelite (o de todos los del catalogo)
sobre una estacion y los publica como JSON.

  /atp/passes            std_msgs/String, TRANSIENT_LOCAL: el ultimo calculo
  /atp/predict_passes    std_srvs/Trigger: recalcula con los parametros
                         actuales (`ros2 param set`) y devuelve el JSON en
                         `message`

Parametros: sat ('' = todos, o nombre / NORAD), station (clave del YAML, ''
= la por defecto), hours, start_utc ('' = ahora), mask_deg (<0 = la de la
estacion), min_el_max_deg (descarta pases mas bajos), refresh_s (>0
recalcula periodicamente; 0 = solo al arrancar y por servicio).

Corre con el reloj de PARED: predecir es una pregunta sobre el calendario, no
parte del lazo simulado. JSON: ver `orbit.pases_json`; los pases salen
ordenados por AOS.
"""
import datetime as dt
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from atp_ros import orbit
from atp_ros.params import declare_num


class PassPredictor(Node):
    def __init__(self):
        super().__init__('pass_predictor')
        self.declare_parameter('sat', '')
        self.declare_parameter('station', '')
        self.declare_parameter('start_utc', '')
        self.declare_parameter('tle_dir', '')
        self.declare_parameter('stations_file', '')
        declare_num(self, 'hours', 24.0)
        declare_num(self, 'mask_deg', -1.0)
        declare_num(self, 'min_el_max_deg', 0.0)
        declare_num(self, 'refresh_s', 0.0)

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/atp/passes', latched)
        self.create_service(Trigger, '/atp/predict_passes', self.on_service)
        self.publicar()
        refresh = float(self.get_parameter('refresh_s').value)
        if refresh > 0.0:
            self.create_timer(refresh, self.publicar)

    def calcular(self):
        """Devuelve el dict JSON-serializable con los pases."""
        p = self.get_parameter
        est = orbit.buscar_estacion(p('station').value or None,
                                    p('stations_file').value or None)
        cat = orbit.cargar_catalogo(p('tle_dir').value or None)
        sat = str(p('sat').value).strip()
        sats = [orbit.buscar_tle(cat, sat)] if sat else cat
        t0 = (orbit.a_utc(p('start_utc').value) if p('start_utc').value
              else dt.datetime.now(orbit.UTC))
        horas = float(p('hours').value)
        mask = float(p('mask_deg').value)
        mask = mask if mask >= 0.0 else est.mascara_deg
        pases = []
        for tle in sats:
            try:
                pases += orbit.predecir_pases(
                    tle, est, t0, horas, mask,
                    el_max_min_deg=float(p('min_el_max_deg').value))
            except Exception as e:      # un TLE malo no tumba a los demas
                self.get_logger().warn(f'{tle.nombre}: {e}')
        pases.sort(key=lambda x: x['aos'])
        ahora = dt.datetime.now(orbit.UTC)
        edades = {t.nombre: round(t.edad_dias(t0), 2) for t in sats}
        return {
            'generado_utc': ahora.strftime('%Y-%m-%dT%H:%M:%SZ'),
            'inicio_utc': t0.strftime('%Y-%m-%dT%H:%M:%SZ'),
            'horas': horas, 'mascara_deg': mask,
            'estacion': {'nombre': est.nombre, 'lat_deg': est.lat_deg,
                         'lon_deg': est.lon_deg, 'alt_m': est.alt_m},
            'edad_tle_dias': edades,
            'n': len(pases), 'pases': orbit.pases_json(pases),
        }

    def publicar(self):
        try:
            texto = json.dumps(self.calcular())
        except Exception as e:
            self.get_logger().error(f'no se pudo predecir: {e}')
            return None
        self.pub.publish(String(data=texto))
        self.get_logger().info(f'{len(texto)} bytes publicados en /atp/passes')
        return texto

    def on_service(self, _req, resp):
        texto = self.publicar()
        resp.success = texto is not None
        resp.message = texto or 'error al predecir (ver log)'
        return resp


def main():
    rclpy.init()
    node = PassPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
