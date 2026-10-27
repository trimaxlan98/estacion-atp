"""Planifica los contactos de una red de estaciones (N satelites x M estaciones)
y publica el plan como JSON.

  /atp/contact_plan      std_msgs/String, TRANSIENT_LOCAL: el ultimo plan
  /atp/plan_contacts     std_srvs/Trigger: recalcula con los parametros
                         actuales (`ros2 param set`) y devuelve el JSON en
                         `message`

Parametros: sats (nombres o NORAD separados por coma; '' = todo el catalogo),
stations (claves de data/estaciones.yaml separadas por coma; '' = todas),
hours, start_utc ('' = ahora; con fecha fija el plan es repetible byte a
byte), band (UHF|S|X|Ka), eirp_dbw, rs_baud, margen_acm_db, vel_az_deg_s,
vel_el_deg_s, t_asent_s, keyhole (marcar|recortar|descartar), el_min_deg,
sep_sat_s, algoritmo (codicioso|optimo|ambos), metodo (auto|bb|milp),
prioridades ("NOAA 19=2,ISS=1": peso por satelite), tle_dir, stations_file.

Corre con el reloj de PARED y es una pregunta sobre el calendario, no parte
del lazo simulado (CONTRATO 8: no usa /clock ni frena el lockstep). Calcular
24 h de 4 estaciones x 8 satelites tarda ~20 s (SGP4): el nodo bloquea ese
tiempo al arrancar y en cada llamada al servicio. JSON: ver
`planificador.calcular_plan`.
"""
import datetime as dt
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from atp_ros import orbit, planificador
from atp_ros.params import declare_num


def _lista(texto):
    return [x.strip() for x in str(texto).split(',') if x.strip()]


def _prioridades(texto):
    out = {}
    for par in _lista(texto):
        nombre, _, valor = par.rpartition('=')
        if not nombre:
            raise ValueError(f'prioridad mal formada: {par!r} (use nombre=numero)')
        out[nombre.strip()] = float(valor)
    return out


class ContactScheduler(Node):
    def __init__(self):
        super().__init__('contact_scheduler')
        self.declare_parameter('sats', 'ISS,NOAA 19,METEOR-M2 3')
        self.declare_parameter('stations', 'cdmx,tromso')
        self.declare_parameter('start_utc', '')
        self.declare_parameter('band', 'X')
        self.declare_parameter('keyhole', 'marcar')
        self.declare_parameter('algoritmo', 'ambos')
        self.declare_parameter('metodo', 'auto')
        self.declare_parameter('prioridades', '')
        self.declare_parameter('tle_dir', '')
        self.declare_parameter('stations_file', '')
        for nombre, defecto in (('hours', 24.0), ('eirp_dbw', 12.0),
                                ('rs_baud', 10e6), ('margen_acm_db', 1.0),
                                ('vel_az_deg_s', 6.0), ('vel_el_deg_s', 3.0),
                                ('t_asent_s', 5.0), ('el_min_deg', 0.0),
                                ('sep_sat_s', 0.0)):
            declare_num(self, nombre, defecto)

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/atp/contact_plan', latched)
        self.create_service(Trigger, '/atp/plan_contacts', self.on_service)
        self.publicar()

    def calcular(self):
        """Devuelve el dict JSON-serializable del plan."""
        p = self.get_parameter
        num = lambda k: float(p(k).value)          # noqa: E731
        todas, _ = orbit.cargar_estaciones(p('stations_file').value or None)
        claves = _lista(p('stations').value) or list(todas)
        estaciones = {}
        for q in claves:
            clave = next((c for c, e in todas.items()
                          if q.lower() in (c.lower(), e.nombre.lower())), None)
            if clave is None:
                raise KeyError(f'estacion {q!r} no existe; hay: {", ".join(todas)}')
            estaciones[clave] = todas[clave]
        cat = orbit.cargar_catalogo(p('tle_dir').value or None)
        nombres = _lista(p('sats').value)
        tles = [orbit.buscar_tle(cat, s) for s in nombres] if nombres else cat
        t0 = (orbit.a_utc(p('start_utc').value) if p('start_utc').value
              else dt.datetime.now(orbit.UTC).replace(microsecond=0))
        enlace = planificador.PerfilEnlace(
            banda=str(p('band').value), eirp_dbw=num('eirp_dbw'),
            rs_baud=num('rs_baud'), margen_acm_db=num('margen_acm_db'))
        rotor = planificador.Rotor(
            vel_az_deg_s=num('vel_az_deg_s'), vel_el_deg_s=num('vel_el_deg_s'),
            t_asent_s=num('t_asent_s'), keyhole=str(p('keyhole').value))
        alg = str(p('algoritmo').value)
        if alg not in ('codicioso', 'optimo', 'ambos'):
            raise ValueError("algoritmo debe ser 'codicioso', 'optimo' o 'ambos'")
        return planificador.calcular_plan(
            tles, estaciones, t0, num('hours'), enlace, rotor,
            prioridades=_prioridades(p('prioridades').value),
            el_min_deg=num('el_min_deg'), sep_sat_s=num('sep_sat_s'),
            metodo=str(p('metodo').value), optimo=alg != 'codicioso')

    def publicar(self):
        try:
            texto = json.dumps(self.calcular())
        except Exception as e:      # parametros malos: no tumba el nodo
            self.get_logger().error(f'no se pudo planificar: {e}')
            return None
        self.pub.publish(String(data=texto))
        self.get_logger().info(f'{len(texto)} bytes publicados en /atp/contact_plan')
        return texto

    def on_service(self, _req, resp):
        texto = self.publicar()
        resp.success = texto is not None
        resp.message = texto or 'error al planificar (ver log)'
        return resp


def main():
    rclpy.init()
    node = ContactScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
