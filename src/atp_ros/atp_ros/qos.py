"""Perfiles QoS comunes de la Estacion."""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

# Topicos de MEDICION que nacen y mueren dentro de la Estacion
# (/atp/pointing_error, /atp/link): con durabilidad transient_local, un
# suscriptor que termina de descubrir al publicador tarde recibe el historial
# en vez de perder el principio del pase. 2000 muestras = 100 s a 20 Hz.
MEDICION = QoSProfile(depth=2000, history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
