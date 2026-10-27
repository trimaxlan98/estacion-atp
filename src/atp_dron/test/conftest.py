"""Deja importar `atp_dron` desde el arbol de fuentes sin instalar."""
import os
import sys

_AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _AQUI not in sys.path:
    sys.path.insert(0, _AQUI)

_ROS = os.path.join(os.path.dirname(_AQUI), 'atp_ros')          # solo para ejecutar `atp_ros.sim_clock` sobre el rclpy falso
if _ROS not in sys.path:
    sys.path.append(_ROS)
