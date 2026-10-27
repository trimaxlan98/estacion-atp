"""Deja importar `atp_optico`, `atp_ros`, `sat_isl` y `sat_adcs` desde el arbol de fuentes sin instalar."""
import os
import sys

_AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (_AQUI, os.path.join(os.path.dirname(_AQUI), 'atp_ros'),
          os.path.join(os.path.dirname(_AQUI), 'sat_isl'),
          os.path.join(os.path.dirname(_AQUI), 'sat_adcs')):
    if p not in sys.path:
        sys.path.insert(0, p)
