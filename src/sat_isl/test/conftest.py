"""Deja importar `sat_isl` (y, sin instalar, `atp_ros` y `sat_adcs`) desde el arbol de fuentes."""
import os
import sys

_AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (_AQUI, os.path.join(os.path.dirname(_AQUI), 'atp_ros'),
          os.path.join(os.path.dirname(_AQUI), 'sat_adcs')):
    if p not in sys.path:
        sys.path.insert(0, p)
