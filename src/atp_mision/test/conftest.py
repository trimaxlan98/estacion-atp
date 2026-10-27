"""Deja importar `atp_mision` y `atp_ros` desde el arbol de fuentes sin instalar."""
import os
import sys

_AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_AQUI))                                   # atp_mision
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(_AQUI)), 'atp_ros'))
sys.path.insert(0, _AQUI)                                                    # util_mision
