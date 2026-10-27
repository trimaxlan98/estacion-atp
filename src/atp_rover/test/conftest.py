"""Deja importar `atp_rover` desde el arbol de fuentes sin instalar."""
import os
import sys

_AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _AQUI not in sys.path:
    sys.path.insert(0, _AQUI)
