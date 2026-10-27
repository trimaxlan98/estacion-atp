"""Deja importar `sat_adcs` desde el arbol de fuentes (src layout) sin instalar."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
