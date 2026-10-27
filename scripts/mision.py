#!/usr/bin/env python3
"""Mision integrada: un dia de operaciones de una red de estaciones (docs/MISION.md).

  planifica (optimo) -> corre CADA contacto asignado en lockstep con el pase SGP4
  real (autotrack y receptor SDR segun la banda) -> OD con lo registrado en los
  primeros pases de cada satelite -> pases siguientes con la orbita estimada ->
  datos descargados con el apuntamiento y el C/N0 MEDIDOS -> informe del dia.

Dentro del contenedor, con ROS cargado (ver scripts/validar.sh nivel `mision`):
  ./run.sh python3 scripts/mision.py --inicio 2026-10-02T12:00:00 --horas 24 --id m1 --salida runs

Todas las opciones: `python3 scripts/mision.py --help`. Con `--id`/`--salida` es el
mismo protocolo que `scripts/planificar.py` (la Estacion lo lanza como `kind: mision`).
"""
import os
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
try:
    from atp_mision import cli
except ImportError:                  # fuera de colcon: usa el arbol de fuentes
    sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_ros'))
    sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_mision'))
    from atp_mision import cli

if __name__ == '__main__':
    sys.exit(cli.main())
