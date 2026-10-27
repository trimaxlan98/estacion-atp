#!/usr/bin/env python3
"""Corre una escena de enlace del rover sin ROS (envoltorio de atp_rover.comunicacion; lo lanza la app para `rover-enlace-*`)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'atp_rover'))
from atp_rover.comunicacion import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
