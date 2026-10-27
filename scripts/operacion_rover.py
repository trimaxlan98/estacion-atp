#!/usr/bin/env python3
"""Corre una escena de operacion del rover sin ROS (envoltorio de atp_rover.operacion; lo lanza la app para `rover-operacion-*`)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'atp_rover'))
from atp_rover.operacion import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
