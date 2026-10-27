#!/usr/bin/env python3
"""Corre una escena del dron sin ROS (envoltorio de atp_dron.cli; es lo que lanza la app para `dron-*`)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'atp_dron'))
from atp_dron.cli import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
