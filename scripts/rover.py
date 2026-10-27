#!/usr/bin/env python3
"""Corre una escena del rover sin ROS (envoltorio de atp_rover.cli; es lo que lanza la app para `rover-*`)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'atp_rover'))
from atp_rover.cli import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
