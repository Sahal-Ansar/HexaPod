"""Pytest bootstrap.

Putting this at the repo root makes the root the pytest ``rootdir`` and, more
importantly, guarantees the ``pi`` package is importable from the tests
regardless of how pytest is launched (`pytest`, `python -m pytest`, from an IDE,
etc.). Without it, a bare ``pytest`` invocation would not have the repo root on
``sys.path`` and ``import pi.kinematics...`` would fail.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
