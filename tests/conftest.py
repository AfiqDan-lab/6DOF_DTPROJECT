"""
Shared pytest fixtures for the 6-DOF digital-twin test suite.

Makes `scripts/` importable (so `import arm_lib`, `import nlp_command` work)
and provides an arm chain + a standard obstacle once per test session, since
loading the URDF is the slowest step.
"""
import os
import sys

import pytest

# tests/ lives next to scripts/; put scripts/ on the import path.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)


@pytest.fixture(scope="session")
def chain():
    """The loaded 6-DOF arm (URDF -> ikpy chain). Built once per session."""
    import arm_lib as A
    return A.load_arm()


@pytest.fixture(scope="session")
def box():
    """The standard 'box on the table' obstacle used across the project."""
    import arm_lib as A
    return A.Box(center=[0.33, 0.0, 0.11], half_extents=[0.09, 0.11, 0.11],
                 name="box on table")
