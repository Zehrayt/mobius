#!/usr/bin/env python3
"""Compatibility entry point for the scene regression suite."""
import unittest
from pathlib import Path

if __name__ == '__main__':
    suite = unittest.defaultTestLoader.discover(str(Path(__file__).parent/'tests'))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
