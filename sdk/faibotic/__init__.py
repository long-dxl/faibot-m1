# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""SDK Python của Faibotic cho robot M1 (2 tay + đế omni + trục nâng, servo Feetech)."""
from .khay import KhayTai
from .robot import Robot

__version__ = "0.5.0"
__all__ = ["KhayTai", "Robot", "__version__"]
