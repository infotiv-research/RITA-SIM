"""Target-object pick-and-place package export surface.

Responsibilities:
- Re-export the main node class for simple imports.
- Keep entry scripts decoupled from module internals.
"""

from .workflow import PickAndPlaceTargetObject

__all__ = ["PickAndPlaceTargetObject"]
