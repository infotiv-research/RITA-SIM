"""Target-object pick-and-place package."""

__all__ = ["PickAndPlaceTargetObject"]


def __getattr__(name):
    if name == "PickAndPlaceTargetObject":
        from .workflow import PickAndPlaceTargetObject

        return PickAndPlaceTargetObject
    raise AttributeError(name)
