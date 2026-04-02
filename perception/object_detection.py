from typing import Dict, Tuple

import pybullet as p


def get_object_position(object_id: int) -> Dict[str, Tuple[float, float, float]]:
    """Return world position and orientation for a simulated object."""
    position, orientation = p.getBasePositionAndOrientation(object_id)
    return {
        "position": position,
        "orientation": orientation,
    }
