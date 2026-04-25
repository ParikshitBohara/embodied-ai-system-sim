from typing import Dict, Tuple
import pybullet as p

# Height of the table surface above the world origin.
# The table URDF base sits at z=0 and its surface is 0.625 m up.
# Expose this so other modules (e.g. main.py) can mount the robot flush
# to the table top without hard-coding the magic number in multiple places.
TABLE_SURFACE_Z: float = 0.625
CUBE_START_POSITION: Tuple[float, float, float] = (0.5, 0.25, TABLE_SURFACE_Z + 0.025)
TARGET_ZONE_POSITION: Tuple[float, float, float] = (0.6, -0.25, TABLE_SURFACE_Z + 0.005)
OBJECT_START_POSITIONS: Dict[str, Tuple[float, float, float]] = {
    "red_cube": (0.42, 0.24, TABLE_SURFACE_Z + 0.025),
    "blue_sphere": (0.55, 0.24, TABLE_SURFACE_Z + 0.025),
    "yellow_rectangle": (0.68, 0.24, TABLE_SURFACE_Z + 0.025),
}
TARGET_ZONE_POSITIONS: Dict[str, Tuple[float, float, float]] = {
    "red_cube": (0.42, -0.24, TABLE_SURFACE_Z + 0.005),
    "blue_sphere": (0.55, -0.24, TABLE_SURFACE_Z + 0.005),
    "yellow_rectangle": (0.68, -0.24, TABLE_SURFACE_Z + 0.005),
}

def create_basic_scene() -> Dict[str, object]:

    plane_id = add_plane()
    table_id = add_table()
    object_bodies = {
        "red_cube": spawn_cube(position=OBJECT_START_POSITIONS["red_cube"]),
        "blue_sphere": spawn_sphere(position=OBJECT_START_POSITIONS["blue_sphere"]),
        "yellow_rectangle": spawn_rectangle(position=OBJECT_START_POSITIONS["yellow_rectangle"]),
    }
    target_zones = {
        class_name: spawn_target_zone(position=position, class_name=class_name)
        for class_name, position in TARGET_ZONE_POSITIONS.items()
    }

    return {
        "plane_id": plane_id,
        "table_id": table_id,
        "object_bodies": object_bodies,
        "target_zones": target_zones,
        "cube_id": object_bodies["red_cube"],
        "target_zone_id": target_zones["red_cube"],
    }


def add_plane() -> int:
    return p.loadURDF("plane.urdf")


def add_table(position: Tuple[float, float, float] = (0.5, 0.0, 0.0)) -> int:
    return p.loadURDF("table/table.urdf", basePosition=position, useFixedBase=True)


def spawn_cube(position: Tuple[float, float, float]) -> int:
    cube_half_extents = [0.025, 0.025, 0.025]
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=cube_half_extents)
    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=cube_half_extents,
        rgbaColor=[1.0, 0.2, 0.2, 1.0],
    )
    return p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
    )


def spawn_sphere(position: Tuple[float, float, float]) -> int:
    radius = 0.025
    collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    visual_shape = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=[0.0, 0.0, 1.0, 1.0],
    )
    return p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
    )


def spawn_rectangle(position: Tuple[float, float, float]) -> int:
    half_extents = [0.04, 0.02, 0.025]
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=[1.0, 0.85, 0.1, 1.0],
    )
    return p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
    )


def spawn_target_zone(position: Tuple[float, float, float], class_name: str = "red_cube") -> int:
    zone_half_extents = [0.08, 0.08, 0.005]
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=zone_half_extents)
    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=zone_half_extents,
        rgbaColor=[0.35, 0.35, 0.35, 0.55],
    )
    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
    )
