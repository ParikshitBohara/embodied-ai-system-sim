from typing import Dict, Tuple
import pybullet as p

# Height of the table surface above the world origin.
# The table URDF base sits at z=0 and its surface is 0.625 m up.
# Expose this so other modules (e.g. main.py) can mount the robot flush
# to the table top without hard-coding the magic number in multiple places.
TABLE_SURFACE_Z: float = 0.625


def create_basic_scene() -> Dict[str, int]:

    plane_id = add_plane()
    table_id = add_table()
    cube_z = TABLE_SURFACE_Z + 0.025          # surface + half-extent
    zone_z = TABLE_SURFACE_Z + 0.005          # surface + half-extent of flat zone
    cube_id = spawn_cube(position=(0.5, 0.0, cube_z))
    target_zone_id = spawn_target_zone(position=(0.6, -0.25, zone_z))

    return {
        "plane_id": plane_id,
        "table_id": table_id,
        "cube_id": cube_id,
        "target_zone_id": target_zone_id,
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


def spawn_target_zone(position: Tuple[float, float, float]) -> int:
    zone_half_extents = [0.08, 0.08, 0.005]
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=zone_half_extents)
    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=zone_half_extents,
        rgbaColor=[0.2, 0.8, 0.2, 0.7],
    )
    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
    )
