import time

import pybullet as p
import pybullet_data

from environment.scene import create_basic_scene, TABLE_SURFACE_Z
from robot.robot_loader import load_robot
from tasks.pick_place import run_pick_and_place_workflow


def main() -> None:
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    scene = create_basic_scene()
    # Robot base is placed at the back edge of the table (x=0.0, y=0.0) and
    # elevated to TABLE_SURFACE_Z so it sits on top rather than protruding through.
    robot_info = load_robot(base_position=(0.0, 0.0, TABLE_SURFACE_Z), use_fixed_base=True)
  

    print("Scene initialized.")
    print(f"Robot ID:       {robot_info['robot_id']}")
    print(f"Cube ID:        {scene['cube_id']}")
    print(f"Target Zone ID: {scene['target_zone_id']}")
    run_pick_and_place_workflow(
        robot_info["robot_id"],
        scene["cube_id"],
        scene["target_zone_id"],
    )

    try:
        while p.isConnected(physicsClientId=physics_client):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        p.disconnect(physicsClientId=physics_client)


if __name__ == "__main__":
    main()
