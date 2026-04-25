import time

import pybullet as p
import pybullet_data

from environment.scene import OBJECT_START_POSITIONS, create_basic_scene, TABLE_SURFACE_Z
from robot.control import move_to_home, step_simulation
from robot.robot_loader import load_robot
from tasks.pick_place import run_sorting_workflow


def reset_demo(robot_id: int, object_bodies: dict[str, int]) -> None:
    for class_name, object_id in object_bodies.items():
        p.resetBasePositionAndOrientation(
            object_id,
            OBJECT_START_POSITIONS[class_name],
            [0, 0, 0, 1],
        )
        p.resetBaseVelocity(object_id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
    move_to_home(robot_id)
    step_simulation(240)


def main() -> None:
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    scene = create_basic_scene()
    robot_info = load_robot(base_position=(0.0, 0.0, TABLE_SURFACE_Z), use_fixed_base=True)
  

    print("Scene initialized.")
    print(f"Robot ID:       {robot_info['robot_id']}")
    print(f"Objects:        {scene['object_bodies']}")
    print(f"Target Zones:   {scene['target_zones']}")
    run_button_id = p.addUserDebugParameter("Run sorting workflow", 1, 0, 0)
    last_run_button_value = p.readUserDebugParameter(run_button_id)

    reset_demo(robot_info["robot_id"], scene["object_bodies"])
    print("Click the 'Run sorting workflow' button in the PyBullet panel to retest.")

    try:
        while p.isConnected(physicsClientId=physics_client):
            run_button_value = p.readUserDebugParameter(run_button_id)
            if run_button_value != last_run_button_value:
                last_run_button_value = run_button_value
                reset_demo(robot_info["robot_id"], scene["object_bodies"])
                run_sorting_workflow(
                    robot_info["robot_id"],
                    scene["object_bodies"],
                    scene["target_zones"],
                )
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        p.disconnect(physicsClientId=physics_client)


if __name__ == "__main__":
    main()
