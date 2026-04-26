import time

import pybullet as p
import pybullet_data

from environment.scene import OBJECT_START_POSITIONS, TABLE_SURFACE_Z, create_basic_scene
from perception.camera_utils import (
    capture_and_save_camera_frame,
    get_top_down_camera_config,
)
from robot.control import move_to_home, step_simulation
from robot.robot_loader import load_robot
from tasks.pick_place import run_sorting_workflow


OBJECT_CLASSES = ("red_cube", "blue_sphere", "yellow_rectangle")


def _format_class_choices(class_names: tuple[str, ...]) -> str:
    return ", ".join(
        f"{index}={class_name}"
        for index, class_name in enumerate(class_names)
    )


def create_target_zone_controls(class_names: tuple[str, ...]) -> dict[str, int]:
    """
    Create sliders in the PyBullet debug panel.

    Each slider allows the user to choose the target zone for each object class.
    """

    controls = {}

    for default_index, class_name in enumerate(class_names):
        controls[class_name] = p.addUserDebugParameter(
            f"Target for {class_name}",
            0,
            len(class_names) - 1,
            default_index,
        )

    return controls


def read_target_assignments(
    control_ids: dict[str, int],
    class_names: tuple[str, ...],
) -> dict[str, str]:
    """
    Read the selected target zone values from the PyBullet debug sliders.
    """

    assignments = {}
    max_index = len(class_names) - 1

    for object_class, control_id in control_ids.items():
        selected_index = int(round(p.readUserDebugParameter(control_id)))
        selected_index = max(0, min(max_index, selected_index))

        assignments[object_class] = class_names[selected_index]

    return assignments


def reset_demo(robot_id: int, object_bodies: dict[str, int]) -> None:
    """
    Reset all objects to their starting positions and move the robot home.
    """

    for class_name, object_id in object_bodies.items():
        p.resetBasePositionAndOrientation(
            object_id,
            OBJECT_START_POSITIONS[class_name],
            [0, 0, 0, 1],
        )

        p.resetBaseVelocity(
            object_id,
            linearVelocity=[0, 0, 0],
            angularVelocity=[0, 0, 0],
        )

    move_to_home(robot_id)
    step_simulation(240)


def save_debug_camera_frame(
    camera_config: dict[str, object],
    output_folder: str,
) -> None:
    """
    Capture and save a camera frame for debugging and report evidence.
    """

    try:
        capture_and_save_camera_frame(
            camera_config,
            output_folder=output_folder,
        )
    except Exception as error:
        print(f"Warning: failed to save camera frame to {output_folder}")
        print(f"Reason: {error}")


def main() -> None:
    physics_client = p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    scene = create_basic_scene()
    robot_info = load_robot(
        base_position=(0.0, 0.0, TABLE_SURFACE_Z),
        use_fixed_base=True,
    )

    camera_config = get_top_down_camera_config()

    print("Scene initialized.")
    print(f"Robot ID:       {robot_info['robot_id']}")
    print(f"Objects:        {scene['object_bodies']}")
    print(f"Target Zones:   {scene['target_zones']}")

    run_button_id = p.addUserDebugParameter(
        "Run sorting workflow",
        1,
        0,
        0,
    )

    target_control_ids = create_target_zone_controls(OBJECT_CLASSES)
    last_run_button_value = p.readUserDebugParameter(run_button_id)

    reset_demo(
        robot_info["robot_id"],
        scene["object_bodies"],
    )

    save_debug_camera_frame(
        camera_config,
        output_folder="outputs/initial_scene",
    )

    print("Click the 'Run sorting workflow' button in the PyBullet panel to retest.")
    print(f"Target selector values: {_format_class_choices(OBJECT_CLASSES)}")

    try:
        while p.isConnected(physicsClientId=physics_client):
            run_button_value = p.readUserDebugParameter(run_button_id)

            if run_button_value != last_run_button_value:
                last_run_button_value = run_button_value

                target_assignments = read_target_assignments(
                    target_control_ids,
                    OBJECT_CLASSES,
                )

                print(f"Selected target assignments: {target_assignments}")

                reset_demo(
                    robot_info["robot_id"],
                    scene["object_bodies"],
                )

                save_debug_camera_frame(
                    camera_config,
                    output_folder="outputs/before_workflow",
                )

                run_sorting_workflow(
                    robot_info["robot_id"],
                    scene["object_bodies"],
                    scene["target_zones"],
                    target_assignments,
                )

                save_debug_camera_frame(
                    camera_config,
                    output_folder="outputs/after_workflow",
                )

            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        print("Simulation stopped by user.")

    finally:
        p.disconnect(physicsClientId=physics_client)


if __name__ == "__main__":
    main()
